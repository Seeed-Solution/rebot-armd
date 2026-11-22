//! Canonical tool/gripper pack configuration, overlay application, unified policy validation,
//! and cross-language deterministic `tool_config_digest` (`v1:<sha256>`).

use serde::{Deserialize, Serialize};

use crate::model::{
    validate_effective_gripper_policy, valid_tool_id, ArmConfig, CalibrationToolSpec,
    GripperPolicy, JawOpening,
};

/// Canonical digest format version.
pub const TOOL_DIGEST_VERSION: u32 = 1;
/// Stock tool identifier.
pub const STOCK_TOOL_ID: &str = "default";
/// Maximum allowed Euclidean norm of `tcp_offset` (m).
pub const MAX_TCP_OFFSET_NORM_M: f64 = 0.30;
/// Maximum allowed Euclidean norm of any check point in `end_link` frame (m).
pub const MAX_CHECK_POINT_NORM_M: f64 = 0.50;
/// Maximum number of points in the deduplicated 4-layer check-point union.
pub const MAX_EFFECTIVE_CHECK_POINTS: usize = 32;
/// Maximum allowed payload mass delta (kg) without a tool-matched dynamics model.
pub const DYNAMICS_MAX_MASS_DELTA_KG: f64 = 0.15;
/// Maximum allowed payload mass delta in micro-kg (mg), quantized identically to the digest.
pub const DYNAMICS_MAX_MASS_DELTA_MG: i64 = 150_000;
/// Maximum allowed worst-case horizontal CoM pitch torque delta (N·m) without a tool-matched dynamics model.
pub const DYNAMICS_MAX_COM_MOMENT_DELTA_NM: f64 = 0.05;
/// Maximum allowed worst-case horizontal CoM pitch torque delta in micro-N·m (uN·m), quantized identically to the digest.
pub const DYNAMICS_MAX_COM_MOMENT_DELTA_UNM: i64 = 50_000;

fn default_kind_gripper() -> String {
    "gripper".to_string()
}

fn default_schema_version() -> u32 {
    1
}

fn default_true() -> bool {
    true
}

/// Active tool identity published on `GET /api/v1/observation/schema` and `/api/v1/safety/describe`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ActiveToolInfo {
    #[serde(default = "default_schema_version")]
    pub schema_version: u32,
    #[serde(default = "default_stock_id")]
    pub id: String,
    #[serde(default = "default_true")]
    pub is_stock: bool,
    #[serde(default = "default_kind_gripper")]
    pub kind: String,
    #[serde(default)]
    pub tcp_offset_m: [f64; 3],
    #[serde(default)]
    pub config_digest: String,
    #[serde(default)]
    pub pack_digest: Option<String>,
    #[serde(default)]
    pub commissioning_required: bool,
}

fn default_stock_id() -> String {
    STOCK_TOOL_ID.to_string()
}

impl Default for ActiveToolInfo {
    fn default() -> Self {
        Self {
            schema_version: 1,
            id: STOCK_TOOL_ID.to_string(),
            is_stock: true,
            kind: "gripper".to_string(),
            tcp_offset_m: [0.0; 3],
            config_digest: String::new(),
            pack_digest: None,
            commissioning_required: false,
        }
    }
}

impl ActiveToolInfo {
    /// True when a persisted tool-bound record `(tool_id, tool_config_digest)` applies to this active tool.
    /// Legacy unkeyed records `(None, None)` apply only to stock configurations.
    pub fn matches_record(&self, tool_id: Option<&str>, tool_config_digest: Option<&str>) -> bool {
        match (tool_id, tool_config_digest) {
            (Some(id), Some(dig)) => id == self.id && dig == self.config_digest,
            (Some(id), None) => self.is_stock && id == STOCK_TOOL_ID && self.id == STOCK_TOOL_ID,
            (None, Some(dig)) => dig == self.config_digest,
            (None, None) => self.is_stock,
        }
    }

    /// True when a `calibration.json` `ToolCalibration` (`tool_id` + optional `tool_config_digest`)
    /// applies to this active tool. Legacy records without `tool_config_digest` apply only to stock.
    pub fn matches_tool_calib(&self, tool_id: &str, tool_config_digest: Option<&str>) -> bool {
        match tool_config_digest {
            Some(dig) => dig == self.config_digest && (self.is_stock || tool_id == self.id),
            None => self.is_stock,
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub const DEFAULT_COMMISSIONING_PATH: &str = "/userdata/rebot-teleop/tool_commissioning.json";

#[cfg(not(target_arch = "wasm32"))]
pub fn commissioning_path_from_env() -> std::path::PathBuf {
    crate::persist::path_from_env("ARMD_TOOL_COMMISSIONING", DEFAULT_COMMISSIONING_PATH)
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolCommissioningRecord {
    pub robot_id: String,
    pub tool_id: String,
    pub tool_config_digest: String,
    pub confirmed_at: String,
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolCommissioningFile {
    pub records: Vec<ToolCommissioningRecord>,
}

#[cfg(not(target_arch = "wasm32"))]
impl ToolCommissioningFile {
    pub fn validate(&self) -> Result<(), String> {
        for r in &self.records {
            if r.robot_id.is_empty()
                || r.tool_id.is_empty()
                || r.tool_config_digest.is_empty()
                || r.confirmed_at.is_empty()
            {
                return Err("tool commissioning record fields must be non-empty".into());
            }
        }
        Ok(())
    }

    pub fn find(&self, robot_id: &str, tool_id: &str, digest: &str) -> Option<&ToolCommissioningRecord> {
        self.records
            .iter()
            .find(|r| r.robot_id == robot_id && r.tool_id == tool_id && r.tool_config_digest == digest)
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub fn active_robot_id(config: &ArmConfig) -> String {
    config
        .profile_id
        .as_deref()
        .and_then(normalize_robot_id)
        .unwrap_or("unknown")
        .to_string()
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub struct PreparedToolCommissioning {
    pub base_revision: u64,
    pub tool_id: String,
    pub tool_config_digest: String,
    pub confirmed_at: String,
    pub write: Option<crate::persist::PendingWrite>,
}

#[cfg(not(target_arch = "wasm32"))]
fn read_commissioning_file_strict(path: &std::path::Path) -> Result<Option<ToolCommissioningFile>, String> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("read {}: {e}", path.display())),
    };
    let file: ToolCommissioningFile = serde_json::from_str(&text)
        .map_err(|e| format!("parse {}: {e}", path.display()))?;
    file.validate()
        .map_err(|e| format!("validate {}: {e}", path.display()))?;
    Ok(Some(file))
}

#[cfg(not(target_arch = "wasm32"))]
pub fn prepare_commissioning_confirm(
    inner: &crate::supervisor::SupervisorInner,
    body: &serde_json::Value,
    require_expected_revision: bool,
) -> Result<PreparedToolCommissioning, crate::supervisor::ArmError> {
    use crate::motion::MotionError;
    use crate::supervisor::ArmError;

    let obj = body
        .as_object()
        .ok_or_else(|| MotionError::InvalidCommand("tool commissioning confirm body must be a JSON object".into()))?;
    for k in obj.keys() {
        if !["expected_revision", "tool_id", "tool_config_digest", "confirm"].contains(&k.as_str()) {
            return Err(MotionError::InvalidCommand(format!("unknown field: {k}")).into());
        }
    }
    if obj.get("confirm").and_then(serde_json::Value::as_bool) != Some(true) {
        return Err(MotionError::InvalidCommand("confirm must be boolean true".into()).into());
    }
    if let Some(exp) = crate::persist::parse_expected_revision(body, require_expected_revision)
        .map_err(MotionError::InvalidCommand)?
    {
        if exp != inner.tool_commissioning_revision {
            return Err(ArmError::PolicyRevisionMismatch {
                expected: exp,
                current: inner.tool_commissioning_revision,
            });
        }
    }
    let active = &inner.config.active_tool;
    let tid = obj
        .get("tool_id")
        .and_then(serde_json::Value::as_str)
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .ok_or_else(|| MotionError::InvalidCommand("tool_id must be a non-empty string".into()))?;
    if tid != active.id {
        return Err(MotionError::InvalidCommand(format!(
            "tool_id '{}' does not match active tool '{}'",
            tid, active.id
        ))
        .into());
    }
    let dig = obj
        .get("tool_config_digest")
        .and_then(serde_json::Value::as_str)
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .ok_or_else(|| MotionError::InvalidCommand("tool_config_digest must be a non-empty string".into()))?;
    if dig != active.config_digest {
        return Err(MotionError::InvalidCommand(format!(
            "tool_config_digest '{}' does not match active tool digest '{}'",
            dig, active.config_digest
        ))
        .into());
    }
    let robot_id = active_robot_id(&inner.config);
    let confirmed_at = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true);
    let write = match inner.tool_commissioning_path.as_ref() {
        Some(path) => {
            let mut file = match read_commissioning_file_strict(path) {
                Ok(Some(f)) => f,
                Ok(None) => ToolCommissioningFile::default(),
                Err(e) => {
                    return Err(ArmError::Rejected {
                        status: 500,
                        code: "COMMISSIONING_STORE_DAMAGED".into(),
                        message: format!(
                            "tool commissioning store is damaged or unreadable ({e}); refusing to overwrite"
                        ),
                    });
                }
            };
            file.records.retain(|r| {
                !(r.robot_id == robot_id
                    && r.tool_id == active.id
                    && r.tool_config_digest == active.config_digest)
            });
            file.records.push(ToolCommissioningRecord {
                robot_id,
                tool_id: active.id.clone(),
                tool_config_digest: active.config_digest.clone(),
                confirmed_at: confirmed_at.clone(),
            });
            Some(crate::persist::PendingWrite::new(
                path.clone(),
                serde_json::to_string_pretty(&file).unwrap_or_default(),
            ))
        }
        None => None,
    };
    Ok(PreparedToolCommissioning {
        base_revision: inner.tool_commissioning_revision,
        tool_id: active.id.clone(),
        tool_config_digest: active.config_digest.clone(),
        confirmed_at,
        write,
    })
}

#[cfg(not(target_arch = "wasm32"))]
pub fn precheck_commissioning_confirm(
    inner: &crate::supervisor::SupervisorInner,
    prepared: &PreparedToolCommissioning,
) -> Result<(), crate::supervisor::ArmError> {
    use crate::motion::MotionError;
    use crate::supervisor::ArmError;
    if inner.tool_commissioning_revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch {
            expected: prepared.base_revision,
            current: inner.tool_commissioning_revision,
        });
    }
    if inner.calibration_busy {
        return Err(ArmError::Rejected {
            status: 409,
            code: "CONFIG_BUSY".into(),
            message: "a camera calibration apply or restore is running".into(),
        });
    }
    if inner.config.active_tool.id != prepared.tool_id
        || inner.config.active_tool.config_digest != prepared.tool_config_digest
    {
        return Err(MotionError::PlanStale("active tool changed before commissioning confirm".into()).into());
    }
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
pub fn commit_commissioning_confirm(
    inner: &mut crate::supervisor::SupervisorInner,
    prepared: PreparedToolCommissioning,
) -> Result<(ActiveToolInfo, u64), crate::supervisor::ArmError> {
    use crate::motion::MotionError;
    use crate::supervisor::ArmError;
    if inner.tool_commissioning_revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch {
            expected: prepared.base_revision,
            current: inner.tool_commissioning_revision,
        });
    }
    if inner.config.active_tool.id != prepared.tool_id
        || inner.config.active_tool.config_digest != prepared.tool_config_digest
    {
        return Err(MotionError::PlanStale("active tool changed before commissioning confirm".into()).into());
    }
    inner.config.active_tool.commissioning_required = false;
    inner.tool_commissioning_confirmed_at = Some(prepared.confirmed_at);
    inner.tool_commissioning_revision += 1;
    Ok((inner.config.active_tool.clone(), inner.tool_commissioning_revision))
}

#[cfg(not(target_arch = "wasm32"))]
pub async fn load_commissioning_into(
    supervisor: &crate::supervisor::ArmSupervisor,
    path: &std::path::Path,
) {
    let mut inner = supervisor.inner().write().await;
    inner.tool_commissioning_path = Some(path.to_path_buf());
    if inner.config.active_tool.is_stock {
        return;
    }
    inner.config.active_tool.commissioning_required = true;
    inner.tool_commissioning_confirmed_at = None;
    let robot_id = active_robot_id(&inner.config);
    let tool_id = inner.config.active_tool.id.clone();
    let digest = inner.config.active_tool.config_digest.clone();
    match read_commissioning_file_strict(path) {
        Ok(Some(f)) => {
            if let Some(rec) = f.find(&robot_id, &tool_id, &digest) {
                inner.config.active_tool.commissioning_required = false;
                inner.tool_commissioning_confirmed_at = Some(rec.confirmed_at.clone());
                tracing::info!(
                    "Loaded tool commissioning confirmation for '{tool_id}' ({digest}) from {}",
                    path.display()
                );
            } else {
                tracing::info!(
                    "Tool '{tool_id}' ({digest}) not commissioned yet in {}; keeping commissioning_required=true",
                    path.display()
                );
            }
        }
        Ok(None) => {}
        Err(e) => {
            tracing::error!(
                "Damaged tool commissioning file {} ({e}); keeping commissioning_required=true without modifying file",
                path.display()
            );
        }
    }
}

/// Optional dynamics delta metadata in `tool.json` (`armd.dynamics`).
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ToolDynamics {
    #[serde(default)]
    pub mass_delta_kg: Option<f64>,
    #[serde(default)]
    pub com_moment_delta_nm: Option<f64>,
}

impl ToolDynamics {
    /// Whether the dynamics delta is explicitly declared, finite, within micro-unit range,
    /// and within stock-model limits (evaluated on the same micro-unit quantized values as the digest).
    pub fn is_qualified(dyn_opt: Option<&Self>) -> bool {
        let Some(d) = dyn_opt else {
            return false;
        };
        let Some(m) = d.mass_delta_kg else {
            return false;
        };
        let Some(c) = d.com_moment_delta_nm else {
            return false;
        };
        let Ok(m_mg) = checked_to_micro_f64(m, "mass_delta_kg") else {
            return false;
        };
        let Ok(c_unm) = checked_to_micro_f64(c, "com_moment_delta_nm") else {
            return false;
        };
        m_mg.abs() <= DYNAMICS_MAX_MASS_DELTA_MG
            && c_unm.abs() <= DYNAMICS_MAX_COM_MOMENT_DELTA_UNM
    }
}

/// Maximum safe integer representable exactly in IEEE-754 f64 (cross-language JS/Python/Rust bound).
pub const MAX_SAFE_MICRO_INT: i64 = (1_i64 << 53) - 1;

/// Allowed `ToolPackConfig.kind` values.
pub fn is_valid_tool_kind(kind: &str) -> bool {
    matches!(kind, "gripper" | "suction" | "screwdriver")
}

/// Validate `attach_body`: empty (defaults to `end_link`) or `"end_link"`.
pub fn validate_attach_body(attach_body: &str) -> Result<(), String> {
    if attach_body.trim().is_empty() || attach_body == "end_link" {
        Ok(())
    } else {
        Err(format!(
            "unsupported attach_body '{attach_body}': only 'end_link' is supported"
        ))
    }
}

/// Optional gripper policy overrides in `tool.json` (`armd.gripper_policy`).
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ArmdGripperOverlay {
    #[serde(default)]
    pub closed_rad: Option<f64>,
    #[serde(default)]
    pub open_rad: Option<f64>,
    #[serde(default)]
    pub jaw_opening_nominal: Option<Vec<[f64; 2]>>,
    #[serde(default)]
    pub holding_gap_rad: Option<f64>,
    #[serde(default)]
    pub holding_torque_nm: Option<f64>,
    #[serde(default)]
    pub contact_torque_nm: Option<f64>,
    #[serde(default)]
    pub squeeze_rad: Option<f64>,
    #[serde(default)]
    pub max_close_torque_nm: Option<f64>,
}

/// Optional grasp setup overrides in `tool.json` (`armd.grasp`).
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ArmdGraspOverlay {
    #[serde(default)]
    pub max_grasp_width_m: Option<f64>,
    #[serde(default)]
    pub grasp_height_m: Option<f64>,
    #[serde(default)]
    pub edge_to_low_m: Option<f64>,
}

/// Daemon overlay block in `tool.json` (`armd`). Unknown fields are rejected.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ArmdToolOverlay {
    #[serde(default)]
    pub extra_check_points_end_m: Vec<[f64; 3]>,
    #[serde(default)]
    pub physical_contact_end_m: Option<[f64; 3]>,
    #[serde(default)]
    pub contact_opening_m: Option<f64>,
    #[serde(default)]
    pub dynamics: Option<ToolDynamics>,
    #[serde(default)]
    pub gripper_policy: Option<ArmdGripperOverlay>,
    #[serde(default)]
    pub grasp: Option<ArmdGraspOverlay>,
}

/// Canonical tool pack manifest (`tool.json`).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ToolPackConfig {
    pub id: String,
    #[serde(default)]
    pub name: String,
    #[serde(default = "default_kind_gripper")]
    pub kind: String,
    #[serde(default)]
    pub version: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub tool_xml: String,
    #[serde(default)]
    pub robots: Vec<String>,
    #[serde(default)]
    pub attach_body: String,
    #[serde(default)]
    pub replace_bodies: Vec<String>,
    #[serde(default)]
    pub replace_geoms: Vec<String>,
    #[serde(default)]
    pub replace_equalities: Vec<String>,
    #[serde(default)]
    pub replace_actuators: Vec<String>,
    #[serde(default)]
    pub driven_joint: Option<String>,
    #[serde(default)]
    pub coupled_joint: Option<String>,
    #[serde(default)]
    pub actuator: Option<String>,
    #[serde(default)]
    pub motor_to_joint_scale: Option<f64>,
    #[serde(default)]
    pub finger_geoms: Vec<String>,
    #[serde(default)]
    pub tcp_site: Option<String>,
    #[serde(default)]
    pub tcp_offset: Option<[f64; 3]>,
    #[serde(default)]
    pub suction_range_m: Option<f64>,
    #[serde(default)]
    pub io: Vec<serde_json::Value>,
    #[serde(default)]
    pub assets: Vec<serde_json::Value>,
    #[serde(default)]
    pub armd: Option<ArmdToolOverlay>,
}

/// Normalize known robot profile/wire names to canonical identifiers.
pub fn normalize_robot_id(raw: &str) -> Option<&'static str> {
    let key = raw.trim().to_ascii_lowercase().replace('-', "_");
    match key.as_str() {
        "rebot_b601dm" | "b601dm" | "rebot_dm" => Some("b601dm"),
        "rebot_b601_rs" | "rebot_b601rs" | "b601rs" | "b601_rs" | "rebot_rs" => Some("b601rs"),
        "so_arm101" | "so101" => Some("so101"),
        "so_arm100" | "so100" => Some("so100"),
        _ => None,
    }
}

/// Maximum input magnitude accepted by micro-unit scaling (`|x| <= 1e9`), so
/// `(x * 1_000_000.0).abs() <= 1e15` fits cleanly within IEEE-754 `f64` exact integer range.
pub const MAX_MICRO_INPUT_ABS: f64 = 1_000_000_000.0;

/// Normalize [`crate::model::HoldingMode`] to its exact serde wire string.
pub fn holding_mode_wire_name(mode: crate::model::HoldingMode) -> &'static str {
    match mode {
        crate::model::HoldingMode::TorqueAndGap => "torque_and_gap",
        crate::model::HoldingMode::PositionGap => "position_gap",
        crate::model::HoldingMode::LoadAndGap => "load_and_gap",
    }
}

/// Build the canonical baseline [`ArmConfig`] for a normalized robot ID (`"b601dm"`, `"so101"`,
/// `"so100"`, `"b601rs"`), working identically on native and wasm32.
pub fn baseline_arm_config_for_robot(robot_id: &str) -> Result<ArmConfig, String> {
    let norm = normalize_robot_id(robot_id)
        .ok_or_else(|| format!("unknown robot profile '{robot_id}'"))?;
    let mut cfg = match norm {
        "b601dm" => ArmConfig::rebot_dm(),
        "so101" => {
            let mut c: ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm101.json"))
                .map_err(|e| format!("embedded so_arm101 profile: {e}"))?;
            c.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf")))?;
            c
        }
        "so100" => {
            let mut c: ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm100.json"))
                .map_err(|e| format!("embedded so_arm100 profile: {e}"))?;
            c.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf")))?;
            c
        }
        "b601rs" => {
            let mut c: ArmConfig = serde_json::from_str(include_str!("../profiles/rebot_b601_rs.json"))
                .map_err(|e| format!("embedded rebot_b601_rs profile: {e}"))?;
            c.init_arm_model_in_memory(Some(include_str!("../assets/ReBot_Arm_RS.urdf")))?;
            c
        }
        _ => unreachable!(),
    };
    cfg.validate()?;
    Ok(cfg)
}

/// Round `x` (in metres / radians / N·m / kg) to micro-units using cross-language exact
/// fractional-part half-away-from-zero (`|x| <= 1e9`).
pub fn checked_to_micro_f64(x: f64, label: &str) -> Result<i64, String> {
    if !x.is_finite() || x.abs() > MAX_MICRO_INPUT_ABS {
        return Err(format!("{label} must be finite and within [-1e9, 1e9]"));
    }
    Ok(to_micro_f64(x))
}

fn to_micro_f64(x: f64) -> i64 {
    let abs_s = (x * 1_000_000.0).abs();
    let tr = abs_s.trunc();
    let mag = (tr as i64) + i64::from(abs_s - tr >= 0.5);
    if x < 0.0 && mag != 0 { -mag } else { mag }
}

fn vec3_to_micro_f64(v: &[f64; 3]) -> [i64; 3] {
    [to_micro_f64(v[0]), to_micro_f64(v[1]), to_micro_f64(v[2])]
}

// Field order on the canonical structs is strictly lexicographical to guarantee sorted keys.
#[derive(Serialize)]
struct CanonicalDynamics {
    com_moment_delta_unm: Option<i64>,
    mass_delta_mg: Option<i64>,
}

#[derive(Serialize)]
struct CanonicalGripperPolicy {
    closed_urad: i64,
    contact_torque_unm: Option<i64>,
    holding_gap_urad: i64,
    holding_load_fraction_um: Option<i64>,
    holding_mode: String,
    holding_torque_unm: i64,
    jaw_opening_nominal: Option<Vec<[i64; 2]>>,
    max_close_torque_unm: Option<i64>,
    max_lead_urad: Option<i64>,
    open_urad: i64,
    squeeze_urad: i64,
    stream_contact_speed_urad_s: Option<i64>,
}

#[derive(Serialize)]
struct CanonicalToolPayload<'a> {
    contact_opening_um: Option<i64>,
    dynamics: Option<CanonicalDynamics>,
    edge_to_low_um: Option<i64>,
    extra_points_um: Vec<[i64; 3]>,
    grasp_height_um: Option<i64>,
    gripper_policy: Option<CanonicalGripperPolicy>,
    kind: &'a str,
    max_grasp_width_um: Option<i64>,
    physical_contact_um: Option<[i64; 3]>,
    robot_id: &'a str,
    tcp_offset_um: [i64; 3],
    tool_id: &'a str,
    v: u32,
}

/// Unnarrowed `f64` gripper policy inputs for canonical digest serialization.
#[derive(Debug, Clone)]
pub struct CanonicalGripperInput {
    pub closed_rad: f64,
    pub open_rad: f64,
    pub jaw_opening_nominal: Option<Vec<[f64; 2]>>,
    pub holding_gap_rad: f64,
    pub holding_load_fraction: Option<f64>,
    pub holding_mode: crate::model::HoldingMode,
    pub holding_torque_nm: f64,
    pub contact_torque_nm: Option<f64>,
    pub max_lead_rad: Option<f64>,
    pub squeeze_rad: f64,
    pub max_close_torque_nm: Option<f64>,
    pub stream_contact_speed_rad_s: Option<f64>,
}

impl From<&GripperPolicy> for CanonicalGripperInput {
    fn from(gp: &GripperPolicy) -> Self {
        Self {
            closed_rad: f64::from(gp.closed_rad),
            open_rad: f64::from(gp.open_rad),
            jaw_opening_nominal: gp.jaw_opening.as_ref().map(|jo| {
                jo.points
                    .iter()
                    .map(|p| [f64::from(p[0]), f64::from(p[1])])
                    .collect()
            }),
            holding_gap_rad: f64::from(gp.holding_gap_rad),
            holding_load_fraction: gp.holding_load_fraction.map(f64::from),
            holding_mode: gp.holding_mode,
            holding_torque_nm: f64::from(gp.holding_torque_nm),
            contact_torque_nm: gp.contact_torque_nm.map(f64::from),
            max_lead_rad: gp.max_lead_rad.map(f64::from),
            squeeze_rad: f64::from(gp.squeeze_rad),
            max_close_torque_nm: gp.max_close_torque_nm.map(f64::from),
            stream_contact_speed_rad_s: gp.stream_contact_speed_rad_s.map(f64::from),
        }
    }
}

/// Produce the exact canonical UTF-8 compact JSON payload used for `tool_config_digest`
/// directly from `f64` values (no intermediate `f32` truncation).
#[allow(clippy::too_many_arguments)]
pub fn canonical_tool_payload_json_f64(
    robot_id: &str,
    tool_id: &str,
    kind: &str,
    tcp_offset_m: [f64; 3],
    permanent_tool_check_points_end_m: &[[f64; 3]],
    physical_contact_end_m: Option<[f64; 3]>,
    contact_opening_m: Option<f64>,
    dynamics: Option<&ToolDynamics>,
    gripper_policy: Option<&CanonicalGripperInput>,
    max_grasp_width_m: Option<f64>,
    grasp_height_m: Option<f64>,
    edge_to_low_m: Option<f64>,
) -> String {
    let mut extra_points_um: Vec<[i64; 3]> = permanent_tool_check_points_end_m
        .iter()
        .map(vec3_to_micro_f64)
        .collect();
    extra_points_um.sort_unstable();
    extra_points_um.dedup();

    let canonical_dynamics = dynamics.map(|d| CanonicalDynamics {
        com_moment_delta_unm: d.com_moment_delta_nm.map(to_micro_f64),
        mass_delta_mg: d.mass_delta_kg.map(to_micro_f64),
    });

    let canonical_gp = gripper_policy.map(|gp| {
        let jaw_opening_nominal = gp.jaw_opening_nominal.as_ref().map(|pts_in| {
            let mut pts: Vec<[i64; 2]> = pts_in
                .iter()
                .map(|p| [to_micro_f64(p[0]), to_micro_f64(p[1])])
                .collect();
            pts.sort_unstable_by_key(|p| p[0]);
            pts
        });
        CanonicalGripperPolicy {
            closed_urad: to_micro_f64(gp.closed_rad),
            contact_torque_unm: gp.contact_torque_nm.map(to_micro_f64),
            holding_gap_urad: to_micro_f64(gp.holding_gap_rad),
            holding_load_fraction_um: gp.holding_load_fraction.map(to_micro_f64),
            holding_mode: holding_mode_wire_name(gp.holding_mode).to_string(),
            holding_torque_unm: to_micro_f64(gp.holding_torque_nm),
            jaw_opening_nominal,
            max_close_torque_unm: gp.max_close_torque_nm.map(to_micro_f64),
            max_lead_urad: gp.max_lead_rad.map(to_micro_f64),
            open_urad: to_micro_f64(gp.open_rad),
            squeeze_urad: to_micro_f64(gp.squeeze_rad),
            stream_contact_speed_urad_s: gp.stream_contact_speed_rad_s.map(to_micro_f64),
        }
    });

    let payload = CanonicalToolPayload {
        contact_opening_um: contact_opening_m.map(to_micro_f64),
        dynamics: canonical_dynamics,
        edge_to_low_um: edge_to_low_m.map(to_micro_f64),
        extra_points_um,
        grasp_height_um: grasp_height_m.map(to_micro_f64),
        gripper_policy: canonical_gp,
        kind,
        max_grasp_width_um: max_grasp_width_m.map(to_micro_f64),
        physical_contact_um: physical_contact_end_m.as_ref().map(vec3_to_micro_f64),
        robot_id,
        tcp_offset_um: vec3_to_micro_f64(&tcp_offset_m),
        tool_id,
        v: TOOL_DIGEST_VERSION,
    };
    serde_json::to_string(&payload).expect("canonical tool payload serialization cannot fail")
}

/// Produce the exact canonical UTF-8 compact JSON payload used for `tool_config_digest`.
#[allow(clippy::too_many_arguments)]
pub fn canonical_tool_payload_json(
    robot_id: &str,
    tool_id: &str,
    kind: &str,
    tcp_offset_m: [f64; 3],
    permanent_tool_check_points_end_m: &[[f32; 3]],
    physical_contact_end_m: Option<[f64; 3]>,
    contact_opening_m: Option<f32>,
    dynamics: Option<&ToolDynamics>,
    gripper_policy: Option<&GripperPolicy>,
    max_grasp_width_m: Option<f64>,
    grasp_height_m: Option<f64>,
    edge_to_low_m: Option<f64>,
) -> String {
    let pts_f64: Vec<[f64; 3]> = permanent_tool_check_points_end_m
        .iter()
        .map(|p| [f64::from(p[0]), f64::from(p[1]), f64::from(p[2])])
        .collect();
    let gp_in = gripper_policy.map(CanonicalGripperInput::from);
    canonical_tool_payload_json_f64(
        robot_id,
        tool_id,
        kind,
        tcp_offset_m,
        &pts_f64,
        physical_contact_end_m,
        contact_opening_m.map(f64::from),
        dynamics,
        gp_in.as_ref(),
        max_grasp_width_m,
        grasp_height_m,
        edge_to_low_m,
    )
}

/// Compute `v1:<sha256>` digest for a resolved `f64` tool configuration.
#[allow(clippy::too_many_arguments)]
pub fn compute_tool_config_digest_f64(
    robot_id: &str,
    tool_id: &str,
    kind: &str,
    tcp_offset_m: [f64; 3],
    permanent_tool_check_points_end_m: &[[f64; 3]],
    physical_contact_end_m: Option<[f64; 3]>,
    contact_opening_m: Option<f64>,
    dynamics: Option<&ToolDynamics>,
    gripper_policy: Option<&CanonicalGripperInput>,
    max_grasp_width_m: Option<f64>,
    grasp_height_m: Option<f64>,
    edge_to_low_m: Option<f64>,
) -> String {
    let json = canonical_tool_payload_json_f64(
        robot_id,
        tool_id,
        kind,
        tcp_offset_m,
        permanent_tool_check_points_end_m,
        physical_contact_end_m,
        contact_opening_m,
        dynamics,
        gripper_policy,
        max_grasp_width_m,
        grasp_height_m,
        edge_to_low_m,
    );
    format!("v1:{}", crate::gravity::compute_sha256(json.as_bytes()))
}

/// Compute `v1:<sha256>` digest for a resolved tool configuration.
#[allow(clippy::too_many_arguments)]
pub fn compute_tool_config_digest(
    robot_id: &str,
    tool_id: &str,
    kind: &str,
    tcp_offset_m: [f64; 3],
    permanent_tool_check_points_end_m: &[[f32; 3]],
    physical_contact_end_m: Option<[f64; 3]>,
    contact_opening_m: Option<f32>,
    dynamics: Option<&ToolDynamics>,
    gripper_policy: Option<&GripperPolicy>,
    max_grasp_width_m: Option<f64>,
    grasp_height_m: Option<f64>,
    edge_to_low_m: Option<f64>,
) -> String {
    let json = canonical_tool_payload_json(
        robot_id,
        tool_id,
        kind,
        tcp_offset_m,
        permanent_tool_check_points_end_m,
        physical_contact_end_m,
        contact_opening_m,
        dynamics,
        gripper_policy,
        max_grasp_width_m,
        grasp_height_m,
        edge_to_low_m,
    );
    format!("v1:{}", crate::gravity::compute_sha256(json.as_bytes()))
}

/// Immutable baseline `(max_grasp_width_m, grasp_height_m)` shared across native and wasm32.
pub fn baseline_grasp_limits(config: &ArmConfig) -> (Option<f64>, Option<f64>) {
    #[cfg(not(target_arch = "wasm32"))]
    {
        config
            .grasp
            .as_ref()
            .map_or((None, None), |g| (Some(g.max_grasp_width_m), Some(g.grasp_height_m)))
    }
    #[cfg(target_arch = "wasm32")]
    {
        if config.gripper_policy.is_none() {
            return (None, None);
        }
        let robot_id = config
            .profile_id
            .as_deref()
            .and_then(normalize_robot_id)
            .unwrap_or("unknown");
        match robot_id {
            "so101" | "so100" => (Some(0.065), Some(0.015)),
            _ => (Some(0.075), Some(0.015)),
        }
    }
}

/// Build [`ActiveToolInfo`] for the stock configuration of `config`.
pub fn stock_active_tool_info(config: &ArmConfig) -> ActiveToolInfo {
    let robot_id = config
        .profile_id
        .as_deref()
        .and_then(normalize_robot_id)
        .unwrap_or_else(|| config.profile_id.as_deref().unwrap_or("unknown"));
    let (max_grasp_width_m, grasp_height_m) = baseline_grasp_limits(config);
    let edge_to_low_m = config
        .calibration_tool
        .as_ref()
        .and_then(|c| c.edge_to_low_m);

    let physical_contact = config.physical_contact_end_m.or(Some([0.0; 3]));
    let contact_opening = config.contact_opening_m.or(Some(0.0));
    let digest = compute_tool_config_digest(
        robot_id,
        STOCK_TOOL_ID,
        "gripper",
        [0.0; 3],
        &[],
        physical_contact,
        contact_opening,
        None,
        config.gripper_policy.as_ref(),
        max_grasp_width_m,
        grasp_height_m,
        edge_to_low_m,
    );
    ActiveToolInfo {
        schema_version: 1,
        id: STOCK_TOOL_ID.to_string(),
        is_stock: true,
        kind: "gripper".to_string(),
        tcp_offset_m: [0.0; 3],
        config_digest: digest,
        pack_digest: None,
        commissioning_required: false,
    }
}

/// Read and parse a `tool.json` file from disk.
#[cfg(not(target_arch = "wasm32"))]
pub fn load_tool_file(path: impl AsRef<std::path::Path>) -> Result<ToolPackConfig, String> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("tool file {}: {e}", path.display()))?;
    serde_json::from_str(&text)
        .map_err(|e| format!("tool file {}: {e}", path.display()))
}

/// Validate `physical_contact_end_m` (finite, micro-unit range, norm <= [`MAX_CHECK_POINT_NORM_M`]).
pub fn validate_physical_contact_end_m(pc: &[f64; 3], label: &str) -> Result<(), String> {
    for (ax, &v) in pc.iter().enumerate() {
        checked_to_micro_f64(v, &format!("{label}[{ax}]"))?;
    }
    let norm = (pc[0] * pc[0] + pc[1] * pc[1] + pc[2] * pc[2]).sqrt();
    if norm > MAX_CHECK_POINT_NORM_M {
        return Err(format!(
            "{label} norm {norm:.4} m exceeds limit {MAX_CHECK_POINT_NORM_M:.2} m"
        ));
    }
    Ok(())
}

/// Validate `contact_opening_m` (finite, micro-unit range, in `[0.0, 0.20]` m).
pub fn validate_contact_opening_m(co: f64, label: &str) -> Result<(), String> {
    checked_to_micro_f64(co, label)?;
    if !(0.0..=0.20).contains(&co) {
        return Err(format!(
            "{label} {co} must be finite and in [0.0, 0.20] m"
        ));
    }
    Ok(())
}

/// Validate `tcp_offset` (finite, norm <= [`MAX_TCP_OFFSET_NORM_M`]).
pub fn validate_tcp_offset(tcp_offset: &[f64; 3]) -> Result<(), String> {
    if !tcp_offset.iter().all(|v| v.is_finite()) {
        return Err("tcp_offset values must be finite".into());
    }
    let norm = (tcp_offset[0] * tcp_offset[0]
        + tcp_offset[1] * tcp_offset[1]
        + tcp_offset[2] * tcp_offset[2])
        .sqrt();
    if norm > MAX_TCP_OFFSET_NORM_M {
        return Err(format!(
            "tcp_offset norm {norm:.4} m exceeds limit {MAX_TCP_OFFSET_NORM_M:.2} m"
        ));
    }
    Ok(())
}

/// Validate a slice of check points in `end_link` frame (finite, norm <= [`MAX_CHECK_POINT_NORM_M`]).
pub fn validate_check_points(points: &[[f32; 3]], label: &str) -> Result<(), String> {
    for (i, p) in points.iter().enumerate() {
        if !p.iter().all(|v| v.is_finite()) {
            return Err(format!("{label}[{i}] values must be finite"));
        }
        let norm = (f64::from(p[0]).powi(2) + f64::from(p[1]).powi(2) + f64::from(p[2]).powi(2))
            .sqrt();
        if norm > MAX_CHECK_POINT_NORM_M {
            return Err(format!(
                "{label}[{i}] norm {norm:.4} m exceeds limit {MAX_CHECK_POINT_NORM_M:.2} m"
            ));
        }
    }
    Ok(())
}

/// Validate `f64` check points in `end_link` frame (finite, norm <= [`MAX_CHECK_POINT_NORM_M`], safe micro range).
pub fn validate_check_points_f64(points: &[[f64; 3]], label: &str) -> Result<(), String> {
    for (i, p) in points.iter().enumerate() {
        for (ax, &v) in p.iter().enumerate() {
            checked_to_micro_f64(v, &format!("{label}[{i}][{ax}]"))?;
        }
        let norm = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        if norm > MAX_CHECK_POINT_NORM_M {
            return Err(format!(
                "{label}[{i}] norm {norm:.4} m exceeds limit {MAX_CHECK_POINT_NORM_M:.2} m"
            ));
        }
    }
    Ok(())
}

/// Apply a [`ToolPackConfig`] onto a cloned copy of `base`, validating all invariants before
/// returning. On error, `base` is untouched.
pub fn apply_tool_overlay(base: &ArmConfig, pack: &ToolPackConfig) -> Result<ArmConfig, String> {
    apply_tool_overlay_with_xml(base, pack, None)
}

/// Canonical summary returned by `armd --tool-info` and WASM `resolve_tool_config`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ResolvedToolConfigSummary {
    pub ok: bool,
    pub id: String,
    pub robot_id: String,
    pub is_stock: bool,
    pub kind: String,
    pub tcp_offset_m: [f64; 3],
    pub config_digest: String,
    pub canonical_json: String,
}

/// Resolve the canonical target robot ID for `pack` given an optional explicit override.
pub fn resolve_robot_target_for_pack(
    pack: &ToolPackConfig,
    explicit_robot: Option<&str>,
) -> Result<&'static str, String> {
    if let Some(raw) = explicit_robot {
        return normalize_robot_id(raw)
            .ok_or_else(|| format!("unknown robot profile '{raw}'"));
    }
    if pack.robots.is_empty() {
        return Err(format!(
            "tool '{}' has empty 'robots' list; specify --robot <id> explicitly",
            pack.id
        ));
    }
    if pack.robots.len() == 1 {
        let raw = &pack.robots[0];
        return normalize_robot_id(raw)
            .ok_or_else(|| format!("tool '{}': unknown robot target '{raw}'", pack.id));
    }
    let mut norms = Vec::with_capacity(pack.robots.len());
    for raw in &pack.robots {
        let norm = normalize_robot_id(raw)
            .ok_or_else(|| format!("tool '{}': unknown robot target '{raw}'", pack.id))?;
        norms.push(norm);
    }
    if norms.contains(&"b601dm") {
        Ok("b601dm")
    } else {
        Err(format!(
            "tool '{}' targets multiple robots {:?}; specify --robot <id> explicitly",
            pack.id, pack.robots
        ))
    }
}

/// Validate `pack` against `robot_id` (or `pack.robots` default) using the exact same
/// `baseline_arm_config_for_robot` + `apply_tool_overlay` pipeline as `armd` startup,
/// returning the authoritative summary + `canonical_json`.
pub fn resolve_tool_config_summary(
    pack: &ToolPackConfig,
    robot_id: Option<&str>,
) -> Result<ResolvedToolConfigSummary, String> {
    let norm_robot = resolve_robot_target_for_pack(pack, robot_id)?;
    let base = baseline_arm_config_for_robot(norm_robot)?;
    let (updated, canonical_json) = apply_tool_overlay_with_details(&base, pack, None)?;
    Ok(ResolvedToolConfigSummary {
        ok: true,
        id: updated.active_tool.id,
        robot_id: norm_robot.to_string(),
        is_stock: updated.active_tool.is_stock,
        kind: updated.active_tool.kind,
        tcp_offset_m: updated.active_tool.tcp_offset_m,
        config_digest: updated.active_tool.config_digest,
        canonical_json,
    })
}

/// Enforce that reserved `id == "default"` cannot carry any custom XML, composition directives,
/// non-default `tcp_site`, non-zero `tcp_offset`, or `armd` overlay before skipping XML resolution
/// or assigning stock identity.
pub fn validate_reserved_stock_pack(pack: &ToolPackConfig, tool_xml: Option<&str>) -> Result<(), String> {
    if pack.id != STOCK_TOOL_ID {
        return Ok(());
    }
    if pack.kind != "gripper" {
        return Err("reserved tool id 'default' must have kind 'gripper'".into());
    }
    if !pack.tool_xml.trim().is_empty() || tool_xml.is_some_and(|x| !x.trim().is_empty()) {
        return Err("reserved tool id 'default' cannot specify custom tool_xml".into());
    }
    if pack.tcp_offset.is_some_and(|o| o != [0.0; 3]) {
        return Err("reserved tool id 'default' cannot override tcp_offset".into());
    }
    if pack.tcp_site.as_deref().is_some_and(|s| s.trim() != "tcp") {
        return Err("reserved tool id 'default' cannot override tcp_site".into());
    }
    if !pack.replace_bodies.is_empty()
        || !pack.replace_geoms.is_empty()
        || !pack.replace_equalities.is_empty()
        || !pack.replace_actuators.is_empty()
        || pack.driven_joint.is_some()
        || pack.coupled_joint.is_some()
        || pack.actuator.is_some()
        || !pack.finger_geoms.is_empty()
        || !pack.io.is_empty()
        || !pack.assets.is_empty()
    {
        return Err(
            "reserved tool id 'default' cannot declare custom MJCF composition or hardware directives"
                .into(),
        );
    }
    if pack.armd.is_some() {
        return Err("reserved tool id 'default' cannot carry an 'armd' overlay block".into());
    }
    Ok(())
}

/// Apply a [`ToolPackConfig`] with optional `tool.xml` content (used only to enforce that
/// reserved `id == "default"` cannot carry custom `tool.xml`). Non-stock tools must explicitly
/// declare `tcp_offset: [x, y, z]`.
pub fn apply_tool_overlay_with_xml(
    base: &ArmConfig,
    pack: &ToolPackConfig,
    tool_xml: Option<&str>,
) -> Result<ArmConfig, String> {
    let (candidate, _) = apply_tool_overlay_with_details(base, pack, tool_xml)?;
    Ok(candidate)
}

fn apply_tool_overlay_with_details(
    base: &ArmConfig,
    pack: &ToolPackConfig,
    tool_xml: Option<&str>,
) -> Result<(ArmConfig, String), String> {
    if !valid_tool_id(&pack.id) {
        return Err("tool id must be 1..64 characters of A-Z, a-z, 0-9, _, -, .".into());
    }
    if !is_valid_tool_kind(&pack.kind) {
        return Err(format!(
            "tool '{}' has unsupported kind '{}': must be one of gripper, suction, screwdriver",
            pack.id, pack.kind
        ));
    }
    validate_attach_body(&pack.attach_body)?;

    let is_stock = pack.id == STOCK_TOOL_ID;
    let armd = pack.armd.as_ref();

    // Reserve `id == "default"` strictly for the genuine stock configuration BEFORE any XML skip.
    validate_reserved_stock_pack(pack, tool_xml)?;

    let mut candidate = base.clone();

    // 1. Robot compatibility check (fail-closed on unresolved or conflicting target).
    let base_robot_id = candidate
        .profile_id
        .as_deref()
        .and_then(normalize_robot_id);
    if !pack.robots.is_empty() {
        let mut allowed_robots = Vec::with_capacity(pack.robots.len());
        for raw in &pack.robots {
            let norm = normalize_robot_id(raw)
                .ok_or_else(|| format!("tool '{}': unknown robot target '{raw}'", pack.id))?;
            allowed_robots.push(norm);
        }
        let target_robot = base_robot_id.ok_or_else(|| {
            format!(
                "tool '{}': base profile_id {:?} cannot be resolved for restricted robots {:?}",
                pack.id, candidate.profile_id, pack.robots
            )
        })?;
        if !allowed_robots.contains(&target_robot) {
            return Err(format!(
                "tool '{}': robot '{target_robot}' is not in supported robots {:?}",
                pack.id, allowed_robots
            ));
        }
    }
    let resolved_robot_id = base_robot_id
        .unwrap_or_else(|| candidate.profile_id.as_deref().unwrap_or("unknown"))
        .to_string();

    // 2. Actuator capability check (runs unconditionally for non-stock tools, even when `armd` overlay is absent).
    let is_gripper = pack.kind == "gripper";
    if !is_stock && is_gripper && candidate.gripper_index().is_none() {
        return Err(format!(
            "tool '{}' has kind 'gripper' but profile has no gripper joint",
            pack.id
        ));
    }

    // 3. Resolve & validate TCP offset: non-stock tools must explicitly declare `tcp_offset: [x, y, z]`.
    let tcp_offset = if is_stock {
        pack.tcp_offset.unwrap_or([0.0; 3])
    } else {
        pack.tcp_offset.ok_or_else(|| {
            format!(
                "non-stock tool '{}' must declare explicit tcp_offset: [x, y, z]",
                pack.id
            )
        })?
    };
    for (ax, &v) in tcp_offset.iter().enumerate() {
        checked_to_micro_f64(v, &format!("tcp_offset[{ax}]"))?;
    }
    validate_tcp_offset(&tcp_offset)?;
    if is_stock && tcp_offset != [0.0; 3] {
        return Err("reserved tool id 'default' cannot resolve a non-zero tcp_offset".into());
    }

    if let Some(a) = armd {
        validate_check_points_f64(&a.extra_check_points_end_m, "armd.extra_check_points_end_m")?;
        if let Some(pc) = &a.physical_contact_end_m {
            validate_physical_contact_end_m(pc, "armd.physical_contact_end_m")?;
        }
        if let Some(co) = a.contact_opening_m {
            validate_contact_opening_m(co, "armd.contact_opening_m")?;
        }
        if let Some(d) = &a.dynamics {
            if let Some(m) = d.mass_delta_kg {
                checked_to_micro_f64(m, "armd.dynamics.mass_delta_kg")?;
            }
            if let Some(c) = d.com_moment_delta_nm {
                checked_to_micro_f64(c, "armd.dynamics.com_moment_delta_nm")?;
            }
        }
    }

    let extra_points_f64: Vec<[f64; 3]> = armd
        .map(|a| a.extra_check_points_end_m.clone())
        .unwrap_or_default();
    let extra_points: Vec<[f32; 3]> = extra_points_f64
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect();

    // 4. Non-zero offset or extra points require kinematics and workspace on the profile.
    if (tcp_offset != [0.0; 3] || !extra_points.is_empty())
        && (candidate.kinematics.is_none() || candidate.workspace.is_none())
    {
        return Err(format!(
            "tool '{}' has nonzero tcp_offset or extra_check_points_end_m, which requires kinematics and workspace on the profile",
            pack.id
        ));
    }

    // 5. Merge gripper policy & grasp setup (or disable for non-jaw tools), preserving f64 for digest.
    let mut canonical_gp_input: Option<CanonicalGripperInput> = None;
    let (base_max_w, base_grasp_h) = baseline_grasp_limits(&candidate);
    let mut effective_max_grasp_width_m: Option<f64> = None;
    let mut effective_grasp_height_m: Option<f64> = None;
    let effective_edge_to_low_m: Option<f64>;

    if !is_gripper {
        if armd.is_some_and(|a| a.gripper_policy.is_some() || a.grasp.is_some()) {
            return Err(format!(
                "tool '{}' with kind '{}' cannot specify armd.gripper_policy or armd.grasp",
                pack.id, pack.kind
            ));
        }
        candidate.gripper_policy = None;
        #[cfg(not(target_arch = "wasm32"))]
        {
            candidate.grasp = None;
        }
    } else {
        if let Some(gp_ov) = armd.and_then(|a| a.gripper_policy.as_ref()) {
            let mut gp = candidate.gripper_policy.clone().ok_or_else(|| {
                format!(
                    "tool '{}': profile has no base gripper_policy to overlay",
                    pack.id
                )
            })?;
            let mut gp_f64 = CanonicalGripperInput::from(&gp);
            if let Some(v) = gp_ov.closed_rad {
                checked_to_micro_f64(v, "armd.gripper_policy.closed_rad")?;
                gp.closed_rad = v as f32;
                gp_f64.closed_rad = v;
            }
            if let Some(v) = gp_ov.open_rad {
                checked_to_micro_f64(v, "armd.gripper_policy.open_rad")?;
                gp.open_rad = v as f32;
                gp_f64.open_rad = v;
            }
            if let Some(v) = gp_ov.holding_gap_rad {
                checked_to_micro_f64(v, "armd.gripper_policy.holding_gap_rad")?;
                gp.holding_gap_rad = v as f32;
                gp_f64.holding_gap_rad = v;
            }
            if let Some(v) = gp_ov.holding_torque_nm {
                checked_to_micro_f64(v, "armd.gripper_policy.holding_torque_nm")?;
                gp.holding_torque_nm = v as f32;
                gp_f64.holding_torque_nm = v;
            }
            if let Some(v) = gp_ov.squeeze_rad {
                checked_to_micro_f64(v, "armd.gripper_policy.squeeze_rad")?;
                gp.squeeze_rad = v as f32;
                gp_f64.squeeze_rad = v;
            }
            if let Some(v) = gp_ov.max_close_torque_nm {
                checked_to_micro_f64(v, "armd.gripper_policy.max_close_torque_nm")?;
                gp.max_close_torque_nm = Some(v as f32);
                gp_f64.max_close_torque_nm = Some(v);
            }
            if let Some(v) = gp_ov.contact_torque_nm {
                checked_to_micro_f64(v, "armd.gripper_policy.contact_torque_nm")?;
                gp.contact_torque_nm = Some(v as f32);
                gp_f64.contact_torque_nm = Some(v);
            }
            if let Some(pts) = &gp_ov.jaw_opening_nominal {
                for (i, p) in pts.iter().enumerate() {
                    checked_to_micro_f64(p[0], &format!("armd.gripper_policy.jaw_opening_nominal[{i}][0]"))?;
                    checked_to_micro_f64(p[1], &format!("armd.gripper_policy.jaw_opening_nominal[{i}][1]"))?;
                }
                gp.jaw_opening = Some(JawOpening {
                    points: pts.iter().map(|p| [p[0] as f32, p[1] as f32]).collect(),
                    calibrated: false,
                });
                gp_f64.jaw_opening_nominal = Some(pts.clone());
            } else if !is_stock {
                gp.jaw_opening = None;
                gp_f64.jaw_opening_nominal = None;
            }
            candidate.gripper_policy = Some(gp);
            canonical_gp_input = Some(gp_f64);
        } else {
            if !is_stock {
                if let Some(gp) = candidate.gripper_policy.as_mut() {
                    gp.jaw_opening = None;
                }
            }
            canonical_gp_input = candidate.gripper_policy.as_ref().map(CanonicalGripperInput::from);
        }

        effective_max_grasp_width_m = base_max_w;
        effective_grasp_height_m = base_grasp_h;

        if let Some(gr_ov) = armd.and_then(|a| a.grasp.as_ref()) {
            if let Some(e) = gr_ov.edge_to_low_m {
                checked_to_micro_f64(e, "armd.grasp.edge_to_low_m")?;
                if !(-0.10..=0.10).contains(&e) {
                    return Err("armd.grasp.edge_to_low_m must be finite and in [-0.10, 0.10] m".into());
                }
            }
            if let Some(w) = gr_ov.max_grasp_width_m {
                checked_to_micro_f64(w, "armd.grasp.max_grasp_width_m")?;
                if !(0.0..=0.2).contains(&w) {
                    return Err("armd.grasp.max_grasp_width_m must be in [0, 0.2] m".into());
                }
                effective_max_grasp_width_m = Some(w);
            }
            if let Some(h) = gr_ov.grasp_height_m {
                checked_to_micro_f64(h, "armd.grasp.grasp_height_m")?;
                if !(0.005..=0.2).contains(&h) {
                    return Err("armd.grasp.grasp_height_m must be in [0.005, 0.2] m".into());
                }
                effective_grasp_height_m = Some(h);
            }
            #[cfg(not(target_arch = "wasm32"))]
            if let Some(g) = candidate.grasp.as_mut() {
                if let Some(v) = gr_ov.max_grasp_width_m { g.max_grasp_width_m = v; }
                if let Some(v) = gr_ov.grasp_height_m { g.grasp_height_m = v; }
            }
        }
    }

    if !is_stock {
        effective_edge_to_low_m = armd
            .and_then(|a| a.grasp.as_ref())
            .and_then(|g| g.edge_to_low_m);
        candidate.calibration_tool = Some(CalibrationToolSpec {
            tool_id: pack.id.clone(),
            edge_to_low_m: effective_edge_to_low_m,
        });
    } else {
        effective_edge_to_low_m = candidate
            .calibration_tool
            .as_ref()
            .and_then(|c| c.edge_to_low_m);
    }

    // 6. Initialize layer 1 if needed, then update layers 2 & 3 and rebuild the 4-layer union.
    if candidate.base_check_points_end_m.is_empty() {
        if let Some(ws) = &candidate.workspace {
            candidate.base_check_points_end_m = ws.check_points_end_m.clone();
        }
    }
    candidate.tcp_offset = tcp_offset;
    candidate.permanent_tool_check_points_end_m = extra_points;
    let contact_opening_f64 = armd
        .and_then(|a| a.contact_opening_m)
        .or_else(|| is_stock.then_some(0.0));
    candidate.physical_contact_end_m = armd
        .and_then(|a| a.physical_contact_end_m)
        .or_else(|| is_stock.then_some([0.0; 3]));
    candidate.contact_opening_m = contact_opening_f64.map(|v| v as f32);
    candidate.rebuild_effective_check_points()?;

    // 7. Dynamics qualification gate.
    candidate.dynamics_qualified = if is_stock && armd.and_then(|a| a.dynamics.as_ref()).is_none() {
        true
    } else {
        ToolDynamics::is_qualified(armd.and_then(|a| a.dynamics.as_ref()))
    };

    // 8. Unified post-merge GripperPolicy + JawOpening + max_grasp_width_m validation.
    let gripper_lim = candidate
        .gripper_index()
        .and_then(|idx| candidate.limits.get(idx));
    if let Some(gp) = candidate.gripper_policy.as_ref() {
        validate_effective_gripper_policy(gp, gripper_lim, effective_max_grasp_width_m)?;
    }

    // 9. Compute canonical digest from unnarrowed f64 values and populate ActiveToolInfo.
    let canonical_json = canonical_tool_payload_json_f64(
        &resolved_robot_id,
        &pack.id,
        &pack.kind,
        candidate.tcp_offset,
        &extra_points_f64,
        candidate.physical_contact_end_m,
        contact_opening_f64,
        armd.and_then(|a| a.dynamics.as_ref()),
        canonical_gp_input.as_ref(),
        effective_max_grasp_width_m,
        effective_grasp_height_m,
        effective_edge_to_low_m,
    );
    let config_digest = format!("v1:{}", crate::gravity::compute_sha256(canonical_json.as_bytes()));
    candidate.active_tool = ActiveToolInfo {
        schema_version: 1,
        id: pack.id.clone(),
        is_stock,
        kind: pack.kind.clone(),
        tcp_offset_m: candidate.tcp_offset,
        config_digest,
        pack_digest: None,
        commissioning_required: !is_stock,
    };

    candidate.validate()?;
    Ok((candidate, canonical_json))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ObservationSchema;

    const SOFT_FINRAY_TOOL_JSON: &str = r#"{
      "id": "soft_finray_gripper",
      "name": "Soft FinRay Parallel Gripper (Plan A)",
      "kind": "gripper",
      "version": "1.0.0",
      "description": "FinRay compliant fingers (332000195) on rigid mounts (332000196) bolted to stock B601-DM 2-SLIDER-FIX (10x15mm M3). +28.8mm tip extension, 3.6mm closed gap.",
      "tool_xml": "tool.xml",
      "robots": ["b601dm"],
      "attach_body": "end_link",
      "replace_bodies": ["finger_left", "finger_right"],
      "replace_geoms": [],
      "replace_equalities": ["jaw_coupling"],
      "replace_actuators": ["m7"],
      "driven_joint": "finray_jaw_left",
      "coupled_joint": "finray_jaw_right",
      "actuator": "m7_finray",
      "motor_to_joint_scale": -0.007456140350877194,
      "finger_geoms": ["finray_left_tip", "finray_left_body", "finray_right_tip", "finray_right_body"],
      "tcp_site": "finray_tcp",
      "tcp_offset": [0.0288, 0.0, 0.0],
      "io": [],
      "assets": [],
      "armd": {
        "extra_check_points_end_m": [
          [0.032,  0.082,  0.022],
          [0.032,  0.082, -0.022],
          [0.032, -0.082,  0.022],
          [0.032, -0.082, -0.022],
          [0.032,  0.000,  0.022],
          [0.032,  0.000, -0.022],
          [0.000,  0.082,  0.022],
          [0.000,  0.082, -0.022],
          [0.000, -0.082,  0.022],
          [0.000, -0.082, -0.022]
        ],
        "physical_contact_end_m": [0.0288, 0.0, 0.0],
        "contact_opening_m": 0.0036,
        "dynamics": {
          "mass_delta_kg": 0.035,
          "com_moment_delta_nm": 0.018
        },
        "gripper_policy": {
          "jaw_opening_nominal": [[-5.7, 0.0886], [0.0, 0.0036]],
          "holding_gap_rad": 0.30,
          "holding_torque_nm": 0.0667,
          "contact_torque_nm": 0.6,
          "squeeze_rad": 0.15,
          "max_close_torque_nm": 1.0
        },
        "grasp": {
          "max_grasp_width_m": 0.075,
          "grasp_height_m": 0.015,
          "edge_to_low_m": null
        }
      }
    }"#;

    #[test]
    fn canonical_digest_and_byte_vectors_for_finray_and_default() {
        let base = ArmConfig::rebot_dm();
        let pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        let applied = apply_tool_overlay(&base, &pack).unwrap();

        let finray_json = canonical_tool_payload_json_f64(
            "b601dm",
            "soft_finray_gripper",
            "gripper",
            applied.tcp_offset,
            &pack.armd.as_ref().unwrap().extra_check_points_end_m,
            applied.physical_contact_end_m,
            pack.armd.as_ref().unwrap().contact_opening_m,
            pack.armd.as_ref().and_then(|a| a.dynamics.as_ref()),
            applied.gripper_policy.as_ref().map(CanonicalGripperInput::from).as_ref(),
            Some(0.075),
            Some(0.015),
            None,
        );
        let expected_finray_json = concat!(
            r#"{"contact_opening_um":3600,"#,
            r#""dynamics":{"com_moment_delta_unm":18000,"mass_delta_mg":35000},"#,
            r#""edge_to_low_um":null,"#,
            r#""extra_points_um":[[0,-82000,-22000],[0,-82000,22000],[0,82000,-22000],[0,82000,22000],[32000,-82000,-22000],[32000,-82000,22000],[32000,0,-22000],[32000,0,22000],[32000,82000,-22000],[32000,82000,22000]],"#,
            r#""grasp_height_um":15000,"#,
            r#""gripper_policy":{"closed_urad":0,"contact_torque_unm":600000,"holding_gap_urad":300000,"holding_load_fraction_um":null,"holding_mode":"torque_and_gap","holding_torque_unm":66700,"jaw_opening_nominal":[[-5700000,88600],[0,3600]],"max_close_torque_unm":1000000,"max_lead_urad":350000,"open_urad":-5700000,"squeeze_urad":150000,"stream_contact_speed_urad_s":300000},"#,
            r#""kind":"gripper","#,
            r#""max_grasp_width_um":75000,"#,
            r#""physical_contact_um":[28800,0,0],"#,
            r#""robot_id":"b601dm","#,
            r#""tcp_offset_um":[28800,0,0],"#,
            r#""tool_id":"soft_finray_gripper","#,
            r#""v":1}"#
        );
        const EXPECTED_FINRAY_DIGEST: &str =
            "v1:bc4e643de8bf65ec9537c2b22ddbb1ae230e14a091a169cd4d199b955af49776";
        eprintln!("CANONICAL_FINRAY_JSON={finray_json}");
        eprintln!("CANONICAL_FINRAY_DIGEST={}", applied.active_tool.config_digest);
        assert_eq!(finray_json, expected_finray_json);
        assert_eq!(applied.active_tool.config_digest, EXPECTED_FINRAY_DIGEST);
        assert!(applied.active_tool.config_digest.starts_with("v1:"));
        assert_eq!(applied.active_tool.config_digest.len(), 3 + 64);
        assert_eq!(applied.active_tool.id, "soft_finray_gripper");
        assert!(!applied.active_tool.is_stock);
        assert!(applied.active_tool.commissioning_required);
        assert!(applied.dynamics_qualified);
        assert_eq!(applied.workspace.as_ref().unwrap().check_points_end_m.len(), 19);

        let default_json = canonical_tool_payload_json(
            "b601dm",
            "default",
            "gripper",
            [0.0; 3],
            &[],
            Some([0.0; 3]),
            Some(0.0),
            None,
            base.gripper_policy.as_ref(),
            Some(0.075),
            Some(0.015),
            Some(0.0023),
        );
        let expected_default_json = concat!(
            r#"{"contact_opening_um":0,"#,
            r#""dynamics":null,"#,
            r#""edge_to_low_um":2300,"#,
            r#""extra_points_um":[],"#,
            r#""grasp_height_um":15000,"#,
            r#""gripper_policy":{"closed_urad":0,"contact_torque_unm":600000,"holding_gap_urad":300000,"holding_load_fraction_um":null,"holding_mode":"torque_and_gap","holding_torque_unm":66700,"jaw_opening_nominal":null,"max_close_torque_unm":1000000,"max_lead_urad":350000,"open_urad":-5700000,"squeeze_urad":150000,"stream_contact_speed_urad_s":300000},"#,
            r#""kind":"gripper","#,
            r#""max_grasp_width_um":75000,"#,
            r#""physical_contact_um":[0,0,0],"#,
            r#""robot_id":"b601dm","#,
            r#""tcp_offset_um":[0,0,0],"#,
            r#""tool_id":"default","#,
            r#""v":1}"#
        );
        assert_eq!(default_json, expected_default_json);
        let schema = ObservationSchema::from_config(&base);
        assert!(schema.active_tool.is_stock);
        assert_eq!(schema.active_tool.id, "default");
        assert_eq!(schema.active_tool.schema_version, 1);
        assert!(!schema.active_tool.commissioning_required);
        assert!(schema.active_tool.config_digest.starts_with("v1:"));
        assert_eq!(schema.active_tool.config_digest.len(), 3 + 64);
        const EXPECTED_DEFAULT_DIGEST: &str =
            "v1:ac4d69ae16085196687d381a4dbfa89a1940a08522dd088f2459645fd91d1064";
        eprintln!("CANONICAL_DEFAULT_JSON={default_json}");
        eprintln!("CANONICAL_DEFAULT_DIGEST={}", schema.active_tool.config_digest);
        assert_eq!(schema.active_tool.config_digest, EXPECTED_DEFAULT_DIGEST);

        let so101 = baseline_arm_config_for_robot("so101").unwrap();
        let so101_schema = ObservationSchema::from_config(&so101);
        let so101_json = canonical_tool_payload_json(
            "so101",
            "default",
            "gripper",
            [0.0; 3],
            &[],
            Some([0.0; 3]),
            Some(0.0),
            None,
            so101.gripper_policy.as_ref(),
            Some(0.065),
            Some(0.015),
            None,
        );
        let expected_so101_json = concat!(
            r#"{"contact_opening_um":0,"#,
            r#""dynamics":null,"#,
            r#""edge_to_low_um":null,"#,
            r#""extra_points_um":[],"#,
            r#""grasp_height_um":15000,"#,
            r#""gripper_policy":{"closed_urad":0,"contact_torque_unm":null,"holding_gap_urad":80000,"holding_load_fraction_um":180000,"holding_mode":"load_and_gap","holding_torque_unm":0,"jaw_opening_nominal":[[0,0],[1500000,65000]],"max_close_torque_unm":null,"max_lead_urad":null,"open_urad":1500000,"squeeze_urad":50000,"stream_contact_speed_urad_s":null},"#,
            r#""kind":"gripper","#,
            r#""max_grasp_width_um":65000,"#,
            r#""physical_contact_um":[0,0,0],"#,
            r#""robot_id":"so101","#,
            r#""tcp_offset_um":[0,0,0],"#,
            r#""tool_id":"default","#,
            r#""v":1}"#
        );
        const EXPECTED_SO101_DIGEST: &str =
            "v1:71cfcf4dafaa5a52c2366c3ace757f714628321b761f195f4456a7ae2ba26a42";
        eprintln!("CANONICAL_SO101_JSON={so101_json}");
        eprintln!("CANONICAL_SO101_DIGEST={}", so101_schema.active_tool.config_digest);
        assert_eq!(so101_json, expected_so101_json);
        assert_eq!(so101_schema.active_tool.config_digest, EXPECTED_SO101_DIGEST);

        // Vector 4: default (so100)
        let so100 = baseline_arm_config_for_robot("so100").unwrap();
        let so100_schema = ObservationSchema::from_config(&so100);
        let so100_json = canonical_tool_payload_json(
            "so100",
            "default",
            "gripper",
            [0.0; 3],
            &[],
            Some([0.0; 3]),
            Some(0.0),
            None,
            so100.gripper_policy.as_ref(),
            Some(0.065),
            Some(0.015),
            None,
        );
        let expected_so100_json = expected_so101_json.replace(r#""robot_id":"so101""#, r#""robot_id":"so100""#);
        const EXPECTED_SO100_DIGEST: &str =
            "v1:efbb9e76729b3b748959c032a789412e0ef0224d197079d16f688a32f82463d4";
        eprintln!("CANONICAL_SO100_JSON={so100_json}");
        eprintln!("CANONICAL_SO100_DIGEST={}", so100_schema.active_tool.config_digest);
        assert_eq!(so100_json, expected_so100_json);
        assert_eq!(so100_schema.active_tool.config_digest, EXPECTED_SO100_DIGEST);

        // Vector 5: non-stock SO-101 pack inheriting SO-101 gripper_policy defaults
        let so101_inherited_pack: ToolPackConfig = serde_json::from_value(serde_json::json!({
            "id": "so101_inherited_gripper",
            "kind": "gripper",
            "robots": ["so101"],
            "tcp_offset": [0.02, 0.0, 0.0],
            "armd": {
                "gripper_policy": {
                    "jaw_opening_nominal": [[0.0, 0.0], [1.5, 0.070]]
                }
            }
        })).unwrap();
        let so101_inherited = apply_tool_overlay(&so101, &so101_inherited_pack).unwrap();
        let so101_inherited_json = canonical_tool_payload_json_f64(
            "so101",
            "so101_inherited_gripper",
            "gripper",
            so101_inherited.tcp_offset,
            &[],
            so101_inherited.physical_contact_end_m,
            None,
            None,
            so101_inherited.gripper_policy.as_ref().map(CanonicalGripperInput::from).as_ref(),
            Some(0.065),
            Some(0.015),
            None,
        );
        let expected_so101_inherited_json = concat!(
            r#"{"contact_opening_um":null,"#,
            r#""dynamics":null,"#,
            r#""edge_to_low_um":null,"#,
            r#""extra_points_um":[],"#,
            r#""grasp_height_um":15000,"#,
            r#""gripper_policy":{"closed_urad":0,"contact_torque_unm":null,"holding_gap_urad":80000,"holding_load_fraction_um":180000,"holding_mode":"load_and_gap","holding_torque_unm":0,"jaw_opening_nominal":[[0,0],[1500000,70000]],"max_close_torque_unm":null,"max_lead_urad":null,"open_urad":1500000,"squeeze_urad":50000,"stream_contact_speed_urad_s":null},"#,
            r#""kind":"gripper","#,
            r#""max_grasp_width_um":65000,"#,
            r#""physical_contact_um":null,"#,
            r#""robot_id":"so101","#,
            r#""tcp_offset_um":[20000,0,0],"#,
            r#""tool_id":"so101_inherited_gripper","#,
            r#""v":1}"#
        );
        const EXPECTED_SO101_INHERITED_DIGEST: &str =
            "v1:6f50d92eb5709a880819d6ddcce4e2c78dad34986929319cb444db6265087d39";
        eprintln!("CANONICAL_SO101_INHERITED_JSON={so101_inherited_json}");
        eprintln!("CANONICAL_SO101_INHERITED_DIGEST={}", so101_inherited.active_tool.config_digest);
        assert_eq!(so101_inherited_json, expected_so101_inherited_json);
        assert_eq!(so101_inherited.active_tool.config_digest, EXPECTED_SO101_INHERITED_DIGEST);

        // Vector 6: b601rs no-gripper expectation (`gripper_policy: null`)
        let b601rs = baseline_arm_config_for_robot("b601rs").unwrap();
        assert!(b601rs.gripper_policy.is_none());
        let b601rs_schema = ObservationSchema::from_config(&b601rs);
        let b601rs_json = canonical_tool_payload_json(
            "b601rs",
            "default",
            "gripper",
            [0.0; 3],
            &[],
            Some([0.0; 3]),
            Some(0.0),
            None,
            b601rs.gripper_policy.as_ref(),
            None,
            None,
            None,
        );
        let expected_b601rs_json = concat!(
            r#"{"contact_opening_um":0,"#,
            r#""dynamics":null,"#,
            r#""edge_to_low_um":null,"#,
            r#""extra_points_um":[],"#,
            r#""grasp_height_um":null,"#,
            r#""gripper_policy":null,"#,
            r#""kind":"gripper","#,
            r#""max_grasp_width_um":null,"#,
            r#""physical_contact_um":[0,0,0],"#,
            r#""robot_id":"b601rs","#,
            r#""tcp_offset_um":[0,0,0],"#,
            r#""tool_id":"default","#,
            r#""v":1}"#
        );
        const EXPECTED_B601RS_DIGEST: &str =
            "v1:940a10d6d0dadaa2c6d2fdf7771cb8e226805c3300233dff9661fc5b8cf89636";
        eprintln!("CANONICAL_B601RS_JSON={b601rs_json}");
        eprintln!("CANONICAL_B601RS_DIGEST={}", b601rs_schema.active_tool.config_digest);
        assert_eq!(b601rs_json, expected_b601rs_json);
        assert_eq!(b601rs_schema.active_tool.config_digest, EXPECTED_B601RS_DIGEST);

        // Changing holding_mode, max_lead_rad, or holding_load_fraction MUST change canonical digest (M13)
        let mut gp_hm = base.gripper_policy.clone().unwrap();
        gp_hm.holding_mode = crate::model::HoldingMode::PositionGap;
        let hm_json = canonical_tool_payload_json_f64(
            "b601dm", "default", "gripper", [0.0; 3], &[], Some([0.0; 3]), Some(0.0), None,
            Some(&CanonicalGripperInput::from(&gp_hm)), Some(0.075), Some(0.015), Some(0.0023),
        );
        assert_ne!(hm_json, default_json);
        let mut ml_gp = base.gripper_policy.clone().unwrap();
        ml_gp.max_lead_rad = Some(0.20);
        let ml_json = canonical_tool_payload_json_f64(
            "b601dm", "default", "gripper", [0.0; 3], &[], Some([0.0; 3]), Some(0.0), None,
            Some(&CanonicalGripperInput::from(&ml_gp)), Some(0.075), Some(0.015), Some(0.0023),
        );
        assert_ne!(ml_json, default_json);
        let mut so101_frac = so101.clone();
        so101_frac.gripper_policy.as_mut().unwrap().holding_load_fraction = Some(0.30);
        let frac_digest = stock_active_tool_info(&so101_frac).config_digest;
        assert_ne!(frac_digest, so101_schema.active_tool.config_digest);

        // Signed half-away-from-zero boundary vectors (+0.5 / -0.5 micro-unit ties), M09 magnitude cap, and no f32 narrowing
        assert_eq!(checked_to_micro_f64(0.0000005, "pos_half").unwrap(), 1);
        assert_eq!(checked_to_micro_f64(-0.0000005, "neg_half").unwrap(), -1);
        assert_eq!(checked_to_micro_f64(-0.0, "neg_zero").unwrap(), 0);
        assert_eq!(checked_to_micro_f64(1_000_000_000.0, "max_pos").unwrap(), 1_000_000_000_000_000);
        assert_eq!(checked_to_micro_f64(-1_000_000_000.0, "max_neg").unwrap(), -1_000_000_000_000_000);
        assert!(checked_to_micro_f64(1_000_000_000.000001, "over_1e9").is_err());
        assert!(checked_to_micro_f64(4_503_599_627.370497, "over_f64_tie").is_err());
        let mut tie_pack = pack.clone();
        tie_pack.tcp_offset = Some([0.0000005, -0.0000005, 0.0]);
        let tie_applied = apply_tool_overlay(&base, &tie_pack).unwrap();
        assert_ne!(tie_applied.active_tool.config_digest, applied.active_tool.config_digest);
        // Changing grasp_height_m and edge_to_low_m MUST change the digest (M17)
        let mut h_pack = pack.clone();
        h_pack.armd.as_mut().unwrap().grasp.as_mut().unwrap().grasp_height_m = Some(0.025);
        assert_ne!(apply_tool_overlay(&base, &h_pack).unwrap().active_tool.config_digest, applied.active_tool.config_digest);
        let mut e_pack = pack;
        e_pack.armd.as_mut().unwrap().grasp.as_mut().unwrap().edge_to_low_m = Some(-0.005);
        assert_ne!(apply_tool_overlay(&base, &e_pack).unwrap().active_tool.config_digest, applied.active_tool.config_digest);

        eprintln!("CANONICAL_FINRAY_JSON={finray_json}");
        eprintln!("CANONICAL_FINRAY_DIGEST={}", applied.active_tool.config_digest);
        eprintln!("CANONICAL_DEFAULT_JSON={default_json}");
        eprintln!("CANONICAL_DEFAULT_DIGEST={}", schema.active_tool.config_digest);
    }

    #[test]
    fn rejects_invalid_tool_overlays_without_mutating_base() {
        let base = ArmConfig::rebot_dm();
        let before = base.clone();

        // 1. Unknown field inside `armd` (deny_unknown_fields)
        let bad_unknown = r#"{"id":"bad","kind":"gripper","robots":["b601dm"],"armd":{"unknown_key":1}}"#;
        assert!(serde_json::from_str::<ToolPackConfig>(bad_unknown).is_err());

        // 2. Out-of-bounds tcp_offset (> 0.30 m)
        let mut pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        pack.tcp_offset = Some([0.31, 0.0, 0.0]);
        assert!(apply_tool_overlay(&base, &pack).unwrap_err().contains("tcp_offset"));

        // 3. Reversed jaw_opening (open_rad narrower than closed_rad)
        let mut pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        pack.armd.as_mut().unwrap().gripper_policy.as_mut().unwrap().jaw_opening_nominal =
            Some(vec![[-5.7, 0.0036], [0.0, 0.0886]]);
        assert!(apply_tool_overlay(&base, &pack).unwrap_err().contains("wider"));

        // 4. Incomplete angular span coverage
        let mut pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        pack.armd.as_mut().unwrap().gripper_policy.as_mut().unwrap().jaw_opening_nominal =
            Some(vec![[-3.0, 0.0886], [0.0, 0.0036]]);
        assert!(apply_tool_overlay(&base, &pack).unwrap_err().contains("cover"));

        // 5. Reachable width < max_grasp_width_m
        let mut pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        pack.armd.as_mut().unwrap().grasp.as_mut().unwrap().max_grasp_width_m = Some(0.12);
        assert!(apply_tool_overlay(&base, &pack).unwrap_err().contains("max_grasp_width_m"));

        // 6. Robot mismatch or unresolved robot
        let mut pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        pack.robots = vec!["so-arm101".into()];
        assert!(apply_tool_overlay(&base, &pack).is_err());
        pack.robots = vec!["mystery_bot".into()];
        assert!(apply_tool_overlay(&base, &pack).is_err());

        // 7. Dynamics qualification gates
        let mut pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        pack.armd.as_mut().unwrap().dynamics = None;
        assert!(!apply_tool_overlay(&base, &pack).unwrap().dynamics_qualified);
        pack.armd.as_mut().unwrap().dynamics = Some(ToolDynamics {
            mass_delta_kg: Some(0.16),
            com_moment_delta_nm: Some(0.01),
        });
        assert!(!apply_tool_overlay(&base, &pack).unwrap().dynamics_qualified);
        pack.armd.as_mut().unwrap().dynamics = Some(ToolDynamics {
            mass_delta_kg: Some(0.05),
            com_moment_delta_nm: Some(0.06),
        });
        assert!(!apply_tool_overlay(&base, &pack).unwrap().dynamics_qualified);

        // 8. Check-point norm (> 0.50 m) and total count (> 32) rejections
        let mut pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        pack.armd.as_mut().unwrap().extra_check_points_end_m = vec![[0.51, 0.0, 0.0]];
        assert!(apply_tool_overlay(&base, &pack).unwrap_err().contains("0.50"));
        pack.armd.as_mut().unwrap().extra_check_points_end_m =
            (0..25).map(|i| [0.01 * i as f64, 0.05, 0.01]).collect();
        assert!(apply_tool_overlay(&base, &pack).unwrap_err().contains("32"));

        // B2: Reserved `default` identity rejects non-zero offset, armd overlay, non-gripper, bad attach_body
        let bad_default_offset = ToolPackConfig {
            id: "default".into(),
            name: "Fake Default".into(),
            kind: "gripper".into(),
            version: "1.0.0".into(),
            description: String::new(),
            tool_xml: String::new(),
            robots: vec!["b601dm".into()],
            attach_body: "end_link".into(),
            replace_bodies: vec![],
            replace_geoms: vec![],
            replace_equalities: vec![],
            replace_actuators: vec![],
            driven_joint: None,
            coupled_joint: None,
            actuator: None,
            motor_to_joint_scale: None,
            finger_geoms: vec![],
            tcp_site: None,
            tcp_offset: Some([0.02, 0.0, 0.0]),
            suction_range_m: None,
            io: vec![],
            assets: vec![],
            armd: None,
        };
        assert!(apply_tool_overlay(&base, &bad_default_offset).unwrap_err().contains("default"));
        let mut bad_default_armd = bad_default_offset.clone();
        bad_default_armd.tcp_offset = None;
        bad_default_armd.armd = Some(ArmdToolOverlay::default());
        assert!(apply_tool_overlay(&base, &bad_default_armd).unwrap_err().contains("default"));

        // M04 (R3): Reserved `default` rejects custom tool_xml, non-"tcp" tcp_site, and MJCF/hw directives
        let stock_clean = ToolPackConfig { tcp_offset: None, ..bad_default_offset };
        assert!(apply_tool_overlay(&base, &stock_clean).is_ok());

        let mut bad_default_xml_field = stock_clean.clone();
        bad_default_xml_field.tool_xml = "tool.xml".into();
        assert!(apply_tool_overlay(&base, &bad_default_xml_field).unwrap_err().contains("default"));

        let displaced_xml = r#"<mujoco><worldbody><site name="tcp" pos="0.05 0 0"/></worldbody></mujoco>"#;
        assert!(apply_tool_overlay_with_xml(&base, &stock_clean, Some(displaced_xml)).unwrap_err().contains("default"));

        let mut bad_default_site = stock_clean.clone();
        bad_default_site.tcp_site = Some("finray_tcp".into());
        assert!(apply_tool_overlay(&base, &bad_default_site).unwrap_err().contains("default"));

        let mut bad_default_bodies = stock_clean.clone();
        bad_default_bodies.replace_bodies = vec!["finger_left".into()];
        assert!(apply_tool_overlay(&base, &bad_default_bodies).unwrap_err().contains("default"));

        let mut bad_default_geoms = stock_clean.clone();
        bad_default_geoms.replace_geoms = vec!["palm".into()];
        assert!(apply_tool_overlay(&base, &bad_default_geoms).unwrap_err().contains("default"));

        let mut bad_default_eq = stock_clean.clone();
        bad_default_eq.replace_equalities = vec!["jaw_coupling".into()];
        assert!(apply_tool_overlay(&base, &bad_default_eq).unwrap_err().contains("default"));

        let mut bad_default_act = stock_clean.clone();
        bad_default_act.replace_actuators = vec!["m7".into()];
        assert!(apply_tool_overlay(&base, &bad_default_act).unwrap_err().contains("default"));

        let mut bad_default_driven = stock_clean.clone();
        bad_default_driven.driven_joint = Some("j".into());
        assert!(apply_tool_overlay(&base, &bad_default_driven).unwrap_err().contains("default"));

        let mut bad_default_coupled = stock_clean.clone();
        bad_default_coupled.coupled_joint = Some("j2".into());
        assert!(apply_tool_overlay(&base, &bad_default_coupled).unwrap_err().contains("default"));

        let mut bad_default_actuator = stock_clean.clone();
        bad_default_actuator.actuator = Some("m".into());
        assert!(apply_tool_overlay(&base, &bad_default_actuator).unwrap_err().contains("default"));

        let mut bad_default_fingers = stock_clean.clone();
        bad_default_fingers.finger_geoms = vec!["left_front".into()];
        assert!(apply_tool_overlay(&base, &bad_default_fingers).unwrap_err().contains("default"));

        let mut bad_default_io = stock_clean.clone();
        bad_default_io.io = vec![serde_json::json!({"name":"x"})];
        assert!(apply_tool_overlay(&base, &bad_default_io).unwrap_err().contains("default"));

        let mut bad_default_assets = stock_clean;
        bad_default_assets.assets = vec![serde_json::json!({"path":"a.stl"})];
        assert!(apply_tool_overlay(&base, &bad_default_assets).unwrap_err().contains("default"));

        // M11: Reject unrecognized kind, non-end_link attach_body, and removed overlay fields
        let mut bad_kind: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        bad_kind.kind = "laser".into();
        assert!(apply_tool_overlay(&base, &bad_kind).unwrap_err().contains("kind"));
        let mut bad_attach: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        bad_attach.attach_body = "link_5".into();
        assert!(apply_tool_overlay(&base, &bad_attach).unwrap_err().contains("end_link"));
        assert!(serde_json::from_str::<ArmdGripperOverlay>(r#"{"holding_mode":"position_gap"}"#).is_err());
        assert!(serde_json::from_str::<ArmdGripperOverlay>(r#"{"max_lead_rad":0.2}"#).is_err());
        assert!(serde_json::from_str::<ArmdGraspOverlay>(r#"{"approach_height_m":0.1}"#).is_err());

        // 9. Missing kinematics or workspace with nonzero offset/points fails closed
        let pack_ok: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        let mut no_kin = base.clone();
        no_kin.kinematics = None;
        assert!(apply_tool_overlay(&no_kin, &pack_ok).unwrap_err().contains("kinematics"));
        let mut no_ws = base.clone();
        no_ws.workspace = None;
        assert!(apply_tool_overlay(&no_ws, &pack_ok).unwrap_err().contains("workspace"));

        // 10. Actuator capability checks: kind=="gripper" requires gripper_index; suction disables jaw grasp
        let no_gripper_cfg = ArmConfig::default();
        let mut pack_any_robot = pack_ok.clone();
        pack_any_robot.robots.clear();
        assert!(apply_tool_overlay(&no_gripper_cfg, &pack_any_robot).unwrap_err().contains("gripper"));
        let mut suction_pack = pack_ok;
        suction_pack.kind = "suction".into();
        // Declaring jaw policy/grasp on a non-gripper kind must fail closed.
        assert!(apply_tool_overlay(&base, &suction_pack).unwrap_err().contains("suction"));
        // Clean suction pack disables jaw gripper_policy and grasp.
        if let Some(a) = suction_pack.armd.as_mut() {
            a.gripper_policy = None;
            a.grasp = None;
        }
        let suction_applied = apply_tool_overlay(&base, &suction_pack).unwrap();
        assert!(suction_applied.gripper_policy.is_none());
        assert!(suction_applied.grasp.is_none());

        assert_eq!(
            serde_json::to_value(&base).unwrap(),
            serde_json::to_value(&before).unwrap()
        );
    }

    #[test]
    fn observation_schema_and_safety_describe_always_emit_active_tool() {
        let base = ArmConfig::rebot_dm();
        let stock_schema = serde_json::to_value(ObservationSchema::from_config(&base)).unwrap();
        assert_eq!(stock_schema["active_tool"]["schema_version"], 1);
        assert_eq!(stock_schema["active_tool"]["id"], "default");
        assert_eq!(stock_schema["active_tool"]["is_stock"], true);
        assert!(stock_schema["active_tool"]["config_digest"]
            .as_str()
            .unwrap()
            .starts_with("v1:"));

        let pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        let applied = apply_tool_overlay(&base, &pack).unwrap();
        let finray_schema = serde_json::to_value(ObservationSchema::from_config(&applied)).unwrap();
        assert_eq!(finray_schema["active_tool"]["id"], "soft_finray_gripper");
        assert_eq!(finray_schema["active_tool"]["is_stock"], false);
        assert_eq!(
            finray_schema["active_tool"]["config_digest"],
            applied.active_tool.config_digest
        );
        let sup = crate::supervisor::SupervisorInner::new(applied.clone());
        let desc = sup.safety_describe();
        assert_eq!(desc["device"]["active_tool"]["id"], "soft_finray_gripper");
        assert_eq!(desc["device"]["dynamics_qualified"], true);
    }

    #[test]
    fn profile_json_cannot_spoof_active_tool_or_dynamics_or_missing_xml_tcp() {
        // B02: Profile JSON attempting to inject active_tool or dynamics_qualified is neutralized.
        let mut raw = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
        raw["active_tool"] = serde_json::json!({
            "schema_version": 1,
            "id": "spoofed_tool",
            "is_stock": false,
            "kind": "gripper",
            "tcp_offset_m": [0.0, 0.0, 0.0],
            "config_digest": "v1:deadbeef",
            "commissioning_required": false
        });
        raw["dynamics_qualified"] = serde_json::json!(false);
        let mut parsed: ArmConfig = serde_json::from_value(raw.clone()).unwrap();
        parsed.validate().unwrap();
        assert!(parsed.active_tool.is_stock);
        assert_eq!(parsed.active_tool.id, "default");
        assert!(!parsed.active_tool.commissioning_required);
        assert!(parsed.dynamics_qualified);
        assert_eq!(
            parsed.active_tool.config_digest,
            stock_active_tool_info(&ArmConfig::rebot_dm()).config_digest
        );
        // Spoofed tcp_offset in profile JSON without tool overlay fails validation as stock.
        raw["tcp_offset"] = serde_json::json!([0.05, 0.0, 0.0]);
        let mut parsed_tcp: ArmConfig = serde_json::from_value(raw).unwrap();
        assert!(parsed_tcp.validate().is_err());

        // B03a/B03b: Non-stock tool omitting tcp_offset MUST be rejected even when tool_xml declares a tcp_site.
        let base = ArmConfig::rebot_dm();
        let mut pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        pack.tcp_offset = None;
        let err_none = apply_tool_overlay_with_xml(&base, &pack, None).unwrap_err();
        assert!(err_none.contains("tcp_offset"), "{err_none}");
        let xml_with_site = r#"<mujoco><worldbody><site name="finray_tcp" pos="0.0288 0 0"/></worldbody></mujoco>"#;
        let err_xml = apply_tool_overlay_with_xml(&base, &pack, Some(xml_with_site)).unwrap_err();
        assert!(err_xml.contains("tcp_offset"), "{err_xml}");
        assert!(err_xml.contains("soft_finray_gripper"), "{err_xml}");
    }

    #[test]
    fn resolve_tool_config_summary_matches_overlay_across_robots() {
        let finray: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        let fin_sum = resolve_tool_config_summary(&finray, None).unwrap();
        assert!(fin_sum.ok);
        assert_eq!(fin_sum.id, "soft_finray_gripper");
        assert_eq!(fin_sum.robot_id, "b601dm");
        assert!(!fin_sum.is_stock);
        assert_eq!(fin_sum.kind, "gripper");
        assert_eq!(fin_sum.tcp_offset_m, [0.0288, 0.0, 0.0]);
        assert_eq!(
            fin_sum.config_digest,
            "v1:bc4e643de8bf65ec9537c2b22ddbb1ae230e14a091a169cd4d199b955af49776"
        );
        assert_eq!(
            format!("v1:{}", crate::gravity::compute_sha256(fin_sum.canonical_json.as_bytes())),
            fin_sum.config_digest
        );

        let stock: ToolPackConfig = serde_json::from_value(serde_json::json!({
            "id": "default",
            "kind": "gripper"
        }))
        .unwrap();
        // Empty robots without explicit robot_id must fail.
        let err = resolve_tool_config_summary(&stock, None).unwrap_err();
        assert!(err.contains("--robot"), "{err}");

        for (robot, expected_digest) in [
            ("b601dm", "v1:ac4d69ae16085196687d381a4dbfa89a1940a08522dd088f2459645fd91d1064"),
            ("so101", "v1:71cfcf4dafaa5a52c2366c3ace757f714628321b761f195f4456a7ae2ba26a42"),
            ("so100", "v1:efbb9e76729b3b748959c032a789412e0ef0224d197079d16f688a32f82463d4"),
            ("b601rs", "v1:940a10d6d0dadaa2c6d2fdf7771cb8e226805c3300233dff9661fc5b8cf89636"),
        ] {
            let s = resolve_tool_config_summary(&stock, Some(robot)).unwrap();
            assert!(s.is_stock);
            assert_eq!(s.robot_id, robot);
            assert_eq!(s.tcp_offset_m, [0.0; 3]);
            assert_eq!(s.config_digest, expected_digest);
        }

        // Non-stock omitting tcp_offset is rejected.
        let mut missing_tcp = finray;
        missing_tcp.tcp_offset = None;
        let err = resolve_tool_config_summary(&missing_tcp, Some("b601dm")).unwrap_err();
        assert!(err.contains("tcp_offset"), "{err}");
    }

    #[test]
    fn regression_fixes_m2_m5_m6_m7_m1() {
        // M6: Changing stream_contact_speed_rad_s on base profile changes canonical digest.
        let base = ArmConfig::rebot_dm();
        let pack: ToolPackConfig = serde_json::from_str(SOFT_FINRAY_TOOL_JSON).unwrap();
        let d1 = apply_tool_overlay(&base, &pack).unwrap().active_tool.config_digest;
        let mut base_mod = base.clone();
        base_mod.gripper_policy.as_mut().unwrap().stream_contact_speed_rad_s = Some(0.15);
        let d2 = apply_tool_overlay(&base_mod, &pack).unwrap().active_tool.config_digest;
        assert_ne!(d1, d2, "stream_contact_speed_rad_s must affect config_digest");
        assert_ne!(
            stock_active_tool_info(&base).config_digest,
            stock_active_tool_info(&base_mod).config_digest
        );

        // M7: Dynamics qualification matches micro-unit digest quantization (0.15 == 0.1500004, 0.1500005 fails).
        let mut p_exact = pack.clone();
        p_exact.armd.as_mut().unwrap().dynamics = Some(ToolDynamics {
            mass_delta_kg: Some(0.15),
            com_moment_delta_nm: Some(0.05),
        });
        let mut p_round_down = pack.clone();
        p_round_down.armd.as_mut().unwrap().dynamics = Some(ToolDynamics {
            mass_delta_kg: Some(0.1500004),
            com_moment_delta_nm: Some(0.0500004),
        });
        let mut p_round_up = pack.clone();
        p_round_up.armd.as_mut().unwrap().dynamics = Some(ToolDynamics {
            mass_delta_kg: Some(0.1500005),
            com_moment_delta_nm: Some(0.05),
        });
        let r_exact = apply_tool_overlay(&base, &p_exact).unwrap();
        let r_down = apply_tool_overlay(&base, &p_round_down).unwrap();
        let r_up = apply_tool_overlay(&base, &p_round_up).unwrap();
        assert_eq!(r_exact.active_tool.config_digest, r_down.active_tool.config_digest);
        assert_eq!(r_exact.dynamics_qualified, r_down.dynamics_qualified);
        assert!(r_exact.dynamics_qualified);
        assert_ne!(r_exact.active_tool.config_digest, r_up.active_tool.config_digest);
        assert!(!r_up.dynamics_qualified);

        // M5: Stock profile with invalid physical_contact_end_m is rejected in validate() and from_profile_file().
        let mut bad_stock = ArmConfig::rebot_dm();
        bad_stock.physical_contact_end_m = Some([1.0, 0.0, 0.0]);
        assert!(bad_stock.validate().unwrap_err().contains("physical_contact_end_m"));
        let dir = tempfile::tempdir().unwrap();
        let bad_profile_path = dir.path().join("bad_profile.json");
        let mut raw_profile = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
        raw_profile["physical_contact_end_m"] = serde_json::json!([1.0, 0.0, 0.0]);
        std::fs::write(&bad_profile_path, serde_json::to_string(&raw_profile).unwrap()).unwrap();
        assert!(ArmConfig::from_profile_file(&bad_profile_path).unwrap_err().contains("physical_contact_end_m"));

        // M2: Shipped profiles/rebot_b601_rs.json matches baseline_arm_config_for_robot("b601rs") and loads suction.
        let rs_file = ArmConfig::from_profile_file(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/profiles/rebot_b601_rs.json"
        ))
        .unwrap();
        let rs_baseline = baseline_arm_config_for_robot("b601rs").unwrap();
        assert_eq!(rs_file.profile_id.as_deref(), Some("rebot_b601_rs"));
        assert_eq!(rs_file.profile_id, rs_baseline.profile_id);
        assert_eq!(
            rs_file.active_tool.config_digest,
            rs_baseline.active_tool.config_digest
        );
        let rs_suction: ToolPackConfig = serde_json::from_value(serde_json::json!({
            "id": "rs_suction",
            "kind": "suction",
            "robots": ["b601rs"],
            "tcp_offset": [0.0, 0.0, 0.0]
        }))
        .unwrap();
        let rs_applied = apply_tool_overlay(&rs_file, &rs_suction).unwrap();
        let rs_summary = resolve_tool_config_summary(&rs_suction, Some("b601rs")).unwrap();
        assert_eq!(rs_applied.active_tool.config_digest, rs_summary.config_digest);

        // m1: ToolPackConfig rejects unknown top-level fields (e.g. "armd_").
        let bad_top = r#"{"id":"bad","kind":"gripper","robots":["b601dm"],"tcp_offset":[0,0,0],"armd_":{"dynamics":{}}}"#;
        assert!(serde_json::from_str::<ToolPackConfig>(bad_top).is_err());
    }
}
