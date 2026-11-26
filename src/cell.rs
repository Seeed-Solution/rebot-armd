//! Multi-arm cell topology configuration loader, validator, and read-only status (`rebot-cell/v1`).
//!
//! Matches the companion-side schema and validation rules (`rebot_companion/cell.py` and
//! `docs/multi_arm_cell.md`). Cell mode is off by default with zero overhead on the RT control
//! path; configuration is parsed once at startup outside the RT loop.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use crate::model::ArmConfig;

/// Cell configuration format identifier (`rebot-cell/v1`).
pub const CELL_CONFIG_FORMAT: &str = "rebot-cell/v1";

/// Maximum allowed `max |R^T R - I|` error for a rigid 3x3 rotation block.
pub const RIGID_ORTHO_TOL: f64 = 1e-4;
/// Maximum allowed `|det(R) - 1|` error for a right-handed rotation block.
pub const RIGID_DET_TOL: f64 = 1e-4;
/// Maximum allowed per-element deviation of the 4x4 bottom row from `[0, 0, 0, 1]`.
pub const RIGID_BOTTOM_TOL: f64 = 1e-6;

/// Static joint metadata for a resolved arm profile.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArmProfileInfo {
    pub profile_id: String,
    pub robot: String,
    pub dof: usize,
    pub joint_names: Vec<String>,
    pub gripper_index: Option<usize>,
}

/// Built-in arm profile metadata matching `rebot_companion/cell.py::BUILTIN_PROFILES`.
pub fn builtin_profile(profile_id: &str) -> Option<ArmProfileInfo> {
    match profile_id {
        "rebot_b601dm" => Some(ArmProfileInfo {
            profile_id: "rebot_b601dm".into(),
            robot: "b601dm".into(),
            dof: 7,
            joint_names: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"]
                .map(String::from)
                .to_vec(),
            gripper_index: Some(6),
        }),
        "so_arm101" => Some(ArmProfileInfo {
            profile_id: "so_arm101".into(),
            robot: "so-arm101".into(),
            dof: 6,
            joint_names: ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
                .map(String::from)
                .to_vec(),
            gripper_index: Some(5),
        }),
        _ => None,
    }
}

/// Returns `true` if `profile_id` is one of the built-in profile IDs.
pub fn is_builtin_profile(profile_id: &str) -> bool {
    matches!(profile_id, "rebot_b601dm" | "so_arm101")
}

/// Resolves a built-in profile alias (case-insensitive) to its canonical profile ID.
pub fn resolve_profile_alias(raw: &str) -> Option<&'static str> {
    match raw.to_ascii_lowercase().as_str() {
        "rebot_b601dm" | "rebot-b601dm" | "b601dm" | "b601-dm" | "b601_dm" | "rebot-dm"
        | "rebot_dm" => Some("rebot_b601dm"),
        "so_arm101" | "so-arm101" | "soarm101" | "so101" | "so-101" | "so_101" => Some("so_arm101"),
        _ => None,
    }
}

/// Normalizes a robot or profile ID string matching `rebot_companion.onnx_policy.canonical_robot_id`.
pub fn canonical_robot_id(raw: &str) -> Option<String> {
    let norm = raw.trim().to_ascii_lowercase();
    if norm.is_empty() {
        return None;
    }
    if let Some(canon) = lookup_robot_alias(&norm) {
        return Some(canon.to_string());
    }
    let collapsed = norm.replace('_', "-");
    Some(lookup_robot_alias(&collapsed).map(str::to_string).unwrap_or(collapsed))
}

fn lookup_robot_alias(norm: &str) -> Option<&'static str> {
    match norm {
        "b601dm" | "b601-dm" | "b601_dm" | "rebot-dm" | "rebot_dm" | "rebot-b601dm"
        | "rebot_b601dm" | "rebot-b601-dm" | "rebot_b601_dm" => Some("b601dm"),
        "so-arm101" | "so_arm101" | "soarm101" | "so101" | "so-101" | "so_101" => Some("so-arm101"),
        _ => None,
    }
}

/// Checks `[A-Za-z0-9][A-Za-z0-9_-]{0,63}`.
pub fn is_valid_identifier(raw: &str) -> bool {
    let b = raw.as_bytes();
    !b.is_empty()
        && b.len() <= 64
        && b[0].is_ascii_alphanumeric()
        && b[1..].iter().all(|c| c.is_ascii_alphanumeric() || *c == b'_' || *c == b'-')
}

fn fmt_opt_value(raw: Option<&Value>) -> String {
    raw.map_or_else(|| "null".into(), ToString::to_string)
}

fn validate_identifier(raw: Option<&Value>, label: &str) -> Result<String, String> {
    match raw.and_then(Value::as_str) {
        Some(s) if is_valid_identifier(s) => Ok(s.to_string()),
        _ => Err(format!("{label} must match [A-Za-z0-9][A-Za-z0-9_-]{{0,63}}, got {}", fmt_opt_value(raw))),
    }
}

fn validate_non_empty_str(raw: Option<&Value>, label: &str) -> Result<String, String> {
    match raw.and_then(Value::as_str) {
        Some(s) if !s.trim().is_empty() => Ok(s.trim().to_string()),
        _ => Err(format!("{label} must be a non-empty string, got {}", fmt_opt_value(raw))),
    }
}

fn validate_finite_f64(raw: Option<&Value>, label: &str) -> Result<f64, String> {
    match raw {
        Some(Value::Number(n)) => match n.as_f64() {
            Some(v) if v.is_finite() => Ok(v),
            _ => Err(format!("{label} must be a finite number, got {n}")),
        },
        _ => Err(format!("{label} must be a finite number, got {}", fmt_opt_value(raw))),
    }
}

fn validate_exact_int(raw: Option<&Value>, label: &str) -> Result<i64, String> {
    match raw {
        Some(Value::Number(n)) => n.as_i64().ok_or_else(|| format!("{label} must be an integer, got {n}")),
        _ => Err(format!("{label} must be an integer, got {}", fmt_opt_value(raw))),
    }
}

fn validate_vec3(raw: Option<&Value>, label: &str) -> Result<[f64; 3], String> {
    let Some(arr) = raw.and_then(Value::as_array) else {
        return Err(format!("{label} must be a 3-element list of finite numbers, got {}", fmt_opt_value(raw)));
    };
    if arr.len() != 3 {
        return Err(format!("{label} must be a 3-element list of finite numbers, got length {}", arr.len()));
    }
    Ok([
        validate_finite_f64(Some(&arr[0]), &format!("{label}[0]"))?,
        validate_finite_f64(Some(&arr[1]), &format!("{label}[1]"))?,
        validate_finite_f64(Some(&arr[2]), &format!("{label}[2]"))?,
    ])
}

/// Z-Y-X rotation matrix `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`, matching
/// `rebot_companion/cell.py::_rotation_from_rpy_rad`.
fn rotation_from_rpy_rad(roll: f64, pitch: f64, yaw: f64) -> [[f64; 3]; 3] {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

fn det3(r: &[[f64; 3]; 3]) -> f64 {
    r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
}

/// Validates and normalizes a rigid SE(3) transform (`T_cell_to_base`) into a 4x4 matrix.
pub fn parse_rigid_transform(raw: &Value, label: &str) -> Result<[[f64; 4]; 4], String> {
    if let Some(obj) = raw.as_object() {
        let mut unknown: Vec<&str> = obj
            .keys()
            .map(String::as_str)
            .filter(|k| !matches!(*k, "xyz" | "rpy" | "rpy_rad" | "rpy_deg"))
            .collect();
        if !unknown.is_empty() {
            unknown.sort_unstable();
            return Err(format!("{label} contains unknown keys: {unknown:?}"));
        }
        let xyz_val = obj.get("xyz").ok_or_else(|| format!("{label} mapping requires 'xyz'"))?;
        let xyz = validate_vec3(Some(xyz_val), &format!("{label}.xyz"))?;
        let rpy_keys: Vec<&str> = ["rpy", "rpy_rad", "rpy_deg"]
            .into_iter()
            .filter(|k| obj.get(*k).is_some_and(|v| !v.is_null()))
            .collect();
        if rpy_keys.len() != 1 {
            return Err(format!("{label} mapping must specify exactly one of 'rpy', 'rpy_rad', or 'rpy_deg', got {rpy_keys:?}"));
        }
        let rpy_key = rpy_keys[0];
        let [mut r, mut p, mut y] = validate_vec3(obj.get(rpy_key), &format!("{label}.{rpy_key}"))?;
        if rpy_key == "rpy_deg" {
            r = r.to_radians();
            p = p.to_radians();
            y = y.to_radians();
        }
        let rot = rotation_from_rpy_rad(r, p, y);
        return Ok([
            [rot[0][0], rot[0][1], rot[0][2], xyz[0]],
            [rot[1][0], rot[1][1], rot[1][2], xyz[1]],
            [rot[2][0], rot[2][1], rot[2][2], xyz[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]);
    }

    if let Some(rows) = raw.as_array().filter(|r| r.len() == 4) {
        let mut mat = [[0.0f64; 4]; 4];
        for (r_idx, row_val) in rows.iter().enumerate() {
            let Some(row) = row_val.as_array().filter(|r| r.len() == 4) else {
                return Err(format!("{label} must be a 4x4 matrix, row {r_idx} has invalid shape: {row_val}"));
            };
            for (c_idx, cell) in row.iter().enumerate() {
                mat[r_idx][c_idx] = validate_finite_f64(Some(cell), &format!("{label}[{r_idx}][{c_idx}]"))?;
            }
        }
        let expected_bottom = [0.0, 0.0, 0.0, 1.0];
        if (0..4).any(|i| (mat[3][i] - expected_bottom[i]).abs() > RIGID_BOTTOM_TOL) {
            return Err(format!("{label} bottom row must be [0, 0, 0, 1], got {:?}", mat[3]));
        }
        let rot = [
            [mat[0][0], mat[0][1], mat[0][2]],
            [mat[1][0], mat[1][1], mat[1][2]],
            [mat[2][0], mat[2][1], mat[2][2]],
        ];
        let mut ortho_err = 0.0f64;
        for i in 0..3 {
            for j in 0..3 {
                let dot = rot[0][i] * rot[0][j] + rot[1][i] * rot[1][j] + rot[2][i] * rot[2][j];
                let err = (dot - if i == j { 1.0 } else { 0.0 }).abs();
                if !err.is_finite() { ortho_err = f64::NAN; } else if err > ortho_err { ortho_err = err; }
            }
        }
        if !ortho_err.is_finite() || ortho_err > RIGID_ORTHO_TOL {
            return Err(format!("{label} rotation block is not orthonormal (max |R^T R - I| = {ortho_err:.6} > {RIGID_ORTHO_TOL})"));
        }
        let det = det3(&rot);
        if !det.is_finite() || (det - 1.0).abs() > RIGID_DET_TOL {
            return Err(format!("{label} rotation block must have det(R) = +1 (proper rigid rotation), got det = {det:.6}"));
        }
        return Ok(mat);
    }

    Err(format!("{label} must be a 4x4 rigid matrix or an {{xyz, rpy_rad|rpy|rpy_deg}} mapping, got {raw}"))
}

#[cfg(not(target_arch = "wasm32"))]
fn load_profile_from_json(path: &Path) -> Result<ArmProfileInfo, String> {
    if !path.is_file() {
        return Err(format!("Profile JSON file not found: {}", path.display()));
    }
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("Failed to read profile JSON {}: {e}", path.display()))?;
    let data: Value = serde_json::from_str(&text)
        .map_err(|e| format!("Invalid profile JSON {}: {e}", path.display()))?;
    let Some(obj) = data.as_object() else {
        return Err(format!("Profile JSON {} must contain an object", path.display()));
    };

    let stem_fallback = path.file_stem().and_then(|s| s.to_str()).map(|s| Value::String(s.into()));
    let raw_pid = if obj.contains_key("profile_id") { obj.get("profile_id") } else { stem_fallback.as_ref() };
    let profile_id = validate_non_empty_str(raw_pid, &format!("{}.profile_id", path.display()))?;

    let Some(raw_names) = obj.get("joint_names").and_then(Value::as_array).filter(|a| !a.is_empty()) else {
        return Err(format!("Profile JSON {} must contain a non-empty 'joint_names' list", path.display()));
    };
    let mut joint_names = Vec::with_capacity(raw_names.len());
    let mut seen = BTreeSet::new();
    for (idx, item) in raw_names.iter().enumerate() {
        let name = validate_non_empty_str(Some(item), &format!("{}.joint_names[{idx}]", path.display()))?;
        if !seen.insert(name.clone()) {
            return Err(format!("Profile JSON {} has duplicate joint_names: {joint_names:?}", path.display()));
        }
        joint_names.push(name);
    }

    let dof = if let Some(raw_dof) = obj.get("dof") {
        let val = validate_exact_int(Some(raw_dof), &format!("{}.dof", path.display()))?;
        if val < 0 || val as usize != joint_names.len() {
            return Err(format!("Profile JSON {} dof ({raw_dof}) does not match len(joint_names) ({})", path.display(), joint_names.len()));
        }
        val as usize
    } else {
        joint_names.len()
    };

    let gripper_index = match obj.get("gripper_joint") {
        Some(v) if !v.is_null() => {
            let g = validate_exact_int(Some(v), &format!("{}.gripper_joint", path.display()))?;
            if g < 0 || (g as usize) >= dof {
                return Err(format!("Profile JSON {} has invalid gripper_joint: {v}", path.display()));
            }
            Some(g as usize)
        }
        _ => joint_names.iter().position(|n| n == "gripper"),
    };

    let robot = canonical_robot_id(&profile_id).unwrap_or_else(|| profile_id.clone());
    Ok(ArmProfileInfo { profile_id, robot, dof, joint_names, gripper_index })
}

/// Resolves an arm profile name (built-in alias table) or JSON file path.
pub fn resolve_arm_profile(profile: &str, base_dir: Option<&Path>) -> Result<(ArmProfileInfo, bool), String> {
    let clean = profile.trim();
    if clean.is_empty() {
        return Err("arm.profile must be a non-empty string".into());
    }
    if let Some(canon_id) = resolve_profile_alias(clean) {
        return Ok((builtin_profile(canon_id).expect("alias maps to builtin profile"), true));
    }

    #[cfg(not(target_arch = "wasm32"))]
    {
        let candidate = Path::new(clean);
        if !candidate.is_absolute() {
            if let Some(rel) = base_dir.map(|d| d.join(candidate)).filter(|p| p.is_file()) {
                return load_profile_from_json(&rel).map(|info| (info, false));
            }
        }
        if candidate.is_file() || clean.ends_with(".json") {
            let target = match base_dir {
                Some(dir) if !candidate.is_absolute() => dir.join(candidate),
                _ => candidate.to_path_buf(),
            };
            return load_profile_from_json(&target).map(|info| (info, false));
        }
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = base_dir;
        if clean.ends_with(".json") {
            return Err(format!("Profile JSON file loading ({clean:?}) is unavailable on wasm32"));
        }
    }

    Err(format!("Unknown arm profile {profile:?}; expected one of (rebot_b601dm, so_arm101) or a path to a profile .json file"))
}

/// Network host participating in a cell (PC coordinator or arm host).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HostConfig {
    #[serde(default, skip_serializing)]
    pub id: String,
    pub address: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub port: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub role: Option<String>,
}

impl HostConfig {
    fn to_canonical_value(&self) -> Value {
        let mut obj = Map::new();
        obj.insert("address".into(), Value::String(self.address.clone()));
        if let Some(port) = self.port { obj.insert("port".into(), Value::from(port)); }
        if let Some(role) = &self.role { obj.insert("role".into(), Value::String(role.clone())); }
        Value::Object(obj)
    }
}

/// Single arm member of a multi-arm cell.
#[derive(Debug, Clone, PartialEq)]
pub struct ArmSpec {
    pub id: String,
    pub arm_type: String,
    pub profile: String,
    pub profile_info: ArmProfileInfo,
    pub host: String,
    pub device: String,
    pub camera: Option<String>,
    pub t_cell_to_base: [[f64; 4]; 4],
}

impl ArmSpec {
    pub fn dof(&self) -> usize { self.profile_info.dof }
    pub fn joint_names(&self) -> &[String] { &self.profile_info.joint_names }
    pub fn gripper_index(&self) -> Option<usize> { self.profile_info.gripper_index }

    fn to_canonical_value(&self) -> Value {
        let mut obj = Map::new();
        let mat = Value::Array(
            self.t_cell_to_base
                .iter()
                .map(|row| Value::Array(row.iter().copied().map(Value::from).collect()))
                .collect(),
        );
        obj.insert("T_cell_to_base".into(), mat);
        obj.insert("camera".into(), self.camera.as_ref().map_or(Value::Null, |c| Value::String(c.clone())));
        obj.insert("device".into(), Value::String(self.device.clone()));
        obj.insert("host".into(), Value::String(self.host.clone()));
        obj.insert("id".into(), Value::String(self.id.clone()));
        obj.insert("profile".into(), Value::String(self.profile.clone()));
        obj.insert("type".into(), Value::String(self.arm_type.clone()));
        Value::Object(obj)
    }
}

/// Peer communication link between two cell hosts.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LinkSpec {
    pub from: String,
    pub to: String,
    pub endpoint: String,
    pub auth_ref: String,
}

impl LinkSpec {
    fn to_canonical_value(&self) -> Value {
        let mut obj = Map::new();
        obj.insert("auth_ref".into(), Value::String(self.auth_ref.clone()));
        obj.insert("endpoint".into(), Value::String(self.endpoint.clone()));
        obj.insert("from".into(), Value::String(self.from.clone()));
        obj.insert("to".into(), Value::String(self.to.clone()));
        Value::Object(obj)
    }
}

/// Validated multi-arm cell configuration (`rebot-cell/v1`).
#[derive(Debug, Clone, PartialEq)]
pub struct CellConfig {
    pub format: String,
    pub cell_id: String,
    pub coordinator: String,
    pub hosts: BTreeMap<String, HostConfig>,
    pub arms: Vec<ArmSpec>,
    pub links: Vec<LinkSpec>,
}

impl CellConfig {
    pub fn arm_ids(&self) -> Vec<String> { self.arms.iter().map(|a| a.id.clone()).collect() }
    pub fn total_dof(&self) -> usize { self.arms.iter().map(ArmSpec::dof).sum() }
    pub fn get_arm(&self, arm_id: &str) -> Option<&ArmSpec> { self.arms.iter().find(|a| a.id == arm_id) }
    pub fn arm_for_host(&self, host_id: &str) -> Option<&ArmSpec> { self.arms.iter().find(|a| a.host == host_id) }

    /// Canonical normalized JSON matching `rebot_companion.cell.CellConfig.to_dict()`.
    pub fn to_canonical_value(&self) -> Value {
        let mut root = Map::new();
        root.insert("arms".into(), Value::Array(self.arms.iter().map(ArmSpec::to_canonical_value).collect()));
        root.insert("cell_id".into(), Value::String(self.cell_id.clone()));
        root.insert("coordinator".into(), Value::String(self.coordinator.clone()));
        root.insert("format".into(), Value::String(self.format.clone()));
        let hosts_obj = self.hosts.iter().map(|(k, v)| (k.clone(), v.to_canonical_value())).collect();
        root.insert("hosts".into(), Value::Object(hosts_obj));
        root.insert("links".into(), Value::Array(self.links.iter().map(LinkSpec::to_canonical_value).collect()));
        Value::Object(root)
    }

    /// Deterministic SHA-256 hex digest byte-identical to `rebot_companion.cell.CellConfig.digest()`
    /// (Python `json.dumps(to_dict(), sort_keys=True, separators=(",", ":"))`, including ASCII
    /// `\uXXXX` escaping and Python float exponent formatting).
    pub fn digest(&self) -> String {
        let mut canonical = String::new();
        write_python_canonical_json(&self.to_canonical_value(), &mut canonical);
        crate::gravity::compute_sha256(canonical.as_bytes())
    }

    /// Parse and validate a `rebot-cell/v1` JSON value.
    pub fn from_value(data: &Value, base_dir: Option<&Path>) -> Result<Self, String> {
        let obj = data.as_object().ok_or_else(|| "cell config must be a JSON object".to_string())?;

        let format = match obj.get("format") {
            None => CELL_CONFIG_FORMAT.to_string(),
            Some(Value::String(s)) if s == CELL_CONFIG_FORMAT => s.clone(),
            Some(other) => return Err(format!("Unsupported cell config format {other}, expected \"{CELL_CONFIG_FORMAT}\"")),
        };
        let cell_id = match obj.get("cell_id") {
            None => "default_cell".to_string(),
            Some(v) => validate_identifier(Some(v), "cell_id")?,
        };

        let Some(raw_hosts) = obj.get("hosts").and_then(Value::as_object).filter(|m| !m.is_empty()) else {
            return Err("hosts must be a non-empty object mapping host_id -> host config".into());
        };
        let mut hosts = BTreeMap::new();
        for (host_id, raw_cfg) in raw_hosts {
            if !is_valid_identifier(host_id) {
                return Err(format!("hosts key must match [A-Za-z0-9][A-Za-z0-9_-]{{0,63}}, got {host_id:?}"));
            }
            let hobj = raw_cfg.as_object().ok_or_else(|| format!("hosts[{host_id:?}] must be an object"))?;
            let address = validate_non_empty_str(hobj.get("address"), &format!("hosts[{host_id:?}].address"))?;
            let port = match hobj.get("port") {
                None | Some(Value::Null) => None,
                Some(v) => {
                    let p = validate_exact_int(Some(v), &format!("hosts[{host_id:?}].port"))?;
                    if !(1..=65535).contains(&p) {
                        return Err(format!("hosts[{host_id:?}].port must be an integer in 1..65535, got {v}"));
                    }
                    Some(p as u16)
                }
            };
            let role = match hobj.get("role") {
                None | Some(Value::Null) => None,
                Some(v) => Some(validate_non_empty_str(Some(v), &format!("hosts[{host_id:?}].role"))?),
            };
            hosts.insert(host_id.clone(), HostConfig { id: host_id.clone(), address, port, role });
        }
        let known_hosts = || hosts.keys().map(String::as_str).collect::<Vec<_>>();

        let coordinator = validate_identifier(obj.get("coordinator"), "coordinator")?;
        if !hosts.contains_key(&coordinator) {
            return Err(format!("coordinator references unknown host {coordinator:?}; known hosts: {:?}", known_hosts()));
        }

        let Some(raw_arms) = obj.get("arms").and_then(Value::as_array).filter(|a| !a.is_empty()) else {
            return Err("arms must be a non-empty list with at least 1 arm".into());
        };
        let mut arms = Vec::with_capacity(raw_arms.len());
        let mut seen_arm_ids = BTreeSet::new();
        let mut host_to_arm: BTreeMap<String, String> = BTreeMap::new();
        let mut seen_host_devices: BTreeMap<(String, String), String> = BTreeMap::new();

        for (idx, raw_arm) in raw_arms.iter().enumerate() {
            let prefix = format!("arms[{idx}]");
            let aobj = raw_arm.as_object().ok_or_else(|| format!("{prefix} must be an object"))?;
            let arm_id = validate_identifier(aobj.get("id"), &format!("{prefix}.id"))?;
            if !seen_arm_ids.insert(arm_id.clone()) {
                return Err(format!("Duplicate arm id {arm_id:?} in arms"));
            }

            let raw_type = validate_non_empty_str(aobj.get("type"), &format!("{prefix}.type"))?;
            let canon_type = canonical_robot_id(&raw_type).unwrap_or_else(|| raw_type.clone());
            let raw_profile = validate_non_empty_str(aobj.get("profile"), &format!("{prefix}.profile"))?;
            let (profile_info, matched_alias) = resolve_arm_profile(&raw_profile, base_dir)?;
            if is_builtin_profile(&profile_info.profile_id) && canon_type != profile_info.robot {
                return Err(format!("{prefix} ({arm_id:?}) type {raw_type:?} (canonical {canon_type:?}) conflicts with profile {raw_profile:?} (robot {:?})", profile_info.robot));
            }

            let host_id = validate_identifier(aobj.get("host"), &format!("{prefix}.host"))?;
            if !hosts.contains_key(&host_id) {
                return Err(format!("{prefix} ({arm_id:?}) references unknown host {host_id:?}; known hosts: {:?}", known_hosts()));
            }
            let device = validate_non_empty_str(aobj.get("device"), &format!("{prefix}.device"))?;
            let host_dev = (host_id.clone(), device.clone());
            if let Some(prev_id) = seen_host_devices.get(&host_dev) {
                return Err(format!("Duplicate device binding {device:?} on host {host_id:?} (arms {prev_id:?} and {arm_id:?}); at most one arm per host is supported"));
            }
            if let Some(prev_id) = host_to_arm.get(&host_id) {
                return Err(format!("Multiple arms on one host are not supported (one arm per host): host {host_id:?} is assigned to both {prev_id:?} and {arm_id:?}"));
            }

            let camera = match aobj.get("camera") {
                None | Some(Value::Null) => None,
                Some(v) => Some(validate_non_empty_str(Some(v), &format!("{prefix}.camera"))?),
            };
            let t_val = aobj.get("T_cell_to_base").filter(|v| !v.is_null())
                .ok_or_else(|| format!("{prefix} ({arm_id:?}) is missing required 'T_cell_to_base'"))?;
            let t_cell_to_base = parse_rigid_transform(t_val, &format!("{prefix}.T_cell_to_base"))?;

            let profile = if matched_alias { profile_info.profile_id.clone() } else { raw_profile };
            seen_host_devices.insert(host_dev, arm_id.clone());
            host_to_arm.insert(host_id.clone(), arm_id.clone());
            arms.push(ArmSpec {
                id: arm_id, arm_type: canon_type, profile, profile_info,
                host: host_id, device, camera, t_cell_to_base,
            });
        }

        let raw_links = match obj.get("links") {
            None => &[][..],
            Some(Value::Array(arr)) => arr.as_slice(),
            Some(other) => return Err(format!("links must be a list, got {other}")),
        };
        let mut links = Vec::with_capacity(raw_links.len());
        let mut seen_link_pairs = BTreeSet::new();
        let mut connected_pairs = BTreeSet::new();

        for (idx, raw_link) in raw_links.iter().enumerate() {
            let prefix = format!("links[{idx}]");
            let lobj = raw_link.as_object().ok_or_else(|| format!("{prefix} must be an object"))?;
            let from = validate_identifier(lobj.get("from"), &format!("{prefix}.from"))?;
            let to = validate_identifier(lobj.get("to"), &format!("{prefix}.to"))?;
            if !hosts.contains_key(&from) {
                return Err(format!("{prefix}.from references unknown host {from:?}; known hosts: {:?}", known_hosts()));
            }
            if !hosts.contains_key(&to) {
                return Err(format!("{prefix}.to references unknown host {to:?}; known hosts: {:?}", known_hosts()));
            }
            if from == to {
                return Err(format!("{prefix} cannot link host {from:?} to itself"));
            }
            if !seen_link_pairs.insert((from.clone(), to.clone())) {
                return Err(format!("Duplicate link from {from:?} to {to:?}"));
            }
            connected_pairs.insert(undirected_pair(&from, &to));
            let endpoint = validate_non_empty_str(lobj.get("endpoint"), &format!("{prefix}.endpoint"))?;
            let auth_ref = validate_non_empty_str(lobj.get("auth_ref"), &format!("{prefix}.auth_ref"))?;
            links.push(LinkSpec { from, to, endpoint, auth_ref });
        }

        for arm in &arms {
            if arm.host != coordinator && !connected_pairs.contains(&undirected_pair(&coordinator, &arm.host)) {
                return Err(format!("Missing link between coordinator {coordinator:?} and arm {:?} host {:?}", arm.id, arm.host));
            }
        }

        Ok(Self { format, cell_id, coordinator, hosts, arms, links })
    }

    /// Parse and validate from a JSON string.
    pub fn from_json_str(text: &str, base_dir: Option<&Path>) -> Result<Self, String> {
        let value: Value = serde_json::from_str(text).map_err(|e| format!("Invalid cell JSON: {e}"))?;
        Self::from_value(&value, base_dir)
    }

    /// Load and validate from a `cell.json` file on disk.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn from_json_file(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        if !path.is_file() {
            return Err(format!("Cell config file not found: {}", path.display()));
        }
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read cell config {}: {e}", path.display()))?;
        let base_dir = path.canonicalize().ok()
            .and_then(|p| p.parent().map(Path::to_path_buf))
            .or_else(|| path.parent().map(Path::to_path_buf));
        Self::from_json_str(&text, base_dir.as_deref())
    }
}

fn undirected_pair(a: &str, b: &str) -> (String, String) {
    if a < b { (a.into(), b.into()) } else { (b.into(), a.into()) }
}

fn write_python_json_string(s: &str, out: &mut String) {
    use std::fmt::Write;
    out.push('"');
    for ch in s.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\u{0008}' => out.push_str("\\b"),
            '\u{000C}' => out.push_str("\\f"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c if c.is_ascii() => out.push(c),
            c => {
                let mut units = [0u16; 2];
                for u in c.encode_utf16(&mut units) {
                    let _ = write!(out, "\\u{:04x}", u);
                }
            }
        }
    }
    out.push('"');
}

fn write_python_json_f64(val: f64, out: &mut String) {
    use std::fmt::Write;
    if val == 0.0 {
        if val.is_sign_negative() { out.push_str("-0.0"); } else { out.push_str("0.0"); }
        return;
    }
    let raw = serde_json::to_string(&val).expect("finite f64");
    let (neg, rest) = match raw.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, raw.as_str()),
    };
    let (mantissa, exp_part) = match rest.find(['e', 'E']) {
        Some(idx) => (&rest[..idx], rest[idx + 1..].parse::<i32>().unwrap_or(0)),
        None => (rest, 0),
    };
    let mut digits = String::with_capacity(mantissa.len());
    let mut frac_len = 0i32;
    let mut seen_dot = false;
    for ch in mantissa.chars() {
        if ch == '.' {
            seen_dot = true;
        } else {
            digits.push(ch);
            if seen_dot { frac_len += 1; }
        }
    }
    let trimmed_lead = digits.trim_start_matches('0');
    let leading_zeros = (digits.len() - trimmed_lead.len()) as i32;
    let sig = trimmed_lead.trim_end_matches('0');
    let exp10 = exp_part + (digits.len() as i32 - frac_len - leading_zeros - 1);

    if neg { out.push('-'); }
    if (-4..16).contains(&exp10) {
        if exp10 >= 0 {
            let int_digits = (exp10 + 1) as usize;
            if sig.len() <= int_digits {
                out.push_str(sig);
                for _ in 0..(int_digits - sig.len()) { out.push('0'); }
                out.push_str(".0");
            } else {
                out.push_str(&sig[..int_digits]);
                out.push('.');
                out.push_str(&sig[int_digits..]);
            }
        } else {
            out.push_str("0.");
            for _ in 0..((-exp10 - 1) as usize) { out.push('0'); }
            out.push_str(sig);
        }
    } else {
        out.push_str(&sig[..1]);
        if sig.len() > 1 {
            out.push('.');
            out.push_str(&sig[1..]);
        }
        let sign = if exp10 < 0 { '-' } else { '+' };
        let _ = write!(out, "e{sign}{:02}", exp10.unsigned_abs());
    }
}

fn write_python_canonical_json(value: &Value, out: &mut String) {
    match value {
        Value::Null => out.push_str("null"),
        Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                out.push_str(&i.to_string());
            } else if let Some(u) = n.as_u64() {
                out.push_str(&u.to_string());
            } else if let Some(f) = n.as_f64() {
                write_python_json_f64(f, out);
            }
        }
        Value::String(s) => write_python_json_string(s, out),
        Value::Array(arr) => {
            out.push('[');
            for (idx, item) in arr.iter().enumerate() {
                if idx > 0 { out.push(','); }
                write_python_canonical_json(item, out);
            }
            out.push(']');
        }
        Value::Object(map) => {
            out.push('{');
            for (idx, (k, v)) in map.iter().enumerate() {
                if idx > 0 { out.push(','); }
                write_python_json_string(k, out);
                out.push(':');
                write_python_canonical_json(v, out);
            }
            out.push('}');
        }
    }
}

fn address_matches_local(host_addr: &str, local_ips: &[String], hostname: Option<&str>) -> bool {
    if hostname.is_some_and(|h| h == host_addr) {
        return true;
    }
    if let Ok(parsed_host) = host_addr.parse::<std::net::IpAddr>() {
        return local_ips
            .iter()
            .any(|ip| ip.parse::<std::net::IpAddr>().is_ok_and(|p| p == parsed_host));
    }
    local_ips.iter().any(|ip| ip == host_addr)
}

/// Pure host-identification resolver (explicit host override, then local hostname, then local IP
/// addresses + port). Fails closed when zero or multiple hosts match.
pub fn resolve_local_host(
    cell: &CellConfig,
    explicit_host: Option<&str>,
    local_hostname: Option<&str>,
    local_ips: &[String],
    local_port: Option<u16>,
) -> Result<String, String> {
    let known: Vec<&str> = cell.hosts.keys().map(String::as_str).collect();
    if let Some(raw) = explicit_host {
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            return Err("cell host identifier must not be empty".into());
        }
        if !cell.hosts.contains_key(trimmed) {
            return Err(format!("cell host {trimmed:?} is not defined in cell.hosts; known hosts: {known:?}"));
        }
        return Ok(trimmed.to_string());
    }

    let hostname_trimmed = local_hostname.map(str::trim).filter(|s| !s.is_empty());
    if let Some(h) = hostname_trimmed {
        if cell.hosts.contains_key(h) {
            return Ok(h.to_string());
        }
    }

    let mut exact_port = Vec::new();
    let mut wildcard_port = Vec::new();
    for (host_id, host) in &cell.hosts {
        if !address_matches_local(&host.address, local_ips, hostname_trimmed) { continue; }
        match (host.port, local_port) {
            (Some(hp), Some(lp)) if hp == lp => exact_port.push(host_id.as_str()),
            (Some(_), Some(_)) => {}
            (None, _) | (Some(_), None) => wildcard_port.push(host_id.as_str()),
        }
    }

    match (exact_port.as_slice(), wildcard_port.as_slice()) {
        ([only], _) => Ok((*only).to_string()),
        (many, _) if many.len() > 1 => Err(format!("Ambiguous local cell host match ({many:?}); pass --cell-host <id> (or ARMD_CELL_HOST)")),
        ([], [only]) => Ok((*only).to_string()),
        ([], many) if many.len() > 1 => Err(format!("Ambiguous local cell host match ({many:?}); pass --cell-host <id> (or ARMD_CELL_HOST)")),
        _ => Err(format!("Could not identify this host in cell.hosts ({known:?}); pass --cell-host <id> (or ARMD_CELL_HOST)")),
    }
}

/// Validates that a local `ArmConfig`, hardware follower model, and bus/serial device match `arm`.
pub fn validate_local_arm_config(
    arm: &ArmSpec,
    config: &ArmConfig,
    follower: Option<&str>,
    resolved_hw_device: Option<&str>,
) -> Result<(), String> {
    if config.dof != arm.dof() {
        return Err(format!("Cell arm {:?} (host {:?}, profile {:?}) requires DOF {}, but loaded ArmConfig has DOF {}", arm.id, arm.host, arm.profile, arm.dof(), config.dof));
    }
    if config.joint_names != arm.joint_names() {
        return Err(format!("Cell arm {:?} joint_names {:?} do not match loaded ArmConfig joint_names {:?}", arm.id, arm.joint_names(), config.joint_names));
    }
    if config.gripper_index() != arm.gripper_index() {
        return Err(format!("Cell arm {:?} gripper_index {:?} does not match loaded ArmConfig gripper_index {:?}", arm.id, arm.gripper_index(), config.gripper_index()));
    }
    if is_builtin_profile(&arm.profile_info.profile_id) {
        if let Some(pid) = config.profile_id.as_deref() {
            if canonical_robot_id(pid).as_deref() != Some(arm.profile_info.robot.as_str()) {
                return Err(format!("Cell arm {:?} expects robot {:?} (profile {:?}), but loaded ArmConfig has profile_id {:?}", arm.id, arm.profile_info.robot, arm.profile, pid));
            }
        }
    }
    if let Some(hw_follower) = follower.filter(|f| !matches!(*f, "mock" | "sim")) {
        let follower_robot = canonical_robot_id(hw_follower);
        let arm_robot = canonical_robot_id(&arm.arm_type);
        if follower_robot != arm_robot {
            return Err(format!(
                "Cell arm {:?} (host {:?}) declares type {:?}, which does not match hardware follower {:?}",
                arm.id, arm.host, arm.arm_type, hw_follower
            ));
        }
    }
    if let Some(hw_dev) = resolved_hw_device {
        if hw_dev != arm.device {
            return Err(format!("Cell arm {:?} (host {:?}) expects device {:?}, but daemon is configured for {:?}", arm.id, arm.host, arm.device, hw_dev));
        }
    }
    Ok(())
}

/// Immutable startup-resolved cell membership context (lives off the RT control thread).
#[derive(Debug, Clone, PartialEq)]
pub struct CellContext {
    pub config: CellConfig,
    pub config_digest: String,
    pub local_host_id: String,
    pub local_arm_id: Option<String>,
    pub is_coordinator: bool,
}

impl CellContext {
    pub fn resolve(
        config: CellConfig,
        explicit_host: Option<&str>,
        local_hostname: Option<&str>,
        local_ips: &[String],
        local_port: Option<u16>,
        arm_config: &ArmConfig,
        follower: Option<&str>,
        resolved_hw_device: Option<&str>,
    ) -> Result<Self, String> {
        let local_host_id = resolve_local_host(&config, explicit_host, local_hostname, local_ips, local_port)?;
        let is_coordinator = config.coordinator == local_host_id;
        let local_arm = config.arm_for_host(&local_host_id);
        if local_arm.is_none() && !is_coordinator {
            return Err(format!("Host {local_host_id:?} is neither an arm host nor the coordinator ({:?}) in cell {:?}", config.coordinator, config.cell_id));
        }
        if let Some(arm) = local_arm {
            validate_local_arm_config(arm, arm_config, follower, resolved_hw_device)?;
        } else if let Some(hw_follower) = follower.filter(|f| !matches!(*f, "mock" | "sim")) {
            return Err(format!(
                "Coordinator-only host {local_host_id:?} has no arm entry in cell {:?}; refusing to mount hardware follower {hw_follower:?}",
                config.cell_id
            ));
        }
        let local_arm_id = local_arm.map(|a| a.id.clone());
        let config_digest = config.digest();
        Ok(Self { config, config_digest, local_host_id, local_arm_id, is_coordinator })
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub fn resolve_local(
        config: CellConfig,
        explicit_host: Option<&str>,
        local_port: Option<u16>,
        arm_config: &ArmConfig,
        follower: Option<&str>,
        resolved_hw_device: Option<&str>,
    ) -> Result<Self, String> {
        let hostname = probe_local_hostname();
        let ips = probe_local_ips();
        Self::resolve(config, explicit_host, hostname.as_deref(), &ips, local_port, arm_config, follower, resolved_hw_device)
    }

    pub fn local_arm(&self) -> Option<&ArmSpec> {
        self.local_arm_id.as_deref().and_then(|id| self.config.get_arm(id))
    }

    pub fn local_role(&self) -> &'static str {
        match (self.is_coordinator, self.local_arm_id.is_some()) {
            (true, true) => "coordinator_arm_host",
            (true, false) => "coordinator",
            (false, true) => "arm_host",
            (false, false) => "unassigned",
        }
    }

    pub fn capabilities(&self) -> CellCapabilities {
        CellCapabilities {
            format: self.config.format.clone(),
            enabled: true,
            cell_id: self.config.cell_id.clone(),
            config_digest: self.config_digest.clone(),
            host_id: self.local_host_id.clone(),
            arm_id: self.local_arm_id.clone(),
            coordinator: self.config.coordinator.clone(),
            is_coordinator: self.is_coordinator,
            arm_ids: self.config.arm_ids(),
            total_dof: self.config.total_dof(),
            peer_networking: false,
        }
    }

    pub fn status_response(&self) -> CellStatusResponse {
        let arms = self.config.arms.iter()
            .map(|arm| CellArmStatus::from_spec(arm, self.local_arm_id.as_deref() == Some(arm.id.as_str())))
            .collect();
        CellStatusResponse {
            enabled: true,
            format: Some(self.config.format.clone()),
            cell_id: Some(self.config.cell_id.clone()),
            config_digest: Some(self.config_digest.clone()),
            coordinator: Some(self.config.coordinator.clone()),
            local_host: Some(self.local_host_id.clone()),
            local_role: Some(self.local_role().into()),
            is_coordinator: Some(self.is_coordinator),
            local_arm_id: self.local_arm_id.clone(),
            local_arm: self.local_arm().map(|arm| CellArmStatus::from_spec(arm, true)),
            total_dof: Some(self.config.total_dof()),
            peer_networking: Some(CellPeerNetworkingStatus { enabled: false, state: "disabled".into() }),
            hosts: Some(self.config.hosts.clone()),
            arms: Some(arms),
            links: Some(self.config.links.clone()),
        }
    }
}

/// `capabilities.cell` published inside `GET /api/v1/observation/schema` when a cell config is
/// active; omitted when cell mode is off (default).
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct CellCapabilities {
    pub format: String,
    pub enabled: bool,
    pub cell_id: String,
    pub config_digest: String,
    pub host_id: String,
    pub arm_id: Option<String>,
    pub coordinator: String,
    pub is_coordinator: bool,
    pub arm_ids: Vec<String>,
    pub total_dof: usize,
    pub peer_networking: bool,
}

/// Read-only arm entry published by `GET /api/v1/cell/status`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CellArmStatus {
    pub id: String,
    #[serde(rename = "type")]
    pub arm_type: String,
    pub profile: String,
    pub dof: usize,
    pub joint_names: Vec<String>,
    pub gripper_index: Option<usize>,
    pub host: String,
    pub device: String,
    pub camera: Option<String>,
    #[serde(rename = "T_cell_to_base")]
    pub t_cell_to_base: [[f64; 4]; 4],
    pub is_local: bool,
}

impl CellArmStatus {
    fn from_spec(spec: &ArmSpec, is_local: bool) -> Self {
        Self {
            id: spec.id.clone(),
            arm_type: spec.arm_type.clone(),
            profile: spec.profile.clone(),
            dof: spec.dof(),
            joint_names: spec.joint_names().to_vec(),
            gripper_index: spec.gripper_index(),
            host: spec.host.clone(),
            device: spec.device.clone(),
            camera: spec.camera.clone(),
            t_cell_to_base: spec.t_cell_to_base,
            is_local,
        }
    }
}

/// Peer networking readiness summary inside `GET /api/v1/cell/status`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CellPeerNetworkingStatus {
    pub enabled: bool,
    pub state: String,
}

/// Response body of `GET /api/v1/cell/status` (`{"enabled": false}` when cell mode is off).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct CellStatusResponse {
    pub enabled: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub format: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cell_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub config_digest: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub coordinator: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub local_host: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub local_role: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub is_coordinator: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub local_arm_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub local_arm: Option<CellArmStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub total_dof: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub peer_networking: Option<CellPeerNetworkingStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hosts: Option<BTreeMap<String, HostConfig>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arms: Option<Vec<CellArmStatus>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub links: Option<Vec<LinkSpec>>,
}

impl CellStatusResponse {
    pub fn disabled() -> Self {
        Self::default()
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn probe_local_hostname() -> Option<String> {
    #[cfg(unix)]
    {
        let mut buf = [0u8; 256];
        // SAFETY: `buf` is a valid writable buffer of length 256 for POSIX `gethostname`.
        if unsafe { libc::gethostname(buf.as_mut_ptr().cast::<libc::c_char>(), buf.len()) } == 0 {
            let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
            if let Ok(s) = std::str::from_utf8(&buf[..end]) {
                if !s.trim().is_empty() { return Some(s.trim().to_string()); }
            }
        }
    }
    std::fs::read_to_string("/etc/hostname").ok().map(|s| s.trim().to_string()).filter(|s| !s.is_empty())
}

#[cfg(not(target_arch = "wasm32"))]
fn probe_local_ips() -> Vec<String> {
    let mut ips = BTreeSet::from(["127.0.0.1".into(), "::1".into(), "localhost".into()]);
    #[cfg(unix)]
    {
        let mut ifap: *mut libc::ifaddrs = std::ptr::null_mut();
        // SAFETY: `getifaddrs` allocates a linked list freed unconditionally with `freeifaddrs`.
        if unsafe { libc::getifaddrs(&mut ifap) } == 0 {
            let mut cur = ifap;
            while !cur.is_null() {
                // SAFETY: `cur` is a non-null node from `getifaddrs`.
                let ifa = unsafe { &*cur };
                if !ifa.ifa_addr.is_null() {
                    // SAFETY: `ifa_addr` is non-null; check `sa_family` before casting.
                    let family = unsafe { (*ifa.ifa_addr).sa_family } as i32;
                    if family == libc::AF_INET {
                        // SAFETY: `sa_family == AF_INET` guarantees `sockaddr_in` layout.
                        let sin = unsafe { &*(ifa.ifa_addr.cast::<libc::sockaddr_in>()) };
                        ips.insert(std::net::Ipv4Addr::from(u32::from_be(sin.sin_addr.s_addr)).to_string());
                    } else if family == libc::AF_INET6 {
                        // SAFETY: `sa_family == AF_INET6` guarantees `sockaddr_in6` layout.
                        let sin6 = unsafe { &*(ifa.ifa_addr.cast::<libc::sockaddr_in6>()) };
                        ips.insert(std::net::Ipv6Addr::from(sin6.sin6_addr.s6_addr).to_string());
                    }
                }
                cur = ifa.ifa_next;
            }
            // SAFETY: frees the list returned by `getifaddrs`.
            unsafe { libc::freeifaddrs(ifap) };
        }
    }
    ips.into_iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn two_so101_json() -> Value {
        json!({
            "format": "rebot-cell/v1", "cell_id": "two_so101", "coordinator": "pc_coord",
            "hosts": {
                "pc_coord": { "address": "192.168.42.10", "role": "pc" },
                "recamera_left": { "address": "192.168.42.11", "port": 8765, "role": "arm_host" },
                "recamera_right": { "address": "192.168.42.12", "port": 8765, "role": "arm_host" }
            },
            "arms": [
                { "id": "left", "type": "so-arm101", "profile": "so_arm101", "host": "recamera_left",
                  "device": "/dev/ttySC0", "camera": "left_wrist",
                  "T_cell_to_base": { "xyz": [0.0, -0.22, 0.0], "rpy_rad": [0.0, 0.0, 0.0] } },
                { "id": "right", "type": "so-arm101", "profile": "so_arm101", "host": "recamera_right",
                  "device": "/dev/ttySC0", "camera": "right_wrist",
                  "T_cell_to_base": { "xyz": [0.0, 0.22, 0.0], "rpy_rad": [0.0, 0.0, 0.0] } }
            ],
            "links": [
                { "from": "pc_coord", "to": "recamera_left", "endpoint": "udp://192.168.42.11:9100", "auth_ref": "env:REBOT_CELL_HMAC_KEY" },
                { "from": "pc_coord", "to": "recamera_right", "endpoint": "udp://192.168.42.12:9100", "auth_ref": "env:REBOT_CELL_HMAC_KEY" }
            ]
        })
    }

    #[test]
    fn digests_match_companion_validator_exactly() {
        let two_so = CellConfig::from_value(&two_so101_json(), None).unwrap();
        assert_eq!(two_so.digest(), "4f13e1e9c588f124cce8273a100c851be90acf125e30edc404e1031b275ff965");
        assert_eq!((two_so.total_dof(), two_so.arm_ids()), (12, vec!["left".into(), "right".into()]));

        let het = CellConfig::from_value(&json!({
            "format": "rebot-cell/v1", "cell_id": "b601_plus_so101", "coordinator": "pc_coord",
            "hosts": {
                "pc_coord": { "address": "192.168.42.10", "role": "pc" },
                "recamera_b601": { "address": "192.168.42.11", "port": 8765, "role": "arm_host" },
                "recamera_so101": { "address": "192.168.42.12", "port": 8765, "role": "arm_host" }
            },
            "arms": [
                { "id": "b601", "type": "b601dm", "profile": "rebot_b601dm", "host": "recamera_b601",
                  "device": "can0", "camera": "b601_wrist",
                  "T_cell_to_base": [[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,-0.25],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]] },
                { "id": "so101", "type": "so-arm101", "profile": "so_arm101", "host": "recamera_so101",
                  "device": "/dev/ttySC0", "camera": "so101_wrist",
                  "T_cell_to_base": { "xyz": [0.0, 0.25, 0.0], "rpy_rad": [0.0, 0.0, 0.0] } }
            ],
            "links": [
                { "from": "pc_coord", "to": "recamera_b601", "endpoint": "udp://192.168.42.11:9100", "auth_ref": "env:REBOT_CELL_HMAC_KEY" },
                { "from": "pc_coord", "to": "recamera_so101", "endpoint": "udp://192.168.42.12:9100", "auth_ref": "env:REBOT_CELL_HMAC_KEY" }
            ]
        }), None).unwrap();
        assert_eq!(het.digest(), "1592599dab0cb34b8e0c6c6085de5ab19e811991428606e8365d6aae41afde44");
        assert_eq!((het.total_dof(), het.arms[0].dof(), het.arms[1].dof()), (13, 7, 6));

        let dual = CellConfig::from_value(&json!({
            "format": "rebot-cell/v1", "cell_id": "dual_b601dm", "coordinator": "pc_coord",
            "hosts": {
                "pc_coord": { "address": "192.168.42.10", "role": "pc" },
                "recamera_left": { "address": "192.168.42.11", "port": 8765, "role": "arm_host" },
                "recamera_right": { "address": "192.168.42.12", "port": 8765, "role": "arm_host" }
            },
            "arms": [
                { "id": "left", "type": "b601dm", "profile": "rebot_b601dm", "host": "recamera_left",
                  "device": "can0", "camera": "L_wrist",
                  "T_cell_to_base": { "xyz": [0.0, -0.25, 0.0], "rpy_rad": [0.0, 0.0, 0.0] } },
                { "id": "right", "type": "b601dm", "profile": "rebot_b601dm", "host": "recamera_right",
                  "device": "can0", "camera": "R_wrist",
                  "T_cell_to_base": { "xyz": [0.0, 0.25, 0.0], "rpy_rad": [0.0, 0.0, 0.0] } }
            ],
            "links": [
                { "from": "pc_coord", "to": "recamera_left", "endpoint": "udp://192.168.42.11:9100", "auth_ref": "env:REBOT_CELL_HMAC_KEY" },
                { "from": "pc_coord", "to": "recamera_right", "endpoint": "udp://192.168.42.12:9100", "auth_ref": "env:REBOT_CELL_HMAC_KEY" }
            ]
        }), None).unwrap();
        assert_eq!(dual.digest(), "5f0c6f59076e1af1b6b37bcb63b71326d0303d925a462185af281ba0208bc30f");
        assert_eq!(dual.total_dof(), 14);
    }

    #[test]
    fn coordinator_is_switchable_by_config_only() {
        let pc = CellConfig::from_value(&two_so101_json(), None).unwrap();
        let mut sw = two_so101_json();
        sw["coordinator"] = json!("recamera_left");
        sw["links"] = json!([{ "from": "recamera_left", "to": "recamera_right", "endpoint": "udp://192.168.42.12:9100", "auth_ref": "env:REBOT_CELL_HMAC_KEY" }]);
        let arm_coord = CellConfig::from_value(&sw, None).unwrap();
        assert_eq!(arm_coord.coordinator, "recamera_left");
        assert_ne!(arm_coord.digest(), pc.digest());
    }

    #[test]
    fn refuses_schema_violations_matching_companion_validator() {
        let mut bad = two_so101_json();
        bad["format"] = json!("rebot-cell/v2");
        assert!(CellConfig::from_value(&bad, None).unwrap_err().contains("Unsupported cell config format"));
        bad["format"] = Value::Null;
        assert!(CellConfig::from_value(&bad, None).unwrap_err().contains("Unsupported cell config format"));

        let mut dup_id = two_so101_json();
        dup_id["arms"][1]["id"] = json!("left");
        assert!(CellConfig::from_value(&dup_id, None).unwrap_err().contains("Duplicate arm id"));

        let mut same_host = two_so101_json();
        same_host["arms"][1]["host"] = json!("recamera_left");
        same_host["arms"][1]["device"] = json!("/dev/ttySC1");
        assert!(CellConfig::from_value(&same_host, None).unwrap_err().contains("one arm per host"));

        let mut same_dev = two_so101_json();
        same_dev["arms"][1]["host"] = json!("recamera_left");
        assert!(CellConfig::from_value(&same_dev, None).unwrap_err().contains("Duplicate device binding"));

        for (mut_fn, expect) in [
            ((|v: &mut Value| v["coordinator"] = json!("missing")) as fn(&mut Value), "unknown host"),
            (|v| v["arms"][1]["host"] = json!("ghost"), "unknown host"),
            (|v| v["links"][0]["to"] = json!("ghost"), "unknown host"),
            (|v| v["links"][0]["to"] = json!("pc_coord"), "to itself"),
            (|v| { let f = v["links"][0].clone(); v["links"].as_array_mut().unwrap().push(f); }, "Duplicate link"),
            (|v| v["links"] = json!([v["links"][0]]), "Missing link between coordinator"),
            (|v| v["arms"][0]["type"] = json!("b601dm"), "conflicts with profile"),
        ] {
            let mut c = two_so101_json();
            mut_fn(&mut c);
            let err = CellConfig::from_value(&c, None).unwrap_err();
            assert!(err.contains(expect), "expected {expect:?} in {err:?}");
        }

        for bad_port in [json!(0), json!(70000), json!(8765.0), json!(true)] {
            let mut c = two_so101_json();
            c["hosts"]["recamera_left"]["port"] = bad_port;
            assert!(CellConfig::from_value(&c, None).is_err());
        }

        for (bad_t, expect) in [
            (json!([[2.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]]), "not orthonormal"),
            (json!([[-1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]]), "det(R) = +1"),
            (json!([[1.0,0.2,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]]), "not orthonormal"),
            (json!([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,2.0]]), "bottom row"),
            (json!({"xyz":[0.0,0.0,0.0],"rpy_rad":[0.0,0.0,0.0],"rpy_deg":[0.0,0.0,0.0]}), "exactly one"),
            (json!({"xyz":[0.0,0.0,0.0],"rpy_rad":[0.0,0.0,0.0],"extra":1}), "unknown keys"),
        ] {
            let err = parse_rigid_transform(&bad_t, "T_cell_to_base").unwrap_err();
            assert!(err.contains(expect), "expected {expect:?} in {err:?}");
        }
    }

    #[test]
    fn host_resolution_and_context_validation() {
        let mut cfg = two_so101_json();
        cfg["hosts"]["observer_box"] = json!({ "address": "192.168.42.99", "role": "observer" });
        let cell = CellConfig::from_value(&cfg, None).unwrap();
        let so101: ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();

        assert_eq!(resolve_local_host(&cell, Some("recamera_left"), None, &[], None).unwrap(), "recamera_left");
        assert!(resolve_local_host(&cell, Some("unknown"), None, &[], None).is_err());
        assert_eq!(resolve_local_host(&cell, None, Some("recamera_right"), &[], None).unwrap(), "recamera_right");
        assert_eq!(resolve_local_host(&cell, None, Some("generic"), &["192.168.42.11".into()], Some(8765)).unwrap(), "recamera_left");
        assert!(resolve_local_host(&cell, None, Some("generic"), &["127.0.0.1".into()], Some(8765)).is_err());

        // Loopback multi-process disambiguation by port & ambiguity rejection
        let mut local = two_so101_json();
        local["hosts"]["pc_coord"]["address"] = json!("127.0.0.1");
        local["hosts"]["recamera_left"] = json!({ "address": "127.0.0.1", "port": 19101 });
        local["hosts"]["recamera_right"] = json!({ "address": "127.0.0.1", "port": 19102 });
        let local_cell = CellConfig::from_value(&local, None).unwrap();
        assert_eq!(resolve_local_host(&local_cell, None, Some("dev"), &["127.0.0.1".into()], Some(19101)).unwrap(), "recamera_left");
        local["hosts"]["recamera_right"]["port"] = json!(19101);
        let ambig = CellConfig::from_value(&local, None).unwrap();
        assert!(resolve_local_host(&ambig, None, Some("dev"), &["127.0.0.1".into()], Some(19101)).unwrap_err().contains("Ambiguous"));

        // Non-participant, DOF mismatch, follower model mismatch, coordinator-only HW, and device mismatch all fail closed
        assert!(CellContext::resolve(cell.clone(), Some("observer_box"), None, &[], None, &so101, Some("mock"), None).unwrap_err().contains("neither an arm host nor the coordinator"));
        assert!(CellContext::resolve(cell.clone(), Some("recamera_left"), None, &[], None, &ArmConfig::rebot_dm(), Some("mock"), None).unwrap_err().contains("requires DOF 6"));
        assert!(CellContext::resolve(cell.clone(), Some("recamera_left"), None, &[], None, &so101, Some("so-arm100"), Some("/dev/ttySC0")).unwrap_err().contains("does not match hardware follower"));
        assert!(CellContext::resolve(cell.clone(), Some("pc_coord"), None, &[], None, &so101, Some("so-arm101"), Some("/dev/ttySC0")).unwrap_err().contains("Coordinator-only host"));
        assert!(CellContext::resolve(cell.clone(), Some("recamera_left"), None, &[], None, &so101, Some("so-arm101"), Some("/dev/ttyUSB9")).unwrap_err().contains("expects device"));

        let ctx = CellContext::resolve(cell.clone(), Some("recamera_left"), None, &[], None, &so101, Some("so-arm101"), Some("/dev/ttySC0")).unwrap();
        assert_eq!((ctx.local_host_id.as_str(), ctx.local_arm_id.as_deref(), ctx.is_coordinator, ctx.local_role()), ("recamera_left", Some("left"), false, "arm_host"));
        // Enabled CellStatusResponse round-trips through serde JSON cleanly.
        let status_json = serde_json::to_value(ctx.status_response()).unwrap();
        let decoded: CellStatusResponse = serde_json::from_value(status_json).unwrap();
        assert!(decoded.enabled && decoded.local_host.as_deref() == Some("recamera_left"));

        // Equivalent IPv6 spellings (0:0:0:0:0:0:0:1 vs ::1) match by parsed IpAddr.
        let mut v6 = two_so101_json();
        v6["hosts"]["recamera_left"]["address"] = json!("0:0:0:0:0:0:0:1");
        let v6_cell = CellConfig::from_value(&v6, None).unwrap();
        assert_eq!(resolve_local_host(&v6_cell, None, Some("dev"), &["::1".into()], Some(8765)).unwrap(), "recamera_left");

        let coord = CellContext::resolve(cell, Some("pc_coord"), None, &[], None, &so101, Some("mock"), None).unwrap();
        assert_eq!((coord.local_host_id.as_str(), coord.local_arm_id.as_deref(), coord.is_coordinator, coord.local_role()), ("pc_coord", None, true, "coordinator"));
        assert_eq!(serde_json::to_value(CellStatusResponse::disabled()).unwrap(), json!({ "enabled": false }));
    }

    #[test]
    fn python_canonical_json_matches_unicode_and_scientific_floats() {
        let mut s1 = String::new();
        write_python_canonical_json(&json!({"s": "相机", "e": "🤖", "ctrl": "\u{0000}\u{001f}\u{0008}\u{000c}\n\r\t\"\\"}), &mut s1);
        assert_eq!(crate::gravity::compute_sha256(s1.as_bytes()), "234f42bb7dd0467458714410f19e60f82cb42effd463a2ebd00e03f898299e83");

        let mut s2 = String::new();
        let floats = Value::Array(
            [0.0, -0.0, 1.0, -0.25, 1e-4, 1e-5, 1e-7, 1.5e-5, 1.23456789e-10, 1e15, 1e16, 1.5e16, 1e100, -1e-7]
                .into_iter().map(Value::from).collect()
        );
        let mut map = Map::new();
        map.insert("f".into(), floats);
        write_python_canonical_json(&Value::Object(map), &mut s2);
        assert_eq!(crate::gravity::compute_sha256(s2.as_bytes()), "4bc16961415095013b8f1b0a712c931364e086c2284c37c41d1b31b1286ff0f1");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn loads_custom_profile_json_relative_to_cell_file() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("custom_4dof.json"), r#"{"profile_id":"custom_4dof","dof":4,"joint_names":["j1","j2","j3","j4"]}"#).unwrap();
        let cell_path = dir.path().join("cell.json");
        std::fs::write(&cell_path, serde_json::to_string(&json!({
            "format": "rebot-cell/v1", "cell_id": "custom_single", "coordinator": "arm_host",
            "hosts": { "arm_host": { "address": "127.0.0.1" } },
            "arms": [{ "id": "a1", "type": "custom_4dof", "profile": "custom_4dof.json", "host": "arm_host",
                       "device": "/dev/ttyUSB0", "camera": null,
                       "T_cell_to_base": { "xyz": [0.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 90.0] } }],
            "links": []
        })).unwrap()).unwrap();
        let cell = CellConfig::from_json_file(&cell_path).unwrap();
        assert_eq!((cell.total_dof(), cell.arms[0].gripper_index()), (4, None));
        assert_eq!(cell.arms[0].joint_names(), &["j1", "j2", "j3", "j4"]);
        assert!((cell.arms[0].t_cell_to_base[0][1] + 1.0).abs() < 1e-12 && (cell.arms[0].t_cell_to_base[1][0] - 1.0).abs() < 1e-12);
    }
}
