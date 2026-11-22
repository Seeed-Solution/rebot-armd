//! Named joint poses: `GET /api/v1/poses`, WS commands `/api/v1/poses/save` and
//! `/api/v1/poses/delete`, action `/api/v1/action/move_to_pose`. Persisted to `ARMD_POSES`.

use std::path::{Path, PathBuf};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::motion::{self, MotionError};
use crate::persist::{self, Loaded, PendingWrite};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const DEFAULT_POSES_PATH: &str = "/userdata/rebot-teleop/poses.json";
/// Most poses the store holds.
pub const MAX_POSES: usize = 64;
pub const MIN_SPEED_SCALE: f64 = 0.1;

/// `ARMD_POSES` or [`DEFAULT_POSES_PATH`].
pub fn path_from_env() -> PathBuf {
    persist::path_from_env("ARMD_POSES", DEFAULT_POSES_PATH)
}

/// End-link pose at save time (base frame; rpy as in [`crate::kinematics::r_to_rpy`]).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PoseTcp {
    pub position: [f64; 3],
    pub rpy: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct NamedPose {
    /// Stable id (ULID): assigned at the first save, or on load for poses saved before ids;
    /// kept across a rename. Sequences reference poses by it.
    #[serde(default)]
    pub id: String,
    pub name: String,
    /// One entry per joint (profile DOF).
    pub positions: Vec<f32>,
    /// Null when the profile has no kinematics model.
    pub tcp: Option<PoseTcp>,
    /// Unix time (ms) of the save.
    pub saved_at_ms: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profile_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arm_dof: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper_joint: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub joint_names: Option<Vec<String>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_config_digest: Option<String>,
    /// Set by a joint zero change: the pose was taught with the old zero, so the arm reaches a
    /// different physical pose. Saving the pose again clears it. Always present (null when not
    /// stale).
    #[serde(default)]
    pub stale: Option<crate::calibration::CalibrationStale>,
}

/// File body and `GET /api/v1/poses` body.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct PoseList {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profile_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arm_dof: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper_joint: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub joint_names: Option<Vec<String>>,
    pub poses: Vec<NamedPose>,
}

/// Compute the canonical SHA-256 hex digest of a pose's identity, joint names, and joint angles.
pub fn compute_pose_content_sha256(pose_id: &str, joint_names: &[String], joints: &[f64]) -> String {
    let mut hasher = crate::gravity::Sha256::new();
    hasher.update(pose_id.as_bytes());
    hasher.update(&[0x00]);
    for name in joint_names {
        hasher.update(name.as_bytes());
        hasher.update(&[0x00]);
    }
    hasher.update(&[0x01]);
    for &v in joints {
        let canonical = if v == 0.0 { 0.0_f64 } else { v };
        hasher.update(&canonical.to_le_bytes());
    }
    hasher.finalize_hex()
}

/// Serialize a [`NamedPose`] enriched with `pose_id`, `content_sha256`, and `joint_names`.
pub fn pose_response_value(pose: &NamedPose, default_joint_names: &[String]) -> Value {
    let effective_joint_names: Vec<String> = pose
        .joint_names
        .clone()
        .unwrap_or_else(|| default_joint_names.to_vec());
    let joints_f64: Vec<f64> = pose.positions.iter().map(|&v| f64::from(v)).collect();
    let content_sha256 = compute_pose_content_sha256(&pose.id, &effective_joint_names, &joints_f64);
    let mut val = serde_json::to_value(pose).unwrap_or_else(|_| serde_json::json!({}));
    if let Some(obj) = val.as_object_mut() {
        obj.insert("pose_id".into(), Value::String(pose.id.clone()));
        obj.insert("content_sha256".into(), Value::String(content_sha256));
        if !obj.contains_key("joint_names") {
            obj.insert("joint_names".into(), serde_json::json!(effective_joint_names));
        }
    }
    val
}

/// `GET /api/v1/poses` body (adds `revision` to [`PoseList`]).
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct PoseListResponse {
    #[serde(flatten)]
    pub list: PoseList,
    pub revision: u64,
}

/// Poses held by the supervisor.
#[derive(Debug, Clone)]
pub struct PoseStore {
    pub list: PoseList,
    /// File the store is written to; None = not persisted.
    pub path: Option<PathBuf>,
    /// Monotonic revision (starts at 1).
    pub revision: u64,
}

impl Default for PoseStore {
    fn default() -> Self {
        Self {
            list: PoseList::default(),
            path: None,
            revision: 1,
        }
    }
}

impl PoseStore {
    /// The stored pose `name`, or `POSE_NOT_FOUND`.
    pub(crate) fn find(&self, name: &str) -> Result<&NamedPose, ArmError> {
        self.list.poses.iter().find(|p| p.name == name).ok_or_else(|| not_found(name))
    }

    /// The stored pose with id `id`.
    pub fn by_id(&self, id: &str) -> Option<&NamedPose> { self.list.poses.iter().find(|p| p.id == id) }

    /// Give every pose without an id a new one. Returns whether any changed.
    pub fn assign_ids(&mut self) -> bool {
        let mut changed = false;
        for p in self.list.poses.iter_mut().filter(|p| p.id.is_empty()) {
            p.id = crate::recording::new_ulid();
            changed = true;
        }
        changed
    }
}

pub fn valid_name(name: &str) -> bool {
    (1..=64).contains(&name.len()) && name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

pub(crate) fn not_found(name: &str) -> ArmError {
    ArmError::Rejected { status: 404, code: "POSE_NOT_FOUND".into(), message: format!("No pose named '{name}'") }
}

fn invalid(msg: impl Into<String>) -> ArmError { MotionError::InvalidCommand(msg.into()).into() }

fn name_of(body: &Value) -> Result<String, ArmError> {
    let name = body.get("name").and_then(Value::as_str).ok_or_else(|| invalid("name must be a string"))?;
    if !valid_name(name) { return Err(invalid("name must match ^[A-Za-z0-9_-]{1,64}$")); }
    Ok(name.to_string())
}

/// Check a loaded pose list: count, names, DOF and finite positions.
pub fn validate_list(list: &PoseList, config: &crate::model::ArmConfig) -> Result<(), String> {
    if list.poses.len() > MAX_POSES { return Err(format!("more than {MAX_POSES} poses")); }
    let cfg_joint_names: Vec<String> = config.limits.iter().map(|l| l.name.clone()).collect();
    if let (Some(ref list_pid), Some(ref cfg_pid)) = (&list.profile_id, &config.profile_id) {
        if list_pid != cfg_pid {
            return Err(format!("POSE_DOF_MISMATCH: pose list profile_id '{list_pid}' does not match config profile_id '{cfg_pid}'"));
        }
    }
    if let Some(ad) = list.arm_dof {
        if ad != config.arm_dof() {
            return Err(format!("POSE_DOF_MISMATCH: pose list arm_dof {ad} does not match config arm_dof {}", config.arm_dof()));
        }
    }
    if let Some(gj) = list.gripper_joint {
        if Some(gj) != config.gripper_index() {
            return Err(format!("POSE_DOF_MISMATCH: pose list gripper_joint {gj} does not match config gripper_joint {:?}", config.gripper_index()));
        }
    }
    if let Some(ref jn) = list.joint_names {
        if jn != &cfg_joint_names {
            return Err(format!("POSE_DOF_MISMATCH: pose list joint_names {:?} do not match config joint_names {:?}", jn, cfg_joint_names));
        }
    }
    for (i, p) in list.poses.iter().enumerate() {
        if !valid_name(&p.name) { return Err(format!("invalid pose name '{}'", p.name)); }
        if list.poses[..i].iter().any(|q| q.name == p.name) { return Err(format!("duplicate pose name '{}'", p.name)); }
        if !p.id.is_empty() && (!crate::recording::valid_episode_id(&p.id) || list.poses[..i].iter().any(|q| q.id == p.id)) {
            return Err(format!("pose '{}': invalid or duplicate id '{}'", p.name, p.id));
        }
        if let (Some(ref p_pid), Some(ref cfg_pid)) = (&p.profile_id, &config.profile_id) {
            if p_pid != cfg_pid {
                return Err(format!("POSE_DOF_MISMATCH: pose '{}' profile_id '{p_pid}' does not match config profile_id '{cfg_pid}'", p.name));
            }
        }
        let effective_arm_dof = p.arm_dof.or(list.arm_dof);
        if config.arm_dof() == 5 && effective_arm_dof.is_none() {
            return Err(format!("POSE_DOF_MISMATCH: pose '{}' missing arm_dof on 5-DOF profile", p.name));
        }
        if let Some(ad) = effective_arm_dof {
            if ad != config.arm_dof() {
                return Err(format!("POSE_DOF_MISMATCH: pose '{}' arm_dof {ad} does not match config arm_dof {}", p.name, config.arm_dof()));
            }
        }
        if let Some(gj) = p.gripper_joint {
            if Some(gj) != config.gripper_index() {
                return Err(format!("POSE_DOF_MISMATCH: pose '{}' gripper_joint {gj} does not match config gripper_joint {:?}", p.name, config.gripper_index()));
            }
        }
        if let Some(ref jn) = p.joint_names {
            if jn != &cfg_joint_names {
                return Err(format!("POSE_DOF_MISMATCH: pose '{}' joint_names {:?} do not match config joint_names {:?}", p.name, jn, cfg_joint_names));
            }
        }
        // Joint limits are checked when a pose is used, so a profile change does not drop the file.
        if p.positions.len() != config.dof || !p.positions.iter().all(|v| v.is_finite()) {
            return Err(format!("pose '{}': positions must be {} finite numbers", p.name, config.dof));
        }
    }
    Ok(())
}

fn check_positions(q: &[f32], config: &crate::model::ArmConfig) -> Result<(), String> {
    if q.len() != config.dof { return Err(format!("positions must have {} entries", config.dof)); }
    for (v, l) in q.iter().zip(&config.limits) {
        if !v.is_finite() || *v < l.min_position || *v > l.max_position {
            return Err(format!("joint '{}' position {v} outside [{}, {}]", l.name, l.min_position, l.max_position));
        }
    }
    Ok(())
}

impl SupervisorInner {
    fn pose_tcp(&self, q: &[f32]) -> Option<PoseTcp> {
        self.config.kinematics?;
        let pose = self.config.tcp_pose_f32(q)?;
        Some(PoseTcp { position: pose.p, rpy: crate::kinematics::r_to_rpy(&pose.r) })
    }

    fn recompute_pose_tcp(&self, p: &NamedPose) -> Option<PoseTcp> {
        let cfg_pid = self.config.profile_id.as_ref();
        let pose_pid = p.profile_id.as_ref().or(self.poses.list.profile_id.as_ref());
        if let (Some(pp), Some(cp)) = (pose_pid, cfg_pid) {
            if pp != cp { return None; }
        }
        let pose_arm_dof = p.arm_dof.or(self.poses.list.arm_dof);
        if self.config.arm_dof() == 5 && pose_arm_dof.is_none() { return None; }
        if pose_arm_dof.is_some_and(|ad| ad != self.config.arm_dof()) { return None; }
        if p.gripper_joint.is_some_and(|gj| Some(gj) != self.config.gripper_index()) { return None; }
        if let Some(ref jn) = p.joint_names {
            let cfg_jn: Vec<String> = self.config.limits.iter().map(|l| l.name.clone()).collect();
            if jn != &cfg_jn { return None; }
        }
        if check_positions(&p.positions, &self.config).is_err() { return None; }
        self.pose_tcp(&p.positions)
    }

    /// Recompute the TCP of every stored pose (in memory) from its joints, clearing stale
    /// cached TCPs whenever the joints are invalid or incompatible with the active config.
    pub fn refresh_pose_tcps(&mut self) {
        let tcps: Vec<_> = self.poses.list.poses.iter().map(|p| self.recompute_pose_tcp(p)).collect();
        for (p, tcp) in self.poses.list.poses.iter_mut().zip(tcps) { p.tcp = tcp; }
    }

    pub(crate) fn poses_write(&self) -> Option<PendingWrite> {
        self.poses.path.clone().map(|path| PendingWrite::new(path, serde_json::to_string_pretty(&self.poses.list).unwrap_or_default()))
    }

    /// Joint-space quintic of arm joints (and gripper with `include_gripper`) to a stored pose,
    /// recorded as a `move_to_pose` action. Returns (action id, expected duration).
    pub fn move_to_pose(&mut self, name: &str, include_gripper: bool, speed_scale: f64, tag: Option<String>, epoch: u64) -> Result<(u64, f64), ArmError> {
        self.move_to_pose_ex(name, include_gripper, speed_scale, tag, epoch, crate::backlash::Request::Profile)
    }

    /// [`move_to_pose`](Self::move_to_pose) with the request field `approach`: with the
    /// same-side approach (`src/backlash.rs`) the quintic ends at the pre-target and a short
    /// segment (phase `approach`) reaches the pose.
    pub fn move_to_pose_ex(&mut self, name: &str, include_gripper: bool, speed_scale: f64, tag: Option<String>, epoch: u64,
        approach: crate::backlash::Request) -> Result<(u64, f64), ArmError> {
        self.check_commissioning_for_ordinary_motion("move_to_pose")?;
        let pose = self.poses.find(name)?;
        let cfg_joint_names: Vec<String> = self.config.limits.iter().map(|l| l.name.clone()).collect();
        let effective_pid = pose.profile_id.as_ref().or(self.poses.list.profile_id.as_ref());
        if let (Some(pose_pid), Some(ref cfg_pid)) = (effective_pid, &self.config.profile_id) {
            if pose_pid != cfg_pid {
                return Err(ArmError::Rejected {
                    status: 400,
                    code: "POSE_DOF_MISMATCH".into(),
                    message: format!("Pose '{name}' profile_id '{pose_pid}' does not match active config profile_id '{cfg_pid}'"),
                });
            }
        }
        let effective_arm_dof = pose.arm_dof.or(self.poses.list.arm_dof);
        if self.config.arm_dof() == 5 && effective_arm_dof.is_none() {
            return Err(ArmError::Rejected {
                status: 400,
                code: "POSE_DOF_MISMATCH".into(),
                message: format!("Pose '{name}' missing arm_dof on 5-DOF profile"),
            });
        }
        if let Some(ad) = effective_arm_dof {
            if ad != self.config.arm_dof() {
                return Err(ArmError::Rejected {
                    status: 400,
                    code: "POSE_DOF_MISMATCH".into(),
                    message: format!("Pose '{name}' arm_dof {ad} does not match active config arm_dof {}", self.config.arm_dof()),
                });
            }
        }
        if let Some(gj) = pose.gripper_joint {
            if Some(gj) != self.config.gripper_index() {
                return Err(ArmError::Rejected {
                    status: 400,
                    code: "POSE_DOF_MISMATCH".into(),
                    message: format!("Pose '{name}' gripper_joint {gj} does not match active config gripper_joint {:?}", self.config.gripper_index()),
                });
            }
        }
        if let Some(ref jn) = pose.joint_names {
            if jn != &cfg_joint_names {
                return Err(ArmError::Rejected {
                    status: 400,
                    code: "POSE_DOF_MISMATCH".into(),
                    message: format!("Pose '{name}' joint_names do not match active config joint_names"),
                });
            }
        }
        check_positions(&pose.positions, &self.config).map_err(invalid)?;
        let gripper_idx = self.config.gripper_index();
        if include_gripper && gripper_idx.is_some()
            && !self
                .config
                .active_tool
                .matches_record(pose.tool_id.as_deref(), pose.tool_config_digest.as_deref())
        {
            return Err(ArmError::Rejected {
                status: 409,
                code: "TOOL_MISMATCH".into(),
                message: format!(
                    "Pose '{name}' gripper position was saved with tool {:?} ({:?}), which does not match active tool '{}' ({}); pass include_gripper: false to move arm joints only",
                    pose.tool_id,
                    pose.tool_config_digest,
                    self.config.active_tool.id,
                    self.config.active_tool.config_digest
                ),
            });
        }
        let mut target = self.commanded_positions.clone();
        for i in (0..self.config.dof).filter(|i| Some(*i) != gripper_idx) { target[i] = pose.positions[i]; }
        if let (true, Some(g)) = (include_gripper, gripper_idx) { target[g] = pose.positions[g]; }
        self.action_preconditions()?;
        let mut limits = self.planning_limits();
        for (i, l) in limits.iter_mut().enumerate() {
            if Some(i) != gripper_idx { l.max_velocity *= speed_scale as f32; }
        }
        let (_, id, duration) = self.start_quintic_same_side(target, Some(epoch), "move_to_pose", tag, &limits, "move_to_pose", approach)?;
        Ok((id, duration))
    }
}

/// Validated pose save prepared without mutating `SupervisorInner`.
#[derive(Debug, Clone)]
pub struct PreparedPosesSave {
    pub base_revision: u64,
    pub pose: NamedPose,
    pub next_list: PoseList,
    pub write: Option<PendingWrite>,
}

/// Validated pose delete prepared without mutating `SupervisorInner`.
#[derive(Debug, Clone)]
pub struct PreparedPosesDelete {
    pub base_revision: u64,
    pub next_list: PoseList,
    pub write: Option<PendingWrite>,
}

fn check_pose_revision(inner: &SupervisorInner, body: &Value, require_expected_revision: bool) -> Result<u64, ArmError> {
    if !body.is_object() {
        return Err(invalid("Command body must be an object"));
    }
    if let Some(exp) = persist::parse_expected_revision(body, require_expected_revision).map_err(invalid)? {
        if exp != inner.poses.revision {
            return Err(ArmError::PolicyRevisionMismatch { expected: exp, current: inner.poses.revision });
        }
    }
    Ok(inner.poses.revision)
}

/// Step 1 of atomic persist-before-commit for `/api/v1/poses/save` `{name, positions?, id?}`:
/// replaces the pose `name`, or with `id` the pose with that id (rename: the id stays).
pub fn prepare_save_with(
    inner: &SupervisorInner,
    body: &Value,
    require_expected_revision: bool,
) -> Result<PreparedPosesSave, ArmError> {
    let base_revision = check_pose_revision(inner, body, require_expected_revision)?;
    let name = name_of(body)?;
    let by_id = match body.get("id") {
        None | Some(Value::Null) => None,
        Some(Value::String(id)) => {
            if inner.poses.by_id(id).is_none() {
                return Err(ArmError::Rejected { status: 404, code: "POSE_NOT_FOUND".into(), message: format!("No pose with id '{id}'") });
            }
            Some(id.clone())
        }
        Some(_) => return Err(invalid("id must be a string")),
    };
    if let Some(id) = &by_id {
        if inner.poses.list.poses.iter().any(|p| p.name == name && &p.id != id) {
            return Err(ArmError::Rejected { status: 409, code: "POSE_EXISTS".into(), message: format!("Another pose is named '{name}'") });
        }
    }
    let positions = match body.get("positions") {
        None | Some(Value::Null) => inner.measured_positions.clone(),
        Some(v) => v.as_array().ok_or_else(|| invalid("positions must be an array"))?.iter()
            .map(|x| x.as_f64().filter(|f| f.is_finite()).map(|f| f as f32).ok_or_else(|| invalid("positions must be finite numbers")))
            .collect::<Result<Vec<_>, _>>()?,
    };
    check_positions(&positions, &inner.config).map_err(invalid)?;
    let list = &inner.poses.list.poses;
    if !list.iter().any(|p| p.name == name) && list.len() >= MAX_POSES {
        return Err(invalid(format!("at most {MAX_POSES} poses; delete one first")));
    }
    let saved_at_ms = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64);
    let profile_id = inner.config.profile_id.clone();
    let arm_dof = inner.config.arm_dof();
    let gripper_joint = inner.config.gripper_index();
    let joint_names: Vec<String> = inner.config.limits.iter().map(|l| l.name.clone()).collect();
    let id = match &by_id {
        Some(id) => id.clone(),
        None => inner.poses.list.poses.iter().find(|p| p.name == name).map(|p| p.id.clone()).filter(|id| !id.is_empty())
            .unwrap_or_else(crate::recording::new_ulid),
    };
    let pose = NamedPose {
        tcp: inner.pose_tcp(&positions),
        id,
        name,
        positions,
        saved_at_ms,
        profile_id: profile_id.clone(),
        arm_dof: Some(arm_dof),
        gripper_joint,
        joint_names: Some(joint_names.clone()),
        tool_id: Some(inner.config.active_tool.id.clone()),
        tool_config_digest: Some(inner.config.active_tool.config_digest.clone()),
        stale: None,
    };
    let mut next_list = inner.poses.list.clone();
    next_list.profile_id = profile_id;
    next_list.arm_dof = Some(arm_dof);
    next_list.gripper_joint = gripper_joint;
    next_list.joint_names = Some(joint_names);
    match next_list.poses.iter_mut().find(|p| p.id == pose.id) {
        Some(p) => *p = pose.clone(),
        None => next_list.poses.push(pose.clone()),
    }
    let write = inner.poses.path.clone().map(|path| {
        PendingWrite::new(path, serde_json::to_string_pretty(&next_list).unwrap_or_default())
    });
    Ok(PreparedPosesSave { base_revision, pose, next_list, write })
}

pub fn precheck_save(inner: &SupervisorInner, prepared: &PreparedPosesSave) -> Result<(), ArmError> {
    if inner.poses.revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch { expected: prepared.base_revision, current: inner.poses.revision });
    }
    if inner.calibration_busy {
        return Err(ArmError::Rejected { status: 409, code: "CONFIG_BUSY".into(), message: "a camera calibration apply or restore is running".into() });
    }
    Ok(())
}

pub fn commit_save(inner: &mut SupervisorInner, prepared: PreparedPosesSave) -> Result<(NamedPose, u64), ArmError> {
    if inner.poses.revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch { expected: prepared.base_revision, current: inner.poses.revision });
    }
    inner.poses.list = prepared.next_list;
    inner.poses.revision += 1;
    Ok((prepared.pose, inner.poses.revision))
}

/// Step 1 of atomic persist-before-commit for `/api/v1/poses/delete`.
pub fn prepare_delete_with(
    inner: &SupervisorInner,
    body: &Value,
    require_expected_revision: bool,
) -> Result<PreparedPosesDelete, ArmError> {
    let base_revision = check_pose_revision(inner, body, require_expected_revision)?;
    let name = name_of(body)?;
    let mut next_list = inner.poses.list.clone();
    let i = next_list.poses.iter().position(|p| p.name == name).ok_or_else(|| not_found(&name))?;
    let users = inner.sequences_using_pose(&next_list.poses[i].id);
    if !users.is_empty() {
        return Err(ArmError::detailed(409, "IN_USE", format!("Pose '{name}' is used by {} sequence(s)", users.len()), serde_json::json!({"sequences": users})));
    }
    next_list.poses.remove(i);
    let write = inner.poses.path.clone().map(|path| {
        PendingWrite::new(path, serde_json::to_string_pretty(&next_list).unwrap_or_default())
    });
    Ok(PreparedPosesDelete { base_revision, next_list, write })
}

pub fn precheck_delete(inner: &SupervisorInner, prepared: &PreparedPosesDelete) -> Result<(), ArmError> {
    if inner.poses.revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch { expected: prepared.base_revision, current: inner.poses.revision });
    }
    // a sequence saved since the prepare may reference the pose now
    for p in inner.poses.list.poses.iter().filter(|p| !prepared.next_list.poses.iter().any(|n| n.id == p.id)) {
        let users = inner.sequences_using_pose(&p.id);
        if !users.is_empty() {
            return Err(ArmError::detailed(409, "IN_USE", format!("Pose '{}' is used by {} sequence(s)", p.name, users.len()), serde_json::json!({"sequences": users})));
        }
    }
    if inner.calibration_busy {
        return Err(ArmError::Rejected { status: 409, code: "CONFIG_BUSY".into(), message: "a camera calibration apply or restore is running".into() });
    }
    Ok(())
}

pub fn commit_delete(inner: &mut SupervisorInner, prepared: PreparedPosesDelete) -> Result<u64, ArmError> {
    if inner.poses.revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch { expected: prepared.base_revision, current: inner.poses.revision });
    }
    inner.poses.list = prepared.next_list;
    inner.poses.revision += 1;
    Ok(inner.poses.revision)
}

/// WS command `/api/v1/poses/save` `{name, positions?}`. Returns the pose and the file write.
pub fn save(inner: &mut SupervisorInner, body: &Value) -> Result<(NamedPose, Option<PendingWrite>), ArmError> {
    let mut prepared = prepare_save_with(inner, body, false)?;
    let write = prepared.write.take();
    let (pose, _) = commit_save(inner, prepared)?;
    Ok((pose, write))
}

/// Step 1 of `/api/v1/poses/rename` `{id, name, expected_revision?}`: the pose keeps its id,
/// positions and stale mark; sequences reference the id and keep working. POSE_EXISTS when
/// another pose has the name. Committed with [`precheck_save`] / [`commit_save`].
pub fn prepare_rename_with(inner: &SupervisorInner, body: &Value, require_expected_revision: bool) -> Result<PreparedPosesSave, ArmError> {
    let base_revision = check_pose_revision(inner, body, require_expected_revision)?;
    let name = name_of(body)?;
    let id = body.get("id").and_then(Value::as_str).ok_or_else(|| invalid("id must be a string"))?;
    if inner.poses.list.poses.iter().any(|p| p.name == name && p.id != id) {
        return Err(ArmError::Rejected { status: 409, code: "POSE_EXISTS".into(), message: format!("Another pose is named '{name}'") });
    }
    let mut next_list = inner.poses.list.clone();
    let pose = next_list.poses.iter_mut().find(|p| p.id == id)
        .ok_or_else(|| ArmError::Rejected { status: 404, code: "POSE_NOT_FOUND".into(), message: format!("No pose with id '{id}'") })?;
    pose.name = name;
    let pose = pose.clone();
    let write = inner.poses.path.clone().map(|path| {
        PendingWrite::new(path, serde_json::to_string_pretty(&next_list).unwrap_or_default())
    });
    Ok(PreparedPosesSave { base_revision, pose, next_list, write })
}

/// WS command `/api/v1/poses/rename` `{id, name}` (in-process callers): prepare and commit.
pub fn rename(inner: &mut SupervisorInner, body: &Value) -> Result<(NamedPose, Option<PendingWrite>), ArmError> {
    let mut prepared = prepare_rename_with(inner, body, false)?;
    let write = prepared.write.take();
    let (pose, _) = commit_save(inner, prepared)?;
    Ok((pose, write))
}

/// WS command `/api/v1/poses/delete` `{name}`. IN_USE `{sequences}` while a sequence
/// references the pose (or a running sequence pinned it).
pub fn delete(inner: &mut SupervisorInner, body: &Value) -> Result<Option<PendingWrite>, ArmError> {
    let mut prepared = prepare_delete_with(inner, body, false)?;
    let write = prepared.write.take();
    let _ = commit_delete(inner, prepared)?;
    Ok(write)
}

/// Optional `speed_scale` in [MIN_SPEED_SCALE, 1.0], default 1.0.
pub(crate) fn speed_scale(body: &Value) -> Result<f64, ArmError> {
    let speed_scale = motion::num(body, "speed_scale")?.unwrap_or(1.0);
    if !(MIN_SPEED_SCALE..=1.0).contains(&speed_scale) {
        return Err(invalid(format!("speed_scale must be in [{MIN_SPEED_SCALE}, 1.0]")));
    }
    Ok(speed_scale)
}

/// Action `/api/v1/action/move_to_pose` `{name, include_gripper?, speed_scale?, tag?, approach?}`.
pub fn start_action(inner: &mut SupervisorInner, body: &Value, epoch: u64) -> Result<(u64, f64), ArmError> {
    let name = name_of(body)?;
    let include_gripper = motion::flag(body, "include_gripper", false)?;
    let speed_scale = speed_scale(body)?;
    let tag = motion::tag(body)?;
    let approach = crate::backlash::parse_request(body)?;
    inner.move_to_pose_ex(&name, include_gripper, speed_scale, tag, epoch, approach)
}

/// Load the persisted poses at startup (logs the outcome). An invalid file is left untouched
/// and the store starts empty; the next successful save replaces the file.
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) {
    let write = load_locked(supervisor, path).await;
    // poses saved before ids got theirs: written back so the ids stay stable
    if let Some(w) = write {
        if let Err(e) = w.run().await { tracing::warn!("Poses file not updated with ids: {e}"); }
    }
}

async fn load_locked(supervisor: &ArmSupervisor, path: &Path) -> Option<PendingWrite> {
    let mut inner = supervisor.inner().write().await;
    inner.poses.path = Some(path.to_path_buf());
    let config = inner.config.clone();
    match persist::load::<PoseList>(path, |l| validate_list(l, &config)) {
        Loaded::Loaded(list) => {
            tracing::info!("{} named poses loaded from {}", list.poses.len(), path.display());
            inner.poses.list = list;
            // Recompute TCP unconditionally from valid compatible joints (or clear stale cached TCPs).
            inner.refresh_pose_tcps();
            if inner.poses.assign_ids() { return inner.poses_write(); }
        }
        Loaded::Missing => {}
        Loaded::Invalid(e) => tracing::warn!("Ignoring poses file: {e}; starting with no poses"),
    }
    None
}
