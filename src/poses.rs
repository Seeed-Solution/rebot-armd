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
    /// Set by a joint zero change: the pose was taught with the old zero, so the arm reaches a
    /// different physical pose. Saving the pose again clears it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
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

/// Poses held by the supervisor.
#[derive(Debug, Clone, Default)]
pub struct PoseStore {
    pub list: PoseList,
    /// File the store is written to; None = not persisted.
    pub path: Option<PathBuf>,
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
        let pose = self.config.arm_model.fk_f32(q)?;
        Some(PoseTcp { position: pose.p, rpy: crate::kinematics::r_to_rpy(&pose.r) })
    }

    /// Recompute the TCP of every stored pose that has one (in memory) after the arm model
    /// changed (kinematic correction).
    pub(crate) fn refresh_pose_tcps(&mut self) {
        let tcps: Vec<_> = self.poses.list.poses.iter().map(|p| p.tcp.as_ref().and(self.pose_tcp(&p.positions))).collect();
        for (p, tcp) in self.poses.list.poses.iter_mut().zip(tcps) { if tcp.is_some() { p.tcp = tcp; } }
    }

    pub(crate) fn poses_write(&self) -> Option<PendingWrite> {
        self.poses.path.clone().map(|path| PendingWrite::new(path, serde_json::to_string_pretty(&self.poses.list).unwrap_or_default()))
    }

    /// Joint-space quintic of arm joints (and gripper with `include_gripper`) to a stored pose,
    /// recorded as a `move_to_pose` action. Returns (action id, expected duration).
    pub fn move_to_pose(&mut self, name: &str, include_gripper: bool, speed_scale: f64, tag: Option<String>, epoch: u64) -> Result<(u64, f64), ArmError> {
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
        let mut target = self.commanded_positions.clone();
        let gripper_idx = self.config.gripper_index();
        for i in (0..self.config.dof).filter(|i| Some(*i) != gripper_idx) { target[i] = pose.positions[i]; }
        if let (true, Some(g)) = (include_gripper, gripper_idx) { target[g] = pose.positions[g]; }
        self.action_preconditions()?;
        let mut limits = self.planning_limits();
        for (i, l) in limits.iter_mut().enumerate() {
            if Some(i) != gripper_idx { l.max_velocity *= speed_scale as f32; }
        }
        let (_, id, duration) = self.start_quintic(target, None, Some(epoch), "move_to_pose", tag, &limits)?;
        Ok((id, duration))
    }
}

/// WS command `/api/v1/poses/save` `{name, positions?, id?}`: replaces the pose `name`, or with
/// `id` the pose with that id (rename: the id stays). Returns the pose and the file write.
pub fn save(inner: &mut SupervisorInner, body: &Value) -> Result<(NamedPose, Option<PendingWrite>), ArmError> {
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
        stale: None,
    };
    inner.poses.list.profile_id = profile_id;
    inner.poses.list.arm_dof = Some(arm_dof);
    inner.poses.list.gripper_joint = gripper_joint;
    inner.poses.list.joint_names = Some(joint_names);
    let list = &mut inner.poses.list.poses;
    match list.iter_mut().find(|p| p.id == pose.id) {
        Some(p) => *p = pose.clone(),
        None => list.push(pose.clone()),
    }
    Ok((pose, inner.poses_write()))
}

/// WS command `/api/v1/poses/delete` `{name}`. IN_USE `{sequences}` while a sequence
/// references the pose (or a running sequence pinned it).
pub fn delete(inner: &mut SupervisorInner, body: &Value) -> Result<Option<PendingWrite>, ArmError> {
    let name = name_of(body)?;
    let i = inner.poses.list.poses.iter().position(|p| p.name == name).ok_or_else(|| not_found(&name))?;
    let id = inner.poses.list.poses[i].id.clone();
    let users = inner.sequences_using_pose(&id);
    if !users.is_empty() {
        return Err(ArmError::detailed(409, "IN_USE", format!("Pose '{name}' is used by {} sequence(s)", users.len()), serde_json::json!({"sequences": users})));
    }
    inner.poses.list.poses.remove(i);
    Ok(inner.poses_write())
}

/// Optional `speed_scale` in [MIN_SPEED_SCALE, 1.0], default 1.0.
pub(crate) fn speed_scale(body: &Value) -> Result<f64, ArmError> {
    let speed_scale = motion::num(body, "speed_scale")?.unwrap_or(1.0);
    if !(MIN_SPEED_SCALE..=1.0).contains(&speed_scale) {
        return Err(invalid(format!("speed_scale must be in [{MIN_SPEED_SCALE}, 1.0]")));
    }
    Ok(speed_scale)
}

/// Action `/api/v1/action/move_to_pose` `{name, include_gripper?, speed_scale?, tag?}`.
pub fn start_action(inner: &mut SupervisorInner, body: &Value, epoch: u64) -> Result<(u64, f64), ArmError> {
    let name = name_of(body)?;
    let include_gripper = motion::flag(body, "include_gripper", false)?;
    let speed_scale = speed_scale(body)?;
    let tag = motion::tag(body)?;
    inner.move_to_pose(&name, include_gripper, speed_scale, tag, epoch)
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
            // a saved TCP follows from the positions; files saved under another kinematic model
            // (before the J6 offset, 2026-11-12) carry a stale one
            let tcps: Vec<_> = inner.poses.list.poses.iter().map(|p| p.tcp.as_ref().and(inner.pose_tcp(&p.positions))).collect();
            for (p, tcp) in inner.poses.list.poses.iter_mut().zip(tcps) { if tcp.is_some() { p.tcp = tcp; } }
            if inner.poses.assign_ids() { return inner.poses_write(); }
        }
        Loaded::Missing => {}
        Loaded::Invalid(e) => tracing::warn!("Ignoring poses file: {e}; starting with no poses"),
    }
    None
}
