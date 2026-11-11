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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NamedPose {
    pub name: String,
    /// One entry per joint (profile DOF).
    pub positions: Vec<f32>,
    /// Null when the profile has no kinematics model.
    pub tcp: Option<PoseTcp>,
    /// Unix time (ms) of the save.
    pub saved_at_ms: u64,
}

/// File body and `GET /api/v1/poses` body.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct PoseList {
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
    for (i, p) in list.poses.iter().enumerate() {
        if !valid_name(&p.name) { return Err(format!("invalid pose name '{}'", p.name)); }
        if list.poses[..i].iter().any(|q| q.name == p.name) { return Err(format!("duplicate pose name '{}'", p.name)); }
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

    fn poses_write(&self) -> Option<PendingWrite> {
        self.poses.path.clone().map(|path| PendingWrite::new(path, serde_json::to_string_pretty(&self.poses.list).unwrap_or_default()))
    }

    /// Joint-space quintic of J1..J6 (and J7 with `include_gripper`) to a stored pose,
    /// recorded as a `move_to_pose` action. Returns (action id, expected duration).
    pub fn move_to_pose(&mut self, name: &str, include_gripper: bool, speed_scale: f64, tag: Option<String>, epoch: u64) -> Result<(u64, f64), ArmError> {
        let pose = self.poses.find(name)?;
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

/// WS command `/api/v1/poses/save` `{name, positions?}`. Returns the pose and the file write.
pub fn save(inner: &mut SupervisorInner, body: &Value) -> Result<(NamedPose, Option<PendingWrite>), ArmError> {
    let name = name_of(body)?;
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
    let pose = NamedPose { tcp: inner.pose_tcp(&positions), name, positions, saved_at_ms };
    let list = &mut inner.poses.list.poses;
    match list.iter_mut().find(|p| p.name == pose.name) {
        Some(p) => *p = pose.clone(),
        None => list.push(pose.clone()),
    }
    Ok((pose, inner.poses_write()))
}

/// WS command `/api/v1/poses/delete` `{name}`.
pub fn delete(inner: &mut SupervisorInner, body: &Value) -> Result<Option<PendingWrite>, ArmError> {
    let name = name_of(body)?;
    let list = &mut inner.poses.list.poses;
    let i = list.iter().position(|p| p.name == name).ok_or_else(|| not_found(&name))?;
    list.remove(i);
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
    let mut inner = supervisor.inner().write().await;
    inner.poses.path = Some(path.to_path_buf());
    let config = inner.config.clone();
    match persist::load::<PoseList>(path, |l| validate_list(l, &config)) {
        Loaded::Loaded(list) => {
            tracing::info!("{} named poses loaded from {}", list.poses.len(), path.display());
            inner.poses.list = list;
        }
        Loaded::Missing => {}
        Loaded::Invalid(e) => tracing::warn!("Ignoring poses file: {e}; starting with no poses"),
    }
}
