//! Persist-before-commit configuration writes of the WS command path (track B).
//!
//! Moved from `api/ws.rs` without changes (armd 1.6 RT-core foundation); moved items, their
//! fields and inherent methods are `pub(super)` so the connection loop keeps using them.

#[allow(unused_imports)]
use super::*;

/// Atomic persist-before-commit config write (Step 1 prepared under lock, Step 2 staged to
/// disk without lock under `config_write_lock`, Step 3 prechecked + published + committed under lock).
pub(super) enum PreparedConfigWrite {
    Keepout(crate::keepout::PreparedKeepout),
    Collision(crate::collision::PreparedCollision),
    GripperCalib(Box<crate::gripper_calib::PreparedGripperCalib>),
    PosesSave(crate::poses::PreparedPosesSave),
    PosesDelete(crate::poses::PreparedPosesDelete),
    PosesRename(crate::poses::PreparedPosesSave),
    Sequences(crate::sequences::PreparedSequences),
    GraspSetup(Box<crate::grasp::PreparedGraspSetup>),
    ToolCommissioning(crate::tool::PreparedToolCommissioning),
}

impl PreparedConfigWrite {
    pub(super) fn prepare(
        inner: &mut SupervisorInner,
        path: &str,
        body: &Value,
        setup_path: Option<&std::path::PathBuf>,
        require_expected_revision: bool,
    ) -> Option<Result<Self, ArmError>> {
        Some(match path {
            "/api/v1/workspace/keepout" => crate::keepout::prepare_update_with(inner, body, require_expected_revision).map(Self::Keepout),
            "/api/v1/safety/collision" => crate::collision::prepare_update_with(inner, body, require_expected_revision).map(Self::Collision),
            "/api/v1/gripper/calibration" => crate::gripper_calib::prepare_update_with(inner, body, require_expected_revision).map(|p| Self::GripperCalib(Box::new(p))),
            "/api/v1/poses/save" => crate::poses::prepare_save_with(inner, body, require_expected_revision).map(Self::PosesSave),
            "/api/v1/poses/delete" => crate::poses::prepare_delete_with(inner, body, require_expected_revision).map(Self::PosesDelete),
            "/api/v1/poses/rename" => crate::poses::prepare_rename_with(inner, body, require_expected_revision).map(Self::PosesRename),
            "/api/v1/sequences/save" => crate::sequences::prepare_save(inner, body).map(Self::Sequences),
            "/api/v1/sequences/delete" => crate::sequences::prepare_delete(inner, body).map(Self::Sequences),
            "/api/v1/grasp/setup" => crate::grasp::prepare_setup_update(inner, body, setup_path, require_expected_revision).map(|p| Self::GraspSetup(Box::new(p))),
            "/api/v1/tool/commissioning/confirm" => crate::tool::prepare_commissioning_confirm(inner, body, require_expected_revision).map(Self::ToolCommissioning),
            _ => return None,
        })
    }

    pub(super) fn path(&self) -> &'static str {
        match self {
            Self::Keepout(_) => "/api/v1/workspace/keepout",
            Self::Collision(_) => "/api/v1/safety/collision",
            Self::GripperCalib(_) => "/api/v1/gripper/calibration",
            Self::PosesSave(_) => "/api/v1/poses/save",
            Self::PosesDelete(_) => "/api/v1/poses/delete",
            Self::PosesRename(_) => "/api/v1/poses/rename",
            Self::Sequences(p) => if p.sequence.is_some() { "/api/v1/sequences/save" } else { "/api/v1/sequences/delete" },
            Self::GraspSetup(_) => "/api/v1/grasp/setup",
            Self::ToolCommissioning(_) => "/api/v1/tool/commissioning/confirm",
        }
    }

    pub(super) fn has_write(&self) -> bool {
        match self {
            Self::Keepout(p) => p.write.is_some(),
            Self::Collision(p) => p.write.is_some(),
            Self::GripperCalib(p) => p.write.is_some(),
            Self::PosesSave(p) => p.write.is_some(),
            Self::PosesDelete(p) => p.write.is_some(),
            Self::PosesRename(p) => p.write.is_some(),
            Self::Sequences(p) => p.write.is_some(),
            Self::GraspSetup(p) => p.write.is_some(),
            Self::ToolCommissioning(p) => p.write.is_some(),
        }
    }

    pub(super) fn take_write(&mut self) -> Option<crate::persist::PendingWrite> {
        match self {
            Self::Keepout(p) => p.write.take(),
            Self::Collision(p) => p.write.take(),
            Self::GripperCalib(p) => p.write.take(),
            Self::PosesSave(p) => p.write.take(),
            Self::PosesDelete(p) => p.write.take(),
            Self::PosesRename(p) => p.write.take(),
            Self::Sequences(p) => p.write.take(),
            Self::GraspSetup(p) => p.write.take(),
            Self::ToolCommissioning(p) => p.write.take(),
        }
    }

    pub(super) fn precheck(&mut self, inner: &mut SupervisorInner) -> Option<ArmError> {
        match self {
            Self::Keepout(p) => {
                if inner.keepout.revision != p.base_revision {
                    return Some(ArmError::PolicyRevisionMismatch { expected: p.base_revision, current: inner.keepout.revision });
                }
                if inner.calibration_busy {
                    return Some(ArmError::Rejected { status: 409, code: "CONFIG_BUSY".into(), message: "a camera calibration apply or restore is running".into() });
                }
                if inner.workspace_policy().is_none() {
                    return Some(crate::motion::MotionError::Unsupported(
                        "Keep-out boxes need a profile with kinematics and a workspace".into(),
                    ).into());
                }
                if let Some(reason) = inner.workspace_change_blocker() {
                    return Some(crate::motion::MotionError::MotionBusy(format!("keep-out boxes cannot change while {reason}")).into());
                }
                None
            }
            Self::Collision(p) => {
                if inner.collision.revision != p.base_revision {
                    return Some(ArmError::PolicyRevisionMismatch { expected: p.base_revision, current: inner.collision.revision });
                }
                if inner.calibration_busy {
                    return Some(ArmError::Rejected { status: 409, code: "CONFIG_BUSY".into(), message: "a camera calibration apply or restore is running".into() });
                }
                if p.enabled == Some(true) && !inner.collision_available() {
                    return Some(crate::motion::MotionError::Unsupported(
                        "The collision guard needs a profile gravity model and a driver that reports torque".into(),
                    ).into());
                }
                None
            }
            Self::GripperCalib(ref mut p) => crate::gripper_calib::precheck_update(inner, p).err(),
            Self::PosesSave(p) => crate::poses::precheck_save(inner, p).err(),
            Self::PosesDelete(p) => crate::poses::precheck_delete(inner, p).err(),
            Self::PosesRename(p) => crate::poses::precheck_save(inner, p).err(),
            Self::Sequences(p) => crate::sequences::precheck(inner, p).err(),
            Self::GraspSetup(p) => crate::grasp::precheck_setup_update(inner, p).err(),
            Self::ToolCommissioning(p) => crate::tool::precheck_commissioning_confirm(inner, p).err(),
        }
    }

    pub(super) fn commit(self, inner: &mut SupervisorInner) -> Result<Vec<(&'static str, Value)>, ArmError> {
        match self {
            Self::Keepout(p) => {
                let state = crate::keepout::commit_update(inner, p)?;
                Ok(vec![
                    ("boxes", json!(state.boxes)),
                    ("source", json!(state.source)),
                    ("revision", json!(state.revision)),
                ])
            }
            Self::Collision(p) => {
                let cfg = crate::collision::commit_update(inner, p)?;
                Ok(vec![
                    ("collision", json!(cfg)),
                    ("revision", json!(cfg.revision)),
                ])
            }
            Self::GripperCalib(p) => {
                let (policy, revision) = crate::gripper_calib::commit_update(inner, *p)?;
                Ok(vec![
                    ("policy", json!(policy)),
                    ("closed_rad", json!(policy.closed_rad)),
                    ("open_rad", json!(policy.open_rad)),
                    ("jaw_opening", json!(policy.jaw_opening)),
                    ("revision", json!(revision)),
                ])
            }
            Self::PosesSave(p) => {
                let (pose, revision) = crate::poses::commit_save(inner, p)?;
                Ok(pose_ack_fields(inner, &pose, revision))
            }
            Self::PosesDelete(p) => {
                let revision = crate::poses::commit_delete(inner, p)?;
                Ok(vec![
                    ("revision", json!(revision)),
                ])
            }
            Self::PosesRename(p) => {
                let (pose, revision) = crate::poses::commit_save(inner, p)?;
                Ok(pose_ack_fields(inner, &pose, revision))
            }
            Self::Sequences(p) => {
                let (seq, revision) = crate::sequences::commit(inner, p)?;
                let mut fields = vec![("revision", json!(revision))];
                if let Some(seq) = seq { fields.push(("sequence", json!(seq))); }
                Ok(fields)
            }
            Self::GraspSetup(p) => {
                let (setup, revision) = crate::grasp::commit_setup_update(inner, *p)?;
                Ok(vec![
                    ("setup", json!(setup)),
                    ("revision", json!(revision)),
                ])
            }
            Self::ToolCommissioning(p) => {
                let (active_tool, revision) = crate::tool::commit_commissioning_confirm(inner, p)?;
                Ok(vec![
                    ("active_tool", json!(active_tool)),
                    ("commissioning_required", json!(active_tool.commissioning_required)),
                    ("revision", json!(revision)),
                ])
            }
        }
    }
}
