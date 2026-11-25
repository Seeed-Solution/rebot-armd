//! Persist-before-commit configuration writes of the WS command path (PR C / A8).
//!
//! Both `RtOwner::Legacy` and `RtOwner::Executor` share the same off-thread prepare +
//! owner-side `ReserveConfig -> rename_only -> PersistedConfig -> ActivateConfig -> sync_parent_dir`
//! pipeline in [`crate::rt_core::requests::config`]:
//! - Preparation, JSON parsing, FK, `EvalBufs` construction, and file serialization run
//!   strictly OFF `SupervisorLock` and OFF `armd-control`, reading from [`PublishedConfig`].
//! - Authority rechecks reuse A1's `AdmissionStamp` / `ClosedConnections` / `ConsumedStamps`
//!   through `requests::config::check_config_authority`.
//! - Pre-rename failures abort staging and release the hold (`Cleared`); post-rename
//!   failures enter `Reconciling` so live state stays unchanged and motion stays inhibited.

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey,
    FrameKind, LeaseRequirement, OwnerDispatchCtx, OwnerReply, ReplyPolicy,
};
use crate::model::{HolderLabel, KeepoutBox};
use crate::motion::MotionError;
use crate::persist::PendingWrite;
use crate::rt_core::config_iface::ConfigDomain;
use crate::rt_core::requests::config::{
    gravity_comp_command, prepare_eval_bufs_off_thread, run_config_commit, ConfigAuthContext,
    ConfigCandidate, ConfigCommitPlan, ConfigReply, ConfigRequest, PublishedConfig,
    ReservePrecheck,
};
use crate::supervisor::AdmissionStamp;

macro_rules! config_entry {
    ($path:expr) => {
        DispatchEntry {
            key: DispatchKey::command($path),
            handler: DispatchHandler::Owner(dispatch_config_entry),
            admission: AdmissionClass::Ordinary,
            lease: LeaseRequirement::Required,
            auth: AuthRequirement::Operator(None),
            reply: ReplyPolicy::CommandDeferred,
        }
    };
}

pub(super) const ENTRIES: &[DispatchEntry] = &[
    config_entry!("/api/v1/workspace/keepout"),
    config_entry!("/api/v1/safety/collision"),
    config_entry!("/api/v1/gripper/calibration"),
    config_entry!("/api/v1/poses/save"),
    config_entry!("/api/v1/poses/delete"),
    config_entry!("/api/v1/poses/rename"),
    config_entry!("/api/v1/sequences/save"),
    config_entry!("/api/v1/sequences/delete"),
    config_entry!("/api/v1/grasp/setup"),
    config_entry!("/api/v1/tool/commissioning/confirm"),
];

/// Returns true if `path` is one of the WS config write endpoints in [`ENTRIES`].
pub(super) fn is_config_path(path: &str) -> bool {
    ENTRIES.iter().any(|e| e.key.path == Some(path))
}

/// Deferred config job captured under the short legacy ingress check (zero JSON/FK/IO under lock).
pub(super) struct PendingConfigJob {
    pub(super) request_id: Value,
    pub(super) path: &'static str,
    pub(super) body: Value,
    pub(super) generation: u64,
    pub(super) epoch: u64,
    pub(super) stamp: AdmissionStamp,
    pub(super) require_expected_revision: bool,
}

fn dispatch_config_entry(ctx: &mut OwnerDispatchCtx<'_>) -> Option<OwnerReply> {
    let static_path = ENTRIES
        .iter()
        .find_map(|e| e.key.path.filter(|&p| p == ctx.path))
        .expect("registered config path");
    *ctx.pending_config_write = Some(PendingConfigJob {
        request_id: ctx.value["request_id"].clone(),
        path: static_path,
        body: ctx.value["body"].clone(),
        generation: ctx.local_generation,
        epoch: ctx.epoch,
        stamp: ctx.stamp,
        require_expected_revision: ctx.ws_auth.is_enforced(),
    });
    None
}

/// Domain-specific off-thread payload and metadata prepared from [`PublishedConfig`].
#[allow(clippy::large_enum_variant)]
enum PreparedPayload {
    Keepout {
        profile_boxes: Vec<KeepoutBox>,
        runtime_boxes: Vec<KeepoutBox>,
        effective_boxes: Vec<KeepoutBox>,
        ws_eval_bufs: crate::chain::EvalBufs,
    },
    Collision {
        enabled: Option<bool>,
        sensitivity: Option<crate::collision::Sensitivity>,
    },
    GripperCalib {
        offset_rad: Option<f32>,
        jaw_opening: crate::model::JawOpening,
        effective_policy: crate::model::GripperPolicy,
    },
    PosesSave {
        pose: crate::poses::NamedPose,
        next_list: crate::poses::PoseList,
    },
    PosesDelete {
        pose_name: String,
        pose_id: String,
        next_list: crate::poses::PoseList,
    },
    Sequences {
        sequence: Option<crate::sequences::Sequence>,
        deleting_id: Option<String>,
        next_file: crate::sequences::SequenceFile,
    },
    GraspSetup {
        setup: crate::grasp::GraspSetup,
        ws_eval_bufs: Option<crate::chain::EvalBufs>,
    },
    ToolCommissioning {
        tool_id: String,
        tool_config_digest: String,
        confirmed_at: String,
    },
}

struct PreparedOffThreadConfig {
    path: &'static str,
    domain: ConfigDomain,
    base_revision: u64,
    workspace_revision: u64,
    write: Option<PendingWrite>,
    payload: PreparedPayload,
}

impl PreparedOffThreadConfig {
    fn has_write(&self) -> bool {
        self.write.is_some()
    }
}

pub(super) fn off_lock_command_reject(request_id: &Value, epoch: u64, error: ApiError) -> Value {
    let mut reply = rejection(error, "command_reject", epoch);
    reply["request_id"] = request_id.clone();
    reply
}

fn check_positions_off_thread(q: &[f32], config: &crate::model::ArmConfig) -> Result<(), ArmError> {
    if q.len() != config.dof {
        return Err(MotionError::InvalidCommand(format!("positions must have {} entries", config.dof)).into());
    }
    for (v, l) in q.iter().zip(&config.limits) {
        if !v.is_finite() || *v < l.min_position || *v > l.max_position {
            return Err(MotionError::InvalidCommand(format!(
                "joint '{}' position {v} outside [{}, {}]",
                l.name, l.min_position, l.max_position
            ))
            .into());
        }
    }
    Ok(())
}

fn pose_name_of(body: &Value) -> Result<String, ArmError> {
    let name = body
        .get("name")
        .and_then(Value::as_str)
        .ok_or_else(|| MotionError::InvalidCommand("name must be a string".into()))?;
    if !crate::poses::valid_name(name) {
        return Err(MotionError::InvalidCommand("name must match ^[A-Za-z0-9_-]{1,64}$".into()).into());
    }
    Ok(name.to_string())
}

fn check_pose_revision_pub(
    pub_cfg: &PublishedConfig,
    body: &Value,
    require_expected_revision: bool,
) -> Result<u64, ArmError> {
    if !body.is_object() {
        return Err(MotionError::InvalidCommand("Command body must be an object".into()).into());
    }
    if let Some(exp) = crate::persist::parse_expected_revision(body, require_expected_revision)
        .map_err(MotionError::InvalidCommand)?
    {
        if exp != pub_cfg.poses.revision {
            return Err(ArmError::PolicyRevisionMismatch {
                expected: exp,
                current: pub_cfg.poses.revision,
            });
        }
    }
    Ok(pub_cfg.poses.revision)
}

fn check_sequence_steps_pub(pub_cfg: &PublishedConfig, steps: &Value) -> Result<Vec<crate::sequences::Step>, ArmError> {
    use crate::sequences::{GripperAction, Step, MAX_STEPS, MAX_WAIT_S};
    let invalid = |msg: String| ArmError::Rejected {
        status: 400,
        code: "INVALID_COMMAND".into(),
        message: msg,
    };
    let arr = steps
        .as_array()
        .ok_or_else(|| invalid("steps must be an array".into()))?;
    if arr.len() > MAX_STEPS {
        return Err(invalid(format!("at most {MAX_STEPS} steps")));
    }
    let mut out = Vec::with_capacity(arr.len());
    for (i, v) in arr.iter().enumerate() {
        let step: Step = serde_json::from_value(v.clone()).map_err(|e| {
            ArmError::detailed(400, "INVALID_COMMAND", format!("step {i}: {e}"), json!({"step": i}))
        })?;
        let bad = |m: String| ArmError::detailed(400, "INVALID_COMMAND", format!("step {i}: {m}"), json!({"step": i}));
        let missing = |m: String| {
            ArmError::detailed(422, "SEQUENCE_REF_MISSING", format!("step {i}: {m}"), json!({"step": i}))
        };
        match &step {
            Step::Pose { pose_id, speed_scale, .. } => {
                if !(crate::poses::MIN_SPEED_SCALE..=1.0).contains(speed_scale) {
                    return Err(bad(format!(
                        "speed_scale must be in [{}, 1.0]",
                        crate::poses::MIN_SPEED_SCALE
                    )));
                }
                if pub_cfg.poses.by_id(pose_id).is_none() {
                    return Err(missing(format!("no pose with id '{pose_id}'")));
                }
            }
            Step::Gripper { action, fraction } => {
                if pub_cfg.config.gripper_index().is_none() || pub_cfg.config.gripper_policy.is_none() {
                    return Err(bad("this arm has no gripper policy".into()));
                }
                match (action, fraction) {
                    (GripperAction::Move, Some(f)) if (0.0..=1.0).contains(f) => {}
                    (GripperAction::Move, _) => return Err(bad("move needs fraction in [0, 1]".into())),
                    (_, Some(_)) => return Err(bad("fraction is only for move".into())),
                    _ => {}
                }
            }
            Step::Wait { s } => {
                if !s.is_finite() || !(0.0..=MAX_WAIT_S).contains(s) {
                    return Err(bad(format!("s must be in [0, {MAX_WAIT_S}]")));
                }
            }
            Step::Replay { episode_id, speed, .. } => {
                if !crate::replay::REPLAY_SPEEDS.iter().any(|x| (x - speed).abs() < 1e-9) {
                    return Err(bad(format!("speed must be one of {:?}", crate::replay::REPLAY_SPEEDS)));
                }
                if !pub_cfg.episodes.contains(episode_id) {
                    return Err(missing(format!("no episode '{episode_id}'")));
                }
            }
        }
        out.push(step);
    }
    Ok(out)
}

/// Off-lock/off-RT preparation of every WS config write from [`PublishedConfig`].
async fn prepare_config_write_off_thread(
    sup: &Arc<ArmSupervisor>,
    job: &PendingConfigJob,
    setup_path: Option<&std::path::PathBuf>,
) -> Result<PreparedOffThreadConfig, ArmError> {
    let pub_cfg = sup.published_config().await?;
    let body = &job.body;
    let req_rev = job.require_expected_revision;
    let ws_rev = pub_cfg.workspace_revision;

    match job.path {
        "/api/v1/workspace/keepout" => {
            let list: crate::keepout::KeepoutList = serde_json::from_value(body.clone()).map_err(|e| {
                MotionError::InvalidCommand(format!(
                    "keep-out body must be {{\"boxes\": [{{name, min_m, max_m}}]}}: {e}"
                ))
            })?;
            if req_rev && list.expected_revision.is_none() {
                return Err(MotionError::InvalidCommand(
                    "expected_revision is required when operator auth is enforced".into(),
                )
                .into());
            }
            if let Some(exp) = list.expected_revision {
                if exp != pub_cfg.keepout.revision {
                    return Err(ArmError::PolicyRevisionMismatch {
                        expected: exp,
                        current: pub_cfg.keepout.revision,
                    });
                }
            }
            let Some((ws_pol, cam)) = pub_cfg.workspace_policy() else {
                return Err(MotionError::Unsupported(
                    "Keep-out boxes need a profile with kinematics and a workspace".into(),
                )
                .into());
            };
            KeepoutBox::validate_list(&list.boxes).map_err(MotionError::InvalidCommand)?;
            let profile_boxes = pub_cfg.profile_keepout();
            let effective_boxes: Vec<KeepoutBox> =
                profile_boxes.iter().chain(&list.boxes).cloned().collect();
            KeepoutBox::validate_list(&effective_boxes).map_err(|e| {
                MotionError::InvalidCommand(format!("{e} (profile boxes included: {})", profile_boxes.len()))
            })?;
            let mut next_ws = ws_pol.clone();
            next_ws.keepout_boxes = effective_boxes.clone();
            let ws_eval_bufs = prepare_eval_bufs_off_thread(
                &pub_cfg.config.arm_model,
                &next_ws,
                cam,
                pub_cfg.config.active_tool.tcp_offset_m,
            );
            let write = pub_cfg.keepout.path.clone().map(|path| {
                PendingWrite::new(
                    path,
                    serde_json::to_string_pretty(&crate::keepout::KeepoutList {
                        boxes: list.boxes.clone(),
                        expected_revision: None,
                    })
                    .unwrap_or_default(),
                )
            });
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::Keepout,
                base_revision: pub_cfg.keepout.revision,
                workspace_revision: ws_rev,
                write,
                payload: PreparedPayload::Keepout {
                    profile_boxes,
                    runtime_boxes: list.boxes,
                    effective_boxes,
                    ws_eval_bufs,
                },
            })
        }
        "/api/v1/safety/collision" => {
            let b: crate::collision::Body = serde_json::from_value(body.clone()).map_err(|e| {
                MotionError::InvalidCommand(format!(
                    "collision body must be {{\"enabled\": bool?, \"sensitivity\": \"low\"|\"medium\"|\"high\"?}}: {e}"
                ))
            })?;
            if req_rev && b.expected_revision.is_none() {
                return Err(MotionError::InvalidCommand(
                    "expected_revision is required when operator auth is enforced".into(),
                )
                .into());
            }
            if let Some(exp) = b.expected_revision {
                if exp != pub_cfg.collision_revision {
                    return Err(ArmError::PolicyRevisionMismatch {
                        expected: exp,
                        current: pub_cfg.collision_revision,
                    });
                }
            }
            if b.enabled.is_none() && b.sensitivity.is_none() {
                return Err(MotionError::InvalidCommand(
                    "collision body needs enabled and/or sensitivity".into(),
                )
                .into());
            }
            if b.enabled == Some(true) && !pub_cfg.collision_available() {
                return Err(MotionError::Unsupported(
                    "The collision guard needs a profile gravity model and a driver that reports torque".into(),
                )
                .into());
            }
            let setting = crate::collision::CollisionSetting {
                enabled: b.enabled.unwrap_or(pub_cfg.collision_enabled),
                sensitivity: b.sensitivity.unwrap_or(pub_cfg.collision_sensitivity),
            };
            let write = pub_cfg.collision_path.clone().map(|path| {
                PendingWrite::new(path, serde_json::to_string_pretty(&setting).unwrap_or_default())
            });
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::Collision,
                base_revision: pub_cfg.collision_revision,
                workspace_revision: ws_rev,
                write,
                payload: PreparedPayload::Collision {
                    enabled: b.enabled,
                    sensitivity: b.sensitivity,
                },
            })
        }
        "/api/v1/gripper/calibration" => {
            let b: crate::gripper_calib::CalibBody = serde_json::from_value(body.clone()).map_err(|e| {
                MotionError::InvalidCommand(format!(
                    "gripper calibration body must have jaw_opening and optional offset_rad: {e}"
                ))
            })?;
            if req_rev && b.expected_revision.is_none() {
                return Err(MotionError::InvalidCommand(
                    "expected_revision is required when operator auth is enforced".into(),
                )
                .into());
            }
            if let Some(exp) = b.expected_revision {
                if exp != pub_cfg.gripper_calib_revision {
                    return Err(ArmError::PolicyRevisionMismatch {
                        expected: exp,
                        current: pub_cfg.gripper_calib_revision,
                    });
                }
            }
            if let Some(off) = b.offset_rad {
                if !off.is_finite() || off.abs() > 0.5 {
                    return Err(MotionError::InvalidCommand(format!(
                        "offset_rad {off} must be in [-0.5, 0.5] rad"
                    ))
                    .into());
                }
            }
            let j7_limit = pub_cfg
                .config
                .gripper_index()
                .and_then(|idx| pub_cfg.config.limits.get(idx));
            b.jaw_opening
                .validate(j7_limit)
                .map_err(MotionError::InvalidCommand)?;
            if pub_cfg.config.gripper_policy.is_none() {
                return Err(MotionError::Unsupported(
                    "Gripper calibration requires a gripper policy".into(),
                )
                .into());
            }
            let offset = b.offset_rad.or(pub_cfg.gripper_calib_offset);
            let base_policy = pub_cfg
                .base_gripper_policy
                .as_ref()
                .or(pub_cfg.config.gripper_policy.as_ref());
            let effective_policy = SupervisorInner::validate_gripper_calibration_off_thread(
                base_policy,
                pub_cfg.gripper_calib_jaw_opening.as_ref(),
                j7_limit,
                pub_cfg.config.active_tool.is_stock,
                offset,
                Some(&b.jaw_opening),
            )
            .map_err(MotionError::InvalidCommand)?;
            let file = crate::gripper_calib::GripperCalibFile {
                offset_rad: offset,
                jaw_opening: Some(b.jaw_opening.clone()),
                ts: Some(
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .map_or(0, |d| d.as_secs()),
                ),
                stale: None,
                tool_id: Some(pub_cfg.config.active_tool.id.clone()),
                tool_config_digest: Some(pub_cfg.config.active_tool.config_digest.clone()),
            };
            let write = pub_cfg
                .gripper_calib_path
                .clone()
                .or_else(|| Some(crate::gripper_calib::path_from_env()))
                .map(|path| PendingWrite::new(path, serde_json::to_string_pretty(&file).unwrap_or_default()));
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::GripperCalibration,
                base_revision: pub_cfg.gripper_calib_revision,
                workspace_revision: ws_rev,
                write,
                payload: PreparedPayload::GripperCalib {
                    offset_rad: offset,
                    jaw_opening: b.jaw_opening,
                    effective_policy,
                },
            })
        }
        "/api/v1/poses/save" => {
            let base_revision = check_pose_revision_pub(&pub_cfg, body, req_rev)?;
            let name = pose_name_of(body)?;
            let by_id = match body.get("id") {
                None | Some(Value::Null) => None,
                Some(Value::String(id)) => {
                    if pub_cfg.poses.by_id(id).is_none() {
                        return Err(ArmError::Rejected {
                            status: 404,
                            code: "POSE_NOT_FOUND".into(),
                            message: format!("No pose with id '{id}'"),
                        });
                    }
                    Some(id.clone())
                }
                Some(_) => return Err(MotionError::InvalidCommand("id must be a string".into()).into()),
            };
            if let Some(id) = &by_id {
                if pub_cfg.poses.list.poses.iter().any(|p| p.name == name && &p.id != id) {
                    return Err(ArmError::Rejected {
                        status: 409,
                        code: "POSE_EXISTS".into(),
                        message: format!("Another pose is named '{name}'"),
                    });
                }
            }
            let positions = match body.get("positions") {
                None | Some(Value::Null) => sup
                    .hub()
                    .latest()
                    .map(|s| s.obs.positions.clone())
                    .filter(|p| !p.is_empty())
                    .unwrap_or_else(|| pub_cfg.fallback_measured_positions.clone()),
                Some(v) => v
                    .as_array()
                    .ok_or_else(|| MotionError::InvalidCommand("positions must be an array".into()))?
                    .iter()
                    .map(|x| {
                        x.as_f64()
                            .filter(|f| f.is_finite())
                            .map(|f| f as f32)
                            .ok_or_else(|| {
                                ArmError::from(MotionError::InvalidCommand(
                                    "positions must be finite numbers".into(),
                                ))
                            })
                    })
                    .collect::<Result<Vec<_>, _>>()?,
            };
            check_positions_off_thread(&positions, &pub_cfg.config)?;
            let list = &pub_cfg.poses.list.poses;
            if !list.iter().any(|p| p.name == name) && list.len() >= crate::poses::MAX_POSES {
                return Err(MotionError::InvalidCommand(format!(
                    "at most {} poses; delete one first",
                    crate::poses::MAX_POSES
                ))
                .into());
            }
            let saved_at_ms = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_millis() as u64);
            let profile_id = pub_cfg.config.profile_id.clone();
            let arm_dof = pub_cfg.config.arm_dof();
            let gripper_joint = pub_cfg.config.gripper_index();
            let joint_names = pub_cfg.joint_names();
            let id = match &by_id {
                Some(id) => id.clone(),
                None => pub_cfg
                    .poses
                    .list
                    .poses
                    .iter()
                    .find(|p| p.name == name)
                    .map(|p| p.id.clone())
                    .filter(|id| !id.is_empty())
                    .unwrap_or_else(crate::recording::new_ulid),
            };
            let pose = crate::poses::NamedPose {
                tcp: pub_cfg.pose_tcp(&positions),
                id,
                name,
                positions,
                saved_at_ms,
                profile_id: profile_id.clone(),
                arm_dof: Some(arm_dof),
                gripper_joint,
                joint_names: Some(joint_names.clone()),
                tool_id: Some(pub_cfg.config.active_tool.id.clone()),
                tool_config_digest: Some(pub_cfg.config.active_tool.config_digest.clone()),
                stale: None,
            };
            let mut next_list = pub_cfg.poses.list.clone();
            next_list.profile_id = profile_id;
            next_list.arm_dof = Some(arm_dof);
            next_list.gripper_joint = gripper_joint;
            next_list.joint_names = Some(joint_names);
            match next_list.poses.iter_mut().find(|p| p.id == pose.id) {
                Some(p) => *p = pose.clone(),
                None => next_list.poses.push(pose.clone()),
            }
            let write = pub_cfg.poses.path.clone().map(|path| {
                PendingWrite::new(path, serde_json::to_string_pretty(&next_list).unwrap_or_default())
            });
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::Poses,
                base_revision,
                workspace_revision: ws_rev,
                write,
                payload: PreparedPayload::PosesSave { pose, next_list },
            })
        }
        "/api/v1/poses/rename" => {
            let base_revision = check_pose_revision_pub(&pub_cfg, body, req_rev)?;
            let name = pose_name_of(body)?;
            let id = body
                .get("id")
                .and_then(Value::as_str)
                .ok_or_else(|| MotionError::InvalidCommand("id must be a string".into()))?;
            if pub_cfg.poses.list.poses.iter().any(|p| p.name == name && p.id != id) {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "POSE_EXISTS".into(),
                    message: format!("Another pose is named '{name}'"),
                });
            }
            let mut next_list = pub_cfg.poses.list.clone();
            let pose = next_list
                .poses
                .iter_mut()
                .find(|p| p.id == id)
                .ok_or_else(|| ArmError::Rejected {
                    status: 404,
                    code: "POSE_NOT_FOUND".into(),
                    message: format!("No pose with id '{id}'"),
                })?;
            pose.name = name;
            let pose = pose.clone();
            let write = pub_cfg.poses.path.clone().map(|path| {
                PendingWrite::new(path, serde_json::to_string_pretty(&next_list).unwrap_or_default())
            });
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::Poses,
                base_revision,
                workspace_revision: ws_rev,
                write,
                payload: PreparedPayload::PosesSave { pose, next_list },
            })
        }
        "/api/v1/poses/delete" => {
            let base_revision = check_pose_revision_pub(&pub_cfg, body, req_rev)?;
            let name = pose_name_of(body)?;
            let mut next_list = pub_cfg.poses.list.clone();
            let i = next_list
                .poses
                .iter()
                .position(|p| p.name == name)
                .ok_or_else(|| crate::poses::not_found(&name))?;
            let pose_id = next_list.poses[i].id.clone();
            let users = pub_cfg.sequences.using_pose(&pose_id);
            if !users.is_empty() {
                return Err(ArmError::detailed(
                    409,
                    "IN_USE",
                    format!("Pose '{name}' is used by {} sequence(s)", users.len()),
                    json!({"sequences": users}),
                ));
            }
            next_list.poses.remove(i);
            let write = pub_cfg.poses.path.clone().map(|path| {
                PendingWrite::new(path, serde_json::to_string_pretty(&next_list).unwrap_or_default())
            });
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::Poses,
                base_revision,
                workspace_revision: ws_rev,
                write,
                payload: PreparedPayload::PosesDelete {
                    pose_name: name,
                    pose_id,
                    next_list,
                },
            })
        }
        "/api/v1/sequences/save" => {
            if !body.is_object() {
                return Err(ArmError::Rejected {
                    status: 400,
                    code: "INVALID_COMMAND".into(),
                    message: "Command body must be an object".into(),
                });
            }
            if let Some(exp) = crate::persist::parse_expected_revision(body, false).map_err(|m| {
                ArmError::Rejected {
                    status: 400,
                    code: "INVALID_COMMAND".into(),
                    message: m,
                }
            })? {
                if exp != pub_cfg.sequences.revision {
                    return Err(ArmError::PolicyRevisionMismatch {
                        expected: exp,
                        current: pub_cfg.sequences.revision,
                    });
                }
            }
            let invalid = |msg: String| ArmError::Rejected {
                status: 400,
                code: "INVALID_COMMAND".into(),
                message: msg,
            };
            let name = body
                .get("name")
                .and_then(Value::as_str)
                .ok_or_else(|| invalid("name must be a string".into()))?
                .to_string();
            if !crate::recording::store::valid_display_name(&name) {
                return Err(invalid(
                    "name must be 1-64 characters from letters, digits, _ - space and Han characters".into(),
                ));
            }
            let description = match body.get("description") {
                None | Some(Value::Null) => String::new(),
                Some(Value::String(d)) if d.chars().count() <= 1024 => d.clone(),
                Some(_) => return Err(invalid("description must be a string of at most 1024 characters".into())),
            };
            let id = match body.get("id") {
                None | Some(Value::Null) => None,
                Some(Value::String(id)) => Some(id.clone()),
                Some(_) => return Err(invalid("id must be a string".into())),
            };
            let rev = match body.get("rev") {
                None | Some(Value::Null) => None,
                Some(v) => Some(
                    v.as_u64()
                        .ok_or_else(|| invalid("rev must be a non-negative integer".into()))?,
                ),
            };
            let steps = check_sequence_steps_pub(&pub_cfg, body.get("steps").unwrap_or(&Value::Null))?;
            if let Some(id) = &id {
                let current = pub_cfg.sequences.get(id).ok_or_else(|| ArmError::Rejected {
                    status: 404,
                    code: "SEQUENCE_NOT_FOUND".into(),
                    message: format!("No sequence with id '{id}'"),
                })?;
                if rev.is_some_and(|r| r != current.rev) {
                    return Err(ArmError::detailed(
                        409,
                        "SEQUENCE_CONFLICT",
                        format!("sequence '{}' is at rev {}, not {}", current.name, current.rev, rev.unwrap_or(0)),
                        json!({"rev": current.rev}),
                    ));
                }
            }
            if pub_cfg
                .sequences
                .file
                .sequences
                .iter()
                .any(|s| s.name == name && Some(&s.id) != id.as_ref())
            {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "SEQUENCE_EXISTS".into(),
                    message: format!("Another sequence is named '{name}'"),
                });
            }
            let now = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
            let mut next_file = pub_cfg.sequences.file.clone();
            let seq = match id {
                Some(id) => {
                    let s = next_file
                        .sequences
                        .iter_mut()
                        .find(|s| s.id == id)
                        .expect("checked above");
                    s.name = name;
                    s.description = description;
                    s.steps = steps;
                    s.updated_at = now;
                    s.rev += 1;
                    s.clone()
                }
                None => {
                    if next_file.sequences.len() >= crate::sequences::MAX_SEQUENCES {
                        return Err(invalid(format!(
                            "at most {} sequences; delete one first",
                            crate::sequences::MAX_SEQUENCES
                        )));
                    }
                    let s = crate::sequences::Sequence {
                        id: crate::recording::new_ulid(),
                        name,
                        description,
                        steps,
                        created_at: now.clone(),
                        updated_at: now,
                        rev: 1,
                    };
                    next_file.sequences.push(s.clone());
                    s
                }
            };
            let write = pub_cfg.sequences.path.clone().map(|path| {
                PendingWrite::new(path, serde_json::to_string_pretty(&next_file).unwrap_or_default())
            });
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::Sequences,
                base_revision: pub_cfg.sequences.revision,
                workspace_revision: ws_rev,
                write,
                payload: PreparedPayload::Sequences {
                    sequence: Some(seq),
                    deleting_id: None,
                    next_file,
                },
            })
        }
        "/api/v1/sequences/delete" => {
            if !body.is_object() {
                return Err(ArmError::Rejected {
                    status: 400,
                    code: "INVALID_COMMAND".into(),
                    message: "Command body must be an object".into(),
                });
            }
            if let Some(exp) = crate::persist::parse_expected_revision(body, false).map_err(|m| {
                ArmError::Rejected {
                    status: 400,
                    code: "INVALID_COMMAND".into(),
                    message: m,
                }
            })? {
                if exp != pub_cfg.sequences.revision {
                    return Err(ArmError::PolicyRevisionMismatch {
                        expected: exp,
                        current: pub_cfg.sequences.revision,
                    });
                }
            }
            let id = body.get("id").and_then(Value::as_str).ok_or_else(|| ArmError::Rejected {
                status: 400,
                code: "INVALID_COMMAND".into(),
                message: "id must be a string".into(),
            })?;
            if pub_cfg.is_sequence_running(id) {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "SEQUENCE_RUNNING".into(),
                    message: "the sequence is running; stop it first".into(),
                });
            }
            let mut next_file = pub_cfg.sequences.file.clone();
            let i = next_file
                .sequences
                .iter()
                .position(|s| s.id == id)
                .ok_or_else(|| ArmError::Rejected {
                    status: 404,
                    code: "SEQUENCE_NOT_FOUND".into(),
                    message: format!("No sequence with id '{id}'"),
                })?;
            next_file.sequences.remove(i);
            let write = pub_cfg.sequences.path.clone().map(|path| {
                PendingWrite::new(path, serde_json::to_string_pretty(&next_file).unwrap_or_default())
            });
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::Sequences,
                base_revision: pub_cfg.sequences.revision,
                workspace_revision: ws_rev,
                write,
                payload: PreparedPayload::Sequences {
                    sequence: None,
                    deleting_id: Some(id.to_string()),
                    next_file,
                },
            })
        }
        "/api/v1/grasp/setup" => {
            let Some(base) = pub_cfg.grasp_setup() else {
                return Err(MotionError::Unsupported("This profile has no grasp setup".into()).into());
            };
            let Some(obj) = body.as_object() else {
                return Err(MotionError::InvalidCommand("grasp setup must be a JSON object".into()).into());
            };
            if let Some(exp) = crate::persist::parse_expected_revision(body, req_rev)
                .map_err(MotionError::InvalidCommand)?
            {
                if exp != pub_cfg.grasp_setup_revision {
                    return Err(ArmError::PolicyRevisionMismatch {
                        expected: exp,
                        current: pub_cfg.grasp_setup_revision,
                    });
                }
            }
            let mut patch = obj.clone();
            patch.remove("expected_revision");
            match patch.remove("table_z_from_tcp") {
                None | Some(Value::Null) | Some(Value::Bool(false)) => {}
                Some(Value::Bool(true)) => {
                    if pub_cfg.config.kinematics.is_none() {
                        return Err(MotionError::Unsupported("table_z_from_tcp requires kinematics".into()).into());
                    }
                    let (pos_arr, pos_len) = match sup
                        .dispatch_config_request(ConfigRequest::SampleTableCapture)
                        .await
                    {
                        Ok(ConfigReply::TableCaptureSampled(Ok(v))) => v,
                        Ok(ConfigReply::TableCaptureSampled(Err(e))) => return Err(e.to_arm_error()),
                        Err(e) => return Err(e.to_arm_error()),
                        _ => return Err(ArmError::Internal("unexpected table capture reply".into())),
                    };
                    let measured = &pos_arr[..pos_len];
                    let Some(end_pose) = pub_cfg.config.arm_model.fk_f32(measured) else {
                        return Err(MotionError::InvalidCommand(
                            "measured positions are not a valid joint vector".into(),
                        )
                        .into());
                    };
                    let contact_end = pub_cfg.config.effective_physical_contact_end_m().ok_or_else(|| {
                        MotionError::InvalidCommand(
                            "table_z_from_tcp requires a known physical_contact_end_m for the active tool".into(),
                        )
                    })?;
                    let tool_x = crate::kinematics::column(&end_pose.r, 0);
                    let tilt_deg = (-tool_x[2]).clamp(-1.0, 1.0).acos().to_degrees();
                    if tilt_deg > 5.0 {
                        return Err(MotionError::InvalidCommand(format!(
                            "table_z_from_tcp requires the tool within 5 deg of vertical (current tilt {tilt_deg:.1} deg)"
                        ))
                        .into());
                    }
                    if let (Some(idx), Some(policy)) = (
                        pub_cfg.config.gripper_index(),
                        pub_cfg.config.gripper_policy.as_ref(),
                    ) {
                        let j7 = measured.get(idx).copied().ok_or_else(|| {
                            MotionError::InvalidCommand("measured gripper position unavailable".into())
                        })?;
                        let span = (policy.open_rad - policy.closed_rad).abs();
                        let frac = if span > 1e-6 {
                            ((j7 - policy.closed_rad) / (policy.open_rad - policy.closed_rad)).clamp(0.0, 1.0)
                        } else {
                            0.0
                        };
                        if frac > 0.05 && (j7 - policy.closed_rad).abs() > 0.20 {
                            return Err(MotionError::InvalidCommand(format!(
                                "table_z_from_tcp requires the gripper closed (measured {j7:.3} rad, closed {:.3} rad)",
                                policy.closed_rad
                            ))
                            .into());
                        }
                    }
                    let contact_world = end_pose.transform_point(&contact_end);
                    let z = contact_world[2];
                    patch.insert("table_z_m".into(), json!((z * 1e4).round() / 1e4));
                }
                Some(_) => {
                    return Err(MotionError::InvalidCommand("table_z_from_tcp must be boolean".into()).into());
                }
            }
            if pub_cfg.camera_calibrated() {
                crate::grasp::refuse_calibrated_fields(&base, &patch, "")?;
            }
            let width_patched = patch.contains_key("max_grasp_width_m");
            let setup = base
                .patched(&Value::Object(patch))
                .map_err(MotionError::InvalidCommand)?;
            match pub_cfg.grasp_width_check(setup.max_grasp_width_m) {
                Some(crate::model::GraspWidthCheck::Clamped { curve_max_m }) if width_patched => {
                    return Err(MotionError::InvalidCommand(format!(
                        "max_grasp_width_m {:.4} m exceeds the jaw curve's open width {curve_max_m:.4} m",
                        setup.max_grasp_width_m
                    ))
                    .into());
                }
                Some(crate::model::GraspWidthCheck::Unverified) => tracing::warn!(
                    "Grasp setup max_grasp_width_m {:.4} m is unverified: no gripper jaw_opening calibration curve",
                    setup.max_grasp_width_m
                ),
                Some(crate::model::GraspWidthCheck::Clamped { curve_max_m }) => tracing::warn!(
                    "Grasp setup max_grasp_width_m {:.4} m exceeds the jaw curve's open width {curve_max_m:.4} m; grasps use {curve_max_m:.4} m",
                    setup.max_grasp_width_m
                ),
                _ => {}
            }
            let ws_eval_bufs = pub_cfg.workspace_policy().map(|(ws, cam)| {
                let mut next_cam = cam.clone();
                next_cam.tilt_deg = setup.camera_tilt_deg as f32;
                prepare_eval_bufs_off_thread(
                    &pub_cfg.config.arm_model,
                    ws,
                    &next_cam,
                    pub_cfg.config.active_tool.tcp_offset_m,
                )
            });
            let write = setup_path.cloned().map(|p| {
                PendingWrite::new(
                    p,
                    crate::grasp::serialize_setup_for_tool(&setup, &pub_cfg.config.active_tool),
                )
            });
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::GraspSetup,
                base_revision: pub_cfg.grasp_setup_revision,
                workspace_revision: ws_rev,
                write,
                payload: PreparedPayload::GraspSetup { setup, ws_eval_bufs },
            })
        }
        "/api/v1/tool/commissioning/confirm" => {
            let prep = crate::tool::prepare_commissioning_confirm_off_thread(&pub_cfg, body, req_rev).await?;
            Ok(PreparedOffThreadConfig {
                path: job.path,
                domain: ConfigDomain::ToolCommissioning,
                base_revision: prep.base_revision,
                workspace_revision: ws_rev,
                write: prep.write,
                payload: PreparedPayload::ToolCommissioning {
                    tool_id: prep.tool_id,
                    tool_config_digest: prep.tool_config_digest,
                    confirmed_at: prep.confirmed_at,
                },
            })
        }
        _ => Err(ArmError::Internal(format!("unknown config path: {}", job.path))),
    }
}

/// Runs the unified off-thread two-phase config commit (`run_config_commit`) for a prepared WS write
/// and formats the exact wire `command_ack` or `command_reject`.
#[allow(clippy::too_many_arguments)]
async fn commit_prepared_config(
    sup: Arc<ArmSupervisor>,
    request_id: Value,
    prepared: PreparedOffThreadConfig,
    generation: u64,
    epoch: u64,
    mut stamp: AdmissionStamp,
    conn_id: u64,
    ws_auth: WsAuth,
    conn: String,
    gen_now: Arc<AtomicU64>,
) -> Value {
    stamp.connection_id = conn_id;
    stamp.lease_gen = generation;
    stamp.estop_epoch = epoch;

    let path = prepared.path;
    let domain = prepared.domain;
    let base_revision = prepared.base_revision;
    let ws_rev = prepared.workspace_revision;
    let write = prepared.write;

    let ingress_deadline = std::time::Instant::now() + crate::supervisor::config::CONFIG_PROPOSAL_BUDGET;
    let write_op = crate::rt_core::requests::config::ConfigWriteOp::from(write);
    let proposal = crate::rt_core::requests::config::build_config_proposal_with_deadline(
        &sup,
        domain,
        base_revision,
        stamp,
        generation,
        ws_rev,
        ingress_deadline,
    );
    let auth = ConfigAuthContext {
        admission: stamp,
        lease_holder: Some((HolderLabel::new(&conn), generation)),
    };

    // Clone only the lightweight pieces needed for the ACK and PublishedConfig update.
    enum AckMeta {
        Keepout {
            profile_boxes: Vec<KeepoutBox>,
            runtime_boxes: Vec<KeepoutBox>,
            effective_boxes: Vec<KeepoutBox>,
        },
        Collision {
            enabled: Option<bool>,
            sensitivity: Option<crate::collision::Sensitivity>,
        },
        GripperCalib {
            offset_rad: Option<f32>,
            jaw_opening: crate::model::JawOpening,
            policy: crate::model::GripperPolicy,
        },
        PosesSave {
            pose: crate::poses::NamedPose,
            next_list: crate::poses::PoseList,
        },
        PosesDelete {
            pose_name: String,
            pose_id: String,
            next_list: crate::poses::PoseList,
        },
        Sequences {
            sequence: Option<crate::sequences::Sequence>,
            next_file: crate::sequences::SequenceFile,
        },
        GraspSetup {
            setup: crate::grasp::GraspSetup,
        },
        ToolCommissioning {
            confirmed_at: String,
        },
    }

    let (precheck, candidate, meta) = match prepared.payload {
        PreparedPayload::Keepout {
            profile_boxes,
            runtime_boxes,
            effective_boxes,
            ws_eval_bufs,
        } => (
            ReservePrecheck::Keepout,
            ConfigCandidate::Keepout {
                profile_boxes: profile_boxes.clone(),
                runtime_boxes: runtime_boxes.clone(),
                effective_boxes: effective_boxes.clone(),
                ws_eval_bufs,
            },
            AckMeta::Keepout {
                profile_boxes,
                runtime_boxes,
                effective_boxes,
            },
        ),
        PreparedPayload::Collision { enabled, sensitivity } => (
            ReservePrecheck::Collision {
                enabling: enabled == Some(true),
            },
            ConfigCandidate::Collision { enabled, sensitivity },
            AckMeta::Collision { enabled, sensitivity },
        ),
        PreparedPayload::GripperCalib {
            offset_rad,
            jaw_opening,
            effective_policy,
        } => (
            ReservePrecheck::GripperCalibration {
                candidate_policy: Some(effective_policy.clone()),
            },
            ConfigCandidate::GripperCalibration {
                offset_rad,
                policy: effective_policy.clone(),
                jaw_opening: Some(jaw_opening.clone()),
                clear_stale: true,
            },
            AckMeta::GripperCalib {
                offset_rad,
                jaw_opening,
                policy: effective_policy,
            },
        ),
        PreparedPayload::PosesSave { pose, next_list } => (
            ReservePrecheck::PosesSave,
            ConfigCandidate::Poses {
                next_list: next_list.clone(),
            },
            AckMeta::PosesSave { pose, next_list },
        ),
        PreparedPayload::PosesDelete {
            pose_name,
            pose_id,
            next_list,
        } => (
            ReservePrecheck::PosesDelete {
                deleting_pose_id: HolderLabel::new(&pose_id),
            },
            ConfigCandidate::Poses {
                next_list: next_list.clone(),
            },
            AckMeta::PosesDelete {
                pose_name,
                pose_id,
                next_list,
            },
        ),
        PreparedPayload::Sequences {
            sequence,
            deleting_id,
            next_file,
        } => {
            let reserved_pins = sequence.as_ref().map(|s| crate::sequences::ReservedSequencePins {
                sequence_id: s.id.clone(),
                pose_ids: s.pose_ids().map(str::to_string).collect(),
                episode_ids: s.episode_ids().map(str::to_string).collect(),
            });
            (
                ReservePrecheck::Sequences {
                    steps: sequence.as_ref().map(|s| s.steps.clone()),
                    deleting_sequence_id: deleting_id.as_deref().map(HolderLabel::new),
                    reserved_pins,
                },
                ConfigCandidate::Sequences {
                    next_file: next_file.clone(),
                },
                AckMeta::Sequences { sequence, next_file },
            )
        }
        PreparedPayload::GraspSetup { setup, ws_eval_bufs } => (
            ReservePrecheck::GraspSetup {
                new_camera_tilt_deg: setup.camera_tilt_deg as f32,
                allow_during_calibration_busy: false,
            },
            ConfigCandidate::GraspSetup {
                setup: setup.clone(),
                ws_eval_bufs,
            },
            AckMeta::GraspSetup { setup },
        ),
        PreparedPayload::ToolCommissioning {
            tool_id,
            tool_config_digest,
            confirmed_at,
        } => (
            ReservePrecheck::ToolCommissioning {
                tool_id: HolderLabel::new(&tool_id),
                tool_config_digest: HolderLabel::new(&tool_config_digest),
            },
            ConfigCandidate::ToolCommissioning {
                tool_id: HolderLabel::new(&tool_id),
                tool_config_digest: HolderLabel::new(&tool_config_digest),
                confirmed_at: confirmed_at.clone(),
            },
            AckMeta::ToolCommissioning { confirmed_at },
        ),
    };

    let expected_workspace_revision = match &candidate {
        ConfigCandidate::Keepout { .. } => Some(ws_rev),
        ConfigCandidate::GraspSetup { ws_eval_bufs: Some(_), .. } => Some(ws_rev),
        _ => None,
    };
    let plan = ConfigCommitPlan {
        proposal,
        auth,
        precheck,
        candidate,
        write: write_op,
        expected_workspace_revision,
        expected_poses_revision: None,
    };

    // Snapshot pose_name / pose_id before moving `meta` into the publish closure for IN_USE enrichment.
    let delete_pose_ctx = match &meta {
        AckMeta::PosesDelete { pose_name, pose_id, .. } => Some((pose_name.clone(), pose_id.clone())),
        _ => None,
    };
    let ack_fields: Arc<std::sync::Mutex<Option<Vec<(&'static str, Value)>>>> =
        Arc::new(std::sync::Mutex::new(None));
    let ack_fields_cb = Arc::clone(&ack_fields);
    let ws_auth_cb = ws_auth.clone();
    let gen_now_cb = Arc::clone(&gen_now);

    let res = run_config_commit(
        &sup,
        plan,
        move || {
            if !ws_auth_cb.is_operator() {
                return Err((
                    "OPERATOR_REQUIRED",
                    ArmError::Rejected {
                        status: 403,
                        code: "OPERATOR_REQUIRED".into(),
                        message: format!("Operator session required for {path}"),
                    },
                ));
            }
            if gen_now_cb.load(Ordering::SeqCst) != generation {
                return Err((
                    "LEASE_LOST",
                    ArmError::Rejected {
                        status: 409,
                        code: "LEASE_LOST".into(),
                        message: "This connection lost the lease while persisting configuration".into(),
                    },
                ));
            }
            Ok(())
        },
        move |cfg, act| {
            cfg.workspace_revision = act.workspace_revision;
            let fields = match meta {
                AckMeta::Keepout {
                    profile_boxes,
                    runtime_boxes,
                    effective_boxes,
                } => {
                    if let Some(ws) = cfg.config.workspace.as_mut() {
                        ws.keepout_boxes = effective_boxes;
                    }
                    cfg.keepout.profile = Some(profile_boxes);
                    cfg.keepout.runtime_boxes = runtime_boxes;
                    cfg.keepout.runtime = true;
                    cfg.keepout.revision = act.revision;
                    let state = cfg.keepout_state();
                    vec![
                        ("boxes", json!(state.boxes)),
                        ("source", json!(state.source)),
                        ("revision", json!(state.revision)),
                    ]
                }
                AckMeta::Collision { enabled, sensitivity } => {
                    if let Some(e) = enabled {
                        cfg.collision_enabled = e;
                    }
                    if !cfg.collision_enabled {
                        cfg.collision_active = false;
                    }
                    if let Some(s) = sensitivity {
                        cfg.collision_sensitivity = s;
                    }
                    cfg.collision_runtime = true;
                    cfg.collision_revision = act.revision;
                    let col = cfg.collision_config(None);
                    vec![("collision", json!(col)), ("revision", json!(col.revision))]
                }
                AckMeta::GripperCalib {
                    offset_rad,
                    jaw_opening,
                    policy,
                } => {
                    if cfg.base_gripper_policy.is_none() {
                        cfg.base_gripper_policy = cfg.config.gripper_policy.clone();
                    }
                    cfg.config.gripper_policy = Some(policy.clone());
                    cfg.gripper_calib_offset = offset_rad;
                    cfg.gripper_calib_jaw_opening = Some(jaw_opening);
                    cfg.gripper_calib_invalid = false;
                    cfg.gripper_calib_stale = None;
                    cfg.gripper_calib_revision = act.revision;
                    vec![
                        ("policy", json!(policy)),
                        ("closed_rad", json!(policy.closed_rad)),
                        ("open_rad", json!(policy.open_rad)),
                        ("jaw_opening", json!(policy.jaw_opening)),
                        ("revision", json!(act.revision)),
                    ]
                }
                AckMeta::PosesSave { pose, next_list } => {
                    cfg.poses.list = next_list;
                    cfg.poses.revision = act.revision;
                    pose_ack_fields_from_published(cfg, &pose, act.revision)
                }
                AckMeta::PosesDelete { next_list, .. } => {
                    cfg.poses.list = next_list;
                    cfg.poses.revision = act.revision;
                    vec![("revision", json!(act.revision))]
                }
                AckMeta::Sequences { sequence, next_file } => {
                    cfg.sequences.file = next_file;
                    cfg.sequences.revision = act.revision;
                    let mut fields = vec![("revision", json!(act.revision))];
                    if let Some(seq) = sequence {
                        fields.push(("sequence", json!(seq)));
                    }
                    fields
                }
                AckMeta::GraspSetup { setup } => {
                    cfg.config.camera_mount.tilt_deg = setup.camera_tilt_deg as f32;
                    cfg.config.grasp = Some(setup.clone());
                    cfg.grasp_setup_revision = act.revision;
                    vec![("setup", json!(setup)), ("revision", json!(act.revision))]
                }
                AckMeta::ToolCommissioning { confirmed_at } => {
                    cfg.config.active_tool.commissioning_required = false;
                    cfg.tool_commissioning_confirmed_at = Some(confirmed_at);
                    cfg.tool_commissioning_revision = act.revision;
                    vec![
                        ("active_tool", json!(cfg.config.active_tool)),
                        (
                            "commissioning_required",
                            json!(cfg.config.active_tool.commissioning_required),
                        ),
                        ("revision", json!(act.revision)),
                    ]
                }
            };
            *ack_fields_cb.lock().unwrap_or_else(|e| e.into_inner()) = Some(fields);
        },
    )
    .await;

    let ack_fields = ack_fields.lock().unwrap_or_else(|e| e.into_inner()).take();

    match res {
        Ok(outcome) => {
            let control_mode = if outcome.activated.gravity_comp_active {
                "gravity_comp"
            } else {
                "position"
            };
            let mut ack = json!({
                "type": "command_ack",
                "request_id": request_id,
                "status": "accepted",
                "epoch_id": outcome.activated.epoch_id,
                "safety_state": outcome.activated.safety_state.to_json_with_hub(&sup),
                "is_torqued": outcome.activated.is_torqued,
                "control_mode": control_mode,
            });
            if let Some(fields) = ack_fields {
                for (k, v) in fields {
                    ack[k] = v;
                }
            }
            if outcome.persisted {
                ack["persisted"] = json!(true);
                ack["durability_confirmed"] = json!(outcome.durability_confirmed);
                if let Some(warn_msg) = outcome.durability_warning {
                    ack["durability_warning"] = json!(warn_msg);
                }
            }
            ack
        }
        Err(mut err) => {
            // Enrich `IN_USE` for `poses/delete` with the referencing sequence ids off-thread.
            if let (
                Some((pose_name, pose_id)),
                ArmError::Rejected { status: 409, code, .. },
            ) = (delete_pose_ctx, &err)
            {
                if code == "IN_USE" {
                    if let Ok(pub_cfg) = sup.published_config().await {
                        let users = pub_cfg.sequences.using_pose(&pose_id);
                        err = ArmError::detailed(
                            409,
                            "IN_USE",
                            format!("Pose '{pose_name}' is used by {} sequence(s)", users.len()),
                            json!({"sequences": users}),
                        );
                    }
                }
            }
            let cur_epoch = current_epoch_off_lock(&sup, epoch);
            off_lock_command_reject(&request_id, cur_epoch, err.into())
        }
    }
}

pub(super) fn current_epoch_off_lock(sup: &ArmSupervisor, fallback: u64) -> u64 {
    let hub_ep = sup.hub().latest().map_or(fallback, |s| s.obs.epoch_id);
    hub_ep.max(sup.estop_epoch())
}

pub(super) fn pose_ack_fields_from_published(
    pub_cfg: &PublishedConfig,
    pose: &crate::poses::NamedPose,
    revision: u64,
) -> Vec<(&'static str, Value)> {
    let default_joint_names = pub_cfg
        .poses
        .list
        .joint_names
        .clone()
        .unwrap_or_else(|| pub_cfg.joint_names());
    let pose_val = crate::poses::pose_response_value(pose, &default_joint_names);
    let pose_id = pose_val.get("pose_id").cloned().unwrap_or_else(|| json!(pose.id));
    let content_sha256 = pose_val.get("content_sha256").cloned().unwrap_or(Value::Null);
    let joint_names = pose_val
        .get("joint_names")
        .cloned()
        .unwrap_or_else(|| json!(default_joint_names));
    let stale = pose_val.get("stale").cloned().unwrap_or(Value::Null);
    vec![
        ("pose", pose_val),
        ("pose_id", pose_id),
        ("content_sha256", content_sha256),
        ("joint_names", joint_names),
        ("stale", stale),
        ("revision", json!(revision)),
    ]
}

/// Complete an off-lock config write after ingress checks:
/// - Preparation runs off `SupervisorLock` and off `armd-control`.
/// - Validation/prepare failures and volatile writes return `Some(reply)` immediately on `response`
///   (preserving exact WS wire shape).
/// - Staged (persisted) writes spawn onto `worker_tasks` and reply via `ack_tx`, returning `None`.
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_or_spawn_config_write(
    job: PendingConfigJob,
    conn_id: u64,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_deps: &crate::grasp::GraspDeps,
    ack_tx: &mpsc::Sender<Value>,
    ws_auth: &WsAuth,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    worker_tasks: &ConnectionTasks,
) -> Option<Value> {
    let prepared = match prepare_config_write_off_thread(
        recv_supervisor,
        &job,
        recv_deps.setup_path.as_ref(),
    )
    .await
    {
        Ok(p) => p,
        Err(err) => {
            return Some(off_lock_command_reject(&job.request_id, job.epoch, err.into()));
        }
    };
    let sup = recv_supervisor.clone();
    let ws_auth_c = ws_auth.clone();
    let conn_c = recv_connection.to_string();
    let gen_now = recv_generation.clone();
    if !prepared.has_write() {
        let reply = commit_prepared_config(
            sup,
            job.request_id,
            prepared,
            job.generation,
            job.epoch,
            job.stamp,
            conn_id,
            ws_auth_c,
            conn_c,
            gen_now,
        )
        .await;
        Some(reply)
    } else {
        let tx = ack_tx.clone();
        worker_tasks.spawn(async move {
            let reply = commit_prepared_config(
                sup,
                job.request_id,
                prepared,
                job.generation,
                job.epoch,
                job.stamp,
                conn_id,
                ws_auth_c,
                conn_c,
                gen_now,
            )
            .await;
            let _ = tx.send(reply).await;
        });
        None
    }
}

/// Executor-owner WS router for PR C paths (config writes, calibration/motor-zero/MIT,
/// and `/api/v1/mode/gravity_comp`).
///
/// Returns `None` for any frame NOT owned by PR C so `ws.rs` falls through untouched to
/// `executor_routing_reject` (leaving L/M/P/S/Y frames strictly unaffected).
/// Returns `Some(opt_reply)` when PR C handled the frame (`Some(reply)` to send immediately,
/// `None` when a background task will reply on `ack_tx`).
#[allow(clippy::too_many_arguments)]
pub(super) async fn handle_executor_ws_command(
    frame_kind: FrameKind,
    entry: Option<&'static DispatchEntry>,
    value: &Value,
    stamp: AdmissionStamp,
    recv_conn_id: u64,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    recv_client_label: &Mutex<Option<String>>,
    last_ordinary_request: &mut Option<u64>,
    ws_auth: &WsAuth,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_deps: &crate::grasp::GraspDeps,
    recv_calib: &crate::calibration::CalibrationDeps,
    ack_tx: &mpsc::Sender<Value>,
    worker_tasks: &ConnectionTasks,
) -> Option<Option<Value>> {
    if frame_kind != FrameKind::Command {
        return None;
    }
    let entry = entry?;
    let path = entry.key.path?;
    let is_cfg = is_config_path(path);
    let is_cal = calibration::is_calibration_path(path);
    let is_gcomp = path == "/api/v1/mode/gravity_comp";
    if !is_cfg && !is_cal && !is_gcomp {
        return None;
    }

    let request_id_val = value.get("request_id").cloned().unwrap_or(Value::Null);
    let epoch = current_epoch_off_lock(recv_supervisor, 1);

    // Out-of-scope hardware calibration / motor-zero / MIT ops fail closed unconditionally
    // on the RT executor before touching any file or live state.
    if is_cal
        && calibration::is_executor_calibration_unsupported(path, &value["body"], recv_supervisor)
            .await
    {
        return Some(Some(calibration::dispatch_executor_calibration_command(
            path, value, epoch,
        )));
    }

    // 1. Lease check (same rules as `lease::check_lease_authority`, without `SupervisorLock`).
    let local_generation = recv_generation.load(Ordering::SeqCst);
    let supplied_generation = value.get("generation").and_then(Value::as_u64).unwrap_or(0);
    let label_matches = {
        let lbl = recv_client_label.lock().unwrap_or_else(|e| e.into_inner());
        value.get("client_id").is_none_or(|id| id.as_str() == lbl.as_deref())
    };
    if local_generation == 0 || supplied_generation != local_generation || !label_matches {
        let lease_msg = if is_cal {
            "This connection does not own an active lease"
        } else {
            "This connection does not hold the active lease"
        };
        return Some(Some(json!({
            "type": "command_reject",
            "request_id": request_id_val,
            "status": 409,
            "code": "LEASE_LOST",
            "error": "LEASE_LOST",
            "message": lease_msg,
            "epoch_id": epoch
        })));
    }

    // 2. Epoch check against live published state + atomic E-stop counter.
    if value.get("epoch_id").and_then(Value::as_u64) != Some(epoch) {
        return Some(Some(json!({
            "type": "command_reject",
            "request_id": request_id_val,
            "status": 409,
            "code": "EPOCH_MISMATCH",
            "message": "Command epoch is no longer current",
            "error": "EPOCH_MISMATCH",
            "epoch_id": epoch
        })));
    }

    // 3. Monotonic request_id check.
    let request_id = value.get("request_id").and_then(Value::as_u64);
    if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last) {
        return Some(Some(json!({
            "type": "command_reject",
            "request_id": value.get("request_id"),
            "status": 400,
            "code": "INVALID_COMMAND",
            "message": "request_id must be present and strictly increase",
            "error": "INVALID_COMMAND"
        })));
    }
    *last_ordinary_request = request_id;

    // 4. Single-authority operator auth from the dispatch table row (`entry.auth`).
    if let AuthRequirement::Operator(label) = entry.auth {
        if !ws_auth.is_operator() {
            return Some(Some(off_lock_command_reject(
                &request_id_val,
                epoch,
                ApiError::operator_required(label.unwrap_or(path)),
            )));
        }
    }

    // 5. Route by family.
    if is_cal {
        let reply = calibration::handle_executor_calibration_ws(
            path,
            request_id_val,
            value["body"].clone(),
            local_generation,
            epoch,
            stamp,
            recv_conn_id,
            recv_connection,
            recv_generation,
            ws_auth,
            recv_supervisor,
            recv_calib,
            ack_tx,
            worker_tasks,
        )
        .await;
        return Some(reply);
    }
    if is_cfg {
        let job = PendingConfigJob {
            request_id: request_id_val,
            path,
            body: value["body"].clone(),
            generation: local_generation,
            epoch,
            stamp,
            require_expected_revision: ws_auth.is_enforced(),
        };
        let reply = complete_or_spawn_config_write(
            job,
            recv_conn_id,
            recv_supervisor,
            recv_deps,
            ack_tx,
            ws_auth,
            recv_connection,
            recv_generation,
            worker_tasks,
        )
        .await;
        return Some(reply);
    }

    // `/api/v1/mode/gravity_comp`
    if let Err(msg) = validate_command_body_common(&value["body"]) {
        return Some(Some(off_lock_command_reject(
            &request_id_val,
            epoch,
            invalid_command(msg),
        )));
    }
    let enable = match value["body"].get("enable").and_then(Value::as_bool) {
        Some(b) => b,
        None => {
            return Some(Some(off_lock_command_reject(
                &request_id_val,
                epoch,
                invalid_command("enable must be boolean"),
            )));
        }
    };
    let (scale, damping_kd) = if enable {
        match motion::gravity_comp_request(&value["body"]) {
            Ok(req) => (req.scale, req.damping_kd),
            Err(err) => {
                return Some(Some(off_lock_command_reject(&request_id_val, epoch, err)));
            }
        }
    } else {
        (None, None)
    };
    let mut gc_stamp = stamp;
    gc_stamp.connection_id = recv_conn_id;
    gc_stamp.lease_gen = local_generation;
    gc_stamp.estop_epoch = epoch;
    let lease_holder = Some((HolderLabel::new(recv_connection), local_generation));
    match gravity_comp_command(recv_supervisor, gc_stamp, lease_holder, enable, scale, damping_kd).await {
        Ok(rep) => {
            let control_mode = if rep.active { "gravity_comp" } else { "position" };
            Some(Some(json!({
                "type": "command_ack",
                "request_id": request_id_val,
                "status": "accepted",
                "epoch_id": rep.epoch_id,
                "safety_state": rep.safety_state.to_json_with_hub(recv_supervisor),
                "is_torqued": rep.is_torqued,
                "control_mode": control_mode,
            })))
        }
        Err(err) => {
            let cur_epoch = current_epoch_off_lock(recv_supervisor, epoch);
            Some(Some(off_lock_command_reject(&request_id_val, cur_epoch, err.into())))
        }
    }
}
