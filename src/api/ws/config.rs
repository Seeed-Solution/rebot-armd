//! Persist-before-commit configuration writes of the WS command path (track B).
//!
//! Moved from `api/ws.rs` (armd 1.6 RT-core foundation); moved items, their fields and
//! inherent methods are `pub(super)` so the connection loop keeps using them.
//!
//! Every write is a versioned change ([`crate::rt_core::config_iface`]): the candidate is
//! validated under the lock ([`PreparedConfigWrite::prepare`]) and its file staged off the
//! lock ([`persist_and_activate`]); at the commit boundary (one supervisor write guard) the
//! connection's authorization, lease and epoch are rechecked, the domain is reserved at the
//! revision the candidate was built from, the file is renamed into place, the candidate is
//! applied and the reservation activated (`DurableUnconfirmed` until the directory sync,
//! which runs after the guard is released and decides `durability_confirmed`).

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey,
    LeaseRequirement, OwnerDispatchCtx, ReplyPolicy,
};
use crate::rt_core::config_iface::{ConfigDomain, Durability};

macro_rules! config_entry {
    ($path:expr) => {
        DispatchEntry {
            key: DispatchKey::command($path),
            handler: DispatchHandler::Owner(dispatch_config_entry),
            admission: AdmissionClass::Ordinary,
            lease: LeaseRequirement::Required,
            auth: AuthRequirement::Operator,
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

fn dispatch_config_entry(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    try_dispatch_config_write(
        ctx.inner,
        ctx.path,
        ctx.value,
        ctx.recv_deps.setup_path.as_ref(),
        ctx.ws_auth.is_enforced(),
        ctx.local_generation,
        ctx.epoch,
        ctx.pending_config_write,
    )
    .expect("registered config path")
}

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

    /// The configuration domain the write changes.
    pub(super) fn domain(&self) -> ConfigDomain {
        match self {
            Self::Keepout(_) => ConfigDomain::Keepout,
            Self::Collision(_) => ConfigDomain::Collision,
            Self::GripperCalib(_) => ConfigDomain::GripperCalibration,
            Self::PosesSave(_) | Self::PosesDelete(_) | Self::PosesRename(_) => ConfigDomain::Poses,
            Self::Sequences(_) => ConfigDomain::Sequences,
            Self::GraspSetup(_) => ConfigDomain::GraspSetup,
            Self::ToolCommissioning(_) => ConfigDomain::ToolCommissioning,
        }
    }

    /// The domain revision the candidate was built from.
    pub(super) fn base_revision(&self) -> u64 {
        match self {
            Self::Keepout(p) => p.base_revision,
            Self::Collision(p) => p.base_revision,
            Self::GripperCalib(p) => p.base_revision,
            Self::PosesSave(p) | Self::PosesRename(p) => p.base_revision,
            Self::PosesDelete(p) => p.base_revision,
            Self::Sequences(p) => p.base_revision,
            Self::GraspSetup(p) => p.base_revision,
            Self::ToolCommissioning(p) => p.base_revision,
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

    /// Commits a write with nothing to persist: reserve, apply, activate (`Volatile`).
    pub(super) fn commit(self, inner: &mut SupervisorInner) -> Result<Vec<(&'static str, Value)>, ArmError> {
        inner.commit_config(self.domain(), self.base_revision(), 0, |inner| self.apply(inner)).map(|(fields, _)| fields)
    }

    /// Applies the candidate (moves its domain to the next revision); called only under a
    /// held reservation.
    fn apply(self, inner: &mut SupervisorInner) -> Result<Vec<(&'static str, Value)>, ArmError> {
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

/// The commit boundary of a staged write, under the supervisor write guard: `Ok` with the
/// ack and the directory sync to run after the guard is released, `Err` with the reject.
/// The staged file is renamed into place only after every check and the reservation passed.
#[allow(clippy::too_many_arguments)]
fn commit_staged(
    inner: &mut SupervisorInner,
    request_id: &Value,
    mut prepared: PreparedConfigWrite,
    staged: crate::persist::StagedWrite,
    generation: u64,
    epoch: u64,
    ws_auth: &WsAuth,
    conn: &str,
    gen_now: &AtomicU64,
) -> Result<(Value, crate::persist::PendingDirSync), Value> {
    let reject = |staged: crate::persist::StagedWrite, reply: Value| {
        staged.abort();
        Err(reply)
    };
    if !ws_auth.is_operator() {
        return reject(staged, command_reject(inner, request_id, ApiError::operator_required(prepared.path())));
    }
    if gen_now.load(Ordering::SeqCst) != generation || !inner.validate_servo_lease(conn, generation) {
        let reply = json!({
            "type": "command_reject",
            "request_id": request_id,
            "status": 409,
            "code": "LEASE_LOST",
            "message": "This connection lost the lease while persisting configuration",
            "error": "LEASE_LOST",
            "epoch_id": inner.safety.epoch_id()
        });
        return reject(staged, reply);
    }
    if inner.safety.epoch_id() != epoch {
        let reply = json!({
            "type": "command_reject",
            "request_id": request_id,
            "status": 409,
            "code": "EPOCH_MISMATCH",
            "message": "Command epoch is no longer current",
            "error": "EPOCH_MISMATCH",
            "epoch_id": inner.safety.epoch_id()
        });
        return reject(staged, reply);
    }
    if let Some(err) = prepared.precheck(inner) {
        return reject(staged, command_reject(inner, request_id, err.into()));
    }
    let reservation = match inner.reserve_config(prepared.domain(), prepared.base_revision(), generation) {
        Ok(r) => r,
        Err(err) => return reject(staged, command_reject(inner, request_id, err.into())),
    };
    let dir_sync = match staged.rename_only() {
        Ok(d) => d,
        Err(e) => {
            let _ = crate::rt_core::config_iface::ConfigAuthority::release(inner, reservation);
            tracing::warn!("State file not published: {e}");
            return Err(command_reject(inner, request_id, ApiError::persist_failed(format!("Failed to persist configuration: {e}"))));
        }
    };
    // renamed; the directory sync runs after the guard (`durability_confirmed` in the ack)
    let reservation = match inner.config_persisted(reservation, Durability::DurableUnconfirmed) {
        Ok(r) => r,
        Err(err) => return Err(command_reject(inner, request_id, err.into())),
    };
    match inner.activate_config_candidate(reservation, |inner| prepared.apply(inner)) {
        Ok((fields, _)) => {
            let mut ack = command_ack(inner, request_id, None);
            for (key, field) in fields { ack[key] = field; }
            ack["persisted"] = json!(true);
            Ok((ack, dir_sync))
        }
        Err(err) => {
            // The file is in place but the candidate is not active: the hold is Reconciling
            // (motion inhibited, further changes CONFIG_BUSY) until the next start, whose
            // loader activates the file.
            tracing::error!("{} written but not activated: {err}", dir_sync.target().display());
            Err(command_reject(inner, request_id, err.into()))
        }
    }
}

/// Atomic two-phase persist-before-commit of a staged config write (keepout, collision,
/// gripper calibration, poses, sequences, grasp setup, tool commissioning). Stage runs off
/// the supervisor lock; the commit boundary ([`commit_staged`]) under its write guard; the
/// directory sync after it. Returns the reply to send.
#[allow(clippy::too_many_arguments)]
pub(super) async fn persist_and_activate(
    sup: Arc<ArmSupervisor>,
    request_id: Value,
    mut prepared: PreparedConfigWrite,
    generation: u64,
    epoch: u64,
    ws_auth: WsAuth,
    conn: String,
    gen_now: Arc<AtomicU64>,
) -> Value {
    let cw_lock = sup.config_write_lock();
    let _cw = cw_lock.lock().await;
    let write = prepared.take_write().expect("checked has_write");
    let staged_res = tokio::task::spawn_blocking(move || crate::persist::StagedWrite::stage(write.path, write.text)).await;
    let staged = match staged_res {
        Err(e) => {
            tracing::warn!("State file stage task failed: {e}");
            let inner = sup.inner().read().await;
            return command_reject(&inner, &request_id, ApiError::persist_failed(format!("Failed to persist configuration: {e}")));
        }
        Ok(Err(e)) => {
            tracing::warn!("State file not staged: {e}");
            let inner = sup.inner().read().await;
            return command_reject(&inner, &request_id, ApiError::persist_failed(format!("Failed to persist configuration: {e}")));
        }
        Ok(Ok(staged)) => staged,
    };
    let committed = {
        let mut inner = sup.inner().write().await;
        commit_staged(&mut inner, &request_id, prepared, staged, generation, epoch, &ws_auth, &conn, &gen_now)
    };
    match committed {
        Err(reply) => reply,
        Ok((mut ack, dir_sync)) => {
            let outcome = tokio::task::spawn_blocking(move || dir_sync.sync_parent_dir())
                .await
                .unwrap_or_else(|e| crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(e.to_string()));
            match outcome {
                crate::persist::PublishOutcome::Published => {
                    ack["durability_confirmed"] = json!(true);
                }
                crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(warn_msg) => {
                    ack["durability_confirmed"] = json!(false);
                    ack["durability_warning"] = json!(warn_msg);
                }
            }
            ack
        }
    }
}

pub(super) fn pose_ack_fields(
    inner: &SupervisorInner,
    pose: &crate::poses::NamedPose,
    revision: u64,
) -> Vec<(&'static str, Value)> {
    let default_joint_names = inner
        .poses
        .list
        .joint_names
        .clone()
        .unwrap_or_else(|| inner.joint_names());
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

/// Dispatch a staged/volatile configuration write command under the supervisor write lock.
/// Returns `Some(response)` if handled synchronously or rejected; `None` if a staged disk
/// write was queued into `pending_config_write`, or `Err(())` if `path` is not a config path.
#[allow(clippy::too_many_arguments)]
pub(super) fn try_dispatch_config_write(
    inner: &mut SupervisorInner,
    path: &str,
    value: &Value,
    setup_path: Option<&std::path::PathBuf>,
    require_expected_revision: bool,
    local_generation: u64,
    epoch: u64,
    pending_config_write: &mut Option<(Value, PreparedConfigWrite, u64, u64)>,
) -> Result<Option<Value>, ()> {
    let Some(result) =
        PreparedConfigWrite::prepare(inner, path, &value["body"], setup_path, require_expected_revision)
    else {
        return Err(());
    };
    Ok(match result {
        Ok(prepared) if !prepared.has_write() => Some(match prepared.commit(inner) {
            Ok(fields) => {
                let mut ack = command_ack(inner, &value["request_id"], None);
                for (key, field) in fields {
                    ack[key] = field;
                }
                ack
            }
            Err(error) => command_reject(inner, &value["request_id"], error.into()),
        }),
        Ok(prepared) => {
            *pending_config_write = Some((value["request_id"].clone(), prepared, local_generation, epoch));
            None
        }
        Err(error) => Some(command_reject(inner, &value["request_id"], error.into())),
    })
}

/// Spawn a tracked staged configuration persistence task and forward its reply to `ack_tx`.
#[allow(clippy::too_many_arguments)]
pub(super) fn spawn_config_write(
    request_id: Value,
    prepared: PreparedConfigWrite,
    generation: u64,
    epoch: u64,
    recv_supervisor: &Arc<ArmSupervisor>,
    ack_tx: &mpsc::Sender<Value>,
    ws_auth: &WsAuth,
    recv_connection: &str,
    recv_generation: &Arc<AtomicU64>,
    worker_tasks: &ConnectionTasks,
) {
    let sup = recv_supervisor.clone();
    let tx = ack_tx.clone();
    let (ws_auth_c, conn_c, gen_now) = (
        ws_auth.clone(),
        recv_connection.to_string(),
        recv_generation.clone(),
    );
    worker_tasks.spawn(async move {
        let reply = persist_and_activate(sup, request_id, prepared, generation, epoch, ws_auth_c, conn_c, gen_now).await;
        let _ = tx.send(reply).await;
    });
}
