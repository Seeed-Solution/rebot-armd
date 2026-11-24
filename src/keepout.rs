//! Runtime keep-out boxes (`/api/v1/workspace/keepout`), persisted to `ARMD_KEEPOUT`.
//! The boxes in force are the profile's `workspace.keepout_boxes` plus the runtime list: a
//! runtime list can only add boxes, never remove a profile box.

use std::path::{Path, PathBuf};
use serde::{Deserialize, Serialize};

use crate::model::KeepoutBox;
use crate::motion::MotionError;
use crate::persist::{self, PendingWrite};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const DEFAULT_KEEPOUT_PATH: &str = "/userdata/rebot-teleop/keepout.json";

/// `ARMD_KEEPOUT` or [`DEFAULT_KEEPOUT_PATH`].
pub fn path_from_env() -> PathBuf {
    persist::path_from_env("ARMD_KEEPOUT", DEFAULT_KEEPOUT_PATH)
}

/// File body and WS command body.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct KeepoutList {
    pub boxes: Vec<KeepoutBox>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expected_revision: Option<u64>,
}

/// One box in force and where it comes from (`profile` or `runtime`).
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct SourcedBox {
    pub name: String,
    pub min_m: [f32; 3],
    pub max_m: [f32; 3],
    pub source: &'static str,
}

/// `GET /api/v1/workspace/keepout` body.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct KeepoutState {
    /// Profile boxes first, then the runtime boxes, each tagged with its source.
    pub boxes: Vec<SourcedBox>,
    /// `runtime` once a runtime list was set, else `profile`.
    pub source: &'static str,
    pub revision: u64,
}

/// Runtime keep-out state kept in the supervisor.
#[derive(Debug, Clone)]
pub struct Keepout {
    /// True once a runtime list was set.
    pub runtime: bool,
    /// The profile list, captured when the first runtime list is applied (until then it is
    /// `config.workspace.keepout_boxes`).
    pub profile: Option<Vec<KeepoutBox>>,
    /// The runtime list (added to the profile list).
    pub runtime_boxes: Vec<KeepoutBox>,
    /// File the runtime list is written to; None = not persisted.
    pub path: Option<PathBuf>,
    /// Monotonic revision (starts at 1).
    pub revision: u64,
}

impl Default for Keepout {
    fn default() -> Self {
        Self {
            runtime: false,
            profile: None,
            runtime_boxes: Vec::new(),
            path: None,
            revision: 1,
        }
    }
}

impl SupervisorInner {
    fn profile_keepout(&self) -> Vec<KeepoutBox> {
        self.keepout.profile.clone()
            .unwrap_or_else(|| self.config.workspace.as_ref().map(|ws| ws.keepout_boxes.clone()).unwrap_or_default())
    }

    pub fn keepout_state(&self) -> KeepoutState {
        let tag = |b: &KeepoutBox, source| SourcedBox { name: b.name.clone(), min_m: b.min_m, max_m: b.max_m, source };
        let boxes = self.profile_keepout().iter().map(|b| tag(b, "profile"))
            .chain(self.keepout.runtime_boxes.iter().map(|b| tag(b, "runtime"))).collect();
        KeepoutState { boxes, source: if self.keepout.runtime { "runtime" } else { "profile" }, revision: self.keepout.revision }
    }

    /// Set the runtime keep-out list; the boxes in force become profile ∪ runtime.
    /// Needs the Cartesian workspace check (kinematics + workspace).
    pub fn set_keepout_boxes(&mut self, boxes: Vec<KeepoutBox>) -> Result<(), ArmError> {
        if self.workspace_policy().is_none() {
            return Err(MotionError::Unsupported("Keep-out boxes need a profile with kinematics and a workspace".into()).into());
        }
        if let Some(reason) = self.workspace_change_blocker() {
            return Err(MotionError::MotionBusy(format!("keep-out boxes cannot change while {reason}")).into());
        }
        KeepoutBox::validate_list(&boxes).map_err(MotionError::InvalidCommand)?;
        let profile = self.profile_keepout();
        let effective: Vec<KeepoutBox> = profile.iter().chain(&boxes).cloned().collect();
        KeepoutBox::validate_list(&effective).map_err(|e| MotionError::InvalidCommand(
            format!("{e} (profile boxes included: {})", profile.len())))?;
        if let Some(ws) = self.config.workspace.as_mut() { ws.keepout_boxes = effective; }
        self.keepout.profile = Some(profile);
        self.keepout.runtime_boxes = boxes;
        self.keepout.runtime = true;
        self.workspace_revision += 1;
        self.bump_motion_config_revision();
        self.servo_ws_min_depths = None;
        self.keepout.revision += 1;
        self.prepare_workspace_eval_bufs();
        Ok(())
    }
}

/// Validated keepout update prepared without mutating `SupervisorInner`.
#[derive(Debug, Clone)]
pub struct PreparedKeepout {
    pub base_revision: u64,
    pub boxes: Vec<KeepoutBox>,
    pub write: Option<PendingWrite>,
}

/// Step 1 of atomic persist-before-commit: validate payload, expected_revision, and motion
/// blocker on a read-only view without mutating `SupervisorInner`.
pub fn prepare_update(inner: &SupervisorInner, body: &serde_json::Value) -> Result<PreparedKeepout, ArmError> {
    prepare_update_with(inner, body, false)
}

/// Same as [`prepare_update`], optionally requiring `expected_revision` (enforced on real hardware).
pub fn prepare_update_with(
    inner: &SupervisorInner,
    body: &serde_json::Value,
    require_expected_revision: bool,
) -> Result<PreparedKeepout, ArmError> {
    let list: KeepoutList = serde_json::from_value(body.clone())
        .map_err(|e| MotionError::InvalidCommand(format!("keep-out body must be {{\"boxes\": [{{name, min_m, max_m}}]}}: {e}")))?;
    if require_expected_revision && list.expected_revision.is_none() {
        return Err(MotionError::InvalidCommand("expected_revision is required when operator auth is enforced".into()).into());
    }
    if let Some(exp) = list.expected_revision {
        if exp != inner.keepout.revision {
            return Err(ArmError::PolicyRevisionMismatch { expected: exp, current: inner.keepout.revision });
        }
    }
    if inner.workspace_policy().is_none() {
        return Err(MotionError::Unsupported("Keep-out boxes need a profile with kinematics and a workspace".into()).into());
    }
    if let Some(reason) = inner.workspace_change_blocker() {
        return Err(MotionError::MotionBusy(format!("keep-out boxes cannot change while {reason}")).into());
    }
    KeepoutBox::validate_list(&list.boxes).map_err(MotionError::InvalidCommand)?;
    let profile = inner.profile_keepout();
    let effective: Vec<KeepoutBox> = profile.iter().chain(&list.boxes).cloned().collect();
    KeepoutBox::validate_list(&effective).map_err(|e| MotionError::InvalidCommand(
        format!("{e} (profile boxes included: {})", profile.len())))?;
    let write = inner.keepout.path.clone().map(|path| PendingWrite::new(
        path,
        serde_json::to_string_pretty(&KeepoutList { boxes: list.boxes.clone(), expected_revision: None }).unwrap_or_default(),
    ));
    Ok(PreparedKeepout {
        base_revision: inner.keepout.revision,
        boxes: list.boxes,
        write,
    })
}

/// Step 3 of atomic persist-before-commit: re-verify revision and motion blocker, then commit.
pub fn commit_update(inner: &mut SupervisorInner, prepared: PreparedKeepout) -> Result<KeepoutState, ArmError> {
    if inner.keepout.revision != prepared.base_revision {
        return Err(ArmError::PolicyRevisionMismatch { expected: prepared.base_revision, current: inner.keepout.revision });
    }
    inner.set_keepout_boxes(prepared.boxes)?;
    Ok(inner.keepout_state())
}

/// WS command `/api/v1/workspace/keepout`: replace the runtime list (profile boxes stay). Returns the new state
/// and the file write to run after the lock is released.
pub fn update(inner: &mut SupervisorInner, body: &serde_json::Value) -> Result<(KeepoutState, Option<PendingWrite>), ArmError> {
    let mut prepared = prepare_update(inner, body)?;
    let write = prepared.write.take();
    let state = commit_update(inner, prepared)?;
    Ok((state, write))
}

/// Load the persisted runtime list at startup (fail-closed: returns `Err` and leaves the file
/// untouched on any read/parse/validation, missing-workspace non-empty boxes, or combined-install error).
///
/// Restart reconciliation (track B): the keep-out domain is `Reconciling` (motion inhibited)
/// from the start of the load until the file is installed, so active and durable versions
/// agree; a load that fails leaves it `Reconciling` (the file stays on disk, the profile
/// boxes stay active).
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) -> Result<(), String> {
    use crate::rt_core::config_iface::{ConfigAuthority, ConfigDomain};
    persist::remove_staged_leftovers(path);
    let reconciling = supervisor.inner().write().await.begin_reconcile(ConfigDomain::Keepout);
    let loaded = load_file_into(supervisor, path).await;
    if loaded.is_ok() && reconciling {
        supervisor.inner().write().await.finish_reconcile(ConfigDomain::Keepout);
    }
    loaded
}

async fn load_file_into(supervisor: &ArmSupervisor, path: &Path) -> Result<(), String> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => Some(t),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => {
            supervisor.inner().write().await.keepout.path = Some(path.to_path_buf());
            return Err(format!("read {}: {e}", path.display()));
        }
    };
    let mut inner = supervisor.inner().write().await;
    inner.keepout.path = Some(path.to_path_buf());
    let Some(text) = text else {
        return Ok(());
    };
    let list: KeepoutList = serde_json::from_str(&text)
        .map_err(|e| format!("parse {}: {e}", path.display()))?;
    KeepoutBox::validate_list(&list.boxes)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    let box_count = list.boxes.len();
    if inner.workspace_policy().is_none() {
        if box_count > 0 {
            return Err(format!(
                "install {}: profile has no kinematics/workspace to enforce {box_count} persisted keep-out box(es)",
                path.display()
            ));
        }
        return Ok(());
    }
    inner
        .set_keepout_boxes(list.boxes)
        .map_err(|e| format!("install {}: {e}", path.display()))?;
    inner.keepout.revision = 1;
    tracing::info!("Keep-out boxes loaded from {} ({box_count} boxes)", path.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ArmConfig;

    #[tokio::test]
    async fn startup_rejects_corrupt_and_combined_overflow_keepout_files() {
        let dir = tempfile::tempdir().unwrap();

        // 1. Corrupt JSON fails closed and leaves file untouched
        let corrupt = dir.path().join("corrupt.json");
        std::fs::write(&corrupt, "{\"boxes\": [broken").unwrap();
        let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
        let err = load_into(&sup, &corrupt).await.unwrap_err();
        assert!(err.contains("parse"), "{err}");
        assert!(corrupt.exists(), "corrupt file must remain in place for fail-closed restarts");
        assert_eq!(sup.inner().read().await.keepout.revision, 1);

        // 2. Combined profile + runtime overflow (> MAX_KEEPOUT_BOXES) fails closed
        let mut cfg = ArmConfig::rebot_dm();
        cfg.workspace.as_mut().unwrap().keepout_boxes = (0..10)
            .map(|i| KeepoutBox { name: format!("p{i}"), min_m: [0.2, 0.2, 0.2], max_m: [0.3, 0.3, 0.3] })
            .collect();
        let overflow = dir.path().join("overflow.json");
        let rt_boxes: Vec<KeepoutBox> = (0..7)
            .map(|i| KeepoutBox { name: format!("r{i}"), min_m: [0.2, 0.2, 0.2], max_m: [0.3, 0.3, 0.3] })
            .collect();
        std::fs::write(&overflow, serde_json::to_string(&KeepoutList { boxes: rt_boxes, expected_revision: None }).unwrap()).unwrap();
        let sup2 = ArmSupervisor::new(cfg);
        let err2 = load_into(&sup2, &overflow).await.unwrap_err();
        assert!(err2.contains("install") && err2.contains("16"), "{err2}");
        let state = sup2.inner().read().await.keepout_state();
        assert_eq!(state.source, "profile");
        assert_eq!(state.boxes.len(), 10);
        assert_eq!(state.revision, 1);

        // 3. Profile without workspace must fail closed on non-empty persisted keepout file,
        //    while allowing missing file or empty boxes list.
        let non_empty = dir.path().join("non_empty.json");
        std::fs::write(
            &non_empty,
            serde_json::to_string(&KeepoutList {
                boxes: vec![KeepoutBox { name: "bench".into(), min_m: [0.2, 0.2, 0.0], max_m: [0.3, 0.3, 0.2] }],
                expected_revision: None,
            })
            .unwrap(),
        )
        .unwrap();
        let sup_no_ws = ArmSupervisor::new(ArmConfig::default());
        let err3 = load_into(&sup_no_ws, &non_empty).await.unwrap_err();
        assert!(err3.contains("no kinematics/workspace"), "{err3}");

        let empty_file = dir.path().join("empty.json");
        std::fs::write(&empty_file, "{\"boxes\": []}").unwrap();
        load_into(&sup_no_ws, &empty_file).await.unwrap();
        load_into(&sup_no_ws, &dir.path().join("missing.json")).await.unwrap();
    }
}
