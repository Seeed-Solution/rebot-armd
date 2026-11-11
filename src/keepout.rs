//! Runtime keep-out boxes (`/api/v1/workspace/keepout`), persisted to `ARMD_KEEPOUT`.
//! The boxes in force are the profile's `workspace.keepout_boxes` plus the runtime list: a
//! runtime list can only add boxes, never remove a profile box.

use std::path::{Path, PathBuf};
use serde::{Deserialize, Serialize};

use crate::model::KeepoutBox;
use crate::motion::MotionError;
use crate::persist::{self, Loaded, PendingWrite};
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
}

/// Runtime keep-out state kept in the supervisor.
#[derive(Debug, Clone, Default)]
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
        KeepoutState { boxes, source: if self.keepout.runtime { "runtime" } else { "profile" } }
    }

    /// Set the runtime keep-out list; the boxes in force become profile ∪ runtime.
    /// Needs the Cartesian workspace check (kinematics + workspace).
    pub fn set_keepout_boxes(&mut self, boxes: Vec<KeepoutBox>) -> Result<(), ArmError> {
        if self.workspace_policy().is_none() {
            return Err(MotionError::Unsupported("Keep-out boxes need a profile with kinematics and a workspace".into()).into());
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
        Ok(())
    }
}

/// WS command `/api/v1/workspace/keepout`: replace the runtime list (profile boxes stay). Returns the new state
/// and the file write to run after the lock is released.
pub fn update(inner: &mut SupervisorInner, body: &serde_json::Value) -> Result<(KeepoutState, Option<PendingWrite>), ArmError> {
    let list: KeepoutList = serde_json::from_value(body.clone())
        .map_err(|e| MotionError::InvalidCommand(format!("keep-out body must be {{\"boxes\": [{{name, min_m, max_m}}]}}: {e}")))?;
    inner.set_keepout_boxes(list.boxes)?;
    let state = inner.keepout_state();
    let write = inner.keepout.path.clone().map(|path| PendingWrite {
        path,
        text: serde_json::to_string_pretty(&KeepoutList { boxes: inner.keepout.runtime_boxes.clone() }).unwrap_or_default(),
    });
    Ok((state, write))
}

/// Load the persisted runtime list at startup (logs the outcome). An invalid file is left
/// untouched and the profile list stays in force.
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) {
    let mut inner = supervisor.inner().write().await;
    inner.keepout.path = Some(path.to_path_buf());
    if inner.workspace_policy().is_none() { return; }
    match persist::load::<KeepoutList>(path, |l| KeepoutBox::validate_list(&l.boxes)) {
        Loaded::Loaded(list) => {
            tracing::info!("Keep-out boxes loaded from {} ({} boxes)", path.display(), list.boxes.len());
            let _ = inner.set_keepout_boxes(list.boxes);
        }
        Loaded::Missing => {}
        Loaded::Invalid(e) => tracing::warn!("Ignoring keep-out file: {e}; using the profile list"),
    }
}
