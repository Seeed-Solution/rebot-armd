//! Runtime keep-out boxes (`/api/v1/workspace/keepout`), persisted to `ARMD_KEEPOUT`.
//! A runtime list replaces the profile's `workspace.keepout_boxes`.

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

/// `GET /api/v1/workspace/keepout` body.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct KeepoutState {
    pub boxes: Vec<KeepoutBox>,
    /// `profile` or `runtime`.
    pub source: &'static str,
}

/// Runtime keep-out state kept in the supervisor.
#[derive(Debug, Clone, Default)]
pub struct Keepout {
    /// True once a runtime list replaced the profile list.
    pub runtime: bool,
    /// File the runtime list is written to; None = not persisted.
    pub path: Option<PathBuf>,
}

impl SupervisorInner {
    pub fn keepout_state(&self) -> KeepoutState {
        let boxes = self.config.workspace.as_ref().map(|ws| ws.keepout_boxes.clone()).unwrap_or_default();
        KeepoutState { boxes, source: if self.keepout.runtime { "runtime" } else { "profile" } }
    }

    /// Replace the keep-out list. Needs the Cartesian workspace check (kinematics + workspace).
    pub fn set_keepout_boxes(&mut self, boxes: Vec<KeepoutBox>) -> Result<(), ArmError> {
        if self.workspace_policy().is_none() {
            return Err(MotionError::Unsupported("Keep-out boxes need a profile with kinematics and a workspace".into()).into());
        }
        KeepoutBox::validate_list(&boxes).map_err(MotionError::InvalidCommand)?;
        if let Some(ws) = self.config.workspace.as_mut() { ws.keepout_boxes = boxes; }
        self.keepout.runtime = true;
        Ok(())
    }
}

/// WS command `/api/v1/workspace/keepout`: replace the runtime list. Returns the new state
/// and the file write to run after the lock is released.
pub fn update(inner: &mut SupervisorInner, body: &serde_json::Value) -> Result<(KeepoutState, Option<PendingWrite>), ArmError> {
    let list: KeepoutList = serde_json::from_value(body.clone())
        .map_err(|e| MotionError::InvalidCommand(format!("keep-out body must be {{\"boxes\": [{{name, min_m, max_m}}]}}: {e}")))?;
    inner.set_keepout_boxes(list.boxes)?;
    let state = inner.keepout_state();
    let write = inner.keepout.path.clone().map(|path| PendingWrite {
        path,
        text: serde_json::to_string_pretty(&KeepoutList { boxes: state.boxes.clone() }).unwrap_or_default(),
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
