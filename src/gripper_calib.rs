//! Runtime gripper calibration (`/api/v1/gripper/calibration`), persisted to `ARMD_GRIPPER_CALIB`.

use std::path::{Path, PathBuf};
use serde::{Deserialize, Serialize};

use crate::model::{GripperPolicy, JawOpening, JointLimit};
use crate::motion::MotionError;
use crate::persist::{self, Loaded, PendingWrite};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const DEFAULT_GRIPPER_CALIB_PATH: &str = "/userdata/rebot-teleop/gripper_calib.json";

/// `ARMD_GRIPPER_CALIB` or [`DEFAULT_GRIPPER_CALIB_PATH`].
pub fn path_from_env() -> PathBuf {
    persist::path_from_env("ARMD_GRIPPER_CALIB", DEFAULT_GRIPPER_CALIB_PATH)
}

/// Persisted file format.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GripperCalibFile {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub offset_rad: Option<f32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jaw_opening: Option<JawOpening>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ts: Option<u64>,
}

impl GripperCalibFile {
    pub fn validate(&self, j7_limit: Option<&JointLimit>) -> Result<(), String> {
        if let Some(off) = self.offset_rad {
            if !off.is_finite() || off.abs() > 0.5 {
                return Err(format!("offset_rad {off} exceeds limit of 0.5 rad"));
            }
        }
        if let Some(ref jo) = self.jaw_opening {
            jo.validate(j7_limit)?;
        }
        Ok(())
    }
}

/// REST response for `GET /api/v1/gripper/calibration`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GripperCalibrationResponse {
    pub offset_rad: Option<f32>,
    pub jaw_opening: Option<JawOpening>,
    pub calibrated: bool,
    pub policy: Option<GripperPolicy>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ts: Option<u64>,
}

/// Body for WS `/api/v1/gripper/calibration`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CalibBody {
    pub jaw_opening: JawOpening,
    #[serde(default)]
    pub offset_rad: Option<f32>,
}

/// WS command `/api/v1/gripper/calibration` `{"jaw_opening": JawOpening, "offset_rad": f32?}`.
/// Validates jaw_opening (monotonic, >= 2 points, inside J7 limits, openings in [0, 0.2]) and |offset_rad| <= 0.5.
/// Returns the effective policy and the file write to run after the lock is released.
pub fn update(inner: &mut SupervisorInner, body: &serde_json::Value) -> Result<(GripperPolicy, Option<PendingWrite>), ArmError> {
    let b: CalibBody = serde_json::from_value(body.clone()).map_err(|e| {
        MotionError::InvalidCommand(format!(
            "gripper calibration body must have jaw_opening and optional offset_rad: {e}"
        ))
    })?;

    if let Some(off) = b.offset_rad {
        if !off.is_finite() || off.abs() > 0.5 {
            return Err(MotionError::InvalidCommand(format!(
                "offset_rad {off} must be in [-0.5, 0.5] rad"
            )).into());
        }
    }

    let j7_limit = inner.config.limits.get(6);
    b.jaw_opening.validate(j7_limit).map_err(MotionError::InvalidCommand)?;

    let offset = b.offset_rad.or(inner.gripper_calib_offset);
    inner.apply_gripper_calibration(offset, Some(b.jaw_opening.clone()));
    inner.gripper_calib_invalid = false;

    let effective = inner.config.gripper_policy.clone().ok_or_else(|| {
        MotionError::Unsupported("Gripper calibration requires a gripper policy".into())
    })?;

    let file = GripperCalibFile {
        offset_rad: inner.gripper_calib_offset,
        jaw_opening: Some(b.jaw_opening),
        ts: Some(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs())),
    };

    let write = inner.gripper_calib_path.clone().or_else(|| Some(path_from_env())).map(|path| {
        PendingWrite::new(path, serde_json::to_string_pretty(&file).unwrap_or_default())
    });

    Ok((effective, write))
}

/// Load the persisted gripper calibration at startup (logs outcome).
/// Invalid/corrupt file -> log a warning, ignore it, report observation.gripper_zero.state "calib_invalid".
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) {
    let mut inner = supervisor.inner().write().await;
    inner.gripper_calib_path = Some(path.to_path_buf());
    let j7_lim = inner.config.limits.get(6).cloned();
    match persist::load::<GripperCalibFile>(path, |f| f.validate(j7_lim.as_ref())) {
        Loaded::Loaded(calib) => {
            tracing::info!("Gripper calibration loaded from {}", path.display());
            inner.apply_gripper_calibration(calib.offset_rad, calib.jaw_opening);
            inner.gripper_calib_invalid = false;
        }
        Loaded::Missing => {}
        Loaded::Invalid(e) => {
            tracing::warn!("Ignoring gripper calibration file: {e}; using uncalibrated default");
            inner.gripper_calib_invalid = true;
        }
    }
}
