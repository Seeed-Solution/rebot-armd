//! Owner-side configuration commit requests (stage 3 A8 / PR C).
//!
//! All configuration writes (WS `PreparedConfigWrite`, calibration commit paths,
//! REST `post_tool_commissioning_confirm`) and published configuration readers flow
//! through this module on both `RtOwner::Legacy` and `RtOwner::Executor`:
//!
//! - Preparation, serialization, staging (`StagedWrite::stage`), atomic rename
//!   (`StagedWrite::rename_only`), and parent-directory fsync (`sync_parent_dir`)
//!   always run OFF the control thread and OFF `SupervisorLock`.
//! - Owner mutations run through [`ReserveConfig`], [`PersistedConfig`],
//!   [`ActivateConfig`], and [`ReleaseConfig`] carried inside a single
//!   preallocated `Box<ConfigExchange>` so `armd-control` performs zero heap
//!   allocations and zero heap frees across reserve, persist, activate, release,
//!   and failure paths.
//! - Authority rechecks reuse A1's [`AdmissionStamp`], [`ClosedConnections`], and
//!   [`ConsumedStamps`] through one single entry point ([`check_config_authority`]).
//! - Pre-rename failure aborts staging and releases the hold (`Released::Cleared`).
//!   Post-rename failure moves the hold to `HoldState::Reconciling` so live state
//!   stays unchanged and motion stays inhibited.

// `ConfigError` and `ConfigExchange` variants are intentionally kept unboxed (stack / inline
// inside the single preallocated `Box<ConfigExchange>`) so `armd-control` performs strictly
// zero heap allocations and zero heap frees across every config request and error path.
#![allow(clippy::result_large_err, clippy::large_enum_variant)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, AtomicUsize, Ordering};
use std::sync::{Arc, RwLock};
use std::time::Instant;

use serde_json::{json, Value};

use crate::calibration::{CalibrationFile, CalibrationStale};
use crate::chain::{ArmModel, EvalBufs};
use crate::collision::{CollisionConfig, CollisionPolicy, Sensitivity};
use crate::grasp::GraspSetup;
use crate::gripper_calib::GripperCalibrationResponse;
use crate::keepout::{Keepout, KeepoutState, SourcedBox};
use crate::model::{
    ArmConfig, CalibrationToolInfo, CalibrationTools, CameraMount, Capabilities,
    GraspWidthCheck, GripperCalibrationTool, GripperPolicy, HolderLabel, JawOpening,
    KeepoutBox, MotorZeroSupport, MotorZeroTool,
};
use crate::motion::MotionError;
use crate::persist::{PendingDirSync, PendingWrite, PublishOutcome, StagedWrite};
use crate::poses::{NamedPose, PoseList, PoseStore, PoseTcp};
use crate::recording::store::EpisodeStore;
use crate::rt_core::broker::{Authority, BootId, ClientId, RequestId, Stamp};
use crate::rt_core::config_iface::{
    Activated, CandidateId, ConfigAuthority, ConfigDomain, ConfigProposal, Durability,
    GrantId, Released, Reservation, Reserved,
};
use crate::rt_core::ids::ArmId;
use crate::rt_core::ops::{CommitCtx, CoreOps};
use crate::safety::BoundedText;
use crate::sequences::{Sequence, SequenceFile, SequenceStore, Step};
use crate::supervisor::{
    AdmissionStamp, ArmError, ArmSupervisor, SupervisorInner,
    OWNER_EXECUTOR, OWNER_LEGACY,
};
use crate::supervisor::config::CONFIG_PROPOSAL_BUDGET;

static NEXT_CONFIG_REQ_SEQ: AtomicU64 = AtomicU64::new(1);
static NEXT_CALIB_TXN_ID: AtomicU64 = AtomicU64::new(1);

/// Allocates a unique non-zero calibration transaction ID for executor-mode busy ownership.
pub fn next_calibration_txn_id() -> u64 {
    NEXT_CALIB_TXN_ID.fetch_add(1, Ordering::Relaxed)
}

/// Three-valued persistence commit outcome preserved across calibration and config commits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommitDisposition {
    /// Disk rename + parent-dir fsync + post-rename authority recheck + live activation succeeded.
    Applied,
    /// Failure occurred at or after rename (or activation outcome is unknown); old live state
    /// remains active and `HoldState::Reconciling` blocks motion until reconciled.
    Uncertain,
    /// Failure occurred strictly before any rename mutated disk; staged files were discarded
    /// and the reservation was released (`Cleared`).
    Unattempted,
}

/// Durable marker file written before multi-file calibration batch rename and removed only after
/// live activation (or bounded reconciliation) succeeds. If present at startup, the process
/// crashed mid-batch and startup enters `HoldState::Reconciling` to inhibit motion.
pub const CALIBRATION_PENDING_MARKER_NAME: &str = ".calibration_commit.pending";

pub(crate) fn pending_commit_marker_path(calib_path: &Path) -> PathBuf {
    let dir = calib_path
        .parent()
        .filter(|d| !d.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    dir.join(CALIBRATION_PENDING_MARKER_NAME)
}

pub(crate) fn write_pending_commit_marker(path: &Path, text: &str) -> Result<(), String> {
    let staged = StagedWrite::stage(path.to_path_buf(), text.to_string())?;
    let ds = staged.rename_only()?;
    match ds.sync_parent_dir() {
        PublishOutcome::Published => Ok(()),
        PublishOutcome::PublishedDurabilityUnconfirmed(w) => Err(format!(
            "pending commit marker {} renamed but parent dir fsync failed: {w}",
            path.display()
        )),
    }
}

pub(crate) fn remove_pending_commit_marker(path: &Path) -> Result<(), String> {
    match std::fs::remove_file(path) {
        Ok(()) => match PendingDirSync::for_path(path).sync_parent_dir() {
            PublishOutcome::Published => Ok(()),
            PublishOutcome::PublishedDurabilityUnconfirmed(w) => {
                tracing::warn!(
                    "Removed calibration commit marker {} but parent dir fsync failed: {w}",
                    path.display()
                );
                Err(format!(
                    "removed pending commit marker {} but parent dir fsync failed: {w}",
                    path.display()
                ))
            }
        },
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(e) => {
            tracing::warn!("Could not remove calibration commit marker {}: {e}", path.display());
            Err(format!("could not remove calibration commit marker {}: {e}", path.display()))
        }
    }
}

/// Default delay between bounded post-rename parent-directory fsync reconciliation retries.
pub const DEFAULT_RECONCILE_RETRY_INTERVAL: std::time::Duration = std::time::Duration::from_secs(1);
/// Default maximum number of bounded post-rename reconciliation retry attempts before latching a fault.
pub const DEFAULT_RECONCILE_MAX_ATTEMPTS: u32 = 30;

// ============================================================================
// Published configuration snapshot (off-RT readers + off-thread prepare)
// ============================================================================

/// Immutable off-RT snapshot of configuration and startup capabilities.
///
/// Updated off-thread before ownership transfer (`start_executor_with_hooks`) and
/// after every committed config, calibration, zero-check, or joint-zero mutation.
/// Never locked or mutated on `armd-control`.
#[derive(Clone)]
pub struct PublishedConfig {
    pub config: ArmConfig,
    pub follower: Option<String>,
    pub keepout: Keepout,
    pub collision_enabled: bool,
    pub collision_sensitivity: Sensitivity,
    pub collision_runtime: bool,
    pub collision_path: Option<PathBuf>,
    pub collision_revision: u64,
    pub collision_active: bool,
    pub collision_torque_feedback: bool,
    pub base_gripper_policy: Option<GripperPolicy>,
    pub gripper_calib_path: Option<PathBuf>,
    pub gripper_calib_offset: Option<f32>,
    pub gripper_calib_jaw_opening: Option<JawOpening>,
    pub gripper_calib_invalid: bool,
    pub gripper_calib_stale: Option<CalibrationStale>,
    pub gripper_calib_revision: u64,
    pub poses: PoseStore,
    pub sequences: SequenceStore,
    pub episodes: EpisodeStore,
    pub max_episode_s: f64,
    pub grasp_setup_revision: u64,
    pub tool_commissioning_path: Option<PathBuf>,
    pub tool_commissioning_confirmed_at: Option<String>,
    pub tool_commissioning_revision: u64,
    pub calibration: Option<CalibrationFile>,
    pub calibration_revision: u64,
    pub calibration_busy: bool,
    pub calibration_txn_id: Option<u64>,
    pub calibration_quarantined: bool,
    pub calibration_load_error: Option<String>,
    pub profile_camera_mount: CameraMount,
    pub profile_floor_z_m: Option<f32>,
    pub floor_source: &'static str,
    pub workspace_revision: u64,
    pub joint_zero_offsets: Vec<f32>,
    pub zero_convention: crate::joint_zero::ZeroConvention,
    pub operator_auth_enforced: bool,
    pub has_driver: bool,
    pub driver_torque_feedback: bool,
    pub driver_temperature_feedback: bool,
    pub driver_supports_automatic_unload: bool,
    pub driver_motor_current_cap: Option<Value>,
    pub driver_motor_zero_support: MotorZeroSupport,
    pub driver_config_error: Option<String>,
    pub gravity_compensated: bool,
    pub gravity_comp_supported: bool,
    pub torque_guard_thresholds: Vec<Option<f32>>,
    pub effective_planned_close_speed: f64,
    pub policy_live_allowed: bool,
    #[cfg(feature = "policy-runtime")]
    pub policy_ledger_path: Option<PathBuf>,
    pub fallback_measured_positions: Vec<f32>,
    pub cell: Option<Arc<crate::cell::CellContext>>,
}

impl PublishedConfig {
    /// Captures a complete configuration snapshot from `inner` off the RT thread.
    pub fn from_inner(inner: &SupervisorInner) -> Self {
        let driver_torque_feedback = inner
            .driver
            .as_ref()
            .is_some_and(|d| d.with_ref(|drv| drv.capabilities().torque_feedback).unwrap_or(false));
        let driver_temperature_feedback = inner
            .driver
            .as_ref()
            .is_none_or(|d| d.with_ref(|drv| drv.capabilities().temperature_feedback).unwrap_or(false));
        let driver_motor_current_cap = inner
            .driver
            .as_ref()
            .and_then(|d| d.with_ref(|drv| drv.motor_current_cap()).flatten())
            .and_then(|cap| serde_json::to_value(cap).ok());
        let mut sequences = inner.sequences.clone();
        sequences.pins = inner.active_sequence_pins().cloned();
        Self {
            config: inner.config.clone(),
            follower: inner.follower.clone(),
            keepout: inner.keepout.clone(),
            collision_enabled: inner.collision.enabled,
            collision_sensitivity: inner.collision.sensitivity,
            collision_runtime: inner.collision.runtime,
            collision_path: inner.collision.path.clone(),
            collision_revision: inner.collision.revision,
            collision_active: inner.collision.hold.is_some(),
            collision_torque_feedback: inner.collision.torque_feedback,
            base_gripper_policy: inner.base_gripper_policy.clone(),
            gripper_calib_path: inner.gripper_calib_path.clone(),
            gripper_calib_offset: inner.gripper_calib_offset,
            gripper_calib_jaw_opening: inner.gripper_calib_jaw_opening.clone(),
            gripper_calib_invalid: inner.gripper_calib_invalid,
            gripper_calib_stale: inner.gripper_calib_stale.clone(),
            gripper_calib_revision: inner.gripper_calib_revision,
            poses: inner.poses.clone(),
            sequences,
            episodes: inner.episodes.clone(),
            max_episode_s: inner.max_episode_s,
            grasp_setup_revision: inner.grasp_setup_revision,
            tool_commissioning_path: inner.tool_commissioning_path.clone(),
            tool_commissioning_confirmed_at: inner.tool_commissioning_confirmed_at.clone(),
            tool_commissioning_revision: inner.tool_commissioning_revision,
            calibration: inner.calibration.clone(),
            calibration_revision: inner.calibration_revision,
            calibration_busy: inner.calibration_busy,
            calibration_txn_id: inner.calibration_txn_id,
            calibration_quarantined: inner.calibration_quarantined,
            calibration_load_error: inner.calibration_load_error.clone(),
            profile_camera_mount: inner.profile_camera_mount.clone(),
            profile_floor_z_m: inner.profile_floor_z_m,
            floor_source: inner.floor_source,
            workspace_revision: inner.workspace_revision,
            joint_zero_offsets: inner.joint_zero_offsets.clone(),
            zero_convention: inner.zero_convention,
            operator_auth_enforced: inner.operator_auth_enforced,
            has_driver: inner.driver.is_some(),
            driver_torque_feedback,
            driver_temperature_feedback,
            driver_supports_automatic_unload: inner.supports_automatic_unload(),
            driver_motor_current_cap,
            driver_motor_zero_support: inner.motor_zero_support(),
            driver_config_error: inner.driver_config_error.clone(),
            gravity_compensated: inner.gravity_compensated,
            gravity_comp_supported: inner.gravity_comp_support().is_ok(),
            torque_guard_thresholds: inner.torque_guard_thresholds(),
            effective_planned_close_speed: inner.effective_planned_close_speed(),
            policy_live_allowed: {
                #[cfg(feature = "policy-runtime")]
                {
                    inner.policy_live_allowed()
                }
                #[cfg(not(feature = "policy-runtime"))]
                {
                    false
                }
            },
            #[cfg(feature = "policy-runtime")]
            policy_ledger_path: inner.policy_ledger_path.clone(),
            fallback_measured_positions: inner.measured_positions.clone(),
            cell: inner.cell.clone(),
        }
    }

    pub fn profile_keepout(&self) -> Vec<KeepoutBox> {
        self.keepout.profile.clone().unwrap_or_else(|| {
            self.config
                .workspace
                .as_ref()
                .map(|ws| ws.keepout_boxes.clone())
                .unwrap_or_default()
        })
    }

    pub fn keepout_state(&self) -> KeepoutState {
        let tag = |b: &KeepoutBox, source| SourcedBox {
            name: b.name.clone(),
            min_m: b.min_m,
            max_m: b.max_m,
            source,
        };
        let boxes = self
            .profile_keepout()
            .iter()
            .map(|b| tag(b, "profile"))
            .chain(self.keepout.runtime_boxes.iter().map(|b| tag(b, "runtime")))
            .collect();
        KeepoutState {
            boxes,
            source: if self.keepout.runtime { "runtime" } else { "profile" },
            revision: self.keepout.revision,
        }
    }

    pub fn workspace_policy(&self) -> Option<(&crate::model::WorkspacePolicy, &CameraMount)> {
        self.config.kinematics?;
        self.config.workspace.as_ref().map(|ws| (ws, &self.config.camera_mount))
    }

    pub fn collision_policy(&self) -> CollisionPolicy {
        self.config.collision.unwrap_or_default()
    }

    pub fn collision_available(&self) -> bool {
        self.config.gravity_model.is_some()
            && self.config.arm_dof() == 6
            && matches!(self.config.dof, 6 | 7)
            && self.collision_torque_feedback
    }

    pub fn collision_thresholds(&self) -> [f32; 6] {
        let p = self.collision_policy();
        let f = self.collision_sensitivity.factor();
        std::array::from_fn(|j| p.thresholds_nm[j] * f)
    }

    pub fn collision_config(&self, live_active: Option<bool>) -> CollisionConfig {
        let p = self.collision_policy();
        CollisionConfig {
            available: self.collision_available(),
            enabled: self.collision_enabled,
            sensitivity: self.collision_sensitivity,
            thresholds_nm: self.collision_thresholds(),
            profile_thresholds_nm: p.thresholds_nm,
            persist_ticks: p.persist_ticks,
            clear_ratio: p.clear_ratio,
            resume_after_s: p.resume_after_s,
            max_auto_resumes: p.max_auto_resumes,
            escalate_window_s: p.escalate_window_s,
            source: if self.collision_runtime { "runtime" } else { "profile" },
            active: if self.collision_enabled {
                live_active.unwrap_or(self.collision_active)
            } else {
                false
            },
            revision: self.collision_revision,
        }
    }

    pub fn gripper_calibration_response(&self) -> GripperCalibrationResponse {
        let policy = self.config.gripper_policy.clone();
        let calibrated = policy.as_ref().is_some_and(|p| p.is_calibrated());
        let jaw_opening = policy.as_ref().and_then(|p| p.jaw_opening.clone());
        GripperCalibrationResponse {
            offset_rad: self.gripper_calib_offset,
            jaw_opening,
            calibrated,
            policy,
            ts: None,
            stale: self.gripper_calib_stale.clone(),
            revision: self.gripper_calib_revision,
        }
    }

    pub fn tool_commissioning_response(&self) -> Value {
        json!({
            "active_tool": self.config.active_tool,
            "commissioning_required": self.config.active_tool.commissioning_required,
            "confirmed_at": self.tool_commissioning_confirmed_at,
            "revision": self.tool_commissioning_revision,
        })
    }

    pub fn joint_names(&self) -> Vec<String> {
        self.config.limits.iter().map(|l| l.name.clone()).collect()
    }

    pub fn grasp_setup(&self) -> Option<GraspSetup> {
        if !self.config.has_cartesian_with_gripper()
            || self.config.kinematics.is_none()
            || self.config.gripper_policy.is_none()
        {
            return None;
        }
        let mut s = self.config.grasp.clone()?;
        s.camera_tilt_deg = f64::from(self.config.camera_mount.tilt_deg);
        Some(s)
    }

    pub fn grasp_width_check(&self, max_grasp_width_m: f64) -> Option<GraspWidthCheck> {
        self.config
            .gripper_policy
            .as_ref()
            .map(|p| crate::model::check_grasp_width(p, max_grasp_width_m, self.gripper_calib_invalid))
    }

    pub fn camera_calibrated(&self) -> bool {
        self.calibration
            .as_ref()
            .is_some_and(|c| c.camera_mount.q_end_cam.is_some())
    }

    #[cfg(feature = "policy-runtime")]
    pub fn active_calibration_digest(&self) -> String {
        SupervisorInner::calibration_digest_of(self.calibration.as_ref())
    }

    pub fn pose_tcp(&self, q: &[f32]) -> Option<PoseTcp> {
        self.config.kinematics?;
        let pose = self.config.tcp_pose_f32(q)?;
        Some(PoseTcp {
            position: pose.p,
            rpy: crate::kinematics::r_to_rpy(&pose.r),
        })
    }

    pub fn recompute_pose_tcp_for(&self, arm_model: &ArmModel, p: &NamedPose) -> Option<PoseTcp> {
        let cfg_pid = self.config.profile_id.as_ref();
        let pose_pid = p.profile_id.as_ref().or(self.poses.list.profile_id.as_ref());
        if let (Some(pp), Some(cp)) = (pose_pid, cfg_pid) {
            if pp != cp {
                return None;
            }
        }
        let pose_arm_dof = p.arm_dof.or(self.poses.list.arm_dof);
        if self.config.arm_dof() == 5 && pose_arm_dof.is_none() {
            return None;
        }
        if pose_arm_dof.is_some_and(|ad| ad != self.config.arm_dof()) {
            return None;
        }
        if p.gripper_joint.is_some_and(|gj| Some(gj) != self.config.gripper_index()) {
            return None;
        }
        if let Some(ref jn) = p.joint_names {
            let cfg_jn = self.joint_names();
            if jn != &cfg_jn {
                return None;
            }
        }
        if p.positions.len() != self.config.dof {
            return None;
        }
        for (v, l) in p.positions.iter().zip(&self.config.limits) {
            if !v.is_finite() || *v < l.min_position || *v > l.max_position {
                return None;
            }
        }
        self.config.kinematics?;
        let mut tmp_cfg = self.config.clone();
        tmp_cfg.arm_model = arm_model.clone();
        let pose = tmp_cfg.tcp_pose_f32(&p.positions)?;
        Some(PoseTcp {
            position: pose.p,
            rpy: crate::kinematics::r_to_rpy(&pose.r),
        })
    }

    pub fn poses_response(&self) -> Value {
        let default_joint_names = self
            .poses
            .list
            .joint_names
            .clone()
            .unwrap_or_else(|| self.joint_names());
        let poses: Vec<Value> = self
            .poses
            .list
            .poses
            .iter()
            .map(|p| crate::poses::pose_response_value(p, &default_joint_names))
            .collect();
        let mut obj = serde_json::Map::new();
        if let Some(profile_id) = &self.poses.list.profile_id {
            obj.insert("profile_id".into(), Value::String(profile_id.clone()));
        }
        if let Some(arm_dof) = self.poses.list.arm_dof {
            obj.insert("arm_dof".into(), json!(arm_dof));
        }
        if let Some(gripper_joint) = self.poses.list.gripper_joint {
            obj.insert("gripper_joint".into(), json!(gripper_joint));
        }
        obj.insert(
            "joint_names".into(),
            json!(self.poses.list.joint_names.as_ref().unwrap_or(&default_joint_names)),
        );
        obj.insert("poses".into(), Value::Array(poses));
        obj.insert("revision".into(), json!(self.poses.revision));
        Value::Object(obj)
    }

    /// True when sequence `id` is actively running in this published snapshot.
    /// `ArmSupervisor::published_config` guarantees `self.sequences.pins` holds only active pins.
    pub fn is_sequence_running(&self, id: &str) -> bool {
        self.sequences
            .is_sequence_running_with_pins(id, self.sequences.pins.as_ref())
    }

    pub fn sequence_view(&self, s: &Sequence) -> Value {
        let mut stale = Vec::new();
        let mut missing = Vec::new();
        for (i, step) in s.steps.iter().enumerate() {
            match step {
                Step::Pose { pose_id, .. } => match self.poses.by_id(pose_id) {
                    None => missing.push(i),
                    Some(p) if p.stale.is_some() => stale.push(i),
                    _ => {}
                },
                Step::Replay { episode_id, .. } => match self.episodes.get(episode_id) {
                    None => missing.push(i),
                    Some(e) if e.joint_zero.stale_against(&self.joint_zero_offsets) => stale.push(i),
                    _ => {}
                },
                _ => {}
            }
        }
        let mut v = serde_json::to_value(s).unwrap_or_default();
        v["stale_steps"] = json!(stale);
        v["missing_steps"] = json!(missing);
        v["running"] = json!(self.is_sequence_running(&s.id));
        v
    }

    pub fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::from_config(&self.config);
        caps.gravity_comp = self.gravity_comp_supported;
        caps.collision_guard &= self.driver_torque_feedback;
        caps.torque_guard &= self.driver_torque_feedback;
        caps.power_budget &= self.driver_torque_feedback;
        caps.auto_yaw = self.config.arm_dof() == 5;
        #[cfg(feature = "policy-runtime")]
        if let Some(ref mut pr) = caps.policy_runtime {
            pr.modes = if self.policy_live_allowed {
                vec!["shadow".to_string(), "live_sim".to_string()]
            } else {
                vec!["shadow".to_string(), "live_gated".to_string()]
            };
        }
        caps.calibration_tools = Some(self.calibration_tools_with(&caps));
        if let Some(t) = caps.teach.as_mut() {
            t.episodes = self.episodes.enabled();
        }
        caps.cell = self.cell.as_ref().map(|c| c.capabilities());
        caps
    }

    pub fn calibration_tools(&self) -> CalibrationTools {
        self.calibration_tools_with(&Capabilities::from_config(&self.config))
    }

    pub fn calibration_tools_with(&self, caps: &Capabilities) -> CalibrationTools {
        let no_grasp = "this profile has no grasp setup (the camera calibration needs one)";
        let kin_reason = if self.config.arm_model.chain().is_some() {
            "this profile uses a URDF chain; the kinematic correction applies to the built-in model only"
        } else {
            no_grasp
        };
        let mut support = self.driver_motor_zero_support.clone();
        let (motor, software) = (support.supported, self.config.zero_reference.is_some());
        let methods: Vec<String> = [(software, "software"), (motor, "motor")]
            .into_iter()
            .filter(|(ok, _)| *ok)
            .map(|(_, m)| m.to_string())
            .collect();
        if software && !motor {
            support = MotorZeroSupport {
                supported: true,
                notes: support
                    .reason
                    .take()
                    .map(|r| vec![format!("motor zero: {r}")])
                    .unwrap_or_default(),
                ..Default::default()
            };
        }
        let any = motor || software;
        let motor_zero = MotorZeroTool {
            joints: if any { (1..=self.config.dof).collect() } else { Vec::new() },
            requires: if any {
                ["lease", "torque_off", "no_action", "no_fault"]
                    .map(String::from)
                    .to_vec()
            } else {
                Vec::new()
            },
            max_offset_deg: crate::model::MOTOR_ZERO_MAX_OFFSET_DEG,
            verify_mrad: crate::model::MOTOR_ZERO_VERIFY_MRAD,
            invalidates: if any {
                ["kinematics", "camera", "poses", "gripper_calibration"]
                    .map(String::from)
                    .to_vec()
            } else {
                Vec::new()
            },
            zero_pose: self.config.zero_pose.clone(),
            motor,
            software,
            methods,
            support,
        };
        let policy = self
            .config
            .gripper_policy
            .as_ref()
            .filter(|_| self.config.gripper_index().is_some());
        let gripper = GripperCalibrationTool {
            supported: policy.is_some(),
            zero_check: policy.is_some_and(|p| p.zero_check_supported),
            calibration: policy.is_some(),
            reason: policy
                .is_none()
                .then(|| "this profile has no gripper joint with a gripper policy".to_string()),
        };
        CalibrationTools {
            camera: CalibrationToolInfo::new(caps.calibration_apply, Vec::new(), no_grasp),
            kinematics: CalibrationToolInfo::new(caps.calibration_kinematics, vec![2, 3, 4, 5], kin_reason),
            motor_zero,
            gripper,
        }
    }

    pub fn watchdog_effect(&self) -> &'static str {
        if self.config.resting_pose.is_some() && self.driver_supports_automatic_unload {
            "park_then_torque_off"
        } else {
            "hold_and_fault"
        }
    }

    pub fn refresh_effective_floor_in_place(&mut self) {
        let Some(profile) = self.profile_floor_z_m else {
            return;
        };
        let (_, fp) = crate::calibration::file_fingerprints(
            self.calibration.as_ref(),
            &self.config.camera_mount,
            &self.joint_zero_offsets,
            Some(&self.config.active_tool.config_digest),
        );
        let legacy_fp = crate::calibration::legacy_file_fingerprint(
            self.calibration.as_ref(),
            &self.config.camera_mount,
            &self.joint_zero_offsets,
        );
        let (floor, source, _) = crate::calibration::effective_floor_for_with_legacy(
            profile,
            self.calibration.as_ref(),
            &fp,
            Some(&legacy_fp),
            &self.config.active_tool,
        );
        self.floor_source = source;
        if let Some(ws) = self.config.workspace.as_mut() {
            if ws.floor_z_m != floor {
                ws.floor_z_m = floor;
            }
        }
    }
}

struct PublishedConfigSlot {
    config: Arc<PublishedConfig>,
    legacy_lock_gen: u64,
}

/// Off-RT published configuration store shared by REST readers, off-thread prepare,
/// commit orchestrators, and `armd-persist` (`zero_check`). Never touched on `armd-control`.
pub struct PublishedConfigHub {
    slot: RwLock<PublishedConfigSlot>,
    available: AtomicBool,
}

impl PublishedConfigHub {
    pub fn new(initial: PublishedConfig, legacy_lock_gen: u64) -> Self {
        Self {
            slot: RwLock::new(PublishedConfigSlot {
                config: Arc::new(initial),
                legacy_lock_gen,
            }),
            available: AtomicBool::new(true),
        }
    }

    pub fn latest(&self) -> Option<Arc<PublishedConfig>> {
        if !self.available.load(Ordering::Acquire) {
            return None;
        }
        let g = self.slot.read().unwrap_or_else(|e| e.into_inner());
        Some(Arc::clone(&g.config))
    }

    pub fn latest_with_gen(&self) -> (Arc<PublishedConfig>, u64) {
        let g = self.slot.read().unwrap_or_else(|e| e.into_inner());
        (Arc::clone(&g.config), g.legacy_lock_gen)
    }

    pub fn set_available(&self, available: bool) {
        self.available.store(available, Ordering::Release);
    }

    pub fn publish(&self, next: PublishedConfig, legacy_lock_gen: u64) {
        let mut g = self.slot.write().unwrap_or_else(|e| e.into_inner());
        g.config = Arc::new(next);
        g.legacy_lock_gen = legacy_lock_gen;
        self.available.store(true, Ordering::Release);
    }

    /// Publishes a Legacy read snapshot only if `legacy_lock_gen` is strictly newer than the
    /// currently published generation, preventing a slower concurrent Legacy reader from
    /// overwriting a newer snapshot after dropping its read guard.
    pub fn publish_if_newer(&self, next: PublishedConfig, legacy_lock_gen: u64) -> Arc<PublishedConfig> {
        let mut g = self.slot.write().unwrap_or_else(|e| e.into_inner());
        if legacy_lock_gen > g.legacy_lock_gen {
            g.config = Arc::new(next);
            g.legacy_lock_gen = legacy_lock_gen;
        }
        Arc::clone(&g.config)
    }

    pub fn update(&self, mutate: impl FnOnce(&mut PublishedConfig)) {
        let mut g = self.slot.write().unwrap_or_else(|e| e.into_inner());
        let mut next = (*g.config).clone();
        mutate(&mut next);
        g.config = Arc::new(next);
    }

    /// Updates published gripper calibration state off-thread after `armd-persist`
    /// activates a `gripper_zero_check(store_offset=true)` write.
    pub fn apply_zero_check_commit(
        &self,
        offset_rad: f32,
        policy: GripperPolicy,
        jaw_opening: Option<JawOpening>,
        new_revision: u64,
    ) {
        self.update(|cfg| {
            if cfg.base_gripper_policy.is_none() {
                cfg.base_gripper_policy = cfg.config.gripper_policy.clone();
            }
            cfg.config.gripper_policy = Some(policy);
            cfg.gripper_calib_offset = Some(offset_rad);
            if jaw_opening.is_some() {
                cfg.gripper_calib_jaw_opening = jaw_opening;
            }
            cfg.gripper_calib_invalid = false;
            cfg.gripper_calib_revision = new_revision;
        });
    }

    /// Precomputes `(effective_floor, floor_source)` off-thread for candidate `joint_zero_offsets`
    /// so `SetJointZeroOffsets` on `armd-control` never allocates strings to recompute fingerprints.
    pub fn precompute_effective_floor_for_offsets(&self, offsets: &[f32]) -> Option<(f32, &'static str)> {
        let cfg = self.latest()?;
        let profile = cfg.profile_floor_z_m?;
        let (_, fp) = crate::calibration::file_fingerprints(
            cfg.calibration.as_ref(),
            &cfg.config.camera_mount,
            offsets,
            Some(&cfg.config.active_tool.config_digest),
        );
        let legacy_fp = crate::calibration::legacy_file_fingerprint(
            cfg.calibration.as_ref(),
            &cfg.config.camera_mount,
            offsets,
        );
        let (floor, source, _) = crate::calibration::effective_floor_for_with_legacy(
            profile,
            cfg.calibration.as_ref(),
            &fp,
            Some(&legacy_fp),
            &cfg.config.active_tool,
        );
        Some((floor, source))
    }

    /// Updates published joint-zero offsets off-thread after `SetJointZeroOffsets` succeeds.
    pub fn apply_joint_zero_offsets(&self, offsets: &[f32]) {
        self.update(|cfg| {
            let changed = cfg.joint_zero_offsets.as_slice() != offsets;
            if changed {
                cfg.calibration_revision += 1;
            }
            if cfg.joint_zero_offsets.len() > 5
                && offsets.len() > 5
                && cfg.joint_zero_offsets[5] != offsets[5]
            {
                cfg.zero_convention = Default::default();
            }
            cfg.joint_zero_offsets = offsets.to_vec();
            if changed {
                cfg.refresh_effective_floor_in_place();
            }
        });
    }
}

/// 503 `CONFIG_UNAVAILABLE`: published configuration state is unavailable.
pub fn config_unavailable() -> ArmError {
    ArmError::Rejected {
        status: 503,
        code: "CONFIG_UNAVAILABLE".into(),
        message: "Published configuration state is unavailable while ownership is transitioning or failed".into(),
    }
}

/// 503 `EXECUTOR_CAPABILITY_UNSUPPORTED`: fail-closed refusal for hardware mutations
/// unsupported on the RT executor thread.
pub fn executor_capability_unsupported(detail: &str) -> ArmError {
    ArmError::Rejected {
        status: 503,
        code: "EXECUTOR_CAPABILITY_UNSUPPORTED".into(),
        message: detail.into(),
    }
}

// ============================================================================
// Stack-only RT errors (0 alloc on armd-control)
// ============================================================================

/// Allocation-free refusal produced on `armd-control` and formatted into
/// `ArmError` / `CalibError` strictly off-thread.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConfigError {
    RevisionMismatch { expected: u64, current: u64 },
    Busy(&'static str),
    Unsupported(&'static str),
    MotionBusy { subject: &'static str, reason: &'static str },
    InvalidCommand(&'static str),
    DriverLimitRejected(BoundedText<256>),
    PoseInUse,
    SequenceRefMissing { step: usize, is_pose: bool },
    SequenceRunning,
    PlanStale(&'static str),
    LeaseLost,
    EpochMismatch { current_epoch: u64 },
    Stopped,
    StampConsumed,
    WrongTarget,
    DeadlineExpired,
    ConnectionClosed,
    Reconciling(&'static str),
    NotHeld,
    ExecutorCapabilityUnsupported(&'static str),
    GravityCompRejected {
        code: BoundedText<48>,
        message: BoundedText<256>,
    },
    CalibPrecondition {
        status: u16,
        code: &'static str,
        message: &'static str,
    },
    CalibPreconditionText {
        status: u16,
        code: &'static str,
        message: BoundedText<256>,
    },
    OwnerStopped,
    OwnerQueueFull,
}

impl ConfigError {
    pub fn to_arm_error(&self) -> ArmError {
        match self {
            Self::RevisionMismatch { expected, current } => ArmError::PolicyRevisionMismatch {
                expected: *expected,
                current: *current,
            },
            Self::Busy(msg) => ArmError::Rejected {
                status: 409,
                code: "CONFIG_BUSY".into(),
                message: (*msg).into(),
            },
            Self::Unsupported(msg) => MotionError::Unsupported((*msg).into()).into(),
            Self::MotionBusy { subject, reason } => {
                MotionError::MotionBusy(format!("{subject} cannot change while {reason}")).into()
            }
            Self::InvalidCommand(msg) => MotionError::InvalidCommand((*msg).into()).into(),
            Self::DriverLimitRejected(msg) => MotionError::InvalidCommand(msg.as_str().into()).into(),
            Self::PoseInUse => ArmError::Rejected {
                status: 409,
                code: "IN_USE".into(),
                message: "Pose is referenced by a sequence".into(),
            },
            Self::SequenceRefMissing { step, is_pose } => ArmError::detailed(
                422,
                "SEQUENCE_REF_MISSING",
                format!(
                    "step {step}: referenced {} no longer exists",
                    if *is_pose { "pose" } else { "episode" }
                ),
                json!({ "step": step }),
            ),
            Self::SequenceRunning => ArmError::Rejected {
                status: 409,
                code: "SEQUENCE_RUNNING".into(),
                message: "the sequence is running; stop it first".into(),
            },
            Self::PlanStale(msg) => MotionError::PlanStale((*msg).into()).into(),
            Self::LeaseLost | Self::ConnectionClosed => ArmError::Rejected {
                status: 409,
                code: "LEASE_LOST".into(),
                message: "This connection lost the lease while persisting configuration".into(),
            },
            Self::EpochMismatch { .. } => ArmError::Rejected {
                status: 409,
                code: "EPOCH_MISMATCH".into(),
                message: "Command epoch is no longer current".into(),
            },
            Self::Stopped => ArmError::Rejected {
                status: 409,
                code: "STOPPED".into(),
                message: "configuration change overtaken by an authorized stop".into(),
            },
            Self::StampConsumed => ArmError::Rejected {
                status: 409,
                code: "STAMP_CONSUMED".into(),
                message: "configuration admission stamp was already consumed".into(),
            },
            Self::WrongTarget => ArmError::Rejected {
                status: 409,
                code: "WRONG_TARGET".into(),
                message: "configuration proposal targets another arm or stale boot identity".into(),
            },
            Self::DeadlineExpired => ArmError::Rejected {
                status: 409,
                code: "DEADLINE_EXPIRED".into(),
                message: "configuration proposal deadline expired before reservation".into(),
            },
            Self::Reconciling(code) => ArmError::Rejected {
                status: 409,
                code: (*code).into(),
                message: format!(
                    "configuration persisted on disk but activation failed ({code}); reconciling required"
                ),
            },
            Self::NotHeld => ArmError::Internal("configuration operation attempted without active reservation".into()),
            Self::ExecutorCapabilityUnsupported(msg) => executor_capability_unsupported(msg),
            Self::GravityCompRejected { code, message } => {
                let status = match code.as_str() {
                    "UNSUPPORTED" | "GRAVITY_MODEL_UNAVAILABLE" => 422,
                    "NOT_ARMED" => 412,
                    "INVALID_COMMAND" => 400,
                    _ => 409,
                };
                ArmError::Rejected {
                    status,
                    code: code.as_str().into(),
                    message: message.as_str().into(),
                }
            }
            Self::CalibPrecondition { status, code, message } => ArmError::Rejected {
                status: *status,
                code: (*code).into(),
                message: (*message).into(),
            },
            Self::CalibPreconditionText { status, code, message } => ArmError::Rejected {
                status: *status,
                code: (*code).into(),
                message: message.as_str().into(),
            },
            Self::OwnerStopped => ArmError::Rejected {
                status: 503,
                code: "EXECUTOR_STOPPED".into(),
                message: "RT executor stopped before completing configuration request".into(),
            },
            Self::OwnerQueueFull => ArmError::Rejected {
                status: 503,
                code: "BACKLOG_FULL".into(),
                message: "RT executor maintenance queue is full".into(),
            },
        }
    }
}

// ============================================================================
// Single-entry-point authority check (reuses A1 AdmissionStamp/ClosedConnections/ConsumedStamps)
// ============================================================================

/// Caller authority context carried from ingress through staging, reservation, and activation.
#[derive(Debug, Clone, Copy)]
pub struct ConfigAuthContext {
    pub admission: AdmissionStamp,
    /// `Some((connection_id_label, lease_generation))` when a WS lease is required;
    /// `None` for lease-free REST writes (`POST /api/v1/tool/commissioning/confirm`).
    pub lease_holder: Option<(HolderLabel, u64)>,
}

/// Single authority check for `ReserveConfig`, `VerifyAuthority`, `ActivateConfig`,
/// and `FinishReconcile`.
///
/// Reuses A1's `AdmissionStamp`, `ClosedConnections`, and `ConsumedStamps` alongside
/// `ConfigProposal::check` (`arm`, `boot`, `deadline`, `stop_generation`, `epoch`,
/// `lease_generation`) with zero heap allocations.
pub(crate) fn check_config_authority(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    proposal: &ConfigProposal,
    auth: &ConfigAuthContext,
    enforce_unused_stamp: bool,
) -> Result<(), ConfigError> {
    check_config_authority_inner(core, ctx, proposal, auth, enforce_unused_stamp, true)
}

pub(crate) fn check_config_authority_inner(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    proposal: &ConfigProposal,
    auth: &ConfigAuthContext,
    enforce_unused_stamp: bool,
    enforce_deadline: bool,
) -> Result<(), ConfigError> {
    if proposal.stamp.boot.is_zero() || ctx.boot.is_zero() {
        return Err(ConfigError::WrongTarget);
    }
    if auth.admission.connection_id != 0 && core.is_connection_closed(auth.admission.connection_id) {
        return Err(ConfigError::ConnectionClosed);
    }
    if core.estop_request_unapplied() {
        return Err(ConfigError::EpochMismatch {
            current_epoch: core.safety.epoch_id(),
        });
    }
    if core.urgent_stop_unapplied() {
        return Err(ConfigError::Stopped);
    }
    let current = core.authority(ctx);
    let check_now = if enforce_deadline {
        ctx.now
    } else {
        proposal.stamp.deadline
    };
    if let Some(code) = proposal.check(ctx.arm, ctx.boot, check_now, &current) {
        return Err(match code {
            "WRONG_TARGET" => ConfigError::WrongTarget,
            "DEADLINE_EXPIRED" => ConfigError::DeadlineExpired,
            "STOPPED" => ConfigError::Stopped,
            "EPOCH_MISMATCH" => ConfigError::EpochMismatch {
                current_epoch: current.epoch,
            },
            "LEASE_LOST" => ConfigError::LeaseLost,
            _ => ConfigError::WrongTarget,
        });
    }
    if let Some((conn, gen)) = auth.lease_holder {
        if gen == 0 || !core.validate_lease_at(conn.as_str(), gen, ctx.now) {
            return Err(ConfigError::LeaseLost);
        }
    } else if proposal.domain == ConfigDomain::Calibration {
        return Err(ConfigError::LeaseLost);
    }
    if enforce_unused_stamp && core.is_stamp_consumed(auth.admission.request_id) {
        return Err(ConfigError::StampConsumed);
    }
    Ok(())
}

// ============================================================================
// Request, Candidate, Reply, and Exchange payloads (zero-alloc/zero-free on RT)
// ============================================================================

/// Domain-specific precheck executed atomically on the owner inside [`ReserveConfig`].
pub enum ReservePrecheck {
    Keepout,
    Collision { enabling: bool },
    GripperCalibration { candidate_policy: Option<GripperPolicy> },
    PosesSave,
    PosesDelete { deleting_pose_id: HolderLabel },
    Sequences {
        steps: Option<Vec<Step>>,
        deleting_sequence_id: Option<HolderLabel>,
        /// Pose + replay-episode pins installed atomically on the owner when `Reserve` grants.
        reserved_pins: Option<crate::sequences::ReservedSequencePins>,
    },
    GraspSetup {
        new_camera_tilt_deg: f32,
        allow_during_calibration_busy: bool,
    },
    ToolCommissioning {
        tool_id: HolderLabel,
        tool_config_digest: HolderLabel,
    },
    Calibration {
        allow_during_calibration_busy: bool,
        expected_txn_id: Option<u64>,
        operator_grant: crate::auth::OperatorGrant,
        require_kinematics_disarmed: bool,
        expected_grasp_setup_revision: Option<u64>,
    },
    SoftwareJointZero {
        allow_during_calibration_busy: bool,
        expected_txn_id: Option<u64>,
        operator_grant: crate::auth::OperatorGrant,
        expected_gripper_calib_revision: Option<u64>,
        is_hardware_motor_zero: bool,
    },
    MitCommissioning {
        allow_during_calibration_busy: bool,
        expected_txn_id: Option<u64>,
        operator_grant: crate::auth::OperatorGrant,
        require_driver: bool,
    },
}

/// Zero-allocation domain precondition bundle rechecked at pre-rename `VerifyAuthority`.
#[derive(Debug, Clone, Copy, Default)]
pub struct VerifyDomainChecks {
    pub expected_grasp_setup_revision: Option<u64>,
    pub expected_gripper_calib_revision: Option<u64>,
    pub require_workspace_unblocked: bool,
    pub require_kinematics_disarmed: bool,
    pub require_software_zero_idle: bool,
    pub require_mit_torque_off: bool,
    pub expected_txn_id: Option<u64>,
    pub operator_grant: Option<crate::auth::OperatorGrant>,
}

/// Kind of executor-mode calibration transaction started via `ConfigRequest::BeginCalibrationTxn`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationTxnKind {
    /// Authoritative live-authority + `calibration_busy` check before body parsing for zero paths.
    IngressBusyCheck,
    /// Authoritative preflight (`begin_checks` + live lease/epoch/stop) before body parsing.
    CameraPreflight,
    CameraApply {
        require_kinematics_disarmed: bool,
        dry_run: bool,
    },
    CameraRestore,
    SoftwareZero {
        set_busy: bool,
    },
    MotorZero {
        set_busy: bool,
    },
    JointZeroRestore,
    MitTimeout {
        dry_run: bool,
    },
    MitQualification,
}

/// Stack-only snapshot returned by `BeginCalibrationTxn` / `SampleCalibrationState` (0 alloc on RT).
#[derive(Debug, Clone, Copy)]
pub struct CalibrationStateSample {
    pub positions: [f32; crate::rt_core::mailbox::MAX_DOF],
    pub dof: usize,
    pub has_driver: bool,
    pub feedback_valid: bool,
    pub feedback_instants: [Option<Instant>; crate::rt_core::mailbox::MAX_DOF],
    pub is_torqued: bool,
    pub safety_state: SafetyStateTag,
    pub epoch_id: u64,
    pub calibration_revision: u64,
}

/// Pre-built candidate payload swapped into `SupervisorInner` in-place during [`ActivateConfig`].
pub enum ConfigCandidate {
    Keepout {
        profile_boxes: Vec<KeepoutBox>,
        runtime_boxes: Vec<KeepoutBox>,
        effective_boxes: Vec<KeepoutBox>,
        ws_eval_bufs: EvalBufs,
    },
    Collision {
        enabled: Option<bool>,
        sensitivity: Option<Sensitivity>,
    },
    GripperCalibration {
        offset_rad: Option<f32>,
        policy: GripperPolicy,
        jaw_opening: Option<JawOpening>,
        clear_stale: bool,
    },
    Poses {
        next_list: PoseList,
    },
    Sequences {
        next_file: SequenceFile,
    },
    GraspSetup {
        setup: GraspSetup,
        ws_eval_bufs: Option<EvalBufs>,
    },
    ToolCommissioning {
        tool_id: HolderLabel,
        tool_config_digest: HolderLabel,
        confirmed_at: String,
    },
    Calibration {
        calibration: Option<CalibrationFile>,
        #[cfg(feature = "policy-runtime")]
        calibration_digest: String,
        camera_mount: CameraMount,
        arm_model: ArmModel,
        kinematics_changed: bool,
        next_pose_list: Option<PoseList>,
        grasp_setup: Option<GraspSetup>,
        expected_grasp_setup_revision: Option<u64>,
        expected_txn_id: Option<u64>,
        operator_grant: crate::auth::OperatorGrant,
        effective_floor: Option<(f32, &'static str)>,
        ws_eval_bufs: Option<EvalBufs>,
    },
    SoftwareJointZero {
        new_offsets: [f32; crate::rt_core::mailbox::MAX_DOF],
        dof: usize,
        is_hardware_motor_zero: bool,
        zeroed_mask: u64,
        raw_after: [f32; crate::rt_core::mailbox::MAX_DOF],
        calibration: Option<CalibrationFile>,
        #[cfg(feature = "policy-runtime")]
        calibration_digest: String,
        next_arm_model: Option<ArmModel>,
        next_pose_list: Option<(PoseList, bool)>,
        next_gripper_stale: Option<CalibrationStale>,
        expected_gripper_calib_revision: Option<u64>,
        expected_txn_id: Option<u64>,
        operator_grant: crate::auth::OperatorGrant,
        effective_floor: Option<(f32, &'static str)>,
        ws_eval_bufs: Option<EvalBufs>,
        reset_gripper_baseline: bool,
        replacement_baseline: Option<crate::gripper_baseline::FreeTorqueBaseline>,
        replacement_baseline_pending: Option<Vec<crate::gripper_baseline::MoveRecord>>,
    },
    MitQualification {
        calibration: Option<CalibrationFile>,
        #[cfg(feature = "policy-runtime")]
        calibration_digest: String,
        expected_txn_id: Option<u64>,
        operator_grant: crate::auth::OperatorGrant,
    },
}

/// Reserve `proposal.domain` at `proposal.expected_revision` after authority + domain prechecks.
pub struct ReserveConfig {
    pub proposal: ConfigProposal,
    pub auth: ConfigAuthContext,
    pub precheck: ReservePrecheck,
    /// When `Some(rev)`, requires `core.workspace_revision == rev` (for candidates carrying
    /// workspace-dependent `EvalBufs`, floor, or camera-mount inputs).
    pub expected_workspace_revision: Option<u64>,
    /// When `Some(rev)`, requires `core.poses.revision == rev` (for candidates carrying
    /// precomputed `pose_tcps`).
    pub expected_poses_revision: Option<u64>,
}

/// Mark a granted reservation renamed on disk (`Reserved` -> `PersistedAwaitingActivation`).
pub struct PersistedConfig {
    pub reservation: Reservation,
    pub durability: Durability,
}

/// Activate a held reservation (volatile or persisted) by swapping `candidate` in-place.
pub struct ActivateConfig {
    pub reservation: Reservation,
    pub auth: ConfigAuthContext,
    /// Non-None if off-thread operator auth or connection lease generation was revoked
    /// after rename and before activation: owner enters `Reconciling` without mutating state.
    pub revoked_off_thread: Option<&'static str>,
    pub candidate: ConfigCandidate,
    /// Confirmed durability achieved before activation (`Durability::Durable` after parent-dir
    /// fsync succeeds, or `Durability::Volatile` when no disk file was written).
    pub durability: Durability,
    pub expected_workspace_revision: Option<u64>,
    pub expected_poses_revision: Option<u64>,
}

/// Release a held reservation (`Cleared` before rename; `Reconciling` after rename).
pub struct ReleaseConfig {
    pub reservation: Reservation,
}

/// Complete post-rename reconciliation once parent-directory fsync succeeds: rechecks live
/// authority (allowing the bounded reconciliation retry window) and domain invariants, swaps
/// `candidate` in-place, and calls `ConfigAuthority::finish_reconcile` to lift motion inhibit.
pub struct FinishReconcileConfig {
    pub domain: ConfigDomain,
    pub expected_from: u64,
    pub proposal: ConfigProposal,
    pub auth: ConfigAuthContext,
    pub candidate: ConfigCandidate,
    pub expected_workspace_revision: Option<u64>,
    pub expected_poses_revision: Option<u64>,
    /// When `Some(tid)`, atomically installs `calibration_busy = true; calibration_txn_id = Some(tid)`
    /// as `HoldState::Reconciling` clears so no new calibration can start before this retry unlinks
    /// its pending commit marker and calls `EndCalibrationTxn { txn_id: tid, force_reconcile: false }`.
    pub cleanup_marker_txn_id: Option<u64>,
}

/// Owner-routed gravity compensation request (mock zero-alloc; hardware driver fails closed in Executor).
pub enum GravityCompOp {
    Enter {
        admission: AdmissionStamp,
        lease_holder: Option<(HolderLabel, u64)>,
        policy: Option<crate::model::GravityCompPolicy>,
        scale: Option<f32>,
        damping_kd: Option<[f32; 6]>,
    },
    Exit {
        lease_holder: Option<(HolderLabel, u64)>,
        code: &'static str,
        message: &'static str,
    },
}

/// Allocation-free tag mirroring `SafetyState` for owner replies without cloning `FaultReason`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyStateTag {
    Booting,
    Idle,
    Armed,
    Moving,
    EStop,
    Fault,
}

impl SafetyStateTag {
    pub fn from_state(state: &crate::safety::SafetyState) -> Self {
        match state {
            crate::safety::SafetyState::Booting => Self::Booting,
            crate::safety::SafetyState::Idle => Self::Idle,
            crate::safety::SafetyState::Armed => Self::Armed,
            crate::safety::SafetyState::Moving => Self::Moving,
            crate::safety::SafetyState::EStop => Self::EStop,
            crate::safety::SafetyState::Fault(_) => Self::Fault,
        }
    }

    pub fn to_json_with_hub(self, sup: &ArmSupervisor) -> Value {
        match self {
            Self::Booting => json!("Booting"),
            Self::Idle => json!("Idle"),
            Self::Armed => json!("Armed"),
            Self::Moving => json!("Moving"),
            Self::EStop => json!("EStop"),
            Self::Fault => sup
                .hub()
                .latest()
                .map_or_else(|| json!("Fault"), |s| json!(s.obs.safety_state)),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GravityCompReply {
    pub active: bool,
    pub epoch_id: u64,
    pub is_torqued: bool,
    pub safety_state: SafetyStateTag,
}

/// Outcome of a successful [`ActivateConfig`].
#[derive(Debug, Clone, Copy)]
pub struct ConfigActivated {
    pub revision: u64,
    pub durability: Durability,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
    pub poses_revision: u64,
    pub gripper_calib_revision: u64,
    pub grasp_setup_revision: u64,
    pub calibration_revision: u64,
    pub epoch_id: u64,
    pub is_torqued: bool,
    pub safety_state: SafetyStateTag,
    pub gravity_comp_active: bool,
    pub activated_at: Instant,
    pub limit_min: [f32; crate::rt_core::mailbox::MAX_DOF],
    pub limit_max: [f32; crate::rt_core::mailbox::MAX_DOF],
    pub limit_vel: [f32; crate::rt_core::mailbox::MAX_DOF],
    pub limit_len: usize,
}

impl ConfigActivated {
    fn from_core(
        core: &SupervisorInner,
        revision: u64,
        durability: Durability,
        activated_at: Instant,
    ) -> Self {
        let mut limit_min = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let mut limit_max = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let mut limit_vel = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let limit_len = core
            .config
            .limits
            .len()
            .min(crate::rt_core::mailbox::MAX_DOF);
        for i in 0..limit_len {
            limit_min[i] = core.config.limits[i].min_position;
            limit_max[i] = core.config.limits[i].max_position;
            limit_vel[i] = core.config.limits[i].max_velocity;
        }
        Self {
            revision,
            durability,
            workspace_revision: core.workspace_revision,
            motion_config_revision: core.motion_config_revision,
            poses_revision: core.poses.revision,
            gripper_calib_revision: core.gripper_calib_revision,
            grasp_setup_revision: core.grasp_setup_revision,
            calibration_revision: core.calibration_revision,
            epoch_id: core.safety.epoch_id(),
            is_torqued: core.is_torqued,
            safety_state: SafetyStateTag::from_state(core.safety.state()),
            gravity_comp_active: core.gravity_comp_active(),
            activated_at,
            limit_min,
            limit_max,
            limit_vel,
            limit_len,
        }
    }
}

/// All owner-side configuration and companion maintenance requests routed via `MaintenanceOp::Config`.
pub enum ConfigRequest {
    Reserve(ReserveConfig),
    /// Re-verifies live owner authority + hold grant + dependency revisions immediately before
    /// atomic filesystem commit (`rename_only` or `remove_file`), zero-alloc on `armd-control`.
    VerifyAuthority {
        domain: ConfigDomain,
        grant: GrantId,
        proposal: ConfigProposal,
        auth: ConfigAuthContext,
        expected_workspace_revision: Option<u64>,
        expected_poses_revision: Option<u64>,
        domain_checks: VerifyDomainChecks,
    },
    Persisted(PersistedConfig),
    Activate(ActivateConfig),
    FinishReconcile(FinishReconcileConfig),
    LatchConfigReconcileFault {
        domain: ConfigDomain,
        message: BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
    },
    Release(ReleaseConfig),
    ForceReconcile(ConfigDomain),
    SampleTableCapture,
    GravityComp(GravityCompOp),
    InspectHold(ConfigDomain),
    /// Zero-alloc check on live owner whether `episode_id` may be deleted (not referenced by
    /// saved sequences, active running sequence pins, or an active `Sequences` save reservation).
    CheckEpisodeDeleteAllowed(HolderLabel),
    /// Zero-alloc start/preflight of an executor calibration or software-zero transaction.
    BeginCalibrationTxn {
        txn_id: u64,
        auth: ConfigAuthContext,
        operator_grant: crate::auth::OperatorGrant,
        expected_epoch: u64,
        expected_stop_gen: u64,
        kind: CalibrationTxnKind,
    },
    /// Zero-alloc transaction-bound cleanup of `calibration_busy` / `motor_zero_busy`.
    /// When `force_reconcile` is true, atomically enters `HoldState::Reconciling` before clearing
    /// busy flags so an abnormal drop after remote Vision mutation never lifts motion inhibit.
    EndCalibrationTxn {
        txn_id: u64,
        force_reconcile: bool,
    },
    /// Zero-alloc live authority + zero state check + joint reading/freshness sample.
    SampleCalibrationState {
        auth: ConfigAuthContext,
        operator_grant: crate::auth::OperatorGrant,
        expected_epoch: u64,
        expected_stop_gen: u64,
        expected_txn_id: Option<u64>,
        require_zero_state_checks: bool,
        check_authority: bool,
    },
    /// Installs active `RunPins` on the live owner (`OWNER_EXECUTOR`) with zero alloc/free on
    /// `armd-control`; any displaced or stale pins are retired off-thread via `ConfigRetiredPayloads`.
    InstallSequenceRunPins(crate::sequences::RunPins),
    /// Clears `RunPins` for `action_id` on the live owner (`OWNER_EXECUTOR`) with zero free on
    /// `armd-control`.
    ClearSequenceRunPins { action_id: u64 },
    /// Starts one tick-driven non-blocking hardware commissioning op on the RT owner (0 alloc).
    StartCommissioning(crate::mit::CommissioningStartRequest),
    /// Binds durable `.calibration_commit.pending` marker proof to an active `Reserved` calibration
    /// hold before irreversible hardware commissioning TX (B2; 0 alloc).
    BindCommissioningMarker { grant_id: GrantId },
    /// Polls status and per-joint outcomes of commissioning op `op_id` (0 alloc).
    PollCommissioning { op_id: u64 },
    /// Cancels an active commissioning op matching `op_id` (stale ids ignored; forces
    /// `Reconciling` if hardware or disk was already mutated; 0 alloc).
    CancelCommissioning { op_id: u64, reason: &'static str },
    /// Clears a terminal commissioning slot owned by `op_id` once off-thread handling completes.
    ClearCommissioning {
        op_id: u64,
        clean_complete: bool,
        force_reconcile: bool,
    },
    /// Starts one tick-driven PR0C `ModeSwitch` session on `SessionSlot::ExclusiveOutput` (0 alloc).
    /// The optional P-F1 terminal ticket (reserved off-RT on the shared router) binds the
    /// session's terminal for delivery to the caller's reply destination.
    StartModeSwitch(
        ModeSwitchStartRequest,
        Option<crate::rt_core::reply_router::SessionTerminalTicket>,
    ),
    /// Cancels active `ModeSwitch` session matching `session_id` (stale IDs ignored; 0 alloc).
    CancelModeSwitch { session_id: u64, reason: &'static str },
    /// Clears terminal `ModeSwitch` session slot matching `session_id` once terminal flush completes.
    ClearModeSwitch { session_id: u64 },
    /// Commits an intermediate durable `calibration.json` update (e.g. pre-hardware MIT
    /// qualification invalidation during `motor_zero`) on the single held `Calibration`
    /// reservation without releasing it, advancing `reservation.from` in lockstep (0 alloc/free).
    CommitIntermediateCalibration {
        reservation: Reservation,
        auth: ConfigAuthContext,
        calibration: Option<CalibrationFile>,
        #[cfg(feature = "policy-runtime")]
        calibration_digest: String,
        expected_txn_id: u64,
        operator_grant: crate::auth::OperatorGrant,
    },
}

impl ConfigRequest {
    /// Static command path attributed to this config request in lock diagnostics (zero allocation).
    pub fn command_label(&self) -> &'static str {
        match self {
            Self::Reserve(r) => r.proposal.domain.command_path(),
            Self::VerifyAuthority { domain, .. }
            | Self::LatchConfigReconcileFault { domain, .. }
            | Self::ForceReconcile(domain)
            | Self::InspectHold(domain) => domain.command_path(),
            Self::Persisted(p) => p.reservation.domain().command_path(),
            Self::Activate(a) => a.reservation.domain().command_path(),
            Self::FinishReconcile(f) => f.domain.command_path(),
            Self::Release(r) => r.reservation.domain().command_path(),
            Self::SampleTableCapture => "/api/v1/grasp/setup",
            Self::GravityComp(_) => "/api/v1/mode/gravity_comp",
            Self::CheckEpisodeDeleteAllowed(_) => "/api/v1/episodes/delete",
            Self::BeginCalibrationTxn { .. }
            | Self::EndCalibrationTxn { .. }
            | Self::SampleCalibrationState { .. }
            | Self::StartCommissioning(_)
            | Self::BindCommissioningMarker { .. }
            | Self::PollCommissioning { .. }
            | Self::CancelCommissioning { .. }
            | Self::ClearCommissioning { .. }
            | Self::CommitIntermediateCalibration { .. } => "/api/v1/calibration",
            Self::StartModeSwitch(..)
            | Self::CancelModeSwitch { .. }
            | Self::ClearModeSwitch { .. } => "mode_switch",
            Self::InstallSequenceRunPins(_) | Self::ClearSequenceRunPins { .. } => {
                "/api/v1/action/run_sequence"
            }
        }
    }
}

/// Reply produced on the owner and read off-thread from `ConfigExchange`.
pub enum ConfigReply {
    Reserved(Result<Reservation, ConfigError>),
    AuthorityVerified(Result<(), ConfigError>),
    Persisted(Result<Reservation, ConfigError>),
    Activated(Result<ConfigActivated, ConfigError>),
    ReconcileFaultLatched,
    Released(Released),
    ReconcilingForced,
    TableCaptureSampled(Result<([f32; 8], usize), ConfigError>),
    GravityCompDone(Result<GravityCompReply, ConfigError>),
    HoldInspected {
        config_inhibited: bool,
        state: Option<crate::rt_core::config_iface::HoldState>,
    },
    EpisodeDeleteAllowed(bool),
    CalibrationTxnBegan(Result<CalibrationStateSample, ConfigError>),
    CalibrationTxnEnded(bool),
    CalibrationStateSampled(Result<CalibrationStateSample, ConfigError>),
    SequenceRunPinsUpdated,
    CommissioningStarted(Result<u64, ConfigError>),
    CommissioningMarkerBound(Result<(), &'static str>),
    CommissioningPolled(Option<crate::mit::CommissioningSnapshot>),
    CommissioningCancelled(Option<crate::mit::CommissioningSnapshot>),
    CommissioningCleared(bool),
    ModeSwitchStarted(Result<u64, ConfigError>),
    ModeSwitchCancelled(Option<crate::motor_mode::ModeSwitchSnapshot>),
    ModeSwitchCleared(bool),
    IntermediateCalibrationCommitted(Result<(Reservation, u64), ConfigError>),
}

/// Inline, `Copy`, zero-allocation request to admit a PR0C `ModeSwitch` session on the active owner.
#[derive(Debug, Clone, Copy)]
pub struct ModeSwitchStartRequest {
    pub plan: crate::motor_mode::ModeSwitchPlan,
    pub admission: AdmissionStamp,
    pub operator_required: bool,
    pub operator_grant: crate::auth::OperatorGrant,
    pub deadline: Instant,
    pub client: ClientId,
}

/// Displaced or unused heap payloads carried back inside the SAME `Box<ConfigExchange>`
/// and dropped strictly off-thread (zero `dealloc` on `armd-control`).
#[derive(Default)]
pub struct ConfigRetiredPayloads {
    pub keepout_profile: Option<Vec<KeepoutBox>>,
    pub keepout_runtime: Option<Vec<KeepoutBox>>,
    pub keepout_effective: Option<Vec<KeepoutBox>>,
    pub eval_bufs: Option<EvalBufs>,
    pub gripper_policy: Option<GripperPolicy>,
    pub jaw_opening: Option<JawOpening>,
    pub gripper_stale: Option<CalibrationStale>,
    pub pose_list: Option<PoseList>,
    pub sequence_file: Option<SequenceFile>,
    pub reserved_sequence_pins: Option<crate::sequences::ReservedSequencePins>,
    pub run_sequence_pins: Option<crate::sequences::RunPins>,
    pub steps: Option<Vec<Step>>,
    pub grasp_setup: Option<GraspSetup>,
    pub string_a: Option<String>,
    pub string_b: Option<String>,
    pub calibration_file: Option<CalibrationFile>,
    pub arm_model: Option<ArmModel>,
    pub gripper_baseline: Option<crate::gripper_baseline::FreeTorqueBaseline>,
    pub gripper_baseline_pending: Option<Vec<crate::gripper_baseline::MoveRecord>>,
    pub(crate) tracking: crate::tracking::TrackingRetiredBuffers,
    pub gravity_comp_policy: Option<crate::model::GravityCompPolicy>,
    pub unused_candidate: Option<ConfigCandidate>,
}

/// Single heap allocation carrying a config request into `armd-control` and its reply +
/// retired heap payloads back out so `armd-control` executes 0 allocs and 0 deallocs.
pub struct ConfigExchange {
    pub req: Option<ConfigRequest>,
    pub reply: Option<ConfigReply>,
    pub retired: ConfigRetiredPayloads,
}

impl ConfigExchange {
    pub fn new(req: ConfigRequest) -> Box<Self> {
        Box::new(Self {
            req: Some(req),
            reply: None,
            retired: ConfigRetiredPayloads::default(),
        })
    }
}

// ============================================================================
// Owner-side handlers (zero-alloc / zero-free on armd-control)
// ============================================================================

/// Executes one [`ConfigExchange`] in-place on the single owner (`SupervisorInner`).
/// Called identically by `armd-control` (`MaintenanceOp::Config`) and Legacy (`SupervisorLock`).
pub(crate) fn handle_config_exchange(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    ex: &mut ConfigExchange,
) {
    let Some(req) = ex.req.take() else {
        return;
    };
    let reply = match req {
        ConfigRequest::Reserve(r) => ConfigReply::Reserved(install_reserve_config(core, ctx, r, &mut ex.retired)),
        ConfigRequest::VerifyAuthority {
            domain,
            grant,
            proposal,
            auth,
            expected_workspace_revision,
            expected_poses_revision,
            domain_checks,
        } => ConfigReply::AuthorityVerified(install_verify_authority(
            core,
            ctx,
            domain,
            grant,
            &proposal,
            &auth,
            expected_workspace_revision,
            expected_poses_revision,
            domain_checks,
        )),
        ConfigRequest::Persisted(p) => ConfigReply::Persisted(install_persisted_config(core, p)),
        ConfigRequest::Activate(a) => ConfigReply::Activated(install_activate_config(core, ctx, a, &mut ex.retired)),
        ConfigRequest::FinishReconcile(f) => {
            ConfigReply::Activated(install_finish_reconcile(core, ctx, f, &mut ex.retired))
        }
        ConfigRequest::LatchConfigReconcileFault { domain, message } => {
            install_latch_config_reconcile_fault(core, domain, message);
            ConfigReply::ReconcileFaultLatched
        }
        ConfigRequest::Release(r) => {
            let domain = r.reservation.domain();
            let released = ConfigAuthority::release(core, r.reservation);
            if domain == ConfigDomain::Sequences && released == Released::Cleared {
                ex.retired.reserved_sequence_pins = core.sequences.reserved_pins.take();
            }
            ConfigReply::Released(released)
        }
        ConfigRequest::ForceReconcile(domain) => {
            let active_rev = ConfigAuthority::revision(core, domain);
            core.core.config_mut().force_reconcile(domain, active_rev);
            ConfigReply::ReconcilingForced
        }
        ConfigRequest::SampleTableCapture => ConfigReply::TableCaptureSampled(install_sample_table_capture(core)),
        ConfigRequest::GravityComp(op) => {
            ConfigReply::GravityCompDone(install_gravity_comp(core, ctx, op, &mut ex.retired))
        }
        ConfigRequest::InspectHold(domain) => ConfigReply::HoldInspected {
            config_inhibited: core.core.motion_inhibited_by_config(),
            state: core.core.config_held(domain).map(|h| h.state),
        },
        ConfigRequest::CheckEpisodeDeleteAllowed(episode_id) => {
            ConfigReply::EpisodeDeleteAllowed(!core.is_episode_in_use(episode_id.as_str()))
        }
        ConfigRequest::InstallSequenceRunPins(pins) => {
            if core.running_action_id() == Some(pins.action_id) {
                ex.retired.run_sequence_pins = core.sequences.pins.replace(pins);
            } else {
                ex.retired.run_sequence_pins = Some(pins);
            }
            ConfigReply::SequenceRunPinsUpdated
        }
        ConfigRequest::ClearSequenceRunPins { action_id } => {
            if core.sequences.pins.as_ref().is_some_and(|p| p.action_id == action_id) {
                ex.retired.run_sequence_pins = core.sequences.pins.take();
            }
            ConfigReply::SequenceRunPinsUpdated
        }
        ConfigRequest::BeginCalibrationTxn {
            txn_id,
            auth,
            operator_grant,
            expected_epoch,
            expected_stop_gen,
            kind,
        } => ConfigReply::CalibrationTxnBegan(install_begin_calibration_txn(
            core,
            ctx,
            txn_id,
            &auth,
            operator_grant,
            expected_epoch,
            expected_stop_gen,
            kind,
        )),
        ConfigRequest::EndCalibrationTxn {
            txn_id,
            force_reconcile,
        } => ConfigReply::CalibrationTxnEnded(install_end_calibration_txn(
            core,
            txn_id,
            force_reconcile,
        )),
        ConfigRequest::SampleCalibrationState {
            auth,
            operator_grant,
            expected_epoch,
            expected_stop_gen,
            expected_txn_id,
            require_zero_state_checks,
            check_authority,
        } => ConfigReply::CalibrationStateSampled(install_sample_calibration_state(
            core,
            ctx,
            &auth,
            operator_grant,
            expected_epoch,
            expected_stop_gen,
            expected_txn_id,
            require_zero_state_checks,
            check_authority,
        )),
        ConfigRequest::StartCommissioning(req) => {
            ConfigReply::CommissioningStarted(core.start_commissioning_rt(req, ctx.now))
        }
        ConfigRequest::BindCommissioningMarker { grant_id } => ConfigReply::CommissioningMarkerBound(
            core.core.config_mut().bind_commissioning_marker(grant_id),
        ),
        ConfigRequest::PollCommissioning { op_id } => {
            ConfigReply::CommissioningPolled(core.poll_commissioning_rt(op_id))
        }
        ConfigRequest::CancelCommissioning { op_id, reason } => {
            ConfigReply::CommissioningCancelled(core.cancel_commissioning_rt(op_id, reason))
        }
        ConfigRequest::ClearCommissioning {
            op_id,
            clean_complete,
            force_reconcile,
        } => ConfigReply::CommissioningCleared(core.clear_commissioning_rt(
            op_id,
            clean_complete,
            force_reconcile,
        )),
        ConfigRequest::StartModeSwitch(req, terminal) => {
            ConfigReply::ModeSwitchStarted(core.start_mode_switch_rt(req, terminal, ctx.now))
        }
        ConfigRequest::CancelModeSwitch { session_id, reason } => {
            ConfigReply::ModeSwitchCancelled(core.cancel_mode_switch_rt(session_id, reason))
        }
        ConfigRequest::ClearModeSwitch { session_id } => {
            ConfigReply::ModeSwitchCleared(core.clear_mode_switch_rt(session_id))
        }
        ConfigRequest::CommitIntermediateCalibration {
            mut reservation,
            auth,
            calibration,
            #[cfg(feature = "policy-runtime")]
            calibration_digest,
            expected_txn_id,
            operator_grant,
        } => {
            let mut cal_slot = calibration;
            #[cfg(feature = "policy-runtime")]
            let mut digest_slot = Some(calibration_digest);
            let res = (|| {
                let domain = reservation.domain();
                if domain != ConfigDomain::Calibration
                    || core
                        .core
                        .config_held(domain)
                        .is_none_or(|h| h.grant != Some(reservation.grant()))
                {
                    return Err(ConfigError::Reconciling("NOT_HELD"));
                }
                if core.calibration_txn_id != Some(expected_txn_id) {
                    return Err(ConfigError::Reconciling("CONFIG_BUSY"));
                }
                if !operator_grant.is_valid_at(ctx.now, core.operator_auth_enforced) {
                    return Err(ConfigError::Reconciling("OPERATOR_REQUIRED"));
                }
                let recheck_proposal = ConfigProposal {
                    domain,
                    expected_revision: reservation.from(),
                    candidate: reservation.candidate(),
                    stamp: *reservation.stamp(),
                };
                if check_config_authority(core, ctx, &recheck_proposal, &auth, false).is_err() {
                    return Err(ConfigError::Reconciling("AUTHORITY_REVOKED"));
                }
                let current_rev = ConfigAuthority::revision(core, domain);
                if current_rev != reservation.from() {
                    return Err(ConfigError::Reconciling("REVISION_MISMATCH"));
                }
                let next_rev = core.calibration_revision + 1;
                if let Err(code) = core.core.config_mut().advance_from(&mut reservation, next_rev) {
                    return Err(ConfigError::Reconciling(code));
                }
                ex.retired.calibration_file =
                    std::mem::replace(&mut core.calibration, cal_slot.take());
                #[cfg(feature = "policy-runtime")]
                if let Some(d) = digest_slot.take() {
                    ex.retired.string_a = Some(std::mem::replace(&mut core.calibration_digest, d));
                }
                core.calibration_revision = next_rev;
                if !core.rt_executor_owned {
                    core.refresh_schema_cache();
                }
                Ok((reservation, next_rev))
            })();
            if res.is_err() {
                ex.retired.calibration_file = cal_slot;
                #[cfg(feature = "policy-runtime")]
                {
                    ex.retired.string_a = digest_slot;
                }
                let active_rev = ConfigAuthority::revision(core, ConfigDomain::Calibration);
                core.core
                    .config_mut()
                    .force_reconcile(ConfigDomain::Calibration, active_rev);
            }
            ConfigReply::IntermediateCalibrationCommitted(res)
        }
    };
    ex.reply = Some(reply);
}

fn run_reserve_precheck(
    core: &SupervisorInner,
    now: Instant,
    precheck: ReservePrecheck,
    retired: &mut ConfigRetiredPayloads,
) -> Result<(), ConfigError> {
    match precheck {
        ReservePrecheck::Keepout => {
            if core.calibration_busy {
                return Err(ConfigError::Busy("a camera calibration apply or restore is running"));
            }
            if core.workspace_policy().is_none() {
                return Err(ConfigError::Unsupported(
                    "Keep-out boxes need a profile with kinematics and a workspace",
                ));
            }
            if let Some(reason) = core.workspace_change_blocker_static() {
                return Err(ConfigError::MotionBusy {
                    subject: "keep-out boxes",
                    reason,
                });
            }
            Ok(())
        }
        ReservePrecheck::Collision { enabling } => {
            if core.calibration_busy {
                return Err(ConfigError::Busy("a camera calibration apply or restore is running"));
            }
            if enabling && !core.collision_available() {
                return Err(ConfigError::Unsupported(
                    "The collision guard needs a profile gravity model and a driver that reports torque",
                ));
            }
            Ok(())
        }
        ReservePrecheck::GripperCalibration { candidate_policy } => {
            let check_res = (|| {
                if core.calibration_busy || core.zero_check_persisting.is_some() {
                    return Err(ConfigError::Busy("another calibration operation is running"));
                }
                if core.config.gripper_policy.is_none() {
                    return Err(ConfigError::Unsupported("Gripper calibration requires a gripper policy"));
                }
                if let Some(ref gp) = candidate_policy {
                    if core.driver.is_some() || core.driver_config_error.is_some() {
                        core.validate_gripper_policy_against_driver_bounded(gp)
                            .map_err(ConfigError::DriverLimitRejected)?;
                    }
                }
                Ok(())
            })();
            retired.gripper_policy = candidate_policy;
            check_res
        }
        ReservePrecheck::PosesSave => {
            if core.calibration_busy {
                return Err(ConfigError::Busy("a camera calibration apply or restore is running"));
            }
            Ok(())
        }
        ReservePrecheck::PosesDelete { deleting_pose_id } => {
            if core.calibration_busy {
                return Err(ConfigError::Busy("a camera calibration apply or restore is running"));
            }
            if !deleting_pose_id.is_empty() && core.is_pose_in_use(deleting_pose_id.as_str()) {
                return Err(ConfigError::PoseInUse);
            }
            Ok(())
        }
        ReservePrecheck::Sequences {
            steps,
            deleting_sequence_id,
            reserved_pins,
        } => {
            let check_res = (|| {
                if core.calibration_busy {
                    return Err(ConfigError::Busy("a camera calibration apply or restore is running"));
                }
                if let Some(ref steps) = steps {
                    for (i, step) in steps.iter().enumerate() {
                        if let Step::Pose { pose_id, .. } = step {
                            if core.poses.by_id(pose_id).is_none() {
                                return Err(ConfigError::SequenceRefMissing { step: i, is_pose: true });
                            }
                        }
                    }
                }
                if let Some(del_id) = deleting_sequence_id {
                    if core.is_sequence_running(del_id.as_str())
                        && core.sequences.get(del_id.as_str()).is_some()
                    {
                        return Err(ConfigError::SequenceRunning);
                    }
                }
                Ok(())
            })();
            retired.steps = steps;
            retired.reserved_sequence_pins = reserved_pins;
            check_res
        }
        ReservePrecheck::GraspSetup {
            new_camera_tilt_deg,
            allow_during_calibration_busy,
        } => {
            if !allow_during_calibration_busy && core.calibration_busy {
                return Err(ConfigError::Busy("a camera calibration apply or restore is running"));
            }
            if new_camera_tilt_deg != core.config.camera_mount.tilt_deg {
                if let Some(reason) = core.workspace_change_blocker_static() {
                    return Err(ConfigError::MotionBusy {
                        subject: "camera_tilt_deg",
                        reason,
                    });
                }
            }
            Ok(())
        }
        ReservePrecheck::ToolCommissioning {
            tool_id,
            tool_config_digest,
        } => {
            if core.calibration_busy {
                return Err(ConfigError::Busy("a camera calibration apply or restore is running"));
            }
            if core.config.active_tool.id != tool_id.as_str()
                || core.config.active_tool.config_digest != tool_config_digest.as_str()
            {
                return Err(ConfigError::PlanStale("active tool changed before commissioning confirm"));
            }
            Ok(())
        }
        ReservePrecheck::Calibration {
            allow_during_calibration_busy,
            expected_txn_id,
            operator_grant,
            require_kinematics_disarmed,
            expected_grasp_setup_revision,
        } => {
            if !operator_grant.is_valid_at(now, core.operator_auth_enforced) {
                return Err(ConfigError::CalibPrecondition {
                    status: 403,
                    code: "OPERATOR_REQUIRED",
                    message: "Operator session expired or locked during calibration",
                });
            }
            if let Some(tid) = expected_txn_id {
                if core.calibration_txn_id != Some(tid) {
                    return Err(ConfigError::Busy("calibration transaction ownership was lost"));
                }
            } else if !allow_during_calibration_busy && core.calibration_busy {
                return Err(ConfigError::Busy(
                    "a calibration apply, restore, or configuration change is running",
                ));
            }
            if let Some(reason) = core.workspace_change_blocker_for_calibration_static() {
                return Err(ConfigError::MotionBusy {
                    subject: "the camera calibration",
                    reason,
                });
            }
            if !has_grasp_setup_rt(core) {
                return Err(ConfigError::CalibPrecondition {
                    status: 400,
                    code: "UNSUPPORTED",
                    message: "This profile has no grasp setup",
                });
            }
            if require_kinematics_disarmed {
                check_kinematics_disarmed_rt(core)?;
            }
            if let Some(exp_gs) = expected_grasp_setup_revision {
                if core.grasp_setup_revision != exp_gs {
                    return Err(ConfigError::RevisionMismatch {
                        expected: exp_gs,
                        current: core.grasp_setup_revision,
                    });
                }
            }
            Ok(())
        }
        ReservePrecheck::SoftwareJointZero {
            allow_during_calibration_busy,
            expected_txn_id,
            operator_grant,
            expected_gripper_calib_revision,
            is_hardware_motor_zero,
        } => {
            if !operator_grant.is_valid_at(now, core.operator_auth_enforced) {
                return Err(ConfigError::CalibPrecondition {
                    status: 403,
                    code: "OPERATOR_REQUIRED",
                    message: "Operator session expired or locked during calibration",
                });
            }
            if let Some(tid) = expected_txn_id {
                if core.calibration_txn_id != Some(tid) {
                    return Err(ConfigError::Busy("calibration transaction ownership was lost"));
                }
            } else if !allow_during_calibration_busy && core.calibration_busy {
                return Err(ConfigError::Busy(
                    "a calibration apply, restore, zero, or configuration change is running",
                ));
            }
            if !is_hardware_motor_zero && core.config.zero_reference.is_none() {
                return Err(ConfigError::CalibPrecondition {
                    status: 400,
                    code: "UNSUPPORTED",
                    message: "the software zero needs a profile zero_reference",
                });
            }
            check_software_zero_idle_rt(core)?;
            if let Some(exp_gc) = expected_gripper_calib_revision {
                if core.gripper_calib_revision != exp_gc {
                    return Err(ConfigError::PlanStale(
                        "gripper calibration changed while preparing software joint zero",
                    ));
                }
            }
            Ok(())
        }
        ReservePrecheck::MitCommissioning {
            allow_during_calibration_busy,
            expected_txn_id,
            operator_grant,
            require_driver,
        } => {
            if !operator_grant.is_valid_at(now, core.operator_auth_enforced) {
                return Err(ConfigError::CalibPrecondition {
                    status: 403,
                    code: "OPERATOR_REQUIRED",
                    message: "Operator session expired or locked during calibration",
                });
            }
            if require_driver && core.driver.is_none() {
                return Err(ConfigError::CalibPrecondition {
                    status: 400,
                    code: "UNSUPPORTED",
                    message: "no motor driver",
                });
            }
            if let Some(tid) = expected_txn_id {
                if core.calibration_txn_id != Some(tid) {
                    return Err(ConfigError::Busy("calibration transaction ownership was lost"));
                }
            } else if !allow_during_calibration_busy && core.calibration_busy {
                return Err(ConfigError::Busy("a calibration transaction is running"));
            }
            check_mit_torque_off_rt(core)?;
            Ok(())
        }
    }
}

fn check_mit_torque_off_rt(core: &SupervisorInner) -> Result<(), ConfigError> {
    if core.driver.is_none() {
        return Err(ConfigError::CalibPrecondition {
            status: 400,
            code: "UNSUPPORTED",
            message: "no motor driver",
        });
    }
    if core.estop_pending() || core.safety.state().is_stopped() {
        return Err(ConfigError::CalibPrecondition {
            status: 409,
            code: "MIT_TIMEOUT_ARMED",
            message: "the arm is stopped or faulted; clear the fault first",
        });
    }
    if core.is_torqued
        || core.disable_pending
        || core.torque_transition.is_some()
        || core.handover_settling_since.is_some()
    {
        return Err(ConfigError::CalibPrecondition {
            status: 409,
            code: "MIT_TIMEOUT_ARMED",
            message: "torque must be off (load mechanically supported) for the TIMEOUT commissioning",
        });
    }
    if core.active_trajectory.is_some()
        || core.servo_target.is_some()
        || core.gravity_comp.is_some()
        || core.last_action.as_ref().is_some_and(|a| a.is_running())
        || core.macro_run.is_some()
    {
        return Err(ConfigError::CalibPrecondition {
            status: 409,
            code: "MIT_TIMEOUT_ARMED",
            message: "a motion is running",
        });
    }
    Ok(())
}

fn has_grasp_setup_rt(core: &SupervisorInner) -> bool {
    core.config.has_cartesian_with_gripper()
        && core.config.kinematics.is_some()
        && core.config.gripper_policy.is_some()
        && core.config.grasp.is_some()
}

fn check_kinematics_disarmed_rt(core: &SupervisorInner) -> Result<(), ConfigError> {
    let running = core.last_action.as_ref().is_some_and(|a| a.is_running()) || core.macro_run.is_some();
    if core.is_torqued || running {
        let message = if running {
            "the kinematic correction can change only with torque off and no action running (an action is running); park or disable torque first"
        } else {
            "the kinematic correction can change only with torque off and no action running (torque is on); park or disable torque first"
        };
        return Err(ConfigError::CalibPrecondition {
            status: 409,
            code: "KINEMATICS_ARMED",
            message,
        });
    }
    Ok(())
}

fn check_software_zero_idle_rt(core: &SupervisorInner) -> Result<(), ConfigError> {
    use crate::safety::SafetyState;
    if core.safety.state().is_stopped() {
        let mut message = BoundedText::<256>::empty();
        let _ = core::fmt::write(
            &mut message,
            format_args!("the arm is in {:?}; clear the fault first", core.safety.state()),
        );
        return Err(ConfigError::CalibPreconditionText {
            status: 409,
            code: "MOTOR_ZERO_FAULT",
            message,
        });
    }
    let msg = if core.is_torqued || core.disable_pending {
        Some("motor zero needs torque off and no action running (torque is on); park or disable torque first")
    } else if core.last_action.as_ref().is_some_and(|a| a.is_running()) || core.macro_run.is_some() {
        Some("motor zero needs torque off and no action running (an action is running); park or disable torque first")
    } else if core.gravity_comp.is_some() {
        Some("motor zero needs torque off and no action running (gravity compensation is on); park or disable torque first")
    } else if core.active_trajectory.is_some() || core.servo_target.is_some() {
        Some("motor zero needs torque off and no action running (a motion is running); park or disable torque first")
    } else {
        match core.safety.state() {
            SafetyState::Idle => None,
            SafetyState::Booting => Some("motor zero needs torque off and no action running (the arm is Booting); park or disable torque first"),
            SafetyState::Armed => Some("motor zero needs torque off and no action running (the arm is Armed); park or disable torque first"),
            SafetyState::Moving => Some("motor zero needs torque off and no action running (the arm is Moving); park or disable torque first"),
            SafetyState::EStop | SafetyState::Fault(_) => None,
        }
    };
    if let Some(message) = msg {
        return Err(ConfigError::CalibPrecondition {
            status: 409,
            code: "MOTOR_ZERO_ARMED",
            message,
        });
    }
    Ok(())
}

fn snapshot_calibration_sample_rt(
    core: &SupervisorInner,
    require_readings: bool,
) -> Result<CalibrationStateSample, ConfigError> {
    let dof = core.config.dof;
    let has_driver = core.driver.is_some();
    let feedback_valid = if has_driver {
        core.commit_feedback_valid()
    } else {
        true
    };
    if require_readings {
        if has_driver && !feedback_valid {
            return Err(ConfigError::CalibPrecondition {
                status: 409,
                code: "MOTOR_ZERO_NO_FEEDBACK",
                message: "the motor feedback is not fresh; check the bus and the motor power",
            });
        }
        if core.measured_positions.len() != dof
            || dof > crate::rt_core::mailbox::MAX_DOF
            || !core.measured_positions.iter().all(|v| v.is_finite())
        {
            return Err(ConfigError::CalibPrecondition {
                status: 409,
                code: "MOTOR_ZERO_NO_FEEDBACK",
                message: "no valid joint readings",
            });
        }
    }
    let mut positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
    let copy_len = core.measured_positions.len().min(crate::rt_core::mailbox::MAX_DOF);
    positions[..copy_len].copy_from_slice(&core.measured_positions[..copy_len]);
    let mut feedback_instants = [None; crate::rt_core::mailbox::MAX_DOF];
    let fb = core.core.feedback().instants();
    let fb_len = fb.len().min(crate::rt_core::mailbox::MAX_DOF);
    feedback_instants[..fb_len].copy_from_slice(&fb[..fb_len]);
    Ok(CalibrationStateSample {
        positions,
        dof,
        has_driver,
        feedback_valid,
        feedback_instants,
        is_torqued: core.is_torqued,
        safety_state: SafetyStateTag::from_state(core.safety.state()),
        epoch_id: core.safety.epoch_id(),
        calibration_revision: core.calibration_revision,
    })
}

fn check_calib_live_authority_rt(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    auth: &ConfigAuthContext,
    expected_epoch: u64,
    expected_stop_gen: u64,
) -> Result<(), ConfigError> {
    if auth.admission.connection_id != 0 && core.is_connection_closed(auth.admission.connection_id) {
        return Err(ConfigError::ConnectionClosed);
    }
    if core.estop_request_unapplied() || core.safety.epoch_id() != expected_epoch {
        return Err(ConfigError::EpochMismatch {
            current_epoch: core.safety.epoch_id(),
        });
    }
    if core.urgent_stop_unapplied() || ctx.stop_generation != expected_stop_gen {
        return Err(ConfigError::Stopped);
    }
    if let Some((conn, gen)) = auth.lease_holder {
        if gen == 0 || !core.validate_lease_at(conn.as_str(), gen, ctx.now) {
            return Err(ConfigError::LeaseLost);
        }
    } else {
        return Err(ConfigError::LeaseLost);
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn install_begin_calibration_txn(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    txn_id: u64,
    auth: &ConfigAuthContext,
    operator_grant: crate::auth::OperatorGrant,
    expected_epoch: u64,
    expected_stop_gen: u64,
    kind: CalibrationTxnKind,
) -> Result<CalibrationStateSample, ConfigError> {
    check_calib_live_authority_rt(core, ctx, auth, expected_epoch, expected_stop_gen)?;
    if !operator_grant.is_valid_at(ctx.now, core.operator_auth_enforced) {
        return Err(ConfigError::CalibPrecondition {
            status: 403,
            code: "OPERATOR_REQUIRED",
            message: "Operator session expired or locked during calibration",
        });
    }
    match kind {
        CalibrationTxnKind::IngressBusyCheck => {
            if core.calibration_busy {
                return Err(ConfigError::Busy(
                    "a calibration apply, restore or motor zero is running",
                ));
            }
            snapshot_calibration_sample_rt(core, false)
        }
        CalibrationTxnKind::CameraPreflight => {
            if core.calibration_busy {
                return Err(ConfigError::Busy(
                    "a calibration apply, restore or motor zero is running",
                ));
            }
            if core.core.config().held().is_some() {
                return Err(ConfigError::Busy(
                    "a calibration apply, restore, or configuration change is running",
                ));
            }
            if let Some(reason) = core.workspace_change_blocker_for_calibration_static() {
                return Err(ConfigError::MotionBusy {
                    subject: "the camera calibration",
                    reason,
                });
            }
            if !has_grasp_setup_rt(core) {
                return Err(ConfigError::CalibPrecondition {
                    status: 400,
                    code: "UNSUPPORTED",
                    message: "This profile has no grasp setup",
                });
            }
            snapshot_calibration_sample_rt(core, false)
        }
        CalibrationTxnKind::CameraApply {
            require_kinematics_disarmed,
            dry_run,
        } => {
            if core.calibration_busy || core.core.config().held().is_some() {
                return Err(ConfigError::Busy(
                    "a calibration apply, restore, or configuration change is running",
                ));
            }
            if let Some(reason) = core.workspace_change_blocker_for_calibration_static() {
                return Err(ConfigError::MotionBusy {
                    subject: "the camera calibration",
                    reason,
                });
            }
            if !has_grasp_setup_rt(core) {
                return Err(ConfigError::CalibPrecondition {
                    status: 400,
                    code: "UNSUPPORTED",
                    message: "This profile has no grasp setup",
                });
            }
            if require_kinematics_disarmed {
                check_kinematics_disarmed_rt(core)?;
            }
            let sample = snapshot_calibration_sample_rt(core, false)?;
            if !dry_run {
                core.calibration_busy = true;
                core.calibration_txn_id = Some(txn_id);
            }
            Ok(sample)
        }
        CalibrationTxnKind::CameraRestore => {
            if core.calibration_busy || core.core.config().held().is_some() {
                return Err(ConfigError::Busy(
                    "a calibration apply, restore, or configuration change is running",
                ));
            }
            if let Some(reason) = core.workspace_change_blocker_for_calibration_static() {
                return Err(ConfigError::MotionBusy {
                    subject: "the camera calibration",
                    reason,
                });
            }
            if !has_grasp_setup_rt(core) {
                return Err(ConfigError::CalibPrecondition {
                    status: 400,
                    code: "UNSUPPORTED",
                    message: "This profile has no grasp setup",
                });
            }
            let sample = snapshot_calibration_sample_rt(core, false)?;
            core.calibration_busy = true;
            core.calibration_txn_id = Some(txn_id);
            Ok(sample)
        }
        CalibrationTxnKind::SoftwareZero { set_busy }
        | CalibrationTxnKind::MotorZero { set_busy } => {
            if core.calibration_busy || core.core.config().held().is_some() {
                return Err(ConfigError::Busy(
                    "a calibration apply, restore, motor zero, or configuration change is running",
                ));
            }
            check_software_zero_idle_rt(core)?;
            let sample = snapshot_calibration_sample_rt(core, true)?;
            if set_busy {
                core.calibration_busy = true;
                core.motor_zero_busy = true;
                core.calibration_txn_id = Some(txn_id);
            }
            Ok(sample)
        }
        CalibrationTxnKind::JointZeroRestore => {
            if core.calibration_busy || core.core.config().held().is_some() {
                return Err(ConfigError::Busy(
                    "a calibration apply, restore, zero, or configuration change is running",
                ));
            }
            check_software_zero_idle_rt(core)?;
            let sample = snapshot_calibration_sample_rt(core, false)?;
            core.calibration_busy = true;
            core.motor_zero_busy = true;
            core.calibration_txn_id = Some(txn_id);
            Ok(sample)
        }
        CalibrationTxnKind::MitTimeout { dry_run } => {
            if core.driver.is_none() {
                return Err(ConfigError::CalibPrecondition {
                    status: 400,
                    code: "UNSUPPORTED",
                    message: "no motor driver",
                });
            }
            if core.calibration_busy || core.core.config().held().is_some() {
                return Err(ConfigError::Busy("a calibration transaction is running"));
            }
            check_mit_torque_off_rt(core)?;
            let sample = snapshot_calibration_sample_rt(core, false)?;
            if !dry_run {
                core.calibration_busy = true;
                core.calibration_txn_id = Some(txn_id);
            }
            Ok(sample)
        }
        CalibrationTxnKind::MitQualification => {
            if core.driver.is_none() {
                return Err(ConfigError::CalibPrecondition {
                    status: 400,
                    code: "UNSUPPORTED",
                    message: "no motor driver",
                });
            }
            if core.calibration_busy || core.core.config().held().is_some() {
                return Err(ConfigError::Busy("a calibration transaction is running"));
            }
            check_mit_torque_off_rt(core)?;
            let sample = snapshot_calibration_sample_rt(core, false)?;
            core.calibration_busy = true;
            core.calibration_txn_id = Some(txn_id);
            Ok(sample)
        }
    }
}

fn install_end_calibration_txn(
    core: &mut SupervisorInner,
    txn_id: u64,
    force_reconcile: bool,
) -> bool {
    if core.calibration_txn_id == Some(txn_id) {
        if force_reconcile {
            let active_rev = ConfigAuthority::revision(core, ConfigDomain::Calibration);
            core.core
                .config_mut()
                .force_reconcile(ConfigDomain::Calibration, active_rev);
        }
        core.calibration_busy = false;
        core.motor_zero_busy = false;
        core.calibration_txn_id = None;
        true
    } else {
        false
    }
}

#[allow(clippy::too_many_arguments)]
fn install_sample_calibration_state(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    auth: &ConfigAuthContext,
    operator_grant: crate::auth::OperatorGrant,
    expected_epoch: u64,
    expected_stop_gen: u64,
    expected_txn_id: Option<u64>,
    require_zero_state_checks: bool,
    check_authority: bool,
) -> Result<CalibrationStateSample, ConfigError> {
    if let Some(tid) = expected_txn_id {
        if core.calibration_txn_id != Some(tid) {
            return Err(ConfigError::Busy("calibration transaction ownership was lost"));
        }
    }
    if check_authority {
        check_calib_live_authority_rt(core, ctx, auth, expected_epoch, expected_stop_gen)?;
        if !operator_grant.is_valid_at(ctx.now, core.operator_auth_enforced) {
            return Err(ConfigError::CalibPrecondition {
                status: 403,
                code: "OPERATOR_REQUIRED",
                message: "Operator session expired or locked during calibration",
            });
        }
    }
    if require_zero_state_checks {
        check_software_zero_idle_rt(core)?;
    }
    snapshot_calibration_sample_rt(core, require_zero_state_checks)
}

fn install_reserve_config(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    req: ReserveConfig,
    retired: &mut ConfigRetiredPayloads,
) -> Result<Reservation, ConfigError> {
    let ReserveConfig {
        proposal,
        auth,
        mut precheck,
        expected_workspace_revision,
        expected_poses_revision,
    } = req;
    let seq_reserved_pins = match &mut precheck {
        ReservePrecheck::Sequences { reserved_pins, .. } => reserved_pins.take(),
        _ => None,
    };
    // Always retire any heap payload inside `precheck` even if authority or revision check fails.
    let auth_res = check_config_authority(core, ctx, &proposal, &auth, true);
    if let Err(err) = auth_res {
        retire_precheck_payloads(precheck, retired);
        retired.reserved_sequence_pins = seq_reserved_pins;
        return Err(err);
    }
    let current_rev = ConfigAuthority::revision(core, proposal.domain);
    if proposal.expected_revision != current_rev {
        retire_precheck_payloads(precheck, retired);
        retired.reserved_sequence_pins = seq_reserved_pins;
        return Err(ConfigError::RevisionMismatch {
            expected: proposal.expected_revision,
            current: current_rev,
        });
    }
    if let Some(exp_ws) = expected_workspace_revision {
        if core.workspace_revision != exp_ws {
            retire_precheck_payloads(precheck, retired);
            retired.reserved_sequence_pins = seq_reserved_pins;
            return Err(ConfigError::PlanStale(
                "workspace changed while preparing configuration candidate",
            ));
        }
    }
    if let Some(exp_poses) = expected_poses_revision {
        if core.poses.revision != exp_poses {
            retire_precheck_payloads(precheck, retired);
            retired.reserved_sequence_pins = seq_reserved_pins;
            return Err(ConfigError::PlanStale(
                "poses changed while preparing calibration candidate",
            ));
        }
    }
    if let Err(err) = run_reserve_precheck(core, ctx.now, precheck, retired) {
        retired.reserved_sequence_pins = seq_reserved_pins;
        return Err(err);
    }
    match ConfigAuthority::reserve(core, &proposal, ctx) {
        Reserved::Granted(reservation) => {
            core.record_consumed_stamp(auth.admission.request_id);
            if proposal.domain == ConfigDomain::Sequences {
                retired.reserved_sequence_pins =
                    std::mem::replace(&mut core.sequences.reserved_pins, seq_reserved_pins);
            } else {
                retired.reserved_sequence_pins = seq_reserved_pins;
            }
            Ok(reservation)
        }
        Reserved::Refused { code: "REVISION_MISMATCH", current } => {
            retired.reserved_sequence_pins = seq_reserved_pins;
            Err(ConfigError::RevisionMismatch {
                expected: proposal.expected_revision,
                current,
            })
        }
        Reserved::Refused { code: "CONFIG_BUSY", .. } => {
            retired.reserved_sequence_pins = seq_reserved_pins;
            Err(ConfigError::Busy("another configuration change is being activated or reconciled"))
        }
        Reserved::Refused { code: "EPOCH_MISMATCH", .. } => {
            retired.reserved_sequence_pins = seq_reserved_pins;
            Err(ConfigError::EpochMismatch {
                current_epoch: core.safety.epoch_id(),
            })
        }
        Reserved::Refused { code: "LEASE_LOST", .. } => {
            retired.reserved_sequence_pins = seq_reserved_pins;
            Err(ConfigError::LeaseLost)
        }
        Reserved::Refused { code: "STOPPED", .. } => {
            retired.reserved_sequence_pins = seq_reserved_pins;
            Err(ConfigError::Stopped)
        }
        Reserved::Refused { code: "DEADLINE_EXPIRED", .. } => {
            retired.reserved_sequence_pins = seq_reserved_pins;
            Err(ConfigError::DeadlineExpired)
        }
        Reserved::Refused { .. } => {
            retired.reserved_sequence_pins = seq_reserved_pins;
            Err(ConfigError::WrongTarget)
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn install_verify_authority(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    domain: ConfigDomain,
    grant: GrantId,
    proposal: &ConfigProposal,
    auth: &ConfigAuthContext,
    expected_workspace_revision: Option<u64>,
    expected_poses_revision: Option<u64>,
    domain_checks: VerifyDomainChecks,
) -> Result<(), ConfigError> {
    if core
        .core
        .config_held(domain)
        .is_none_or(|h| h.grant != Some(grant))
    {
        return Err(ConfigError::NotHeld);
    }
    if proposal.domain != domain {
        return Err(ConfigError::WrongTarget);
    }
    check_config_authority(core, ctx, proposal, auth, false)?;
    let current_rev = ConfigAuthority::revision(core, domain);
    if proposal.expected_revision != current_rev {
        return Err(ConfigError::RevisionMismatch {
            expected: proposal.expected_revision,
            current: current_rev,
        });
    }
    if let Some(exp_ws) = expected_workspace_revision {
        if core.workspace_revision != exp_ws {
            return Err(ConfigError::PlanStale(
                "workspace changed before configuration rename",
            ));
        }
    }
    if let Some(exp_poses) = expected_poses_revision {
        if core.poses.revision != exp_poses {
            return Err(ConfigError::PlanStale(
                "poses changed before calibration rename",
            ));
        }
    }
    if domain == ConfigDomain::Calibration {
        let Some(op_grant) = domain_checks.operator_grant else {
            return Err(ConfigError::CalibPrecondition {
                status: 403,
                code: "OPERATOR_REQUIRED",
                message: "Operator session expired or locked during calibration",
            });
        };
        if !op_grant.is_valid_at(ctx.now, core.operator_auth_enforced) {
            return Err(ConfigError::CalibPrecondition {
                status: 403,
                code: "OPERATOR_REQUIRED",
                message: "Operator session expired or locked during calibration",
            });
        }
    } else if let Some(op_grant) = domain_checks.operator_grant {
        if !op_grant.is_valid_at(ctx.now, core.operator_auth_enforced) {
            return Err(ConfigError::CalibPrecondition {
                status: 403,
                code: "OPERATOR_REQUIRED",
                message: "Operator session expired or locked during calibration",
            });
        }
    }
    if let Some(tid) = domain_checks.expected_txn_id {
        if core.calibration_txn_id != Some(tid) {
            return Err(ConfigError::Busy("calibration transaction ownership was lost"));
        }
    }
    if let Some(exp_gs) = domain_checks.expected_grasp_setup_revision {
        if core.grasp_setup_revision != exp_gs {
            return Err(ConfigError::RevisionMismatch {
                expected: exp_gs,
                current: core.grasp_setup_revision,
            });
        }
    }
    if let Some(exp_gc) = domain_checks.expected_gripper_calib_revision {
        if core.gripper_calib_revision != exp_gc {
            return Err(ConfigError::PlanStale(
                "gripper calibration changed before software joint zero rename",
            ));
        }
    }
    if domain_checks.require_workspace_unblocked {
        if let Some(reason) = core.workspace_change_blocker_for_calibration_static() {
            return Err(ConfigError::MotionBusy {
                subject: "the camera calibration",
                reason,
            });
        }
    }
    if domain_checks.require_kinematics_disarmed {
        check_kinematics_disarmed_rt(core)?;
    }
    if domain_checks.require_software_zero_idle {
        check_software_zero_idle_rt(core)?;
    }
    if domain_checks.require_mit_torque_off {
        check_mit_torque_off_rt(core)?;
    }
    Ok(())
}

fn retire_precheck_payloads(precheck: ReservePrecheck, retired: &mut ConfigRetiredPayloads) {
    match precheck {
        ReservePrecheck::GripperCalibration { candidate_policy } => {
            retired.gripper_policy = candidate_policy;
        }
        ReservePrecheck::Sequences {
            steps,
            reserved_pins,
            ..
        } => {
            retired.steps = steps;
            retired.reserved_sequence_pins = reserved_pins;
        }
        _ => {}
    }
}

fn install_persisted_config(
    core: &mut SupervisorInner,
    req: PersistedConfig,
) -> Result<Reservation, ConfigError> {
    let domain = req.reservation.domain();
    match ConfigAuthority::persisted(core, &req.reservation, req.durability) {
        Ok(()) => Ok(req.reservation),
        Err(code) => {
            // Persistence #5: disk rename already occurred; unconditionally force Reconciling.
            let active_rev = ConfigAuthority::revision(core, domain);
            core.core.config_mut().force_reconcile(domain, active_rev);
            Err(ConfigError::Reconciling(code))
        }
    }
}

fn fail_activation(
    core: &mut SupervisorInner,
    reservation: Reservation,
    code: &'static str,
    candidate: ConfigCandidate,
    retired: &mut ConfigRetiredPayloads,
) -> ConfigError {
    retired.unused_candidate = Some(candidate);
    let domain = reservation.domain();
    match core.core.config_mut().activation_failed(reservation, code) {
        Activated::Reconciling { code } => ConfigError::Reconciling(code),
        Activated::Refused { code, reservation } => {
            let rel = ConfigAuthority::release(core, reservation);
            if domain == ConfigDomain::Sequences && rel == Released::Cleared {
                retired.reserved_sequence_pins = core.sequences.reserved_pins.take();
            }
            match code {
                "LEASE_LOST" => ConfigError::LeaseLost,
                "EPOCH_MISMATCH" => ConfigError::EpochMismatch {
                    current_epoch: core.safety.epoch_id(),
                },
                "STOPPED" => ConfigError::Stopped,
                "CONNECTION_CLOSED" => ConfigError::ConnectionClosed,
                "DEADLINE_EXPIRED" => ConfigError::DeadlineExpired,
                "PLAN_STALE" => ConfigError::PlanStale("workspace or pose state changed before activation"),
                "MOTION_BUSY" => ConfigError::MotionBusy {
                    subject: "configuration",
                    reason: "motion started before activation",
                },
                other => ConfigError::Busy(other),
            }
        }
        Activated::Active { .. } => {
            let active_rev = ConfigAuthority::revision(core, domain);
            core.core.config_mut().force_reconcile(domain, active_rev);
            ConfigError::Reconciling(code)
        }
    }
}

fn install_activate_config(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    req: ActivateConfig,
    retired: &mut ConfigRetiredPayloads,
) -> Result<ConfigActivated, ConfigError> {
    let ActivateConfig {
        reservation,
        auth,
        revoked_off_thread,
        candidate,
        durability,
        expected_workspace_revision,
        expected_poses_revision,
    } = req;
    let domain = reservation.domain();
    let expected_from = reservation.from();

    if core
        .core
        .config_held(domain)
        .is_none_or(|h| h.grant != Some(reservation.grant()))
    {
        retired.unused_candidate = Some(candidate);
        let _ = ConfigAuthority::release(core, reservation);
        if durability != Durability::Volatile {
            let active_rev = ConfigAuthority::revision(core, domain);
            core.core.config_mut().force_reconcile(domain, active_rev);
            return Err(ConfigError::Reconciling("NOT_HELD"));
        }
        return Err(ConfigError::NotHeld);
    }

    if let Some(code) = revoked_off_thread {
        return Err(fail_activation(core, reservation, code, candidate, retired));
    }

    // Re-verify live authority AND original proposal deadline at activation (without re-checking
    // stamp consumption, which was recorded on Reserve).
    let recheck_proposal = ConfigProposal {
        domain,
        expected_revision: expected_from,
        candidate: reservation.candidate(),
        stamp: *reservation.stamp(),
    };
    if let Err(err) = check_config_authority(core, ctx, &recheck_proposal, &auth, false) {
        let code = match err {
            ConfigError::LeaseLost => "LEASE_LOST",
            ConfigError::ConnectionClosed => "CONNECTION_CLOSED",
            ConfigError::EpochMismatch { .. } => "EPOCH_MISMATCH",
            ConfigError::Stopped => "STOPPED",
            ConfigError::DeadlineExpired => "DEADLINE_EXPIRED",
            _ => "AUTHORITY_REVOKED",
        };
        return Err(fail_activation(core, reservation, code, candidate, retired));
    }

    let current_rev = ConfigAuthority::revision(core, domain);
    if current_rev != expected_from {
        return Err(fail_activation(core, reservation, "REVISION_MISMATCH", candidate, retired));
    }
    if let Some(exp_ws) = expected_workspace_revision {
        if core.workspace_revision != exp_ws {
            return Err(fail_activation(core, reservation, "PLAN_STALE", candidate, retired));
        }
    }
    if let Some(exp_poses) = expected_poses_revision {
        if core.poses.revision != exp_poses {
            return Err(fail_activation(core, reservation, "PLAN_STALE", candidate, retired));
        }
    }

    // Domain-specific post-rename invariant rechecks before swapping live state.
    if let Err(code) = check_candidate_domain_invariants(core, ctx.now, &candidate, false) {
        return Err(fail_activation(core, reservation, code, candidate, retired));
    }

    // Infallible zero-alloc, zero-free in-place swap into SupervisorInner.
    swap_config_candidate_in_place(core, candidate, retired);

    if durability == Durability::Durable {
        let _ = core.core.config_mut().confirm_durable(&reservation);
    }

    match ConfigAuthority::activate(core, reservation) {
        Activated::Active { revision, durability } => {
            Ok(ConfigActivated::from_core(core, revision, durability, ctx.now))
        }
        Activated::Refused { code, reservation } => {
            let _ = ConfigAuthority::release(core, reservation);
            Err(ConfigError::Busy(code))
        }
        Activated::Reconciling { code } => Err(ConfigError::Reconciling(code)),
    }
}

fn check_candidate_domain_invariants(
    core: &SupervisorInner,
    now: Instant,
    candidate: &ConfigCandidate,
    for_reconcile: bool,
) -> Result<(), &'static str> {
    match candidate {
        ConfigCandidate::Keepout { .. } => {
            if core.workspace_policy().is_none() {
                Err("UNSUPPORTED")
            } else if core.workspace_change_blocker_static().is_some() {
                Err("MOTION_BUSY")
            } else {
                Ok(())
            }
        }
        ConfigCandidate::Collision { enabled, .. } => {
            if *enabled == Some(true) && !core.collision_available() {
                Err("UNSUPPORTED")
            } else {
                Ok(())
            }
        }
        ConfigCandidate::GripperCalibration { policy, .. } => {
            if core.config.gripper_policy.is_none() {
                Err("UNSUPPORTED")
            } else if (core.driver.is_some() || core.driver_config_error.is_some())
                && core.validate_gripper_policy_against_driver_bounded(policy).is_err()
            {
                Err("INVALID_CALIBRATION")
            } else {
                Ok(())
            }
        }
        ConfigCandidate::GraspSetup { setup, .. } => {
            if setup.camera_tilt_deg as f32 != core.config.camera_mount.tilt_deg
                && core.workspace_change_blocker_static().is_some()
            {
                Err("MOTION_BUSY")
            } else {
                Ok(())
            }
        }
        ConfigCandidate::ToolCommissioning {
            tool_id,
            tool_config_digest,
            ..
        } => {
            if core.config.active_tool.id != tool_id.as_str()
                || core.config.active_tool.config_digest != tool_config_digest.as_str()
            {
                Err("PLAN_STALE")
            } else {
                Ok(())
            }
        }
        ConfigCandidate::Calibration {
            kinematics_changed,
            expected_grasp_setup_revision,
            expected_txn_id,
            operator_grant,
            ..
        } => {
            if !operator_grant.is_valid_at(now, core.operator_auth_enforced) {
                return Err("OPERATOR_REQUIRED");
            }
            if for_reconcile {
                if core.calibration_txn_id.is_some() && core.calibration_txn_id != *expected_txn_id {
                    return Err("CONFIG_BUSY");
                }
            } else if let Some(tid) = expected_txn_id {
                if core.calibration_txn_id != Some(*tid) {
                    return Err("CONFIG_BUSY");
                }
            }
            if !has_grasp_setup_rt(core) {
                return Err("UNSUPPORTED");
            }
            if core.workspace_change_blocker_for_calibration_static().is_some() {
                return Err("MOTION_BUSY");
            }
            if *kinematics_changed && check_kinematics_disarmed_rt(core).is_err() {
                return Err("KINEMATICS_ARMED");
            }
            if let Some(exp_gs) = expected_grasp_setup_revision {
                if core.grasp_setup_revision != *exp_gs {
                    return Err("REVISION_MISMATCH");
                }
            }
            Ok(())
        }
        ConfigCandidate::SoftwareJointZero {
            new_offsets,
            dof,
            is_hardware_motor_zero,
            expected_gripper_calib_revision,
            expected_txn_id,
            operator_grant,
            ..
        } => {
            if !operator_grant.is_valid_at(now, core.operator_auth_enforced) {
                return Err("OPERATOR_REQUIRED");
            }
            if for_reconcile {
                if core.calibration_txn_id.is_some() && core.calibration_txn_id != *expected_txn_id {
                    return Err("CONFIG_BUSY");
                }
            } else if let Some(tid) = expected_txn_id {
                if core.calibration_txn_id != Some(*tid) {
                    return Err("CONFIG_BUSY");
                }
            }
            if (!*is_hardware_motor_zero && core.config.zero_reference.is_none())
                || *dof != core.config.dof
                || *dof > crate::rt_core::mailbox::MAX_DOF
            {
                return Err("UNSUPPORTED");
            }
            if !new_offsets[..*dof].iter().all(|v| v.is_finite()) {
                return Err("INVALID_COMMAND");
            }
            if let Err(err) = check_software_zero_idle_rt(core) {
                return Err(match err {
                    ConfigError::CalibPrecondition { code, .. }
                    | ConfigError::CalibPreconditionText { code, .. } => code,
                    _ => "MOTOR_ZERO_ARMED",
                });
            }
            if let Some(exp_gc) = expected_gripper_calib_revision {
                if core.gripper_calib_revision != *exp_gc {
                    return Err("PLAN_STALE");
                }
            }
            if let Some(ref d) = core.driver {
                let has_pos_limits = d
                    .with_ref(|drv| (0..*dof).any(|i| drv.position_limits(i).is_some()))
                    .unwrap_or(false);
                if has_pos_limits
                    && core
                        .probe_driver_limits_for_offsets_bounded(&new_offsets[..*dof])
                        .is_err()
                {
                    return Err("INVALID_CALIBRATION");
                }
            }
            Ok(())
        }
        ConfigCandidate::MitQualification {
            expected_txn_id,
            operator_grant,
            ..
        } => {
            if !operator_grant.is_valid_at(now, core.operator_auth_enforced) {
                return Err("OPERATOR_REQUIRED");
            }
            if for_reconcile {
                if core.calibration_txn_id.is_some() && core.calibration_txn_id != *expected_txn_id {
                    return Err("CONFIG_BUSY");
                }
            } else if let Some(tid) = expected_txn_id {
                if core.calibration_txn_id != Some(*tid) {
                    return Err("CONFIG_BUSY");
                }
            }
            if let Err(err) = check_mit_torque_off_rt(core) {
                return Err(match err {
                    ConfigError::CalibPrecondition { code, .. }
                    | ConfigError::CalibPreconditionText { code, .. } => code,
                    _ => "MIT_TIMEOUT_ARMED",
                });
            }
            Ok(())
        }
        ConfigCandidate::Poses { .. } | ConfigCandidate::Sequences { .. } => Ok(()),
    }
}

fn swap_config_candidate_in_place(
    core: &mut SupervisorInner,
    candidate: ConfigCandidate,
    retired: &mut ConfigRetiredPayloads,
) {
    match candidate {
        ConfigCandidate::Keepout {
            profile_boxes,
            runtime_boxes,
            effective_boxes,
            ws_eval_bufs,
        } => {
            if let Some(ws) = core.config.workspace.as_mut() {
                retired.keepout_effective = Some(std::mem::replace(&mut ws.keepout_boxes, effective_boxes));
            } else {
                retired.keepout_effective = Some(effective_boxes);
            }
            retired.keepout_profile = core.keepout.profile.replace(profile_boxes);
            retired.keepout_runtime = Some(std::mem::replace(&mut core.keepout.runtime_boxes, runtime_boxes));
            core.keepout.runtime = true;
            core.workspace_revision += 1;
            core.retire_servo_ws_min_depths();
            core.keepout.revision += 1;
            retired.eval_bufs = Some(std::mem::replace(&mut core.ws_eval_bufs, ws_eval_bufs));
        }
        ConfigCandidate::Collision { enabled, sensitivity } => {
            if let Some(e) = enabled {
                core.collision.enabled = e;
            }
            if !core.collision.enabled {
                core.collision.clear_hold();
            }
            if let Some(s) = sensitivity {
                core.collision.sensitivity = s;
            }
            core.collision.runtime = true;
            core.collision.revision += 1;
        }
        ConfigCandidate::GripperCalibration {
            offset_rad,
            policy,
            jaw_opening,
            clear_stale,
        } => {
            let old_policy = core.config.gripper_policy.replace(policy);
            if core.base_gripper_policy.is_none() {
                core.base_gripper_policy = old_policy;
            } else {
                retired.gripper_policy = old_policy;
            }
            core.gripper_calib_offset = offset_rad;
            if let Some(jo) = jaw_opening {
                retired.jaw_opening = core.gripper_calib_jaw_opening.replace(jo);
            }
            core.gripper_calib_invalid = false;
            if clear_stale {
                retired.gripper_stale = core.gripper_calib_stale.take();
            }
            core.gripper_calib_revision += 1;
        }
        ConfigCandidate::Poses { next_list } => {
            retired.pose_list = Some(std::mem::replace(&mut core.poses.list, next_list));
            core.poses.revision += 1;
        }
        ConfigCandidate::Sequences { next_file } => {
            retired.sequence_file = Some(std::mem::replace(&mut core.sequences.file, next_file));
            retired.reserved_sequence_pins = core.sequences.reserved_pins.take();
            core.sequences.revision += 1;
        }
        ConfigCandidate::GraspSetup { setup, ws_eval_bufs } => {
            let tilt = setup.camera_tilt_deg as f32;
            if core.config.camera_mount.tilt_deg != tilt {
                core.workspace_revision += 1;
                core.retire_servo_ws_min_depths();
            }
            core.config.camera_mount.tilt_deg = tilt;
            retired.grasp_setup = core.config.grasp.replace(setup);
            if let Some(bufs) = ws_eval_bufs {
                retired.eval_bufs = Some(std::mem::replace(&mut core.ws_eval_bufs, bufs));
            }
            core.grasp_setup_revision += 1;
        }
        ConfigCandidate::ToolCommissioning { confirmed_at, .. } => {
            core.config.active_tool.commissioning_required = false;
            retired.string_a = core.tool_commissioning_confirmed_at.replace(confirmed_at);
            core.tool_commissioning_revision += 1;
        }
        ConfigCandidate::Calibration {
            calibration,
            #[cfg(feature = "policy-runtime")]
            calibration_digest,
            camera_mount,
            arm_model,
            kinematics_changed,
            next_pose_list,
            grasp_setup,
            expected_grasp_setup_revision: _,
            expected_txn_id: _,
            operator_grant: _,
            effective_floor,
            ws_eval_bufs,
        } => {
            let mut ws_changed = kinematics_changed;
            if core.config.camera_mount != camera_mount {
                ws_changed = true;
            }
            core.config.camera_mount = camera_mount;
            retired.arm_model = Some(std::mem::replace(&mut core.config.arm_model, arm_model));
            retired.calibration_file = std::mem::replace(&mut core.calibration, calibration);
            #[cfg(feature = "policy-runtime")]
            {
                retired.string_a = Some(std::mem::replace(
                    &mut core.calibration_digest,
                    calibration_digest,
                ));
            }
            retired.string_b = core.calibration_load_error.take();
            if let Some(next_poses) = next_pose_list {
                retired.pose_list = Some(std::mem::replace(&mut core.poses.list, next_poses));
            }
            if let Some(setup) = grasp_setup {
                let tilt = setup.camera_tilt_deg as f32;
                if core.config.camera_mount.tilt_deg != tilt {
                    ws_changed = true;
                }
                core.config.camera_mount.tilt_deg = tilt;
                retired.grasp_setup = core.config.grasp.replace(setup);
                core.grasp_setup_revision += 1;
            } else if let Some(g) = core.config.grasp.as_mut() {
                g.camera_tilt_deg = f64::from(core.config.camera_mount.tilt_deg);
            }
            if let Some((eff_floor, floor_src)) = effective_floor {
                core.floor_source = floor_src;
                if let Some(ws) = core.config.workspace.as_mut() {
                    if ws.floor_z_m != eff_floor {
                        ws.floor_z_m = eff_floor;
                        ws_changed = true;
                    }
                }
            }
            if let Some(bufs) = ws_eval_bufs {
                retired.eval_bufs = Some(std::mem::replace(&mut core.ws_eval_bufs, bufs));
            }
            if ws_changed {
                core.workspace_revision += 1;
                core.bump_motion_config_revision();
                core.retire_servo_ws_min_depths();
            }
            core.calibration_revision += 1;
            if !core.rt_executor_owned {
                core.refresh_schema_cache();
            }
        }
        ConfigCandidate::SoftwareJointZero {
            new_offsets,
            dof,
            is_hardware_motor_zero,
            zeroed_mask,
            raw_after,
            calibration,
            #[cfg(feature = "policy-runtime")]
            calibration_digest,
            next_arm_model,
            next_pose_list,
            next_gripper_stale,
            expected_gripper_calib_revision: _,
            expected_txn_id: _,
            operator_grant: _,
            effective_floor,
            ws_eval_bufs,
            reset_gripper_baseline,
            replacement_baseline,
            replacement_baseline_pending,
        } => {
            let mut ws_changed = false;
            let mut j6_changed = false;
            let n = dof.min(core.joint_zero_offsets.len()).min(crate::rt_core::mailbox::MAX_DOF);
            for i in 0..n {
                let old = core.joint_zero_offsets[i];
                let new_v = new_offsets[i];
                let d = old - new_v;
                core.joint_zero_offsets[i] = new_v;
                if is_hardware_motor_zero && (zeroed_mask & (1u64 << i)) != 0 {
                    if i == 5 {
                        j6_changed = true;
                    }
                    let pos_after = if raw_after[i].is_finite() {
                        raw_after[i] - new_v
                    } else {
                        -new_v
                    };
                    if let Some(p) = core.measured_positions.get_mut(i) {
                        *p = pos_after;
                    }
                    if let Some(p) = core.commanded_positions.get_mut(i) {
                        *p = pos_after;
                    }
                    if let Some(p) = core.target_positions.get_mut(i) {
                        *p = pos_after;
                    }
                } else if d != 0.0 {
                    if i == 5 {
                        j6_changed = true;
                    }
                    if let Some(p) = core.measured_positions.get_mut(i) {
                        *p += d;
                    }
                    if let Some(p) = core.commanded_positions.get_mut(i) {
                        *p += d;
                    }
                    if let Some(p) = core.target_positions.get_mut(i) {
                        *p += d;
                    }
                }
            }
            if let Some(ref mut d) = core.driver {
                let has_pos_limits = d
                    .with_mut(|drv| {
                        drv.set_joint_zero_offsets(&core.joint_zero_offsets[..n]);
                        (0..n).any(|i| drv.position_limits(i).is_some())
                    })
                    .unwrap_or(false);
                if has_pos_limits {
                    let _ = core.reapply_driver_limits_bounded();
                }
            }
            retired.tracking = core.tracking.invalidate_retiring();
            if j6_changed {
                core.zero_convention = Default::default();
            }
            core.zero_check_pending = true;
            if reset_gripper_baseline && core.gripper_baseline.is_some() {
                retired.gripper_baseline =
                    std::mem::replace(&mut core.gripper_baseline, replacement_baseline);
                core.gripper_last_contact = None;
                if let Some(fresh_pending) = replacement_baseline_pending {
                    retired.gripper_baseline_pending = Some(std::mem::replace(
                        &mut core.gripper_baseline_pending,
                        fresh_pending,
                    ));
                }
                core.gripper_baseline_generation += 1;
                core.gripper_baseline_dirty = true;
            } else {
                retired.gripper_baseline = replacement_baseline;
                retired.gripper_baseline_pending = replacement_baseline_pending;
            }
            if let Some(arm_model) = next_arm_model {
                retired.arm_model = Some(std::mem::replace(&mut core.config.arm_model, arm_model));
                ws_changed = true;
            }
            retired.calibration_file = std::mem::replace(&mut core.calibration, calibration);
            #[cfg(feature = "policy-runtime")]
            {
                retired.string_a = Some(std::mem::replace(
                    &mut core.calibration_digest,
                    calibration_digest,
                ));
            }
            if let Some((next_poses, bump_rev)) = next_pose_list {
                retired.pose_list = Some(std::mem::replace(&mut core.poses.list, next_poses));
                if bump_rev {
                    core.poses.revision += 1;
                }
            }
            if let Some(stale) = next_gripper_stale {
                retired.gripper_stale = core.gripper_calib_stale.replace(stale);
                core.gripper_calib_revision += 1;
            }
            if let Some((eff_floor, floor_src)) = effective_floor {
                core.floor_source = floor_src;
                if let Some(ws) = core.config.workspace.as_mut() {
                    if ws.floor_z_m != eff_floor {
                        ws.floor_z_m = eff_floor;
                        ws_changed = true;
                    }
                }
            }
            if let Some(bufs) = ws_eval_bufs {
                retired.eval_bufs = Some(std::mem::replace(&mut core.ws_eval_bufs, bufs));
            }
            if ws_changed {
                core.workspace_revision += 1;
                core.bump_motion_config_revision();
                core.retire_servo_ws_min_depths();
            }
            core.calibration_revision += 1;
            if !core.rt_executor_owned {
                core.refresh_schema_cache();
            }
        }
        ConfigCandidate::MitQualification {
            calibration,
            #[cfg(feature = "policy-runtime")]
            calibration_digest,
            expected_txn_id: _,
            operator_grant: _,
        } => {
            retired.calibration_file = std::mem::replace(&mut core.calibration, calibration);
            #[cfg(feature = "policy-runtime")]
            {
                retired.string_a = Some(std::mem::replace(
                    &mut core.calibration_digest,
                    calibration_digest,
                ));
            }
            retired.string_b = core.calibration_load_error.take();
            core.calibration_revision += 1;
            if !core.rt_executor_owned {
                core.refresh_schema_cache();
            }
        }
    }
}

fn install_finish_reconcile(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    req: FinishReconcileConfig,
    retired: &mut ConfigRetiredPayloads,
) -> Result<ConfigActivated, ConfigError> {
    let FinishReconcileConfig {
        domain,
        expected_from,
        proposal,
        auth,
        candidate,
        expected_workspace_revision,
        expected_poses_revision,
        cleanup_marker_txn_id,
    } = req;
    let is_reconciling = core
        .core
        .config_held(domain)
        .is_some_and(|h| h.state == crate::rt_core::config_iface::HoldState::Reconciling);
    if !is_reconciling {
        retired.unused_candidate = Some(candidate);
        return Err(ConfigError::NotHeld);
    }
    if let Err(err) = check_config_authority_inner(core, ctx, &proposal, &auth, false, false) {
        retired.unused_candidate = Some(candidate);
        return Err(err);
    }
    let current_rev = ConfigAuthority::revision(core, domain);
    if current_rev != expected_from {
        retired.unused_candidate = Some(candidate);
        return Err(ConfigError::RevisionMismatch {
            expected: expected_from,
            current: current_rev,
        });
    }
    if let Some(exp_ws) = expected_workspace_revision {
        if core.workspace_revision != exp_ws {
            retired.unused_candidate = Some(candidate);
            return Err(ConfigError::PlanStale(
                "workspace changed before configuration reconciliation completed",
            ));
        }
    }
    if let Some(exp_poses) = expected_poses_revision {
        if core.poses.revision != exp_poses {
            retired.unused_candidate = Some(candidate);
            return Err(ConfigError::PlanStale(
                "poses changed before configuration reconciliation completed",
            ));
        }
    }
    if let Err(code) = check_candidate_domain_invariants(core, ctx.now, &candidate, true) {
        retired.unused_candidate = Some(candidate);
        return Err(match code {
            "MOTION_BUSY" => ConfigError::MotionBusy {
                subject: "configuration",
                reason: "motion started before reconciliation completed",
            },
            "PLAN_STALE" => ConfigError::PlanStale("active configuration changed before reconciliation completed"),
            other => ConfigError::Reconciling(other),
        });
    }

    swap_config_candidate_in_place(core, candidate, retired);
    let cleared = ConfigAuthority::finish_reconcile(core, domain);
    debug_assert!(cleared);
    if let Some(cleanup_tid) = cleanup_marker_txn_id {
        // Retain calibration transaction exclusion atomically while the retry unlinks its
        // pending commit marker so a new calibration cannot overwrite the shared marker path.
        core.calibration_busy = true;
        core.calibration_txn_id = Some(cleanup_tid);
    }
    Ok(ConfigActivated::from_core(
        core,
        ConfigAuthority::revision(core, domain),
        Durability::Durable,
        ctx.now,
    ))
}

/// Zero-allocation owner-side fault latch called when bounded post-rename reconciliation
/// exhausts its retries or hits an unrecoverable authority/invariant divergence.
/// Keeps `HoldState::Reconciling` active and latches a non-recoverable fault naming the file.
pub(crate) fn install_latch_config_reconcile_fault(
    core: &mut SupervisorInner,
    domain: ConfigDomain,
    message: BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
) {
    let is_reconciling = core
        .core
        .config_held(domain)
        .is_some_and(|h| h.state == crate::rt_core::config_iface::HoldState::Reconciling);
    if !is_reconciling {
        return;
    }
    core.config_reconcile_fault = Some(BoundedText::new(message.as_str()));
    let detail = crate::safety::FaultDetail::formatted(crate::safety::FaultCode::Other, |buf| {
        use std::fmt::Write;
        let _ = buf.write_str(message.as_str());
    });
    core.safety.latch(crate::safety::FaultKind::Other, detail);
}

fn install_sample_table_capture(core: &SupervisorInner) -> Result<([f32; 8], usize), ConfigError> {
    if let Err(msg) = core.check_table_capture_stillness_static() {
        return Err(ConfigError::InvalidCommand(msg));
    }
    let len = core.measured_positions.len();
    if len > 8 {
        return Err(ConfigError::Unsupported("table_z_from_tcp supports at most 8 joints"));
    }
    let mut pos = [0.0f32; 8];
    pos[..len].copy_from_slice(&core.measured_positions);
    Ok((pos, len))
}

fn install_gravity_comp(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    op: GravityCompOp,
    retired: &mut ConfigRetiredPayloads,
) -> Result<GravityCompReply, ConfigError> {
    match op {
        GravityCompOp::Enter {
            admission,
            lease_holder,
            policy,
            scale,
            damping_kd,
        } => {
            if crate::gravity_comp::executor_blocks_hardware_gravity_comp(
                core.rt_executor_owned,
                core.driver.is_some(),
            ) {
                retired.gravity_comp_policy = policy;
                return Err(ConfigError::ExecutorCapabilityUnsupported(
                    "gravity_comp hardware MIT mode-switch is not supported on the RT executor",
                ));
            }
            if core.calibration_busy {
                retired.gravity_comp_policy = policy;
                return Err(ConfigError::CalibPrecondition {
                    status: 409,
                    code: "MOTION_BUSY",
                    message: "a calibration apply, restore or motor zero is running",
                });
            }
            if let Some((conn, gen)) = lease_holder {
                if gen == 0 || !core.validate_lease_at(conn.as_str(), gen, ctx.now) {
                    retired.gravity_comp_policy = policy;
                    return Err(ConfigError::LeaseLost);
                }
            }
            let admitted = match core.check_admission(admission) {
                Ok(a) => a,
                Err(e) => {
                    retired.gravity_comp_policy = policy;
                    return Err(match e {
                        crate::safety::SafetyError::StampAlreadyConsumed => ConfigError::StampConsumed,
                        crate::safety::SafetyError::LeaseLost => ConfigError::LeaseLost,
                        crate::safety::SafetyError::Stopped => ConfigError::Stopped,
                        crate::safety::SafetyError::EpochMismatch { .. } => ConfigError::EpochMismatch {
                            current_epoch: core.safety.epoch_id(),
                        },
                        _ => ConfigError::Stopped,
                    });
                }
            };
            match SupervisorInner::enter_gravity_comp_rt(admitted, policy, scale, damping_kd) {
                Ok(unused_policy) => {
                    retired.gravity_comp_policy = unused_policy;
                    Ok(GravityCompReply {
                        active: core.gravity_comp_active(),
                        epoch_id: core.safety.epoch_id(),
                        is_torqued: core.is_torqued,
                        safety_state: SafetyStateTag::from_state(core.safety.state()),
                    })
                }
                Err((reason, unused_policy)) => {
                    retired.gravity_comp_policy = unused_policy;
                    Err(ConfigError::GravityCompRejected {
                        code: reason.code,
                        message: reason.message,
                    })
                }
            }
        }
        GravityCompOp::Exit {
            lease_holder,
            code,
            message,
        } => {
            if crate::gravity_comp::executor_blocks_hardware_gravity_comp(
                core.rt_executor_owned,
                core.driver.is_some(),
            ) {
                return Err(ConfigError::ExecutorCapabilityUnsupported(
                    "gravity_comp hardware MIT mode-switch is not supported on the RT executor",
                ));
            }
            if core.calibration_busy {
                return Err(ConfigError::CalibPrecondition {
                    status: 409,
                    code: "MOTION_BUSY",
                    message: "a calibration apply, restore or motor zero is running",
                });
            }
            if let Some((conn, gen)) = lease_holder {
                if gen == 0 || !core.validate_lease_at(conn.as_str(), gen, ctx.now) {
                    return Err(ConfigError::LeaseLost);
                }
            }
            core.exit_gravity_comp_rt(code, message);
            Ok(GravityCompReply {
                active: core.gravity_comp_active(),
                epoch_id: core.safety.epoch_id(),
                is_torqued: core.is_torqued,
                safety_state: SafetyStateTag::from_state(core.safety.state()),
            })
        }
    }
}

// ============================================================================
// Off-thread orchestrator (shared by BOTH RtOwner::Legacy and RtOwner::Executor)
// ============================================================================

/// Builds a [`ConfigProposal`] stamped with live boot identity, monotonic deadline,
/// and the caller's ingress [`AdmissionStamp`].
pub fn build_config_proposal(
    sup: &ArmSupervisor,
    domain: ConfigDomain,
    expected_revision: u64,
    admission: AdmissionStamp,
    lease_generation: u64,
    workspace_revision: u64,
) -> ConfigProposal {
    let seq = NEXT_CONFIG_REQ_SEQ.fetch_add(1, Ordering::Relaxed);
    let boot = sup
        .executor_handle()
        .map_or_else(BootId::process, |h| h.boot());
    let now = Instant::now();
    ConfigProposal {
        domain,
        expected_revision,
        candidate: CandidateId(seq),
        stamp: Stamp {
            arm: ArmId::LOCAL,
            boot,
            request: RequestId(seq),
            client: ClientId(admission.connection_id),
            client_seq: seq,
            authority: Authority {
                lease_generation,
                epoch: admission.estop_epoch,
                stop_generation: admission.stop_gen,
                config_revision: workspace_revision,
                parent_generation: 0,
            },
            deadline: now + CONFIG_PROPOSAL_BUDGET,
        },
    }
}

/// Builds a [`ConfigProposal`] stamped with live boot identity, explicit `deadline`,
/// and the caller's ingress [`AdmissionStamp`].
pub fn build_config_proposal_with_deadline(
    sup: &ArmSupervisor,
    domain: ConfigDomain,
    expected_revision: u64,
    admission: AdmissionStamp,
    lease_generation: u64,
    workspace_revision: u64,
    deadline: Instant,
) -> ConfigProposal {
    let seq = NEXT_CONFIG_REQ_SEQ.fetch_add(1, Ordering::Relaxed);
    let boot = sup
        .executor_handle()
        .map_or_else(BootId::process, |h| h.boot());
    ConfigProposal {
        domain,
        expected_revision,
        candidate: CandidateId(seq),
        stamp: Stamp {
            arm: ArmId::LOCAL,
            boot,
            request: RequestId(seq),
            client: ClientId(admission.connection_id),
            client_seq: seq,
            authority: Authority {
                lease_generation,
                epoch: admission.estop_epoch,
                stop_generation: admission.stop_gen,
                config_revision: workspace_revision,
                parent_generation: 0,
            },
            deadline,
        },
    }
}

/// Captures a config-commit [`AdmissionStamp`] reflecting both `stop_gen` and any live
/// executor `StopRequests` generation without acquiring `SupervisorLock`.
pub fn capture_config_admission(
    sup: &ArmSupervisor,
    connection_id: u64,
    lease_gen: u64,
    epoch_override: Option<u64>,
) -> AdmissionStamp {
    let mut stamp = sup.capture_admission_stamp();
    if let Some(h) = sup.executor_handle() {
        stamp.stop_gen = stamp.stop_gen.max(h.stop_requests().current());
    }
    stamp.connection_id = connection_id;
    stamp.lease_gen = lease_gen;
    if let Some(ep) = epoch_override {
        stamp.estop_epoch = ep;
    }
    stamp
}

/// Optional test hooks injected on [`ArmSupervisor`] to deterministically exercise
/// rename failures, dir-sync failures, and mid-transaction revocations on both owners.
#[doc(hidden)]
#[derive(Clone, Default)]
pub struct ConfigCommitTestHooks {
    pub before_reserve: Option<Arc<dyn Fn() + Send + Sync>>,
    pub after_reserve_before_rename: Option<Arc<dyn Fn() + Send + Sync>>,
    pub after_rename_before_activate: Option<Arc<dyn Fn() + Send + Sync>>,
    pub fail_rename_once: Option<Arc<AtomicBool>>,
    /// Optional 0-based file index at which a multi-file batch rename fails once (`usize::MAX` = disabled).
    pub fail_rename_at_index: Option<Arc<AtomicUsize>>,
    pub fail_dir_sync_once: Option<Arc<AtomicBool>>,
    pub fail_persisted_once: Option<Arc<AtomicBool>>,
    /// Optional parent-directory fsync override invoked on both initial commit and reconciliation retries.
    pub dir_sync_override: Option<DirSyncOverride>,
    /// Override delay between post-rename reconciliation retries (defaults to `DEFAULT_RECONCILE_RETRY_INTERVAL`).
    pub reconcile_retry_interval: Option<std::time::Duration>,
    /// Override maximum post-rename reconciliation retry attempts (defaults to `DEFAULT_RECONCILE_MAX_ATTEMPTS`).
    pub reconcile_max_attempts: Option<u32>,
}

/// RAII config reservation guard for legacy zero and calibration transactions
/// (`motor_zero::run`, `motor_zero::restore`, `calibration::apply`, `calibration::restore`).
///
/// Acquires the same single-slot [`ConfigHold`] reservation (`ConfigDomain::Calibration`)
/// shared with all PR C configuration commits (`Poses`, `GripperCalibration`, `Keepout`,
/// `Collision`, `Sequences`, `GraspSetup`, `ToolCommissioning`) after verifying no staged
/// config commit currently holds `config_write_lock`. Holding this reservation across the
/// entire legacy zero + invalidation prevents any staged pose/gripper/calibration write from
/// interleaving or overwriting stale marks, and fails closed with `409 CONFIG_BUSY` if any
/// reservation or reconciliation is already active.
pub(crate) struct LegacyZeroReservationGuard {
    sup: Option<Arc<ArmSupervisor>>,
    reservation: Option<Reservation>,
}

impl LegacyZeroReservationGuard {
    pub(crate) async fn try_acquire(
        sup: &Arc<ArmSupervisor>,
    ) -> Result<Self, crate::calibration::CalibError> {
        if sup.owner_mode() != OWNER_LEGACY {
            return Err(crate::calibration::CalibError::new(
                409,
                "EXECUTOR_UNSUPPORTED",
                "operation is not available while the RT executor owns the arm; use the legacy owner",
            ));
        }
        // Fail closed immediately if any staged config commit currently holds `config_write_lock`
        // (including between lock acquisition and `ConfigRequest::Reserve`). Drop the mutex guard
        // right after installing the single-slot `ConfigHold` reservation so concurrent callers
        // reaching `Reserve` fail immediately with `409 CONFIG_BUSY` instead of blocking, and
        // legacy `Txn::write_grasp` can acquire `config_write_lock` without self-deadlocking.
        let cw_lock = sup.config_write_lock();
        let _cw_probe = cw_lock.try_lock().map_err(|_| {
            crate::calibration::CalibError::new(
                409,
                "CONFIG_BUSY",
                "a configuration change or reconciliation is in progress",
            )
        })?;
        let mut inner = sup.inner().write().await;
        if inner.detached {
            return Err(crate::calibration::CalibError::new(
                409,
                "EXECUTOR_UNSUPPORTED",
                "operation is not available while the RT executor owns the arm; use the legacy owner",
            ));
        }
        if inner.core.config().held().is_some() {
            return Err(crate::calibration::CalibError::new(
                409,
                "CONFIG_BUSY",
                "a configuration change or reconciliation is in progress",
            ));
        }
        let ctx = CommitCtx {
            arm: inner.core.id(),
            boot: BootId::process(),
            now: Instant::now(),
            stop_generation: inner.current_stop_gen(),
        };
        let seq = NEXT_CONFIG_REQ_SEQ.fetch_add(1, Ordering::Relaxed);
        let proposal = ConfigProposal {
            domain: ConfigDomain::Calibration,
            expected_revision: ConfigAuthority::revision(&*inner, ConfigDomain::Calibration),
            candidate: CandidateId(seq),
            stamp: Stamp {
                arm: ctx.arm,
                boot: ctx.boot,
                request: RequestId(seq),
                client: ClientId(0),
                client_seq: seq,
                authority: inner.authority(&ctx),
                deadline: ctx.now + std::time::Duration::from_secs(600),
            },
        };
        match ConfigAuthority::reserve(&mut *inner, &proposal, &ctx) {
            Reserved::Granted(reservation) => Ok(Self {
                sup: Some(Arc::clone(sup)),
                reservation: Some(reservation),
            }),
            Reserved::Refused { .. } => Err(crate::calibration::CalibError::new(
                409,
                "CONFIG_BUSY",
                "a configuration change or reconciliation is in progress",
            )),
        }
    }

    pub(crate) fn grant(&self) -> GrantId {
        self.reservation
            .as_ref()
            .expect("LegacyZeroReservationGuard holds a reservation until release")
            .grant()
    }

    pub(crate) async fn release(mut self) {
        if let (Some(sup), Some(reservation)) = (self.sup.take(), self.reservation.take()) {
            let mut inner = sup.inner().write().await;
            if !inner.detached {
                let _ = ConfigAuthority::release(&mut *inner, reservation);
            }
        }
    }
}

impl Drop for LegacyZeroReservationGuard {
    fn drop(&mut self) {
        let (Some(sup), Some(reservation)) = (self.sup.take(), self.reservation.take()) else {
            return;
        };
        if let Ok(mut inner) = sup.inner().try_write() {
            if !inner.detached {
                let _ = ConfigAuthority::release(&mut *inner, reservation);
            }
            return;
        }
        match tokio::runtime::Handle::try_current() {
            Ok(h) => {
                h.spawn(async move {
                    let mut inner = sup.inner().write().await;
                    if !inner.detached {
                        let _ = ConfigAuthority::release(&mut *inner, reservation);
                    }
                });
            }
            Err(_) => {
                let mut inner = sup.inner().blocking_write();
                if !inner.detached {
                    let _ = ConfigAuthority::release(&mut *inner, reservation);
                }
            }
        }
    }
}

/// RAII guard for a granted [`Reservation`] held off-thread across disk rename/activation.
///
/// If dropped early (cancelled future, panic, or early return):
/// - before rename (`renamed == false`): releases the hold (`Released::Cleared`);
/// - after rename (`renamed == true`): forces `PersistedConfig` + `ReleaseConfig` so
///   the hold enters `Reconciling` and motion stays inhibited.
pub struct ReservationGuard {
    sup: Arc<ArmSupervisor>,
    reservation: Option<Reservation>,
    renamed: bool,
    hw_mutated: bool,
    pub_phase: Option<Arc<AtomicU8>>,
}

impl ReservationGuard {
    pub fn new(sup: Arc<ArmSupervisor>, reservation: Reservation) -> Self {
        Self {
            sup,
            reservation: Some(reservation),
            renamed: false,
            hw_mutated: false,
            pub_phase: None,
        }
    }

    pub(crate) fn with_phase(
        sup: Arc<ArmSupervisor>,
        reservation: Reservation,
        pub_phase: Arc<AtomicU8>,
    ) -> Self {
        Self {
            sup,
            reservation: Some(reservation),
            renamed: false,
            hw_mutated: false,
            pub_phase: Some(pub_phase),
        }
    }

    pub fn grant(&self) -> Option<GrantId> {
        self.reservation.as_ref().map(Reservation::grant)
    }

    pub fn mark_renamed(&mut self) {
        self.renamed = true;
    }

    pub fn mark_hw_mutated(&mut self) {
        self.hw_mutated = true;
    }

    pub fn take(&mut self) -> Option<Reservation> {
        self.reservation.take()
    }
}

impl Drop for ReservationGuard {
    fn drop(&mut self) {
        let Some(reservation) = self.reservation.take() else {
            return;
        };
        let domain = reservation.domain();
        let sup = Arc::clone(&self.sup);
        let renamed = self.renamed
            || self.hw_mutated
            || self
                .pub_phase
                .as_ref()
                .is_some_and(|p| p.load(Ordering::SeqCst) >= PUB_PHASE_RENAMING);
        match tokio::runtime::Handle::try_current() {
            Ok(h) => {
                h.spawn(async move {
                    if renamed {
                        let res = sup
                            .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(
                                PersistedConfig {
                                    reservation,
                                    durability: Durability::DurableUnconfirmed,
                                },
                            ))
                            .await;
                        match res {
                            Ok(ConfigReply::Persisted(Ok(r))) => {
                                let rel = sup
                                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                                        ReleaseConfig { reservation: r },
                                    ))
                                    .await;
                                if !matches!(rel, Ok(ConfigReply::Released(Released::Reconciling))) {
                                    let _ = sup
                                        .dispatch_config_request_reliable(|| {
                                            ConfigRequest::ForceReconcile(domain)
                                        })
                                        .await;
                                }
                            }
                            _ => {
                                let _ = sup
                                    .dispatch_config_request_reliable(|| {
                                        ConfigRequest::ForceReconcile(domain)
                                    })
                                    .await;
                            }
                        }
                    } else {
                        let _ = sup
                            .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                                ReleaseConfig { reservation },
                            ))
                            .await;
                    }
                });
            }
            Err(_) => {
                // Synchronous fallback outside a Tokio runtime (unit tests).
                if sup.owner_mode() == OWNER_EXECUTOR {
                    if let Some(h) = sup.executor_handle() {
                        if renamed {
                            if let Ok(ConfigReply::Persisted(Ok(r))) =
                                h.call_config(ConfigRequest::Persisted(PersistedConfig {
                                    reservation,
                                    durability: Durability::DurableUnconfirmed,
                                }))
                            {
                                let _ = h.call_config(ConfigRequest::Release(ReleaseConfig {
                                    reservation: r,
                                }));
                            } else {
                                let _ = h.call_config(ConfigRequest::ForceReconcile(domain));
                            }
                        } else {
                            let _ = h.call_config(ConfigRequest::Release(ReleaseConfig {
                                reservation,
                            }));
                        }
                    }
                } else if sup.owner_mode() == OWNER_LEGACY {
                    let mut inner = sup.inner().blocking_write();
                    if !inner.detached {
                        let ctx = CommitCtx {
                            arm: inner.core.id(),
                            boot: BootId::process(),
                            now: Instant::now(),
                            stop_generation: inner.current_stop_gen(),
                        };
                        if renamed {
                            let mut ex = ConfigExchange::new(ConfigRequest::Persisted(PersistedConfig {
                                reservation,
                                durability: Durability::DurableUnconfirmed,
                            }));
                            handle_config_exchange(&mut inner, &ctx, &mut ex);
                            if let Some(ConfigReply::Persisted(Ok(r))) = ex.reply.take() {
                                let mut rel = ConfigExchange::new(ConfigRequest::Release(ReleaseConfig {
                                    reservation: r,
                                }));
                                handle_config_exchange(&mut inner, &ctx, &mut rel);
                            } else {
                                let mut frc = ConfigExchange::new(ConfigRequest::ForceReconcile(domain));
                                handle_config_exchange(&mut inner, &ctx, &mut frc);
                            }
                        } else {
                            let mut rel = ConfigExchange::new(ConfigRequest::Release(ReleaseConfig {
                                reservation,
                            }));
                            handle_config_exchange(&mut inner, &ctx, &mut rel);
                        }
                    }
                }
            }
        }
    }
}

/// Single filesystem mutation inside a [`ConfigWriteOp::Batch`].
#[derive(Debug, Clone)]
pub enum FileWriteOp {
    /// Stage and atomically rename `PendingWrite` over its target path.
    Replace(PendingWrite),
    /// Remove the file at `PathBuf` (`NotFound` is treated as a no-op that did not mutate disk).
    Remove(PathBuf),
}

impl FileWriteOp {
    pub fn target_path(&self) -> &Path {
        match self {
            Self::Replace(pw) => &pw.path,
            Self::Remove(p) => p,
        }
    }
}

/// Filesystem operation performed during a unified [`run_config_commit_detailed`] transaction.
///
/// Every persisted write runs its staging, file fsync, rename, and parent-directory fsync
/// strictly AFTER [`ReserveConfig`] and strictly BEFORE [`ActivateConfig`].
pub enum ConfigWriteOp {
    /// Memory-only (volatile) configuration update with no disk file.
    Volatile,
    /// Stage `PendingWrite` off-thread after reservation, then rename + dir-sync before activation.
    StageAndReplace(PendingWrite),
    /// Ordered batch of file replacements/removals (1..=3 files) with an optional durable
    /// pending-commit marker file written before pre-rename verification (or an already-written
    /// marker created before external vision PUT) and removed only after live activation or
    /// bounded reconciliation succeeds.
    Batch {
        ops: Vec<FileWriteOp>,
        pending_marker: Option<(PathBuf, String)>,
        /// Pre-existing durable intent marker created before an external side effect (e.g.
        /// rebot-vision PUT). Removed ONLY after live activation or reconciliation succeeds;
        /// left for the outer transaction to clean up if pre-rename rollback succeeds.
        existing_marker: Option<PathBuf>,
    },
}

impl From<Option<PendingWrite>> for ConfigWriteOp {
    fn from(w: Option<PendingWrite>) -> Self {
        match w {
            Some(pw) => Self::StageAndReplace(pw),
            None => Self::Volatile,
        }
    }
}

/// Inputs to the unified off-thread two-phase config commit orchestrator.
pub struct ConfigCommitPlan {
    pub proposal: ConfigProposal,
    pub auth: ConfigAuthContext,
    pub precheck: ReservePrecheck,
    pub candidate: ConfigCandidate,
    pub write: ConfigWriteOp,
    /// When `Some(rev)`, enforces `core.workspace_revision == rev` at reserve, pre-commit verify, and activate.
    pub expected_workspace_revision: Option<u64>,
    /// When `Some(rev)`, enforces `core.poses.revision == rev` at reserve, pre-commit verify, and activate.
    pub expected_poses_revision: Option<u64>,
}

/// Outcome of [`run_config_commit`] / [`run_config_commit_detailed`].
#[derive(Debug, Clone)]
pub struct ConfigCommitOutcome {
    pub activated: ConfigActivated,
    pub persisted: bool,
    pub durability_confirmed: bool,
    pub durability_warning: Option<String>,
}

impl ConfigCommitOutcome {
    pub fn disposition(&self) -> CommitDisposition {
        CommitDisposition::Applied
    }
}

/// Typed failure outcome from [`run_config_commit_detailed`] preserving whether disk publication
/// happened, whether the owner entered `HoldState::Reconciling`, and whether activation outcome
/// is unknown (transport error / worker panic after publication).
#[derive(Debug)]
pub struct ConfigCommitError {
    pub error: ArmError,
    pub disk_published: bool,
    pub reconciling: bool,
    pub outcome_unknown: bool,
}

impl ConfigCommitError {
    fn pre_publish(error: ArmError) -> Self {
        Self {
            error,
            disk_published: false,
            reconciling: false,
            outcome_unknown: false,
        }
    }

    fn post_publish_reconciling(error: ArmError) -> Self {
        Self {
            error,
            disk_published: true,
            reconciling: true,
            outcome_unknown: false,
        }
    }

    fn post_publish_unknown(error: ArmError) -> Self {
        Self {
            error,
            disk_published: true,
            reconciling: true,
            outcome_unknown: true,
        }
    }

    pub fn disposition(&self) -> CommitDisposition {
        if self.disk_published || self.reconciling || self.outcome_unknown {
            CommitDisposition::Uncertain
        } else {
            CommitDisposition::Unattempted
        }
    }
}

const PUB_PHASE_PRE_RENAME: u8 = 0;
const PUB_PHASE_RENAMING: u8 = 1;
const PUB_PHASE_RENAMED: u8 = 2;

/// Unified, cancellation-safe off-thread two-phase config commit orchestrator for BOTH
/// `RtOwner::Legacy` and `RtOwner::Executor`.
///
/// Runs inside a detached worker task so dropping/cancelling the caller's future mid-`Reserve`,
/// mid-`rename_only`, or mid-`remove_file` NEVER strands a reservation or clears a hold while a
/// blocking rename/unlink publishes to disk: the worker retains the exact filesystem outcome and
/// drives `ReleaseConfig` (`Cleared`) or `PersistedConfig` + `ActivateConfig`/`Reconciling` to
/// completion using queue-backpressure-resilient owner dispatch.
pub async fn run_config_commit_detailed<FAuth, FPub>(
    sup: &Arc<ArmSupervisor>,
    plan: ConfigCommitPlan,
    auth_check: FAuth,
    update_published: FPub,
) -> Result<ConfigCommitOutcome, ConfigCommitError>
where
    FAuth: Fn() -> Result<(), (&'static str, ArmError)> + Send + 'static,
    FPub: FnOnce(&mut PublishedConfig, ConfigActivated) + Send + 'static,
{
    let sup_w = Arc::clone(sup);
    let pub_phase = Arc::new(AtomicU8::new(PUB_PHASE_PRE_RENAME));
    let pub_phase_w = Arc::clone(&pub_phase);
    let (done_tx, done_rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        let res = run_config_commit_worker(sup_w, plan, None, false, auth_check, update_published, pub_phase_w).await;
        let _ = done_tx.send(res);
    });
    match done_rx.await {
        Ok(res) => res,
        Err(_) => {
            if pub_phase.load(Ordering::Acquire) != PUB_PHASE_PRE_RENAME {
                Err(ConfigCommitError::post_publish_unknown(ArmError::Internal(
                    "configuration commit worker aborted after disk publication began; reconciliation required".into(),
                )))
            } else {
                Err(ConfigCommitError::pre_publish(ArmError::Internal(
                    "configuration commit worker aborted".into(),
                )))
            }
        }
    }
}

/// Runs the unified off-thread two-phase config commit orchestrator on an ALREADY HELD
/// [`Reservation`] (e.g. a single uninterrupted `Calibration` reservation held across hardware
/// commissioning readback/write + staging + both commit boundaries + `PublishedConfigHub` update).
///
/// When `hw_mutated` is `true`, hardware registers/flash have already been mutated:
/// - Any pre-rename, post-rename, or dir-sync failure enters `HoldState::Reconciling` and latches
///   a non-recoverable fault rather than releasing `Cleared` or spawning config-only dir-sync retry.
/// - Any pending commit marker is preserved on failure so restart quarantines until reconciled.
pub async fn run_config_commit_on_held_reservation<FAuth, FPub>(
    sup: &Arc<ArmSupervisor>,
    plan: ConfigCommitPlan,
    reservation: Reservation,
    hw_mutated: bool,
    auth_check: FAuth,
    update_published: FPub,
) -> Result<ConfigCommitOutcome, ConfigCommitError>
where
    FAuth: Fn() -> Result<(), (&'static str, ArmError)> + Send + 'static,
    FPub: FnOnce(&mut PublishedConfig, ConfigActivated) + Send + 'static,
{
    let sup_w = Arc::clone(sup);
    let pub_phase = Arc::new(AtomicU8::new(if hw_mutated {
        PUB_PHASE_RENAMING
    } else {
        PUB_PHASE_PRE_RENAME
    }));
    let pub_phase_w = Arc::clone(&pub_phase);
    let (done_tx, done_rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        let res = run_config_commit_worker(
            sup_w,
            plan,
            Some(reservation),
            hw_mutated,
            auth_check,
            update_published,
            pub_phase_w,
        )
        .await;
        let _ = done_tx.send(res);
    });
    match done_rx.await {
        Ok(res) => res,
        Err(_) => {
            if hw_mutated || pub_phase.load(Ordering::Acquire) != PUB_PHASE_PRE_RENAME {
                Err(ConfigCommitError::post_publish_unknown(ArmError::Internal(
                    "configuration commit worker aborted after hardware/disk publication began; reconciliation required".into(),
                )))
            } else {
                Err(ConfigCommitError::pre_publish(ArmError::Internal(
                    "configuration commit worker aborted".into(),
                )))
            }
        }
    }
}

/// Convenience wrapper over [`run_config_commit_detailed`] returning `ArmError`.
pub async fn run_config_commit<FAuth, FPub>(
    sup: &Arc<ArmSupervisor>,
    plan: ConfigCommitPlan,
    auth_check: FAuth,
    update_published: FPub,
) -> Result<ConfigCommitOutcome, ArmError>
where
    FAuth: Fn() -> Result<(), (&'static str, ArmError)> + Send + 'static,
    FPub: FnOnce(&mut PublishedConfig, ConfigActivated) + Send + 'static,
{
    run_config_commit_detailed(sup, plan, auth_check, update_published)
        .await
        .map_err(|e| e.error)
}

/// Optional parent-directory fsync override used by deterministic executor tests.
pub(crate) type DirSyncOverride = Arc<dyn Fn(&std::path::Path) -> std::io::Result<()> + Send + Sync>;

/// Owner-transport adapter driving the ONE common commit worker ([`execute_common_commit`])
/// for every PR C config family (`run_config_commit_worker`) AND `gripper_zero_check(store_offset=true)`
/// on BOTH `RtOwner::Legacy` (async + sync) and `RtOwner::Executor` (`armd-persist`).
pub(crate) trait CommonCommitWorker: Send {
    type Ok: Send;
    type Err: Send;

    /// When `true`, blocking filesystem steps (`StagedWrite::stage`, `rename_only`,
    /// `sync_parent_dir`) run on `tokio::task::spawn_blocking`; when `false` (dedicated blocking
    /// thread `armd-persist` or synchronous unit tests), they execute directly inline.
    const RUN_BLOCKING_ON_TOKIO: bool;

    fn before_reserve(&mut self) {}
    fn should_fail_rename_once(&self) -> bool {
        false
    }
    fn fail_rename_at_index(&self) -> Option<usize> {
        None
    }
    fn should_fail_dir_sync_once(&self) -> bool {
        false
    }
    fn dir_sync_override(&self) -> Option<DirSyncOverride> {
        None
    }
    /// Step 1: Reserve domain on the single live owner strictly BEFORE any serialization or staging.
    fn reserve(&mut self) -> impl std::future::Future<Output = Result<(), Self::Err>> + Send;
    /// Step 1b: Serialize / produce `Option<PendingWrite>` (and pin replay episodes for `Sequences`)
    /// strictly AFTER `reserve()` succeeded.
    fn prepare_write_after_reserve(
        &mut self,
    ) -> impl std::future::Future<Output = Result<Option<PendingWrite>, Self::Err>> + Send;
    /// Step 1b (generalized): produce a [`ConfigWriteOp`] (single write, volatile, or multi-file batch).
    /// Defaults to wrapping [`Self::prepare_write_after_reserve`].
    fn prepare_write_op_after_reserve(
        &mut self,
    ) -> impl std::future::Future<Output = Result<ConfigWriteOp, Self::Err>> + Send {
        async { self.prepare_write_after_reserve().await.map(ConfigWriteOp::from) }
    }
    /// Optional pre-rename hook; returning `true` simulates worker disconnect (without Release).
    fn after_reserve_before_rename(&mut self) -> bool {
        false
    }
    /// Step 3: Fresh pre-rename off-thread auth + live owner authority recheck.
    fn verify_authority_before_rename(
        &mut self,
    ) -> impl std::future::Future<Output = Result<(), Self::Err>> + Send;
    /// Called immediately before the blocking rename/remove step starts (marks publication phase).
    fn on_rename_starting(&mut self) {}
    /// Called immediately after at least one file rename/remove succeeded (before `mark_persisted`).
    fn on_renamed(&mut self) {}
    /// Step 4b: Record `Persisted(DurableUnconfirmed)` on the live owner immediately after rename.
    fn mark_persisted(&mut self) -> impl std::future::Future<Output = Result<(), Self::Err>> + Send;
    /// Step 4 error (partial batch): a later rename/remove failed AFTER >=1 earlier file mutated disk.
    /// Must enter `Reconciling` and latch a non-recoverable fault (never retry half-published batch).
    fn reconcile_after_partial_rename(
        &mut self,
        _failed_path: PathBuf,
        err: String,
    ) -> impl std::future::Future<Output = Self::Err> + Send {
        self.reconcile_rename_join_error(err)
    }
    /// Step 5 error: Parent-directory fsync failed after rename -> enter `Reconciling`, NO Activate,
    /// and kick off bounded off-RT reconciliation retry for `target_path`.
    fn reconcile_after_dir_sync_failure(
        &mut self,
        target_path: PathBuf,
        warn: String,
    ) -> impl std::future::Future<Output = Self::Err> + Send;
    /// Step 5 error (multi-path): parent-directory fsync failed across one or more target paths.
    fn reconcile_after_dir_sync_failure_paths(
        &mut self,
        target_paths: Vec<PathBuf>,
        warn: String,
    ) -> impl std::future::Future<Output = Self::Err> + Send {
        let first = target_paths
            .into_iter()
            .next()
            .unwrap_or_else(|| PathBuf::from("state"));
        self.reconcile_after_dir_sync_failure(first, warn)
    }
    /// Optional post-dir-sync hook; returning `true` simulates worker disconnect (without Activate).
    fn after_rename_before_activate(&mut self) -> bool {
        false
    }
    /// Step 6: Fresh post-dir-sync authority recheck + atomic activation + `PublishedConfigHub` update.
    fn activate_and_publish(
        &mut self,
        disk_published: bool,
    ) -> impl std::future::Future<Output = Result<Self::Ok, Self::Err>> + Send;
    /// Pre-rename release (`Cleared`) after a failure in `prepare_write_after_reserve` or `verify_authority_before_rename`.
    fn release_pre_rename(&mut self) -> impl std::future::Future<Output = ()> + Send;
    /// Pre-rename filesystem error (`stage` or `rename_only` failed before publishing): release `Cleared` and build `Err`.
    fn abort_pre_rename_fs(
        &mut self,
        phase: &'static str,
        err: String,
    ) -> impl std::future::Future<Output = Self::Err> + Send;
    /// Blocking rename task panicked/joined with error after starting: enter `Reconciling` and build `Err`.
    fn reconcile_rename_join_error(
        &mut self,
        err: String,
    ) -> impl std::future::Future<Output = Self::Err> + Send;
    /// Called when an activated commit's pending-commit marker could not be removed cleanly.
    fn note_marker_cleanup_warning(&mut self, _ok: &mut Self::Ok, _warn: String) {}
    /// Called when a marker written by this worker could not be removed after pre-rename abort:
    /// must enter `Reconciling` rather than silently releasing `Cleared` while a marker remains.
    fn reconcile_after_marker_error(
        &mut self,
        err: String,
    ) -> impl std::future::Future<Output = Self::Err> + Send {
        self.reconcile_rename_join_error(err)
    }
    /// When true (hardware already mutated), any pending-commit marker is preserved on abort
    /// so restart quarantines until physical hardware and disk are reconciled.
    fn preserve_marker_on_abort(&self) -> bool {
        false
    }
    fn worker_disconnected_err(&mut self) -> Self::Err;
}

enum StagedFileOp {
    Replace(StagedWrite),
    Remove(PathBuf),
}

fn abort_staged_ops(ops: Vec<StagedFileOp>) {
    for op in ops {
        if let StagedFileOp::Replace(staged) = op {
            staged.abort();
        }
    }
}

fn unique_dir_syncs(dir_syncs: Vec<PendingDirSync>) -> Vec<PendingDirSync> {
    let mut seen: Vec<PathBuf> = Vec::with_capacity(dir_syncs.len());
    let mut out: Vec<PendingDirSync> = Vec::with_capacity(dir_syncs.len());
    for ds in dir_syncs {
        let parent = ds
            .target()
            .parent()
            .filter(|d| !d.as_os_str().is_empty())
            .unwrap_or(Path::new("."))
            .to_path_buf();
        if !seen.contains(&parent) {
            seen.push(parent);
            out.push(ds);
        }
    }
    out
}

async fn run_fs_step<T: Send + 'static>(
    use_tokio: bool,
    f: impl FnOnce() -> T + Send + 'static,
) -> Result<T, String> {
    if use_tokio {
        tokio::task::spawn_blocking(f)
            .await
            .map_err(|e| e.to_string())
    } else {
        Ok(f())
    }
}

/// THE ONE common commit worker for EVERY config family and `gripper_zero_check(store_offset=true)`
/// on BOTH `RtOwner::Legacy` and `RtOwner::Executor`.
///
/// Enforces the mandatory ordering in one single place:
/// `reserve -> off-thread serialize + stage + file fsync -> authority recheck ->
///  rename -> parent-directory fsync -> authority recheck -> activate/publish`.
///
/// - Any failure before `rename` aborts staging and releases (`Cleared`); live state is untouched.
/// - Any failure at or after `rename` (including parent-directory fsync failure) skips activation,
///   keeps the old policy active, and enters `Reconciling` with motion inhibited.
pub(crate) async fn execute_common_commit<W: CommonCommitWorker>(
    w: &mut W,
) -> Result<W::Ok, W::Err> {
    w.before_reserve();

    // 1. Reserve domain on the single live owner strictly BEFORE serialization or staging.
    w.reserve().await?;

    // 1b. Produce serialized write(s) (and pin sequence replay episodes) under the granted reservation.
    let write_op = match w.prepare_write_op_after_reserve().await {
        Ok(op) => op,
        Err(err) => {
            w.release_pre_rename().await;
            return Err(err);
        }
    };

    let (file_ops, pending_marker, existing_marker): (
        Vec<FileWriteOp>,
        Option<(PathBuf, String)>,
        Option<PathBuf>,
    ) = match write_op {
        ConfigWriteOp::Volatile => (Vec::new(), None, None),
        ConfigWriteOp::StageAndReplace(pw) => (vec![FileWriteOp::Replace(pw)], None, None),
        ConfigWriteOp::Batch {
            ops,
            pending_marker,
            existing_marker,
        } => (ops, pending_marker, existing_marker),
    };

    // 2. Off-thread stage + file fsync (`StagedWrite::stage`) strictly AFTER reserve.
    let mut staged_ops: Vec<StagedFileOp> = Vec::with_capacity(file_ops.len());
    for op in file_ops {
        match op {
            FileWriteOp::Replace(pw) => {
                let stage_res = run_fs_step(W::RUN_BLOCKING_ON_TOKIO, move || {
                    StagedWrite::stage(pw.path, pw.text)
                })
                .await
                .map_err(|e| format!("stage task failed: {e}"))
                .and_then(|r| r);
                match stage_res {
                    Ok(staged) => staged_ops.push(StagedFileOp::Replace(staged)),
                    Err(e) => {
                        abort_staged_ops(staged_ops);
                        tracing::warn!("State file not staged: {e}");
                        return Err(w.abort_pre_rename_fs("stage", e).await);
                    }
                }
            }
            FileWriteOp::Remove(path) => staged_ops.push(StagedFileOp::Remove(path)),
        }
    }

    // 2b. Write & durably fsync any worker-owned pending-commit marker BEFORE boundary-1
    //     authority verification so no blocking marker I/O runs between recheck and rename.
    let mut worker_marker: Option<PathBuf> = None;
    if !staged_ops.is_empty() {
        if let Some((marker_path, marker_text)) = pending_marker {
            let mp = marker_path.clone();
            let marker_res = run_fs_step(W::RUN_BLOCKING_ON_TOKIO, move || {
                let staged = StagedWrite::stage(mp.clone(), marker_text)
                    .map_err(|e| (false, e))?;
                let ds = staged.rename_only().map_err(|e| (false, e))?;
                match ds.sync_parent_dir() {
                    PublishOutcome::Published => Ok(()),
                    PublishOutcome::PublishedDurabilityUnconfirmed(w) => Err((
                        true,
                        format!("marker {} renamed but dir fsync failed: {w}", mp.display()),
                    )),
                }
            })
            .await
            .map_err(|e| (true, format!("marker task failed: {e}")))
            .and_then(|r| r);
            match marker_res {
                Ok(()) => worker_marker = Some(marker_path),
                Err((false, e)) => {
                    abort_staged_ops(staged_ops);
                    tracing::warn!("Pending commit marker not written: {e}");
                    return Err(w.abort_pre_rename_fs("stage", e).await);
                }
                Err((true, e)) => {
                    abort_staged_ops(staged_ops);
                    let mp_clean = marker_path.clone();
                    let rm_res = run_fs_step(W::RUN_BLOCKING_ON_TOKIO, move || {
                        remove_pending_commit_marker(&mp_clean)
                    })
                    .await
                    .map_err(|re| re.to_string())
                    .and_then(|r| r);
                    if let Err(rm_err) = rm_res {
                        tracing::error!(
                            "Pending commit marker left after failed dir fsync ({e}); cleanup failed ({rm_err})"
                        );
                        return Err(w.reconcile_after_marker_error(format!("{e}; cleanup: {rm_err}")).await);
                    }
                    return Err(w.abort_pre_rename_fs("stage", e).await);
                }
            }
        }
    }

    if w.after_reserve_before_rename() {
        abort_staged_ops(staged_ops);
        return Err(w.worker_disconnected_err());
    }

    // 3. Fresh pre-rename off-thread auth + live owner authority recheck (immediately before
    //    blocking target renames; no blocking marker I/O runs after this check).
    if let Err(err) = w.verify_authority_before_rename().await {
        abort_staged_ops(staged_ops);
        if !w.preserve_marker_on_abort() {
            if let Some(mp) = worker_marker.take() {
                let rm_res = run_fs_step(W::RUN_BLOCKING_ON_TOKIO, move || {
                    remove_pending_commit_marker(&mp)
                })
                .await
                .map_err(|re| re.to_string())
                .and_then(|r| r);
                if let Err(rm_err) = rm_res {
                    tracing::error!(
                        "Pre-rename verification failed and pending marker cleanup failed ({rm_err}); entering Reconciling"
                    );
                    return Err(w.reconcile_after_marker_error(rm_err).await);
                }
            }
        }
        w.release_pre_rename().await;
        return Err(err);
    }

    // 4. Rename/remove off-thread, mark Persisted(DurableUnconfirmed), then parent-directory fsync
    //    strictly BEFORE activation.
    let mut disk_published = false;
    let mut cleanup_marker_on_activate: Option<PathBuf> =
        worker_marker.clone().or(existing_marker);

    if !staged_ops.is_empty() {
        let sim_rename_fail = w.should_fail_rename_once();
        let fail_idx = w.fail_rename_at_index();
        w.on_rename_starting();

        let rename_outcome = run_fs_step(W::RUN_BLOCKING_ON_TOKIO, move || {
            let mut dir_syncs: Vec<PendingDirSync> = Vec::with_capacity(staged_ops.len());
            let mut iter = staged_ops.into_iter().enumerate();
            while let Some((idx, op)) = iter.next() {
                if (sim_rename_fail && idx == 0) || fail_idx == Some(idx) {
                    let failed_path = match op {
                        StagedFileOp::Replace(s) => {
                            let p = s.target_path().to_path_buf();
                            s.abort();
                            p
                        }
                        StagedFileOp::Remove(p) => p,
                    };
                    for (_, rem) in iter {
                        if let StagedFileOp::Replace(s) = rem {
                            s.abort();
                        }
                    }
                    return Err((
                        !dir_syncs.is_empty(),
                        failed_path,
                        format!("simulated rename failure at index {idx}"),
                    ));
                }
                match op {
                    StagedFileOp::Replace(staged) => {
                        let target = staged.target_path().to_path_buf();
                        match staged.rename_only() {
                            Ok(ds) => dir_syncs.push(ds),
                            Err(e) => {
                                for (_, rem) in iter {
                                    if let StagedFileOp::Replace(s) = rem {
                                        s.abort();
                                    }
                                }
                                return Err((!dir_syncs.is_empty(), target, e));
                            }
                        }
                    }
                    StagedFileOp::Remove(path) => match std::fs::remove_file(&path) {
                        Ok(()) => dir_syncs.push(PendingDirSync::for_path(&path)),
                        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                        Err(e) => {
                            for (_, rem) in iter {
                                if let StagedFileOp::Replace(s) = rem {
                                    s.abort();
                                }
                            }
                            return Err((!dir_syncs.is_empty(), path, e.to_string()));
                        }
                    },
                }
            }
            Ok(dir_syncs)
        })
        .await;

        let dir_syncs = match rename_outcome {
            Ok(Ok(ds)) => ds,
            Ok(Err((false, _failed_path, e))) => {
                if !w.preserve_marker_on_abort() {
                    if let Some(mp) = worker_marker.take() {
                        let rm_res = run_fs_step(W::RUN_BLOCKING_ON_TOKIO, move || {
                            remove_pending_commit_marker(&mp)
                        })
                        .await
                        .map_err(|re| re.to_string())
                        .and_then(|r| r);
                        if let Err(rm_err) = rm_res {
                            tracing::error!(
                                "Pre-rename publish failed ({e}) and marker cleanup failed ({rm_err}); entering Reconciling"
                            );
                            return Err(w.reconcile_after_marker_error(format!("{e}; cleanup: {rm_err}")).await);
                        }
                    }
                }
                tracing::warn!("State file not published: {e}");
                return Err(w.abort_pre_rename_fs("rename", e).await);
            }
            Ok(Err((true, failed_path, e))) => {
                w.on_renamed();
                tracing::error!(
                    "Partial batch rename failure at {} ({e}); entering Reconciling and latching fault",
                    failed_path.display()
                );
                return Err(w.reconcile_after_partial_rename(failed_path, e).await);
            }
            Err(e) => {
                w.on_renamed();
                return Err(w.reconcile_rename_join_error(e).await);
            }
        };

        if !dir_syncs.is_empty() {
            w.on_renamed();
            disk_published = true;

            let target_paths: Vec<PathBuf> =
                dir_syncs.iter().map(|ds| ds.target().to_path_buf()).collect();
            let unique_syncs = unique_dir_syncs(dir_syncs);

            // 4b. Mark Persisted(DurableUnconfirmed) immediately after rename.
            w.mark_persisted().await?;

            // 5. Parent-directory fsync strictly AFTER rename and strictly BEFORE activation.
            //    A failure here MUST NOT activate the new policy; it enters Reconciling and keeps
            //    motion inhibited until bounded reconciliation succeeds (or latches a fault).
            let dir_sync_res: Result<(), String> = if w.should_fail_dir_sync_once() {
                Err("simulated parent dir fsync failure".into())
            } else {
                let override_fn = w.dir_sync_override();
                run_fs_step(W::RUN_BLOCKING_ON_TOKIO, move || {
                    for ds in unique_syncs {
                        let outcome = match override_fn.as_ref() {
                            Some(f) => ds.sync_with(|dir| f(dir)),
                            None => ds.sync_parent_dir(),
                        };
                        if let PublishOutcome::PublishedDurabilityUnconfirmed(warn) = outcome {
                            return Err(warn);
                        }
                    }
                    Ok(())
                })
                .await
                .map_err(|e| e.to_string())
                .and_then(|r| r)
            };

            if let Err(warn) = dir_sync_res {
                tracing::error!(
                    "Parent directory fsync failed after rename for {:?} ({warn}); keeping old policy active and entering Reconciling",
                    target_paths
                );
                return Err(w.reconcile_after_dir_sync_failure_paths(target_paths, warn).await);
            }

            if w.after_rename_before_activate() {
                return Err(w.worker_disconnected_err());
            }
        }
    }

    // 6. Fresh post-dir-sync authority recheck + atomic activation + PublishedConfigHub update.
    let mut res = w.activate_and_publish(disk_published).await;
    if let Ok(ref mut ok) = res {
        if let Some(mp) = cleanup_marker_on_activate.take() {
            let rm_res = run_fs_step(W::RUN_BLOCKING_ON_TOKIO, move || {
                remove_pending_commit_marker(&mp)
            })
            .await
            .map_err(|re| re.to_string())
            .and_then(|r| r);
            if let Err(rm_err) = rm_res {
                w.note_marker_cleanup_warning(ok, rm_err);
            }
        }
    } else if !disk_published && !w.preserve_marker_on_abort() {
        if let Some(mp) = worker_marker.take() {
            let rm_res = run_fs_step(W::RUN_BLOCKING_ON_TOKIO, move || {
                remove_pending_commit_marker(&mp)
            })
            .await
            .map_err(|re| re.to_string())
            .and_then(|r| r);
            if let Err(rm_err) = rm_res {
                return Err(w.reconcile_after_marker_error(rm_err).await);
            }
        }
    }
    res
}

/// Synchronously drives [`execute_common_commit`] to completion for dedicated blocking workers
/// (`armd-persist` OS thread on Executor and non-deferred `SupervisorInner` legacy tests).
pub(crate) fn run_common_commit_sync<W: CommonCommitWorker>(w: &mut W) -> Result<W::Ok, W::Err> {
    use std::future::Future;
    use std::task::{Context, Poll, Waker};
    let mut cx = Context::from_waker(Waker::noop());
    let mut fut = std::pin::pin!(execute_common_commit(w));
    match fut.as_mut().poll(&mut cx) {
        Poll::Ready(res) => res,
        Poll::Pending => unreachable!("sync CommonCommitWorker backend returned Poll::Pending"),
    }
}

struct ConfigCommitBackend<FAuth, FPub> {
    sup: Arc<ArmSupervisor>,
    hooks: ConfigCommitTestHooks,
    proposal: ConfigProposal,
    auth: ConfigAuthContext,
    precheck: Option<ReservePrecheck>,
    domain_checks: VerifyDomainChecks,
    candidate: Option<ConfigCandidate>,
    write_op: Option<ConfigWriteOp>,
    pending_marker_path: Option<PathBuf>,
    expected_workspace_revision: Option<u64>,
    expected_poses_revision: Option<u64>,
    replay_episode_refs: Vec<(usize, String)>,
    auth_check: Option<FAuth>,
    update_published: Option<FPub>,
    guard: Option<ReservationGuard>,
    pre_held: Option<Reservation>,
    hw_mutated: bool,
    grant: Option<GrantId>,
    domain: ConfigDomain,
    episode_pin_guard: Option<crate::recording::store::EpisodePinGuard>,
    pub_phase: Arc<AtomicU8>,
}

impl<FAuth, FPub> CommonCommitWorker for ConfigCommitBackend<FAuth, FPub>
where
    FAuth: Fn() -> Result<(), (&'static str, ArmError)> + Send + 'static,
    FPub: FnOnce(&mut PublishedConfig, ConfigActivated) + Send + 'static,
{
    type Ok = ConfigCommitOutcome;
    type Err = ConfigCommitError;
    const RUN_BLOCKING_ON_TOKIO: bool = true;

    fn before_reserve(&mut self) {
        if let Some(h) = self.hooks.before_reserve.as_ref() {
            h();
        }
    }

    fn should_fail_rename_once(&self) -> bool {
        self.hooks
            .fail_rename_once
            .as_ref()
            .is_some_and(|f| f.swap(false, Ordering::AcqRel))
    }

    fn fail_rename_at_index(&self) -> Option<usize> {
        self.hooks.fail_rename_at_index.as_ref().and_then(|f| {
            let idx = f.swap(usize::MAX, Ordering::AcqRel);
            (idx != usize::MAX).then_some(idx)
        })
    }

    fn should_fail_dir_sync_once(&self) -> bool {
        self.hooks
            .fail_dir_sync_once
            .as_ref()
            .is_some_and(|f| f.swap(false, Ordering::AcqRel))
    }

    fn dir_sync_override(&self) -> Option<DirSyncOverride> {
        self.hooks.dir_sync_override.clone()
    }

    async fn reserve(&mut self) -> Result<(), Self::Err> {
        if let Some(reservation) = self.pre_held.take() {
            self.grant = Some(reservation.grant());
            self.domain = reservation.domain();
            let mut g = ReservationGuard::with_phase(
                Arc::clone(&self.sup),
                reservation,
                Arc::clone(&self.pub_phase),
            );
            if self.hw_mutated {
                g.mark_hw_mutated();
            }
            self.guard = Some(g);
            if let Err((_, err)) = (self.auth_check.as_ref().expect("auth_check present"))() {
                self.release_pre_rename().await;
                return Err(if self.hw_mutated {
                    ConfigCommitError::post_publish_reconciling(err)
                } else {
                    ConfigCommitError::pre_publish(err)
                });
            }
            return Ok(());
        }
        if let Err((_, err)) = (self.auth_check.as_ref().expect("auth_check present"))() {
            return Err(ConfigCommitError::pre_publish(err));
        }
        let precheck = self.precheck.take().expect("precheck taken once in reserve");
        let reservation = match self
            .sup
            .dispatch_config_request_owned_reliable(ConfigRequest::Reserve(ReserveConfig {
                proposal: self.proposal,
                auth: self.auth,
                precheck,
                expected_workspace_revision: self.expected_workspace_revision,
                expected_poses_revision: self.expected_poses_revision,
            }))
            .await
        {
            Ok(ConfigReply::Reserved(Ok(r))) => r,
            Ok(ConfigReply::Reserved(Err(e))) => {
                return Err(ConfigCommitError::pre_publish(e.to_arm_error()));
            }
            Err(e) => {
                return Err(ConfigCommitError::pre_publish(e.to_arm_error()));
            }
            _ => {
                return Err(ConfigCommitError::pre_publish(ArmError::Internal(
                    "unexpected config reserve reply".into(),
                )));
            }
        };
        self.grant = Some(reservation.grant());
        self.domain = reservation.domain();
        self.guard = Some(ReservationGuard::with_phase(
            Arc::clone(&self.sup),
            reservation,
            Arc::clone(&self.pub_phase),
        ));
        Ok(())
    }

    async fn prepare_write_after_reserve(&mut self) -> Result<Option<PendingWrite>, Self::Err> {
        unreachable!("ConfigCommitBackend uses prepare_write_op_after_reserve")
    }

    async fn prepare_write_op_after_reserve(&mut self) -> Result<ConfigWriteOp, Self::Err> {
        // For `Sequences` saves referencing replay episodes, atomically verify existence and pin
        // them in `EpisodeStore` right under the granted `Sequences` reservation so concurrent
        // episode deletion is refused with 409 IN_USE and an already-deleted episode aborts
        // cleanly with 422 SEQUENCE_REF_MISSING.
        if !self.replay_episode_refs.is_empty() {
            let store = self.sup.episode_store().await;
            match store.pin_episodes_for_sequence(&self.replay_episode_refs) {
                Ok(pg) => self.episode_pin_guard = Some(pg),
                Err((step, ep_id)) => {
                    return Err(ConfigCommitError::pre_publish(ArmError::detailed(
                        422,
                        "SEQUENCE_REF_MISSING",
                        format!("step {step}: no episode '{ep_id}'"),
                        json!({"step": step}),
                    )));
                }
            }
        }
        Ok(self.write_op.take().unwrap_or(ConfigWriteOp::Volatile))
    }

    fn after_reserve_before_rename(&mut self) -> bool {
        if let Some(h) = self.hooks.after_reserve_before_rename.as_ref() {
            h();
        }
        false
    }

    async fn verify_authority_before_rename(&mut self) -> Result<(), Self::Err> {
        if let Err((_, err)) = (self.auth_check.as_ref().expect("auth_check present"))() {
            return Err(ConfigCommitError::pre_publish(err));
        }
        let grant = self.grant.expect("reservation granted before verify");
        match self
            .sup
            .dispatch_config_request_owned_reliable(ConfigRequest::VerifyAuthority {
                domain: self.domain,
                grant,
                proposal: self.proposal,
                auth: self.auth,
                expected_workspace_revision: self.expected_workspace_revision,
                expected_poses_revision: self.expected_poses_revision,
                domain_checks: self.domain_checks,
            })
            .await
        {
            Ok(ConfigReply::AuthorityVerified(Ok(()))) => Ok(()),
            Ok(ConfigReply::AuthorityVerified(Err(e))) => {
                Err(ConfigCommitError::pre_publish(e.to_arm_error()))
            }
            Err(e) => Err(ConfigCommitError::pre_publish(e.to_arm_error())),
            _ => Err(ConfigCommitError::pre_publish(ArmError::Internal(
                "unexpected authority verification reply".into(),
            ))),
        }
    }

    fn on_rename_starting(&mut self) {
        self.pub_phase.store(PUB_PHASE_RENAMING, Ordering::Release);
    }

    fn on_renamed(&mut self) {
        self.pub_phase.store(PUB_PHASE_RENAMED, Ordering::Release);
        if let Some(g) = self.guard.as_mut() {
            g.mark_renamed();
        }
    }

    async fn mark_persisted(&mut self) -> Result<(), Self::Err> {
        let sim_persisted_fail = self
            .hooks
            .fail_persisted_once
            .as_ref()
            .is_some_and(|f| f.swap(false, Ordering::AcqRel));
        let r = self
            .guard
            .as_mut()
            .and_then(|g| g.take())
            .expect("reservation held by guard");
        if sim_persisted_fail {
            if let Some(pg) = self.episode_pin_guard.take() {
                pg.keep_pinned_for_reconcile();
            }
            if let Ok(ConfigReply::Persisted(Ok(pr))) = self
                .sup
                .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(PersistedConfig {
                    reservation: r,
                    durability: Durability::DurableUnconfirmed,
                }))
                .await
            {
                let _ = self
                    .sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                        reservation: pr,
                    }))
                    .await;
            }
            return Err(ConfigCommitError::post_publish_reconciling(
                ConfigError::Reconciling("SIMULATED_PERSISTED_ERR").to_arm_error(),
            ));
        }
        let persisted_r = match self
            .sup
            .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(PersistedConfig {
                reservation: r,
                durability: Durability::DurableUnconfirmed,
            }))
            .await
        {
            Ok(ConfigReply::Persisted(Ok(r))) => r,
            Ok(ConfigReply::Persisted(Err(e))) => {
                if let Some(pg) = self.episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
                let domain = self.domain;
                let _ = self
                    .sup
                    .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                    .await;
                return Err(ConfigCommitError::post_publish_reconciling(e.to_arm_error()));
            }
            Err(e) => {
                if let Some(pg) = self.episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
                let domain = self.domain;
                let _ = self
                    .sup
                    .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                    .await;
                return Err(ConfigCommitError::post_publish_reconciling(e.to_arm_error()));
            }
            _ => {
                if let Some(pg) = self.episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
                let domain = self.domain;
                let _ = self
                    .sup
                    .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                    .await;
                return Err(ConfigCommitError::post_publish_reconciling(
                    ArmError::Internal("unexpected config persisted reply".into()),
                ));
            }
        };
        let mut g = ReservationGuard::with_phase(
            Arc::clone(&self.sup),
            persisted_r,
            Arc::clone(&self.pub_phase),
        );
        g.mark_renamed();
        self.guard = Some(g);
        Ok(())
    }

    async fn reconcile_after_partial_rename(
        &mut self,
        failed_path: PathBuf,
        err: String,
    ) -> Self::Err {
        if let Some(pg) = self.episode_pin_guard.take() {
            pg.keep_pinned_for_reconcile();
        }
        if let Some(g) = self.guard.as_mut() {
            g.mark_renamed();
            if let Some(r) = g.take() {
                if let Ok(ConfigReply::Persisted(Ok(pr))) = self
                    .sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(
                        PersistedConfig {
                            reservation: r,
                            durability: Durability::DurableUnconfirmed,
                        },
                    ))
                    .await
                {
                    let _ = self
                        .sup
                        .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                            ReleaseConfig { reservation: pr },
                        ))
                        .await;
                } else {
                    let domain = self.domain;
                    let _ = self
                        .sup
                        .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                        .await;
                }
            }
        }
        let file_name = target_file_label(&failed_path);
        let msg = format!(
            "Configuration persistence failed mid-batch at {file_name} ({err}); disk state may be partial, reconciliation required"
        );
        let bounded = BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::new(&msg);
        let domain = self.domain;
        let _ = self
            .sup
            .dispatch_config_request_reliable(|| ConfigRequest::LatchConfigReconcileFault {
                domain,
                message: bounded,
            })
            .await;
        ConfigCommitError::post_publish_reconciling(ArmError::Rejected {
            status: 500,
            code: "PERSIST_FAILED".into(),
            message: msg,
        })
    }

    async fn reconcile_after_dir_sync_failure(
        &mut self,
        target_path: PathBuf,
        warn: String,
    ) -> Self::Err {
        self.reconcile_after_dir_sync_failure_paths(vec![target_path], warn)
            .await
    }

    async fn reconcile_after_dir_sync_failure_paths(
        &mut self,
        target_paths: Vec<PathBuf>,
        warn: String,
    ) -> Self::Err {
        let mut expected_from = self.proposal.expected_revision;
        if let Some(r) = self.guard.as_mut().and_then(|g| g.take()) {
            expected_from = r.from();
            let rel = self
                .sup
                .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                    reservation: r,
                }))
                .await;
            if !matches!(rel, Ok(ConfigReply::Released(Released::Reconciling))) {
                let domain = self.domain;
                let _ = self
                    .sup
                    .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                    .await;
            }
        }
        if self.hw_mutated {
            // B3: Hardware has already mutated; do NOT spawn config-only dir-sync retry.
            // Keep HoldState::Reconciling and .calibration_commit.pending until reconciled offline.
            if let Some(pg) = self.episode_pin_guard.take() {
                pg.keep_pinned_for_reconcile();
            }
            let file_name = target_paths
                .first()
                .map(|p| target_file_label(p))
                .unwrap_or_else(|| "calibration.json".to_string());
            let msg = format!(
                "Hardware commissioning persisted {file_name} but parent dir fsync failed ({warn}); hardware and disk reconciliation required"
            );
            let bounded = BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::new(&msg);
            let domain = self.domain;
            let _ = self
                .sup
                .dispatch_config_request_reliable(|| ConfigRequest::LatchConfigReconcileFault {
                    domain,
                    message: bounded,
                })
                .await;
            return ConfigCommitError::post_publish_reconciling(ArmError::Rejected {
                status: 500,
                code: "PERSIST_FAILED".into(),
                message: msg,
            });
        }
        if let (Some(candidate), Some(auth_check), Some(update_published)) = (
            self.candidate.take(),
            self.auth_check.take(),
            self.update_published.take(),
        ) {
            let interval = self
                .hooks
                .reconcile_retry_interval
                .unwrap_or(DEFAULT_RECONCILE_RETRY_INTERVAL);
            let max_attempts = self
                .hooks
                .reconcile_max_attempts
                .unwrap_or(DEFAULT_RECONCILE_MAX_ATTEMPTS);
            spawn_config_reconcile_retry(
                Arc::clone(&self.sup),
                self.domain,
                target_paths,
                self.pending_marker_path.take(),
                expected_from,
                self.proposal,
                self.auth,
                candidate,
                self.expected_workspace_revision,
                self.expected_poses_revision,
                auth_check,
                update_published,
                self.episode_pin_guard.take(),
                self.hooks.dir_sync_override.clone(),
                interval,
                max_attempts,
                warn.clone(),
            );
        } else if let Some(pg) = self.episode_pin_guard.take() {
            pg.keep_pinned_for_reconcile();
        }
        ConfigCommitError::post_publish_reconciling(ArmError::Rejected {
            status: 500,
            code: "PERSIST_FAILED".into(),
            message: format!("Parent directory fsync failed after rename ({warn}); reconciliation required"),
        })
    }

    fn after_rename_before_activate(&mut self) -> bool {
        if let Some(h) = self.hooks.after_rename_before_activate.as_ref() {
            h();
        }
        false
    }

    async fn activate_and_publish(&mut self, disk_published: bool) -> Result<Self::Ok, Self::Err> {
        let post_rename_revoked = match (self.auth_check.as_ref().expect("auth_check present"))() {
            Ok(()) => None,
            Err((code, err)) => {
                if !disk_published {
                    self.release_pre_rename().await;
                    return Err(ConfigCommitError::pre_publish(err));
                }
                Some(code)
            }
        };
        let reservation = self
            .guard
            .as_mut()
            .and_then(|g| g.take())
            .expect("reservation held by guard");
        let candidate = self.candidate.take().expect("candidate taken once in activate");
        let durability = if disk_published {
            Durability::Durable
        } else {
            Durability::Volatile
        };
        let activated = match self
            .sup
            .dispatch_config_request_owned_reliable(ConfigRequest::Activate(ActivateConfig {
                reservation,
                auth: self.auth,
                revoked_off_thread: post_rename_revoked,
                candidate,
                durability,
                expected_workspace_revision: self.expected_workspace_revision,
                expected_poses_revision: self.expected_poses_revision,
            }))
            .await
        {
            Ok(ConfigReply::Activated(Ok(act))) => act,
            Ok(ConfigReply::Activated(Err(e))) => {
                let is_reconciling = matches!(e, ConfigError::Reconciling(_)) || disk_published;
                if is_reconciling {
                    if let Some(pg) = self.episode_pin_guard.take() {
                        pg.keep_pinned_for_reconcile();
                    }
                }
                if disk_published {
                    if !matches!(e, ConfigError::Reconciling(_)) {
                        let domain = self.domain;
                        let _ = self
                            .sup
                            .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                            .await;
                    }
                    tracing::error!(
                        "Configuration written on disk but not activated: {:?}; reconciling",
                        e
                    );
                }
                return Err(ConfigCommitError {
                    error: e.to_arm_error(),
                    disk_published,
                    reconciling: is_reconciling,
                    outcome_unknown: false,
                });
            }
            Err(e) => {
                if disk_published {
                    if let Some(pg) = self.episode_pin_guard.take() {
                        pg.keep_pinned_for_reconcile();
                    }
                    // Transport failed after disk publication: activation outcome and
                    // PublishedConfigHub sync are unconfirmed, so unconditionally force
                    // Reconciling to keep motion inhibited until reconciled.
                    let domain = self.domain;
                    let _ = self
                        .sup
                        .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                        .await;
                    return Err(ConfigCommitError::post_publish_unknown(e.to_arm_error()));
                }
                return Err(ConfigCommitError::pre_publish(e.to_arm_error()));
            }
            _ => {
                if disk_published {
                    if let Some(pg) = self.episode_pin_guard.take() {
                        pg.keep_pinned_for_reconcile();
                    }
                    let domain = self.domain;
                    let _ = self
                        .sup
                        .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                        .await;
                    return Err(ConfigCommitError::post_publish_unknown(ArmError::Internal(
                        "unexpected config activate reply".into(),
                    )));
                }
                return Err(ConfigCommitError::pre_publish(ArmError::Internal(
                    "unexpected config activate reply".into(),
                )));
            }
        };

        // Publish committed sequence revision BEFORE dropping `episode_pin_guard` so concurrent
        // episode deletes linearize cleanly with the newly active `SequenceFile`.
        if self.domain == ConfigDomain::Sequences {
            self.sup.set_sequences_revision(activated.revision);
        }

        let update_published = self
            .update_published
            .take()
            .expect("update_published called once");
        self.sup.published_config_hub().update(|cfg| {
            cfg.workspace_revision = activated.workspace_revision;
            for i in 0..activated.limit_len.min(cfg.config.limits.len()) {
                cfg.config.limits[i].min_position = activated.limit_min[i];
                cfg.config.limits[i].max_position = activated.limit_max[i];
                cfg.config.limits[i].max_velocity = activated.limit_vel[i];
            }
            update_published(cfg, activated);
        });
        sync_prep_env_from_published(&self.sup, Some(activated.motion_config_revision));
        drop(self.episode_pin_guard.take());

        Ok(ConfigCommitOutcome {
            activated,
            persisted: disk_published,
            durability_confirmed: true,
            durability_warning: None,
        })
    }

    async fn release_pre_rename(&mut self) {
        if self.hw_mutated {
            if let Some(pg) = self.episode_pin_guard.take() {
                pg.keep_pinned_for_reconcile();
            }
            if let Some(g) = self.guard.as_mut() {
                g.mark_renamed();
                if let Some(r) = g.take() {
                    if let Ok(ConfigReply::Persisted(Ok(pr))) = self
                        .sup
                        .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(
                            PersistedConfig {
                                reservation: r,
                                durability: Durability::DurableUnconfirmed,
                            },
                        ))
                        .await
                    {
                        let _ = self
                            .sup
                            .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                                ReleaseConfig { reservation: pr },
                            ))
                            .await;
                    } else {
                        let domain = self.domain;
                        let _ = self
                            .sup
                            .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                            .await;
                    }
                }
            } else {
                let domain = self.domain;
                let _ = self
                    .sup
                    .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                    .await;
            }
            return;
        }
        drop(self.episode_pin_guard.take());
        if let Some(r) = self.guard.as_mut().and_then(|g| g.take()) {
            let _ = self
                .sup
                .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                    reservation: r,
                }))
                .await;
        }
    }

    async fn abort_pre_rename_fs(&mut self, _phase: &'static str, err: String) -> Self::Err {
        self.release_pre_rename().await;
        let arm_err = ArmError::Rejected {
            status: 500,
            code: "PERSIST_FAILED".into(),
            message: format!("Failed to persist configuration: {err}"),
        };
        if self.hw_mutated {
            let domain = self.domain;
            let msg = format!(
                "Hardware commissioning persistence failed ({err}); hardware and disk reconciliation required"
            );
            let bounded = BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::new(&msg);
            let _ = self
                .sup
                .dispatch_config_request_reliable(|| ConfigRequest::LatchConfigReconcileFault {
                    domain,
                    message: bounded,
                })
                .await;
            ConfigCommitError::post_publish_reconciling(arm_err)
        } else {
            ConfigCommitError::pre_publish(arm_err)
        }
    }

    async fn reconcile_rename_join_error(&mut self, err: String) -> Self::Err {
        if let Some(pg) = self.episode_pin_guard.take() {
            pg.keep_pinned_for_reconcile();
        }
        if let Some(g) = self.guard.as_mut() {
            g.mark_renamed();
            if let Some(r) = g.take() {
                if let Ok(ConfigReply::Persisted(Ok(pr))) = self
                    .sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(
                        PersistedConfig {
                            reservation: r,
                            durability: Durability::DurableUnconfirmed,
                        },
                    ))
                    .await
                {
                    let _ = self
                        .sup
                        .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                            ReleaseConfig { reservation: pr },
                        ))
                        .await;
                } else {
                    let domain = self.domain;
                    let _ = self
                        .sup
                        .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                        .await;
                }
            }
        }
        ConfigCommitError::post_publish_reconciling(ArmError::Rejected {
            status: 500,
            code: "PERSIST_FAILED".into(),
            message: format!("Failed to persist configuration: rename task failed: {err}"),
        })
    }

    fn note_marker_cleanup_warning(&mut self, ok: &mut Self::Ok, warn: String) {
        ok.durability_confirmed = false;
        ok.durability_warning = Some(warn);
    }

    async fn reconcile_after_marker_error(&mut self, err: String) -> Self::Err {
        self.reconcile_rename_join_error(format!("pending commit marker error: {err}"))
            .await
    }

    fn preserve_marker_on_abort(&self) -> bool {
        self.hw_mutated
    }

    fn worker_disconnected_err(&mut self) -> Self::Err {
        if self.hw_mutated || self.pub_phase.load(Ordering::Acquire) != PUB_PHASE_PRE_RENAME {
            ConfigCommitError::post_publish_unknown(ArmError::Internal(
                "config commit worker disconnected after rename".into(),
            ))
        } else {
            ConfigCommitError::pre_publish(ArmError::Internal(
                "config commit worker disconnected".into(),
            ))
        }
    }
}

async fn run_config_commit_worker<FAuth, FPub>(
    sup: Arc<ArmSupervisor>,
    plan: ConfigCommitPlan,
    pre_held: Option<Reservation>,
    hw_mutated: bool,
    auth_check: FAuth,
    update_published: FPub,
    pub_phase: Arc<AtomicU8>,
) -> Result<ConfigCommitOutcome, ConfigCommitError>
where
    FAuth: Fn() -> Result<(), (&'static str, ArmError)> + Send + 'static,
    FPub: FnOnce(&mut PublishedConfig, ConfigActivated) + Send + 'static,
{
    let hooks = sup.config_commit_test_hooks();
    let cw_lock = sup.config_write_lock();
    let _cw = cw_lock.lock().await;

    let ConfigCommitPlan {
        proposal,
        auth,
        mut precheck,
        candidate,
        write,
        expected_workspace_revision,
        expected_poses_revision,
    } = plan;

    let domain_checks = match &precheck {
        ReservePrecheck::Calibration {
            expected_txn_id,
            operator_grant,
            require_kinematics_disarmed,
            expected_grasp_setup_revision,
            ..
        } => VerifyDomainChecks {
            expected_grasp_setup_revision: *expected_grasp_setup_revision,
            expected_gripper_calib_revision: None,
            require_workspace_unblocked: true,
            require_kinematics_disarmed: *require_kinematics_disarmed,
            require_software_zero_idle: false,
            require_mit_torque_off: false,
            expected_txn_id: *expected_txn_id,
            operator_grant: Some(*operator_grant),
        },
        ReservePrecheck::SoftwareJointZero {
            expected_txn_id,
            operator_grant,
            expected_gripper_calib_revision,
            ..
        } => VerifyDomainChecks {
            expected_grasp_setup_revision: None,
            expected_gripper_calib_revision: *expected_gripper_calib_revision,
            require_workspace_unblocked: false,
            require_kinematics_disarmed: false,
            require_software_zero_idle: true,
            require_mit_torque_off: false,
            expected_txn_id: *expected_txn_id,
            operator_grant: Some(*operator_grant),
        },
        ReservePrecheck::MitCommissioning {
            expected_txn_id,
            operator_grant,
            ..
        } => VerifyDomainChecks {
            expected_grasp_setup_revision: None,
            expected_gripper_calib_revision: None,
            require_workspace_unblocked: false,
            require_kinematics_disarmed: false,
            require_software_zero_idle: false,
            require_mit_torque_off: true,
            expected_txn_id: *expected_txn_id,
            operator_grant: Some(*operator_grant),
        },
        _ => VerifyDomainChecks::default(),
    };

    // For `Sequences` saves, collect replay-step `(step_idx, episode_id)` references and ensure
    // `reserved_pins` covers BOTH pose and replay episode IDs across the reservation lifetime.
    let mut replay_episode_refs: Vec<(usize, String)> = Vec::new();
    if let ReservePrecheck::Sequences {
        steps: Some(ref steps),
        ref mut reserved_pins,
        ..
    } = precheck
    {
        for (i, step) in steps.iter().enumerate() {
            if let Step::Replay { episode_id, .. } = step {
                replay_episode_refs.push((i, episode_id.clone()));
            }
        }
        if reserved_pins.is_none() {
            let pose_ids = steps
                .iter()
                .filter_map(|s| match s {
                    Step::Pose { pose_id, .. } => Some(pose_id.clone()),
                    _ => None,
                })
                .collect();
            let episode_ids = replay_episode_refs.iter().map(|(_, id)| id.clone()).collect();
            *reserved_pins = Some(crate::sequences::ReservedSequencePins {
                sequence_id: String::new(),
                pose_ids,
                episode_ids,
            });
        }
    }

    let pending_marker_path = match &write {
        ConfigWriteOp::Batch {
            pending_marker,
            existing_marker,
            ..
        } => pending_marker
            .as_ref()
            .map(|(p, _)| p.clone())
            .or_else(|| existing_marker.clone()),
        _ => None,
    };
    let domain = proposal.domain;
    let mut backend = ConfigCommitBackend {
        sup,
        hooks,
        proposal,
        auth,
        precheck: Some(precheck),
        domain_checks,
        candidate: Some(candidate),
        write_op: Some(write),
        pending_marker_path,
        expected_workspace_revision,
        expected_poses_revision,
        replay_episode_refs,
        auth_check: Some(auth_check),
        update_published: Some(update_published),
        guard: None,
        pre_held,
        hw_mutated,
        grant: None,
        domain,
        episode_pin_guard: None,
        pub_phase,
    };
    execute_common_commit(&mut backend).await
}

fn sync_prep_env_from_published(sup: &ArmSupervisor, motion_config_rev: Option<u64>) {
    let Some(pub_cfg) = sup.published_config_hub().latest() else {
        return;
    };
    if let Some(cur) = sup.hub().prep_env() {
        let mut next = (*cur).clone();
        next.config = pub_cfg.config.clone();
        next.workspace_policy = pub_cfg.workspace_policy().map(|(ws, _)| ws.clone());
        next.poses = pub_cfg.poses.list.clone();
        next.sequences = pub_cfg.sequences.clone();
        next.grasp_setup = pub_cfg.grasp_setup();
        next.calibration = pub_cfg.calibration.clone();
        next.camera_calibrated = pub_cfg.camera_calibrated();
        next.joint_zero_offsets = pub_cfg.joint_zero_offsets.clone();
        next.workspace_revision = pub_cfg.workspace_revision;
        next.calibration_revision = pub_cfg.calibration_revision;
        next.poses_revision = pub_cfg.poses.revision;
        next.sequences_revision = pub_cfg.sequences.revision;
        next.grasp_setup_revision = pub_cfg.grasp_setup_revision;
        next.gripper_calib_revision = pub_cfg.gripper_calib_revision;
        sup.hub().update_prep_env(Arc::new(next));
    }
    sup.hub().update_service_identity_from_config(&pub_cfg);
    let apply_mc = |cur_mc: &crate::rt_core::requests::motion::MotionConfigSnapshot| {
        let mut next_mc = cur_mc.clone();
        next_mc.config = pub_cfg.config.clone();
        next_mc.workspace = pub_cfg.workspace_policy().map(|(ws, _)| ws.clone());
        // Mirror `SupervisorInner::power_budget()`: disabled budgets filter to `None` (hash = 0).
        next_mc.power_budget = pub_cfg.config.power_budget.as_ref().filter(|p| p.enabled).cloned();
        next_mc.power_budget_hash =
            crate::rt_core::requests::motion::power_budget_fingerprint(next_mc.power_budget.as_ref());
        next_mc.workspace_revision = pub_cfg.workspace_revision;
        if let Some(rev) = motion_config_rev {
            next_mc.motion_config_revision = rev;
        }
        Arc::new(next_mc)
    };
    if let Some(cur_mc) = sup.hub().motion_config() {
        sup.hub().set_motion_config(apply_mc(&cur_mc));
    }
    if let Some(cur_legacy_mc) = sup.hub().legacy_motion_config() {
        sup.hub().set_legacy_motion_config_if_newer(apply_mc(&cur_legacy_mc));
    }
}

fn target_file_label(path: &std::path::Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string())
}

async fn is_owner_stopped(sup: &ArmSupervisor) -> bool {
    match sup.owner_mode() {
        OWNER_EXECUTOR => sup.executor_handle().is_none_or(|h| h.is_exited()),
        OWNER_LEGACY => sup.inner().read().await.detached,
        _ => true,
    }
}

/// Bounded off-RT post-rename reconciliation retry loop for every PR C config family.
///
/// Re-runs parent-directory fsync and live authority rechecks on a bounded backoff strictly off
/// `armd-control`:
/// - On success: swaps the candidate in-place, calls `ConfigAuthority::finish_reconcile`, updates
///   `PublishedConfigHub`, and lifts the motion inhibit.
/// - On exhaustion or non-recoverable post-rename authority/invariant divergence: latches a clear
///   non-recoverable config fault naming the file so the user sees why motion stays inhibited.
#[allow(clippy::too_many_arguments)]
fn spawn_config_reconcile_retry<FAuth, FPub>(
    sup: Arc<ArmSupervisor>,
    domain: ConfigDomain,
    target_paths: Vec<PathBuf>,
    mut pending_marker_path: Option<PathBuf>,
    expected_from: u64,
    proposal: ConfigProposal,
    auth: ConfigAuthContext,
    candidate: ConfigCandidate,
    expected_workspace_revision: Option<u64>,
    expected_poses_revision: Option<u64>,
    auth_check: FAuth,
    update_published: FPub,
    mut episode_pin_guard: Option<crate::recording::store::EpisodePinGuard>,
    dir_sync_override: Option<DirSyncOverride>,
    interval: std::time::Duration,
    max_attempts: u32,
    initial_warn: String,
) where
    FAuth: Fn() -> Result<(), (&'static str, ArmError)> + Send + 'static,
    FPub: FnOnce(&mut PublishedConfig, ConfigActivated) + Send + 'static,
{
    tokio::spawn(async move {
        let file_name = target_paths
            .first()
            .map(|p| target_file_label(p))
            .unwrap_or_else(|| "state".to_string());
        let mut last_warn = initial_warn;
        let mut candidate_opt = Some(candidate);
        let mut update_pub_opt = Some(update_published);
        let total_attempts = max_attempts.max(1);

        let latch_fault = |sup: Arc<ArmSupervisor>,
                           domain: ConfigDomain,
                           msg: String,
                           ep_guard: Option<crate::recording::store::EpisodePinGuard>| async move {
            tracing::error!("{msg}");
            if let Some(pg) = ep_guard {
                pg.keep_pinned_for_reconcile();
            }
            let bounded = BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::new(&msg);
            let _ = sup
                .dispatch_config_request_reliable(|| ConfigRequest::LatchConfigReconcileFault {
                    domain,
                    message: bounded,
                })
                .await;
        };

        for attempt in 1..=total_attempts {
            tokio::time::sleep(interval).await;
            if is_owner_stopped(&sup).await {
                if let Some(pg) = episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
                return;
            }
            match sup
                .dispatch_config_request_reliable(|| ConfigRequest::InspectHold(domain))
                .await
            {
                Ok(ConfigReply::HoldInspected {
                    state: Some(crate::rt_core::config_iface::HoldState::Reconciling),
                    ..
                }) => {}
                Ok(_) => {
                    // Already reconciled externally.
                    drop(episode_pin_guard.take());
                    return;
                }
                Err(_) => {
                    if let Some(pg) = episode_pin_guard.take() {
                        pg.keep_pinned_for_reconcile();
                    }
                    return;
                }
            }

            let unique_syncs = unique_dir_syncs(
                target_paths
                    .iter()
                    .map(|p| PendingDirSync::for_path(p))
                    .collect(),
            );
            let ov = dir_sync_override.clone();
            let sync_res = tokio::task::spawn_blocking(move || {
                for pds in unique_syncs {
                    let outcome = match ov.as_ref() {
                        Some(f) => pds.sync_with(|dir| f(dir)),
                        None => pds.sync_parent_dir(),
                    };
                    if let PublishOutcome::PublishedDurabilityUnconfirmed(w) = outcome {
                        return Err(w);
                    }
                }
                Ok(())
            })
            .await
            .map_err(|e| e.to_string())
            .and_then(|r| r);

            if let Err(w) = sync_res {
                last_warn = w;
                tracing::warn!(
                    "Config reconciliation retry {attempt}/{total_attempts} for {file_name} failed parent dir fsync: {last_warn}"
                );
                continue;
            }

            // Directory fsync succeeded; serialize with config writes and recheck authority + finish_reconcile.
            let cw_lock = sup.config_write_lock();
            let _cw = cw_lock.lock().await;
            if let Err((code, _)) = (auth_check)() {
                let msg = format!(
                    "Configuration persistence failed for {file_name}: post-rename authority revoked ({code})"
                );
                latch_fault(sup, domain, msg, episode_pin_guard.take()).await;
                return;
            }
            let Some(cand) = candidate_opt.take() else {
                return;
            };
            let cleanup_marker_txn_id = pending_marker_path
                .is_some()
                .then(next_calibration_txn_id);
            match sup
                .dispatch_config_request_owned_reliable(ConfigRequest::FinishReconcile(
                    FinishReconcileConfig {
                        domain,
                        expected_from,
                        proposal,
                        auth,
                        candidate: cand,
                        expected_workspace_revision,
                        expected_poses_revision,
                        cleanup_marker_txn_id,
                    },
                ))
                .await
            {
                Ok(ConfigReply::Activated(Ok(activated))) => {
                    let cleanup_guard = cleanup_marker_txn_id.map(|tid| {
                        sup.published_config_hub().update(|cfg| {
                            cfg.calibration_busy = true;
                            cfg.calibration_txn_id = Some(tid);
                        });
                        crate::calibration::ExecutorTxnGuard::new(Arc::clone(&sup), tid)
                    });
                    if domain == ConfigDomain::Sequences {
                        sup.set_sequences_revision(activated.revision);
                    }
                    if let Some(update_pub) = update_pub_opt.take() {
                        sup.published_config_hub().update(|cfg| {
                            cfg.workspace_revision = activated.workspace_revision;
                            for i in 0..activated.limit_len.min(cfg.config.limits.len()) {
                                cfg.config.limits[i].min_position = activated.limit_min[i];
                                cfg.config.limits[i].max_position = activated.limit_max[i];
                                cfg.config.limits[i].max_velocity = activated.limit_vel[i];
                            }
                            update_pub(cfg, activated);
                        });
                        sync_prep_env_from_published(&sup, Some(activated.motion_config_revision));
                    }
                    if let Some(mp) = pending_marker_path.take() {
                        let rm_res = tokio::task::spawn_blocking(move || {
                            remove_pending_commit_marker(&mp)
                        })
                        .await
                        .map_err(|e| e.to_string())
                        .and_then(|r| r);
                        if let Err(e) = rm_res {
                            tracing::error!(
                                "Config reconciliation activated {file_name} but failed to remove pending commit marker: {e}"
                            );
                        }
                    }
                    if let Some(g) = cleanup_guard {
                        g.release().await;
                    }
                    drop(episode_pin_guard.take());
                    tracing::info!(
                        "Config reconciliation succeeded for {file_name} on retry {attempt}/{total_attempts}; motion inhibit lifted"
                    );
                    return;
                }
                Ok(ConfigReply::Activated(Err(ConfigError::NotHeld))) => {
                    drop(episode_pin_guard.take());
                    return;
                }
                Ok(ConfigReply::Activated(Err(err))) => {
                    let msg = format!(
                        "Configuration persistence failed for {file_name}: post-rename reconciliation rejected ({})",
                        err.to_arm_error()
                    );
                    latch_fault(sup, domain, msg, episode_pin_guard.take()).await;
                    return;
                }
                Err(_) | Ok(_) => {
                    if let Some(pg) = episode_pin_guard.take() {
                        pg.keep_pinned_for_reconcile();
                    }
                    return;
                }
            }
        }

        let msg = format!(
            "Configuration persistence failed for {file_name} after {total_attempts} reconciliation attempts: {last_warn}"
        );
        latch_fault(sup, domain, msg, episode_pin_guard.take()).await;
    });
}

// ============================================================================
// Unified gripper_zero_check(store_offset=true) commit path (Executor + Legacy + Sync)
// ============================================================================

/// Zero-allocation live owner authority check shared across Reserve, pre-rename VerifyAuthority,
/// and post-dir-sync Activate for `gripper_zero_check(store_offset=true)`.
///
/// Calls the single authority entry point [`check_config_authority`] (boot, connection, unapplied
/// estop/stop, stop_generation, epoch, lease_holder, deadline) plus gripper-calibration invariants.
#[allow(clippy::too_many_arguments)]
pub(crate) fn check_zero_check_authority(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    proposal: &ConfigProposal,
    auth: &ConfigAuthContext,
    action_id: u64,
    base_revision: u64,
    candidate_policy: Option<&GripperPolicy>,
) -> Result<(), (&'static str, &'static str, &'static str)> {
    if core.zero_check_persisting != Some(action_id)
        || !core
            .last_action
            .as_ref()
            .is_some_and(|a| a.id == action_id && a.is_running())
    {
        return Err((
            "cancelled",
            "CANCELLED",
            "Gripper zero check cancelled before calibration persisted",
        ));
    }
    if core.safety.state().is_stopped() {
        return Err(match core.safety.state() {
            crate::safety::SafetyState::EStop => (
                "failed",
                "ESTOP_ACTIVE",
                "Emergency stop intervened before gripper calibration persisted",
            ),
            crate::safety::SafetyState::Fault(crate::safety::FaultReason::ThermalTrip { .. }) => (
                "failed",
                "THERMAL_TRIP",
                "Thermal trip intervened before gripper calibration persisted",
            ),
            crate::safety::SafetyState::Fault(crate::safety::FaultReason::BusLoss) => (
                "failed",
                "BUS_LOSS",
                "Bus loss intervened before gripper calibration persisted",
            ),
            _ => (
                "cancelled",
                "STOPPED",
                "Safety stop intervened before gripper calibration persisted",
            ),
        });
    }
    if let Some((gid, grant)) = core.zero_check_persisting_grant {
        if gid == action_id && !grant.is_valid_at(ctx.now, core.operator_auth_enforced) {
            return Err((
                "failed",
                "OPERATOR_REQUIRED",
                "Operator session expired or locked before gripper calibration persisted",
            ));
        }
    }
    if let Err(err) = check_config_authority(core, ctx, proposal, auth, false) {
        return Err(match err {
            ConfigError::LeaseLost | ConfigError::ConnectionClosed => (
                "cancelled",
                "LEASE_LOST",
                "Control lease lost or expired before gripper calibration persisted",
            ),
            ConfigError::EpochMismatch { .. } => (
                "cancelled",
                "EPOCH_MISMATCH",
                "Safety epoch changed before gripper calibration persisted",
            ),
            ConfigError::Stopped => (
                "cancelled",
                "STOPPED",
                "Authorized stop intervened before gripper calibration persisted",
            ),
            ConfigError::DeadlineExpired => (
                "failed",
                "DEADLINE_EXPIRED",
                "Gripper calibration persistence deadline expired",
            ),
            _ => (
                "failed",
                "WRONG_TARGET",
                "Gripper calibration authority check failed",
            ),
        });
    }
    if core.gripper_calib_revision != base_revision {
        return Err((
            "failed",
            "POLICY_REVISION_MISMATCH",
            "Gripper calibration revision changed concurrently",
        ));
    }
    if core.driver_config_error.is_some() {
        return Err((
            "failed",
            "INVALID_CALIBRATION",
            "Driver limit configuration error prevents gripper calibration",
        ));
    }
    if let Some(gp) = candidate_policy {
        if (core.driver.is_some() || core.driver_config_error.is_some())
            && core.validate_gripper_policy_against_driver_bounded(gp).is_err()
        {
            return Err((
                "failed",
                "INVALID_CALIBRATION",
                "Gripper calibration rejected by driver position limits",
            ));
        }
    }
    Ok(())
}

/// Phase 1 (owner-side, zero-alloc on `armd-control`): reserve `ConfigDomain::GripperCalibration`
/// for `gripper_zero_check(store_offset=true)` strictly BEFORE serialization/staging.
#[allow(clippy::too_many_arguments)]
pub(crate) fn zero_check_owner_reserve(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    action_id: u64,
    base_revision: u64,
    connection_id: u64,
    lease_owner: Option<(HolderLabel, u64)>,
    estop_epoch: u64,
    stop_gen: u64,
    deadline: Instant,
    candidate_policy: &GripperPolicy,
    req_seq: u64,
    code_buf: Option<String>,
    msg_buf: Option<String>,
) -> Result<
    (Reservation, ConfigProposal, ConfigAuthContext, Option<String>, Option<String>),
    (Option<String>, Option<String>),
> {
    let lease_gen = lease_owner.map_or(0, |(_, g)| g);
    let proposal = ConfigProposal {
        domain: ConfigDomain::GripperCalibration,
        expected_revision: base_revision,
        candidate: CandidateId(req_seq),
        stamp: Stamp {
            arm: ctx.arm,
            boot: ctx.boot,
            request: RequestId(req_seq),
            client: ClientId(connection_id),
            client_seq: req_seq,
            authority: Authority {
                lease_generation: lease_gen,
                epoch: estop_epoch,
                stop_generation: stop_gen,
                config_revision: core.workspace_revision,
                parent_generation: 0,
            },
            deadline,
        },
    };
    let auth = ConfigAuthContext {
        admission: AdmissionStamp {
            stop_gen,
            estop_epoch,
            lease_gen,
            connection_id,
            request_id: req_seq,
        },
        lease_holder: lease_owner,
    };
    if let Err((state, code, msg)) = check_zero_check_authority(
        core,
        ctx,
        &proposal,
        &auth,
        action_id,
        base_revision,
        Some(candidate_policy),
    ) {
        if core.zero_check_persisting == Some(action_id) {
            core.zero_check_persisting = None;
            if core.zero_check_persisting_grant.is_some_and(|(gid, _)| gid == action_id) {
                core.zero_check_persisting_grant = None;
            }
        }
        let ret = if core
            .last_action
            .as_ref()
            .is_some_and(|a| a.id == action_id && a.is_running())
        {
            core.finish_zero_check_action_in_place(action_id, state, code, msg, code_buf, msg_buf)
        } else {
            (code_buf, msg_buf)
        };
        return Err(ret);
    }
    match ConfigAuthority::reserve(core, &proposal, ctx) {
        Reserved::Granted(reservation) => Ok((reservation, proposal, auth, code_buf, msg_buf)),
        Reserved::Refused { code, .. } => {
            if core.zero_check_persisting == Some(action_id) {
                core.zero_check_persisting = None;
                if core.zero_check_persisting_grant.is_some_and(|(gid, _)| gid == action_id) {
                    core.zero_check_persisting_grant = None;
                }
            }
            let ret = core.finish_zero_check_action_in_place(
                action_id,
                "failed",
                code,
                "Failed to reserve gripper calibration domain",
                code_buf,
                msg_buf,
            );
            Err(ret)
        }
    }
}

/// Phase 3 (owner-side, zero-alloc on `armd-control`): fresh pre-rename live owner authority
/// recheck strictly BEFORE `rename_only()`. On refusal, releases the reservation (`Cleared`) and
/// finishes the action so no file is renamed onto disk.
#[allow(clippy::too_many_arguments)]
pub(crate) fn zero_check_owner_verify_authority(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    action_id: u64,
    reservation: Reservation,
    proposal: &ConfigProposal,
    auth: &ConfigAuthContext,
    base_revision: u64,
    code_buf: Option<String>,
    msg_buf: Option<String>,
) -> Result<(Reservation, Option<String>, Option<String>), (Option<String>, Option<String>)> {
    let hold_ok = core
        .core
        .config_held(ConfigDomain::GripperCalibration)
        .is_some_and(|h| h.grant == Some(reservation.grant()));
    if !hold_ok {
        let _ = ConfigAuthority::release(core, reservation);
        if core.zero_check_persisting == Some(action_id) {
            core.zero_check_persisting = None;
            if core.zero_check_persisting_grant.is_some_and(|(gid, _)| gid == action_id) {
                core.zero_check_persisting_grant = None;
            }
        }
        let ret = core.finish_zero_check_action_in_place(
            action_id,
            "failed",
            "PERSIST_FAILED",
            "Gripper calibration reservation lost before rename",
            code_buf,
            msg_buf,
        );
        return Err(ret);
    }
    if let Err((state, code, msg)) =
        check_zero_check_authority(core, ctx, proposal, auth, action_id, base_revision, None)
    {
        let _ = ConfigAuthority::release(core, reservation);
        if core.zero_check_persisting == Some(action_id) {
            core.zero_check_persisting = None;
            if core.zero_check_persisting_grant.is_some_and(|(gid, _)| gid == action_id) {
                core.zero_check_persisting_grant = None;
            }
        }
        let ret = if core
            .last_action
            .as_ref()
            .is_some_and(|a| a.id == action_id && a.is_running())
        {
            core.finish_zero_check_action_in_place(action_id, state, code, msg, code_buf, msg_buf)
        } else {
            (code_buf, msg_buf)
        };
        return Err(ret);
    }
    Ok((reservation, code_buf, msg_buf))
}

/// Phase 4b (owner-side, zero-alloc on `armd-control`): mark reservation persisted after
/// `rename_only()`. On refusal, releases the hold (`Cleared`) when `official_file_untouched` is
/// `true` (executor `.pending` staging); otherwise unconditionally enters `Reconciling`.
pub(crate) fn zero_check_owner_persisted(
    core: &mut SupervisorInner,
    action_id: u64,
    reservation: Reservation,
    simulated_err: bool,
    official_file_untouched: bool,
    code_buf: Option<String>,
    msg_buf: Option<String>,
) -> Result<(Reservation, Option<String>, Option<String>), (Option<String>, Option<String>)> {
    let res = if simulated_err {
        Err("SIMULATED_PERSISTED_ERR")
    } else {
        ConfigAuthority::persisted(core, &reservation, Durability::DurableUnconfirmed)
    };
    if res.is_ok() {
        Ok((reservation, code_buf, msg_buf))
    } else {
        if official_file_untouched {
            if !matches!(
                core.core.config_mut().release_staged_unactivated(reservation),
                Released::Cleared
            ) {
                core.core
                    .config_mut()
                    .clear_orphaned_staged_hold(ConfigDomain::GripperCalibration);
            }
        } else {
            let active_rev = ConfigAuthority::revision(core, ConfigDomain::GripperCalibration);
            core.core
                .config_mut()
                .force_reconcile(ConfigDomain::GripperCalibration, active_rev);
        }
        if core.zero_check_persisting == Some(action_id) {
            core.zero_check_persisting = None;
            if core.zero_check_persisting_grant.is_some_and(|(gid, _)| gid == action_id) {
                core.zero_check_persisting_grant = None;
            }
        }
        let ret = core.finish_zero_check_action_in_place(
            action_id,
            "failed",
            "PERSIST_FAILED",
            if official_file_untouched {
                "Failed to record pre-activation staging state"
            } else {
                "Failed to record post-rename persistence state; reconciling required"
            },
            code_buf,
            msg_buf,
        );
        Err(ret)
    }
}

/// Phase 5-error (owner-side, zero-alloc on `armd-control`): parent-directory fsync failed after
/// `rename_only()`. Keeps the OLD policy active and fails the action with `PERSIST_FAILED` +
/// durability warning. When `official_file_untouched` is `true` (executor `.pending` staging),
/// releases the hold (`Cleared`) so an untouched official file never leaves `HoldState::Reconciling`;
/// otherwise enters `HoldState::Reconciling` (motion stays inhibited) for legacy/sync callers.
pub(crate) fn zero_check_owner_reconcile_dir_sync_failed(
    core: &mut SupervisorInner,
    action_id: u64,
    reservation: Reservation,
    warning: String,
    official_file_untouched: bool,
    msg_buf: Option<String>,
) -> (Option<String>, Option<String>) {
    if official_file_untouched {
        if !matches!(
            core.core.config_mut().release_staged_unactivated(reservation),
            Released::Cleared
        ) {
            core.core
                .config_mut()
                .clear_orphaned_staged_hold(ConfigDomain::GripperCalibration);
        }
    } else if !matches!(
        ConfigAuthority::release(core, reservation),
        Released::Reconciling
    ) {
        let active_rev = ConfigAuthority::revision(core, ConfigDomain::GripperCalibration);
        core.core
            .config_mut()
            .force_reconcile(ConfigDomain::GripperCalibration, active_rev);
    }
    if core.zero_check_persisting == Some(action_id) {
        core.zero_check_persisting = None;
        if core.zero_check_persisting_grant.is_some_and(|(gid, _)| gid == action_id) {
            core.zero_check_persisting_grant = None;
        }
    }
    core.fail_zero_check_dir_sync_in_place(action_id, warning, msg_buf)
}

/// Phase 6 (owner-side, zero-alloc on `armd-control`): post-dir-sync live owner authority recheck
/// and atomic in-place activation. When `defer_publish` is `true` (executor `.pending` flow),
/// live policy and `ConfigAuthority::activate` complete here, while the action and session stay
/// running until the worker reports `PersistCmd::Published`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn zero_check_owner_activate(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    action_id: u64,
    reservation: Reservation,
    proposal: &ConfigProposal,
    auth: &ConfigAuthContext,
    base_revision: u64,
    offset_rad: f32,
    new_policy: GripperPolicy,
    new_jaw: Option<JawOpening>,
    operator_revoked: bool,
    defer_publish: bool,
    code_buf: Option<String>,
    msg_buf: Option<String>,
) -> Result<
    (Option<GripperPolicy>, Option<JawOpening>, Option<String>, Option<String>),
    (Option<GripperPolicy>, Option<JawOpening>, Option<String>, Option<String>),
> {
    let auth_res = if operator_revoked {
        Err((
            "failed",
            "OPERATOR_REQUIRED",
            if defer_publish {
                "Operator session revoked before activation"
            } else {
                "Operator session revoked after disk rename; reconciliation required"
            },
        ))
    } else {
        check_zero_check_authority(core, ctx, proposal, auth, action_id, base_revision, None)
    };
    match auth_res {
        Ok(()) => {
            let (old_policy, old_jaw) =
                core.swap_gripper_calibration_in_place(offset_rad, new_policy, new_jaw);
            let _ = core.core.config_mut().confirm_durable(&reservation);
            if !matches!(
                ConfigAuthority::activate(core, reservation),
                Activated::Active { .. }
            ) {
                let active_rev = ConfigAuthority::revision(core, ConfigDomain::GripperCalibration);
                core.core
                    .config_mut()
                    .force_reconcile(ConfigDomain::GripperCalibration, active_rev);
            }
            if !defer_publish {
                if core.zero_check_persisting == Some(action_id) {
                    core.zero_check_persisting = None;
                    if core.zero_check_persisting_grant.is_some_and(|(gid, _)| gid == action_id) {
                        core.zero_check_persisting_grant = None;
                    }
                }
                let _ = core.complete_zero_check_action(action_id, Durability::Durable, None);
            }
            Ok((old_policy, old_jaw, code_buf, msg_buf))
        }
        Err((state, code, msg)) => {
            if defer_publish {
                // Candidate is staged only at `.pending`; the official file on disk is intact,
                // so activation refusal releases the hold (`Cleared`) instead of `Reconciling`.
                if !matches!(
                    core.core.config_mut().release_staged_unactivated(reservation),
                    Released::Cleared
                ) {
                    let active_rev = ConfigAuthority::revision(core, ConfigDomain::GripperCalibration);
                    core.core
                        .config_mut()
                        .force_reconcile(ConfigDomain::GripperCalibration, active_rev);
                }
            } else if !matches!(
                core.core.config_mut().activation_failed(reservation, code),
                Activated::Reconciling { .. }
            ) {
                let active_rev = ConfigAuthority::revision(core, ConfigDomain::GripperCalibration);
                core.core
                    .config_mut()
                    .force_reconcile(ConfigDomain::GripperCalibration, active_rev);
            }
            if core.zero_check_persisting == Some(action_id) {
                core.zero_check_persisting = None;
                if core.zero_check_persisting_grant.is_some_and(|(gid, _)| gid == action_id) {
                    core.zero_check_persisting_grant = None;
                }
            }
            let (old_code, old_msg) = if core
                .last_action
                .as_ref()
                .is_some_and(|a| a.id == action_id && a.is_running())
            {
                core.finish_zero_check_action_in_place(action_id, state, code, msg, code_buf, msg_buf)
            } else {
                (code_buf, msg_buf)
            };
            Err((Some(new_policy), new_jaw, old_code, old_msg))
        }
    }
}

/// Owner-side zero-allocation post-rename reconciliation for `gripper_zero_check(store_offset=true)`
/// once parent-directory fsync succeeds on a bounded retry.
#[allow(clippy::too_many_arguments)]
pub(crate) fn zero_check_owner_finish_reconcile(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    proposal: &ConfigProposal,
    auth: &ConfigAuthContext,
    base_revision: u64,
    offset_rad: f32,
    new_policy: GripperPolicy,
    new_jaw: Option<JawOpening>,
    operator_revoked: bool,
    operator_grant: Option<crate::auth::OperatorGrant>,
) -> Result<(Option<GripperPolicy>, Option<JawOpening>), (&'static str, Option<GripperPolicy>, Option<JawOpening>)> {
    let is_reconciling = core
        .core
        .config_held(ConfigDomain::GripperCalibration)
        .is_some_and(|h| h.state == crate::rt_core::config_iface::HoldState::Reconciling);
    if !is_reconciling {
        return Err(("NOT_HELD", Some(new_policy), new_jaw));
    }
    // Executor: the worker's sample is advisory; the owner validates the grant carried through
    // reconciliation at commit time. Legacy (`None`) samples and commits under one lock.
    if operator_revoked
        || operator_grant.is_some_and(|g| !g.is_valid_at(ctx.now, core.operator_auth_enforced))
    {
        return Err(("OPERATOR_REQUIRED", Some(new_policy), new_jaw));
    }
    if core.safety.state().is_stopped() {
        return Err(("STOPPED", Some(new_policy), new_jaw));
    }
    if let Err(err) = check_config_authority_inner(core, ctx, proposal, auth, false, false) {
        let code = match err {
            ConfigError::LeaseLost | ConfigError::ConnectionClosed => "LEASE_LOST",
            ConfigError::EpochMismatch { .. } => "EPOCH_MISMATCH",
            ConfigError::Stopped => "STOPPED",
            _ => "AUTHORITY_REVOKED",
        };
        return Err((code, Some(new_policy), new_jaw));
    }
    if core.gripper_calib_revision != base_revision {
        return Err(("POLICY_REVISION_MISMATCH", Some(new_policy), new_jaw));
    }
    if core.driver_config_error.is_some()
        || ((core.driver.is_some() || core.driver_config_error.is_some())
            && core.validate_gripper_policy_against_driver_bounded(&new_policy).is_err())
    {
        return Err(("INVALID_CALIBRATION", Some(new_policy), new_jaw));
    }
    let (old_policy, old_jaw) = core.swap_gripper_calibration_in_place(offset_rad, new_policy, new_jaw);
    let cleared = ConfigAuthority::finish_reconcile(core, ConfigDomain::GripperCalibration);
    debug_assert!(cleared);
    Ok((old_policy, old_jaw))
}

/// Control outcome returned by [`CommonCommitWorker`] adapters for `gripper_zero_check(store_offset=true)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ZeroCheckCommitStop {
    /// Failed/aborted cleanly or entered `Reconciling`; worker loop may continue to next job.
    Aborted,
    /// Dedicated worker thread disconnected or stopped; worker loop must exit immediately.
    WorkerDisconnected,
}

fn zero_check_op_auth_ok(ctx: &crate::supervisor::ZeroCheckPersistContext) -> bool {
    ctx.operator_auth
        .as_ref()
        .is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()))
}

struct LegacyZeroCheckBackend {
    sup_inner: Arc<crate::supervisor_lock::SupervisorLock<SupervisorInner>>,
    cw_lock: Arc<tokio::sync::Mutex<()>>,
    published_config: Arc<PublishedConfigHub>,
    hooks: ConfigCommitTestHooks,
    job: crate::supervisor::PendingZeroCheckPersist,
    candidate_policy: Option<GripperPolicy>,
    reservation: Option<Reservation>,
    proposal: Option<ConfigProposal>,
    auth: Option<ConfigAuthContext>,
}

impl CommonCommitWorker for LegacyZeroCheckBackend {
    type Ok = ();
    type Err = ZeroCheckCommitStop;
    const RUN_BLOCKING_ON_TOKIO: bool = true;

    fn before_reserve(&mut self) {
        if let Some(h) = self.hooks.before_reserve.as_ref() {
            h();
        }
    }

    fn should_fail_rename_once(&self) -> bool {
        self.hooks
            .fail_rename_once
            .as_ref()
            .is_some_and(|f| f.swap(false, Ordering::AcqRel))
    }

    fn should_fail_dir_sync_once(&self) -> bool {
        self.hooks
            .fail_dir_sync_once
            .as_ref()
            .is_some_and(|f| f.swap(false, Ordering::AcqRel))
    }

    fn dir_sync_override(&self) -> Option<DirSyncOverride> {
        self.hooks.dir_sync_override.clone()
    }

    async fn reserve(&mut self) -> Result<(), Self::Err> {
        let req_seq = NEXT_CONFIG_REQ_SEQ.fetch_add(1, Ordering::Relaxed);
        let policy = self
            .candidate_policy
            .as_ref()
            .expect("candidate_policy validated before reserve");
        let mut inner = self.sup_inner.write().await;
        if inner.detached {
            return Err(ZeroCheckCommitStop::Aborted);
        }
        let ctx = CommitCtx {
            arm: inner.core.id(),
            boot: BootId::process(),
            now: Instant::now(),
            stop_generation: inner.current_stop_gen(),
        };
        match zero_check_owner_reserve(
            &mut inner,
            &ctx,
            self.job.action_id,
            self.job.base_revision,
            self.job.connection_id,
            self.job.lease_owner,
            self.job.estop_epoch,
            self.job.stop_gen,
            self.job.deadline,
            policy,
            req_seq,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        ) {
            Ok((r, p, a, _, _)) => {
                self.reservation = Some(r);
                self.proposal = Some(p);
                self.auth = Some(a);
                Ok(())
            }
            Err(_) => Err(ZeroCheckCommitStop::Aborted),
        }
    }

    async fn prepare_write_after_reserve(&mut self) -> Result<Option<PendingWrite>, Self::Err> {
        Ok(Some(self.job.ctx.serialize_write(self.job.offset_rad)))
    }

    fn after_reserve_before_rename(&mut self) -> bool {
        if let Some(h) = self.hooks.after_reserve_before_rename.as_ref() {
            h();
        }
        false
    }

    async fn verify_authority_before_rename(&mut self) -> Result<(), Self::Err> {
        if !zero_check_op_auth_ok(&self.job.ctx) {
            let mut inner = self.sup_inner.write().await;
            if let Some(r) = self.reservation.take() {
                let _ = ConfigAuthority::release(&mut *inner, r);
            }
            if inner.zero_check_persisting == Some(self.job.action_id) {
                inner.zero_check_persisting = None;
            }
            let _ = inner.fail_zero_check_action_owned(
                self.job.action_id,
                "OPERATOR_REQUIRED".into(),
                "Operator session expired or locked before gripper calibration persisted".into(),
            );
            return Err(ZeroCheckCommitStop::Aborted);
        }
        let r = self.reservation.take().expect("reservation held");
        let proposal = self.proposal.expect("proposal recorded");
        let auth = self.auth.expect("auth recorded");
        let mut inner = self.sup_inner.write().await;
        if inner.detached {
            return Err(ZeroCheckCommitStop::Aborted);
        }
        let ctx = CommitCtx {
            arm: inner.core.id(),
            boot: BootId::process(),
            now: Instant::now(),
            stop_generation: inner.current_stop_gen(),
        };
        match zero_check_owner_verify_authority(
            &mut inner,
            &ctx,
            self.job.action_id,
            r,
            &proposal,
            &auth,
            self.job.base_revision,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        ) {
            Ok((r, _, _)) => {
                self.reservation = Some(r);
                Ok(())
            }
            Err(_) => Err(ZeroCheckCommitStop::Aborted),
        }
    }

    async fn mark_persisted(&mut self) -> Result<(), Self::Err> {
        let sim_persisted_fail = self
            .hooks
            .fail_persisted_once
            .as_ref()
            .is_some_and(|f| f.swap(false, Ordering::AcqRel));
        let r = self.reservation.take().expect("reservation held");
        let mut inner = self.sup_inner.write().await;
        match zero_check_owner_persisted(
            &mut inner,
            self.job.action_id,
            r,
            sim_persisted_fail,
            false,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        ) {
            Ok((r, _, _)) => {
                self.reservation = Some(r);
                Ok(())
            }
            Err(_) => Err(ZeroCheckCommitStop::Aborted),
        }
    }

    async fn reconcile_after_dir_sync_failure(
        &mut self,
        target_path: PathBuf,
        warn: String,
    ) -> Self::Err {
        if let Some(r) = self.reservation.take() {
            let mut inner = self.sup_inner.write().await;
            let _ = zero_check_owner_reconcile_dir_sync_failed(
                &mut inner,
                self.job.action_id,
                r,
                warn.clone(),
                false,
                Some(String::with_capacity(160)),
            );
        }
        if let (Some(proposal), Some(auth), Some(policy)) = (
            self.proposal,
            self.auth,
            self.candidate_policy.take(),
        ) {
            let sup_inner = Arc::clone(&self.sup_inner);
            let cw_lock = Arc::clone(&self.cw_lock);
            let published_config = Arc::clone(&self.published_config);
            let dir_sync_override = self.hooks.dir_sync_override.clone();
            let interval = self
                .hooks
                .reconcile_retry_interval
                .unwrap_or(DEFAULT_RECONCILE_RETRY_INTERVAL);
            let max_attempts = self
                .hooks
                .reconcile_max_attempts
                .unwrap_or(DEFAULT_RECONCILE_MAX_ATTEMPTS)
                .max(1);
            let base_revision = self.job.base_revision;
            let offset_rad = self.job.offset_rad;
            let ctx_clone = self.job.ctx.clone();
            tokio::spawn(async move {
                let file_name = target_file_label(&target_path);
                let mut last_warn = warn;
                let mut policy_opt = Some(policy);
                let mut jaw_opt = ctx_clone.jaw_opening.clone();
                for attempt in 1..=max_attempts {
                    tokio::time::sleep(interval).await;
                    {
                        let inner = sup_inner.read().await;
                        if inner.detached {
                            return;
                        }
                        let still_reconciling = inner
                            .core
                            .config_held(ConfigDomain::GripperCalibration)
                            .is_some_and(|h| h.state == crate::rt_core::config_iface::HoldState::Reconciling);
                        if !still_reconciling {
                            return;
                        }
                    }
                    let pds = PendingDirSync::for_path(&target_path);
                    let ov = dir_sync_override.clone();
                    let sync_res = tokio::task::spawn_blocking(move || {
                        let outcome = match ov {
                            Some(f) => pds.sync_with(|dir| f(dir)),
                            None => pds.sync_parent_dir(),
                        };
                        match outcome {
                            PublishOutcome::Published => Ok(()),
                            PublishOutcome::PublishedDurabilityUnconfirmed(w) => Err(w),
                        }
                    })
                    .await
                    .map_err(|e| e.to_string())
                    .and_then(|r| r);
                    if let Err(w) = sync_res {
                        last_warn = w;
                        tracing::warn!(
                            "Legacy zero_check reconciliation retry {attempt}/{max_attempts} for {file_name} failed dir fsync: {last_warn}"
                        );
                        continue;
                    }
                    let _cw = cw_lock.lock().await;
                    let operator_revoked = !zero_check_op_auth_ok(&ctx_clone);
                    let Some(pol) = policy_opt.take() else {
                        return;
                    };
                    let pub_pol = pol.clone();
                    let jaw = jaw_opt.take();
                    let pub_jaw = jaw.clone();
                    let finish_res = {
                        let mut inner = sup_inner.write().await;
                        if inner.detached {
                            return;
                        }
                        let commit_ctx = CommitCtx {
                            arm: inner.core.id(),
                            boot: BootId::process(),
                            now: Instant::now(),
                            stop_generation: inner.current_stop_gen(),
                        };
                        zero_check_owner_finish_reconcile(
                            &mut inner,
                            &commit_ctx,
                            &proposal,
                            &auth,
                            base_revision,
                            offset_rad,
                            pol,
                            jaw,
                            operator_revoked,
                            None,
                        )
                    };
                    match finish_res {
                        Ok(_) => {
                            published_config.apply_zero_check_commit(
                                offset_rad,
                                pub_pol,
                                pub_jaw,
                                base_revision + 1,
                            );
                            tracing::info!(
                                "Legacy zero_check reconciliation succeeded for {file_name} on retry {attempt}/{max_attempts}"
                            );
                            return;
                        }
                        Err(("NOT_HELD", _, _)) => return,
                        Err((code, _, _)) => {
                            let msg = format!(
                                "Configuration persistence failed for {file_name}: post-rename reconciliation rejected ({code})"
                            );
                            tracing::error!("{msg}");
                            let mut inner = sup_inner.write().await;
                            if !inner.detached {
                                install_latch_config_reconcile_fault(
                                    &mut inner,
                                    ConfigDomain::GripperCalibration,
                                    BoundedText::new(&msg),
                                );
                            }
                            return;
                        }
                    }
                }
                let msg = format!(
                    "Configuration persistence failed for {file_name} after {max_attempts} reconciliation attempts: {last_warn}"
                );
                tracing::error!("{msg}");
                let mut inner = sup_inner.write().await;
                if !inner.detached {
                    install_latch_config_reconcile_fault(
                        &mut inner,
                        ConfigDomain::GripperCalibration,
                        BoundedText::new(&msg),
                    );
                }
            });
        }
        ZeroCheckCommitStop::Aborted
    }

    fn after_rename_before_activate(&mut self) -> bool {
        if let Some(h) = self.hooks.after_rename_before_activate.as_ref() {
            h();
        }
        false
    }

    async fn activate_and_publish(&mut self, _disk_published: bool) -> Result<Self::Ok, Self::Err> {
        let operator_revoked = !zero_check_op_auth_ok(&self.job.ctx);
        let r = self.reservation.take().expect("reservation held");
        let proposal = self.proposal.expect("proposal recorded");
        let auth = self.auth.expect("auth recorded");
        let policy = self.candidate_policy.take().expect("candidate_policy held");
        let pub_policy = policy.clone();
        let pub_jaw = self.job.ctx.jaw_opening.clone();
        let jaw = self.job.ctx.jaw_opening.take();
        let activated_ok = {
            let mut inner = self.sup_inner.write().await;
            if inner.detached {
                return Err(ZeroCheckCommitStop::Aborted);
            }
            let ctx = CommitCtx {
                arm: inner.core.id(),
                boot: BootId::process(),
                now: Instant::now(),
                stop_generation: inner.current_stop_gen(),
            };
            zero_check_owner_activate(
                &mut inner,
                &ctx,
                self.job.action_id,
                r,
                &proposal,
                &auth,
                self.job.base_revision,
                self.job.offset_rad,
                policy,
                jaw,
                operator_revoked,
                false,
                Some(String::with_capacity(64)),
                Some(String::with_capacity(160)),
            )
            .is_ok()
        };
        if activated_ok {
            self.published_config.apply_zero_check_commit(
                self.job.offset_rad,
                pub_policy,
                pub_jaw,
                self.job.base_revision + 1,
            );
        }
        Ok(())
    }

    async fn release_pre_rename(&mut self) {
        if let Some(r) = self.reservation.take() {
            let mut inner = self.sup_inner.write().await;
            let _ = ConfigAuthority::release(&mut *inner, r);
        }
    }

    async fn abort_pre_rename_fs(&mut self, _phase: &'static str, err: String) -> Self::Err {
        let mut inner = self.sup_inner.write().await;
        if let Some(r) = self.reservation.take() {
            let _ = ConfigAuthority::release(&mut *inner, r);
        }
        if inner.zero_check_persisting == Some(self.job.action_id) {
            inner.zero_check_persisting = None;
        }
        let _ = inner.fail_zero_check_action_owned(
            self.job.action_id,
            "PERSIST_FAILED".into(),
            format!("Failed to persist gripper calibration: {err}"),
        );
        ZeroCheckCommitStop::Aborted
    }

    async fn reconcile_rename_join_error(&mut self, err: String) -> Self::Err {
        let mut inner = self.sup_inner.write().await;
        if let Some(r) = self.reservation.take() {
            let _ = zero_check_owner_persisted(
                &mut inner,
                self.job.action_id,
                r,
                true,
                false,
                Some(String::with_capacity(64)),
                Some(String::with_capacity(160)),
            );
        }
        tracing::error!("Legacy zero_check rename task failed: {err}");
        ZeroCheckCommitStop::Aborted
    }

    fn worker_disconnected_err(&mut self) -> Self::Err {
        ZeroCheckCommitStop::WorkerDisconnected
    }
}

/// Legacy `tick_shared` async entry point for `gripper_zero_check(store_offset=true)`:
/// executes through the ONE common commit worker ([`execute_common_commit`]).
pub(crate) async fn run_legacy_zero_check_commit(
    sup_inner: Arc<crate::supervisor_lock::SupervisorLock<SupervisorInner>>,
    cw_lock: Arc<tokio::sync::Mutex<()>>,
    published_config: Arc<PublishedConfigHub>,
    hooks: ConfigCommitTestHooks,
    job: crate::supervisor::PendingZeroCheckPersist,
) {
    let _cw = cw_lock.lock().await;
    if !zero_check_op_auth_ok(&job.ctx) {
        let mut inner = sup_inner.write().await;
        if inner.zero_check_persisting == Some(job.action_id) {
            inner.zero_check_persisting = None;
        }
        let _ = inner.fail_zero_check_action_owned(
            job.action_id,
            "OPERATOR_REQUIRED".into(),
            "Operator session expired or locked before gripper calibration persisted".into(),
        );
        return;
    }
    let candidate_policy = match SupervisorInner::validate_gripper_calibration_off_thread(
        job.ctx.base_gripper_policy.as_ref(),
        job.ctx.jaw_opening.as_ref(),
        job.ctx.gripper_limit.as_ref(),
        job.ctx.active_tool_is_stock,
        Some(job.offset_rad),
        job.ctx.jaw_opening.as_ref(),
    ) {
        Ok(p) => p,
        Err(e) => {
            tracing::warn!("Failed to validate gripper calibration after zero_check: {e}");
            let mut inner = sup_inner.write().await;
            if inner.zero_check_persisting == Some(job.action_id) {
                inner.zero_check_persisting = None;
            }
            let _ = inner.fail_zero_check_action_owned(
                job.action_id,
                "INVALID_CALIBRATION".into(),
                format!("Gripper calibration rejected: {e}"),
            );
            return;
        }
    };
    let mut backend = LegacyZeroCheckBackend {
        sup_inner,
        cw_lock: Arc::clone(&cw_lock),
        published_config,
        hooks,
        job,
        candidate_policy: Some(candidate_policy),
        reservation: None,
        proposal: None,
        auth: None,
    };
    let _ = execute_common_commit(&mut backend).await;
}

struct SyncZeroCheckBackend<'a> {
    core: &'a mut SupervisorInner,
    action_id: u64,
    offset_rad: f32,
    base_revision: u64,
    lease_owner: Option<(HolderLabel, u64)>,
    estop_epoch: u64,
    stop_gen: u64,
    deadline: Instant,
    ctx_box: Box<crate::supervisor::ZeroCheckPersistContext>,
    candidate_policy: Option<GripperPolicy>,
    reservation: Option<Reservation>,
    proposal: Option<ConfigProposal>,
    auth: Option<ConfigAuthContext>,
}

impl CommonCommitWorker for SyncZeroCheckBackend<'_> {
    type Ok = ();
    type Err = ZeroCheckCommitStop;
    const RUN_BLOCKING_ON_TOKIO: bool = false;

    async fn reserve(&mut self) -> Result<(), Self::Err> {
        let req_seq = NEXT_CONFIG_REQ_SEQ.fetch_add(1, Ordering::Relaxed);
        let policy = self
            .candidate_policy
            .as_ref()
            .expect("candidate_policy validated before reserve");
        let commit_ctx = CommitCtx {
            arm: self.core.core.id(),
            boot: BootId::process(),
            now: Instant::now(),
            stop_generation: self.stop_gen,
        };
        match zero_check_owner_reserve(
            self.core,
            &commit_ctx,
            self.action_id,
            self.base_revision,
            0,
            self.lease_owner,
            self.estop_epoch,
            self.stop_gen,
            self.deadline,
            policy,
            req_seq,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        ) {
            Ok((r, p, a, _, _)) => {
                self.reservation = Some(r);
                self.proposal = Some(p);
                self.auth = Some(a);
                Ok(())
            }
            Err(_) => Err(ZeroCheckCommitStop::Aborted),
        }
    }

    async fn prepare_write_after_reserve(&mut self) -> Result<Option<PendingWrite>, Self::Err> {
        Ok(Some(self.ctx_box.serialize_write(self.offset_rad)))
    }

    async fn verify_authority_before_rename(&mut self) -> Result<(), Self::Err> {
        if !zero_check_op_auth_ok(&self.ctx_box) {
            if let Some(r) = self.reservation.take() {
                let _ = ConfigAuthority::release(self.core, r);
            }
            if self.core.zero_check_persisting == Some(self.action_id) {
                self.core.zero_check_persisting = None;
            }
            let _ = self.core.fail_zero_check_action_owned(
                self.action_id,
                "OPERATOR_REQUIRED".into(),
                "Operator session expired or locked before gripper calibration persisted".into(),
            );
            return Err(ZeroCheckCommitStop::Aborted);
        }
        let r = self.reservation.take().expect("reservation held");
        let proposal = self.proposal.expect("proposal recorded");
        let auth = self.auth.expect("auth recorded");
        let verify_ctx = CommitCtx {
            arm: self.core.core.id(),
            boot: BootId::process(),
            now: Instant::now(),
            stop_generation: self.core.current_stop_gen(),
        };
        match zero_check_owner_verify_authority(
            self.core,
            &verify_ctx,
            self.action_id,
            r,
            &proposal,
            &auth,
            self.base_revision,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        ) {
            Ok((r, _, _)) => {
                self.reservation = Some(r);
                Ok(())
            }
            Err(_) => Err(ZeroCheckCommitStop::Aborted),
        }
    }

    async fn mark_persisted(&mut self) -> Result<(), Self::Err> {
        let r = self.reservation.take().expect("reservation held");
        match zero_check_owner_persisted(
            self.core,
            self.action_id,
            r,
            false,
            false,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        ) {
            Ok((r, _, _)) => {
                self.reservation = Some(r);
                Ok(())
            }
            Err(_) => Err(ZeroCheckCommitStop::Aborted),
        }
    }

    async fn reconcile_after_dir_sync_failure(
        &mut self,
        target_path: PathBuf,
        warn: String,
    ) -> Self::Err {
        if let Some(r) = self.reservation.take() {
            let _ = zero_check_owner_reconcile_dir_sync_failed(
                self.core,
                self.action_id,
                r,
                warn.clone(),
                false,
                Some(String::with_capacity(160)),
            );
        }
        let file_name = target_file_label(&target_path);
        let msg = format!(
            "Configuration persistence failed for {file_name}: parent directory fsync failed ({warn})"
        );
        install_latch_config_reconcile_fault(
            self.core,
            ConfigDomain::GripperCalibration,
            BoundedText::new(&msg),
        );
        ZeroCheckCommitStop::Aborted
    }

    async fn activate_and_publish(&mut self, _disk_published: bool) -> Result<Self::Ok, Self::Err> {
        let operator_revoked = !zero_check_op_auth_ok(&self.ctx_box);
        let r = self.reservation.take().expect("reservation held");
        let proposal = self.proposal.expect("proposal recorded");
        let auth = self.auth.expect("auth recorded");
        let policy = self.candidate_policy.take().expect("candidate_policy held");
        let jaw = self.ctx_box.jaw_opening.take();
        let act_ctx = CommitCtx {
            arm: self.core.core.id(),
            boot: BootId::process(),
            now: Instant::now(),
            stop_generation: self.core.current_stop_gen(),
        };
        let _ = zero_check_owner_activate(
            self.core,
            &act_ctx,
            self.action_id,
            r,
            &proposal,
            &auth,
            self.base_revision,
            self.offset_rad,
            policy,
            jaw,
            operator_revoked,
            false,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        );
        Ok(())
    }

    async fn release_pre_rename(&mut self) {
        if let Some(r) = self.reservation.take() {
            let _ = ConfigAuthority::release(self.core, r);
        }
    }

    async fn abort_pre_rename_fs(&mut self, _phase: &'static str, err: String) -> Self::Err {
        if let Some(r) = self.reservation.take() {
            let _ = ConfigAuthority::release(self.core, r);
        }
        if self.core.zero_check_persisting == Some(self.action_id) {
            self.core.zero_check_persisting = None;
        }
        let _ = self.core.fail_zero_check_action_owned(
            self.action_id,
            "PERSIST_FAILED".into(),
            format!("Failed to persist gripper calibration: {err}"),
        );
        ZeroCheckCommitStop::Aborted
    }

    async fn reconcile_rename_join_error(&mut self, _err: String) -> Self::Err {
        ZeroCheckCommitStop::Aborted
    }

    fn worker_disconnected_err(&mut self) -> Self::Err {
        ZeroCheckCommitStop::WorkerDisconnected
    }
}

/// Synchronous legacy fallback when `defer_zero_check_persist == false` (bare `SupervisorInner`
/// tests): executes through the ONE common commit worker ([`run_common_commit_sync`]).
pub(crate) fn commit_zero_check_sync(
    core: &mut SupervisorInner,
    action_id: u64,
    offset_rad: f32,
    ctx_box: Box<crate::supervisor::ZeroCheckPersistContext>,
) {
    if let Some(a) = core.last_action.as_mut().filter(|a| a.id == action_id) {
        a.state = HolderLabel::new("running");
        a.finished_at = None;
    }
    core.zero_check_persisting = Some(action_id);

    if !zero_check_op_auth_ok(&ctx_box) {
        core.zero_check_persisting = None;
        let _ = core.fail_zero_check_action_owned(
            action_id,
            "OPERATOR_REQUIRED".into(),
            "Operator session expired or locked before gripper calibration persisted".into(),
        );
        return;
    }
    let candidate_policy = match SupervisorInner::validate_gripper_calibration_off_thread(
        ctx_box.base_gripper_policy.as_ref(),
        ctx_box.jaw_opening.as_ref(),
        ctx_box.gripper_limit.as_ref(),
        ctx_box.active_tool_is_stock,
        Some(offset_rad),
        ctx_box.jaw_opening.as_ref(),
    ) {
        Ok(p) => p,
        Err(e) => {
            tracing::warn!("Failed to apply gripper calibration after zero_check: {e}");
            core.zero_check_persisting = None;
            let _ = core.fail_zero_check_action_owned(
                action_id,
                "INVALID_CALIBRATION".into(),
                format!("Gripper calibration rejected: {e}"),
            );
            return;
        }
    };

    let base_revision = core.gripper_calib_revision;
    let lease_owner = core
        .active_lease
        .as_ref()
        .map(|l| (HolderLabel::new(&l.client_id), l.generation));
    let estop_epoch = core.safety.epoch_id();
    let stop_gen = core.current_stop_gen();
    let deadline = Instant::now() + CONFIG_PROPOSAL_BUDGET;
    let mut backend = SyncZeroCheckBackend {
        core,
        action_id,
        offset_rad,
        base_revision,
        lease_owner,
        estop_epoch,
        stop_gen,
        deadline,
        ctx_box,
        candidate_policy: Some(candidate_policy),
        reservation: None,
        proposal: None,
        auth: None,
    };
    let _ = run_common_commit_sync(&mut backend);
}

/// Pre-builds an `EvalBufs` off-thread for a given `(ArmModel, WorkspacePolicy, CameraMount, tcp_offset)`.
pub fn prepare_eval_bufs_off_thread(
    arm_model: &ArmModel,
    ws: &crate::model::WorkspacePolicy,
    cam: &CameraMount,
    tcp_offset: [f64; 3],
) -> EvalBufs {
    let mut bufs = EvalBufs::default();
    bufs.prepare_for(arm_model, ws, cam, tcp_offset);
    bufs
}

/// Owner-routed gravity-compensation enter/exit command (shared across `Legacy` and `Executor`).
///
/// In `OWNER_EXECUTOR`, mock (driverless) entry/exit runs zero-alloc on `armd-control` using a
/// pre-cloned `GravityCompPolicy`; hardware drivers requiring synchronous CAN MIT mode-switch
/// fail closed with `503 EXECUTOR_CAPABILITY_UNSUPPORTED`.
pub async fn gravity_comp_command(
    sup: &Arc<ArmSupervisor>,
    admission: AdmissionStamp,
    lease_holder: Option<(HolderLabel, u64)>,
    enable: bool,
    scale: Option<f32>,
    damping_kd: Option<Vec<f32>>,
) -> Result<GravityCompReply, ArmError> {
    let op = if enable {
        let pub_cfg = sup.published_config().await?;
        if crate::gravity_comp::executor_blocks_hardware_gravity_comp(
            sup.owner_mode() == OWNER_EXECUTOR,
            pub_cfg.has_driver,
        ) {
            return Err(executor_capability_unsupported(
                "gravity_comp hardware MIT mode-switch is not supported on the RT executor",
            ));
        }
        let policy = pub_cfg
            .config
            .gravity_comp
            .clone()
            .filter(|_| pub_cfg.config.six_plus_gripper() && pub_cfg.config.gravity_model.is_some());
        let damping_arr = match damping_kd {
            None => None,
            Some(v) => {
                let max_kd = policy.as_ref().map_or(f32::INFINITY, |p| p.damping_kd_max);
                if v.len() != 6 || !v.iter().all(|k| k.is_finite() && *k >= 0.0 && *k <= max_kd) {
                    return Err(ArmError::Rejected {
                        status: 400,
                        code: "INVALID_COMMAND".into(),
                        message: format!("damping_kd must be 6 values in [0, {max_kd}]"),
                    });
                }
                let mut arr = [0.0f32; 6];
                arr.copy_from_slice(&v);
                Some(arr)
            }
        };
        GravityCompOp::Enter {
            admission,
            lease_holder,
            policy,
            scale,
            damping_kd: damping_arr,
        }
    } else {
        if sup.owner_mode() == OWNER_EXECUTOR {
            let pub_cfg = sup.published_config().await?;
            if crate::gravity_comp::executor_blocks_hardware_gravity_comp(true, pub_cfg.has_driver) {
                return Err(executor_capability_unsupported(
                    "gravity_comp hardware MIT mode-switch is not supported on the RT executor",
                ));
            }
        }
        GravityCompOp::Exit {
            lease_holder,
            code: "REQUESTED",
            message: "Exit requested",
        }
    };
    match sup
        .dispatch_config_request(ConfigRequest::GravityComp(op))
        .await
    {
        Ok(ConfigReply::GravityCompDone(Ok(reply))) => Ok(reply),
        Ok(ConfigReply::GravityCompDone(Err(e))) => Err(e.to_arm_error()),
        Err(e) => Err(e.to_arm_error()),
        _ => Err(ArmError::Internal("unexpected gravity_comp reply".into())),
    }
}
