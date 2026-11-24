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

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, RwLock};
use std::time::Instant;

use serde_json::{json, Value};

use crate::calibration::{
    CalibError, CalibrationFile, CalibrationStale,
};
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
use crate::persist::{PendingWrite, PublishOutcome, StagedWrite};
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
static NEXT_CALIB_JOB_TOKEN: AtomicU64 = AtomicU64::new(1);

/// Allocates a unique monotonic job token for binding `calibration_busy` acquisition,
/// owner mutations, and cleanup to the same calibration / motor-zero transaction.
pub fn next_calib_job_token() -> u64 {
    NEXT_CALIB_JOB_TOKEN.fetch_add(1, Ordering::Relaxed)
}

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
    pub grasp_setup_revision: u64,
    pub tool_commissioning_path: Option<PathBuf>,
    pub tool_commissioning_confirmed_at: Option<String>,
    pub tool_commissioning_revision: u64,
    pub calibration: Option<CalibrationFile>,
    pub calibration_revision: u64,
    pub calibration_busy: bool,
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
    pub fallback_measured_positions: Vec<f32>,
}

impl PublishedConfig {
    /// Captures a complete configuration snapshot from `inner` off the RT thread.
    pub fn from_inner(inner: &SupervisorInner) -> Self {
        let driver_torque_feedback = inner
            .driver
            .as_ref()
            .is_some_and(|d| d.lock().is_ok_and(|drv| drv.capabilities().torque_feedback));
        let driver_temperature_feedback = inner
            .driver
            .as_ref()
            .is_none_or(|d| d.lock().is_ok_and(|drv| drv.capabilities().temperature_feedback));
        let driver_motor_current_cap = inner
            .driver
            .as_ref()
            .and_then(|d| d.lock().ok().and_then(|drv| drv.motor_current_cap()))
            .and_then(|cap| serde_json::to_value(cap).ok());
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
            sequences: inner.sequences.clone(),
            episodes: inner.episodes.clone(),
            grasp_setup_revision: inner.grasp_setup_revision,
            tool_commissioning_path: inner.tool_commissioning_path.clone(),
            tool_commissioning_confirmed_at: inner.tool_commissioning_confirmed_at.clone(),
            tool_commissioning_revision: inner.tool_commissioning_revision,
            calibration: inner.calibration.clone(),
            calibration_revision: inner.calibration_revision,
            calibration_busy: inner.calibration_busy,
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
            policy_live_allowed: inner.policy_live_allowed(),
            fallback_measured_positions: inner.measured_positions.clone(),
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
        v["running"] = json!(self.sequences.pins.as_ref().is_some_and(|p| p.sequence_id == s.id));
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

    pub fn to_calib_error(&self) -> CalibError {
        match self {
            Self::RevisionMismatch { expected, current } => {
                let mut err = CalibError::new(
                    409,
                    "POLICY_REVISION_MISMATCH",
                    format!("Policy revision mismatch: expected {expected}, current {current}"),
                );
                err.extra.insert("current_revision".into(), json!(current));
                err
            }
            Self::Busy(msg) => CalibError::new(409, "CONFIG_BUSY", *msg),
            Self::Unsupported(msg) => CalibError::new(400, "UNSUPPORTED", *msg),
            Self::MotionBusy { subject, reason } => {
                CalibError::new(409, "MOTION_BUSY", format!("{subject} cannot change while {reason}"))
            }
            Self::LeaseLost | Self::ConnectionClosed => CalibError::new(
                409,
                "LEASE_LOST",
                "This connection lost the lease during the calibration",
            ),
            Self::EpochMismatch { .. } => CalibError::new(
                409,
                "EPOCH_MISMATCH",
                "The epoch changed during the calibration (stop or e-stop)",
            ),
            Self::Stopped => CalibError::new(
                409,
                "STOPPED",
                "An authorized stop intervened during the calibration",
            ),
            Self::CalibPrecondition { status, code, message } => CalibError::new(*status, code, *message),
            Self::ExecutorCapabilityUnsupported(msg) => {
                CalibError::new(503, "EXECUTOR_CAPABILITY_UNSUPPORTED", *msg)
            }
            Self::Reconciling(code) => CalibError::new(
                409,
                code,
                format!("calibration persisted on disk but activation failed ({code}); reconciling required"),
            ),
            other => match other.to_arm_error() {
                ArmError::Rejected { status, code, message } => CalibError::new(status, &code, message),
                ArmError::PolicyRevisionMismatch { expected, current } => {
                    let mut err = CalibError::new(
                        409,
                        "POLICY_REVISION_MISMATCH",
                        format!("Policy revision mismatch: expected {expected}, current {current}"),
                    );
                    err.extra.insert("current_revision".into(), json!(current));
                    err
                }
                e => CalibError::new(500, "INTERNAL", e.to_string()),
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

/// Single authority check for `ReserveConfig` and `ActivateConfig`.
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
    if let Some(code) = proposal.check(ctx.arm, ctx.boot, ctx.now, &current) {
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
        kin_changes: bool,
        allow_during_calibration_busy: bool,
    },
    JointZeroCommit {
        require_zero_idle: bool,
        job_token: Option<u64>,
        expected_gripper_calib_revision: Option<u64>,
    },
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
    Calibration(Box<CalibrationCandidate>),
    JointZeroCommit(Box<JointZeroCommitCandidate>),
}

/// Pre-built calibration live-state payload (zero heap alloc/free on RT swap).
pub struct CalibrationCandidate {
    pub calibration: Option<CalibrationFile>,
    pub camera_mount: CameraMount,
    pub kin_changed: bool,
    pub arm_model: Option<ArmModel>,
    pub pose_tcps: Option<Vec<(String, Option<PoseTcp>)>>,
    pub effective_floor: Option<(f32, &'static str)>,
    pub ws_eval_bufs: Option<EvalBufs>,
    #[cfg(feature = "policy-runtime")]
    pub calibration_digest: String,
    pub require_kinematics_disarmed: bool,
}

/// Pre-built atomic joint-zero + invalidation live payload (swapped in-place on RT with
/// zero heap allocation and zero heap free).
pub struct JointZeroCommitCandidate {
    /// Software/restore zero offsets installed strictly at activation (after rename + dir-sync).
    /// `None` for post-motor-zero or pre-motor-zero MIT invalidations that do not change offsets.
    pub new_offsets: Option<Vec<f32>>,
    pub file_changes: bool,
    pub calibration: Option<CalibrationFile>,
    pub arm_model_uncorrected: Option<ArmModel>,
    pub pose_tcps: Option<Vec<(String, Option<PoseTcp>)>>,
    pub effective_floor: Option<(f32, &'static str)>,
    pub ws_eval_bufs: Option<EvalBufs>,
    #[cfg(feature = "policy-runtime")]
    pub calibration_digest: Option<String>,
    pub next_poses: Option<PoseList>,
    pub gripper_stale: Option<CalibrationStale>,
    pub reset_gripper_baseline: bool,
    pub require_zero_idle: bool,
    pub job_token: Option<u64>,
    pub expected_gripper_calib_revision: Option<u64>,
}

#[derive(Debug, Clone, Copy)]
pub struct CalibBeginCheck {
    pub check_grasp: bool,
    pub check_kinematics_armed: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct CalibBeginSnapshot {
    pub is_torqued: bool,
    pub epoch_id: u64,
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
    pub poses_revision: u64,
    pub gripper_calib_revision: u64,
    pub epoch_id: u64,
    pub is_torqued: bool,
    pub safety_state: SafetyStateTag,
    pub gravity_comp_active: bool,
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
    },
    Persisted(PersistedConfig),
    Activate(ActivateConfig),
    Release(ReleaseConfig),
    SetCalibrationBusy {
        busy: bool,
        motor_zero: bool,
        job_token: u64,
        check_begin: Option<CalibBeginCheck>,
    },
    CalibrationRevert {
        before: Box<CalibrationCandidate>,
    },
    CalibrationClearLoadError,
    ForceReconcile(ConfigDomain),
    SetJointZeroOffsets {
        offsets: Vec<f32>,
        precomputed_floor: Option<(f32, &'static str)>,
        auth: Option<ConfigAuthContext>,
        expected_stop_gen: Option<u64>,
        deadline: Option<Instant>,
        job_token: Option<u64>,
    },
    SetMotorZero {
        joint: usize,
        auth: Option<ConfigAuthContext>,
        expected_stop_gen: Option<u64>,
        deadline: Option<Instant>,
        job_token: Option<u64>,
    },
    SampleTableCapture,
    GravityComp(GravityCompOp),
    InspectHold(ConfigDomain),
    /// Zero-alloc check on live owner whether `episode_id` may be deleted (not referenced by
    /// saved sequences, running sequence pins, or an active `Sequences` save reservation).
    CheckEpisodeDeleteAllowed(HolderLabel),
}

/// Reply produced on the owner and read off-thread from `ConfigExchange`.
pub enum ConfigReply {
    Reserved(Result<Reservation, ConfigError>),
    AuthorityVerified(Result<(), ConfigError>),
    Persisted(Result<Reservation, ConfigError>),
    Activated(Result<ConfigActivated, ConfigError>),
    Released(Released),
    CalibrationBusySet(Result<CalibBeginSnapshot, ConfigError>),
    CalibrationReverted {
        calibration_revision: u64,
        workspace_revision: u64,
    },
    CalibrationLoadErrorCleared,
    ReconcilingForced,
    JointZeroOffsetsSet(Result<(), ConfigError>),
    MotorZeroSet(Result<(), ConfigError>),
    TableCaptureSampled(Result<([f32; 8], usize), ConfigError>),
    GravityCompDone(Result<GravityCompReply, ConfigError>),
    HoldInspected {
        config_inhibited: bool,
        state: Option<crate::rt_core::config_iface::HoldState>,
    },
    EpisodeDeleteAllowed(bool),
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
    pub steps: Option<Vec<Step>>,
    pub grasp_setup: Option<GraspSetup>,
    pub string_a: Option<String>,
    pub string_b: Option<String>,
    pub calibration_file: Option<CalibrationFile>,
    pub arm_model: Option<ArmModel>,
    pub pose_tcps: Option<Vec<(String, Option<PoseTcp>)>>,
    pub calib_candidate: Option<Box<CalibrationCandidate>>,
    pub joint_zero_commit_candidate: Option<Box<JointZeroCommitCandidate>>,
    pub gravity_comp_policy: Option<crate::model::GravityCompPolicy>,
    pub unused_candidate: Option<ConfigCandidate>,
    pub gripper_baseline: Option<crate::gripper_baseline::FreeTorqueBaseline>,
    pub offsets: Option<Vec<f32>>,
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
        } => ConfigReply::AuthorityVerified(install_verify_authority(
            core,
            ctx,
            domain,
            grant,
            &proposal,
            &auth,
            expected_workspace_revision,
            expected_poses_revision,
        )),
        ConfigRequest::Persisted(p) => ConfigReply::Persisted(install_persisted_config(core, p)),
        ConfigRequest::Activate(a) => ConfigReply::Activated(install_activate_config(core, ctx, a, &mut ex.retired)),
        ConfigRequest::Release(r) => {
            let domain = r.reservation.domain();
            let released = ConfigAuthority::release(core, r.reservation);
            if domain == ConfigDomain::Sequences && released == Released::Cleared {
                ex.retired.reserved_sequence_pins = core.sequences.reserved_pins.take();
            }
            ConfigReply::Released(released)
        }
        ConfigRequest::SetCalibrationBusy {
            busy,
            motor_zero,
            job_token,
            check_begin,
        } => ConfigReply::CalibrationBusySet(install_set_calibration_busy(
            core,
            busy,
            motor_zero,
            job_token,
            check_begin,
        )),
        ConfigRequest::CalibrationRevert { before } => {
            let (calibration_revision, workspace_revision) =
                install_calibration_revert(core, before, &mut ex.retired);
            ConfigReply::CalibrationReverted {
                calibration_revision,
                workspace_revision,
            }
        }
        ConfigRequest::CalibrationClearLoadError => {
            ex.retired.string_a = core.calibration_load_error.take();
            ConfigReply::CalibrationLoadErrorCleared
        }
        ConfigRequest::ForceReconcile(domain) => {
            let active_rev = ConfigAuthority::revision(core, domain);
            core.core.config_mut().force_reconcile(domain, active_rev);
            ConfigReply::ReconcilingForced
        }
        ConfigRequest::SetJointZeroOffsets {
            offsets,
            precomputed_floor,
            auth,
            expected_stop_gen,
            deadline,
            job_token,
        } => ConfigReply::JointZeroOffsetsSet(install_set_joint_zero_offsets(
            core,
            ctx,
            offsets,
            precomputed_floor,
            auth,
            expected_stop_gen,
            deadline,
            job_token,
            &mut ex.retired,
        )),
        ConfigRequest::SetMotorZero {
            joint,
            auth,
            expected_stop_gen,
            deadline,
            job_token,
        } => ConfigReply::MotorZeroSet(install_set_motor_zero(
            core,
            ctx,
            joint,
            auth,
            expected_stop_gen,
            deadline,
            job_token,
        )),
        ConfigRequest::SampleTableCapture => ConfigReply::TableCaptureSampled(install_sample_table_capture(core)),
        ConfigRequest::GravityComp(op) => {
            ConfigReply::GravityCompDone(install_gravity_comp(core, ctx, op, &mut ex.retired))
        }
        ConfigRequest::InspectHold(domain) => ConfigReply::HoldInspected {
            config_inhibited: core.core.motion_inhibited_by_config(),
            state: core.core.config_held(domain).map(|h| h.state),
        },
        ConfigRequest::CheckEpisodeDeleteAllowed(episode_id) => {
            ConfigReply::EpisodeDeleteAllowed(!core.sequences.is_episode_in_use(episode_id.as_str()))
        }
    };
    ex.reply = Some(reply);
}

fn check_kinematics_armed_static(core: &SupervisorInner) -> Result<(), ConfigError> {
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

fn run_reserve_precheck(
    core: &SupervisorInner,
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
            if !deleting_pose_id.is_empty() && core.sequences.is_pose_in_use(deleting_pose_id.as_str()) {
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
                    if core
                        .sequences
                        .pins
                        .as_ref()
                        .is_some_and(|p| p.sequence_id == del_id.as_str())
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
            kin_changes,
            allow_during_calibration_busy,
        } => {
            if !allow_during_calibration_busy && core.calibration_busy {
                return Err(ConfigError::Busy("a camera calibration apply or restore is running"));
            }
            if kin_changes {
                check_kinematics_armed_static(core)?;
            }
            Ok(())
        }
        ReservePrecheck::JointZeroCommit {
            require_zero_idle,
            job_token,
            expected_gripper_calib_revision,
        } => {
            if let Some(tok) = job_token {
                if !core.calibration_busy || core.calibration_busy_token != Some(tok) {
                    return Err(ConfigError::Busy(
                        "motor zero job no longer holds calibration lock",
                    ));
                }
            }
            if let Some(exp_grip) = expected_gripper_calib_revision {
                if core.gripper_calib_revision != exp_grip {
                    return Err(ConfigError::PlanStale(
                        "gripper calibration changed while preparing joint zero commit",
                    ));
                }
            }
            if require_zero_idle {
                if core.rt_executor_owned && core.driver.is_some() {
                    return Err(ConfigError::ExecutorCapabilityUnsupported(
                        "software joint zero with attached hardware driver is unsupported on the RT executor",
                    ));
                }
                if *core.safety.state() != crate::safety::SafetyState::Idle
                    || core.is_torqued
                    || core.disable_pending
                    || core.active_trajectory.is_some()
                {
                    return Err(ConfigError::CalibPrecondition {
                        status: 409,
                        code: "MOTOR_ZERO_FAILED",
                        message: "Arm must be Idle with torque off and no active trajectory to set joint zero offsets",
                    });
                }
            }
            Ok(())
        }
    }
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
    if let Err(err) = run_reserve_precheck(core, precheck, retired) {
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
) -> Result<(), ConfigError> {
    if core
        .core
        .config_held(domain)
        .is_none_or(|h| h.grant != Some(grant))
    {
        return Err(ConfigError::NotHeld);
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
                "KINEMATICS_ARMED" => ConfigError::CalibPrecondition {
                    status: 409,
                    code: "KINEMATICS_ARMED",
                    message: "the kinematic correction can change only with torque off and no action running",
                },
                "MOTOR_ZERO_FAILED" => ConfigError::CalibPrecondition {
                    status: 409,
                    code: "MOTOR_ZERO_FAILED",
                    message: "Arm must be Idle with torque off and no active trajectory to set joint zero offsets",
                },
                "EXECUTOR_CAPABILITY_UNSUPPORTED" => ConfigError::ExecutorCapabilityUnsupported(
                    "software joint zero with attached hardware driver is unsupported on the RT executor",
                ),
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
    let domain_ok: Result<(), &'static str> = match &candidate {
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
        ConfigCandidate::Calibration(cand) => {
            if cand.require_kinematics_disarmed && check_kinematics_armed_static(core).is_err() {
                Err("KINEMATICS_ARMED")
            } else {
                Ok(())
            }
        }
        ConfigCandidate::JointZeroCommit(cand) => {
            if let Some(tok) = cand.job_token {
                if !core.calibration_busy || core.calibration_busy_token != Some(tok) {
                    return Err(fail_activation(core, reservation, "CONFIG_BUSY", candidate, retired));
                }
            }
            if let Some(exp_grip) = cand.expected_gripper_calib_revision {
                if core.gripper_calib_revision != exp_grip {
                    return Err(fail_activation(core, reservation, "PLAN_STALE", candidate, retired));
                }
            }
            if cand.new_offsets.is_some() && core.rt_executor_owned && core.driver.is_some() {
                Err("EXECUTOR_CAPABILITY_UNSUPPORTED")
            } else if cand.require_zero_idle
                && (*core.safety.state() != crate::safety::SafetyState::Idle
                    || core.is_torqued
                    || core.disable_pending
                    || core.active_trajectory.is_some())
            {
                Err("MOTOR_ZERO_FAILED")
            } else if cand
                .new_offsets
                .as_ref()
                .is_some_and(|o| o.len() != core.config.dof || o.iter().any(|v| !v.is_finite()))
            {
                Err("MOTOR_ZERO_FAILED")
            } else {
                Ok(())
            }
        }
        ConfigCandidate::Poses { .. } | ConfigCandidate::Sequences { .. } => Ok(()),
    };
    if let Err(code) = domain_ok {
        return Err(fail_activation(core, reservation, code, candidate, retired));
    }

    // Infallible zero-alloc, zero-free in-place swap into SupervisorInner.
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
        ConfigCandidate::Calibration(cand) => {
            apply_calibration_candidate_in_place(core, cand, true, retired);
        }
        ConfigCandidate::JointZeroCommit(cand) => {
            apply_joint_zero_commit_in_place(core, cand, retired);
        }
    }

    if durability == Durability::Durable {
        let _ = core.core.config_mut().confirm_durable(&reservation);
    }

    match ConfigAuthority::activate(core, reservation) {
        Activated::Active { revision, durability } => Ok(ConfigActivated {
            revision,
            durability,
            workspace_revision: core.workspace_revision,
            poses_revision: core.poses.revision,
            gripper_calib_revision: core.gripper_calib_revision,
            epoch_id: core.safety.epoch_id(),
            is_torqued: core.is_torqued,
            safety_state: SafetyStateTag::from_state(core.safety.state()),
            gravity_comp_active: core.gravity_comp_active(),
        }),
        Activated::Refused { code, reservation } => {
            let _ = ConfigAuthority::release(core, reservation);
            Err(ConfigError::Busy(code))
        }
        Activated::Reconciling { code } => Err(ConfigError::Reconciling(code)),
    }
}

fn apply_calibration_candidate_in_place(
    core: &mut SupervisorInner,
    mut cand: Box<CalibrationCandidate>,
    advance_calib_revision: bool,
    retired: &mut ConfigRetiredPayloads,
) {
    let mut workspace_bumped = false;
    if cand.kin_changed {
        if let Some(new_model) = cand.arm_model.take() {
            retired.arm_model = Some(std::mem::replace(&mut core.config.arm_model, new_model));
            workspace_bumped = true;
        }
        if let Some(tcps) = cand.pose_tcps.take() {
            for (pose, (id, tcp)) in core.poses.list.poses.iter_mut().zip(tcps.iter()) {
                if pose.id == *id {
                    pose.tcp = tcp.clone();
                }
            }
            retired.pose_tcps = Some(tcps);
        }
    }
    if core.config.camera_mount != cand.camera_mount {
        workspace_bumped = true;
    }
    core.config.camera_mount = cand.camera_mount.clone();
    let tilt = f64::from(core.config.camera_mount.tilt_deg);
    if let Some(g) = core.config.grasp.as_mut() {
        g.camera_tilt_deg = tilt;
    }
    retired.calibration_file = std::mem::replace(&mut core.calibration, cand.calibration.take());
    #[cfg(feature = "policy-runtime")]
    {
        let new_digest = std::mem::take(&mut cand.calibration_digest);
        retired.string_a = Some(std::mem::replace(&mut core.calibration_digest, new_digest));
    }
    if let Some((floor, source)) = cand.effective_floor {
        core.floor_source = source;
        if let Some(ws) = core.config.workspace.as_mut() {
            if ws.floor_z_m != floor {
                ws.floor_z_m = floor;
                workspace_bumped = true;
            }
        }
    }
    if workspace_bumped {
        core.workspace_revision += 1;
        core.retire_servo_ws_min_depths();
    }
    if let Some(bufs) = cand.ws_eval_bufs.take() {
        retired.eval_bufs = Some(std::mem::replace(&mut core.ws_eval_bufs, bufs));
    }
    if advance_calib_revision {
        core.calibration_revision += 1;
    }
    // Keep the Box<CalibrationCandidate> shell alive until off-thread drop (0 dealloc on RT).
    retired.calib_candidate = Some(cand);
}

fn install_set_calibration_busy(
    core: &mut SupervisorInner,
    busy: bool,
    motor_zero: bool,
    job_token: u64,
    check_begin: Option<CalibBeginCheck>,
) -> Result<CalibBeginSnapshot, ConfigError> {
    if busy {
        if let Some(chk) = check_begin {
            if core.calibration_busy {
                return Err(ConfigError::Busy("a calibration apply or restore is running"));
            }
            if let Some(reason) = core.workspace_change_blocker_static() {
                return Err(ConfigError::MotionBusy {
                    subject: "the camera calibration",
                    reason,
                });
            }
            if chk.check_grasp
                && (core.config.grasp.is_none()
                    || !core.config.has_cartesian_with_gripper()
                    || core.config.kinematics.is_none()
                    || core.config.gripper_policy.is_none())
            {
                return Err(ConfigError::Unsupported("This profile has no grasp setup"));
            }
            if chk.check_kinematics_armed {
                check_kinematics_armed_static(core)?;
            }
        } else if core.calibration_busy {
            return Err(ConfigError::Busy("a calibration apply, restore or zero is running"));
        }
        core.calibration_busy = true;
        core.calibration_busy_token = Some(job_token);
        if motor_zero {
            core.motor_zero_busy = true;
        }
    } else if job_token == 0 || core.calibration_busy_token.is_none() || core.calibration_busy_token == Some(job_token) {
        core.calibration_busy = false;
        core.calibration_busy_token = None;
        core.motor_zero_busy = false;
    }
    Ok(CalibBeginSnapshot {
        is_torqued: core.is_torqued,
        epoch_id: core.safety.epoch_id(),
    })
}

fn install_calibration_revert(
    core: &mut SupervisorInner,
    before: Box<CalibrationCandidate>,
    retired: &mut ConfigRetiredPayloads,
) -> (u64, u64) {
    apply_calibration_candidate_in_place(core, before, true, retired);
    (core.calibration_revision, core.workspace_revision)
}

fn check_motor_zero_authority(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    auth: Option<ConfigAuthContext>,
    expected_stop_gen: Option<u64>,
    deadline: Option<Instant>,
    job_token: Option<u64>,
) -> Result<(), ConfigError> {
    if let Some(tok) = job_token {
        if !core.calibration_busy || core.calibration_busy_token != Some(tok) {
            return Err(ConfigError::Busy("motor zero job no longer holds calibration lock"));
        }
    }
    if let Some(dl) = deadline {
        if ctx.now > dl {
            return Err(ConfigError::DeadlineExpired);
        }
    }
    if let Some(a) = auth {
        if a.admission.connection_id != 0 && core.is_connection_closed(a.admission.connection_id) {
            return Err(ConfigError::ConnectionClosed);
        }
        if core.estop_request_unapplied() || core.safety.epoch_id() != a.admission.estop_epoch {
            return Err(ConfigError::EpochMismatch {
                current_epoch: core.safety.epoch_id(),
            });
        }
        if core.urgent_stop_unapplied() {
            return Err(ConfigError::Stopped);
        }
        if let Some(exp_stop) = expected_stop_gen {
            if ctx.stop_generation != exp_stop {
                return Err(ConfigError::Stopped);
            }
        }
        if let Some((conn, gen)) = a.lease_holder {
            if gen == 0 || !core.validate_lease_at(conn.as_str(), gen, ctx.now) {
                return Err(ConfigError::LeaseLost);
            }
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn install_set_joint_zero_offsets(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    mut offsets: Vec<f32>,
    precomputed_floor: Option<(f32, &'static str)>,
    auth: Option<ConfigAuthContext>,
    expected_stop_gen: Option<u64>,
    deadline: Option<Instant>,
    job_token: Option<u64>,
    retired: &mut ConfigRetiredPayloads,
) -> Result<(), ConfigError> {
    let res = (|| {
        check_motor_zero_authority(core, ctx, auth, expected_stop_gen, deadline, job_token)?;
        if core.rt_executor_owned && core.driver.is_some() {
            return Err(ConfigError::ExecutorCapabilityUnsupported(
                "software joint zero with attached hardware driver is unsupported on the RT executor",
            ));
        }
        if *core.safety.state() != crate::safety::SafetyState::Idle
            || core.is_torqued
            || core.disable_pending
            || core.active_trajectory.is_some()
        {
            return Err(ConfigError::CalibPrecondition {
                status: 409,
                code: "MOTOR_ZERO_FAILED",
                message: "Arm must be Idle with torque off and no active trajectory to set joint zero offsets",
            });
        }
        match core.set_joint_zero_offsets_in_place(&mut offsets, precomputed_floor) {
            Ok(retired_base) => {
                retired.gripper_baseline = retired_base;
                Ok(())
            }
            Err(msg) => Err(ConfigError::CalibPrecondition {
                status: 409,
                code: "MOTOR_ZERO_FAILED",
                message: msg,
            }),
        }
    })();
    retired.offsets = Some(offsets);
    res
}

fn install_set_motor_zero(
    core: &mut SupervisorInner,
    ctx: &CommitCtx,
    joint: usize,
    auth: Option<ConfigAuthContext>,
    expected_stop_gen: Option<u64>,
    deadline: Option<Instant>,
    job_token: Option<u64>,
) -> Result<(), ConfigError> {
    check_motor_zero_authority(core, ctx, auth, expected_stop_gen, deadline, job_token)?;
    if core.driver.is_some() {
        return Err(ConfigError::ExecutorCapabilityUnsupported(
            "hardware motor zero requires synchronous CAN register I/O unsupported on the RT executor",
        ));
    }
    if *core.safety.state() != crate::safety::SafetyState::Idle
        || core.is_torqued
        || core.disable_pending
    {
        return Err(ConfigError::CalibPrecondition {
            status: 409,
            code: "MOTOR_ZERO_FAILED",
            message: "Arm must be Idle with torque off to set motor zero",
        });
    }
    core.set_motor_zero_in_place(joint).map_err(|msg| ConfigError::CalibPrecondition {
        status: 409,
        code: "MOTOR_ZERO_FAILED",
        message: msg,
    })
}

fn apply_joint_zero_commit_in_place(
    core: &mut SupervisorInner,
    mut cand: Box<JointZeroCommitCandidate>,
    retired: &mut ConfigRetiredPayloads,
) {
    let start_calib_rev = core.calibration_revision;
    let mut workspace_bumped = false;
    if let Some(mut new_offsets) = cand.new_offsets.take() {
        // Offsets length and finiteness were validated in install_activate_config before swapping.
        if let Ok(retired_base) = core.set_joint_zero_offsets_in_place(&mut new_offsets, cand.effective_floor) {
            if let Some(gb) = retired_base {
                retired.gripper_baseline = Some(gb);
            }
        }
        retired.offsets = Some(new_offsets);
    }
    if let Some(uncorrected) = cand.arm_model_uncorrected.take() {
        if uncorrected.correction() != core.config.arm_model.correction() {
            retired.arm_model = Some(std::mem::replace(&mut core.config.arm_model, uncorrected));
            workspace_bumped = true;
            if let Some(tcps) = cand.pose_tcps.take() {
                for (pose, (id, tcp)) in core.poses.list.poses.iter_mut().zip(tcps.iter()) {
                    if pose.id == *id {
                        pose.tcp = tcp.clone();
                    }
                }
                retired.pose_tcps = Some(tcps);
            }
        } else {
            retired.arm_model = Some(uncorrected);
        }
    }
    if cand.file_changes {
        retired.calibration_file = std::mem::replace(&mut core.calibration, cand.calibration.take());
        #[cfg(feature = "policy-runtime")]
        if let Some(new_digest) = cand.calibration_digest.take() {
            retired.string_a = Some(std::mem::replace(&mut core.calibration_digest, new_digest));
        }
    }
    if let Some((floor, source)) = cand.effective_floor {
        core.floor_source = source;
        if let Some(ws) = core.config.workspace.as_mut() {
            if ws.floor_z_m != floor {
                ws.floor_z_m = floor;
                workspace_bumped = true;
            }
        }
    }
    if workspace_bumped {
        core.workspace_revision += 1;
        core.retire_servo_ws_min_depths();
    }
    if let Some(bufs) = cand.ws_eval_bufs.take() {
        retired.eval_bufs = Some(std::mem::replace(&mut core.ws_eval_bufs, bufs));
    }
    if let Some(next_poses) = cand.next_poses.take() {
        retired.pose_list = Some(std::mem::replace(&mut core.poses.list, next_poses));
        core.poses.revision += 1;
    }
    if cand.reset_gripper_baseline {
        if let Some(gb) = core.gripper_baseline.take() {
            retired.gripper_baseline = Some(gb);
        }
        core.gripper_baseline_pending.clear();
        core.gripper_baseline_dirty = false;
        core.gripper_baseline_generation += 1;
    }
    if let Some(stale) = cand.gripper_stale.take() {
        retired.gripper_stale = core.gripper_calib_stale.replace(stale);
        core.gripper_calib_revision += 1;
    }
    core.calibration_revision = start_calib_rev + 1;
    retired.joint_zero_commit_candidate = Some(cand);
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
            if core.rt_executor_owned && core.driver.is_some() {
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
            if core.rt_executor_owned && core.driver.is_some() {
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
    pub fail_dir_sync_once: Option<Arc<AtomicBool>>,
    pub fail_persisted_once: Option<Arc<AtomicBool>>,
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
}

impl ReservationGuard {
    pub fn new(sup: Arc<ArmSupervisor>, reservation: Reservation) -> Self {
        Self {
            sup,
            reservation: Some(reservation),
            renamed: false,
        }
    }

    pub fn grant(&self) -> Option<GrantId> {
        self.reservation.as_ref().map(Reservation::grant)
    }

    pub fn mark_renamed(&mut self) {
        self.renamed = true;
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
        let sup = Arc::clone(&self.sup);
        let renamed = self.renamed;
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
                        if let Ok(ConfigReply::Persisted(Ok(r))) = res {
                            let _ = sup
                                .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                                    ReleaseConfig { reservation: r },
                                ))
                                .await;
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

/// One atomic file mutation inside a [`ConfigWriteOp::Batch`].
pub enum FileWriteOp {
    /// Stage `PendingWrite` off-thread (write + file `sync_all()`), then atomically rename into place.
    Replace(PendingWrite),
    /// Remove an existing config file on disk if present.
    Remove(PathBuf),
}

/// Filesystem operation performed during a unified [`run_config_commit_detailed`] transaction.
///
/// Every variant runs its staging, file fsync, rename/unlink, and parent-directory fsync
/// strictly AFTER [`ReserveConfig`] and strictly BEFORE [`ActivateConfig`].
pub enum ConfigWriteOp {
    /// Memory-only (volatile) configuration update with no disk file.
    Volatile,
    /// Stage `PendingWrite` off-thread after reservation, then rename + dir-sync before activation.
    StageAndReplace(PendingWrite),
    /// Remove an existing config file on disk (e.g., calibration restore to uncalibrated backup),
    /// using the exact same cancellation-safe reservation + pre-unlink live authority + reconciliation protocol.
    Remove(PathBuf),
    /// Multi-file atomic commit executed under one reservation (e.g., joint zero writing
    /// `calibration.json` + stale `poses.json` + stale `gripper_calib.json` together).
    /// All replacements are staged + file-fsynced before any rename; the first entry is renamed
    /// first so a failure on it leaves all other files untouched and releases cleanly.
    Batch(Vec<FileWriteOp>),
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

/// Typed failure outcome from [`run_config_commit_detailed`] preserving whether disk publication
/// happened and whether the owner entered `HoldState::Reconciling`.
#[derive(Debug)]
pub struct ConfigCommitError {
    pub error: ArmError,
    pub disk_published: bool,
    pub reconciling: bool,
}

impl ConfigCommitError {
    fn pre_publish(error: ArmError) -> Self {
        Self {
            error,
            disk_published: false,
            reconciling: false,
        }
    }

    fn post_publish_reconciling(error: ArmError) -> Self {
        Self {
            error,
            disk_published: true,
            reconciling: true,
        }
    }
}

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
    let (done_tx, done_rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        let res = run_config_commit_worker(sup_w, plan, auth_check, update_published).await;
        let _ = done_tx.send(res);
    });
    match done_rx.await {
        Ok(res) => res,
        Err(_) => Err(ConfigCommitError::pre_publish(ArmError::Internal(
            "configuration commit worker aborted".into(),
        ))),
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

async fn run_config_commit_worker<FAuth, FPub>(
    sup: Arc<ArmSupervisor>,
    plan: ConfigCommitPlan,
    auth_check: FAuth,
    update_published: FPub,
) -> Result<ConfigCommitOutcome, ConfigCommitError>
where
    FAuth: Fn() -> Result<(), (&'static str, ArmError)>,
    FPub: FnOnce(&mut PublishedConfig, ConfigActivated),
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

    let file_ops: Vec<FileWriteOp> = match write {
        ConfigWriteOp::Volatile => Vec::new(),
        ConfigWriteOp::StageAndReplace(w) => vec![FileWriteOp::Replace(w)],
        ConfigWriteOp::Remove(path) => vec![FileWriteOp::Remove(path)],
        ConfigWriteOp::Batch(ops) => ops,
    };

    if let Some(h) = hooks.before_reserve.as_ref() {
        h();
    }

    if let Err((_, err)) = auth_check() {
        return Err(ConfigCommitError::pre_publish(err));
    }

    // 1. Reserve domain on the single live owner BEFORE any staging or file mutation.
    let reservation = match sup
        .dispatch_config_request_owned_reliable(ConfigRequest::Reserve(ReserveConfig {
            proposal,
            auth,
            precheck,
            expected_workspace_revision,
            expected_poses_revision,
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

    let grant = reservation.grant();
    let domain = reservation.domain();
    let mut guard = ReservationGuard::new(Arc::clone(&sup), reservation);

    let release_pre_rename = |sup: Arc<ArmSupervisor>, mut guard: ReservationGuard| async move {
        if let Some(r) = guard.take() {
            let _ = sup
                .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                    reservation: r,
                }))
                .await;
        }
    };

    // 1b. For `Sequences` saves referencing replay episodes, atomically verify existence and pin
    //     them in `EpisodeStore` right under the granted `Sequences` reservation (before staging,
    //     hooks, or rename) so concurrent episode deletion is refused with 409 IN_USE and an
    //     already-deleted episode aborts cleanly with 422 SEQUENCE_REF_MISSING.
    let mut episode_pin_guard: Option<crate::recording::store::EpisodePinGuard> =
        if !replay_episode_refs.is_empty() {
            let store = sup.episode_store().await;
            match store.pin_episodes_for_sequence(&replay_episode_refs) {
                Ok(pg) => Some(pg),
                Err((step, ep_id)) => {
                    release_pre_rename(Arc::clone(&sup), guard).await;
                    return Err(ConfigCommitError::pre_publish(ArmError::detailed(
                        422,
                        "SEQUENCE_REF_MISSING",
                        format!("step {step}: no episode '{ep_id}'"),
                        json!({"step": step}),
                    )));
                }
            }
        } else {
            None
        };

    // 2. Off-thread stage + file fsync (`StagedWrite::stage`) strictly AFTER reserve.
    enum StagedOp {
        Replace(StagedWrite),
        Remove(PathBuf),
    }
    let abort_staged = |ops: Vec<StagedOp>| {
        for op in ops {
            if let StagedOp::Replace(s) = op {
                s.abort();
            }
        }
    };

    let staged_ops: Vec<StagedOp> = if file_ops.is_empty() {
        Vec::new()
    } else {
        let stage_res = tokio::task::spawn_blocking(move || -> Result<Vec<StagedOp>, String> {
            let mut out = Vec::with_capacity(file_ops.len());
            for op in file_ops {
                match op {
                    FileWriteOp::Replace(w) => match StagedWrite::stage(w.path, w.text) {
                        Ok(s) => out.push(StagedOp::Replace(s)),
                        Err(e) => {
                            for prev in out {
                                if let StagedOp::Replace(s) = prev {
                                    s.abort();
                                }
                            }
                            return Err(e);
                        }
                    },
                    FileWriteOp::Remove(p) => out.push(StagedOp::Remove(p)),
                }
            }
            Ok(out)
        })
        .await
        .map_err(|e| format!("stage task failed: {e}"))
        .and_then(|r| r);

        match stage_res {
            Ok(ops) => ops,
            Err(e) => {
                release_pre_rename(Arc::clone(&sup), guard).await;
                tracing::warn!("State file not staged: {e}");
                return Err(ConfigCommitError::pre_publish(ArmError::Rejected {
                    status: 500,
                    code: "PERSIST_FAILED".into(),
                    message: format!("Failed to persist configuration: {e}"),
                }));
            }
        }
    };

    if let Some(h) = hooks.after_reserve_before_rename.as_ref() {
        h();
    }

    // 3. Fresh pre-rename authority recheck (off-thread auth + live owner VerifyAuthority).
    if let Err((_, err)) = auth_check() {
        abort_staged(staged_ops);
        release_pre_rename(Arc::clone(&sup), guard).await;
        return Err(ConfigCommitError::pre_publish(err));
    }
    match sup
        .dispatch_config_request_owned_reliable(ConfigRequest::VerifyAuthority {
            domain,
            grant,
            proposal,
            auth,
            expected_workspace_revision,
            expected_poses_revision,
        })
        .await
    {
        Ok(ConfigReply::AuthorityVerified(Ok(()))) => {}
        Ok(ConfigReply::AuthorityVerified(Err(e))) => {
            abort_staged(staged_ops);
            release_pre_rename(Arc::clone(&sup), guard).await;
            return Err(ConfigCommitError::pre_publish(e.to_arm_error()));
        }
        Err(e) => {
            abort_staged(staged_ops);
            release_pre_rename(Arc::clone(&sup), guard).await;
            return Err(ConfigCommitError::pre_publish(e.to_arm_error()));
        }
        _ => {
            abort_staged(staged_ops);
            release_pre_rename(Arc::clone(&sup), guard).await;
            return Err(ConfigCommitError::pre_publish(ArmError::Internal(
                "unexpected authority verification reply".into(),
            )));
        }
    }

    // 4. Rename / unlink files off-thread, then record Persisted(DurableUnconfirmed),
    //    then parent-directory fsync strictly BEFORE activation.
    let mut disk_published = false;
    if !staged_ops.is_empty() {
        let sim_rename_fail = hooks
            .fail_rename_once
            .as_ref()
            .is_some_and(|f| f.swap(false, Ordering::AcqRel));

        let rename_outcome = tokio::task::spawn_blocking(move || {
            let mut dir_syncs: Vec<crate::persist::PendingDirSync> = Vec::with_capacity(staged_ops.len());
            let mut published_any = false;
            let mut iter = staged_ops.into_iter();
            while let Some(op) = iter.next() {
                match op {
                    StagedOp::Replace(staged) => {
                        if sim_rename_fail {
                            staged.abort();
                            for rem in iter {
                                if let StagedOp::Replace(s) = rem {
                                    s.abort();
                                }
                            }
                            return (published_any, dir_syncs, Err("simulated rename failure".to_string()));
                        }
                        match staged.rename_only() {
                            Ok(ds) => {
                                published_any = true;
                                dir_syncs.push(ds);
                            }
                            Err(e) => {
                                for rem in iter {
                                    if let StagedOp::Replace(s) = rem {
                                        s.abort();
                                    }
                                }
                                return (published_any, dir_syncs, Err(e));
                            }
                        }
                    }
                    StagedOp::Remove(path) => match std::fs::remove_file(&path) {
                        Ok(()) => {
                            published_any = true;
                            dir_syncs.push(crate::persist::PendingDirSync::for_path(&path));
                        }
                        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                        Err(e) => {
                            for rem in iter {
                                if let StagedOp::Replace(s) = rem {
                                    s.abort();
                                }
                            }
                            return (
                                published_any,
                                dir_syncs,
                                Err(format!("remove {}: {e}", path.display())),
                            );
                        }
                    },
                }
            }
            (published_any, dir_syncs, Ok(()))
        })
        .await;

        let (published_any, dir_syncs, rename_res) = match rename_outcome {
            Ok(tuple) => tuple,
            Err(e) => {
                // JoinError after spawn_blocking started: conservatively treat as post-publish
                // reconciling because a rename may have completed before the panic/abort.
                if let Some(pg) = episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
                guard.mark_renamed();
                if let Some(r) = guard.take() {
                    if let Ok(ConfigReply::Persisted(Ok(pr))) = sup
                        .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(
                            PersistedConfig {
                                reservation: r,
                                durability: Durability::DurableUnconfirmed,
                            },
                        ))
                        .await
                    {
                        let _ = sup
                            .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                                ReleaseConfig { reservation: pr },
                            ))
                            .await;
                    } else {
                        let _ = sup
                            .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                            .await;
                    }
                }
                return Err(ConfigCommitError::post_publish_reconciling(ArmError::Rejected {
                    status: 500,
                    code: "PERSIST_FAILED".into(),
                    message: format!("Failed to persist configuration: rename task failed: {e}"),
                }));
            }
        };

        if published_any {
            guard.mark_renamed();
        }

        if let Err(e) = rename_res {
            tracing::warn!("State file not published: {e}");
            if !published_any {
                release_pre_rename(Arc::clone(&sup), guard).await;
                return Err(ConfigCommitError::pre_publish(ArmError::Rejected {
                    status: 500,
                    code: "PERSIST_FAILED".into(),
                    message: format!("Failed to persist configuration: {e}"),
                }));
            }
            // At least one file was already renamed/removed before a later file in the batch failed:
            // disk and live state may diverge -> enter Reconciling and keep motion inhibited.
            if let Some(pg) = episode_pin_guard.take() {
                pg.keep_pinned_for_reconcile();
            }
            if let Some(r) = guard.take() {
                if let Ok(ConfigReply::Persisted(Ok(pr))) = sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(
                        PersistedConfig {
                            reservation: r,
                            durability: Durability::DurableUnconfirmed,
                        },
                    ))
                    .await
                {
                    let _ = sup
                        .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                            ReleaseConfig { reservation: pr },
                        ))
                        .await;
                } else {
                    let _ = sup
                        .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                        .await;
                }
            }
            return Err(ConfigCommitError::post_publish_reconciling(ArmError::Rejected {
                status: 500,
                code: "PERSIST_FAILED".into(),
                message: format!("Failed to persist configuration batch after partial rename: {e}; reconciliation required"),
            }));
        }

        if published_any {
            disk_published = true;
            let sim_persisted_fail = hooks
                .fail_persisted_once
                .as_ref()
                .is_some_and(|f| f.swap(false, Ordering::AcqRel));
            let r = guard.take().expect("reservation held by guard");
            if sim_persisted_fail {
                if let Some(pg) = episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
                if let Ok(ConfigReply::Persisted(Ok(r))) = sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(
                        PersistedConfig {
                            reservation: r,
                            durability: Durability::DurableUnconfirmed,
                        },
                    ))
                    .await
                {
                    let _ = sup
                        .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                            ReleaseConfig { reservation: r },
                        ))
                        .await;
                }
                return Err(ConfigCommitError::post_publish_reconciling(
                    ConfigError::Reconciling("SIMULATED_PERSISTED_ERR").to_arm_error(),
                ));
            }
            let persisted_r = match sup
                .dispatch_config_request_owned_reliable(ConfigRequest::Persisted(PersistedConfig {
                    reservation: r,
                    durability: Durability::DurableUnconfirmed,
                }))
                .await
            {
                Ok(ConfigReply::Persisted(Ok(r))) => r,
                Ok(ConfigReply::Persisted(Err(e))) => {
                    if let Some(pg) = episode_pin_guard.take() {
                        pg.keep_pinned_for_reconcile();
                    }
                    return Err(ConfigCommitError::post_publish_reconciling(e.to_arm_error()));
                }
                Err(e) => {
                    if let Some(pg) = episode_pin_guard.take() {
                        pg.keep_pinned_for_reconcile();
                    }
                    return Err(ConfigCommitError::post_publish_reconciling(e.to_arm_error()));
                }
                _ => {
                    if let Some(pg) = episode_pin_guard.take() {
                        pg.keep_pinned_for_reconcile();
                    }
                    return Err(ConfigCommitError::post_publish_reconciling(
                        ArmError::Internal("unexpected config persisted reply".into()),
                    ));
                }
            };
            guard = ReservationGuard::new(Arc::clone(&sup), persisted_r);
            guard.mark_renamed();

            // 5. Parent-directory fsync strictly AFTER rename and strictly BEFORE activation.
            //    A failure here MUST NOT activate the new policy; it enters Reconciling and keeps
            //    motion inhibited until reconciliation succeeds.
            let sim_dir_sync_fail = hooks
                .fail_dir_sync_once
                .as_ref()
                .is_some_and(|f| f.swap(false, Ordering::AcqRel));
            let dir_sync_res: Result<(), String> = if sim_dir_sync_fail {
                Err("simulated parent dir fsync failure".into())
            } else {
                tokio::task::spawn_blocking(move || -> Result<(), String> {
                    let mut synced_dirs: Vec<PathBuf> = Vec::new();
                    for ds in dir_syncs {
                        let dir = ds.dir().to_path_buf();
                        if synced_dirs.iter().any(|d| d == &dir) {
                            continue;
                        }
                        match ds.sync_parent_dir() {
                            PublishOutcome::Published => synced_dirs.push(dir),
                            PublishOutcome::PublishedDurabilityUnconfirmed(w) => return Err(w),
                        }
                    }
                    Ok(())
                })
                .await
                .map_err(|e| e.to_string())
                .and_then(|r| r)
            };

            if let Err(warn) = dir_sync_res {
                if let Some(pg) = episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
                let r = guard.take().expect("persisted reservation held by guard");
                let rel = sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                        reservation: r,
                    }))
                    .await;
                if !matches!(rel, Ok(ConfigReply::Released(Released::Reconciling))) {
                    let _ = sup
                        .dispatch_config_request_reliable(|| ConfigRequest::ForceReconcile(domain))
                        .await;
                }
                tracing::error!(
                    "Parent directory fsync failed after rename ({warn}); keeping old policy active and entering Reconciling"
                );
                return Err(ConfigCommitError::post_publish_reconciling(ArmError::Rejected {
                    status: 500,
                    code: "PERSIST_FAILED".into(),
                    message: format!("Parent directory fsync failed after rename ({warn}); reconciliation required"),
                }));
            }

            if let Some(h) = hooks.after_rename_before_activate.as_ref() {
                h();
            }
        }
    }

    // 6. Fresh post-dir-sync authority recheck + atomic activation/publication.
    let post_rename_revoked = match auth_check() {
        Ok(()) => None,
        Err((code, err)) => {
            if !disk_published {
                release_pre_rename(Arc::clone(&sup), guard).await;
                return Err(ConfigCommitError::pre_publish(err));
            }
            Some(code)
        }
    };

    let reservation = guard.take().expect("reservation held by guard");
    let durability = if disk_published {
        Durability::Durable
    } else {
        Durability::Volatile
    };
    let activated = match sup
        .dispatch_config_request_owned_reliable(ConfigRequest::Activate(ActivateConfig {
            reservation,
            auth,
            revoked_off_thread: post_rename_revoked,
            candidate,
            durability,
            expected_workspace_revision,
            expected_poses_revision,
        }))
        .await
    {
        Ok(ConfigReply::Activated(Ok(act))) => act,
        Ok(ConfigReply::Activated(Err(e))) => {
            let is_reconciling = matches!(e, ConfigError::Reconciling(_)) || disk_published;
            if is_reconciling {
                if let Some(pg) = episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
            }
            if disk_published {
                tracing::error!("Configuration written on disk but not activated: {:?}; reconciling", e);
            }
            return Err(ConfigCommitError {
                error: e.to_arm_error(),
                disk_published,
                reconciling: is_reconciling,
            });
        }
        Err(e) => {
            if disk_published {
                if let Some(pg) = episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
            }
            return Err(ConfigCommitError {
                error: e.to_arm_error(),
                disk_published,
                reconciling: disk_published,
            });
        }
        _ => {
            if disk_published {
                if let Some(pg) = episode_pin_guard.take() {
                    pg.keep_pinned_for_reconcile();
                }
            }
            return Err(ConfigCommitError {
                error: ArmError::Internal("unexpected config activate reply".into()),
                disk_published,
                reconciling: disk_published,
            });
        }
    };

    // Publish committed sequence revision BEFORE dropping `episode_pin_guard` so concurrent
    // episode deletes linearize cleanly with the newly active `SequenceFile`.
    if domain == ConfigDomain::Sequences {
        sup.set_sequences_revision(activated.revision);
    }

    // Update PublishedConfigHub off-thread immediately after owner activation.
    sup.published_config_hub().update(|cfg| {
        cfg.workspace_revision = activated.workspace_revision;
        update_published(cfg, activated);
    });
    drop(episode_pin_guard);

    Ok(ConfigCommitOutcome {
        activated,
        persisted: disk_published,
        durability_confirmed: true,
        durability_warning: None,
    })
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
        return Err((
            "cancelled",
            "STOPPED",
            "Safety stop intervened before gripper calibration persisted",
        ));
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
            client: ClientId(0),
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
            connection_id: 0,
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
/// `rename_only()`. On refusal, unconditionally enters `Reconciling`.
pub(crate) fn zero_check_owner_persisted(
    core: &mut SupervisorInner,
    action_id: u64,
    reservation: Reservation,
    simulated_err: bool,
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
        let active_rev = ConfigAuthority::revision(core, ConfigDomain::GripperCalibration);
        core.core
            .config_mut()
            .force_reconcile(ConfigDomain::GripperCalibration, active_rev);
        if core.zero_check_persisting == Some(action_id) {
            core.zero_check_persisting = None;
        }
        let ret = core.finish_zero_check_action_in_place(
            action_id,
            "failed",
            "PERSIST_FAILED",
            "Failed to record post-rename persistence state; reconciling required",
            code_buf,
            msg_buf,
        );
        Err(ret)
    }
}

/// Phase 5-error (owner-side, zero-alloc on `armd-control`): parent-directory fsync failed after
/// `rename_only()`. Keeps the OLD policy active, enters `HoldState::Reconciling` (motion stays
/// inhibited), and fails the action with `PERSIST_FAILED` + durability warning.
pub(crate) fn zero_check_owner_reconcile_dir_sync_failed(
    core: &mut SupervisorInner,
    action_id: u64,
    reservation: Reservation,
    warning: String,
    msg_buf: Option<String>,
) -> (Option<String>, Option<String>) {
    if !matches!(
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
    }
    core.fail_zero_check_dir_sync_in_place(action_id, warning, msg_buf)
}

/// Phase 6 (owner-side, zero-alloc on `armd-control`): post-dir-sync live owner authority recheck
/// and atomic in-place activation (`Durability::Durable`). Any failure enters `Reconciling`
/// without mutating live gripper calibration.
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
            "Operator session revoked after disk rename; reconciliation required",
        ))
    } else {
        check_zero_check_authority(core, ctx, proposal, auth, action_id, base_revision, None)
    };
    if auth_res.is_ok() {
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
        if core.zero_check_persisting == Some(action_id) {
            core.zero_check_persisting = None;
        }
        let _ = core.complete_zero_check_action(action_id, Durability::Durable, None);
        Ok((old_policy, old_jaw, code_buf, msg_buf))
    } else {
        let (state, code, msg) = auth_res.unwrap_err();
        if !matches!(
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

/// Legacy `tick_shared` async worker for `gripper_zero_check(store_offset=true)` routed through
/// the exact common commit ordering:
/// `reserve -> off-thread serialize + stage + file fsync -> fresh owner authority recheck ->
/// rename -> parent-directory fsync (failure -> Reconciling) -> fresh owner authority recheck ->
/// activate/publish`.
pub(crate) async fn run_legacy_zero_check_commit(
    sup_inner: Arc<crate::supervisor_lock::SupervisorLock<SupervisorInner>>,
    cw_lock: Arc<tokio::sync::Mutex<()>>,
    published_config: Arc<PublishedConfigHub>,
    hooks: ConfigCommitTestHooks,
    job: crate::supervisor::PendingZeroCheckPersist,
) {
    let _cw = cw_lock.lock().await;
    let check_op_auth = |ctx: &crate::supervisor::ZeroCheckPersistContext| -> bool {
        ctx.operator_auth
            .as_ref()
            .is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()))
    };
    if !check_op_auth(&job.ctx) {
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

    if let Some(h) = hooks.before_reserve.as_ref() {
        h();
    }

    // 1. Reserve strictly BEFORE serialization and staging.
    let req_seq = NEXT_CONFIG_REQ_SEQ.fetch_add(1, Ordering::Relaxed);
    let (reservation, proposal, auth) = {
        let mut inner = sup_inner.write().await;
        if inner.detached {
            return;
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
            job.action_id,
            job.base_revision,
            job.lease_owner,
            job.estop_epoch,
            job.stop_gen,
            job.deadline,
            &candidate_policy,
            req_seq,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        ) {
            Ok((r, p, a, _, _)) => (r, p, a),
            Err(_) => return,
        }
    };

    // 2. Off-thread serialize + stage + file fsync strictly AFTER reserve.
    let write = job.ctx.serialize_write(job.offset_rad);
    let staged_res = tokio::task::spawn_blocking(move || StagedWrite::stage(write.path, write.text))
        .await
        .map_err(|e| format!("stage task failed: {e}"))
        .and_then(|r| r);
    let staged = match staged_res {
        Ok(s) => s,
        Err(e) => {
            tracing::warn!("Failed to stage gripper calibration after zero_check: {e}");
            let mut inner = sup_inner.write().await;
            let _ = ConfigAuthority::release(&mut *inner, reservation);
            if inner.zero_check_persisting == Some(job.action_id) {
                inner.zero_check_persisting = None;
            }
            let _ = inner.fail_zero_check_action_owned(
                job.action_id,
                "PERSIST_FAILED".into(),
                format!("Failed to persist gripper calibration: {e}"),
            );
            return;
        }
    };

    if let Some(h) = hooks.after_reserve_before_rename.as_ref() {
        h();
    }

    // 3. Fresh pre-rename operator auth + live owner authority recheck.
    if !check_op_auth(&job.ctx) {
        staged.abort();
        let mut inner = sup_inner.write().await;
        let _ = ConfigAuthority::release(&mut *inner, reservation);
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
    let reservation = {
        let mut inner = sup_inner.write().await;
        if inner.detached {
            staged.abort();
            return;
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
            job.action_id,
            reservation,
            &proposal,
            &auth,
            job.base_revision,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        ) {
            Ok((r, _, _)) => r,
            Err(_) => {
                staged.abort();
                return;
            }
        }
    };

    // 4a. Rename off-thread.
    let sim_rename_fail = hooks
        .fail_rename_once
        .as_ref()
        .is_some_and(|f| f.swap(false, Ordering::AcqRel));
    let rename_res = if sim_rename_fail {
        staged.abort();
        Err("simulated rename failure".to_string())
    } else {
        tokio::task::spawn_blocking(move || staged.rename_only())
            .await
            .map_err(|e| format!("rename task failed: {e}"))
            .and_then(|r| r)
    };
    let dir_sync = match rename_res {
        Ok(ds) => ds,
        Err(e) => {
            tracing::warn!("Failed to publish gripper calibration after zero_check: {e}");
            let mut inner = sup_inner.write().await;
            let _ = ConfigAuthority::release(&mut *inner, reservation);
            if inner.zero_check_persisting == Some(job.action_id) {
                inner.zero_check_persisting = None;
            }
            let _ = inner.fail_zero_check_action_owned(
                job.action_id,
                "PERSIST_FAILED".into(),
                format!("Failed to persist gripper calibration: {e}"),
            );
            return;
        }
    };

    // 4b. Mark Persisted(DurableUnconfirmed) immediately after rename.
    let sim_persisted_fail = hooks
        .fail_persisted_once
        .as_ref()
        .is_some_and(|f| f.swap(false, Ordering::AcqRel));
    let reservation = {
        let mut inner = sup_inner.write().await;
        match zero_check_owner_persisted(
            &mut inner,
            job.action_id,
            reservation,
            sim_persisted_fail,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        ) {
            Ok((r, _, _)) => r,
            Err(_) => return,
        }
    };

    // 5. Parent-directory fsync strictly BEFORE activation. Failure enters Reconciling!
    let sim_dir_sync_fail = hooks
        .fail_dir_sync_once
        .as_ref()
        .is_some_and(|f| f.swap(false, Ordering::AcqRel));
    let outcome = if sim_dir_sync_fail {
        PublishOutcome::PublishedDurabilityUnconfirmed("simulated parent dir fsync failure".into())
    } else {
        tokio::task::spawn_blocking(move || dir_sync.sync_parent_dir())
            .await
            .unwrap_or_else(|e| PublishOutcome::PublishedDurabilityUnconfirmed(e.to_string()))
    };
    if let PublishOutcome::PublishedDurabilityUnconfirmed(warn) = outcome {
        let mut inner = sup_inner.write().await;
        let _ = zero_check_owner_reconcile_dir_sync_failed(
            &mut inner,
            job.action_id,
            reservation,
            warn,
            Some(String::with_capacity(160)),
        );
        return;
    }

    if let Some(h) = hooks.after_rename_before_activate.as_ref() {
        h();
    }

    // 6. Fresh post-dir-sync authority recheck + atomic activation + PublishedConfigHub update.
    let operator_revoked = !check_op_auth(&job.ctx);
    let pub_policy = candidate_policy.clone();
    let pub_jaw = job.ctx.jaw_opening.clone();
    let activated_ok = {
        let mut inner = sup_inner.write().await;
        if inner.detached {
            return;
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
            job.action_id,
            reservation,
            &proposal,
            &auth,
            job.base_revision,
            job.offset_rad,
            candidate_policy,
            job.ctx.jaw_opening,
            operator_revoked,
            Some(String::with_capacity(64)),
            Some(String::with_capacity(160)),
        )
        .is_ok()
    };
    if activated_ok {
        published_config.apply_zero_check_commit(
            job.offset_rad,
            pub_policy,
            pub_jaw,
            job.base_revision + 1,
        );
    }
}

/// Synchronous legacy fallback when `defer_zero_check_persist == false` (bare `SupervisorInner`
/// tests): runs the exact same common commit state machine (`reserve -> serialize + stage +
/// file fsync -> fresh authority recheck -> rename -> persisted -> parent-dir fsync (failure ->
/// Reconciling) -> fresh authority recheck -> activate`).
pub(crate) fn commit_zero_check_sync(
    core: &mut SupervisorInner,
    action_id: u64,
    offset_rad: f32,
    ctx_box: Box<crate::supervisor::ZeroCheckPersistContext>,
) {
    let check_op_auth = |c: &crate::supervisor::ZeroCheckPersistContext| -> bool {
        c.operator_auth
            .as_ref()
            .is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()))
    };
    // Hold action as running under `zero_check_persisting` so every owner primitive runs identically.
    if let Some(a) = core.last_action.as_mut().filter(|a| a.id == action_id) {
        a.state = HolderLabel::new("running");
        a.finished_at = None;
    }
    core.zero_check_persisting = Some(action_id);

    if !check_op_auth(&ctx_box) {
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
    let req_seq = NEXT_CONFIG_REQ_SEQ.fetch_add(1, Ordering::Relaxed);
    let commit_ctx = CommitCtx {
        arm: core.core.id(),
        boot: BootId::process(),
        now: Instant::now(),
        stop_generation: stop_gen,
    };

    // 1. Reserve strictly BEFORE serialization and staging.
    let (reservation, proposal, auth) = match zero_check_owner_reserve(
        core,
        &commit_ctx,
        action_id,
        base_revision,
        lease_owner,
        estop_epoch,
        stop_gen,
        deadline,
        &candidate_policy,
        req_seq,
        Some(String::with_capacity(64)),
        Some(String::with_capacity(160)),
    ) {
        Ok((r, p, a, _, _)) => (r, p, a),
        Err(_) => return,
    };

    // 2. Serialize + stage + file fsync strictly AFTER reserve.
    let write = ctx_box.serialize_write(offset_rad);
    let staged = match StagedWrite::stage(write.path, write.text) {
        Ok(s) => s,
        Err(e) => {
            tracing::warn!("Failed to stage gripper calibration after zero_check: {e}");
            let _ = ConfigAuthority::release(core, reservation);
            if core.zero_check_persisting == Some(action_id) {
                core.zero_check_persisting = None;
            }
            let _ = core.fail_zero_check_action_owned(
                action_id,
                "PERSIST_FAILED".into(),
                format!("Failed to persist gripper calibration: {e}"),
            );
            return;
        }
    };

    // 3. Fresh pre-rename operator auth + live owner authority recheck.
    if !check_op_auth(&ctx_box) {
        staged.abort();
        let _ = ConfigAuthority::release(core, reservation);
        if core.zero_check_persisting == Some(action_id) {
            core.zero_check_persisting = None;
        }
        let _ = core.fail_zero_check_action_owned(
            action_id,
            "OPERATOR_REQUIRED".into(),
            "Operator session expired or locked before gripper calibration persisted".into(),
        );
        return;
    }
    let verify_ctx = CommitCtx {
        arm: core.core.id(),
        boot: BootId::process(),
        now: Instant::now(),
        stop_generation: core.current_stop_gen(),
    };
    let reservation = match zero_check_owner_verify_authority(
        core,
        &verify_ctx,
        action_id,
        reservation,
        &proposal,
        &auth,
        base_revision,
        Some(String::with_capacity(64)),
        Some(String::with_capacity(160)),
    ) {
        Ok((r, _, _)) => r,
        Err(_) => {
            staged.abort();
            return;
        }
    };

    // 4a. Rename.
    let dir_sync = match staged.rename_only() {
        Ok(ds) => ds,
        Err(e) => {
            tracing::warn!("Failed to publish gripper calibration after zero_check: {e}");
            let _ = ConfigAuthority::release(core, reservation);
            if core.zero_check_persisting == Some(action_id) {
                core.zero_check_persisting = None;
            }
            let _ = core.fail_zero_check_action_owned(
                action_id,
                "PERSIST_FAILED".into(),
                format!("Failed to persist gripper calibration: {e}"),
            );
            return;
        }
    };

    // 4b. Mark Persisted(DurableUnconfirmed).
    let reservation = match zero_check_owner_persisted(
        core,
        action_id,
        reservation,
        false,
        Some(String::with_capacity(64)),
        Some(String::with_capacity(160)),
    ) {
        Ok((r, _, _)) => r,
        Err(_) => return,
    };

    // 5. Parent-directory fsync strictly BEFORE activation. Failure enters Reconciling.
    if let PublishOutcome::PublishedDurabilityUnconfirmed(warn) = dir_sync.sync_parent_dir() {
        let _ = zero_check_owner_reconcile_dir_sync_failed(
            core,
            action_id,
            reservation,
            warn,
            Some(String::with_capacity(160)),
        );
        return;
    }

    // 6. Fresh post-dir-sync authority recheck + atomic activation.
    let operator_revoked = !check_op_auth(&ctx_box);
    let act_ctx = CommitCtx {
        arm: core.core.id(),
        boot: BootId::process(),
        now: Instant::now(),
        stop_generation: core.current_stop_gen(),
    };
    let _ = zero_check_owner_activate(
        core,
        &act_ctx,
        action_id,
        reservation,
        &proposal,
        &auth,
        base_revision,
        offset_rad,
        candidate_policy,
        ctx_box.jaw_opening,
        operator_revoked,
        Some(String::with_capacity(64)),
        Some(String::with_capacity(160)),
    );
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
        if sup.owner_mode() == OWNER_EXECUTOR && pub_cfg.has_driver {
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
            if pub_cfg.has_driver {
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
