//! Published configuration snapshot and read-side accessors for `ArmSupervisor`.
//!
//! Legacy cache-miss refreshes split into three phases so `SupervisorLock::read()` is never
//! held across a contended legacy `DriverLock` mutex, capability JSON serialization, or
//! displaced-snapshot deallocation:
//! 1. **Capture** under `SupervisorLock::read()`: acquire driver capabilities via a single
//!    non-blocking `DriverSlot::try_with_ref` borrow (retrying after dropping the supervisor
//!    guard if `WouldBlock`, preserving poisoned-driver fallbacks), then copy live fields.
//! 2. **Finalize** outside all locks: serialize `motor_current_cap` to JSON, compute derived
//!    capabilities/thresholds, and wrap in `Arc<PublishedConfig>`.
//! 3. **Publish** under `PublishedConfigHub`'s write guard only if ownership is still
//!    `OWNER_LEGACY` and the captured generation is strictly newer, dropping any displaced
//!    or rejected `Arc` after unlock.

use std::future::Future;
use std::path::PathBuf;
use std::time::Duration;

#[allow(unused_imports)]
use super::super::*;
use crate::calibration::{CalibrationFile, CalibrationStale};
use crate::collision::Sensitivity;
use crate::driver::{DriverCapabilities, GripperPosForceInfo, MotorCurrentCap};
use crate::keepout::Keepout;
use crate::model::{ArmConfig, CameraMount, GripperPolicy, JawOpening, MotorZeroSupport};
use crate::poses::PoseStore;
use crate::recording::store::EpisodeStore;
use crate::rt_core::requests::config::{config_unavailable, PublishedConfig, PublishedConfigHub};
use crate::sequences::SequenceStore;
use crate::supervisor_lock::{DriverSlot, DriverTryError};

const BUSY_YIELD_RETRIES: u32 = 4;
const BUSY_RETRY_SLEEP: Duration = Duration::from_millis(1);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DriverBusy;

// Transient stack capture value: keep unboxed to avoid an extra heap allocation under SupervisorLock::read().
#[allow(clippy::large_enum_variant)]
enum CapturedDriverCaps {
    None,
    Poisoned,
    Live {
        supports_nonblocking_mode_switch: bool,
        torque_feedback: bool,
        temperature_feedback: bool,
        velocity_feedback: bool,
        enable_feedback: bool,
        supports_mit_mode_switch: bool,
        motor_current_cap: Option<MotorCurrentCap>,
        motor_zero_support: MotorZeroSupport,
        gripper_pos_force_info: Option<GripperPosForceInfo>,
    },
}

fn read_driver_caps(drv: &dyn crate::driver::ArmDriver) -> CapturedDriverCaps {
    let caps = drv.capabilities();
    CapturedDriverCaps::Live {
        supports_nonblocking_mode_switch: drv.mode_switch().supports_nonblocking_mode_switch(),
        torque_feedback: caps.torque_feedback,
        temperature_feedback: caps.temperature_feedback,
        velocity_feedback: caps.velocity_feedback,
        enable_feedback: caps.enable_feedback,
        supports_mit_mode_switch: drv.joint_mode().supports_mit_mode_switch(),
        motor_current_cap: drv.pos_force().motor_current_cap(),
        motor_zero_support: drv.motor_zero_support(),
        gripper_pos_force_info: drv.pos_force().gripper_pos_force_info(),
    }
}

fn try_capture_driver_caps(driver: Option<&DriverSlot>) -> Result<CapturedDriverCaps, DriverBusy> {
    let Some(slot) = driver else {
        return Ok(CapturedDriverCaps::None);
    };
    match slot.try_with_ref(read_driver_caps) {
        Ok(caps) => Ok(caps),
        Err(DriverTryError::WouldBlock) => Err(DriverBusy),
        Err(DriverTryError::Poisoned) => Ok(CapturedDriverCaps::Poisoned),
    }
}

fn capture_driver_caps_blocking(driver: Option<&DriverSlot>) -> CapturedDriverCaps {
    let Some(slot) = driver else {
        return CapturedDriverCaps::None;
    };
    slot.with_ref(read_driver_caps)
        .unwrap_or(CapturedDriverCaps::Poisoned)
}

/// Raw coherent fields copied under `SupervisorLock` before off-lock finalization.
struct PublishedConfigCapture {
    config: ArmConfig,
    follower: Option<String>,
    keepout: Keepout,
    collision_enabled: bool,
    collision_sensitivity: Sensitivity,
    collision_runtime: bool,
    collision_path: Option<PathBuf>,
    collision_revision: u64,
    collision_active: bool,
    collision_torque_feedback: bool,
    base_gripper_policy: Option<GripperPolicy>,
    gripper_calib_path: Option<PathBuf>,
    gripper_calib_offset: Option<f32>,
    gripper_calib_jaw_opening: Option<JawOpening>,
    gripper_calib_invalid: bool,
    gripper_calib_stale: Option<CalibrationStale>,
    gripper_calib_revision: u64,
    poses: PoseStore,
    sequences: SequenceStore,
    episodes: EpisodeStore,
    max_episode_s: f64,
    grasp_setup_revision: u64,
    tool_commissioning_path: Option<PathBuf>,
    tool_commissioning_confirmed_at: Option<String>,
    tool_commissioning_revision: u64,
    calibration: Option<CalibrationFile>,
    calibration_revision: u64,
    calibration_busy: bool,
    calibration_txn_id: Option<u64>,
    calibration_quarantined: bool,
    calibration_load_error: Option<String>,
    profile_camera_mount: CameraMount,
    profile_floor_z_m: Option<f32>,
    floor_source: &'static str,
    workspace_revision: u64,
    joint_zero_offsets: Vec<f32>,
    zero_convention: crate::joint_zero::ZeroConvention,
    operator_auth_enforced: bool,
    driver_caps: CapturedDriverCaps,
    cached_driver_caps: DriverCapabilities,
    driver_config_error: Option<String>,
    gravity_compensated: bool,
    rt_executor_owned: bool,
    driver_gripper_close_speed_limit: Option<f32>,
    #[cfg(feature = "policy-runtime")]
    policy_ledger_path: Option<PathBuf>,
    fallback_measured_positions: Vec<f32>,
    cell: Option<Arc<crate::cell::CellContext>>,
}

impl PublishedConfigCapture {
    #[inline(never)]
    fn try_from_inner(inner: &SupervisorInner) -> Result<Box<Self>, DriverBusy> {
        let driver_caps = try_capture_driver_caps(inner.driver.as_ref())?;
        Ok(Self::from_inner_with_driver_caps(inner, driver_caps))
    }

    #[inline(never)]
    fn blocking_from_inner(inner: &SupervisorInner) -> Box<Self> {
        let driver_caps = capture_driver_caps_blocking(inner.driver.as_ref());
        Self::from_inner_with_driver_caps(inner, driver_caps)
    }

    #[inline(never)]
    fn from_inner_with_driver_caps(
        inner: &SupervisorInner,
        driver_caps: CapturedDriverCaps,
    ) -> Box<Self> {
        let mut sequences = inner.sequences.clone();
        sequences.pins = inner.active_sequence_pins().cloned();
        Box::new(Self {
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
            driver_caps,
            cached_driver_caps: inner.driver_caps,
            driver_config_error: inner.driver_config_error.clone(),
            gravity_compensated: inner.gravity_compensated,
            rt_executor_owned: inner.rt_executor_owned,
            driver_gripper_close_speed_limit: inner.driver_gripper_close_speed_limit,
            #[cfg(feature = "policy-runtime")]
            policy_ledger_path: inner.policy_ledger_path.clone(),
            fallback_measured_positions: inner.measured_positions.clone(),
            cell: inner.cell.clone(),
        })
    }

    fn compute_gravity_comp_supported(&self, has_driver: bool) -> bool {
        let policy_ok = self.config.gravity_comp.is_some()
            && self.config.six_plus_gripper()
            && self.config.gravity_model.is_some();
        let driver_ok = match &self.driver_caps {
            CapturedDriverCaps::None => true,
            CapturedDriverCaps::Poisoned => false,
            CapturedDriverCaps::Live {
                supports_mit_mode_switch,
                velocity_feedback,
                torque_feedback,
                enable_feedback,
                supports_nonblocking_mode_switch,
                ..
            } => {
                *supports_mit_mode_switch
                    && *velocity_feedback
                    && *torque_feedback
                    && *enable_feedback
                    && (!self.rt_executor_owned || *supports_nonblocking_mode_switch)
            }
        };
        policy_ok
            && self.gravity_compensated
            && self.config.dynamics_qualified
            && driver_ok
            && !crate::gravity_comp::executor_blocks_hardware_gravity_comp(
                self.rt_executor_owned,
                has_driver,
            )
    }

    fn compute_effective_planned_close_speed(&self) -> f64 {
        let cap_limit = if self.rt_executor_owned {
            self.driver_gripper_close_speed_limit
        } else {
            self.config.gripper_index().and_then(|gripper_idx| match &self.driver_caps {
                CapturedDriverCaps::Live {
                    gripper_pos_force_info: Some(info),
                    ..
                } if info.joint == gripper_idx => {
                    Some(info.close_speed_rad_s).filter(|v| v.is_finite() && *v > 0.0)
                }
                _ => None,
            })
        };
        let mut v = self.config.gripper_policy.as_ref().map_or(
            f64::from(crate::model::DEFAULT_CLOSE_SPEED_RAD_S),
            |p| p.close_speed(),
        );
        if let Some(max_v) = self
            .config
            .gripper_index()
            .and_then(|g| self.config.limits.get(g))
            .map(|l| f64::from(l.max_velocity))
            .filter(|v| v.is_finite() && *v > 0.0)
        {
            v = v.min(max_v);
        }
        if let Some(cap) = cap_limit {
            v = v.min(f64::from(cap));
            let raw = crate::model::DamiaoGripperDrive::quantize_vel_raw_f64(v);
            if raw > 0 {
                return f64::from(raw) / 100.0;
            }
        }
        v
    }

    fn compute_automatic_unload(&self, has_driver: bool) -> bool {
        if !has_driver {
            return true;
        }
        self.config.resting_pose.as_ref().is_some_and(|policy| {
            let c = self.cached_driver_caps;
            c.velocity_feedback
                && c.enable_feedback
                && (c.torque_feedback || policy.support_torque_limits.is_empty())
                && (c.load_feedback || policy.support_load_limits.is_empty())
        })
    }

    #[inline(never)]
    fn finalize(self: Box<Self>) -> PublishedConfig {
        let this = *self;
        let has_driver = !matches!(this.driver_caps, CapturedDriverCaps::None);
        let gravity_comp_supported = this.compute_gravity_comp_supported(has_driver);
        let effective_planned_close_speed = this.compute_effective_planned_close_speed();
        let driver_supports_automatic_unload = this.compute_automatic_unload(has_driver);
        let torque_guard_thresholds = this
            .config
            .torque_guard
            .map(|p| p.thresholds(&this.config))
            .unwrap_or_default();
        let policy_live_allowed = {
            #[cfg(feature = "policy-runtime")]
            {
                matches!(
                    (has_driver, this.follower.as_deref()),
                    (false, None) | (false, Some("mock")) | (_, Some("sim"))
                )
            }
            #[cfg(not(feature = "policy-runtime"))]
            {
                false
            }
        };
        let (
            driver_supports_nonblocking_mode_switch,
            driver_torque_feedback,
            driver_temperature_feedback,
            driver_motor_current_cap,
            driver_motor_zero_support,
        ) = match this.driver_caps {
            CapturedDriverCaps::None => (
                false,
                false,
                true,
                None,
                MotorZeroSupport::set_current(
                    "process",
                    true,
                    vec!["mock follower: the readings of the zeroed joints become 0 (the simulated arm has no encoder offset)".into()],
                ),
            ),
            CapturedDriverCaps::Poisoned => (
                false,
                false,
                false,
                None,
                MotorZeroSupport::unsupported("driver unavailable"),
            ),
            CapturedDriverCaps::Live {
                supports_nonblocking_mode_switch,
                torque_feedback,
                temperature_feedback,
                motor_current_cap,
                motor_zero_support,
                ..
            } => (
                supports_nonblocking_mode_switch,
                torque_feedback,
                temperature_feedback,
                motor_current_cap.and_then(|cap| serde_json::to_value(cap).ok()),
                motor_zero_support,
            ),
        };
        PublishedConfig {
            config: this.config,
            follower: this.follower,
            keepout: this.keepout,
            collision_enabled: this.collision_enabled,
            collision_sensitivity: this.collision_sensitivity,
            collision_runtime: this.collision_runtime,
            collision_path: this.collision_path,
            collision_revision: this.collision_revision,
            collision_active: this.collision_active,
            collision_torque_feedback: this.collision_torque_feedback,
            base_gripper_policy: this.base_gripper_policy,
            gripper_calib_path: this.gripper_calib_path,
            gripper_calib_offset: this.gripper_calib_offset,
            gripper_calib_jaw_opening: this.gripper_calib_jaw_opening,
            gripper_calib_invalid: this.gripper_calib_invalid,
            gripper_calib_stale: this.gripper_calib_stale,
            gripper_calib_revision: this.gripper_calib_revision,
            poses: this.poses,
            sequences: this.sequences,
            episodes: this.episodes,
            max_episode_s: this.max_episode_s,
            grasp_setup_revision: this.grasp_setup_revision,
            tool_commissioning_path: this.tool_commissioning_path,
            tool_commissioning_confirmed_at: this.tool_commissioning_confirmed_at,
            tool_commissioning_revision: this.tool_commissioning_revision,
            calibration: this.calibration,
            calibration_revision: this.calibration_revision,
            calibration_busy: this.calibration_busy,
            calibration_txn_id: this.calibration_txn_id,
            calibration_quarantined: this.calibration_quarantined,
            calibration_load_error: this.calibration_load_error,
            profile_camera_mount: this.profile_camera_mount,
            profile_floor_z_m: this.profile_floor_z_m,
            floor_source: this.floor_source,
            workspace_revision: this.workspace_revision,
            joint_zero_offsets: this.joint_zero_offsets,
            zero_convention: this.zero_convention,
            operator_auth_enforced: this.operator_auth_enforced,
            has_driver,
            driver_supports_nonblocking_mode_switch,
            driver_torque_feedback,
            driver_temperature_feedback,
            driver_supports_automatic_unload,
            driver_motor_current_cap,
            driver_motor_zero_support,
            driver_config_error: this.driver_config_error,
            gravity_compensated: this.gravity_compensated,
            gravity_comp_supported,
            torque_guard_thresholds,
            effective_planned_close_speed,
            policy_live_allowed,
            #[cfg(feature = "policy-runtime")]
            policy_ledger_path: this.policy_ledger_path,
            fallback_measured_positions: this.fallback_measured_positions,
            cell: this.cell,
        }
    }

    /// Finalizes into a heap-allocated `Arc<PublishedConfig>` strictly inside a non-inlined
    /// synchronous frame so `PublishedConfig` never inflates `published_config()`'s async state.
    #[inline(never)]
    fn finalize_arc(self: Box<Self>) -> Arc<PublishedConfig> {
        Arc::new(self.finalize())
    }
}

impl PublishedConfig {
    /// Captures a complete configuration snapshot from `inner` off the RT thread.
    pub fn from_inner(inner: &SupervisorInner) -> Self {
        PublishedConfigCapture::blocking_from_inner(inner).finalize()
    }
}

impl ArmSupervisor {
    pub fn config_write_lock(&self) -> Arc<tokio::sync::Mutex<()>> {
        Arc::clone(&self.config_write_mutex)
    }

    pub fn published_config_hub(&self) -> &Arc<PublishedConfigHub> {
        &self.published_config
    }

    /// Filters `PublishedConfig::sequences.pins` on `OWNER_EXECUTOR` against the live published
    /// `StateHub` action state so stale sequence pins are never exposed once the owning action has
    /// reached a terminal state (even before async `cleanup()` clears `pins`).
    fn filter_executor_published_config(&self, cfg: Arc<PublishedConfig>) -> Arc<PublishedConfig> {
        if cfg.sequences.pins.is_none() {
            return cfg;
        }
        let running_action_id = self
            .hub()
            .latest()
            .and_then(|s| s.obs.action.as_ref().filter(|a| a.is_running()).map(|a| a.id));
        if cfg.sequences.active_pins(running_action_id).is_some() {
            return cfg;
        }
        let mut filtered = (*cfg).clone();
        filtered.sequences.pins = None;
        Arc::new(filtered)
    }

    fn executor_published_config(&self) -> Result<Arc<PublishedConfig>, ArmError> {
        let Some(handle) = self.executor_handle() else {
            return Err(config_unavailable());
        };
        if handle.is_exited() {
            return Err(config_unavailable());
        }
        let cfg = self
            .published_config
            .latest()
            .ok_or_else(config_unavailable)?;
        Ok(self.filter_executor_published_config(cfg))
    }

    /// Returns the latest published configuration snapshot without touching `SupervisorLock`
    /// in `OWNER_EXECUTOR` mode. Fails closed with `503 CONFIG_UNAVAILABLE` if ownership is
    /// transitioning or failed.
    pub async fn published_config(&self) -> Result<Arc<PublishedConfig>, ArmError> {
        self.published_config_with_after_capture_hook(|_| std::future::ready(()))
            .await
    }

    /// Test-accessible entry point executing the exact `published_config()` capture / finalize /
    /// publish pipeline with a deterministic hook invoked after dropping `SupervisorLock::read()`
    /// and before finalizing/publishing into `PublishedConfigHub`.
    #[doc(hidden)]
    pub async fn published_config_with_after_capture_hook<F, Fut>(
        &self,
        mut after_capture: F,
    ) -> Result<Arc<PublishedConfig>, ArmError>
    where
        F: FnMut(u64) -> Fut,
        Fut: Future<Output = ()>,
    {
        let mut busy_retries: u32 = 0;
        loop {
            match self.owner_mode() {
                OWNER_EXECUTOR => return self.executor_published_config(),
                OWNER_LEGACY => {}
                _ => return Err(config_unavailable()),
            }
            let current_gen = self.inner.generation();
            let (cached, cached_gen) = self.published_config.latest_with_gen();
            if cached_gen == current_gen {
                return Ok(cached);
            }
            let capture_res = {
                let inner = self.inner.read().await;
                if inner.detached {
                    drop(inner);
                    if self.owner_mode() == OWNER_EXECUTOR {
                        return self.executor_published_config();
                    }
                    return Err(config_unavailable());
                }
                let gen = self.inner.generation();
                PublishedConfigCapture::try_from_inner(&inner).map(|capture| (capture, gen))
            };
            let (capture, gen) = match capture_res {
                Ok(pair) => pair,
                Err(DriverBusy) => {
                    busy_retries = busy_retries.saturating_add(1);
                    if busy_retries <= BUSY_YIELD_RETRIES {
                        tokio::task::yield_now().await;
                    } else {
                        tokio::time::sleep(BUSY_RETRY_SLEEP).await;
                    }
                    continue;
                }
            };
            after_capture(gen).await;
            let snap = capture.finalize_arc();
            match self
                .published_config
                .publish_legacy_arc_if_newer(snap, gen, || self.owner_mode() == OWNER_LEGACY)
            {
                Some(published) => return Ok(published),
                None => match self.owner_mode() {
                    OWNER_EXECUTOR => return self.executor_published_config(),
                    OWNER_LEGACY => continue,
                    _ => return Err(config_unavailable()),
                },
            }
        }
    }
}
