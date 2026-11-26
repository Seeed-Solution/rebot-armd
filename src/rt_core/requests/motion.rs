//! Owner-side off-lock motion preparation and single-authority commit transactions for
//! jog, home, park, and preset requests (PR M: spec sections A3, A4, A5).
//!
//! One install function per request kind (`install_jog`, `install_home`, `install_park`,
//! `install_park_continuation`) is shared by legacy WS, `ArmSupervisor::{jog,home}`, and
//! RT `CoreOps`, reusing A1's `AdmissionStamp` / `Admitted<'g>` without a second tracker.

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::driver::DriverCapabilities;
use crate::kinematics::{WORKSPACE_EPS, WORKSPACE_PARK_ESCAPE_EPS};
use crate::model::{ArmConfig, JointLimit, WorkspacePolicy, MAX_MOTION_DOF};
use crate::motion::{ActiveTrajectory, MotionError};
use crate::power::{PowerBudget, PowerHold};
use crate::rt_core::broker::{ActionId, BootId, Descriptor};
use crate::rt_core::mailbox::MAX_DOF;
use crate::rt_core::ops::{CommitCtx, Committed, PlanSlot, Primitive};
use crate::rt_core::publish::Snapshot;
use crate::rt_core::scratch::copy_into;
use crate::safety::{SafetyError, SafetyState};
use crate::supervisor::{
    check_path_with_tcp_tol, keepout_sweep_with_tcp_tol, AdmissionStamp, ArmError,
    ParkContinuation, ParkContinuationPhase, ParkRearmStart, SupervisorInner, MAX_KEEPOUT_EVALS_PER_PLAN,
};
use crate::torque_guard::TorqueHold;
use crate::trajectory::{
    JointPolynomials, MultiJointQuinticTrajectory, QuinticPolynomial, TrajectoryPlanner, INLINE_POLYS,
};

const EXACT_START_TOL: f32 = 1e-6;
const PARK_DURATION_S: f64 = 1.5;
const PARK_EXTRA_DURATION_S: f64 = 0.8;
/// Default bounded planning deadline applied from request entry on legacy WS and direct
/// `ArmSupervisor::{jog,home}` requests (and as default for `MotionPlanSnapshot::from_published`).
pub const DEFAULT_MOTION_PLAN_DEADLINE: Duration = Duration::from_secs(5);

/// Request-time authority & bounded deadline carried by a prepared motion plan and enforced
/// atomically inside the single install path (`verify_slot_authority`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlanAuthority {
    pub boot: BootId,
    pub epoch: u64,
    pub lease_generation: u64,
    pub stop_generation: u64,
    pub deadline: Instant,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionKind { Jog, Home, Park }

#[derive(Debug, Clone)]
pub struct MotionRequest {
    pub kind: MotionKind,
    pub target_positions: Option<Vec<f32>>,
    pub duration_secs: Option<f64>,
    pub speed_scale: Option<f64>,
    pub action_kind: &'static str,
}

impl MotionRequest {
    pub fn jog(target_positions: Vec<f32>, duration_secs: Option<f64>, speed_scale: Option<f64>) -> Self {
        Self { kind: MotionKind::Jog, target_positions: Some(target_positions), duration_secs, speed_scale, action_kind: "joint_move" }
    }
    pub fn home(duration_secs: Option<f64>) -> Self {
        Self { kind: MotionKind::Home, target_positions: None, duration_secs, speed_scale: None, action_kind: "home" }
    }
    pub fn park() -> Self {
        Self { kind: MotionKind::Park, target_positions: None, duration_secs: Some(PARK_DURATION_S), speed_scale: None, action_kind: "park" }
    }
}

/// Deterministic FNV-1a hash of the active planning `PowerBudget` (0 alloc on RT).
pub fn power_budget_fingerprint(budget: Option<&PowerBudget>) -> u64 {
    let Some(b) = budget else { return 0; };
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut mix = |bytes: &[u8]| { for &byte in bytes { h = (h ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3); } };
    mix(&[u8::from(b.enabled), u8::from(b.planning.enabled)]);
    for f in [b.supply_rated_a, b.supply_trip_a, b.v_bus, b.base_w, b.k_cu, b.efficiency, b.planning.fraction] { mix(&f.to_bits().to_le_bytes()); }
    mix(&b.planning.min_scale.to_bits().to_le_bytes());
    mix(&(b.joints.len() as u64).to_le_bytes());
    for j in &b.joints {
        for f in [j.kt_nm_per_a, j.r_phase_ohm, j.idle_w, j.viscous_nm_s, j.coulomb_nm] { mix(&f.to_bits().to_le_bytes()); }
    }
    h.max(1)
}

/// Immutable motion planning configuration cached off-RT (in `StateHub` or per snapshot).
#[derive(Debug, Clone)]
pub struct MotionConfigSnapshot {
    pub config: ArmConfig,
    pub workspace: Option<WorkspacePolicy>,
    pub power_budget: Option<PowerBudget>,
    pub power_budget_hash: u64,
    pub driver_present: bool,
    pub driver_caps: DriverCapabilities,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
}

impl MotionConfigSnapshot {
    pub fn from_supervisor(inner: &SupervisorInner) -> Self {
        let power_budget = inner.power_budget().cloned();
        let power_budget_hash = power_budget_fingerprint(power_budget.as_ref());
        Self {
            config: inner.config.clone(),
            workspace: inner.workspace_policy().map(|(ws, _)| ws.clone()),
            power_budget,
            power_budget_hash,
            driver_present: inner.driver.is_some(),
            driver_caps: inner.driver_caps,
            workspace_revision: inner.workspace_revision,
            motion_config_revision: inner.motion_config_revision,
        }
    }

    pub fn planning_limits(&self, speed_scale: Option<f64>) -> Vec<JointLimit> {
        let scale = self.config.speed_cap.and_then(|c| c.joint_velocity_scale).unwrap_or(1.0);
        let (gripper, comm) = (self.config.gripper_index(), self.config.active_tool.commissioning_required);
        let mut limits = self.config.limits.clone();
        for (i, l) in limits.iter_mut().enumerate() {
            if Some(i) != gripper {
                if let Some(cap) = self.config.planned_max_velocity { l.max_velocity = l.max_velocity.min(cap); }
                l.max_velocity *= scale;
            }
            if let Some(s) = speed_scale { if Some(i) != gripper || comm { l.max_velocity *= s as f32; } }
        }
        limits
    }
}

/// Coherent off-lock planning input captured from either `SupervisorInner` or `StateHub`.
#[derive(Debug, Clone)]
pub struct MotionPlanSnapshot {
    pub config: Arc<MotionConfigSnapshot>,
    pub commanded_positions: Vec<f32>,
    pub commanded_velocities: Vec<f32>,
    pub measured_positions: Vec<f32>,
    pub torque_hold: Option<TorqueHold>,
    pub power_hold: Option<PowerHold>,
    pub holding_after_clear: bool,
    pub mit_active: bool,
    pub authority: PlanAuthority,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
}

impl MotionPlanSnapshot {
    pub fn from_supervisor(inner: &SupervisorInner) -> Self {
        Self::from_supervisor_with_config(inner, Arc::new(MotionConfigSnapshot::from_supervisor(inner)))
    }

    pub fn from_supervisor_with_config(inner: &SupervisorInner, config: Arc<MotionConfigSnapshot>) -> Self {
        let authority = PlanAuthority {
            boot: inner.boot_id,
            epoch: inner.safety.epoch_id(),
            lease_generation: inner.current_active_lease_gen(),
            stop_generation: inner.current_stop_gen(),
            deadline: Instant::now() + inner.motion_plan_deadline,
        };
        Self::from_supervisor_with_authority(inner, config, authority)
    }

    pub fn from_supervisor_with_authority(
        inner: &SupervisorInner,
        config: Arc<MotionConfigSnapshot>,
        authority: PlanAuthority,
    ) -> Self {
        let workspace_revision = config.workspace_revision;
        let motion_config_revision = config.motion_config_revision;
        Self {
            config,
            commanded_positions: inner.commanded_positions.clone(),
            commanded_velocities: inner.commanded_velocities.clone(),
            measured_positions: inner.measured_positions.clone(),
            torque_hold: inner.torque_guard.hold,
            power_hold: inner.power.gov.hold,
            holding_after_clear: inner.holding_after_clear(),
            mit_active: inner.mit_active(),
            authority,
            workspace_revision,
            motion_config_revision,
        }
    }

    /// Reuses `prev.config` and preserves `prev.authority` (including original request boot,
    /// epoch, lease_generation, stop_generation, and deadline) for a `StartDrifted` retry ONLY
    /// while both owner revisions still match. Returns `None` if workspace or motion config changed.
    pub fn refresh_for_drift_retry(inner: &SupervisorInner, prev: &MotionPlanSnapshot) -> Option<Self> {
        if prev.config.workspace_revision != inner.workspace_revision
            || prev.config.motion_config_revision != inner.motion_config_revision
        {
            return None;
        }
        Some(Self::from_supervisor_with_authority(inner, Arc::clone(&prev.config), prev.authority))
    }

    /// Zero-lock snapshot from published `StateHub` telemetry + cached `MotionConfigSnapshot`.
    pub fn from_published(snap: &Snapshot, config: Arc<MotionConfigSnapshot>) -> Option<Self> {
        let dof = config.config.dof;
        if snap.rt.workspace_revision != config.workspace_revision
            || snap.rt.motion_config_revision != config.motion_config_revision
            || snap.obs.commanded_positions.len() != dof
            || snap.rt.commanded_velocities.len() != dof
            || snap.obs.positions.len() != dof
        {
            return None;
        }
        let clock_now = snap.rt.clock_now.unwrap_or(snap.meta.sampled_at);
        Some(Self {
            config,
            commanded_positions: snap.obs.commanded_positions.clone(),
            commanded_velocities: snap.rt.commanded_velocities.clone(),
            measured_positions: snap.obs.positions.clone(),
            torque_hold: None,
            power_hold: None,
            holding_after_clear: snap.rt.holding_after_clear,
            mit_active: false,
            authority: PlanAuthority {
                boot: snap.rt.boot_id,
                epoch: snap.obs.epoch_id,
                lease_generation: snap.rt.lease_generation,
                stop_generation: snap.rt.stop_generation,
                deadline: clock_now + DEFAULT_MOTION_PLAN_DEADLINE,
            },
            workspace_revision: snap.rt.workspace_revision,
            motion_config_revision: snap.rt.motion_config_revision,
        })
    }

    /// Overrides the snapshot's deadline (e.g. to bind to a client's exact broker request deadline).
    pub fn with_deadline(mut self, deadline: Instant) -> Self {
        self.authority.deadline = deadline;
        self
    }

    /// Overrides the snapshot's authority tuple.
    pub fn with_authority(mut self, authority: PlanAuthority) -> Self {
        self.authority = authority;
        self
    }
}

/// Inline, `Copy` precomputed motion plan staged in `PlanSlot` (zero heap ops on RT).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PreparedMotionSlot {
    pub kind: MotionKind,
    pub dof: u8,
    pub start: [f32; MAX_MOTION_DOF],
    pub v0: [f32; MAX_MOTION_DOF],
    pub requested_target: [f32; MAX_MOTION_DOF],
    pub target: [f32; MAX_MOTION_DOF],
    pub vel_caps: [f32; MAX_MOTION_DOF],
    pub has_speed_scale: bool,
    pub speed_scale: f64,
    pub polynomials: [QuinticPolynomial; INLINE_POLYS],
    pub duration: f64,
    pub power_scale: Option<f64>,
    pub power_budget_hash: u64,
    pub tcp_offset: [f64; 3],
    pub checked_workspace: bool,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
    pub authority: PlanAuthority,
    pub for_holding_after_clear: bool,
    pub for_mit_deferral: bool,
    pub continuation_action: Option<u64>,
}

impl PreparedMotionSlot {
    pub fn speed_scale_opt(&self) -> Option<f64> { self.has_speed_scale.then_some(self.speed_scale) }

    pub fn active_trajectory(&self, action_id: Option<u64>, heap_traj: Option<MultiJointQuinticTrajectory>) -> ActiveTrajectory {
        let traj = heap_traj.unwrap_or(MultiJointQuinticTrajectory {
            polynomials: JointPolynomials::Inline { data: self.polynomials, len: usize::from(self.dof) },
            duration: self.duration,
            elapsed: 0.0,
        });
        let active = ActiveTrajectory::quintic(traj, action_id);
        if self.has_speed_scale { active.with_motion_vel_caps(self.vel_caps, usize::from(self.dof)) } else { active }
    }
}

#[derive(Debug, Clone)]
pub struct PreparedMotionPlan {
    pub slot: PreparedMotionSlot,
    pub heap_traj: Option<MultiJointQuinticTrajectory>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionInstallError {
    InvalidCommand, SpeedScaleRange, CommissioningRequired, GripperCloseTorqueFeedback,
    UnsupportedUnload, ConfigBusy, MotionBusy, CollisionHold, HandoverSettling,
    EStopActive, FaultActive, NotArmed, WrongTarget, DeadlineExpired,
    EpochMismatch, LeaseLost, Stopped, StampReused, TerminalsBusy,
    FeedbackStale, J7FeedbackStale, StartDrifted, PlanStale, SoftLimitExceeded, ContinuationMismatch, EnableFailed,
}

impl MotionInstallError {
    pub fn code(self) -> &'static str {
        match self {
            Self::InvalidCommand | Self::SpeedScaleRange => "INVALID_COMMAND",
            Self::CommissioningRequired => "COMMISSIONING_REQUIRED",
            Self::GripperCloseTorqueFeedback | Self::UnsupportedUnload => "UNSUPPORTED",
            Self::ConfigBusy => "CONFIG_BUSY",
            Self::MotionBusy => "MOTION_BUSY",
            Self::CollisionHold => "COLLISION_HOLD",
            Self::HandoverSettling => "REJECTED",
            Self::EStopActive => "ESTOP_ACTIVE",
            Self::FaultActive => "FAULT_ACTIVE",
            Self::NotArmed => "NOT_ARMED",
            Self::WrongTarget => "WRONG_TARGET",
            Self::DeadlineExpired => "DEADLINE_EXPIRED",
            Self::EpochMismatch => "EPOCH_MISMATCH",
            Self::LeaseLost => "LEASE_LOST",
            Self::Stopped => "STOPPED",
            Self::StampReused => "STAMP_REUSED",
            Self::TerminalsBusy => "TERMINALS_BUSY",
            Self::FeedbackStale => "FEEDBACK_STALE",
            Self::J7FeedbackStale => "J7_FEEDBACK_STALE",
            Self::StartDrifted | Self::PlanStale | Self::ContinuationMismatch => "PLAN_STALE",
            Self::SoftLimitExceeded => "JOINT_LIMIT",
            Self::EnableFailed => "ENABLE_FAILED",
        }
    }

    pub fn into_arm_error(self, inner: &SupervisorInner, kind: &str, expected_epoch: Option<u64>) -> ArmError {
        let rej = |code: &str, msg: &str| ArmError::Rejected { status: 409, code: code.into(), message: msg.into() };
        match self {
            Self::InvalidCommand => MotionError::InvalidCommand("invalid motion target or dimensions".into()).into(),
            Self::SpeedScaleRange => MotionError::InvalidCommand("speed_scale must be in [0.05, 1.0]".into()).into(),
            Self::CommissioningRequired => SafetyError::CommissioningRequired(
                if kind == "home" { "home".into() } else { format!("{kind} (requires speed_scale <= 0.2)") },
            ).into(),
            Self::GripperCloseTorqueFeedback => MotionError::Unsupported("Gripper close during commissioning jog requires torque_feedback when max_close_torque_nm is configured".into()).into(),
            Self::UnsupportedUnload => ArmError::Unsupported("Automatic unload requires a resting-pose profile and velocity, torque and enable-state feedback; no motion was started".into()),
            Self::ConfigBusy => rej("CONFIG_BUSY", "a configuration change is being activated or reconciled; motion is refused until it ends"),
            Self::MotionBusy => MotionError::MotionBusy("arm motion is busy or a transition is pending".into()).into(),
            Self::CollisionHold => SafetyError::CollisionHold("Collision hold active; clear fault or move clear".into()).into(),
            Self::HandoverSettling => SafetyError::HandoverSettling.into(),
            Self::EStopActive => SafetyError::EStopActive.into(),
            Self::FaultActive => match inner.safety.state() { SafetyState::Fault(r) => SafetyError::FaultActive(r.clone()).into(), _ => SafetyError::EStopActive.into() },
            Self::NotArmed => SafetyError::NotArmed(inner.safety.state().clone()).into(),
            Self::WrongTarget => rej("WRONG_TARGET", "prepared motion plan belongs to a different boot or arm target"),
            Self::DeadlineExpired => rej("DEADLINE_EXPIRED", "motion plan deadline expired before install"),
            Self::EpochMismatch => SafetyError::EpochMismatch { expected: inner.safety.epoch_id(), received: expected_epoch.unwrap_or(0) }.into(),
            Self::LeaseLost => SafetyError::LeaseLost.into(),
            Self::Stopped => SafetyError::Stopped.into(),
            Self::StampReused => SafetyError::StampAlreadyConsumed.into(),
            Self::TerminalsBusy => SafetyError::TerminalsBusy.into(),
            Self::FeedbackStale => rej("FEEDBACK_STALE", "Hardware feedback is stale or invalid"),
            Self::J7FeedbackStale => ArmError::RejectedStatic {
                status: 409,
                code: "J7_FEEDBACK_STALE",
                message: "Gripper (J7) position-force feedback is stale (> 100 ms); new gripper targets are refused until fresh feedback returns",
            },
            Self::StartDrifted | Self::PlanStale | Self::ContinuationMismatch => MotionError::PlanStale("arm state or motion configuration changed while planning".into()).into(),
            Self::SoftLimitExceeded => MotionError::InvalidCommand("target position exceeds joint soft limits".into()).into(),
            Self::EnableFailed => SafetyError::ClearFaultRejected("Park rearm enable failed".into()).into(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotionInstalled {
    Started { epoch: u64, action_id: u64, expected_duration_s: f64 },
    DeferredContinuation { epoch: u64, action_id: u64, expected_duration_s: f64 },
}

impl MotionInstalled {
    pub fn action_id(self) -> u64 { match self { Self::Started { action_id, .. } | Self::DeferredContinuation { action_id, .. } => action_id } }
    pub fn epoch(self) -> u64 { match self { Self::Started { epoch, .. } | Self::DeferredContinuation { epoch, .. } => epoch } }
    pub fn expected_duration_s(self) -> f64 { match self { Self::Started { expected_duration_s, .. } | Self::DeferredContinuation { expected_duration_s, .. } => expected_duration_s } }
}

fn zero_poly() -> QuinticPolynomial {
    QuinticPolynomial { a0: 0.0, a1: 0.0, a2: 0.0, a3: 0.0, a4: 0.0, a5: 0.0, duration: 1.0, p0: 0.0, p1: 0.0, v0: 0.0, v1: 0.0, a_start: 0.0, a_end: 0.0 }
}

fn map_admission_err(e: SafetyError) -> MotionInstallError {
    match e {
        SafetyError::EStopActive => MotionInstallError::EStopActive,
        SafetyError::EpochMismatch { .. } => MotionInstallError::EpochMismatch,
        SafetyError::Stopped => MotionInstallError::Stopped,
        SafetyError::LeaseLost => MotionInstallError::LeaseLost,
        SafetyError::StampAlreadyConsumed => MotionInstallError::StampReused,
        SafetyError::TerminalsBusy => MotionInstallError::TerminalsBusy,
        _ => MotionInstallError::NotArmed,
    }
}

/// Shared zero-allocation speed_scale + commissioning + gripper-close torque-feedback gate.
fn check_commissioning(
    arm_cfg: &ArmConfig,
    driver_present: bool,
    driver_caps: DriverCapabilities,
    kind: MotionKind,
    speed_scale: Option<f64>,
    target: &[f32],
    commanded: &[f32],
) -> Result<(), MotionInstallError> {
    if let Some(s) = speed_scale {
        if !s.is_finite() || !(0.05..=1.0).contains(&s) { return Err(MotionInstallError::SpeedScaleRange); }
    }
    if !arm_cfg.active_tool.commissioning_required { return Ok(()); }
    if kind == MotionKind::Home { return Err(MotionInstallError::CommissioningRequired); }
    if kind == MotionKind::Jog {
        if !speed_scale.is_some_and(|s| s <= 0.2 + 1e-6) { return Err(MotionInstallError::CommissioningRequired); }
        if let (Some(g), Some(p)) = (arm_cfg.gripper_index(), arm_cfg.gripper_policy.as_ref()) {
            if let (Some(&tg), Some(&cg)) = (target.get(g), commanded.get(g)) {
                let dir = if p.closed_rad >= p.open_rad { 1.0_f32 } else { -1.0_f32 };
                if (tg - cg) * dir > 1e-4 && p.max_close_torque_nm.is_some() && driver_present && !driver_caps.torque_feedback {
                    return Err(MotionInstallError::GripperCloseTorqueFeedback);
                }
            }
        }
    }
    Ok(())
}

/// Pure, lock-free off-RT motion preparation (quintic, power fit, swept workspace).
pub fn prepare_motion(snap: &MotionPlanSnapshot, req: &MotionRequest) -> Result<PreparedMotionPlan, ArmError> {
    let (cfg, arm_cfg, dof) = (&snap.config, &snap.config.config, snap.config.config.dof);
    if dof == 0 || dof > MAX_MOTION_DOF {
        return Err(MotionError::InvalidCommand(format!(
            "profile DOF {dof} exceeds supported inline motion limit {MAX_MOTION_DOF}"
        )).into());
    }
    let raw_target: &[f32] = match req.kind {
        MotionKind::Jog => req.target_positions.as_deref().ok_or_else(|| MotionError::InvalidCommand("missing target_positions".into()))?,
        MotionKind::Home => &arm_cfg.home_positions,
        MotionKind::Park => arm_cfg.resting_positions(),
    };
    if let Err(err) = check_commissioning(arm_cfg, cfg.driver_present, cfg.driver_caps, req.kind, req.speed_scale, raw_target, &snap.commanded_positions) {
        return Err(match err {
            MotionInstallError::SpeedScaleRange => MotionError::InvalidCommand("speed_scale must be in [0.05, 1.0]".into()).into(),
            MotionInstallError::CommissioningRequired => SafetyError::CommissioningRequired(
                if req.kind == MotionKind::Home { "home".into() } else { format!("{} (requires speed_scale <= 0.2)", req.action_kind) },
            ).into(),
            _ => MotionError::Unsupported("Gripper close during commissioning jog requires torque_feedback when max_close_torque_nm is configured".into()).into(),
        });
    }
    if req.kind == MotionKind::Park && (snap.holding_after_clear || snap.mit_active) {
        return Ok(PreparedMotionPlan {
            slot: PreparedMotionSlot {
                kind: MotionKind::Park, dof: dof as u8,
                start: [0.0; MAX_MOTION_DOF], v0: [0.0; MAX_MOTION_DOF], requested_target: [0.0; MAX_MOTION_DOF], target: [0.0; MAX_MOTION_DOF], vel_caps: [0.0; MAX_MOTION_DOF],
                has_speed_scale: false, speed_scale: 1.0, polynomials: [zero_poly(); INLINE_POLYS],
                duration: PARK_DURATION_S, power_scale: None, power_budget_hash: cfg.power_budget_hash,
                tcp_offset: arm_cfg.tcp_offset, checked_workspace: cfg.workspace.is_some(),
                workspace_revision: snap.workspace_revision, motion_config_revision: snap.motion_config_revision,
                authority: snap.authority,
                for_holding_after_clear: snap.holding_after_clear,
                for_mit_deferral: !snap.holding_after_clear && snap.mit_active,
                continuation_action: None,
            },
            heap_traj: None,
        });
    }

    let is_park = req.kind == MotionKind::Park;
    let (start, mut target, duration_secs, limits) = match req.kind {
        MotionKind::Jog => {
            if raw_target.len() != dof { return Err(SafetyError::DimensionMismatch { expected: dof, received: raw_target.len() }.into()); }
            for (&val, lim) in raw_target.iter().zip(&arm_cfg.limits) {
                if !val.is_finite() { return Err(SafetyError::InvalidValue.into()); }
                if val < lim.min_position || val > lim.max_position {
                    return Err(SafetyError::SoftLimitExceeded { joint: lim.name.clone(), value: val, min: lim.min_position, max: lim.max_position }.into());
                }
            }
            (snap.commanded_positions.clone(), raw_target.to_vec(), req.duration_secs, cfg.planning_limits(req.speed_scale))
        }
        MotionKind::Home => (snap.commanded_positions.clone(), arm_cfg.home_positions.clone(), req.duration_secs, cfg.planning_limits(None)),
        MotionKind::Park => {
            let clamped = snap.commanded_positions.iter().enumerate()
                .map(|(i, &p)| arm_cfg.limits.get(i).map_or(p, |l| p.clamp(l.min_position, l.max_position))).collect();
            (clamped, arm_cfg.resting_positions().to_vec(), Some(PARK_DURATION_S), cfg.planning_limits(None))
        }
    };
    let requested_unfiltered = target.clone();
    if !is_park {
        if let Some(h) = snap.torque_hold {
            if let Some(t) = target.get_mut(h.joint) { if (*t - h.hold_q) * h.sign > 0.0 { *t = h.hold_q; } }
        }
        if let Some(h) = snap.power_hold {
            for (j, t) in target.iter_mut().enumerate().take(crate::power::MAX_JOINTS) {
                if h.push[j] != 0.0 && (*t - h.hold_q[j]) * h.push[j] > 0.0 { *t = h.hold_q[j]; }
            }
        }
    }

    let zeros = vec![0.0f32; dof];
    let traj = TrajectoryPlanner::plan(&start, &snap.commanded_velocities, &target, &zeros, duration_secs, &limits)?;
    let (traj, power_scale) = match crate::power::power_fit_quintic_with(
        cfg.power_budget.as_ref(), &arm_cfg.arm_model, arm_cfg.arm_dof(), traj.clone(),
        &start, &snap.commanded_velocities, &target, &limits, !is_park,
    ) {
        Ok(pair) => pair,
        Err(_) if is_park => (traj, None),
        Err(e) => return Err(e),
    };

    if let Some(ws) = cfg.workspace.as_ref() {
        if is_park {
            let rest_v = arm_cfg.arm_model.workspace_violation_with_tcp(&target, ws, &arm_cfg.camera_mount, arm_cfg.tcp_offset);
            if rest_v.total > WORKSPACE_EPS {
                return Err(rest_v.worst.map_or_else(
                    || SafetyError::from_workspace(crate::chain::WorkspaceFailure::invalid("park", "resting pose violates workspace")),
                    |w| SafetyError::WorkspaceViolation {
                        limit: w.label(ws), value: w.value as f32, bound: w.bound as f32, point: w.point,
                        detail: Box::new(crate::kinematics::WorkspaceViolationDetail {
                            constraint_id: Some(w.keepout.and_then(|bi| ws.keepout_boxes.get(bi)).map_or_else(|| w.label(ws), |kb| format!("keepout:{}", kb.name))),
                            geometry_id: Some(if w.point == 0 { "end_origin".into() } else { format!("end_point:{}", w.point) }),
                            violation_phase: Some("park".into()),
                            reason: Some(format!("resting pose violates workspace ({})", w.label(ws))),
                        }),
                    },
                ).into());
            }
        }
        let eps = if is_park { WORKSPACE_PARK_ESCAPE_EPS } else { WORKSPACE_EPS };
        let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
        let at = |t: f64| traj.sample(t).positions;
        let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
        let map_ws = |e: SafetyError| ArmError::Safety(if is_park { e.with_workspace_phase("park") } else { e });
        let samples = keepout_sweep_with_tcp_tol(&arm_cfg.arm_model, ws, &arm_cfg.camera_mount, arm_cfg.tcp_offset, eps, &at, &travel, &knots, MAX_KEEPOUT_EVALS_PER_PLAN).map_err(map_ws)?;
        check_path_with_tcp_tol(&arm_cfg.arm_model, ws, &arm_cfg.camera_mount, arm_cfg.tcp_offset, eps, &start, samples.iter().map(Vec::as_slice)).map_err(map_ws)?;
    }

    let (mut s_arr, mut v_arr, mut r_arr, mut t_arr, mut vc_arr, mut p_arr) = (
        [0.0f32; MAX_MOTION_DOF],
        [0.0f32; MAX_MOTION_DOF],
        [0.0f32; MAX_MOTION_DOF],
        [0.0f32; MAX_MOTION_DOF],
        [0.0f32; MAX_MOTION_DOF],
        [zero_poly(); INLINE_POLYS],
    );
    if traj.polynomials.len() != dof || dof > INLINE_POLYS {
        return Err(MotionError::InvalidCommand(format!(
            "planned trajectory polynomial count {} does not match profile DOF {dof} (inline max {INLINE_POLYS})",
            traj.polynomials.len()
        )).into());
    }
    s_arr[..dof].copy_from_slice(&start);
    v_arr[..dof].copy_from_slice(&snap.commanded_velocities);
    r_arr[..dof].copy_from_slice(&requested_unfiltered);
    t_arr[..dof].copy_from_slice(&target);
    for i in 0..dof { vc_arr[i] = limits[i].max_velocity; p_arr[i] = traj.polynomials[i]; }
    Ok(PreparedMotionPlan {
        slot: PreparedMotionSlot {
            kind: req.kind, dof: dof as u8, start: s_arr, v0: v_arr, requested_target: r_arr, target: t_arr, vel_caps: vc_arr,
            has_speed_scale: req.speed_scale.is_some(), speed_scale: req.speed_scale.unwrap_or(1.0),
            polynomials: p_arr, duration: traj.duration, power_scale,
            power_budget_hash: cfg.power_budget_hash, tcp_offset: arm_cfg.tcp_offset,
            checked_workspace: cfg.workspace.is_some(), workspace_revision: snap.workspace_revision,
            motion_config_revision: snap.motion_config_revision, authority: snap.authority,
            for_holding_after_clear: false,
            for_mit_deferral: false, continuation_action: None,
        },
        heap_traj: None,
    })
}

/// Non-consuming Lock #1 preflight delegating to single-authority `verify_motion_authority`.
pub fn preflight_motion_request(inner: &SupervisorInner, stamp: AdmissionStamp, req: &MotionRequest, expected_epoch: Option<u64>) -> Result<MotionPlanSnapshot, ArmError> {
    preflight_motion_request_at(inner, stamp, req, expected_epoch, Instant::now())
}

/// Like [`preflight_motion_request`], anchoring the bounded plan deadline to the request's
/// entry timestamp `request_at` and binding `PlanAuthority` to the request's admission stamp.
pub fn preflight_motion_request_at(
    inner: &SupervisorInner,
    stamp: AdmissionStamp,
    req: &MotionRequest,
    expected_epoch: Option<u64>,
    request_at: Instant,
) -> Result<MotionPlanSnapshot, ArmError> {
    if let Some(s) = req.speed_scale {
        if !s.is_finite() || !(0.05..=1.0).contains(&s) { return Err(MotionError::InvalidCommand("speed_scale must be in [0.05, 1.0]".into()).into()); }
    }
    inner.preflight_admission(stamp)?;
    let target: &[f32] = match req.kind {
        MotionKind::Jog => req.target_positions.as_deref().ok_or_else(|| MotionError::InvalidCommand("missing target_positions".into()))?,
        MotionKind::Home => &inner.config.home_positions,
        MotionKind::Park => inner.config.resting_positions(),
    };
    if let Err(err) = verify_motion_authority(inner, req.kind, req.speed_scale, target, expected_epoch, false) {
        if matches!(err, MotionInstallError::InvalidCommand | MotionInstallError::SoftLimitExceeded) && req.kind != MotionKind::Park {
            inner.safety.validate_motion_command(expected_epoch, target, &inner.config.limits)?;
        }
        return Err(err.into_arm_error(inner, req.action_kind, expected_epoch));
    }
    let authority = PlanAuthority {
        boot: inner.boot_id,
        epoch: expected_epoch.unwrap_or(stamp.estop_epoch),
        lease_generation: stamp.lease_gen,
        stop_generation: stamp.stop_gen,
        deadline: request_at + inner.motion_plan_deadline,
    };
    Ok(MotionPlanSnapshot::from_supervisor_with_authority(
        inner,
        Arc::new(MotionConfigSnapshot::from_supervisor(inner)),
        authority,
    ))
}

/// Single shared authority + deadline check for EVERY prepared motion install (`install_jog`,
/// `install_home`, `install_park`, and `install_park_continuation`) on both legacy and RT owners.
/// Enforces `boot` (fail-closed on `BootId(0)`), `deadline`, `stop_generation`, `epoch`, and
/// `lease_generation` without heap allocation.
pub fn verify_slot_authority(
    inner: &SupervisorInner,
    slot: &PreparedMotionSlot,
    expected_boot: BootId,
    expected_epoch: Option<u64>,
    now: Instant,
) -> Result<(), MotionInstallError> {
    let auth = slot.authority;
    if auth.boot.is_zero() || expected_boot.is_zero() || auth.boot != expected_boot || auth.boot != inner.boot_id {
        return Err(MotionInstallError::WrongTarget);
    }
    if now > auth.deadline {
        return Err(MotionInstallError::DeadlineExpired);
    }
    if inner.urgent_stop_unapplied() || auth.stop_generation != inner.current_stop_gen() {
        return Err(MotionInstallError::Stopped);
    }
    let live_epoch = inner.safety.epoch_id();
    if auth.epoch != live_epoch || expected_epoch.is_some_and(|e| e != live_epoch) {
        if inner.e_stop_latched.load(Ordering::SeqCst) || matches!(inner.safety.state(), SafetyState::EStop) {
            return Err(MotionInstallError::EStopActive);
        }
        return Err(MotionInstallError::EpochMismatch);
    }
    let live_lease = inner.lease_valid_at(now).map_or(0, |l| l.generation);
    if auth.lease_generation != live_lease {
        return Err(MotionInstallError::LeaseLost);
    }
    Ok(())
}

fn check_common_blockers(inner: &SupervisorInner) -> Result<(), MotionInstallError> {
    if inner.calibration_busy || !inner.can_replace_active_trajectory() || inner.gravity_comp.is_some() { return Err(MotionInstallError::MotionBusy); }
    #[cfg(not(target_arch = "wasm32"))]
    if inner.sessions.blocks_ordinary_output(None) { return Err(MotionInstallError::MotionBusy); }
    if inner.core.motion_inhibited_by_config() { return Err(MotionInstallError::ConfigBusy); }
    if inner.collision.hold.is_some() { return Err(MotionInstallError::CollisionHold); }
    if inner.handover_settling_since.is_some() { return Err(MotionInstallError::HandoverSettling); }
    if inner.shutdown_committed || inner.e_stop_latched.load(Ordering::SeqCst) { return Err(MotionInstallError::EStopActive); }
    Ok(())
}

/// Zero-allocation, side-effect-free authority & safety validation for jog/home/park commits.
pub fn verify_motion_authority(inner: &SupervisorInner, kind: MotionKind, speed_scale: Option<f64>, target: &[f32], expected_epoch: Option<u64>, require_fresh_feedback: bool) -> Result<(), MotionInstallError> {
    check_commissioning(&inner.config, inner.driver.is_some(), inner.driver_caps, kind, speed_scale, target, &inner.commanded_positions)?;
    inner.check_terminal_fifo_for_new_action().map_err(map_admission_err)?;
    check_common_blockers(inner)?;
    if kind != MotionKind::Park {
        if inner.park_continuation.is_some() || inner.mit.park_after_hand_back || inner.mit.revoke_pending
            || (inner.is_torqued && crate::mit::mit_joint_mask(&inner.config) != 0 && !inner.mit_active())
        { return Err(MotionInstallError::MotionBusy); }
        #[cfg(feature = "policy-runtime")]
        if inner.policy_loading || inner.policy_run.is_some() { return Err(MotionInstallError::MotionBusy); }
    } else if !inner.supports_automatic_unload() {
        return Err(MotionInstallError::UnsupportedUnload);
    }
    if inner.disable_pending
        || inner.torque_transition.is_some()
        || inner.park_continuation.is_some()
        || inner.mit.park_after_hand_back
        || inner.watchdog_park_pending.is_some()
        || inner.macro_run.as_ref().is_some_and(|m| m.finishing)
    {
        return Err(MotionInstallError::MotionBusy);
    }
    if expected_epoch.is_some_and(|ep| ep != inner.safety.epoch_id()) { return Err(MotionInstallError::EpochMismatch); }
    match inner.safety.state() {
        SafetyState::EStop => return Err(MotionInstallError::EStopActive),
        SafetyState::Fault(_) => return Err(MotionInstallError::FaultActive),
        s if kind == MotionKind::Park => {
            if !inner.holding_after_clear() && (!s.is_operational() || !inner.is_torqued || inner.pending_rearm || inner.pending_normal_unload) {
                return Err(MotionInstallError::NotArmed);
            }
        }
        s if !s.is_operational() || inner.pending_rearm || inner.pending_normal_unload => return Err(MotionInstallError::NotArmed),
        _ => {}
    }
    if require_fresh_feedback && !inner.commit_feedback_valid() { return Err(MotionInstallError::FeedbackStale); }
    if kind != MotionKind::Park {
        let dof = inner.config.dof;
        if target.len() != dof || inner.config.limits.len() != dof { return Err(MotionInstallError::InvalidCommand); }
        for (&val, lim) in target.iter().zip(&inner.config.limits) {
            if !val.is_finite() { return Err(MotionInstallError::InvalidCommand); }
            if val < lim.min_position || val > lim.max_position { return Err(MotionInstallError::SoftLimitExceeded); }
        }
        if inner.config.gripper_index().is_some_and(|g| {
            target.len() > g
                && inner.commanded_positions.len() > g
                && (target[g] - inner.commanded_positions[g]).abs() > 1e-4
        }) {
            inner.check_pos_force_gripper_fresh_static().map_err(|_| MotionInstallError::J7FeedbackStale)?;
        }
    }
    Ok(())
}

/// Zero-allocation exact freshness check separating `StartDrifted` from `PlanStale`.
pub fn verify_motion_freshness(inner: &SupervisorInner, slot: &PreparedMotionSlot) -> Result<(Option<usize>, Option<usize>), MotionInstallError> {
    let dof = inner.config.dof;
    if usize::from(slot.dof) != dof || dof == 0 || dof > MAX_MOTION_DOF || !slot.duration.is_finite() || slot.duration <= 0.0 {
        return Err(MotionInstallError::InvalidCommand);
    }
    if slot.workspace_revision != inner.workspace_revision || slot.motion_config_revision != inner.motion_config_revision
        || slot.tcp_offset != inner.config.tcp_offset || slot.checked_workspace != inner.workspace_policy().is_some()
        || slot.power_budget_hash != power_budget_fingerprint(inner.power_budget())
    { return Err(MotionInstallError::PlanStale); }
    let Some((limits, lim_n)) = inner.planning_limits_inline(slot.speed_scale_opt()) else { return Err(MotionInstallError::InvalidCommand); };
    if lim_n != dof || (0..dof).any(|i| (limits[i].max_velocity - slot.vel_caps[i]).abs() > EXACT_START_TOL) {
        return Err(MotionInstallError::PlanStale);
    }
    let notes = match slot.kind {
        MotionKind::Park => {
            let rest = inner.config.resting_positions();
            if rest.len() != dof || (0..dof).any(|i| (rest[i] - slot.target[i]).abs() > EXACT_START_TOL) { return Err(MotionInstallError::PlanStale); }
            (None, None)
        }
        MotionKind::Home | MotionKind::Jog => {
            let mut filtered = [0.0f32; MAX_MOTION_DOF];
            if slot.kind == MotionKind::Home {
                if inner.config.home_positions.len() != dof { return Err(MotionInstallError::PlanStale); }
                filtered[..dof].copy_from_slice(&inner.config.home_positions);
            } else {
                filtered[..dof].copy_from_slice(&slot.requested_target[..dof]);
            }
            let notes = inner.preview_hold_filters(&mut filtered[..dof]);
            if (0..dof).any(|i| (filtered[i] - slot.target[i]).abs() > EXACT_START_TOL) { return Err(MotionInstallError::PlanStale); }
            notes
        }
    };
    let clamp = slot.kind == MotionKind::Park;
    if inner.commanded_positions.len() != dof || inner.commanded_velocities.len() != dof { return Err(MotionInstallError::StartDrifted); }
    let mut eff_pos = [0.0f32; MAX_DOF];
    let (eff_dof, quiesce_macro_gripper) = if !clamp && dof <= MAX_DOF {
        inner.post_quiesce_commanded_positions(&mut eff_pos)
    } else {
        (0, false)
    };
    let gripper_idx = inner.config.gripper_index();
    for i in 0..dof {
        let raw_cur = if !clamp && i < eff_dof {
            eff_pos[i]
        } else {
            inner.commanded_positions[i]
        };
        let cur = if clamp {
            inner.config.limits.get(i).map_or(raw_cur, |l| raw_cur.clamp(l.min_position, l.max_position))
        } else {
            raw_cur
        };
        let cur_vel = if quiesce_macro_gripper && gripper_idx == Some(i) {
            0.0
        } else {
            inner.commanded_velocities[i]
        };
        if (cur - slot.start[i]).abs() >= crate::rt_core::ops::START_TOLERANCE
            || (cur_vel - slot.v0[i]).abs() > EXACT_START_TOL
        {
            return Err(MotionInstallError::StartDrifted);
        }
    }
    Ok(notes)
}

fn commit_quintic_slot(
    inner: &mut SupervisorInner,
    stamp: AdmissionStamp,
    slot: &PreparedMotionSlot,
    heap_traj: Option<MultiJointQuinticTrajectory>,
    expected_epoch: Option<u64>,
    require_fresh_feedback: bool,
    expected_boot: BootId,
    now: Instant,
    action_kind: &'static str,
) -> Result<MotionInstalled, MotionInstallError> {
    let dof = usize::from(slot.dof);
    if dof == 0 || dof > MAX_MOTION_DOF { return Err(MotionInstallError::InvalidCommand); }
    verify_slot_authority(inner, slot, expected_boot, expected_epoch, now)?;
    verify_motion_authority(inner, slot.kind, slot.speed_scale_opt(), &slot.target[..dof], expected_epoch, require_fresh_feedback)?;
    let (torque_clamped, power_clamped) = verify_motion_freshness(inner, slot)?;
    inner.preflight_admission(stamp).map_err(map_admission_err)?;
    inner.check_terminal_fifo_for_new_action().map_err(map_admission_err)?;
    let mut eff_pos = [0.0f32; MAX_DOF];
    let (_, quiesce_macro_gripper) = if dof <= MAX_DOF {
        inner.post_quiesce_commanded_positions(&mut eff_pos)
    } else {
        (0, false)
    };

    let (this, _proof) = inner.check_admission(stamp).map_err(map_admission_err)?.consume().map_err(map_admission_err)?;
    if quiesce_macro_gripper {
        this.hold_gripper_at_measured();
        this.releasing = false;
    }
    this.pending_normal_unload = false;
    this.commit_hold_filter_notes(torque_clamped, power_clamped);
    let target = &slot.target[..dof];
    if let Some(g) = this.config.gripper_index() {
        if this.gripper_stalled && target.len() > g && this.commanded_positions.len() > g {
            let (delta, tau_g) = (target[g] - this.commanded_positions[g], this.torques.get(g).copied().unwrap_or(0.0));
            if (delta * tau_g < -1e-4) || (delta * this.gripper_stall_sign < -1e-4) {
                this.gripper_stalled = false; this.gripper_high_torque_ticks = 0; this.gripper_low_torque_ticks = 0; this.gripper_stall_window.reset();
            }
        }
        if target.len() > g && (target[g] - this.commanded_positions[g]).abs() > 1e-4 { this.gripper_hold_vel_ceiling = None; }
    }
    this.last_heartbeat = Instant::now();
    this.release_servo_target();
    this.servo_settled_ticks.fill(0);
    this.retire_servo_ws_min_depths();
    copy_into(&mut this.target_positions, target);
    this.safety.mark_motion_started().map_err(map_admission_err)?;
    let id = this.begin_action_rt(action_kind);
    if let Some(s) = slot.power_scale { this.note_action_power_scale(id, s); }
    this.replace_active_trajectory(Some(slot.active_trajectory(Some(id), heap_traj)));
    this.prepared_quintics_used += 1;
    Ok(MotionInstalled::Started { epoch: this.safety.epoch_id(), action_id: id, expected_duration_s: slot.duration })
}

pub fn install_jog(inner: &mut SupervisorInner, stamp: AdmissionStamp, slot: &PreparedMotionSlot, heap_traj: Option<MultiJointQuinticTrajectory>, expected_epoch: Option<u64>, require_fresh_feedback: bool) -> Result<MotionInstalled, MotionInstallError> {
    let boot = inner.boot_id;
    install_jog_at(inner, stamp, slot, heap_traj, expected_epoch, require_fresh_feedback, boot, Instant::now())
}

pub fn install_jog_at(inner: &mut SupervisorInner, stamp: AdmissionStamp, slot: &PreparedMotionSlot, heap_traj: Option<MultiJointQuinticTrajectory>, expected_epoch: Option<u64>, require_fresh_feedback: bool, expected_boot: BootId, now: Instant) -> Result<MotionInstalled, MotionInstallError> {
    if slot.kind != MotionKind::Jog { return Err(MotionInstallError::InvalidCommand); }
    commit_quintic_slot(inner, stamp, slot, heap_traj, expected_epoch, require_fresh_feedback, expected_boot, now, "joint_move")
}

pub fn install_home(inner: &mut SupervisorInner, stamp: AdmissionStamp, slot: &PreparedMotionSlot, heap_traj: Option<MultiJointQuinticTrajectory>, expected_epoch: Option<u64>, require_fresh_feedback: bool) -> Result<MotionInstalled, MotionInstallError> {
    let boot = inner.boot_id;
    install_home_at(inner, stamp, slot, heap_traj, expected_epoch, require_fresh_feedback, boot, Instant::now())
}

pub fn install_home_at(inner: &mut SupervisorInner, stamp: AdmissionStamp, slot: &PreparedMotionSlot, heap_traj: Option<MultiJointQuinticTrajectory>, expected_epoch: Option<u64>, require_fresh_feedback: bool, expected_boot: BootId, now: Instant) -> Result<MotionInstalled, MotionInstallError> {
    if slot.kind != MotionKind::Home { return Err(MotionInstallError::InvalidCommand); }
    commit_quintic_slot(inner, stamp, slot, heap_traj, expected_epoch, require_fresh_feedback, expected_boot, now, "home")
}

pub fn install_park(inner: &mut SupervisorInner, stamp: AdmissionStamp, slot: &PreparedMotionSlot, heap_traj: Option<MultiJointQuinticTrajectory>, expected_epoch: Option<u64>, require_fresh_feedback: bool) -> Result<MotionInstalled, MotionInstallError> {
    let boot = inner.boot_id;
    install_park_at(inner, stamp, slot, heap_traj, expected_epoch, require_fresh_feedback, boot, Instant::now())
}

pub fn install_park_at(inner: &mut SupervisorInner, stamp: AdmissionStamp, slot: &PreparedMotionSlot, heap_traj: Option<MultiJointQuinticTrajectory>, expected_epoch: Option<u64>, require_fresh_feedback: bool, expected_boot: BootId, now: Instant) -> Result<MotionInstalled, MotionInstallError> {
    if slot.kind != MotionKind::Park { return Err(MotionInstallError::InvalidCommand); }
    let dof = usize::from(slot.dof);
    if dof == 0 || dof > MAX_MOTION_DOF { return Err(MotionInstallError::InvalidCommand); }
    verify_slot_authority(inner, slot, expected_boot, expected_epoch, now)?;
    verify_motion_authority(inner, MotionKind::Park, None, &slot.target[..dof], expected_epoch, require_fresh_feedback)?;
    let holding = inner.holding_after_clear();
    if holding {
        if !slot.for_holding_after_clear || slot.workspace_revision != inner.workspace_revision || slot.motion_config_revision != inner.motion_config_revision {
            return Err(MotionInstallError::PlanStale);
        }
    } else if slot.for_holding_after_clear {
        return Err(MotionInstallError::PlanStale);
    } else if slot.for_mit_deferral {
        if slot.workspace_revision != inner.workspace_revision || slot.motion_config_revision != inner.motion_config_revision {
            return Err(MotionInstallError::PlanStale);
        }
        if !inner.mit_active() {
            return Err(MotionInstallError::StartDrifted);
        }
    } else if !inner.mit_active() {
        let _ = verify_motion_freshness(inner, slot)?;
    } else if slot.workspace_revision != inner.workspace_revision || slot.motion_config_revision != inner.motion_config_revision {
        return Err(MotionInstallError::PlanStale);
    }
    inner.preflight_admission(stamp).map_err(map_admission_err)?;
    inner.check_terminal_fifo_for_new_action().map_err(map_admission_err)?;

    let (this, proof) = inner.check_admission(stamp).map_err(map_admission_err)?.consume().map_err(map_admission_err)?;
    let (epoch, stop_gen, lease_gen, ws_rev, mc_rev) = (this.safety.epoch_id(), stamp.stop_gen, this.active_lease.as_ref().map(|l| l.generation), this.workspace_revision, this.motion_config_revision);

    if holding {
        let tentative_id = this.next_action_id + 1;
        let rearm = this.begin_park_rearm_transition(&proof, tentative_id, stop_gen)?;
        let id = this.begin_action_rt("park");
        debug_assert_eq!(id, tentative_id);
        let phase = match rearm { ParkRearmStart::ConfirmedNow => ParkContinuationPhase::ReadyForPlan, ParkRearmStart::Pending => ParkContinuationPhase::WaitingRearm };
        this.park_continuation = Some(ParkContinuation { action_id: id, phase, epoch: this.safety.epoch_id(), stop_gen, lease_gen, workspace_revision: ws_rev, motion_config_revision: mc_rev, deadline: slot.authority.deadline });
        if phase == ParkContinuationPhase::ReadyForPlan { this.notify_park_continuation(); }
        return Ok(MotionInstalled::DeferredContinuation { epoch: this.safety.epoch_id(), action_id: id, expected_duration_s: PARK_DURATION_S + PARK_EXTRA_DURATION_S });
    }

    if this.mit_active() {
        // Never invoke blocking `mit_hand_back` / `set_joint_modes_confirmed` under the WS write
        // lock: defer immediately to the control tick, which brakes/hands back off the WS lock
        // and advances `park_continuation` to `ReadyForPlan`.
        this.replace_active_trajectory(None);
        this.release_servo_target();
        this.servo_stream_active = false;
        this.mit.park_after_hand_back = true;
        this.mit.deferred_park_off_owner = true;
        let id = this.begin_action_rt("park");
        this.park_continuation = Some(ParkContinuation { action_id: id, phase: ParkContinuationPhase::WaitingMitHandback, epoch, stop_gen, lease_gen, workspace_revision: ws_rev, motion_config_revision: mc_rev, deadline: slot.authority.deadline });
        return Ok(MotionInstalled::DeferredContinuation { epoch, action_id: id, expected_duration_s: PARK_DURATION_S + PARK_EXTRA_DURATION_S });
    }
    install_park_trajectory(this, ParkActionMode::New, slot, heap_traj)
}

pub fn install_park_continuation(inner: &mut SupervisorInner, action_id: u64, slot: &PreparedMotionSlot, heap_traj: Option<MultiJointQuinticTrajectory>, expected_epoch: Option<u64>, require_fresh_feedback: bool) -> Result<MotionInstalled, MotionInstallError> {
    let boot = inner.boot_id;
    install_park_continuation_at(inner, action_id, slot, heap_traj, expected_epoch, require_fresh_feedback, boot, Instant::now())
}

pub fn install_park_continuation_at(inner: &mut SupervisorInner, action_id: u64, slot: &PreparedMotionSlot, heap_traj: Option<MultiJointQuinticTrajectory>, expected_epoch: Option<u64>, require_fresh_feedback: bool, expected_boot: BootId, now: Instant) -> Result<MotionInstalled, MotionInstallError> {
    if slot.kind != MotionKind::Park || slot.for_holding_after_clear || slot.for_mit_deferral { return Err(MotionInstallError::InvalidCommand); }
    // Bound continuation authority by the ORIGINAL park request deadline stored in `ParkContinuation`
    // so a separately timed `ContinuePark` or freshly prepared slot cannot outlive the original park.
    let mut bounded_slot = *slot;
    if let Some(cont) = inner.park_continuation.filter(|c| c.action_id == action_id) {
        bounded_slot.authority.deadline = bounded_slot.authority.deadline.min(cont.deadline);
    }
    verify_slot_authority(inner, &bounded_slot, expected_boot, expected_epoch, now)?;
    let Some(cont) = inner.park_continuation.filter(|c| c.action_id == action_id && c.phase == ParkContinuationPhase::ReadyForPlan) else {
        return Err(MotionInstallError::ContinuationMismatch);
    };
    if inner.urgent_stop_unapplied() || cont.stop_gen != inner.current_stop_gen() {
        return Err(MotionInstallError::Stopped);
    }
    if !inner.last_action.as_ref().is_some_and(|a| a.id == action_id && a.is_running() && a.kind == "park") {
        return Err(MotionInstallError::ContinuationMismatch);
    }
    check_common_blockers(inner)?;
    match inner.safety.state() {
        SafetyState::EStop => return Err(MotionInstallError::EStopActive),
        SafetyState::Fault(_) => return Err(MotionInstallError::FaultActive),
        s if !s.is_operational() => return Err(MotionInstallError::NotArmed),
        _ => {}
    }
    if inner.safety.epoch_id() != cont.epoch || expected_epoch.is_some_and(|e| e != cont.epoch) { return Err(MotionInstallError::EpochMismatch); }
    let live_lease_gen = if inner.lease_enabled {
        inner.lease_valid_at(now).map(|l| l.generation)
    } else {
        inner.active_lease.as_ref().map(|l| l.generation)
    };
    if live_lease_gen != cont.lease_gen { return Err(MotionInstallError::LeaseLost); }
    if inner.workspace_revision != cont.workspace_revision || inner.motion_config_revision != cont.motion_config_revision {
        return Err(MotionInstallError::PlanStale);
    }
    if !inner.is_torqued || inner.pending_rearm || inner.disable_pending || inner.torque_transition.is_some() || inner.mit_active() {
        return Err(MotionInstallError::MotionBusy);
    }
    if require_fresh_feedback && !inner.commit_feedback_valid() { return Err(MotionInstallError::FeedbackStale); }
    let _ = verify_motion_freshness(inner, slot)?;
    inner.park_continuation = None;
    install_park_trajectory(inner, ParkActionMode::Existing(action_id), slot, heap_traj)
}

/// System stop-direction watchdog park install (exempt from client `Admitted` / lease, still
/// prepared off-thread and installed through the single shared `install_park_trajectory` path).
/// Zero heap allocations and zero frees on `armd-control`.
pub fn install_watchdog_park(
    inner: &mut SupervisorInner,
    generation: u64,
    slot: &PreparedMotionSlot,
    heap_traj: Option<MultiJointQuinticTrajectory>,
) -> Result<MotionInstalled, MotionInstallError> {
    if slot.kind != MotionKind::Park || slot.for_holding_after_clear || slot.for_mit_deferral {
        return Err(MotionInstallError::InvalidCommand);
    }
    let Some(pending) = inner.watchdog_park_pending.filter(|p| p.generation == generation && !p.waiting_mit_handback) else {
        return Err(MotionInstallError::ContinuationMismatch);
    };
    if inner.watchdog_park.is_none() {
        return Err(MotionInstallError::ContinuationMismatch);
    }
    if slot.authority.boot.is_zero() || slot.authority.boot != inner.boot_id {
        return Err(MotionInstallError::WrongTarget);
    }
    if inner.e_stop_latched.load(Ordering::SeqCst) || inner.shutdown_committed {
        return Err(MotionInstallError::EStopActive);
    }
    match inner.safety.state() {
        SafetyState::EStop => return Err(MotionInstallError::EStopActive),
        SafetyState::Fault(_) => return Err(MotionInstallError::FaultActive),
        s if !s.is_operational() => return Err(MotionInstallError::NotArmed),
        _ => {}
    }
    if inner.safety.epoch_id() != pending.epoch {
        return Err(MotionInstallError::EpochMismatch);
    }
    if inner.workspace_revision != pending.workspace_revision || inner.motion_config_revision != pending.motion_config_revision {
        return Err(MotionInstallError::PlanStale);
    }
    if !inner.is_torqued || inner.pending_rearm || inner.disable_pending || inner.torque_transition.is_some()
        || inner.mit_active() || inner.handover_settling_since.is_some() || inner.collision.hold.is_some()
        || inner.core.motion_inhibited_by_config() || inner.calibration_busy
    {
        return Err(MotionInstallError::MotionBusy);
    }
    if !inner.supports_automatic_unload() {
        return Err(MotionInstallError::UnsupportedUnload);
    }
    let _ = verify_motion_freshness(inner, slot)?;
    #[cfg(not(target_arch = "wasm32"))]
    if inner.sessions.blocks_ordinary_output(None) {
        // Safety watchdog park preempts any ordinary non-safety session cleanly.
        inner.revoke_sessions_on_stop("LEASE_LOST");
        if inner.sessions.is_safe_hold_requested() {
            return Err(MotionInstallError::EStopActive);
        }
    }
    inner.watchdog_park_pending = None;
    let installed = install_park_trajectory(inner, ParkActionMode::Watchdog, slot, heap_traj)?;
    let budget = inner.watchdog_park_timeout();
    if let Some(park) = inner.watchdog_park.as_mut() {
        park.budget_s = budget.as_secs_f64();
    }
    Ok(installed)
}

/// Completes a failed off-thread watchdog park preparation by holding torque and latching
/// `Fault(WatchdogTimeout)` (zero heap allocations on `armd-control`).
pub fn fail_watchdog_park_plan(
    inner: &mut SupervisorInner,
    generation: u64,
    reason: crate::safety::BoundedText<160>,
) {
    if inner.watchdog_park_pending.is_some_and(|p| p.generation == generation) {
        inner.watchdog_park_pending = None;
        inner.watchdog_park = None;
        inner.safety.log_watchdog_park_blocked(reason);
        inner.latch_watchdog_fault();
    }
}

pub fn fail_park_continuation(inner: &mut SupervisorInner, action_id: u64, code: &'static str, text: &str) {
    if inner.park_continuation.is_some_and(|c| c.action_id == action_id) {
        inner.park_continuation = None;
        inner.mit.park_after_hand_back = false;
        inner.mit.deferred_park_off_owner = false;
        inner.abort_park_rearm_transition();
        let mut msg = inner.take_action_msg_buf(action_id, 128);
        msg.clear(); msg.push_str(text);
        inner.finish_action(action_id, "failed", Some(code), Some(msg), None, None, None);
        inner.notify_park_continuation();
    }
}

enum ParkActionMode { New, Existing(u64), Watchdog }

fn install_park_trajectory(this: &mut SupervisorInner, mode: ParkActionMode, slot: &PreparedMotionSlot, heap_traj: Option<MultiJointQuinticTrajectory>) -> Result<MotionInstalled, MotionInstallError> {
    let dof = usize::from(slot.dof);
    this.release_servo_target();
    this.servo_settled_ticks.fill(0);
    this.retire_servo_ws_min_depths();
    this.servo_stream_active = false;
    let _ = this.safety.set_torque_rt(true);
    let _ = this.safety.mark_motion_started();
    this.unload_planned_duration = Some(slot.duration);
    let action_id = match mode {
        ParkActionMode::New => Some(this.begin_action_rt("park")),
        ParkActionMode::Existing(id) => Some(id),
        ParkActionMode::Watchdog => None,
    };
    if let (Some(id), Some(s)) = (action_id, slot.power_scale) { this.note_action_power_scale(id, s); }
    this.replace_active_trajectory(Some(slot.active_trajectory(action_id, heap_traj)));
    copy_into(&mut this.target_positions, &slot.target[..dof]);
    this.pending_normal_unload = true;
    this.clear_unload_settle();
    this.park_gripper_contact = false;
    this.park_release = None;
    this.prepared_quintics_used += 1;
    let now = Instant::now();
    this.last_heartbeat = now;
    this.last_motion_frame = now;
    Ok(MotionInstalled::Started { epoch: this.safety.epoch_id(), action_id: action_id.unwrap_or(0), expected_duration_s: slot.duration + PARK_EXTRA_DURATION_S })
}

fn stage_slot(slot: &mut PlanSlot, prepared: &PreparedMotionSlot, kind: MotionKind, continuation_action: Option<u64>) {
    let mut s = *prepared;
    s.kind = kind;
    s.continuation_action = continuation_action;
    let n = usize::from(s.dof).min(MAX_DOF);
    let (mut p_start, mut p_q) = ([0.0f32; MAX_DOF], [0.0f32; MAX_DOF]);
    p_start[..n].copy_from_slice(&s.start[..n]);
    p_q[..n].copy_from_slice(&s.target[..n]);
    slot.set(Primitive::JointQuintic { dof: s.dof, start: p_start, q: p_q, duration_s: s.duration });
    slot.motion = Some(s);
}

pub fn stage_jog(slot: &mut PlanSlot, prepared: &PreparedMotionSlot) { stage_slot(slot, prepared, MotionKind::Jog, None); }
pub fn stage_home(slot: &mut PlanSlot, prepared: &PreparedMotionSlot) { stage_slot(slot, prepared, MotionKind::Home, None); }
pub fn stage_park(slot: &mut PlanSlot, prepared: &PreparedMotionSlot) { stage_slot(slot, prepared, MotionKind::Park, None); }
pub fn stage_park_continuation(slot: &mut PlanSlot, action_id: u64, prepared: &PreparedMotionSlot) { stage_slot(slot, prepared, MotionKind::Park, Some(action_id)); }

fn commit_broker(inner: &mut SupervisorInner, d: &Descriptor, ctx: &CommitCtx, f: impl FnOnce(&mut SupervisorInner, AdmissionStamp, u64) -> Result<MotionInstalled, MotionInstallError>) -> Result<Committed, &'static str> {
    inner.check_commit_for_motion(d, ctx)?;
    inner.sync_stop_generation(ctx.stop_generation);
    let stamp = AdmissionStamp::from_broker_conn(
        d.stamp.authority.stop_generation,
        d.stamp.authority.epoch,
        d.stamp.authority.lease_generation,
        d.stamp.client.0,
        d.stamp.request.0,
    );
    let installed = f(inner, stamp, d.stamp.authority.epoch).map_err(MotionInstallError::code)?;
    Ok(Committed::authorized(d, Some(ActionId(installed.action_id()))))
}

pub(crate) fn commit_install_slot(inner: &mut SupervisorInner, d: &Descriptor, slot: &PreparedMotionSlot, ctx: &CommitCtx) -> Result<Committed, &'static str> {
    commit_broker(inner, d, ctx, |i, stamp, ep| install_jog_at(i, stamp, slot, None, Some(ep), true, ctx.boot, ctx.now))
}
pub(crate) fn commit_home(inner: &mut SupervisorInner, d: &Descriptor, slot: &PreparedMotionSlot, ctx: &CommitCtx) -> Result<Committed, &'static str> {
    commit_broker(inner, d, ctx, |i, stamp, ep| install_home_at(i, stamp, slot, None, Some(ep), true, ctx.boot, ctx.now))
}
pub(crate) fn commit_park(inner: &mut SupervisorInner, d: &Descriptor, slot: &PreparedMotionSlot, ctx: &CommitCtx) -> Result<Committed, &'static str> {
    let mut bounded = *slot;
    bounded.authority.deadline = bounded.authority.deadline.min(d.stamp.deadline);
    commit_broker(inner, d, ctx, |i, stamp, ep| install_park_at(i, stamp, &bounded, None, Some(ep), true, ctx.boot, ctx.now))
}
pub(crate) fn commit_park_continuation(inner: &mut SupervisorInner, d: &Descriptor, slot: &PreparedMotionSlot, ctx: &CommitCtx) -> Result<Committed, &'static str> {
    let Some(action_id) = slot.continuation_action else { return Err("INVALID_COMMAND"); };
    commit_broker(inner, d, ctx, |i, _, ep| install_park_continuation_at(i, action_id, slot, None, Some(ep), true, ctx.boot, ctx.now))
}
