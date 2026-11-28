//! Edge-triggered, allocation-free, lock-free terminal-action capture (`armd-control` -> `armd-dispatch`).
//!
//! Every authoritative `running -> !running` transition assigns a monotonic `terminal_seq`.
//! On the RT/guarded control path (`defer_terminal_capture || rt_executor_owned`), transitions
//! are staged into a fixed-size inline buffer (`StagedTerminalBuf`) and captured/flushed into a
//! preallocated SPSC `rtrb::RingBuffer<TerminalCaptureRecord>` inside `capture_edge_terminals`
//! strictly before `hub.publish` (zero heap allocations, zero driver mutexes, zero broadcaster mutexes).
//!
//! Off-thread (`armd-dispatch`, subscription registration, and off-tick Legacy producers),
//! `TerminalCaptureChannel::drain_into` holds `rx` across
//! `peek -> pop -> TerminalBroadcaster::record_finished_with_seq -> inflight.fetch_sub(1)`,
//! preserving strict `terminal_seq` FIFO ordering, end-to-end admission backpressure across
//! `staged -> inflight -> fifo`, and exact `publish_seq` synchronization bound to the consumed
//! `StateHub` snapshot's `meta.seq`.

use std::collections::{BTreeMap, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::model::{
    ArmObservation, ControlMode, DriverCapabilities, EnableState, GripperBaselineObs, GripperObs,
    HolderLabel, TcpState, MAX_MOTION_DOF,
};
use crate::motion::{ActionStatus, PolicyStats};
use crate::rt_core::publish::{Snapshot, StateHub};
use crate::safety::{BoundedFaultReason, BoundedText, FaultInfo, FaultReason, SafetyState};

use super::{SupervisorInner, TerminalBroadcaster, TERMINAL_FIFO_CAPACITY};

mod channel;
pub(crate) use channel::{
    compute_cached_feedback_state, StagedTerminal, StagedTerminalBuf,
    TerminalCaptureChannel, TerminalCaptureTx, TerminalEdgeTracker,
};
#[cfg(test)]
pub(crate) use channel::evaluate_cached_feedback_fresh;

/// Physical SPSC ring capacity (2x `TERMINAL_FIFO_CAPACITY` for in-flight headroom).
pub const TERMINAL_CAPTURE_RING_CAPACITY: usize = TERMINAL_FIFO_CAPACITY * 2;
/// Logical admission reservation ceiling matching `TERMINAL_FIFO_CAPACITY`.
pub const TERMINAL_CAPTURE_ADMISSION_CAP: usize = TERMINAL_FIFO_CAPACITY;
/// Inline joint-array width matching the maximum valid profile DOF (`MAX_MOTION_DOF`).
pub(crate) const CAPTURE_MAX_DOF: usize = MAX_MOTION_DOF;

/// Inline stack mirror of [`SafetyState`] for zero-allocation terminal capture.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum CapturedSafetyState {
    Booting,
    Idle,
    Armed,
    Moving,
    EStop,
    Fault(BoundedFaultReason),
}

impl CapturedSafetyState {
    #[inline]
    pub(crate) fn from_safety_state(s: &SafetyState) -> Self {
        match s {
            SafetyState::Booting => Self::Booting,
            SafetyState::Idle => Self::Idle,
            SafetyState::Armed => Self::Armed,
            SafetyState::Moving => Self::Moving,
            SafetyState::EStop => Self::EStop,
            SafetyState::Fault(r) => Self::Fault(BoundedFaultReason::from_reason(r)),
        }
    }

    pub(crate) fn to_safety_state(self) -> SafetyState {
        match self {
            Self::Booting => SafetyState::Booting,
            Self::Idle => SafetyState::Idle,
            Self::Armed => SafetyState::Armed,
            Self::Moving => SafetyState::Moving,
            Self::EStop => SafetyState::EStop,
            Self::Fault(r) => SafetyState::Fault(match r {
                BoundedFaultReason::JointLimitExceeded {
                    joint_idx,
                    joint_name,
                    value,
                    min,
                    max,
                } => FaultReason::JointLimitExceeded {
                    joint_idx,
                    joint_name: joint_name.as_str().to_owned(),
                    value,
                    min,
                    max,
                },
                BoundedFaultReason::ThermalTrip {
                    joint_idx,
                    temperature,
                    limit,
                } => FaultReason::ThermalTrip {
                    joint_idx,
                    temperature,
                    limit,
                },
                BoundedFaultReason::BusLoss | BoundedFaultReason::BusLossAnnotated(_) => FaultReason::BusLoss,
                BoundedFaultReason::WatchdogTimeout => FaultReason::WatchdogTimeout,
                BoundedFaultReason::CanTxStalled(_) => FaultReason::CanTxStalled,
                BoundedFaultReason::J7FeedbackStale(_) => FaultReason::J7FeedbackStale,
                BoundedFaultReason::Other(m) => FaultReason::Other(m.as_str().to_owned()),
            }),
        }
    }
}

/// Dedicated per-tick cached driver state owned exclusively by terminal capture so seeding at
/// `install_driver` or failing closed on mutex/read errors never mutates hardware park/torque-off
/// interlock fields (`last_poll_enable_state`, `last_poll_partial_enable`, `last_poll_oldest_age`).
#[derive(Debug, Clone, Copy)]
pub(crate) struct CaptureDriverCache {
    pub(crate) enable_state: EnableState,
    pub(crate) driver_caps: DriverCapabilities,
    pub(crate) feedback_valid: bool,
    pub(crate) ages: [Option<Duration>; CAPTURE_MAX_DOF],
    pub(crate) ages_len: u8,
    pub(crate) feedback_at: Option<Instant>,
}

impl Default for CaptureDriverCache {
    fn default() -> Self {
        Self {
            enable_state: EnableState::Unknown,
            driver_caps: DriverCapabilities::default(),
            feedback_valid: false,
            ages: [None; CAPTURE_MAX_DOF],
            ages_len: 0,
            feedback_at: None,
        }
    }
}

impl CaptureDriverCache {
    #[inline]
    pub(crate) fn copy_ages_from(&mut self, src: &[Option<Duration>], dof: usize, polled_at: Instant) {
        self.ages = [None; CAPTURE_MAX_DOF];
        let n = src.len().min(dof).min(CAPTURE_MAX_DOF);
        self.ages[..n].copy_from_slice(&src[..n]);
        self.ages_len = n as u8;
        self.feedback_at = (n > 0 && self.ages[..n].iter().any(Option::is_some)).then_some(polled_at);
    }

    #[inline]
    pub(crate) fn clear_ages(&mut self) {
        self.ages = [None; CAPTURE_MAX_DOF];
        self.ages_len = 0;
        self.feedback_at = None;
    }
}

/// `Copy` inline mirror of [`GripperObs`] so `TerminalCaptureRecord` stays `Copy` without
/// modifying `GripperObs`'s public trait impls.
#[derive(Debug, Clone, Copy)]
pub(crate) struct CapturedGripperObs {
    pub(crate) joint: usize,
    pub(crate) rad: f32,
    pub(crate) fraction: f32,
    pub(crate) opening_m: Option<f32>,
    pub(crate) stalled: bool,
    pub(crate) force_limited: bool,
    pub(crate) force_limit_trips: u64,
    pub(crate) baseline: Option<GripperBaselineObs>,
}

impl CapturedGripperObs {
    #[inline]
    fn to_gripper_obs(self) -> GripperObs {
        GripperObs {
            joint: self.joint,
            rad: self.rad,
            fraction: self.fraction,
            opening_m: self.opening_m,
            stalled: self.stalled,
            force_limited: self.force_limited,
            force_limit_trips: self.force_limit_trips,
            baseline: self.baseline,
        }
    }
}

/// `Copy` inline mirror of [`TcpState`] so synthesized observations recompute TCP pose from the
/// same captured joint sample rather than retaining a stale snapshot's pose.
#[derive(Debug, Clone, Copy)]
pub(crate) struct CapturedTcpState {
    pub(crate) xyz_m: [f32; 3],
    pub(crate) rpy_rad: [f32; 3],
    pub(crate) quat_wxyz: [f32; 4],
}

impl CapturedTcpState {
    #[inline]
    fn to_tcp_state(self) -> TcpState {
        TcpState {
            xyz_m: self.xyz_m,
            rpy_rad: self.rpy_rad,
            quat_wxyz: self.quat_wxyz,
        }
    }
}

/// Fixed-size `Copy` terminal record captured on `armd-control` with zero heap ops and zero locks.
#[derive(Clone, Copy)]
pub(crate) struct TerminalCaptureRecord {
    pub(crate) terminal_seq: u64,
    pub(crate) publish_seq: Option<u64>,
    pub(crate) action_id: u64,
    pub(crate) kind: HolderLabel,
    pub(crate) state: HolderLabel,
    pub(crate) code: Option<HolderLabel>,
    pub(crate) has_message: bool,
    pub(crate) message: BoundedText<256>,
    pub(crate) tag: Option<HolderLabel>,
    pub(crate) task: Option<HolderLabel>,
    pub(crate) sequence_id: Option<HolderLabel>,
    pub(crate) step_type: Option<HolderLabel>,
    pub(crate) phase: Option<HolderLabel>,
    pub(crate) episode: Option<HolderLabel>,
    pub(crate) constraint_id: Option<HolderLabel>,
    pub(crate) geometry_id: Option<HolderLabel>,
    pub(crate) violation_phase: Option<HolderLabel>,
    pub(crate) reason: Option<HolderLabel>,
    /// Bitmask of truncated workspace labels (bit 0: constraint_id, 1: geometry_id,
    /// 2: violation_phase, 3: reason) so UTF-8 boundary truncation is exact.
    pub(crate) label_truncated_mask: u8,
    pub(crate) durability_confirmed: Option<bool>,
    pub(crate) has_durability_warning: bool,
    pub(crate) durability_warning: BoundedText<256>,
    pub(crate) step_index: Option<usize>,
    pub(crate) failed_step: Option<usize>,
    pub(crate) repeat_index: Option<u32>,
    pub(crate) repeats: Option<u32>,
    pub(crate) progress: f32,
    pub(crate) started_at: f64,
    pub(crate) finished_at: Option<f64>,
    pub(crate) obs_timestamp: f64,
    pub(crate) holding: Option<bool>,
    pub(crate) gripper_gap_rad: Option<f32>,
    pub(crate) settled: Option<bool>,
    pub(crate) t_s: Option<f32>,
    pub(crate) power_scale: Option<f32>,
    pub(crate) stop_rad: Option<f32>,
    pub(crate) offset_rad: Option<f32>,
    pub(crate) pass: Option<bool>,
    pub(crate) policy_stats: Option<PolicyStats>,
    pub(crate) phase_ms_len: u8,
    pub(crate) phase_ms: [(BoundedText<24>, u32); 12],
    pub(crate) dof: u8,
    pub(crate) positions: [f32; CAPTURE_MAX_DOF],
    pub(crate) velocities: [f32; CAPTURE_MAX_DOF],
    pub(crate) torques: [f32; CAPTURE_MAX_DOF],
    pub(crate) temperatures: [f32; CAPTURE_MAX_DOF],
    pub(crate) commanded_positions: [f32; CAPTURE_MAX_DOF],
    pub(crate) tcp: Option<CapturedTcpState>,
    pub(crate) gripper: Option<CapturedGripperObs>,
    /// Inline `(state, j7, offset_rad)` snapshot matching `gripper_zero_observation_into` so
    /// synthesized observations keep `positions[joint]`, `gripper.rad`, and `gripper_zero.j7`
    /// consistent from the same sample.
    pub(crate) gripper_zero: Option<(HolderLabel, f32, Option<f32>)>,
    pub(crate) gravity_compensated: bool,
    pub(crate) gravity_torques_len: u8,
    pub(crate) gravity_torques: [f32; CAPTURE_MAX_DOF],
    pub(crate) gravity_comp_phase: Option<HolderLabel>,
    pub(crate) gravity_comp_ramp: f32,
    pub(crate) gravity_comp_scale: f32,
    pub(crate) gravity_comp_damping_kd_len: u8,
    pub(crate) gravity_comp_damping_kd: [f32; CAPTURE_MAX_DOF],
    pub(crate) gravity_comp_tau_ff_len: u8,
    pub(crate) gravity_comp_tau_ff: [f32; CAPTURE_MAX_DOF],
    pub(crate) gravity_comp_workspace_outside: bool,
    pub(crate) gravity_comp_workspace_edge: Option<HolderLabel>,
    pub(crate) gravity_comp_workspace_depth_m: Option<f64>,
    pub(crate) gravity_comp_workspace_recovery_dir: Option<crate::chain::WorkspaceRecoveryDir>,
    pub(crate) feedback_age_ms: [Option<u64>; CAPTURE_MAX_DOF],
    pub(crate) is_torqued: bool,
    pub(crate) disable_pending: bool,
    pub(crate) feedback_valid: bool,
    pub(crate) epoch_id: u64,
    pub(crate) safety_state: CapturedSafetyState,
    pub(crate) fault: Option<FaultInfo>,
    pub(crate) control_mode: ControlMode,
    pub(crate) driver_caps: DriverCapabilities,
    pub(crate) enable_state: EnableState,
    pub(crate) lease_holder: Option<HolderLabel>,
}

#[inline]
fn copy_f32_slice(src: &[f32]) -> ([f32; CAPTURE_MAX_DOF], u8) {
    let mut out = [0.0f32; CAPTURE_MAX_DOF];
    let n = src.len().min(CAPTURE_MAX_DOF);
    out[..n].copy_from_slice(&src[..n]);
    (out, n as u8)
}

#[inline]
fn opt_label(s: Option<&String>) -> Option<HolderLabel> {
    s.map(|v| HolderLabel::new(v))
}

#[inline]
fn opt_label_tracked(s: Option<&String>) -> (Option<HolderLabel>, bool) {
    match s {
        Some(v) => (Some(HolderLabel::new(v)), v.len() > HolderLabel::CAPACITY),
        None => (None, false),
    }
}

fn overlay_opt_label(dst: &mut Option<String>, src: Option<HolderLabel>, truncated: bool) {
    match src {
        Some(v) => {
            let s = v.as_str();
            if truncated
                && dst
                    .as_deref()
                    .is_some_and(|existing| existing.starts_with(s))
            {
                return;
            }
            if dst.as_deref() != Some(s) {
                *dst = Some(s.to_owned());
            }
        }
        None => {
            *dst = None;
        }
    }
}

impl TerminalCaptureRecord {
    /// Captures a fixed-size terminal snapshot from `inner` and `action` using strictly
    /// per-tick cached driver state (zero heap allocations, zero driver mutexes).
    pub(crate) fn capture_from(
        inner: &SupervisorInner,
        action: &ActionStatus,
        terminal_seq: u64,
        obs_timestamp: f64,
    ) -> Self {
        let (has_message, message) = match action.message.as_deref() {
            Some(m) => (true, BoundedText::<256>::new(m)),
            None => (false, BoundedText::empty()),
        };
        let (has_durability_warning, durability_warning) = match action.durability_warning.as_deref()
        {
            Some(w) => (true, BoundedText::<256>::new(w)),
            None => (false, BoundedText::empty()),
        };
        let tag = opt_label(action.tag.as_ref());
        let task = opt_label(action.task.as_ref());
        let sequence_id = opt_label(action.sequence_id.as_ref());
        let step_type = opt_label(action.step_type.as_ref());
        let (constraint_id, t_c) = opt_label_tracked(action.constraint_id.as_ref());
        let (geometry_id, t_g) = opt_label_tracked(action.geometry_id.as_ref());
        let (violation_phase, t_v) = opt_label_tracked(action.violation_phase.as_ref());
        let (reason, t_r) = opt_label_tracked(action.reason.as_ref());
        let label_truncated_mask =
            u8::from(t_c) | (u8::from(t_g) << 1) | (u8::from(t_v) << 2) | (u8::from(t_r) << 3);

        let mut phase_ms = [(BoundedText::<24>::empty(), 0u32); 12];
        let mut phase_ms_len = 0u8;
        for (k, &v) in &action.phase_ms {
            if usize::from(phase_ms_len) < phase_ms.len() {
                phase_ms[usize::from(phase_ms_len)] = (BoundedText::<24>::new(k), v);
                phase_ms_len += 1;
            } else {
                break;
            }
        }

        let state = inner.safety.state();
        let fault = if state.is_stopped() || inner.safety.has_latched_fault() {
            inner.safety.fault()
        } else {
            None
        };
        let dof = inner.config.dof.min(CAPTURE_MAX_DOF) as u8;
        let tcp = if inner.config.dof >= inner.config.arm_dof()
            && inner.measured_positions.len() >= inner.config.arm_dof()
        {
            inner.config.arm_model.fk_f32(&inner.measured_positions).map(|end| {
                let pose = inner.config.end_to_tcp(&end);
                let rpy = crate::kinematics::r_to_rpy(&pose.r);
                let quat = crate::kinematics::r_to_quat(&pose.r);
                CapturedTcpState {
                    xyz_m: [pose.p[0] as f32, pose.p[1] as f32, pose.p[2] as f32],
                    rpy_rad: [rpy[0] as f32, rpy[1] as f32, rpy[2] as f32],
                    quat_wxyz: [
                        quat[0] as f32,
                        quat[1] as f32,
                        quat[2] as f32,
                        quat[3] as f32,
                    ],
                }
            })
        } else {
            None
        };
        let gripper = if let (Some(idx), Some(policy)) = (
            inner.config.gripper_index(),
            inner.config.gripper_policy.as_ref(),
        ) {
            inner.measured_positions.get(idx).map(|&rad| {
                let span = policy.open_rad - policy.closed_rad;
                let fraction = if span.abs() < 1e-6 {
                    0.0
                } else {
                    ((rad - policy.closed_rad) / span).clamp(0.0, 1.0)
                };
                let opening_m = policy.jaw_opening.as_ref().map(|jo| jo.opening_m(rad));
                CapturedGripperObs {
                    joint: idx,
                    rad,
                    fraction,
                    opening_m,
                    stalled: inner.gripper_stalled,
                    force_limited: inner.gripper_stalled && inner.gripper_force_capped,
                    force_limit_trips: inner.gripper_force_trips,
                    baseline: Some(inner.gripper_baseline_observation()),
                }
            })
        } else {
            None
        };
        let gripper_zero = if let (Some(idx), Some(policy)) = (
            inner.config.gripper_index(),
            inner.config.gripper_policy.as_ref(),
        ) {
            let j7 = inner.measured_positions.get(idx).copied().unwrap_or(0.0);
            let offset_rad = inner.gripper_calib_offset.or(inner.last_zero_check_offset);
            let gz_state = if inner.gripper_calib_invalid {
                "calib_invalid"
            } else if inner.gripper_zero_checked {
                "checked"
            } else {
                let min_j7 = policy.closed_rad.min(policy.open_rad) - 0.3;
                let max_j7 = policy.closed_rad.max(policy.open_rad) + 0.3;
                if j7 < min_j7 || j7 > max_j7 {
                    "out_of_range"
                } else {
                    "ok"
                }
            };
            Some((HolderLabel::new(gz_state), j7, offset_rad))
        } else {
            None
        };
        let now_clock = inner.cached_driver_clock_now();
        let (feedback_age_ms, feedback_valid) = compute_cached_feedback_state(
            inner.feedback_numeric_valid,
            inner.capture_driver_cache.feedback_valid,
            inner.driver.is_some(),
            usize::from(dof),
            inner.core.feedback().instants(),
            &inner.capture_driver_cache.ages[..usize::from(inner.capture_driver_cache.ages_len)],
            inner.capture_driver_cache.feedback_at,
            now_clock,
            Duration::from_millis(250),
        );

        let driver_caps = inner.cached_capture_driver_caps();
        let gravity_compensated = inner.gravity_compensated
            && inner.config.dynamics_qualified
            && inner.config.gravity_model.is_some()
            && inner.config.dof >= inner.config.arm_dof()
            && (inner.driver.is_none() || driver_caps.torque_feedforward);
        let (positions, _) = copy_f32_slice(&inner.measured_positions);
        let (velocities, _) = copy_f32_slice(&inner.measured_velocities);
        let (torques, _) = copy_f32_slice(&inner.torques);
        let (temperatures, _) = copy_f32_slice(&inner.temperatures);
        let (commanded_positions, _) = copy_f32_slice(&inner.commanded_positions);
        let (gravity_torques, gravity_torques_len) = copy_f32_slice(&inner.gravity_torques);
        let (
            gravity_comp_phase,
            gravity_comp_ramp,
            gravity_comp_scale,
            gravity_comp_damping_kd,
            gravity_comp_damping_kd_len,
            gravity_comp_tau_ff,
            gravity_comp_tau_ff_len,
            gravity_comp_workspace_outside,
            gravity_comp_workspace_edge,
            gravity_comp_workspace_depth_m,
            gravity_comp_workspace_recovery_dir,
        ) = match &inner.gravity_comp {
            Some(run) => {
                let (kd, kd_len) = copy_f32_slice(&run.damping_kd);
                let (tau, tau_len) = copy_f32_slice(&run.tau_ff);
                let out = run.workspace_status.outside_info();
                (
                    Some(HolderLabel::new(run.phase.name())),
                    run.ramp(),
                    run.scale,
                    kd,
                    kd_len,
                    tau,
                    tau_len,
                    out.is_some(),
                    out.map(|w| w.edge),
                    out.map(|w| w.depth_m),
                    out.map(|w| w.recovery_dir),
                )
            }
            None => {
                let policy = inner
                    .config
                    .gravity_comp
                    .as_ref()
                    .filter(|_| inner.config.six_plus_gripper() && inner.config.gravity_model.is_some());
                let (scale, kd, kd_len) = match policy {
                    Some(p) => {
                        let (kd, len) = copy_f32_slice(&p.damping_kd);
                        (p.scale, kd, len)
                    }
                    None => (0.0, [0.0; CAPTURE_MAX_DOF], 0),
                };
                (None, 0.0, scale, kd, kd_len, [0.0; CAPTURE_MAX_DOF], 0, false, None, None, None)
            }
        };

        Self {
            terminal_seq,
            publish_seq: None,
            action_id: action.id,
            kind: action.kind,
            state: action.state,
            code: action.code,
            has_message,
            message,
            tag,
            task,
            sequence_id,
            step_type,
            phase: action.phase,
            episode: action.episode,
            constraint_id,
            geometry_id,
            violation_phase,
            reason,
            label_truncated_mask,
            durability_confirmed: action.durability_confirmed,
            has_durability_warning,
            durability_warning,
            step_index: action.step_index,
            failed_step: action.failed_step,
            repeat_index: action.repeat_index,
            repeats: action.repeats,
            progress: action.progress,
            started_at: action.started_at,
            finished_at: action.finished_at,
            obs_timestamp,
            holding: action.holding,
            gripper_gap_rad: action.gripper_gap_rad,
            settled: action.settled,
            t_s: action.t_s,
            power_scale: action.power_scale,
            stop_rad: action.stop_rad,
            offset_rad: action.offset_rad,
            pass: action.pass,
            policy_stats: action.policy_stats,
            phase_ms_len,
            phase_ms,
            dof,
            positions,
            velocities,
            torques,
            temperatures,
            commanded_positions,
            tcp,
            gripper,
            gripper_zero,
            gravity_compensated,
            gravity_torques_len,
            gravity_torques,
            gravity_comp_phase,
            gravity_comp_ramp,
            gravity_comp_scale,
            gravity_comp_damping_kd_len,
            gravity_comp_damping_kd,
            gravity_comp_tau_ff_len,
            gravity_comp_tau_ff,
            gravity_comp_workspace_outside,
            gravity_comp_workspace_edge,
            gravity_comp_workspace_depth_m,
            gravity_comp_workspace_recovery_dir,
            feedback_age_ms,
            is_torqued: inner.is_torqued,
            disable_pending: inner.disable_pending,
            feedback_valid,
            epoch_id: inner.safety.epoch_id(),
            safety_state: CapturedSafetyState::from_safety_state(state),
            fault,
            control_mode: if inner.gravity_comp.is_some() {
                ControlMode::GravityComp
            } else {
                ControlMode::Position
            },
            driver_caps,
            enable_state: inner.cached_enable_state(),
            lease_holder: inner.lease_holder_str().map(HolderLabel::new),
        }
    }

    /// Reconstructs an [`ActionStatus`] off-thread from this captured record.
    pub(crate) fn build_action_status(&self) -> ActionStatus {
        let mut a = ActionStatus::blank();
        a.id = self.action_id;
        a.kind = self.kind;
        a.tag = self.tag.map(|l| l.as_str().to_owned());
        a.state = self.state;
        a.code = self.code;
        a.message = self.has_message.then(|| self.message.as_str().to_owned());
        a.progress = self.progress;
        a.started_at = self.started_at;
        a.finished_at = self.finished_at;
        a.holding = self.holding;
        a.gripper_gap_rad = self.gripper_gap_rad;
        a.settled = self.settled;
        a.task = self.task.map(|l| l.as_str().to_owned());
        a.phase = self.phase;
        a.power_scale = self.power_scale;
        a.episode = self.episode;
        a.t_s = self.t_s;
        a.sequence_id = self.sequence_id.map(|l| l.as_str().to_owned());
        a.step_index = self.step_index;
        a.step_type = self.step_type.map(|l| l.as_str().to_owned());
        a.repeat_index = self.repeat_index;
        a.repeats = self.repeats;
        a.failed_step = self.failed_step;
        a.stop_rad = self.stop_rad;
        a.offset_rad = self.offset_rad;
        a.pass = self.pass;
        a.constraint_id = self.constraint_id.map(|l| l.as_str().to_owned());
        a.geometry_id = self.geometry_id.map(|l| l.as_str().to_owned());
        a.violation_phase = self.violation_phase.map(|l| l.as_str().to_owned());
        a.reason = self.reason.map(|l| l.as_str().to_owned());
        a.durability_confirmed = self.durability_confirmed;
        a.durability_warning = self
            .has_durability_warning
            .then(|| self.durability_warning.as_str().to_owned());
        a.policy_stats = self.policy_stats;
        if self.phase_ms_len > 0 {
            let mut map = BTreeMap::new();
            for i in 0..usize::from(self.phase_ms_len).min(self.phase_ms.len()) {
                let (k, v) = self.phase_ms[i];
                map.insert(k.as_str().to_owned(), v);
            }
            a.phase_ms = map;
        }
        a
    }

    /// Applies terminal transition fields captured on-tick onto a rich cached off-RT `ActionStatus`
    /// (preserving `episodes`, `motions`, `height_estimates`, `measurement`, grasp diagnostics, and
    /// full untruncated strings while updating terminal state, phase, and timing fields).
    pub(crate) fn apply_terminal_overlay(&self, dst: &mut ActionStatus) {
        dst.state = self.state;
        dst.code = self.code;
        if self.has_message {
            if dst.message.as_deref() != Some(self.message.as_str()) && !self.message.is_truncated()
            {
                dst.message = Some(self.message.as_str().to_owned());
            }
        } else {
            dst.message = None;
        }
        dst.progress = self.progress;
        dst.finished_at = self.finished_at;
        dst.phase = self.phase;
        if self.holding.is_some() {
            dst.holding = self.holding;
        }
        if self.gripper_gap_rad.is_some() {
            dst.gripper_gap_rad = self.gripper_gap_rad;
        }
        if self.settled.is_some() {
            dst.settled = self.settled;
        }
        if self.t_s.is_some() {
            dst.t_s = self.t_s;
        }
        if self.power_scale.is_some() {
            dst.power_scale = self.power_scale;
        }
        if self.step_index.is_some() {
            dst.step_index = self.step_index;
        }
        if self.repeat_index.is_some() {
            dst.repeat_index = self.repeat_index;
        }
        if self.repeats.is_some() {
            dst.repeats = self.repeats;
        }
        if self.failed_step.is_some() {
            dst.failed_step = self.failed_step;
        }
        if self.stop_rad.is_some() {
            dst.stop_rad = self.stop_rad;
        }
        if self.offset_rad.is_some() {
            dst.offset_rad = self.offset_rad;
        }
        if self.pass.is_some() {
            dst.pass = self.pass;
        }
        overlay_opt_label(
            &mut dst.constraint_id,
            self.constraint_id,
            (self.label_truncated_mask & 1) != 0,
        );
        overlay_opt_label(
            &mut dst.geometry_id,
            self.geometry_id,
            (self.label_truncated_mask & 2) != 0,
        );
        overlay_opt_label(
            &mut dst.violation_phase,
            self.violation_phase,
            (self.label_truncated_mask & 4) != 0,
        );
        overlay_opt_label(
            &mut dst.reason,
            self.reason,
            (self.label_truncated_mask & 8) != 0,
        );
        if self.durability_confirmed.is_some() {
            dst.durability_confirmed = self.durability_confirmed;
        }
        if self.has_durability_warning && !self.durability_warning.is_truncated() {
            dst.durability_warning = Some(self.durability_warning.as_str().to_owned());
        }
        if self.policy_stats.is_some() {
            dst.policy_stats = self.policy_stats;
        }
        for i in 0..usize::from(self.phase_ms_len).min(self.phase_ms.len()) {
            let (k, v) = self.phase_ms[i];
            if !k.is_truncated() {
                dst.phase_ms.insert(k.as_str().to_owned(), v);
            }
        }
    }

    /// Builds a full [`ArmObservation`] off-thread by overlaying this captured record's
    /// timestamps, inline joint arrays, safety/driver state, and action onto `base_obs`.
    pub(crate) fn build_observation(
        &self,
        base_obs: &ArmObservation,
        action: ActionStatus,
    ) -> ArmObservation {
        let mut obs = base_obs.clone();
        let dof = usize::from(self.dof).min(CAPTURE_MAX_DOF);
        obs.timestamp = self.obs_timestamp;
        obs.positions.clear();
        obs.positions.extend_from_slice(&self.positions[..dof]);
        obs.velocities.clear();
        obs.velocities.extend_from_slice(&self.velocities[..dof]);
        obs.torques.clear();
        obs.torques.extend_from_slice(&self.torques[..dof]);
        obs.temperatures.clear();
        obs.temperatures.extend_from_slice(&self.temperatures[..dof]);
        obs.commanded_positions.clear();
        obs.commanded_positions
            .extend_from_slice(&self.commanded_positions[..dof]);
        obs.tcp = self.tcp.map(CapturedTcpState::to_tcp_state);
        obs.gripper = self.gripper.map(CapturedGripperObs::to_gripper_obs);
        obs.gripper_zero = self.gripper_zero.map(|(state, j7, offset_rad)| {
            crate::model::GripperZeroObservation {
                state: state.as_str().to_owned(),
                j7,
                offset_rad,
            }
        });
        obs.gravity_compensated = self.gravity_compensated;
        let gt_len = usize::from(self.gravity_torques_len).min(CAPTURE_MAX_DOF);
        obs.gravity_torques.clear();
        obs.gravity_torques
            .extend_from_slice(&self.gravity_torques[..gt_len]);
        obs.feedback_age_ms.clear();
        obs.feedback_age_ms
            .extend_from_slice(&self.feedback_age_ms[..dof]);
        obs.is_torqued = self.is_torqued;
        obs.disable_pending = self.disable_pending;
        obs.feedback_valid = self.feedback_valid;
        obs.epoch_id = self.epoch_id;
        obs.safety_state = self.safety_state.to_safety_state();
        obs.fault = self.fault;
        obs.control_mode = self.control_mode;
        obs.gravity_comp.phase = self.gravity_comp_phase;
        obs.gravity_comp.ramp = self.gravity_comp_ramp;
        obs.gravity_comp.scale = self.gravity_comp_scale;
        let kd_len = usize::from(self.gravity_comp_damping_kd_len).min(CAPTURE_MAX_DOF);
        obs.gravity_comp.damping_kd.clear();
        obs.gravity_comp
            .damping_kd
            .extend_from_slice(&self.gravity_comp_damping_kd[..kd_len]);
        let tau_len = usize::from(self.gravity_comp_tau_ff_len).min(CAPTURE_MAX_DOF);
        obs.gravity_comp.tau_ff.clear();
        obs.gravity_comp
            .tau_ff
            .extend_from_slice(&self.gravity_comp_tau_ff[..tau_len]);
        obs.gravity_comp.workspace_outside = self.gravity_comp_workspace_outside;
        obs.gravity_comp.workspace_edge = self.gravity_comp_workspace_edge;
        obs.gravity_comp.workspace_depth_m = self.gravity_comp_workspace_depth_m;
        obs.gravity_comp.workspace_recovery_dir = self.gravity_comp_workspace_recovery_dir;
        obs.driver_capabilities = self.driver_caps;
        obs.enable_state = self.enable_state;
        obs.lease_holder = self.lease_holder;
        obs.action = Some(action);
        obs.previous_action = None;
        obs
    }
}

#[cfg(test)]
mod tests;
