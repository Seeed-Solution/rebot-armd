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
    HolderLabel,
};
use crate::motion::{ActionStatus, PolicyStats};
use crate::rt_core::mailbox::MAX_DOF;
use crate::rt_core::publish::{Snapshot, StateHub};
use crate::safety::{BoundedFaultReason, BoundedText, FaultInfo, FaultReason, SafetyState};

use super::{SupervisorInner, TerminalBroadcaster, TERMINAL_FIFO_CAPACITY};

/// Physical SPSC ring capacity (2x `TERMINAL_FIFO_CAPACITY` for in-flight headroom).
pub const TERMINAL_CAPTURE_RING_CAPACITY: usize = TERMINAL_FIFO_CAPACITY * 2;
/// Logical admission reservation ceiling matching `TERMINAL_FIFO_CAPACITY`.
pub const TERMINAL_CAPTURE_ADMISSION_CAP: usize = TERMINAL_FIFO_CAPACITY;

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
                BoundedFaultReason::BusLoss => FaultReason::BusLoss,
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
    pub(crate) oldest_age: Option<Duration>,
    pub(crate) feedback_at: Option<Instant>,
}

impl Default for CaptureDriverCache {
    fn default() -> Self {
        Self {
            enable_state: EnableState::Unknown,
            driver_caps: DriverCapabilities::default(),
            feedback_valid: false,
            oldest_age: None,
            feedback_at: None,
        }
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
    pub(crate) positions: [f32; MAX_DOF],
    pub(crate) velocities: [f32; MAX_DOF],
    pub(crate) torques: [f32; MAX_DOF],
    pub(crate) temperatures: [f32; MAX_DOF],
    pub(crate) commanded_positions: [f32; MAX_DOF],
    pub(crate) gripper: Option<CapturedGripperObs>,
    pub(crate) gravity_compensated: bool,
    pub(crate) gravity_torques: [f32; MAX_DOF],
    pub(crate) feedback_age_ms: [Option<u64>; MAX_DOF],
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
fn copy_f32_slice(src: &[f32]) -> [f32; MAX_DOF] {
    let mut out = [0.0f32; MAX_DOF];
    let n = src.len().min(MAX_DOF);
    out[..n].copy_from_slice(&src[..n]);
    out
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
        let dof = inner.config.dof.min(MAX_DOF) as u8;
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
        let mut feedback_age_ms = [None; MAX_DOF];
        let now_clock = inner.cached_driver_clock_now();
        let dof_usize = usize::from(dof);
        if inner.driver.is_none() {
            for slot in &mut feedback_age_ms[..dof_usize] {
                *slot = Some(0);
            }
        } else {
            let instants = inner.core.feedback().instants();
            if instants.len() >= dof_usize && instants[..dof_usize].iter().any(Option::is_some) {
                for (dst, &inst) in feedback_age_ms[..dof_usize].iter_mut().zip(&instants[..dof_usize]) {
                    *dst = inst.map(|t| {
                        now_clock
                            .saturating_duration_since(t)
                            .as_millis()
                            .min(u128::from(u64::MAX)) as u64
                    });
                }
            } else if let (Some(oldest), Some(polled_at)) = (
                inner.capture_driver_cache.oldest_age,
                inner.capture_driver_cache.feedback_at,
            ) {
                let total_ms = oldest
                    .saturating_add(now_clock.saturating_duration_since(polled_at))
                    .as_millis()
                    .min(u128::from(u64::MAX)) as u64;
                for slot in &mut feedback_age_ms[..dof_usize] {
                    *slot = Some(total_ms);
                }
            }
        }

        let driver_caps = inner.cached_capture_driver_caps();
        let gravity_compensated = inner.gravity_compensated
            && inner.config.dynamics_qualified
            && inner.config.gravity_model.is_some()
            && inner.config.dof >= inner.config.arm_dof()
            && (inner.driver.is_none() || driver_caps.torque_feedforward);

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
            positions: copy_f32_slice(&inner.measured_positions),
            velocities: copy_f32_slice(&inner.measured_velocities),
            torques: copy_f32_slice(&inner.torques),
            temperatures: copy_f32_slice(&inner.temperatures),
            commanded_positions: copy_f32_slice(&inner.commanded_positions),
            gripper,
            gravity_compensated,
            gravity_torques: copy_f32_slice(&inner.gravity_torques),
            feedback_age_ms,
            is_torqued: inner.is_torqued,
            disable_pending: inner.disable_pending,
            feedback_valid: inner.cached_hardware_feedback_fresh(Duration::from_millis(250)),
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
        let dof = usize::from(self.dof).min(MAX_DOF);
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
        obs.gripper = self.gripper.map(CapturedGripperObs::to_gripper_obs);
        obs.gravity_compensated = self.gravity_compensated;
        obs.gravity_torques.clear();
        obs.gravity_torques
            .extend_from_slice(&self.gravity_torques[..dof]);
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
        if self.control_mode != ControlMode::GravityComp {
            obs.gravity_comp.phase = None;
        }
        obs.driver_capabilities = self.driver_caps;
        obs.enable_state = self.enable_state;
        obs.lease_holder = self.lease_holder;
        obs.action = Some(action);
        obs.previous_action = None;
        obs
    }
}

/// Inline staged entry for one release before `capture_edge_terminals` runs:
/// - `Deferred`: ordinary in-tick `finish_action` / `config.rs` transition whose `ActionStatus`
///   remains in `last_action` (or `previous_action`), so `TerminalCaptureRecord::capture_from`
///   runs inside `capture_edge_terminals` (timed in `phases_us.terminal` and covered by capture
///   alloc/lock proofs).
/// - `Captured`: inline record captured immediately when an in-tick superseding `begin_action_inner`
///   is about to overwrite the old slot before `capture_edge_terminals`.
#[allow(clippy::large_enum_variant)]
pub(crate) enum StagedTerminal {
    Deferred {
        action_id: u64,
        terminal_seq: u64,
    },
    Captured(TerminalCaptureRecord),
}

/// Fixed-capacity inline staging buffer for terminal transitions produced during one control
/// release before `publish_at` flushes them into the SPSC ring (0 heap allocations).
#[derive(Default)]
pub(crate) struct StagedTerminalBuf {
    slots: [Option<StagedTerminal>; 4],
    len: usize,
}

impl StagedTerminalBuf {
    #[inline]
    pub(crate) fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub(crate) fn has_deferred(&self, action_id: u64) -> bool {
        self.deferred_seq_for(action_id).is_some()
    }

    #[inline]
    pub(crate) fn deferred_seq_for(&self, action_id: u64) -> Option<u64> {
        for slot in &self.slots[..self.len] {
            if let Some(StagedTerminal::Deferred {
                action_id: id,
                terminal_seq,
            }) = slot
            {
                if *id == action_id {
                    return Some(*terminal_seq);
                }
            }
        }
        None
    }

    #[inline]
    pub(crate) fn replace_deferred_with_captured(
        &mut self,
        action_id: u64,
        rec: TerminalCaptureRecord,
    ) -> bool {
        for slot in &mut self.slots[..self.len] {
            if let Some(StagedTerminal::Deferred { action_id: id, .. }) = slot {
                if *id == action_id {
                    *slot = Some(StagedTerminal::Captured(rec));
                    return true;
                }
            }
        }
        false
    }

    /// Stages a terminal entry inline; if all 4 slots are already occupied, evicts and returns
    /// the OLDEST staged entry so the caller can flush it to the SPSC ring first and preserve
    /// strict `terminal_seq` FIFO ordering.
    #[inline]
    pub(crate) fn push_evicting_oldest(&mut self, item: StagedTerminal) -> Option<StagedTerminal> {
        let evicted = if self.len == self.slots.len() {
            self.pop_front()
        } else {
            None
        };
        self.slots[self.len] = Some(item);
        self.len += 1;
        evicted
    }

    #[inline]
    pub(crate) fn pop_front(&mut self) -> Option<StagedTerminal> {
        if self.len == 0 {
            return None;
        }
        let item = self.slots[0].take();
        for i in 1..self.len {
            self.slots[i - 1] = self.slots[i].take();
        }
        self.len -= 1;
        item
    }
}

/// Fixed-size deduplication tracker for terminal edges keyed by `(action_id, terminal_seq)`.
/// Guarantees an already-captured action slot is never re-captured during dwell ticks or
/// after moving from `last_action` to `previous_action`.
#[derive(Debug, Default)]
pub(crate) struct TerminalEdgeTracker {
    /// Recent `(action_id, terminal_seq)` pairs that have emitted a terminal edge.
    seen: [(u64, u64); 16],
    head: usize,
}

impl TerminalEdgeTracker {
    #[inline]
    pub(crate) fn has_seen_action(&self, action_id: u64) -> bool {
        action_id != 0 && self.seen.iter().any(|&(id, _)| id == action_id)
    }

    #[inline]
    pub(crate) fn has_seen_edge(&self, action_id: u64, terminal_seq: u64) -> bool {
        action_id != 0
            && self
                .seen
                .iter()
                .any(|&(id, seq)| id == action_id && (terminal_seq == 0 || seq == terminal_seq))
    }

    #[inline]
    pub(crate) fn record_edge(&mut self, action_id: u64, terminal_seq: u64) -> bool {
        if action_id == 0 || self.has_seen_action(action_id) {
            return false;
        }
        self.seen[self.head] = (action_id, terminal_seq);
        self.head = (self.head + 1) % self.seen.len();
        true
    }
}

/// Exclusive-access (`&mut self` only) producer wrapper around `rtrb::Producer<TerminalCaptureRecord>`
/// so `SupervisorInner` remains `Sync` while `armd-control` pushes lock-free with zero allocations.
pub(crate) struct TerminalCaptureTx {
    producer: rtrb::Producer<TerminalCaptureRecord>,
    channel: Arc<TerminalCaptureChannel>,
}

// SAFETY: `TerminalCaptureTx` exposes no `&self` access to `self.producer`; all push operations
// require `&mut self`, enforcing single-producer exclusive access (identical to `PolicyTerminalTx`).
unsafe impl Sync for TerminalCaptureTx {}

impl TerminalCaptureTx {
    /// Attempts a lock-free, allocation-free push into the preallocated SPSC ring.
    /// Charges `channel.inflight` with `Release` strictly BEFORE making the element visible to
    /// the consumer; on saturation (`Full`), rolls back `inflight`, increments `saturated`, and
    /// latches `ever_saturated` without allocating or blocking.
    #[inline]
    pub(crate) fn try_push(&mut self, rec: TerminalCaptureRecord) -> bool {
        let cap_limit = self.channel.test_push_cap.load(Ordering::Relaxed);
        let cur_inflight = self.channel.inflight.load(Ordering::Acquire);
        if cur_inflight >= cap_limit {
            self.channel.saturated.fetch_add(1, Ordering::Relaxed);
            self.channel.ever_saturated.store(true, Ordering::Release);
            return false;
        }
        // Charge inflight BEFORE pushing so the consumer can never pop & decrement before this add.
        self.channel.inflight.fetch_add(1, Ordering::Release);
        match self.producer.push(rec) {
            Ok(()) => {
                self.channel.captured.fetch_add(1, Ordering::Relaxed);
                true
            }
            Err(rtrb::PushError::Full(_)) => {
                self.channel.inflight.fetch_sub(1, Ordering::AcqRel);
                self.channel.saturated.fetch_add(1, Ordering::Relaxed);
                self.channel.ever_saturated.store(true, Ordering::Release);
                false
            }
        }
    }
}

/// Outcome of draining ready terminal records from the SPSC ring into `TerminalBroadcaster`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct DrainOutcome {
    pub(crate) drained: usize,
    /// Action ID of the latest synthetic (`obs_override.is_some()`) terminal drained, so
    /// `dispatch_loop` can suppress any older `running` `pending_telemetry` for the same action.
    pub(crate) last_synthetic_action_id: Option<u64>,
}

struct CachedRichEntry {
    id: u64,
    status: Option<ActionStatus>,
    message: Option<String>,
    durability_warning: Option<String>,
}

/// Shared off-thread consumer and lock-free atomic accounting for terminal capture.
/// `rx` and `rich_action_cache` are NEVER locked on `armd-control`; lock order is strictly
/// `channel.rx -> terminals.inner` (and `rich_action_cache` is a leaf mutex).
pub(crate) struct TerminalCaptureChannel {
    rx: Mutex<rtrb::Consumer<TerminalCaptureRecord>>,
    /// Off-RT mirror of off-RT mutated `ActionStatus` payloads and >256-byte worker strings
    /// (bounded to `TERMINAL_FIFO_CAPACITY`) so delayed dispatch past the 2 s window never drops
    /// or truncates any field without touching `SupervisorLock`. Never accessed on `armd-control`.
    rich_action_cache: Mutex<VecDeque<CachedRichEntry>>,
    control_thread_lock_attempts: AtomicU64,
    next_terminal_seq: AtomicU64,
    captured: AtomicU64,
    saturated: AtomicU64,
    ever_saturated: AtomicBool,
    inflight: AtomicUsize,
    admission_cap: AtomicUsize,
    test_push_cap: AtomicUsize,
    drain_paused: AtomicBool,
}

impl TerminalCaptureChannel {
    pub(crate) fn new() -> (TerminalCaptureTx, Arc<Self>) {
        let (producer, consumer) = rtrb::RingBuffer::new(TERMINAL_CAPTURE_RING_CAPACITY);
        let channel = Arc::new(Self {
            rx: Mutex::new(consumer),
            rich_action_cache: Mutex::new(VecDeque::with_capacity(TERMINAL_FIFO_CAPACITY)),
            control_thread_lock_attempts: AtomicU64::new(0),
            next_terminal_seq: AtomicU64::new(0),
            captured: AtomicU64::new(0),
            saturated: AtomicU64::new(0),
            ever_saturated: AtomicBool::new(false),
            inflight: AtomicUsize::new(0),
            admission_cap: AtomicUsize::new(TERMINAL_CAPTURE_ADMISSION_CAP),
            test_push_cap: AtomicUsize::new(TERMINAL_CAPTURE_RING_CAPACITY),
            drain_paused: AtomicBool::new(false),
        });
        let tx = TerminalCaptureTx {
            producer,
            channel: Arc::clone(&channel),
        };
        (tx, channel)
    }

    #[inline]
    fn lock_rx(&self) -> std::sync::MutexGuard<'_, rtrb::Consumer<TerminalCaptureRecord>> {
        if crate::rt_core::executor::on_rt_control_thread() {
            self.control_thread_lock_attempts
                .fetch_add(1, Ordering::SeqCst);
            crate::rt_core::executor::assert_not_on_rt_control_thread(
                "TerminalCaptureChannel::rx.lock",
            );
        }
        crate::rt_core::executor::assert_not_in_rt_release("TerminalCaptureChannel::rx.lock");
        self.rx.lock().unwrap_or_else(|e| e.into_inner())
    }

    #[inline]
    fn lock_rich_cache(&self) -> std::sync::MutexGuard<'_, VecDeque<CachedRichEntry>> {
        if crate::rt_core::executor::on_rt_control_thread() {
            self.control_thread_lock_attempts
                .fetch_add(1, Ordering::SeqCst);
            crate::rt_core::executor::assert_not_on_rt_control_thread(
                "TerminalCaptureChannel::rich_action_cache.lock",
            );
        }
        crate::rt_core::executor::assert_not_in_rt_release(
            "TerminalCaptureChannel::rich_action_cache.lock",
        );
        self.rich_action_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn control_thread_lock_attempts(&self) -> u64 {
        self.control_thread_lock_attempts.load(Ordering::Acquire)
    }

    pub(crate) fn lock_rx_for_test(&self) -> impl Drop + '_ {
        self.rx.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn lock_rich_cache_for_test(&self) -> impl Drop + '_ {
        self.rich_action_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn touch_rx_lock_for_test(&self) {
        let _g = self.lock_rx();
    }

    pub(crate) fn touch_rich_cache_lock_for_test(&self) {
        let _g = self.lock_rich_cache();
    }

    /// Unconditional off-RT mirror for `ActionStatus` payloads (never called on `armd-control`).
    pub(crate) fn mirror_rich_action(&self, action: &ActionStatus) {
        let mut cache = self.lock_rich_cache();
        if let Some(existing) = cache.iter_mut().find(|e| e.id == action.id) {
            match existing.status.as_mut() {
                Some(s) => s.clone_from(action),
                None => existing.status = Some(action.clone()),
            }
            return;
        }
        while cache.len() >= TERMINAL_FIFO_CAPACITY {
            cache.pop_front();
        }
        cache.push_back(CachedRichEntry {
            id: action.id,
            status: Some(action.clone()),
            message: None,
            durability_warning: None,
        });
    }

    /// Off-RT mirror for >256-byte persistence worker error messages so delayed recovery past
    /// the 2 s window never truncates `ActionStatus::message`.
    pub(crate) fn mirror_action_message(&self, action_id: u64, message: &str) {
        if action_id == 0 || message.len() <= 256 {
            return;
        }
        let mut cache = self.lock_rich_cache();
        if let Some(existing) = cache.iter_mut().find(|e| e.id == action_id) {
            existing.message = Some(message.to_owned());
            return;
        }
        while cache.len() >= TERMINAL_FIFO_CAPACITY {
            cache.pop_front();
        }
        cache.push_back(CachedRichEntry {
            id: action_id,
            status: None,
            message: Some(message.to_owned()),
            durability_warning: None,
        });
    }

    /// Off-RT mirror for >256-byte persistence worker durability warnings.
    pub(crate) fn mirror_action_durability_warning(&self, action_id: u64, warning: &str) {
        if action_id == 0 || warning.len() <= 256 {
            return;
        }
        let mut cache = self.lock_rich_cache();
        if let Some(existing) = cache.iter_mut().find(|e| e.id == action_id) {
            existing.durability_warning = Some(warning.to_owned());
            return;
        }
        while cache.len() >= TERMINAL_FIFO_CAPACITY {
            cache.pop_front();
        }
        cache.push_back(CachedRichEntry {
            id: action_id,
            status: None,
            message: None,
            durability_warning: Some(warning.to_owned()),
        });
    }

    fn take_rich_entry(&self, action_id: u64) -> Option<CachedRichEntry> {
        let mut cache = self.lock_rich_cache();
        let pos = cache.iter().position(|e| e.id == action_id)?;
        let item = cache.remove(pos);
        // Prune older consumed entries boundedly.
        while cache.front().is_some_and(|e| e.id < action_id)
            && cache.len() > TERMINAL_FIFO_CAPACITY / 2
        {
            cache.pop_front();
        }
        item
    }

    /// Allocates the next authoritative monotonic `terminal_seq` (`1, 2, 3, ...`), visible
    /// immediately to subscription cutoffs (`Release`).
    #[inline]
    pub(crate) fn alloc_terminal_seq(&self) -> u64 {
        self.next_terminal_seq.fetch_add(1, Ordering::AcqRel) + 1
    }

    /// Latest authoritative `terminal_seq` assigned so far (`Acquire`).
    #[inline]
    pub(crate) fn latest_terminal_seq(&self) -> u64 {
        self.next_terminal_seq.load(Ordering::Acquire)
    }

    /// Number of captured records currently in flight between producer push and broadcaster enqueue.
    #[inline]
    pub(crate) fn inflight(&self) -> usize {
        self.inflight.load(Ordering::Acquire)
    }

    #[inline]
    pub(crate) fn admission_cap(&self) -> usize {
        self.admission_cap.load(Ordering::Acquire)
    }

    pub(crate) fn set_admission_cap(&self, cap: usize) {
        self.admission_cap.store(cap.max(1), Ordering::Release);
    }

    pub(crate) fn set_test_push_cap(&self, cap: usize) {
        self.test_push_cap
            .store(cap.clamp(1, TERMINAL_CAPTURE_RING_CAPACITY), Ordering::Release);
    }

    pub(crate) fn set_drain_paused(&self, paused: bool) {
        self.drain_paused.store(paused, Ordering::Release);
    }

    pub(crate) fn stats(&self) -> crate::diagnostics::tick::TerminalCaptureStats {
        crate::diagnostics::tick::TerminalCaptureStats {
            captured: self.captured.load(Ordering::Relaxed),
            saturated: self.saturated.load(Ordering::Relaxed),
            ever_saturated: self.ever_saturated.load(Ordering::Acquire),
            queued: self.inflight.load(Ordering::Acquire) as u64,
        }
    }

    fn drain_locked(
        &self,
        rx: &mut rtrb::Consumer<TerminalCaptureRecord>,
        terminals: &TerminalBroadcaster,
        hub: &StateHub,
        latest_snap: Option<&Snapshot>,
        live_slots: &[Option<&ActionStatus>],
        template_obs: &ArmObservation,
        respect_publish_barrier: bool,
    ) -> DrainOutcome {
        if self.drain_paused.load(Ordering::Acquire) {
            return DrainOutcome::default();
        }
        let mut outcome = DrainOutcome::default();
        let hub_closed = hub.is_closed();
        // Single authoritative barrier policy governing BOTH ring eligibility AND synthesis:
        // when the hub is closed or barrier is disabled, an unmet `publish_seq` synthesizes a
        // self-contained captured observation instead of carrying an unmet sequence into the FIFO.
        let enforce_barrier = respect_publish_barrier && !hub_closed;
        let consumed_seq = latest_snap.map_or(0, |s| s.meta.seq);
        let latest_obs = latest_snap.map(|s| &s.obs);
        while let Ok(head) = rx.peek() {
            if enforce_barrier {
                if let Some(req_seq) = head.publish_seq {
                    if consumed_seq < req_seq {
                        break;
                    }
                }
            }
            let Ok(rec) = rx.pop() else {
                break;
            };
            let mut action = resolve_full_action(&rec, live_slots, latest_obs, self)
                .unwrap_or_else(|| rec.build_action_status());
            hub.attach_action_measurement_to_action(&mut action);

            let needs_captured_obs = rec.publish_seq.is_none()
                || (latest_obs.is_none() && hub.published_seq() == 0)
                || (!enforce_barrier && rec.publish_seq.is_some_and(|req| consumed_seq < req));
            let (obs_override, required_publish_seq) = if needs_captured_obs {
                outcome.last_synthetic_action_id = Some(rec.action_id);
                (
                    Some(rec.build_observation(latest_obs.unwrap_or(template_obs), action.clone())),
                    None,
                )
            } else {
                (None, rec.publish_seq)
            };

            // Enqueue into `TerminalBroadcaster` (updating `max_fifo_len` & `pending_total` with
            // `Release`) STRICTLY BEFORE decrementing `self.inflight` (`Release`).
            let _ = terminals.record_finished_with_seq(
                rec.terminal_seq,
                action,
                obs_override,
                required_publish_seq,
            );
            self.inflight.fetch_sub(1, Ordering::Release);
            outcome.drained += 1;
        }
        outcome
    }

    /// Drains ready records from the SPSC ring into `terminals` in strict `terminal_seq` order.
    pub(crate) fn drain_into(
        &self,
        terminals: &TerminalBroadcaster,
        hub: &StateHub,
        latest_snap: Option<&Snapshot>,
        live_slots: &[Option<&ActionStatus>],
        template_obs: &ArmObservation,
        respect_publish_barrier: bool,
    ) -> DrainOutcome {
        if self.drain_paused.load(Ordering::Acquire) {
            return DrainOutcome::default();
        }
        let mut rx = self.lock_rx();
        self.drain_locked(
            &mut rx,
            terminals,
            hub,
            latest_snap,
            live_slots,
            template_obs,
            respect_publish_barrier,
        )
    }

    /// Atomically drains ready pre-subscription captures and activates `sub_id`'s cutoff under
    /// the `(channel.rx -> terminals.inner)` lock order so a newly registered subscriber can
    /// neither inherit a full pre-subscription ring nor lose its first eligible terminal.
    pub(crate) fn drain_and_activate_subscriber(
        &self,
        sub_id: u64,
        terminals: &TerminalBroadcaster,
        hub: &StateHub,
        latest_snap: Option<&Snapshot>,
        template_obs: &ArmObservation,
    ) {
        let mut rx = self.lock_rx();
        let _ = self.drain_locked(
            &mut rx,
            terminals,
            hub,
            latest_snap,
            &[],
            template_obs,
            true,
        );
        terminals.activate_subscriber_cutoff(sub_id, || self.latest_terminal_seq());
    }
}

fn match_action(cand: &ActionStatus, rec: &TerminalCaptureRecord) -> bool {
    cand.id == rec.action_id
        && !cand.is_running()
        && cand.state == rec.state
        && cand.code == rec.code
}

fn resolve_full_action(
    rec: &TerminalCaptureRecord,
    live_slots: &[Option<&ActionStatus>],
    latest_obs: Option<&ArmObservation>,
    channel: &TerminalCaptureChannel,
) -> Option<ActionStatus> {
    for slot in live_slots.iter().flatten() {
        if match_action(slot, rec) {
            let _ = channel.take_rich_entry(rec.action_id);
            return Some((*slot).clone());
        }
    }
    if let Some(obs) = latest_obs {
        for slot in [obs.previous_action.as_ref(), obs.action.as_ref()]
            .into_iter()
            .flatten()
        {
            if match_action(slot, rec) {
                let _ = channel.take_rich_entry(rec.action_id);
                return Some(slot.clone());
            }
        }
    }
    if let Some(entry) = channel.take_rich_entry(rec.action_id) {
        let mut out = match entry.status {
            Some(mut cached) => {
                rec.apply_terminal_overlay(&mut cached);
                cached
            }
            None => rec.build_action_status(),
        };
        if rec.has_message && rec.message.is_truncated() {
            if let Some(full_msg) = entry.message {
                out.message = Some(full_msg);
            }
        }
        if rec.has_durability_warning && rec.durability_warning.is_truncated() {
            if let Some(full_warn) = entry.durability_warning {
                out.durability_warning = Some(full_warn);
            }
        }
        return Some(out);
    }
    None
}

/// Pure, lock-free cached driver freshness check shared by terminal capture and tests.
/// Gates on `feedback_numeric_valid` and `last_poll_feedback_valid` without touching
/// `ArmCore::feedback().instants()` on failed polls (preserving synthesized feedback sequences).
#[inline]
pub(crate) fn evaluate_cached_feedback_fresh(
    feedback_numeric_valid: bool,
    last_poll_feedback_valid: bool,
    has_driver: bool,
    dof: usize,
    instants: &[Option<Instant>],
    last_poll_oldest_age: Option<Duration>,
    last_poll_feedback_at: Option<Instant>,
    now: Instant,
    max_age: Duration,
) -> bool {
    if !feedback_numeric_valid {
        return false;
    }
    if !has_driver {
        return true;
    }
    if !last_poll_feedback_valid {
        return false;
    }
    if instants.len() == dof && instants.iter().all(Option::is_some) {
        return instants
            .iter()
            .all(|t| t.is_some_and(|rx| now.saturating_duration_since(rx) <= max_age));
    }
    match (last_poll_oldest_age, last_poll_feedback_at) {
        (Some(oldest_at_poll), Some(polled_before)) => {
            let elapsed = now.saturating_duration_since(polled_before);
            oldest_at_poll.saturating_add(elapsed) <= max_age
        }
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ArmConfig;
    use crate::motion::{EstimateEntry, MacroMotionDiag};
    use crate::supervisor::ArmSupervisor;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn edge_trigger_captures_once_zero_on_dwell_and_preserves_2s_previous_action() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        sup.start_dispatcher().unwrap();
        let delivered = Arc::new(Mutex::new(Vec::<ArmObservation>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            delivered_c.lock().unwrap().push(obs.clone());
            true
        });

        let id1 = {
            let mut inner = sup.inner().write().await;
            inner.test_stage_completed_action_rt("jog")
        };

        sup.control_tick(0.02).await;
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);

        // 50 dwell ticks must perform 0 additional captures.
        for _ in 0..50 {
            sup.control_tick(0.02).await;
        }
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);
        assert_eq!(sup.test_terminal_capture_stats().saturated, 0);

        // Starting action 2 moves already-terminal action 1 to `previous_action` (2 s window):
        // must NOT re-capture action 1, while telemetry still exposes `previous_action` within 2 s.
        let id2 = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("park");
            // Keep `park` running during the tick so reconcile_actions does not auto-cancel it.
            inner.pending_normal_unload = true;
            inner.defer_terminal_capture = false;
            id
        };
        let obs_with_prev = sup.tick(0.02).await;
        assert_eq!(obs_with_prev.action.as_ref().map(|a| a.id), Some(id2));
        assert_eq!(obs_with_prev.previous_action.as_ref().map(|a| a.id), Some(id1));
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);

        // Repeat finish_action on already-terminal id1 must be a strict no-op.
        {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            inner.finish_action(id1, "done", None, None, None, None, None);
            inner.defer_terminal_capture = false;
        }
        sup.control_tick(0.02).await;
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);

        // Expire previous_action past 2 s window and confirm obs omits it without extra capture.
        {
            let mut inner = sup.inner().write().await;
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
        }
        let obs_after_2s = sup.tick(0.02).await;
        assert!(obs_after_2s.previous_action.is_none());
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);

        // Wait for dispatcher delivery: action 1 delivered strictly once.
        let dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < dl {
            if delivered
                .lock()
                .unwrap()
                .iter()
                .any(|o| o.action.as_ref().is_some_and(|a| a.id == id1))
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        tokio::time::sleep(Duration::from_millis(25)).await;
        let ids: Vec<u64> = delivered
            .lock()
            .unwrap()
            .iter()
            .filter_map(|o| o.action.as_ref().map(|a| a.id))
            .collect();
        assert_eq!(ids, vec![id1]);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn edge_trigger_off_tick_ordering_failed_step_and_zero_to_one_cutoff() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;

        // Pre-subscription terminal while subscriber_count == 0 and drain paused.
        sup.test_set_terminal_capture_drain_paused(true);
        let pre_id = {
            let mut inner = sup.inner().write().await;
            inner.test_stage_completed_action_rt("jog")
        };
        sup.control_tick(0.02).await;
        assert_eq!(sup.test_terminal_capture_stats().captured, 1);
        assert_eq!(sup.test_terminal_capture_stats().queued, 1);

        // Unpause capture drain and register first subscriber: pre-subscription record must be
        // cleared from the ring and NOT delivered to the new subscriber.
        sup.test_set_terminal_capture_drain_paused(false);
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        });
        assert_eq!(sup.test_terminal_capture_stats().queued, 0);

        // Action A finishes in-tick (queued in SPSC ring), then Action B finishes off-tick,
        // and `update_recorded_terminal_failed_step` patches Action A before dispatch flushes.
        sup.test_set_terminal_dispatch_paused(true);
        let id_a = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("sequence");
            inner.finish_action(id, "failed", Some("STEP_ERR"), None, None, None, None);
            inner.defer_terminal_capture = false;
            id
        };
        sup.control_tick(0.02).await;
        {
            let inner = sup.inner().read().await;
            inner.update_recorded_terminal_failed_step(id_a, Some(3));
        }
        let id_b = {
            let mut inner = sup.inner().write().await;
            let id = inner.begin_action("jog", None);
            inner.finish_action(id, "done", None, None, None, None, None);
            id
        };
        sup.test_set_terminal_dispatch_paused(false);
        let _ = sup.test_drain_terminal_captures();

        let got = delivered.lock().unwrap().clone();
        let ids: Vec<u64> = got.iter().map(|a| a.id).collect();
        assert_eq!(
            ids,
            vec![id_a, id_b],
            "must skip pre-sub {pre_id} and preserve strict FIFO [A, B]"
        );
        assert_eq!(
            got[0].failed_step,
            Some(3),
            "failed_step patch must apply to in-tick captured action"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn gated_telemetry_delivers_once_terminals_clear_without_new_publication() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        sup.start_dispatcher().unwrap();
        let mut tel_rx = sup.subscribe_telemetry().unwrap();
        let delivered = Arc::new(Mutex::new(Vec::<u64>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.id);
            }
            true
        });

        // Pause broadcaster drain so telemetry is gated on pending terminal.
        sup.test_set_terminal_dispatch_paused(true);
        let id = {
            let mut inner = sup.inner().write().await;
            inner.test_stage_completed_action_rt("jog")
        };
        sup.control_tick(0.02).await;
        tokio::time::sleep(Duration::from_millis(40)).await;
        assert!(
            tel_rx.try_recv().is_err(),
            "telemetry must remain gated while terminal is pending"
        );

        // Unpause WITHOUT running another tick: both terminal AND gated telemetry must arrive.
        sup.test_set_terminal_dispatch_paused(false);
        let tel_obs = tokio::time::timeout(Duration::from_secs(2), tel_rx.recv())
            .await
            .expect("gated telemetry must flush once terminal clears")
            .expect("broadcast ok");
        assert_eq!(tel_obs.action.as_ref().map(|a| a.id), Some(id));
        assert_eq!(*delivered.lock().unwrap(), vec![id]);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn delayed_macro_full_rich_payload_preserved_past_2s_window_under_lock_contention() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        sup.test_set_terminal_fifo_capacity(2);
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        });

        // Create a rich macro off-tick with `episodes`, `motions`, `height_estimates`, `phase_ms`,
        // AND grasp diagnostic fields (`held_width_m`, `close_height_m`), then update `phase` to
        // `"done"` under guarded deferral, finish it in-tick + start park + expire 2 s window.
        let (id_macro, id_park, expected_action) = {
            let mut inner = sup.inner().write().await;
            let id1 = inner.begin_action("place", Some("tag-1".into()));
            inner.set_action_phase(id1, "close", 0.75, Some(2), Some(3.5));
            inner.update_action(id1, |a| {
                a.episodes.push("ep-001".into());
                a.motions.push(MacroMotionDiag {
                    phase: "descend".into(),
                    ms: 145,
                    track_mm: 1.2,
                    track_deg: 0.4,
                    goal_mm: 0.5,
                    goal_deg: 0.2,
                    contact: Some(true),
                });
                a.height_estimates.push(EstimateEntry {
                    source: "depth:metric_table".into(),
                    value: Some(0.085),
                    used: true,
                    record_only: false,
                    reason: None,
                });
                a.held_width_m = Some(0.032);
                a.close_height_m = Some(0.041);
                a.phase_ms.insert("descend".into(), 145);
            });
            inner.defer_terminal_capture = true;
            // Simulate a guarded Legacy completion updating `phase` while mirroring is disabled.
            inner.set_action_phase(id1, "done", 1.0, None, None);
            if let Some(a) = inner.last_action.as_mut() {
                a.phase_ms.insert("close".into(), 80);
            }
            inner.finish_action(id1, "done", None, None, Some(true), None, None);
            let expected = inner.last_action.clone().expect("finished macro slot");
            // Staged terminal (1) + starting park (1) == 2 (allowed).
            assert!(inner.check_terminal_fifo_for_new_action().is_ok());
            let id2 = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            // Now staged (1) + running park (needed=2) == 3 > 2 -> MUST return TerminalsBusy!
            assert!(matches!(
                inner.check_terminal_fifo_for_new_action(),
                Err(crate::safety::SafetyError::TerminalsBusy)
            ));
            // Expire previous_action past 2 s window BEFORE publish + drain.
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
            inner.defer_terminal_capture = false;
            (id1, id2, expected)
        };

        sup.control_tick(0.02).await;
        // Hold SupervisorLock write guard during drain to prove rich payload recovery does NOT
        // depend on `SupervisorLock::try_read()`.
        let _write_hold = sup.inner().write().await;
        let _ = sup.test_drain_terminal_captures();
        drop(_write_hold);

        let got = delivered.lock().unwrap().clone();
        assert_eq!(got.len(), 1);
        assert_eq!(
            got[0], expected_action,
            "delayed rich terminal ActionStatus must match full terminal payload field-for-field"
        );
        assert_eq!(got[0].id, id_macro);
        assert_eq!(got[0].phase.as_deref(), Some("done"));
        assert_eq!(got[0].held_width_m, Some(0.032));
        assert_eq!(got[0].close_height_m, Some(0.041));
        assert_ne!(id_macro, id_park);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn deferred_staging_materializes_before_slot_overwrite_preserving_all_ids_in_order() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        });

        // Finish A under deferral, start B (A -> previous_action), then start C (B supersedes and
        // overwrites A in previous_action) BEFORE `capture_edge_terminals` runs.
        let (id_a, id_b, id_c) = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let a = inner.begin_action_rt("jog");
            inner.finish_action(a, "done", None, None, None, None, None);
            let b = inner.begin_action_rt("jog");
            let c = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            inner.defer_terminal_capture = false;
            (a, b, c)
        };

        sup.control_tick(0.02).await;
        let _ = sup.test_drain_terminal_captures();

        let got = delivered.lock().unwrap().clone();
        let ids: Vec<u64> = got.iter().map(|a| a.id).collect();
        assert_eq!(
            ids,
            vec![id_a, id_b],
            "materialized deferred A and superseded B must both be delivered in strict FIFO order (C={id_c} still running)"
        );
        assert_eq!(got[0].state.as_str(), "done");
        assert_eq!(got[1].state.as_str(), "cancelled");
        assert_eq!(got[1].code.as_deref(), Some("SUPERSEDED"));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn publish_seq_barrier_carried_through_broadcaster_prevents_stale_snapshot_pairing() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<(u64, f64)>::new()));
        let delivered_c = Arc::clone(&delivered);
        // Subscribe directly on `TerminalBroadcaster` without spawning `armd-dispatch` so the
        // manual drains below are 100% deterministic and cannot race with a background thread.
        let sub_id = sup.terminals.subscribe(Box::new(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push((a.id, obs.timestamp));
            }
            true
        }));

        // Snapshot N published before terminal A.
        sup.control_tick(0.02).await;
        let snap_n = sup.hub().latest().expect("snapshot N");

        // Legacy tick publishes terminal A at snapshot N+1 while dispatcher is paused on N.
        let id_a = {
            let mut inner = sup.inner().write().await;
            inner.test_stage_completed_action_rt("jog")
        };
        sup.control_tick(0.02).await;
        let snap_n1 = sup.hub().latest().expect("snapshot N+1");
        assert!(snap_n1.meta.seq > snap_n.meta.seq);

        // Off-tick Legacy action start drains A from the SPSC ring into `TerminalBroadcaster`
        // (bypassing the ring barrier with `respect_publish_barrier = false`).
        let _id_b = {
            let mut inner = sup.inner().write().await;
            inner.begin_action("jog", None)
        };
        assert_eq!(sup.test_terminal_capture_stats().queued, 0);

        // Attempting to flush `TerminalBroadcaster` with stale snapshot N MUST NOT emit A!
        let blocked = sup.terminals.drain_all(&snap_n.obs, snap_n.meta.seq);
        assert!(
            blocked,
            "drain_all against stale snapshot N must hold terminal A gated on required_publish_seq"
        );
        assert!(
            delivered.lock().unwrap().is_empty(),
            "terminal A must not be paired with stale snapshot N"
        );

        // Once snapshot N+1 is used, terminal A flushes paired with snapshot N+1.
        let blocked_after = sup.terminals.drain_all(&snap_n1.obs, snap_n1.meta.seq);
        assert!(!blocked_after);
        let got = delivered.lock().unwrap().clone();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].0, id_a);
        assert_eq!(got[0].1, snap_n1.obs.timestamp);
        sup.terminals.unsubscribe(sub_id);
    }

    #[test]
    fn install_driver_does_not_advance_feedback_history_before_first_poll() {
        use std::sync::atomic::{AtomicBool, Ordering as AOrd};

        struct InstantPollDriver {
            dof: usize,
            valid: Arc<AtomicBool>,
            read_ok: Arc<AtomicBool>,
            instant: Instant,
        }

        impl crate::driver::ArmDriver for InstantPollDriver {
            fn read_joint_states(
                &mut self,
            ) -> Result<Vec<crate::model::JointState>, crate::driver::ArmDriverError> {
                if !self.read_ok.load(AOrd::Acquire) {
                    return Err(crate::driver::ArmDriverError::Protocol(
                        "injected poll fail".into(),
                    ));
                }
                Ok(vec![crate::model::JointState::default(); self.dof])
            }
            fn write_joint_targets(
                &mut self,
                _targets: &[crate::JointTarget],
            ) -> Result<(), crate::driver::ArmDriverError> {
                Ok(())
            }
            fn set_torque(&mut self, _enabled: bool) -> Result<(), crate::driver::ArmDriverError> {
                Ok(())
            }
            fn is_feedback_valid(&self) -> bool {
                self.valid.load(AOrd::Acquire)
            }
            fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
                out.clear();
                out.resize(self.dof, Some(self.instant));
            }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(self.dof, Some(Duration::from_millis(5)));
            }
        }

        let cfg = ArmConfig::rebot_dm();
        let dof = cfg.dof;
        let mut inner = SupervisorInner::new(cfg);
        let valid = Arc::new(AtomicBool::new(true));
        let read_ok = Arc::new(AtomicBool::new(false));
        let drv = InstantPollDriver {
            dof,
            valid: Arc::clone(&valid),
            read_ok: Arc::clone(&read_ok),
            instant: Instant::now(),
        };

        let seqs_before = inner.core.feedback().seqs().to_vec();
        let interlock_before = (
            inner.last_poll_enable_state,
            inner.last_poll_partial_enable,
            inner.last_poll_feedback_valid,
            inner.last_poll_oldest_age,
        );
        inner.install_driver(Box::new(drv));
        assert_eq!(
            (
                inner.last_poll_enable_state,
                inner.last_poll_partial_enable,
                inner.last_poll_feedback_valid,
                inner.last_poll_oldest_age,
            ),
            interlock_before,
            "install_driver must not mutate hardware park/torque-off interlock last_poll_* fields"
        );
        assert_eq!(
            inner.core.feedback().seqs(),
            seqs_before.as_slice(),
            "install_driver must not advance ArmCore feedback sequences before the first control poll"
        );
        assert!(
            inner.cached_hardware_feedback_fresh(Duration::from_millis(250)),
            "seeded poll-age fallback keeps freshness valid immediately after attachment"
        );

        // First poll fails -> cached freshness fails closed while sequences and interlock stay unchanged.
        inner.poll_hardware_rx(None, None, None);
        assert!(
            !inner.cached_hardware_feedback_fresh(Duration::from_millis(250)),
            "failed poll must make cached freshness fail closed"
        );
        assert_eq!(inner.core.feedback().seqs(), seqs_before.as_slice());
        assert_eq!(
            (
                inner.last_poll_enable_state,
                inner.last_poll_partial_enable,
                inner.last_poll_feedback_valid,
                inner.last_poll_oldest_age,
            ),
            interlock_before,
            "failed poll must not mutate hardware interlock last_poll_* fields"
        );

        // Subsequent valid poll -> freshness becomes true and sequences advance by exactly 1.
        read_ok.store(true, AOrd::Release);
        inner.poll_hardware_rx(None, None, None);
        assert!(
            inner.cached_hardware_feedback_fresh(Duration::from_millis(250)),
            "successful poll must restore cached freshness"
        );
        for (i, (&after, &before)) in inner
            .core
            .feedback()
            .seqs()
            .iter()
            .zip(seqs_before.iter())
            .enumerate()
        {
            assert_eq!(
                after,
                before + 1,
                "joint {i} feedback sequence must advance exactly once on first valid poll"
            );
        }
        // Subsequent read error after a valid poll matches baseline `last_poll_*` behavior
        // (`feedback_valid=false, oldest_age=None`, enable state unchanged) while failing closed
        // in `capture_driver_cache`.
        let enable_after_valid = (inner.last_poll_enable_state, inner.last_poll_partial_enable);
        read_ok.store(false, AOrd::Release);
        inner.poll_hardware_rx(None, None, None);
        assert!(
            !inner.cached_hardware_feedback_fresh(Duration::from_millis(250)),
            "read error after valid poll must fail closed in capture_driver_cache"
        );
        assert_eq!(
            (inner.last_poll_enable_state, inner.last_poll_partial_enable),
            enable_after_valid,
            "read error must preserve baseline enable interlock fields"
        );
        assert!(!inner.last_poll_feedback_valid);
        assert!(inner.last_poll_oldest_age.is_none());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn delayed_cancellation_clears_mirrored_workspace_violation_diagnostics() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        });

        let (id_a, expected) = {
            let mut inner = sup.inner().write().await;
            let id = inner.begin_action("grasp", None);
            inner.set_action_workspace_violation(
                id,
                crate::kinematics::WorkspaceViolationDetail {
                    constraint_id: Some("table_plane".into()),
                    geometry_id: Some("jaw_tip".into()),
                    violation_phase: Some("descend".into()),
                    reason: Some("z below table".into()),
                },
            );
            // Guarded cancellation clears workspace violation fields (`finish_action`).
            inner.defer_terminal_capture = true;
            inner.finish_action(id, "cancelled", Some("STOPPED"), None, None, None, None);
            let exp = inner.last_action.clone().expect("cancelled slot");
            assert!(exp.constraint_id.is_none());
            assert!(exp.geometry_id.is_none());
            assert!(exp.violation_phase.is_none());
            assert!(exp.reason.is_none());
            let _id2 = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
            inner.defer_terminal_capture = false;
            (id, exp)
        };

        sup.control_tick(0.02).await;
        let _write_hold = sup.inner().write().await;
        let _ = sup.test_drain_terminal_captures();
        drop(_write_hold);

        let got = delivered.lock().unwrap().clone();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].id, id_a);
        assert_eq!(
            got[0], expected,
            "delayed cancellation must clear previously mirrored workspace violation fields"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn post_publish_and_shutdown_captures_synthesize_self_contained_observation() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<ArmObservation>::new()));
        let delivered_c = Arc::clone(&delivered);
        // Subscribe directly on `TerminalBroadcaster` (no background dispatcher) for deterministic
        // manual drain verification.
        let sub_id = sup.terminals.subscribe(Box::new(move |obs| {
            delivered_c.lock().unwrap().push(obs.clone());
            true
        }));

        // Publish baseline snapshot N with initial epoch and timestamp.
        sup.control_tick(0.02).await;
        let snap_n = sup.hub().latest().expect("snapshot N");

        // Simulate post-publish capture with `publish_seq: None` (like persistence saturation or
        // control_stop) after moving the gripper joint and advancing timestamp without another
        // publish_at. Verify self-contained field consistency (`positions[gripper.joint] == gripper.rad`,
        // `feedback_age_ms`) AND `DrainOutcome.last_synthetic_action_id`.
        let gripper_idx = {
            let inner = sup.inner().read().await;
            inner.config.gripper_index().expect("rebot_dm has gripper")
        };
        let moved_gripper_rad = 0.42f32;
        let id_sat = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("gripper_zero_check");
            inner.measured_positions[gripper_idx] = moved_gripper_rad;
            inner.finish_action(id, "failed", Some("PERSIST_FAILED"), None, None, None, None);
            let ts_after = snap_n.obs.timestamp + 1.25;
            let _ = inner.capture_edge_terminals(ts_after, None);
            inner.defer_terminal_capture = false;
            id
        };
        let out_sat = sup.terminal_capture_channel.drain_into(
            &sup.terminals,
            &sup.hub,
            Some(&snap_n),
            &[],
            &sup.template_obs,
            true,
        );
        assert_eq!(out_sat.drained, 1);
        assert_eq!(
            out_sat.last_synthetic_action_id,
            Some(id_sat),
            "synthetic post-publish drain must report action_id so stale running pending_telemetry is suppressed"
        );
        assert!(!sup.terminals.drain_all(&snap_n.obs, snap_n.meta.seq));
        {
            let got = delivered.lock().unwrap();
            assert_eq!(got.len(), 1);
            assert_eq!(got[0].action.as_ref().map(|a| a.id), Some(id_sat));
            assert!(
                (got[0].timestamp - (snap_n.obs.timestamp + 1.25)).abs() < 1e-6,
                "publish_seq: None capture must synthesize self-contained observation at capture timestamp"
            );
            let grip = got[0].gripper.as_ref().expect("synthesized obs must include gripper");
            assert_eq!(grip.joint, gripper_idx);
            assert!(
                (got[0].positions[gripper_idx] - moved_gripper_rad).abs() < 1e-6
                    && (grip.rad - moved_gripper_rad).abs() < 1e-6,
                "synthesized positions[gripper.joint] ({}) and gripper.rad ({}) must agree",
                got[0].positions[gripper_idx],
                grip.rad
            );
            assert_eq!(got[0].feedback_age_ms.len(), got[0].positions.len());
        }

        // Simulate hub closing between loop check and normal `drain_into(..., respect_publish_barrier = true)`
        // when a queued record has `publish_seq > final_snap.meta.seq`: unified `enforce_barrier`
        // MUST synthesize `obs_override` (NOT carry an unmet `required_publish_seq`), so `drain_all`
        // delivers it against `snap_n.meta.seq` without stranding.
        let id_shut = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("jog");
            inner.finish_action(id, "done", None, None, None, None, None);
            let ts_shut = snap_n.obs.timestamp + 2.50;
            let future_seq = snap_n.meta.seq + 10;
            let _ = inner.capture_edge_terminals(ts_shut, Some(future_seq));
            inner.defer_terminal_capture = false;
            id
        };
        sup.hub().close();
        let out_shut = sup.terminal_capture_channel.drain_into(
            &sup.terminals,
            &sup.hub,
            Some(&snap_n),
            &[],
            &sup.template_obs,
            true,
        );
        assert_eq!(out_shut.drained, 1);
        assert_eq!(out_shut.last_synthetic_action_id, Some(id_shut));
        let blocked = sup.terminals.drain_all(&snap_n.obs, snap_n.meta.seq);
        assert!(
            !blocked,
            "hub_closed drain must synthesize self-contained obs so drain_all does not strand"
        );
        let got = delivered.lock().unwrap();
        assert_eq!(got.len(), 2);
        assert_eq!(got[1].action.as_ref().map(|a| a.id), Some(id_shut));
        assert!(
            (got[1].timestamp - (snap_n.obs.timestamp + 2.50)).abs() < 1e-6,
            "hub_closed drain ahead of final_snap.seq must emit captured observation"
        );
        sup.terminals.unsubscribe(sub_id);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn delayed_recovery_preserves_long_error_message_and_utf8_boundary_label() {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let delivered = Arc::new(Mutex::new(Vec::<ActionStatus>::new()));
        let delivered_c = Arc::clone(&delivered);
        let sub_id = sup.terminals.subscribe(Box::new(move |obs| {
            if let Some(a) = obs.action.as_ref() {
                delivered_c.lock().unwrap().push(a.clone());
            }
            true
        }));

        // 1) 127-byte ASCII prefix + multibyte UTF-8 char (`'é'`, 2 bytes -> total 129 bytes)
        //    produces a 127-byte `HolderLabel` that MUST NOT overwrite the original cached string.
        let utf8_reason = format!("{}é-suffix", "a".repeat(127));
        assert!(utf8_reason.len() > HolderLabel::CAPACITY);
        assert_eq!(HolderLabel::new(&utf8_reason).len(), 127);

        let (id_utf8, expected_utf8) = {
            let mut inner = sup.inner().write().await;
            let id = inner.begin_action("grasp", None);
            inner.set_action_workspace_violation(
                id,
                crate::kinematics::WorkspaceViolationDetail {
                    constraint_id: Some("c1".into()),
                    geometry_id: Some("g1".into()),
                    violation_phase: Some("descend".into()),
                    reason: Some(utf8_reason.clone()),
                },
            );
            inner.defer_terminal_capture = true;
            inner.finish_action(
                id,
                "failed",
                Some("WORKSPACE_VIOLATION"),
                None,
                None,
                None,
                None,
            );
            let exp = inner.last_action.clone().expect("workspace violation slot");
            let _id2 = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
            inner.defer_terminal_capture = false;
            (id, exp)
        };
        sup.control_tick(0.02).await;
        let _ = sup.test_drain_terminal_captures();

        // 2) >256-byte persistence worker error message mirrored via `mirror_action_message`
        //    must survive 2 s expiry without 256-byte truncation.
        let long_msg = format!("Failed to sync gripper calibration: {}", "x".repeat(360));
        assert!(long_msg.len() > 256);
        let id_long = {
            let mut inner = sup.inner().write().await;
            inner.defer_terminal_capture = true;
            let id = inner.begin_action_rt("gripper_zero_check");
            sup.terminal_capture_channel
                .mirror_action_message(id, &long_msg);
            inner.finish_action(
                id,
                "failed",
                Some("PERSIST_FAILED"),
                Some(long_msg.clone()),
                None,
                None,
                None,
            );
            let _id2 = inner.begin_action_rt("park");
            inner.pending_normal_unload = true;
            if let Some((_, until)) = inner.previous_action.as_mut() {
                *until = 0.0;
            }
            inner.defer_terminal_capture = false;
            id
        };
        sup.control_tick(0.02).await;
        let _ = sup.test_drain_terminal_captures();

        let got = delivered.lock().unwrap().clone();
        assert_eq!(got.len(), 3, "expected [id_utf8, superseded park, id_long]");
        assert_eq!(got[0].id, id_utf8);
        assert_eq!(got[0], expected_utf8, "UTF-8 boundary truncated label must preserve full cached reason");
        assert_eq!(got[0].reason.as_deref(), Some(utf8_reason.as_str()));
        assert_eq!(got[1].code.as_deref(), Some("SUPERSEDED"));
        assert_eq!(got[2].id, id_long);
        assert_eq!(
            got[2].message.as_deref(),
            Some(long_msg.as_str()),
            "delayed >256-byte persistence error message must be recovered losslessly"
        );
        sup.terminals.unsubscribe(sub_id);
    }

    #[test]
    fn cached_feedback_freshness_respects_numeric_validity_poll_valid_instants_and_250ms_boundary() {
        let t0 = Instant::now();
        let max_age = Duration::from_millis(250);
        // Driverless: solely gated by feedback_numeric_valid.
        assert!(evaluate_cached_feedback_fresh(
            true, false, false, 7, &[], None, None, t0, max_age
        ));
        assert!(!evaluate_cached_feedback_fresh(
            false, true, false, 7, &[], None, None, t0, max_age
        ));

        // Driver with per-joint instants: fresh at +240ms, stale at +260ms, false when numeric
        // invalid or last_poll_feedback_valid is false (without mutating instants!).
        let instants = vec![Some(t0); 7];
        assert!(evaluate_cached_feedback_fresh(
            true,
            true,
            true,
            7,
            &instants,
            None,
            None,
            t0 + Duration::from_millis(240),
            max_age,
        ));
        assert!(!evaluate_cached_feedback_fresh(
            true,
            true,
            true,
            7,
            &instants,
            None,
            None,
            t0 + Duration::from_millis(260),
            max_age,
        ));
        assert!(!evaluate_cached_feedback_fresh(
            false,
            true,
            true,
            7,
            &instants,
            None,
            None,
            t0 + Duration::from_millis(10),
            max_age,
        ));
        assert!(!evaluate_cached_feedback_fresh(
            true,
            false,
            true,
            7,
            &instants,
            None,
            None,
            t0 + Duration::from_millis(10),
            max_age,
        ));

        // Fallback (empty instants): conservative oldest_age + (now - poll_before) <= max_age.
        assert!(evaluate_cached_feedback_fresh(
            true,
            true,
            true,
            7,
            &[],
            Some(Duration::from_millis(200)),
            Some(t0),
            t0 + Duration::from_millis(40),
            max_age,
        ));
        assert!(!evaluate_cached_feedback_fresh(
            true,
            true,
            true,
            7,
            &[],
            Some(Duration::from_millis(200)),
            Some(t0),
            t0 + Duration::from_millis(60),
            max_age,
        ));
        // Cleared poll cache (after read/lock error) fails closed.
        assert!(!evaluate_cached_feedback_fresh(
            true, false, true, 7, &[], None, None, t0, max_age
        ));
    }
}
