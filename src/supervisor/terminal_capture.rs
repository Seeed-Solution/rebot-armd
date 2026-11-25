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

use crate::model::{ArmObservation, ControlMode, DriverCapabilities, EnableState, HolderLabel};
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

/// Returns true if `action` carries rich off-RT macro/diagnostic fields that exceed the inline
/// `TerminalCaptureRecord` representation.
#[inline]
pub(crate) fn action_has_rich_fields(action: &ActionStatus) -> bool {
    !action.episodes.is_empty()
        || !action.height_estimates.is_empty()
        || !action.yaw_estimates.is_empty()
        || !action.width_estimates.is_empty()
        || !action.motions.is_empty()
        || action.measurement.is_some()
        || action.align.is_some()
        || action.orientation.is_some()
        || action.height_source.is_some()
        || action.height_reason.is_some()
        || action.depth_anchor.is_some()
        || action.border.is_some()
        || action.probe_mode.is_some()
        || action.recentre.is_some()
        || action.post_lift.is_some()
        || action.pre_open.is_some()
        || action.approach.is_some()
        || action.approach_reason.is_some()
        || action.approach_signs.is_some()
        || action.approach_flipped.is_some()
        || action.iteration.is_some()
        || action.error_px.is_some()
        || action.final_offset_px.is_some()
        || action.acquire_height_m.is_some()
        || action.descend_contact.is_some()
        || action.phase_ms.len() > 12
        || action.message.as_ref().is_some_and(|m| m.len() > 256)
        || action
            .durability_warning
            .as_ref()
            .is_some_and(|w| w.len() > 256)
        || action
            .tag
            .as_ref()
            .is_some_and(|s| s.len() > HolderLabel::CAPACITY)
        || action
            .task
            .as_ref()
            .is_some_and(|s| s.len() > HolderLabel::CAPACITY)
        || action
            .sequence_id
            .as_ref()
            .is_some_and(|s| s.len() > HolderLabel::CAPACITY)
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
    pub(crate) has_rich_fields: bool,
    pub(crate) dof: u8,
    pub(crate) positions: [f32; MAX_DOF],
    pub(crate) velocities: [f32; MAX_DOF],
    pub(crate) torques: [f32; MAX_DOF],
    pub(crate) temperatures: [f32; MAX_DOF],
    pub(crate) commanded_positions: [f32; MAX_DOF],
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
fn opt_label(s: Option<&String>) -> (Option<HolderLabel>, bool) {
    match s {
        Some(v) => (Some(HolderLabel::new(v)), v.len() > HolderLabel::CAPACITY),
        None => (None, false),
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
        let mut rich = action_has_rich_fields(action);

        let (has_message, message) = match action.message.as_deref() {
            Some(m) => {
                let bt = BoundedText::<256>::new(m);
                if bt.is_truncated() {
                    rich = true;
                }
                (true, bt)
            }
            None => (false, BoundedText::empty()),
        };
        let (has_durability_warning, durability_warning) = match action.durability_warning.as_deref()
        {
            Some(w) => {
                let bt = BoundedText::<256>::new(w);
                if bt.is_truncated() {
                    rich = true;
                }
                (true, bt)
            }
            None => (false, BoundedText::empty()),
        };
        let (tag, t1) = opt_label(action.tag.as_ref());
        let (task, t2) = opt_label(action.task.as_ref());
        let (sequence_id, t3) = opt_label(action.sequence_id.as_ref());
        let (step_type, t4) = opt_label(action.step_type.as_ref());
        let (constraint_id, t5) = opt_label(action.constraint_id.as_ref());
        let (geometry_id, t6) = opt_label(action.geometry_id.as_ref());
        let (violation_phase, t7) = opt_label(action.violation_phase.as_ref());
        let (reason, t8) = opt_label(action.reason.as_ref());
        if t1 || t2 || t3 || t4 || t5 || t6 || t7 || t8 {
            rich = true;
        }

        let mut phase_ms = [(BoundedText::<24>::empty(), 0u32); 12];
        let mut phase_ms_len = 0u8;
        for (k, &v) in &action.phase_ms {
            if usize::from(phase_ms_len) < phase_ms.len() {
                let bt = BoundedText::<24>::new(k);
                if bt.is_truncated() {
                    rich = true;
                }
                phase_ms[usize::from(phase_ms_len)] = (bt, v);
                phase_ms_len += 1;
            } else {
                rich = true;
            }
        }

        let state = inner.safety.state();
        let fault = if state.is_stopped() || inner.safety.has_latched_fault() {
            inner.safety.fault()
        } else {
            None
        };
        let dof = inner.config.dof.min(MAX_DOF) as u8;

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
            has_rich_fields: rich,
            dof,
            positions: copy_f32_slice(&inner.measured_positions),
            velocities: copy_f32_slice(&inner.measured_velocities),
            torques: copy_f32_slice(&inner.torques),
            temperatures: copy_f32_slice(&inner.temperatures),
            commanded_positions: copy_f32_slice(&inner.commanded_positions),
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
            driver_caps: inner.driver_caps,
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
    /// (preserving `episodes`, `motions`, `height_estimates`, `measurement`, and full strings).
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
        if self.holding.is_some() {
            dst.holding = self.holding;
        }
        if self.gripper_gap_rad.is_some() {
            dst.gripper_gap_rad = self.gripper_gap_rad;
        }
        if self.settled.is_some() {
            dst.settled = self.settled;
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
        obs.is_torqued = self.is_torqued;
        obs.disable_pending = self.disable_pending;
        obs.feedback_valid = self.feedback_valid;
        obs.epoch_id = self.epoch_id;
        obs.safety_state = self.safety_state.to_safety_state();
        obs.fault = self.fault;
        obs.control_mode = self.control_mode;
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

/// Shared off-thread consumer and lock-free atomic accounting for terminal capture.
/// `rx` and `rich_action_cache` are NEVER locked on `armd-control`; lock order is strictly
/// `channel.rx -> terminals.inner` (and `rich_action_cache` is a leaf mutex).
pub(crate) struct TerminalCaptureChannel {
    rx: Mutex<rtrb::Consumer<TerminalCaptureRecord>>,
    /// Off-RT mirror of rich macro `ActionStatus` payloads (bounded to `TERMINAL_FIFO_CAPACITY`)
    /// so delayed dispatch past the 2 s window never drops `episodes`, `motions`, `height_estimates`,
    /// or long strings without touching `SupervisorLock`. Never accessed on `armd-control`.
    rich_action_cache: Mutex<VecDeque<ActionStatus>>,
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

    /// Off-RT mirror for rich macro `ActionStatus` payloads (never called on `armd-control`).
    pub(crate) fn mirror_rich_action(&self, action: &ActionStatus) {
        if !action_has_rich_fields(action) {
            return;
        }
        let mut cache = self
            .rich_action_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if let Some(existing) = cache.iter_mut().find(|a| a.id == action.id) {
            existing.clone_from(action);
            return;
        }
        while cache.len() >= TERMINAL_FIFO_CAPACITY {
            cache.pop_front();
        }
        cache.push_back(action.clone());
    }

    fn take_rich_action(&self, action_id: u64) -> Option<ActionStatus> {
        let mut cache = self
            .rich_action_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let pos = cache.iter().position(|a| a.id == action_id)?;
        let item = cache.remove(pos);
        // Prune older consumed entries boundedly.
        while cache.front().is_some_and(|a| a.id < action_id) && cache.len() > TERMINAL_FIFO_CAPACITY / 2 {
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
    ) -> usize {
        if self.drain_paused.load(Ordering::Acquire) {
            return 0;
        }
        let mut drained = 0usize;
        let hub_closed = hub.is_closed();
        let consumed_seq = latest_snap.map_or(0, |s| s.meta.seq);
        let latest_obs = latest_snap.map(|s| &s.obs);
        while let Ok(head) = rx.peek() {
            // Bind eligibility strictly to `consumed_seq` (the `meta.seq` of the observation
            // actually being consumed) so a concurrent publish cannot open the barrier against
            // a stale snapshot.
            if respect_publish_barrier && !hub_closed {
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

            // Preserve HEAD's wire contract: when `hub` has published, `drain_all` clones the
            // latest `base_obs` and injects `action`. Only when no published snapshot exists
            // (`hub.published_seq() == 0 && latest_obs.is_none()`), synthesize a full
            // `ArmObservation` off-thread from `rec` overlaid onto `template_obs`.
            let obs_override = if latest_obs.is_none() && hub.published_seq() == 0 {
                Some(rec.build_observation(template_obs, action.clone()))
            } else {
                None
            };

            // Enqueue into `TerminalBroadcaster` (updating `max_fifo_len` & `pending_total` with
            // `Release`) STRICTLY BEFORE decrementing `self.inflight` (`Release`).
            let _ = terminals.record_finished_with_seq(rec.terminal_seq, action, obs_override);
            self.inflight.fetch_sub(1, Ordering::Release);
            drained += 1;
        }
        drained
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
    ) -> usize {
        if self.drain_paused.load(Ordering::Acquire) {
            return 0;
        }
        let mut rx = self.rx.lock().unwrap_or_else(|e| e.into_inner());
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
        let mut rx = self.rx.lock().unwrap_or_else(|e| e.into_inner());
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
            let _ = channel.take_rich_action(rec.action_id);
            return Some((*slot).clone());
        }
    }
    if let Some(obs) = latest_obs {
        for slot in [obs.previous_action.as_ref(), obs.action.as_ref()]
            .into_iter()
            .flatten()
        {
            if match_action(slot, rec) {
                let _ = channel.take_rich_action(rec.action_id);
                return Some(slot.clone());
            }
        }
    }
    if rec.has_rich_fields {
        if let Some(mut cached) = channel.take_rich_action(rec.action_id) {
            rec.apply_terminal_overlay(&mut cached);
            return Some(cached);
        }
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

        // Create a rich macro off-tick with `episodes`, `motions`, `height_estimates`, and `phase_ms`
        // (via `update_action`), then finish it in-tick + park + expire 2 s window.
        let (id_macro, id_park) = {
            let mut inner = sup.inner().write().await;
            let id1 = inner.begin_action("place", Some("tag-1".into()));
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
                a.phase_ms.insert("descend".into(), 145);
            });
            inner.defer_terminal_capture = true;
            if let Some(a) = inner.last_action.as_mut() {
                a.phase_ms.insert("close".into(), 80);
            }
            inner.finish_action(id1, "done", None, None, Some(true), None, None);
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
            (id1, id2)
        };

        sup.control_tick(0.02).await;
        // Hold SupervisorLock write guard during drain to prove rich payload recovery does NOT
        // depend on `SupervisorLock::try_read()`.
        let _write_hold = sup.inner().write().await;
        let _ = sup.test_drain_terminal_captures();
        drop(_write_hold);

        let got = delivered.lock().unwrap().clone();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].id, id_macro);
        assert_eq!(got[0].tag.as_deref(), Some("tag-1"));
        assert_eq!(got[0].episodes, vec!["ep-001".to_string()]);
        assert_eq!(got[0].motions.len(), 1);
        assert_eq!(got[0].motions[0].phase, "descend");
        assert_eq!(got[0].height_estimates.len(), 1);
        assert_eq!(got[0].height_estimates[0].source, "depth:metric_table");
        assert_eq!(got[0].phase_ms.get("descend"), Some(&145));
        assert_eq!(got[0].phase_ms.get("close"), Some(&80));
        assert_ne!(id_macro, id_park);
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
