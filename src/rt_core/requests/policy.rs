//! Owner-routed policy lifecycle requests (Stage 3 C2 / PR Y).
//!
//! Moves WS `/api/v1/action/policy/{start,stop}` and load/runner completion onto `Copy` broker
//! commands (`ReserveLoad`, `BeginPolicy`, `EndPolicy`) so `armd-control` performs strictly zero
//! heap allocations, zero heap frees, zero file I/O, and zero `SupervisorLock` reads while
//! preserving all real-arm safety, operator, lease, release-gate, and stop-ordering invariants.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use crate::auth::OperatorGrant;
use crate::model::HolderLabel;
use crate::motion::PolicyStats;
use crate::policy_runtime::runner::RtGateBinding;
use crate::policy_runtime::{PolicyMode, DEFAULT_MAX_FIRST_STEP_RAD};
use crate::rt_core::broker::{ActionId, BootId, Descriptor};
use crate::rt_core::ops::{CommitCtx, Committed};
use crate::rt_core::requests::lease::ConnectionHandle;
use crate::rt_core::scratch::set_str;
use crate::safety::{FaultReason, SafetyError, SafetyState};
use crate::supervisor::streams::push_str_bounded;
use crate::supervisor::{AdmissionStamp, SupervisorInner};

/// Fixed count of exclusive-lifetime per-attempt cancellation slots (strictly greater than
/// `Broker` command ring capacity 32).
pub const POLICY_CANCEL_SLOTS: usize = 64;

#[derive(Default)]
struct CancelSlot {
    token: AtomicU64,
    cancelled: AtomicBool,
}

#[derive(Clone, Copy, Default)]
struct AllocEntry {
    token: u64,
    max_admitted_req_id: u64,
    retired_by_caller: bool,
}

struct AllocState {
    next_token: u64,
    entries: [AllocEntry; POLICY_CANCEL_SLOTS],
}

/// Exclusive-lifetime lock-free cancellation table shared between `ArmSupervisor` and
/// `SupervisorInner` (zero RT allocs, zero RT locks).
///
/// A slot stays exclusively owned by `token` until ALL of the following hold:
/// 1. `retired_by_caller == true` (the caller / runner dropped its [`PolicyTokenGuard`]);
/// 2. `last_completed_policy_req_id` (`Acquire` load strictly BEFORE reading active tokens)
///    is `>= max_admitted_req_id` (every admitted broker descriptor for `token` has fully
///    completed its final slot read and active-token update on `armd-control`);
/// 3. `active_load_token != token && active_run_token != token`.
pub struct PolicyCancelSlots {
    slots: [CancelSlot; POLICY_CANCEL_SLOTS],
    active_load_token: AtomicU64,
    active_run_token: AtomicU64,
    last_completed_policy_req_id: AtomicU64,
    alloc: Mutex<AllocState>,
}

impl Default for PolicyCancelSlots {
    fn default() -> Self {
        Self::new()
    }
}

impl PolicyCancelSlots {
    pub fn new() -> Self {
        Self {
            slots: std::array::from_fn(|_| CancelSlot::default()),
            active_load_token: AtomicU64::new(0),
            active_run_token: AtomicU64::new(0),
            last_completed_policy_req_id: AtomicU64::new(0),
            alloc: Mutex::new(AllocState {
                next_token: 1,
                entries: [AllocEntry::default(); POLICY_CANCEL_SLOTS],
            }),
        }
    }

    /// Claims an exclusive cancellation slot off-thread before submitting `ReserveLoad`.
    /// Refuses with `"MOTION_BUSY"` if all 64 slots are still exclusively owned.
    pub fn claim_slot(self: &Arc<Self>) -> Result<PolicyTokenGuard, &'static str> {
        let mut st = self.alloc.lock().unwrap_or_else(|e| e.into_inner());
        // Acquire completion watermark STRICTLY BEFORE loading active tokens (happens-before
        // `mark_policy_req_completed`'s Release store after final slot read & active-token update).
        let completed = self.last_completed_policy_req_id.load(Ordering::Acquire);
        let active_load = self.active_load_token.load(Ordering::Acquire);
        let active_run = self.active_run_token.load(Ordering::Acquire);

        let mut free_idx = None;
        for (i, entry) in st.entries.iter_mut().enumerate() {
            if entry.token != 0
                && entry.retired_by_caller
                && completed >= entry.max_admitted_req_id
                && active_load != entry.token
                && active_run != entry.token
            {
                self.slots[i].token.store(0, Ordering::Release);
                self.slots[i].cancelled.store(false, Ordering::Release);
                *entry = AllocEntry::default();
            }
            if free_idx.is_none() && entry.token == 0 {
                free_idx = Some(i);
            }
        }
        let Some(idx) = free_idx else {
            return Err("MOTION_BUSY");
        };
        let token = st.next_token;
        st.next_token = st.next_token.wrapping_add(1).max(1);
        self.slots[idx].cancelled.store(false, Ordering::Release);
        self.slots[idx].token.store(token, Ordering::Release);
        st.entries[idx] = AllocEntry {
            token,
            max_admitted_req_id: 0,
            retired_by_caller: false,
        };
        Ok(PolicyTokenGuard {
            slots: Arc::clone(self),
            slot: idx as u8,
            token,
            disarmed: false,
        })
    }

    /// Records the broker-assigned `RequestId` for an admitted descriptor strictly BEFORE the
    /// descriptor is pushed into the command ring (never called on failed admission).
    pub fn record_admitted_req(&self, slot: u8, token: u64, req_id: u64) {
        let mut st = self.alloc.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(entry) = st.entries.get_mut(usize::from(slot)) {
            if entry.token == token {
                entry.max_admitted_req_id = entry.max_admitted_req_id.max(req_id);
            }
        }
    }

    /// Lock-free, zero-alloc cancellation check on `armd-control`.
    #[inline]
    pub fn is_cancelled(&self, slot: u8, token: u64) -> bool {
        if token == 0 {
            return false;
        }
        let Some(s) = self.slots.get(usize::from(slot)) else {
            return true;
        };
        s.token.load(Ordering::Acquire) == token && s.cancelled.load(Ordering::Acquire)
    }

    #[inline]
    pub fn set_active_load_token(&self, token: u64) {
        self.active_load_token.store(token, Ordering::Release);
    }

    #[inline]
    pub fn set_active_run_token(&self, token: u64) {
        self.active_run_token.store(token, Ordering::Release);
    }

    /// Publishes post-completion watermark with `Release` strictly AFTER `commit_policy` (or
    /// refusal/expiry) finishes its final slot read and updates active tokens.
    #[inline]
    pub fn mark_policy_req_completed(&self, req_id: u64) {
        if req_id != 0 {
            self.last_completed_policy_req_id
                .fetch_max(req_id, Ordering::Release);
        }
    }

    #[cfg(test)]
    pub fn active_occupied_slots(&self) -> usize {
        self.alloc
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .entries
            .iter()
            .filter(|e| e.token != 0)
            .count()
    }
}

/// Armed per-attempt cancellation guard spanning `ReserveLoad` -> off-thread load/warmup ->
/// `BeginPolicy` -> `spawn_executor_policy_runner` handoff -> clean `EndPolicy` exit.
pub struct PolicyTokenGuard {
    slots: Arc<PolicyCancelSlots>,
    slot: u8,
    token: u64,
    disarmed: bool,
}

impl PolicyTokenGuard {
    pub fn token(&self) -> u64 {
        self.token
    }

    pub fn slot(&self) -> u8 {
        self.slot
    }

    /// Disarms cancellation on drop (used ONLY when `EndPolicy` already succeeded or RT already
    /// ended the run; `Drop` still marks `retired_by_caller = true` for safe slot reclamation).
    pub fn disarm(&mut self) {
        self.disarmed = true;
    }

    /// Records an admitted broker request ID strictly before ring publication.
    pub fn record_admitted_req(&self, req_id: u64) {
        self.slots.record_admitted_req(self.slot, self.token, req_id);
    }
}

impl Drop for PolicyTokenGuard {
    fn drop(&mut self) {
        if !self.disarmed {
            if let Some(s) = self.slots.slots.get(usize::from(self.slot)) {
                if s.token.load(Ordering::Acquire) == self.token {
                    s.cancelled.store(true, Ordering::Release);
                }
            }
        }
        let mut st = self.slots.alloc.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(entry) = st.entries.get_mut(usize::from(self.slot)) {
            if entry.token == self.token {
                entry.retired_by_caller = true;
            }
        }
    }
}

/// Inline owner-side reservation held while a policy pack loads and warms up off-thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PolicyLoadReservation {
    pub token: u64,
    pub cancel_slot: u8,
    pub conn_id: u64,
    pub lease_generation: u64,
    pub epoch_id: u64,
    pub stop_generation: u64,
    pub expires_at: Instant,
}

/// `Copy` spec for `PolicyCommand::ReserveLoad`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReservePolicyLoadSpec {
    pub load_token: u64,
    pub cancel_slot: u8,
    pub admission_request_id: u64,
    pub prepared_boot: BootId,
    pub connection: ConnectionHandle,
    pub mode: PolicyMode,
    pub sim_only: bool,
    pub i_am_at_the_arm: bool,
    pub operator: OperatorGrant,
    pub reservation_expires_at: Instant,
}

/// `Copy` spec for `PolicyCommand::BeginPolicy`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeginPolicySpec {
    pub load_token: u64,
    pub cancel_slot: u8,
    pub admission_request_id: u64,
    pub prepared_boot: BootId,
    pub connection: ConnectionHandle,
    pub operator: OperatorGrant,
    pub calibration_revision: u64,
    pub policy_id: HolderLabel,
    pub artifact_digest: HolderLabel,
    pub backend: HolderLabel,
    pub mode: PolicyMode,
    pub sim_only: bool,
    pub i_am_at_the_arm: bool,
    pub max_first_step_rad: f32,
    pub max_step_rad: Option<f32>,
    pub max_input_age_ms: u64,
    pub max_vision_age_ms: Option<u64>,
    pub vision_stale_hold: bool,
    pub period_us: u64,
    pub has_verified_release_gate: bool,
    pub gate_policy_sha256: HolderLabel,
    pub gate_calibration_digest: HolderLabel,
    pub gate_max_speed_scale: f64,
    pub gate_is_production_release: bool,
    pub production_qualified_verified: bool,
}

/// `Copy` spec for `PolicyCommand::EndPolicy`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndPolicySpec {
    /// Client WS `/api/v1/action/policy/stop` (lease-scoped, advances shared stop generation).
    Stop {
        connection: ConnectionHandle,
        lease_generation: u64,
    },
    /// Runner-originated live hold (requires matching `action_id` and `load_token`).
    Hold {
        action_id: u64,
        load_token: u64,
        lease_generation: u64,
    },
    /// Runner-originated normal completion (e.g. `MAX_SECONDS`).
    Finish {
        action_id: u64,
        load_token: u64,
        lease_generation: u64,
        reason: HolderLabel,
        final_steps: u64,
        final_stats: PolicyStats,
    },
    /// Runner-originated failure (e.g. `VISION_STALE`, `INFERENCE_ERROR`, `DEPLOYMENT_CHANGED`).
    Fail {
        action_id: u64,
        load_token: u64,
        lease_generation: u64,
        reason: HolderLabel,
        message: HolderLabel,
        final_steps: u64,
        final_stats: PolicyStats,
    },
}

/// `Copy` broker command for all executor policy lifecycle mutations.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum PolicyCommand {
    ReserveLoad(ReservePolicyLoadSpec),
    BeginPolicy(BeginPolicySpec),
    EndPolicy(EndPolicySpec),
}

impl PolicyCommand {
    /// Only `BeginPolicy` starts a top-level action (`Ack` + `Terminal`).
    pub fn starts_action(&self) -> bool {
        matches!(self, Self::BeginPolicy(_))
    }
}

fn check_policy_start_preconditions_rt(
    inner: &SupervisorInner,
    mode: PolicyMode,
    sim_only: bool,
    i_am_at_the_arm: bool,
    epoch_id: u64,
) -> Result<(), &'static str> {
    if inner.config.active_tool.commissioning_required {
        return Err("COMMISSIONING_REQUIRED");
    }
    if inner.estop_pending() {
        return Err("ESTOP_ACTIVE");
    }
    if epoch_id != inner.safety.epoch_id() {
        return Err("EPOCH_MISMATCH");
    }
    if inner.handover_settling_since.is_some() {
        return Err("HANDOVER_SETTLING");
    }
    if sim_only && !inner.policy_live_allowed() {
        return Err("POLICY_SIM_ONLY");
    }
    match inner.safety.state() {
        SafetyState::EStop => return Err("ESTOP_ACTIVE"),
        SafetyState::Fault(_) => return Err("FAULT_ACTIVE"),
        s if mode == PolicyMode::Live && !s.is_operational() => return Err("NOT_ARMED"),
        _ => {}
    }
    if mode == PolicyMode::Live {
        if inner.mit_motion_blocked() {
            return Err("MOTION_BUSY");
        }
        if !inner.policy_live_allowed() && !i_am_at_the_arm {
            return Err("OPERATOR_CONFIRMATION_REQUIRED");
        }
        if inner.gravity_comp.is_some() {
            return Err("MOTION_BUSY");
        }
        if inner.collision.hold.is_some() {
            return Err("COLLISION_HOLD");
        }
        if inner.pending_rearm || inner.pending_normal_unload || !inner.is_torqued {
            return Err("NOT_ARMED");
        }
        if inner.servo_blocked_hold {
            return Err("BLOCKED_HOLD");
        }
        if inner.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) {
            return Err("MOTION_BUSY");
        }
    }
    if inner.calibration_busy
        || inner.zero_check_persisting.is_some()
        || inner.gravity_comp.is_some()
        || inner.pending_normal_unload
        || inner.active_trajectory.is_some()
        || inner.macro_run.is_some()
        || inner.policy_loading
        || inner.policy_run.is_some()
        || inner.servo_target.is_some()
    {
        return Err("MOTION_BUSY");
    }
    Ok(())
}

fn check_policy_stamp_and_authority(
    inner: &SupervisorInner,
    d: &Descriptor,
    prepared_boot: BootId,
    ctx: &CommitCtx,
    stop_direction: bool,
) -> Result<(), &'static str> {
    if inner.detached {
        return Err("OWNER_TRANSFERRED");
    }
    if prepared_boot.is_zero() || prepared_boot != ctx.boot || d.stamp.boot != ctx.boot {
        return Err("WRONG_TARGET");
    }
    let current = crate::rt_core::ops::CoreOps::authority(inner, ctx);
    let current_no_cfg = crate::rt_core::broker::Authority {
        config_revision: d.stamp.authority.config_revision,
        ..current
    };
    let check = if stop_direction {
        d.stamp.check_stop_direction(ctx.arm, ctx.boot, ctx.now, &current_no_cfg)
    } else {
        d.stamp.check(ctx.arm, ctx.boot, ctx.now, &current_no_cfg)
    };
    if let Some(code) = check {
        return Err(code);
    }
    if d.stamp.client.0 != 0 && inner.is_connection_closed(d.stamp.client.0) {
        return Err("LEASE_LOST");
    }
    Ok(())
}

/// Dispatches one `CommandKind::Policy(cmd)` on `armd-control` (0 heap allocs, 0 frees, 0 locks).
pub fn commit_policy(
    inner: &mut SupervisorInner,
    d: &Descriptor,
    cmd: PolicyCommand,
    ctx: &CommitCtx,
) -> Result<Committed, &'static str> {
    inner.prune_stale_policy_load(ctx.now);
    match cmd {
        PolicyCommand::ReserveLoad(spec) => install_reserve_load(inner, d, &spec, ctx),
        PolicyCommand::BeginPolicy(spec) => install_begin_policy(inner, d, &spec, ctx),
        PolicyCommand::EndPolicy(spec) => install_end_policy(inner, d, &spec, ctx),
    }
}

fn policy_admission_stamp(d: &Descriptor, admission_request_id: u64) -> AdmissionStamp {
    if admission_request_id != 0 {
        AdmissionStamp {
            stop_gen: d.stamp.authority.stop_generation,
            estop_epoch: d.stamp.authority.epoch,
            lease_gen: d.stamp.authority.lease_generation,
            connection_id: d.stamp.client.0,
            request_id: admission_request_id,
        }
    } else {
        AdmissionStamp::from_broker_conn(
            d.stamp.authority.stop_generation,
            d.stamp.authority.epoch,
            d.stamp.authority.lease_generation,
            d.stamp.client.0,
            d.stamp.request.0,
        )
    }
}

fn install_reserve_load(
    inner: &mut SupervisorInner,
    d: &Descriptor,
    spec: &ReservePolicyLoadSpec,
    ctx: &CommitCtx,
) -> Result<Committed, &'static str> {
    check_policy_stamp_and_authority(inner, d, spec.prepared_boot, ctx, false)?;
    if inner.policy_cancel_slots.is_cancelled(spec.cancel_slot, spec.load_token) {
        return Err("STOPPED");
    }
    if !inner.policy_live_allowed() && !spec.operator.is_valid_at(ctx.now, inner.operator_auth_enforced) {
        return Err("OPERATOR_REQUIRED");
    }
    if !inner.validate_lease_at(spec.connection.as_str(), d.stamp.authority.lease_generation, ctx.now) {
        return Err("LEASE_LOST");
    }
    if ctx.now > spec.reservation_expires_at {
        return Err("DEADLINE_EXPIRED");
    }
    inner.sync_stop_generation(ctx.stop_generation);
    let stamp = policy_admission_stamp(d, spec.admission_request_id);
    inner
        .preflight_admission(stamp)
        .map_err(|e| crate::supervisor::targets::safety_code(&e))?;
    inner
        .check_terminal_fifo_for_new_action()
        .map_err(|e| crate::supervisor::targets::safety_code(&e))?;
    if !inner.can_admit_policy_run() {
        return Err("MOTION_BUSY");
    }
    check_policy_start_preconditions_rt(
        inner,
        spec.mode,
        spec.sim_only,
        spec.i_am_at_the_arm,
        d.stamp.authority.epoch,
    )?;

    inner.policy_load_reservation = Some(PolicyLoadReservation {
        token: spec.load_token,
        cancel_slot: spec.cancel_slot,
        conn_id: d.stamp.client.0,
        lease_generation: d.stamp.authority.lease_generation,
        epoch_id: d.stamp.authority.epoch,
        stop_generation: d.stamp.authority.stop_generation,
        expires_at: spec.reservation_expires_at,
    });
    inner.policy_loading = true;
    inner.policy_cancel_slots.set_active_load_token(spec.load_token);
    inner.reconcile_target_stream_authority(ctx.now);
    Ok(Committed::for_action(d, Some(ActionId(spec.load_token))))
}

fn install_begin_policy(
    inner: &mut SupervisorInner,
    d: &Descriptor,
    spec: &BeginPolicySpec,
    ctx: &CommitCtx,
) -> Result<Committed, &'static str> {
    // Verify and consume the matching load reservation BEFORE checking preconditions
    // (if our token still owns `policy_loading`, clear it cleanly on both success and failure).
    let owns_res = inner.policy_load_reservation.is_some_and(|r| {
        r.token == spec.load_token
            && r.conn_id == d.stamp.client.0
            && r.lease_generation == d.stamp.authority.lease_generation
    });
    if owns_res {
        inner.policy_load_reservation = None;
        inner.policy_loading = false;
        inner.policy_cancel_slots.set_active_load_token(0);
    }
    let res = install_begin_policy_body(inner, d, spec, ctx, owns_res);
    if res.is_err() && owns_res {
        inner.reconcile_target_stream_authority(ctx.now);
    }
    res
}

fn install_begin_policy_body(
    inner: &mut SupervisorInner,
    d: &Descriptor,
    spec: &BeginPolicySpec,
    ctx: &CommitCtx,
    owns_res: bool,
) -> Result<Committed, &'static str> {
    check_policy_stamp_and_authority(inner, d, spec.prepared_boot, ctx, false)?;
    if inner.policy_cancel_slots.is_cancelled(spec.cancel_slot, spec.load_token) {
        return Err("STOPPED");
    }
    if !owns_res {
        return Err("STOPPED");
    }
    if !inner.policy_live_allowed() && !spec.operator.is_valid_at(ctx.now, inner.operator_auth_enforced) {
        return Err("OPERATOR_REQUIRED");
    }
    if !inner.validate_lease_at(spec.connection.as_str(), d.stamp.authority.lease_generation, ctx.now) {
        return Err("LEASE_LOST");
    }
    check_policy_start_preconditions_rt(
        inner,
        spec.mode,
        spec.sim_only,
        spec.i_am_at_the_arm,
        d.stamp.authority.epoch,
    )?;
    if spec.mode == PolicyMode::Live && inner.config.gripper_index().is_some() {
        inner.check_pos_force_gripper_fresh_static()?;
    }
    if spec.mode == PolicyMode::Live && !inner.policy_live_allowed() {
        if !spec.i_am_at_the_arm {
            return Err("OPERATOR_CONFIRMATION_REQUIRED");
        }
        if !spec.has_verified_release_gate
            || spec.artifact_digest.is_empty()
            || !spec
                .gate_policy_sha256
                .as_str()
                .eq_ignore_ascii_case(spec.artifact_digest.as_str())
        {
            return Err("RELEASE_GATE_REQUIRED");
        }
        if spec.calibration_revision != inner.calibration_revision
            || !spec
                .gate_calibration_digest
                .as_str()
                .eq_ignore_ascii_case(inner.active_calibration_digest_str())
        {
            return Err("RELEASE_GATE_INVALID");
        }
        if !inner.feedback_numeric_valid
            || !inner.rt_driver_feedback_confirmed()
            || !inner.has_policy_input_age_range()
        {
            return Err("FEEDBACK_UNCONFIRMED");
        }
        if spec.gate_is_production_release && !spec.production_qualified_verified {
            return Err("QUALIFICATION_REQUIRED");
        }
    }
    inner
        .check_terminal_fifo_for_new_action()
        .map_err(|e| crate::supervisor::targets::safety_code(&e))?;
    if !inner.can_admit_policy_run() {
        return Err("MOTION_BUSY");
    }

    inner.sync_stop_generation(ctx.stop_generation);
    let stamp = policy_admission_stamp(d, spec.admission_request_id);
    let prev_j7_stamp = inner.j7_admitted_stamp;
    let admitted = inner
        .check_admission(stamp)
        .map_err(|e| crate::supervisor::targets::safety_code(&e))?;
    let (this, _proof) = admitted
        .consume()
        .map_err(|e| crate::supervisor::targets::safety_code(&e))?;
    // Starting a policy run does not immediately emit a J7 target; preserve/clear cleanly.
    this.j7_admitted_stamp = prev_j7_stamp;

    let Some(mut run) = this.policy_run_spare.take() else {
        return Err("MOTION_BUSY");
    };
    let action_id = this.begin_action_rt("policy");
    let max_first_step_rad = if spec.max_first_step_rad.is_finite() && spec.max_first_step_rad > 0.0 {
        spec.max_first_step_rad.min(DEFAULT_MAX_FIRST_STEP_RAD)
    } else {
        DEFAULT_MAX_FIRST_STEP_RAD
    };

    run.action_id = action_id;
    run.epoch_id = d.stamp.authority.epoch;
    set_str(&mut run.connection_id, spec.connection.as_str());
    run.lease_generation = d.stamp.authority.lease_generation;
    set_str(&mut run.policy_id, spec.policy_id.as_str());
    set_str(&mut run.artifact_digest, spec.artifact_digest.as_str());
    set_str(&mut run.backend, spec.backend.as_str());
    run.mode = spec.mode;
    run.steps = 0;
    run.last_input_age_ms = None;
    run.last_vision_age_ms = None;
    run.vision_frame_id = None;
    run.last_shadow_target = None;
    run.last_live_target = None;
    run.max_first_step_rad = max_first_step_rad;
    run.max_step_rad = spec.max_step_rad;
    run.i_am_at_the_arm = spec.i_am_at_the_arm;
    run.verified_release_gate = None;
    run.rt_gate_binding = spec.has_verified_release_gate.then_some(RtGateBinding {
        policy_sha256: spec.gate_policy_sha256,
        calibration_digest: spec.gate_calibration_digest,
        max_speed_scale: spec.gate_max_speed_scale,
        production_release: spec.gate_is_production_release,
    });
    run.live_target_auth_gen = None;
    run.rt_owned = true;
    run.load_token = spec.load_token;
    run.cancel_slot = spec.cancel_slot;
    run.stop_reason = None;
    run.max_input_age_ms = spec.max_input_age_ms;
    run.step_stats = crate::policy_runtime::PolicyStepStats::default();
    run.step_stats.set_period_us(spec.period_us);
    run.authorization.revoke();
    run.max_vision_age_ms = spec.max_vision_age_ms;
    run.vision_stale_hold = spec.vision_stale_hold;

    if this.last_policy_obs.is_none() {
        this.last_policy_obs = this.last_policy_obs_spare.take();
    }
    let obs_dst = this
        .last_policy_obs
        .get_or_insert_with(crate::model::PolicyObservation::preallocated_rt);
    run.to_observation_into_with_vec_spare(true, obs_dst, &mut this.last_policy_obs_vec_spare);

    this.policy_run = Some(run);
    this.policy_cancel_slots.set_active_run_token(spec.load_token);
    this.reconcile_target_stream_authority(ctx.now);
    Ok(Committed::for_action(d, Some(ActionId(action_id))))
}

fn install_end_policy(
    inner: &mut SupervisorInner,
    d: &Descriptor,
    spec: &EndPolicySpec,
    ctx: &CommitCtx,
) -> Result<Committed, &'static str> {
    // Stop-direction: exempt from prior `stop_generation`, still checks arm/boot/deadline/epoch/lease/closed.
    check_policy_stamp_and_authority(inner, d, d.stamp.boot, ctx, true)?;
    match *spec {
        EndPolicySpec::Stop {
            connection,
            lease_generation,
        } => {
            if !inner.validate_lease_at(connection.as_str(), lease_generation, ctx.now) {
                return Err("LEASE_LOST");
            }
            // Race-free `StopRequests` advancement: if a concurrent general Stop incremented
            // `stops` before our `request()` (i.e. `next - 1 != stops.applied()`), apply full
            // `authorized_stop()` before advancing `applied` so no unrelated active move survives.
            if let Some(stops) = inner.stop_requests.clone() {
                let next = stops.request();
                let prev = next.saturating_sub(1);
                if prev != stops.applied() {
                    inner.authorized_stop();
                }
                inner.sync_stop_generation(next);
                stops.mark_applied(next);
            } else {
                inner.advance_stop_generation();
            }
            let had_loading = inner.policy_load_reservation.take().is_some() || inner.policy_loading;
            if had_loading {
                inner.policy_loading = false;
                inner.policy_cancel_slots.set_active_load_token(0);
            }
            let stopped_run_id = inner.policy_run.as_ref().map(|r| r.action_id);
            if stopped_run_id.is_some() {
                inner.terminate_policy_run_static(
                    "STOPPED",
                    "cancelled",
                    Some("STOPPED"),
                    Some("Policy stopped: STOPPED"),
                );
            } else if had_loading {
                inner.reconcile_target_stream_authority(ctx.now);
            }
            // Return `Some(ActionId(id))` if an active run or load was stopped (0 encodes load-only),
            // or `None` if neither was active so the WS ACK sets `stopped: action.is_some() || had_pending`.
            let ack_action = stopped_run_id
                .map(ActionId)
                .or_else(|| had_loading.then_some(ActionId(0)));
            Ok(Committed::for_action(d, ack_action))
        }
        EndPolicySpec::Hold {
            action_id,
            load_token,
            lease_generation,
        } => {
            let matches = inner.policy_run.as_ref().is_some_and(|r| {
                r.action_id == action_id
                    && r.load_token == load_token
                    && r.lease_generation == lease_generation
            });
            if !matches {
                return Err("POLICY_NOT_ACTIVE");
            }
            inner.hold_policy_run(action_id);
            Ok(Committed::for_action(d, Some(ActionId(action_id))))
        }
        EndPolicySpec::Finish {
            action_id,
            load_token,
            lease_generation,
            reason,
            final_steps,
            final_stats,
        } => {
            let matches = inner.policy_run.as_ref().is_some_and(|r| {
                r.action_id == action_id
                    && r.load_token == load_token
                    && r.lease_generation == lease_generation
            });
            if !matches {
                return Err("POLICY_NOT_ACTIVE");
            }
            if let Some(run) = inner.policy_run.as_mut() {
                if run.mode == PolicyMode::Shadow {
                    run.steps = run.steps.max(final_steps);
                }
            }
            // Fault / E-stop precedence identically to `finish_policy_run`:
            if let SafetyState::Fault(ref r) = inner.safety.state() {
                let code = if matches!(r, FaultReason::WatchdogTimeout) {
                    "WATCHDOG_TIMEOUT"
                } else {
                    "FAULT"
                };
                let mut buf = inner.take_action_msg_buf(action_id, 256);
                push_str_bounded(&mut buf, "Fault active during policy completion");
                inner.terminate_policy_run_with_stats(code, "failed", Some(code), Some(buf), Some(final_stats));
                return Ok(Committed::for_action(d, Some(ActionId(action_id))));
            }
            if inner.estop_pending() || matches!(inner.safety.state(), SafetyState::EStop) {
                let mut buf = inner.take_action_msg_buf(action_id, 256);
                push_str_bounded(&mut buf, "E-stop latched");
                inner.terminate_policy_run_with_stats(
                    "ESTOP",
                    "failed",
                    Some("ESTOP_ACTIVE"),
                    Some(buf),
                    Some(final_stats),
                );
                return Ok(Committed::for_action(d, Some(ActionId(action_id))));
            }
            let stop_reason = if reason.is_empty() { "MAX_SECONDS" } else { reason.as_str() };
            inner.terminate_policy_run_with_stats(stop_reason, "done", None, None, Some(final_stats));
            Ok(Committed::for_action(d, Some(ActionId(action_id))))
        }
        EndPolicySpec::Fail {
            action_id,
            load_token,
            lease_generation,
            reason,
            message,
            final_steps,
            final_stats,
        } => {
            let matches = inner.policy_run.as_ref().is_some_and(|r| {
                r.action_id == action_id
                    && r.load_token == load_token
                    && r.lease_generation == lease_generation
            });
            if !matches {
                return Err("POLICY_NOT_ACTIVE");
            }
            if let Some(run) = inner.policy_run.as_mut() {
                if run.mode == PolicyMode::Shadow {
                    run.steps = run.steps.max(final_steps);
                }
            }
            if inner.estop_pending() || matches!(inner.safety.state(), SafetyState::EStop) {
                let mut buf = inner.take_action_msg_buf(action_id, 256);
                push_str_bounded(&mut buf, "E-stop latched");
                inner.terminate_policy_run_with_stats(
                    "ESTOP",
                    "failed",
                    Some("ESTOP_ACTIVE"),
                    Some(buf),
                    Some(final_stats),
                );
                return Ok(Committed::for_action(d, Some(ActionId(action_id))));
            }
            if let SafetyState::Fault(ref r) = inner.safety.state() {
                let code = if matches!(r, FaultReason::WatchdogTimeout) {
                    "WATCHDOG_TIMEOUT"
                } else {
                    "FAULT"
                };
                let mut buf = inner.take_action_msg_buf(action_id, 256);
                push_str_bounded(&mut buf, "Fault active during policy failure");
                inner.terminate_policy_run_with_stats(code, "failed", Some(code), Some(buf), Some(final_stats));
                return Ok(Committed::for_action(d, Some(ActionId(action_id))));
            }
            let stop_reason = if reason.is_empty() { "POLICY_FAILED" } else { reason.as_str() };
            let msg = if message.is_empty() {
                None
            } else {
                let mut buf = inner.take_action_msg_buf(action_id, 256);
                push_str_bounded(&mut buf, message.as_str());
                Some(buf)
            };
            inner.terminate_policy_run_with_stats(
                stop_reason,
                "failed",
                Some(stop_reason),
                msg,
                Some(final_stats),
            );
            Ok(Committed::for_action(d, Some(ActionId(action_id))))
        }
    }
}

/// Maps static RT policy error codes to legacy-compatible HTTP status and message strings off-RT.
pub fn policy_code_to_api_error(code: &str, epoch_id: u64) -> crate::api::rest::ApiError {
    use crate::api::rest::ApiError;
    use axum::http::StatusCode;
    match code {
        "OPERATOR_REQUIRED" => ApiError::operator_required("policy/start (real arm)"),
        "OPERATOR_CONFIRMATION_REQUIRED" => ApiError(
            StatusCode::FORBIDDEN,
            "Real-arm live policy execution requires explicit i_am_at_the_arm confirmation".into(),
            code.into(),
        ),
        "RELEASE_GATE_REQUIRED" => ApiError(
            StatusCode::FORBIDDEN,
            "Real-arm live policy execution requires a verified rebot-release-gate/v1 bound to the policy artifact digest".into(),
            code.into(),
        ),
        "RELEASE_GATE_INVALID" => ApiError(
            StatusCode::FORBIDDEN,
            "Release gate verification failed at commit (calibration or deployment changed)".into(),
            code.into(),
        ),
        "QUALIFICATION_REQUIRED" => ApiError(
            StatusCode::FORBIDDEN,
            "production_release requires device qualification records".into(),
            code.into(),
        ),
        "POLICY_SIM_ONLY" => ApiError(
            StatusCode::FORBIDDEN,
            "live_sim mode is not permitted on real or unknown hardware drivers".into(),
            code.into(),
        ),
        "LEASE_LOST" => ApiError(
            StatusCode::CONFLICT,
            "Control lease lost or expired".into(),
            code.into(),
        ),
        "EPOCH_MISMATCH" => ApiError::from(crate::supervisor::ArmError::Safety(
            SafetyError::EpochMismatch {
                expected: epoch_id,
                received: epoch_id.wrapping_sub(1),
            },
        )),
        "ESTOP_ACTIVE" => ApiError::from(crate::supervisor::ArmError::Safety(SafetyError::EStopActive)),
        "MOTION_BUSY" => ApiError::from(crate::supervisor::ArmError::Safety(SafetyError::MotionBusy)),
        "STOPPED" => ApiError::from(crate::supervisor::ArmError::Safety(SafetyError::Stopped)),
        "TERMINALS_BUSY" => ApiError::from(crate::supervisor::ArmError::Safety(SafetyError::TerminalsBusy)),
        "HANDOVER_SETTLING" => ApiError::from(crate::supervisor::ArmError::Safety(SafetyError::HandoverSettling)),
        "J7_FEEDBACK_STALE" => ApiError(
            StatusCode::CONFLICT,
            "Gripper (J7) position-force feedback is stale (> 100 ms); new gripper targets are refused until fresh feedback returns".into(),
            code.into(),
        ),
        "FEEDBACK_UNCONFIRMED" => ApiError(
            StatusCode::CONFLICT,
            "Real-arm live policy execution requires confirmed, valid hardware feedback".into(),
            code.into(),
        ),
        "BLOCKED_HOLD" => ApiError(
            StatusCode::CONFLICT,
            "Cannot start live policy while servo blocked hold is active".into(),
            code.into(),
        ),
        "NOT_ARMED" => ApiError(
            StatusCode::PRECONDITION_FAILED,
            "Arm must be armed and torqued before starting live policy".into(),
            code.into(),
        ),
        "COMMISSIONING_REQUIRED" => ApiError(
            StatusCode::CONFLICT,
            "Tool commissioning confirmation required before motion".into(),
            code.into(),
        ),
        "COLLISION_HOLD" => ApiError(
            StatusCode::CONFLICT,
            "Collision hold is active".into(),
            code.into(),
        ),
        other => ApiError(
            StatusCode::CONFLICT,
            format!("Policy request refused: {other}"),
            other.into(),
        ),
    }
}
