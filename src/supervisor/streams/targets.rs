//! Latest-target path of servo, teleop and policy streams (armd 1.6 RT core, track D).

use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::rt_core::broker::{Authority, BootId};
use crate::rt_core::ids::ArmId;
use crate::rt_core::mailbox::{
    Refusal, RefusalDetail, StreamId, Submitted, TargetAuthority, TargetFrame, TargetOutcome, TargetWriter, MAX_DOF,
};
use crate::rt_core::ops::{CommitCtx, CoreOps};
use crate::safety::SafetyError;
#[cfg(feature = "policy-runtime")]
use crate::safety::SafetyState;
use crate::supervisor::{ArmError, SupervisorInner};

/// Servo stream ids carry this bit; policy stream ids are action ids (without it).
pub const SERVO_STREAM_BIT: u64 = 1 << 63;
/// A servo frame not applied within this time is refused (`DEADLINE_EXPIRED`): the servo
/// motion-frame timeout.
pub const SERVO_TARGET_TTL: Duration = Duration::from_millis(200);

/// The stream of WS servo frames under lease `lease_generation`.
pub fn servo_stream(lease_generation: u64) -> StreamId { StreamId(SERVO_STREAM_BIT | lease_generation) }

/// The stream of policy run `action_id`.
pub fn policy_stream(action_id: u64) -> StreamId { StreamId(action_id & !SERVO_STREAM_BIT) }

/// Codes of the policy step that a refusal carries as they are (others: `REJECTED`).
const POLICY_CODES: &[&str] = &[
    "POLICY_NOT_ACTIVE", "LEASE_LOST", "BLOCKED_HOLD", "DEPLOYMENT_CHANGED", "OPERATOR_CONFIRMATION_REQUIRED",
    "RELEASE_GATE_REQUIRED", "FEEDBACK_UNCONFIRMED", "STALE_INPUT", "JUMP_LIMIT", "STEP_LIMIT", "QUALIFICATION_REQUIRED",
    "J7_FEEDBACK_STALE",
];

/// The protocol code of a servo error, as `ApiError::from` gives it (no allocation).
pub fn safety_code(e: &SafetyError) -> &'static str {
    match e {
        SafetyError::HandoverSettling => "HANDOVER_SETTLING",
        SafetyError::ArmNotAtRestingPose { .. } => "ARM_NOT_AT_RESTING_POSE",
        SafetyError::MotionForbiddenInFaultState(_) => "MOTION_FORBIDDEN_IN_FAULT_STATE",
        SafetyError::TorqueOffWouldDrop { .. } => "TORQUE_OFF_WOULD_DROP",
        SafetyError::ClearFaultRejected(_) => "CLEAR_FAULT_REJECTED",
        SafetyError::EStopActive => "ESTOP_ACTIVE",
        SafetyError::CollisionHold(_) => "COLLISION_HOLD",
        SafetyError::CommissioningRequired(_) => "COMMISSIONING_REQUIRED",
        SafetyError::MotionBusy => "MOTION_BUSY",
        SafetyError::WorkspaceViolation { .. } => "WORKSPACE_VIOLATION",
        SafetyError::StaticLoadLimit => "STATIC_LOAD_LIMIT",
        SafetyError::EpochMismatch { .. } => "EPOCH_MISMATCH",
        SafetyError::SoftLimitExceeded { .. } => "SOFT_LIMIT_EXCEEDED",
        SafetyError::FaultActive(_) => "FAULT_ACTIVE",
        SafetyError::NotArmed(_) => "NOT_ARMED",
        SafetyError::OwnerTransferred => "OWNER_TRANSFERRED",
        SafetyError::Stopped => "STOPPED",
        SafetyError::LeaseLost => "LEASE_LOST",
        SafetyError::TerminalsBusy => "TERMINALS_BUSY",
        SafetyError::StampAlreadyConsumed => "STAMP_REUSED",
        _ => "SAFETY_ERROR",
    }
}

/// The protocol code of a policy step error (no allocation).
pub fn policy_code(e: &ArmError) -> &'static str {
    match e {
        ArmError::Safety(s) => safety_code(s),
        ArmError::RejectedStatic { code, .. } => code,
        ArmError::Rejected { code, .. } => POLICY_CODES.iter().find(|k| **k == code.as_str()).copied().unwrap_or("REJECTED"),
        ArmError::Internal(_) => "INTERNAL_ERROR",
        _ => "REJECTED",
    }
}

/// Status, `error` and `safety_state` of a refusal code, as `ApiError::from` gives them.
pub(crate) fn reject_fields(code: &str) -> (u16, &str, Option<&'static str>) {
    match code {
        "ESTOP_ACTIVE" => (409, code, Some("EStop")),
        "FAULT_ACTIVE" => (409, code, Some("Fault")),
        "ARM_NOT_AT_RESTING_POSE" => (409, "ArmNotAtRestingPose", Some("Armed")),
        "MOTION_FORBIDDEN_IN_FAULT_STATE" => (409, "MotionForbiddenInFaultState", Some("Fault")),
        "TORQUE_OFF_WOULD_DROP" => (409, code, None),
        "CLEAR_FAULT_REJECTED" => (409, "ClearFaultRejected", Some("Fault")),
        "WORKSPACE_VIOLATION" | "JUMP_LIMIT" | "STEP_LIMIT" | "STATIC_LOAD_LIMIT" => (422, code, None),
        "SOFT_LIMIT_EXCEEDED" | "SAFETY_ERROR" | "INVALID_COMMAND" => (400, code, None),
        "NOT_ARMED" => (412, code, None),
        "OPERATOR_CONFIRMATION_REQUIRED" | "RELEASE_GATE_REQUIRED" => (403, code, None),
        "INTERNAL_ERROR" => (500, code, None),
        _ => (409, code, None),
    }
}

/// What a WS reply needs from the state at reply time.
#[derive(Debug, Clone)]
pub struct ReplyCtx {
    pub epoch_id: u64,
    pub safety_state: Value,
    /// The connection's lease generation.
    pub lease_generation: u64,
}

/// `servo_reject` with the legacy fields (`status`, `code`, `error`, `message`, `safety_state`, `epoch_id`).
pub fn servo_reject(code: &str, message: &str, epoch_id: u64) -> Value {
    let (status, error, safety_state) = reject_fields(code);
    json!({"type":"servo_reject", "status":status, "code":code, "error":error, "message":message,
        "safety_state":safety_state, "epoch_id":epoch_id})
}

/// The WS reply to a servo target the RT refused (None: no reply).
///
/// As the legacy loop answers the same conditions: a stop-preempted frame with `servo_reject`
/// (`code: "STOPPED"`, `reason: "stopped"` matching `api/ws/servo.rs`), an unauthorized frame
/// (lease lost, stream not authorized, no lease) with `lease_lost` (with the loss reason when
/// known), a stale epoch with `epoch_sync`, a calibration with `servo_reject`
/// `reason: calibration_busy`, other refusals with `servo_reject` (and `current_revision`
/// when known). Messages match the legacy path.
pub fn servo_outcome_reply(refusal: &Refusal, ctx: &ReplyCtx) -> Option<Value> {
    Some(match refusal.code {
        "STOPPED" => {
            let mut v = servo_reject("STOPPED", "Stopped before motion began", ctx.epoch_id);
            v["reason"] = json!("stopped");
            v
        }
        "LEASE_LOST" | "NO_LEASE" | "STREAM_NOT_AUTHORIZED" => {
            let mut v = json!({"type":"lease_lost", "generation":ctx.lease_generation});
            if let (RefusalDetail::LeaseLoss { reason }, true) = (refusal.detail, ctx.lease_generation != 0) { v["reason"] = json!(reason); }
            v
        }
        "EPOCH_MISMATCH" => json!({"type":"epoch_sync", "epoch_id":ctx.epoch_id, "safety_state":ctx.safety_state}),
        "CALIBRATION_BUSY" => json!({"type":"servo_reject", "reason":"calibration_busy"}),
        "STATIC_LOAD_LIMIT" => servo_reject("STATIC_LOAD_LIMIT", crate::power::STATIC_LOAD_LIMIT_MSG, ctx.epoch_id),
        code => {
            let mut v = servo_reject(code, "servo target refused when the control loop applied it", ctx.epoch_id);
            if let RefusalDetail::CurrentRevision(rev) = refusal.detail {
                v["current_revision"] = json!(rev);
            } else {
                crate::chain::attach_workspace_refusal_fields(&mut v, refusal.detail, "servo_target");
            }
            v
        }
    })
}

/// Per-connection servo stream state (the WS loop's `last_sequence`).
#[derive(Debug, Default)]
pub struct ServoStream {
    last_sequence: Option<u64>,
    lease_generation: u64,
}

/// One WS servo message (either form), already authorized and epoch-checked by the loop.
#[derive(Debug, Clone, Copy)]
pub struct ServoMessage<'a> {
    pub sequence_id: Option<u64>,
    pub positions: &'a [f32],
    /// Lease generation, the message's epoch and the stop generation at receipt.
    pub authority: Authority,
    pub received_at: Instant,
    /// A calibration apply / restore runs (the loop's snapshot; the RT checks again).
    pub calibration_busy: bool,
}

/// One policy target, stamped when its inputs were sampled.
#[derive(Debug, Clone, Copy)]
pub struct PolicyTarget<'a> {
    pub action_id: u64,
    pub stamp: PolicyStamp,
    pub targets: &'a [f32],
    /// Capture time of the oldest joint sample the target was computed from.
    pub input_at: Instant,
    /// Capture time and frame id of the camera sample behind the target.
    pub vision_at: Option<Instant>,
    pub vision_frame_id: Option<u64>,
}

/// The broker's side of the latest-target mailbox.
pub struct TargetPort {
    writer: TargetWriter,
    arm: ArmId,
    boot: BootId,
    seq: u64,
    /// Monotonic high-water mark of closed servo lease generations: late submissions for any
    /// non-zero generation `<= closed_servo_gen` are rejected immediately off-thread.
    closed_servo_gen: u64,
    /// Non-zero servo lease generations currently open in `writer` (bounded by `MAX_STREAMS`).
    open_servo_gens: Vec<u64>,
    /// Monotonic high-water mark of closed policy action IDs: durable late-submission fence.
    closed_policy_action_id: u64,
    /// Non-zero policy action IDs currently open in `writer` (bounded by `MAX_STREAMS`).
    open_policy_actions: Vec<u64>,
}

/// What the runner stamps a policy target with, read when it samples the inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PolicyStamp {
    pub authority: Authority,
    pub authorization: u64,
    pub max_input_age: Duration,
}




impl ServoStream {
    pub fn new() -> Self { Self::default() }

    /// The connection got lease `generation`: a new lease starts a new sequence.
    pub fn lease_granted(&mut self, generation: u64) {
        if self.lease_generation != generation { self.last_sequence = None; }
        self.lease_generation = generation;
    }

    /// Synchronize sequence state across an owner handoff (legacy <-> executor) on the same lease.
    pub fn sync_from(&mut self, generation: u64, last_sequence: Option<u64>) {
        self.lease_generation = generation;
        self.last_sequence = last_sequence;
    }

    /// Advance `last_sequence` after passing sequence validation on a malformed-payload frame.
    pub fn advance_sequence(&mut self, sequence: u64) {
        self.last_sequence = Some(sequence);
    }

    pub fn last_sequence(&self) -> Option<u64> { self.last_sequence }
}

impl TargetPort {
    pub fn new(writer: TargetWriter, arm: ArmId, boot: BootId) -> Self {
        Self {
            writer,
            arm,
            boot,
            seq: 0,
            closed_servo_gen: 0,
            open_servo_gens: Vec::with_capacity(crate::rt_core::mailbox::MAX_STREAMS),
            closed_policy_action_id: 0,
            open_policy_actions: Vec::with_capacity(crate::rt_core::mailbox::MAX_STREAMS),
        }
    }

    /// Frames replaced before consumption.
    pub fn coalesced(&self) -> u64 { self.writer.coalesced() }

    /// Sequence number assigned to the most recently built frame (0 before first submission).
    pub fn last_seq(&self) -> u64 { self.seq }

    /// Highest lease generation permanently closed on this port (0 if none closed yet).
    pub fn closed_servo_generation(&self) -> u64 { self.closed_servo_gen }

    /// True when `lease_generation` (non-zero) has been closed via [`Self::close_servo`].
    pub fn is_servo_closed(&self, lease_generation: u64) -> bool {
        lease_generation != 0 && lease_generation <= self.closed_servo_gen
    }

    /// Highest policy action ID permanently closed on this port (0 if none closed yet).
    pub fn closed_policy_action_id(&self) -> u64 { self.closed_policy_action_id }

    /// True when `action_id` (non-zero) has been closed via [`Self::close_policy`].
    pub fn is_policy_closed(&self, action_id: u64) -> bool {
        action_id != 0 && action_id <= self.closed_policy_action_id
    }

    /// Underlying target writer (for low-level mailbox access / tests).
    pub fn writer(&self) -> &TargetWriter { &self.writer }

    /// Mutable access to the underlying target writer (for low-level mailbox access / tests).
    pub fn writer_mut(&mut self) -> &mut TargetWriter { &mut self.writer }

    /// Drains the RT's outcomes of consumed frames.
    pub fn outcomes(&mut self, each: impl FnMut(TargetOutcome)) -> usize { self.writer.outcomes(each) }

    /// The connection holding lease `lease_generation` ended (release, preemption, expiry, or
    /// disconnect): forget its stream (and any older open servo streams) in the mailbox writer
    /// and fence late submissions from that generation.
    pub fn close_servo(&mut self, lease_generation: u64) {
        if lease_generation == 0 {
            return;
        }
        self.closed_servo_gen = self.closed_servo_gen.max(lease_generation);
        self.writer.close_stream(servo_stream(lease_generation));
        let closed = self.closed_servo_gen;
        let writer = &mut self.writer;
        self.open_servo_gens.retain(|&g| {
            if g <= closed {
                writer.close_stream(servo_stream(g));
                false
            } else {
                true
            }
        });
    }

    /// Policy run `action_id` ended: forget its stream (and any older open policy streams)
    /// and fence late submissions for `<= action_id`.
    pub fn close_policy(&mut self, action_id: u64) {
        if action_id == 0 {
            return;
        }
        self.closed_policy_action_id = self.closed_policy_action_id.max(action_id);
        self.writer.close_stream(policy_stream(action_id));
        let closed = self.closed_policy_action_id;
        let writer = &mut self.writer;
        self.open_policy_actions.retain(|&id| {
            if id <= closed {
                writer.close_stream(policy_stream(id));
                false
            } else {
                true
            }
        });
    }

    #[allow(clippy::too_many_arguments)]
    fn frame(&mut self, stream: StreamId, authority: Authority, policy_generation: u64, q: &[f32], input_at: Instant,
        vision_at: Option<Instant>, vision_frame_id: Option<u64>, deadline: Instant, replaceable: bool) -> Option<TargetFrame> {
        if q.is_empty() || q.len() > MAX_DOF || q.iter().any(|v| !v.is_finite()) { return None; }
        let mut a = [0.0; MAX_DOF];
        a[..q.len()].copy_from_slice(q);
        self.seq += 1;
        Some(TargetFrame { arm: self.arm, boot: self.boot, stream, seq: self.seq, authority, policy_generation, input_at, vision_at,
            vision_frame_id, deadline, dof: q.len() as u8, q: a, replaceable })
    }

    /// Submits one WS servo message; returns the WS reply (None: queued, no reply).
    /// Rejects late submissions from closed lease generations before touching sequence state.
    /// Sequence rules as the legacy loop: a missing `sequence_id`, a repeat or a jump of
    /// more than 1000 (wrapping) is refused; the sequence advances once it passes them.
    pub fn submit_servo(&mut self, stream: &mut ServoStream, msg: &ServoMessage<'_>) -> Option<Value> {
        let lease_gen = msg.authority.lease_generation;
        if self.is_servo_closed(lease_gen) {
            return Some(json!({"type":"lease_lost", "generation": lease_gen}));
        }
        if msg.calibration_busy { return Some(json!({"type":"servo_reject", "reason":"calibration_busy"})); }
        let Some(sequence) = msg.sequence_id else { return Some(json!({"type":"servo_reject", "reason":"missing_sequence_id"})); };
        if stream.last_sequence.is_some_and(|last| { let delta = sequence.wrapping_sub(last); delta == 0 || delta > 1000 }) {
            return Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"}));
        }
        stream.last_sequence = Some(sequence);
        let epoch = msg.authority.epoch;
        let stream_id = servo_stream(lease_gen);
        let Some(frame) = self.frame(stream_id, msg.authority, 0, msg.positions, msg.received_at, None, None,
            msg.received_at + SERVO_TARGET_TTL, true) else {
            return Some(servo_reject("SAFETY_ERROR", "servo target is empty, has non-finite values or more joints than the arm", epoch));
        };
        match self.writer.submit(frame) {
            Submitted::Accepted | Submitted::Coalesced { .. } => {
                if lease_gen != 0 && !self.open_servo_gens.contains(&lease_gen) {
                    self.open_servo_gens.push(lease_gen);
                }
                None
            }
            Submitted::Busy => Some(servo_reject("MOTION_BUSY", "a policy target is waiting to be applied", epoch)),
            Submitted::StreamsFull => Some(servo_reject("STREAMS_FULL", "too many open target streams", epoch)),
            Submitted::OutOfOrder => Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"})),
        }
    }

    /// Submits one policy target (not replaceable; deadline: input time + twice the run's
    /// input age bound). `Busy`: the previous target was not consumed yet; the runner counts the
    /// step as skipped. Rejects late submissions for closed action IDs immediately off-thread.
    pub fn submit_policy(&mut self, t: &PolicyTarget<'_>) -> Submitted {
        self.submit_policy_with_validity(
            t,
            crate::rt_core::mailbox::PolicyValidityBounds::default(),
        )
    }

    /// Submits one policy target together with atomic [`PolicyValidityBounds`] (tracking both
    /// `Accepted` and `Coalesced` publications so policy streams that overwrite an unconsumed
    /// replaceable servo frame are never left untracked).
    pub fn submit_policy_with_validity(
        &mut self,
        t: &PolicyTarget<'_>,
        validity: crate::rt_core::mailbox::PolicyValidityBounds,
    ) -> Submitted {
        if self.is_policy_closed(t.action_id) {
            return Submitted::OutOfOrder;
        }
        // after the input age bound, so an aged input ends the run as `STALE_INPUT`
        let deadline = t.input_at + 2 * t.stamp.max_input_age;
        match self.frame(policy_stream(t.action_id), t.stamp.authority, t.stamp.authorization.max(1), t.targets, t.input_at,
            t.vision_at, t.vision_frame_id, deadline, false) {
            Some(frame) => {
                let sub = self.writer.submit_with_validity(frame, validity);
                if matches!(sub, Submitted::Accepted | Submitted::Coalesced { .. })
                    && t.action_id != 0
                    && !self.open_policy_actions.contains(&t.action_id)
                {
                    self.open_policy_actions.push(t.action_id);
                }
                sub
            }
            None => Submitted::Busy,
        }
    }
}

impl SupervisorInner {
    /// Consume the servo frames of the connection holding lease `lease_generation`
    /// (replaces the authorized stream; requires a single-use [`Admitted`] proof).
    pub fn authorize_servo_stream(admitted: crate::supervisor::Admitted<'_>, lease_generation: u64) -> Result<(), SafetyError> {
        // Authorizing a stream does not itself produce a J7 target; preserve any existing
        // J7 admission stamp rather than overwriting it during post-stop policy cleanup.
        let prev_j7_stamp = admitted.inner.j7_admitted_stamp;
        let (this, proof) = admitted.consume()?;
        this.j7_admitted_stamp = prev_j7_stamp;
        let stamp = proof.stamp();
        if lease_generation == 0 || stamp.lease_gen != lease_generation {
            return Err(SafetyError::LeaseLost);
        }
        this.core.authorize_stream(TargetAuthority { stream: Some(servo_stream(lease_generation)), ..TargetAuthority::default() });
        Ok(())
    }

    /// Single centralized owner target-stream authority selector (Live policy stream, Shadow /
    /// loading revoked stream, or valid current-lease-only servo stream). Fails closed
    /// whenever no valid unfenced open lease exists or admission refuses.
    pub(crate) fn reconcile_target_stream_authority(&mut self, now: Instant) {
        #[cfg(feature = "policy-runtime")]
        if let Some(run) = self.policy_run.as_ref() {
            if run.mode == crate::policy_runtime::PolicyMode::Live || !run.rt_owned {
                let input_known = self.has_policy_input_age_range();
                let run = self.policy_run.as_ref().expect("checked");
                self.core.authorize_stream(TargetAuthority {
                    stream: Some(policy_stream(run.action_id)),
                    policy_generation: run.authorization.current(),
                    max_input_age: input_known.then(|| Duration::from_millis(run.max_input_age_ms)),
                    max_vision_age: run.max_vision_age_ms.map(Duration::from_millis),
                });
            } else {
                // Executor Shadow run: zero mailbox authority; servo frames still get MOTION_BUSY.
                self.core.revoke_stream();
            }
            return;
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading {
            self.core.revoke_stream();
            return;
        }
        let g = if self.pending_disconnect_hold.load(std::sync::atomic::Ordering::SeqCst) > 0 {
            0
        } else {
            self.active_lease_gen_at(now)
        };
        if g == 0 {
            self.core.revoke_stream();
            return;
        }
        let stamp = crate::supervisor::AdmissionStamp::for_rt_internal(
            self.current_stop_gen(),
            self.safety.epoch_id(),
            g,
        );
        match self.check_admission(stamp) {
            Ok(admitted) => {
                if Self::authorize_servo_stream(admitted, g).is_err() {
                    self.core.revoke_stream();
                }
            }
            Err(_) => {
                self.core.revoke_stream();
            }
        }
    }

    /// Authorizes the committed lease generation's servo stream through
    /// [`Self::authorize_servo_stream`] (zero allocations, zero locks), preserving any active
    /// policy run (Live or Shadow) or active load reservation. Fails closed (revokes the
    /// stream) if admission cannot currently pass.
    pub(crate) fn authorize_servo_stream_for_committed_lease(&mut self, lease_generation: u64) {
        if lease_generation == 0 {
            self.core.revoke_stream();
            return;
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() || self.policy_loading {
            return;
        }
        let stamp = crate::supervisor::AdmissionStamp::for_rt_internal(
            self.current_stop_gen(),
            self.safety.epoch_id(),
            lease_generation,
        );
        match self.check_admission(stamp) {
            Ok(admitted) => {
                if Self::authorize_servo_stream(admitted, lease_generation).is_err() {
                    self.core.revoke_stream();
                }
            }
            Err(_) => {
                self.core.revoke_stream();
            }
        }
    }

    /// After a policy run of `action_id`: restore the valid current lease holder's servo
    /// stream (revoking when lease was lost, expired, fenced, or disconnected, or under E-stop).
    #[cfg(feature = "policy-runtime")]
    pub(in crate::supervisor) fn end_policy_stream(&mut self, action_id: u64) {
        let was_authorized = self.core.target_authority().stream == Some(policy_stream(action_id));
        if !was_authorized && self.core.target_authority().stream.is_some() {
            return;
        }
        self.reconcile_target_stream_authority(Instant::now());
    }

    /// The stamp of run `action_id`'s next target (None: no such run).
    #[cfg(feature = "policy-runtime")]
    pub fn policy_target_stamp(&self, action_id: u64, stop_generation: u64) -> Option<PolicyStamp> {
        let run = self.policy_run.as_ref().filter(|r| r.action_id == action_id)?;
        let authority = Authority { lease_generation: run.lease_generation, epoch: run.epoch_id, stop_generation,
            config_revision: self.workspace_revision, ..Authority::default() };
        Some(PolicyStamp { authority, authorization: run.authorization.current(), max_input_age: Duration::from_millis(run.max_input_age_ms) })
    }

    /// The authorized policy stream follows its run: authorization generation and age
    /// bounds (the input age is unbounded when the driver reports none, as in the legacy
    /// runner; the step then decides).
    #[cfg(feature = "policy-runtime")]
    fn sync_policy_target_authority(&mut self) {
        let Some(run) = self.policy_run.as_ref() else { return; };
        let stream = policy_stream(run.action_id);
        if self.core.target_authority().stream != Some(stream) { return; }
        let input_known = self.has_policy_input_age_range();
        let run = self.policy_run.as_ref().expect("checked");
        let a = TargetAuthority {
            stream: Some(stream),
            policy_generation: run.authorization.current(),
            max_input_age: input_known.then(|| Duration::from_millis(run.max_input_age_ms)),
            max_vision_age: run.max_vision_age_ms.map(Duration::from_millis),
        };
        if a != self.core.target_authority() { self.core.authorize_stream(a); }
    }

    fn target_refusal(&self, code: &'static str, frame: &TargetFrame) -> Refusal {
        let detail = match code {
            "LEASE_LOST" => {
                let preempted = self.last_evicted.as_ref().is_some_and(|(_, g)| *g == frame.authority.lease_generation);
                RefusalDetail::LeaseLoss { reason: if preempted { "preempted" } else { "expired" } }
            }
            "PLAN_STALE" => RefusalDetail::CurrentRevision(self.workspace_revision),
            _ => RefusalDetail::None,
        };
        Refusal { code, detail }
    }

    /// Applies one target from the latest-target mailbox after rechecking it (see the
    /// module doc) at the release `ctx` (its urgent-stop generation and clock).
    pub fn consume_target_frame(&mut self, frame: &TargetFrame, ctx: &CommitCtx) -> Result<(), Refusal> {
        self.consume_target_frame_with_validity(
            frame,
            crate::rt_core::mailbox::PolicyValidityBounds::default(),
            ctx,
        )
    }

    /// Applies one target together with its atomic [`PolicyValidityBounds`] at `ctx`.
    pub fn consume_target_frame_with_validity(
        &mut self,
        frame: &TargetFrame,
        validity: crate::rt_core::mailbox::PolicyValidityBounds,
        ctx: &CommitCtx,
    ) -> Result<(), Refusal> {
        #[cfg(feature = "policy-runtime")]
        self.sync_policy_target_authority();
        // a target is checked against the active workspace when applied; it carries no plan revision
        let current = Authority { config_revision: frame.authority.config_revision, ..CoreOps::authority(self, ctx) };
        let policy = frame.policy_generation != 0;
        let check_res = if policy {
            frame.check_policy_with_validity(
                ctx.arm,
                ctx.boot,
                ctx.now,
                &current,
                &self.core.target_authority(),
                validity,
            )
        } else {
            frame.check(ctx.arm, ctx.boot, ctx.now, &current, &self.core.target_authority())
        };
        if let Some(code) = check_res {
            if policy { self.policy_target_refused(code, frame); }
            // a servo frame while a policy run owns the targets: the legacy MOTION_BUSY
            #[cfg(feature = "policy-runtime")]
            if code == "STREAM_NOT_AUTHORIZED" && !policy && (self.policy_run.is_some() || self.policy_loading) {
                return Err(Refusal::code("MOTION_BUSY"));
            }
            return Err(self.target_refusal(code, frame));
        }
        if self.lease_enabled && frame.authority.lease_generation == 0 { return Err(Refusal::code("NO_LEASE")); }
        if self.core.motion_inhibited_by_config() { return Err(Refusal::code("CONFIG_BUSY")); }
        #[cfg(not(target_arch = "wasm32"))]
        if self.sessions.blocks_ordinary_output(None) { return Err(Refusal::code("MOTION_BUSY")); }
        if policy { return self.consume_policy_target(frame, ctx.now); }
        if self.calibration_busy { return Err(Refusal::code("CALIBRATION_BUSY")); }
        self.sync_stop_generation(ctx.stop_generation);
        let stamp = crate::supervisor::AdmissionStamp::for_rt_internal(
            frame.authority.stop_generation,
            frame.authority.epoch,
            frame.authority.lease_generation,
        );
        let admitted = self.check_admission(stamp).map_err(|e| Refusal::code(safety_code(&e)))?;
        Self::servo_slice_rt(admitted, frame.positions(), frame.authority.epoch)
    }

    #[cfg(not(feature = "policy-runtime"))]
    pub(crate) fn policy_target_refused(&mut self, _code: &str, _frame: &TargetFrame) {}

    /// A target of the active run was refused by the consumption check: hold or end the
    /// run where the legacy commit holds or ends it (zero heap allocations / frees on RT).
    #[cfg(feature = "policy-runtime")]
    pub(crate) fn policy_target_refused(&mut self, code: &str, frame: &TargetFrame) {
        let Some(run) = self.policy_run.as_ref().filter(|r| frame.stream == policy_stream(r.action_id)) else { return; };
        let (action_id, hold_on_stale) = (run.action_id, run.vision_stale_hold);
        match code {
            // revoked after the target was computed (vision hold or invalidation): brake now
            // without double-revoking the newer generation.
            "POLICY_REVOKED" => self.brake_live_policy_in_place(action_id),
            "DEADLINE_EXPIRED" => self.hold_policy_run(action_id),
            "HANDSHAKE_EXPIRED" => self.terminate_policy_run_static(
                "HANDSHAKE_EXPIRED",
                "failed",
                Some("HANDSHAKE_EXPIRED"),
                Some("Vision clock handshake expired before the target was applied"),
            ),
            "VISION_STALE" if hold_on_stale => self.hold_policy_run(action_id),
            "VISION_STALE" => self.terminate_policy_run_static(
                "VISION_STALE",
                "failed",
                Some("VISION_STALE"),
                Some("Vision sample became stale before the target was applied"),
            ),
            "INPUT_STALE" => self.terminate_policy_run_static(
                "STALE_INPUT",
                "failed",
                Some("STALE_INPUT"),
                Some("Live policy target computed from stale joint positions (aged while buffered)"),
            ),
            "LEASE_LOST" => self.terminate_policy_run_static(
                "LEASE_LOST",
                "cancelled",
                Some("LEASE_LOST"),
                Some("Control lease lost or expired"),
            ),
            "EPOCH_MISMATCH" => {
                if self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop) {
                    self.terminate_policy_run_static("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched"));
                } else if let SafetyState::Fault(ref r) = self.safety.state() {
                    let reason = if matches!(r, crate::safety::FaultReason::WatchdogTimeout) { "WATCHDOG_TIMEOUT" } else { "FAULT" };
                    self.terminate_policy_run_static(reason, "failed", Some(reason), Some("Fault active during policy run"));
                } else {
                    self.terminate_policy_run_static("STOPPED", "cancelled", Some("EPOCH_MISMATCH"), Some("Epoch changed during policy run"));
                }
            }
            _ => {}
        }
    }

    #[cfg(not(feature = "policy-runtime"))]
    fn consume_policy_target(&mut self, _frame: &TargetFrame, _now: Instant) -> Result<(), Refusal> { Err(Refusal::code("POLICY_NOT_ACTIVE")) }

    /// A policy frame that passed the consumption check: zero-alloc `policy_step_slice_rt`
    /// (holds, gates, input age, jump/step caps, single-sample J7 gate, servo checks).
    #[cfg(feature = "policy-runtime")]
    fn consume_policy_target(&mut self, frame: &TargetFrame, now: Instant) -> Result<(), Refusal> {
        let ms = |t: Instant| now.saturating_duration_since(t).as_millis().min(u128::from(u64::MAX)) as u64;
        let Some((action_id, deadline)) = self
            .policy_run
            .as_ref()
            .filter(|r| frame.stream == policy_stream(r.action_id))
            .map(|r| (r.action_id, r.deadline))
        else {
            return Err(Refusal::code("POLICY_NOT_ACTIVE"));
        };
        if deadline.is_some_and(|dl| now >= dl) {
            self.terminate_policy_run_static("MAX_SECONDS", "done", None, None);
            return Err(Refusal::code("POLICY_NOT_ACTIVE"));
        }
        let input_age_ms = self.has_policy_input_age_range().then(|| ms(frame.input_at));
        self.policy_step_slice_rt(action_id, frame, input_age_ms, frame.vision_at.map(ms), frame.vision_frame_id, now)
    }
}
