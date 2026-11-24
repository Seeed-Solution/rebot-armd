//! Action-family RT request kinds, off-thread preparation from published snapshots,
//! single-entry-point owner authority/freshness/admission rules, and macro lifecycle
//! (PR P: spec sections A6 + A7).
//!
//! All seven request kinds share ONE owner-side install function each, called by BOTH
//! the synchronous RT executor (through [`commit_action`]) and the legacy supervisor-lock
//! owner (through the `install_*` functions after off-lock `prepare_*`).

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde_json::Value;

use crate::backlash;
use crate::grasp::GraspSetup;
use crate::macro_action::{ChildSpec, ChildTicket, MacroRun};
use crate::model::{ArmConfig, HolderLabel, JointLimit, WorkspacePolicy};
use crate::motion::{
    self, ActiveTrajectory, GoalFrame, GripperCommand, GripperMotion, GripperRequest,
    JointPath, LearnMode, MotionError, PathMode, PlanInput, Reference, StepErrorText,
};
use crate::poses::PoseList;
use crate::power::PowerBudget;
use crate::recording::EpisodeStore;
use crate::replay::{self, ReplayInput, ReplayPlan, ReplayRequest};
use crate::rt_core::broker::{
    ActionId, Authority, BootId, Broker, ClientId, DeliveryDecision, Descriptor, PlanHandle,
    PlanSlotRef, Reply, RequestId, Stamp,
};
use crate::rt_core::config_iface::{ConfigAuthority, ConfigDomain};
use crate::rt_core::ids::ArmId;
use crate::rt_core::mailbox::MAX_DOF;
use crate::rt_core::ops::{
    static_code, tool_digest_hash, CommitCtx, Committed, EpisodeRef, PathKind, PlanSlot,
    Primitive, START_TOLERANCE,
};
use crate::rt_core::publish::{PublishedOwnerMeta, Snapshot};
use crate::safety::{BoundedText, SafetyError, SafetyState};
use crate::sequences::SequenceStore;
use crate::supervisor::{
    AdmissionStamp, Admitted, ArmError, ArmSupervisor, GripperBaselineCmd, HeldObject,
    JointGoalSnapshot, PreparedQuintic, SupervisorInner, OWNER_EXECUTOR,
};
use crate::trajectory::TrajectoryPlanner;

/// Default prepare-to-install deadline window when a caller does not supply an explicit deadline.
pub const DEFAULT_ACTION_DEADLINE: Duration = Duration::from_secs(30);
/// Default short submission deadline for `FinishMacro` so an orchestrator reporting timeout
/// after the parent macro's own deadline elapsed is not rejected by `DEADLINE_EXPIRED`.
pub const DEFAULT_FINISH_DEADLINE: Duration = Duration::from_secs(5);

// ---------------------------------------------------------------------------------------
// Lock-free exact connection-close fence (A1 RT extension)

/// Number of `AtomicU64` words in [`ClosedBitset`] (`1024 * 64 = 65,536` exact IDs: `1..=65535`).
const CLOSED_BITSET_WORDS: usize = 1024;
/// First connection ID handled by the conservative overflow watermark.
pub const CLOSED_BITSET_EXACT_LIMIT: u64 = (CLOSED_BITSET_WORDS as u64) * 64;

/// Lock-free, zero-allocation, zero-overflow closed-connection fence shared between
/// [`Broker`] and [`SupervisorInner`].
///
/// - Connection IDs `1..65536` have **exact** per-ID membership (no false positives on lower
///   still-open connections, O(1) `SeqCst` mark, O(1) `SeqCst` lookup).
/// - Connection ID `0` is never closed (non-WS / internal caller sentinel).
/// - Connection IDs `>= 65536` update `overflow_min_closed` monotonically downward (once non-zero)
///   and fail closed for every `id >= overflow_min_closed` without affecting `1..65536`.
pub struct ClosedBitset {
    bits: Box<[AtomicU64; CLOSED_BITSET_WORDS]>,
    overflow_min_closed: AtomicU64,
}

impl Default for ClosedBitset {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for ClosedBitset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ClosedBitset")
            .field("overflow_min_closed", &self.overflow_min_closed.load(Ordering::Relaxed))
            .finish()
    }
}

impl ClosedBitset {
    pub fn new() -> Self {
        let words: Vec<AtomicU64> = (0..CLOSED_BITSET_WORDS).map(|_| AtomicU64::new(0)).collect();
        let bits: Box<[AtomicU64; CLOSED_BITSET_WORDS]> =
            words.into_boxed_slice().try_into().expect("exact bitset length");
        Self {
            bits,
            overflow_min_closed: AtomicU64::new(0),
        }
    }

    /// Marks `connection_id` closed with `SeqCst` visibility to `armd-control`.
    pub fn mark_closed(&self, connection_id: u64) {
        if connection_id == 0 {
            return;
        }
        if connection_id < CLOSED_BITSET_EXACT_LIMIT {
            let word = (connection_id / 64) as usize;
            let mask = 1u64 << (connection_id % 64);
            self.bits[word].fetch_or(mask, Ordering::SeqCst);
        } else {
            let mut cur = self.overflow_min_closed.load(Ordering::SeqCst);
            loop {
                let next = if cur == 0 { connection_id } else { cur.min(connection_id) };
                if next == cur {
                    break;
                }
                match self.overflow_min_closed.compare_exchange_weak(
                    cur,
                    next,
                    Ordering::SeqCst,
                    Ordering::SeqCst,
                ) {
                    Ok(_) => break,
                    Err(observed) => cur = observed,
                }
            }
        }
    }

    /// Returns `true` if `connection_id` (`>= 1`) has been marked closed.
    pub fn is_closed(&self, connection_id: u64) -> bool {
        if connection_id == 0 {
            return false;
        }
        if connection_id < CLOSED_BITSET_EXACT_LIMIT {
            let word = (connection_id / 64) as usize;
            let mask = 1u64 << (connection_id % 64);
            (self.bits[word].load(Ordering::SeqCst) & mask) != 0
        } else {
            let min_closed = self.overflow_min_closed.load(Ordering::SeqCst);
            min_closed != 0 && connection_id >= min_closed
        }
    }
}

// ---------------------------------------------------------------------------------------
// Compact Copy request & staged-slot descriptors (A6 + A7)

/// Standalone prepared action family (A6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreparedActionKind {
    MoveToLinear,
    MoveRelativeLinear,
    MoveToJoint,
    MoveRelativeJoint,
    MoveToPose,
    Gripper,
    Replay,
}

impl PreparedActionKind {
    pub fn action_kind_str(self) -> &'static str {
        match self {
            Self::MoveToLinear | Self::MoveToJoint => "move_to",
            Self::MoveRelativeLinear | Self::MoveRelativeJoint => "move_relative",
            Self::MoveToPose => "move_to_pose",
            Self::Gripper => "gripper",
            Self::Replay => "replay",
        }
    }
}

/// Macro parent kind (A7: the 5 migrated workflow adapters).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacroKind {
    Grasp,
    Place,
    Sequence,
    CalibHover,
    MeasureMultiview,
}

impl MacroKind {
    pub fn action_kind_str(self) -> &'static str {
        match self {
            Self::Grasp => "grasp",
            Self::Place => "place",
            Self::Sequence => "sequence",
            Self::CalibHover => "calib_hover",
            Self::MeasureMultiview => "measure_multiview",
        }
    }
}

/// Terminal state requested by [`FinishMacroSpec`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FinishState {
    Done,
    Failed,
    Cancelled,
}

impl FinishState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Done => "done",
            Self::Failed => "failed",
            Self::Cancelled => "cancelled",
        }
    }
}

/// Held-object update requested atomically on macro finish (applied only when the parent
/// is alive and passes the quiescence gate on `FinishState::Done`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HeldUpdate {
    #[default]
    Unchanged,
    Clear,
    Set {
        label: HolderLabel,
        spill_safe: bool,
    },
}

/// Compact `Copy` mirror of [`GripperCommand`] for [`StagedAction::Gripper`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GripperCommandKind {
    Open,
    Close,
    Move,
    Seek,
}

impl From<GripperCommand> for GripperCommandKind {
    fn from(c: GripperCommand) -> Self {
        match c {
            GripperCommand::Open => Self::Open,
            GripperCommand::Close => Self::Close,
            GripperCommand::Move => Self::Move,
            GripperCommand::Seek => Self::Seek,
        }
    }
}

impl From<GripperCommandKind> for GripperCommand {
    fn from(c: GripperCommandKind) -> Self {
        match c {
            GripperCommandKind::Open => Self::Open,
            GripperCommandKind::Close => Self::Close,
            GripperCommandKind::Move => Self::Move,
            GripperCommandKind::Seek => Self::Seek,
        }
    }
}

/// Second-leg same-side approach quintic staged inline (`Copy`, zero-alloc on RT).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StagedQuinticLeg {
    pub target: [f32; MAX_DOF],
    pub duration_s: f64,
    pub power_scale: Option<f64>,
}

/// Fixed-size action payload staged into [`PlanSlot`] (`Primitive::Action(StagedAction)`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StagedAction {
    /// Prepared linear Cartesian path (samples live in `PlanSlot::path`).
    Cartesian {
        child: Option<ChildSpec>,
        kind: PreparedActionKind,
        dof: u8,
        start: [f32; MAX_DOF],
        duration_s: f64,
        power_scale: Option<f64>,
        approach_then: Option<StagedQuinticLeg>,
    },
    /// Prepared joint-space quintic (`move_to`/`move_relative` `path: "joint"`,
    /// `move_to_pose`, or macro `JointQuinticLimited` child).
    Joint {
        child: Option<ChildSpec>,
        kind: PreparedActionKind,
        dof: u8,
        start: [f32; MAX_DOF],
        target: [f32; MAX_DOF],
        duration_s: f64,
        max_velocity: [f32; MAX_DOF],
        power_scale: Option<f64>,
        settle: bool,
        approach_then: Option<StagedQuinticLeg>,
    },
    /// Prepared standalone or macro-child gripper motion.
    Gripper {
        child: Option<ChildSpec>,
        dof: u8,
        start: [f32; MAX_DOF],
        command: GripperCommandKind,
        fraction: Option<f64>,
        grasp: bool,
        speed: f64,
        timeout_s: Option<f64>,
        max_budget_s: Option<f64>,
        extend_pos_force_close_timeout: bool,
        /// `speed_scale` checked against `commissioning_required` off-thread.
        speed_scale: Option<f64>,
    },
    /// Prepared episode replay (resampled trajectory lives in `PlanSlot::path`).
    Replay {
        child: Option<ChildSpec>,
        dof: u8,
        start: [f32; MAX_DOF],
        episode: EpisodeRef,
        speed: f32,
        gripper: bool,
        sample_dt: f64,
        duration_s: f64,
        power_scale: Option<f64>,
        tool_digest: u64,
    },
}

impl StagedAction {
    pub fn child(&self) -> Option<&ChildSpec> {
        match self {
            Self::Cartesian { child, .. }
            | Self::Joint { child, .. }
            | Self::Gripper { child, .. }
            | Self::Replay { child, .. } => child.as_ref(),
        }
    }

    pub fn dof_and_start(&self) -> (usize, &[f32; MAX_DOF]) {
        match self {
            Self::Cartesian { dof, start, .. }
            | Self::Joint { dof, start, .. }
            | Self::Gripper { dof, start, .. }
            | Self::Replay { dof, start, .. } => (usize::from(*dof), start),
        }
    }
}

/// Request descriptor payload for `BeginMacro` (A7). Reserves a parent terminal independently
/// of any subsequent `InstallChild` commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeginMacroSpec {
    pub kind: MacroKind,
    pub prepared_boot: BootId,
    /// Original A1 admission `request_id` (`0` = fall back to broker `RequestId`).
    pub admission_request_id: u64,
    pub check_start: bool,
    pub dof: u8,
    pub start: [f32; MAX_DOF],
    pub check_workspace_revision: bool,
    pub calibration_revision: Option<u64>,
    pub poses_revision: Option<u64>,
    pub sequences_revision: Option<u64>,
    pub grasp_setup_revision: Option<u64>,
    pub power_budget_revision: Option<u64>,
    pub macro_deadline: Option<Instant>,
    pub timeout_code: BoundedText<48>,
    pub orientation_levelled: bool,
    pub task: HolderLabel,
    pub tag: HolderLabel,
}

impl BeginMacroSpec {
    pub fn new(kind: MacroKind, prepared_boot: BootId) -> Self {
        Self {
            kind,
            prepared_boot,
            admission_request_id: 0,
            check_start: false,
            dof: 0,
            start: [0.0; MAX_DOF],
            check_workspace_revision: false,
            calibration_revision: None,
            poses_revision: None,
            sequences_revision: None,
            grasp_setup_revision: None,
            power_budget_revision: None,
            macro_deadline: None,
            timeout_code: BoundedText::empty(),
            orientation_levelled: false,
            task: HolderLabel::default(),
            tag: HolderLabel::default(),
        }
    }

    pub fn with_start(mut self, start: &[f32]) -> Result<Self, &'static str> {
        if start.is_empty() || start.len() > MAX_DOF || start.iter().any(|v| !v.is_finite()) {
            return Err("INVALID_COMMAND");
        }
        self.check_start = true;
        self.dof = start.len() as u8;
        self.start = [0.0; MAX_DOF];
        self.start[..start.len()].copy_from_slice(start);
        Ok(self)
    }
}

/// Request descriptor payload for `FinishMacro` (A7). Reserves ACK/Refused only; triggers
/// emission of the parent's independently reserved terminal after physical quiescence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FinishMacroSpec {
    pub parent: ActionId,
    pub prepared_boot: BootId,
    pub state: FinishState,
    pub code: HolderLabel,
    pub message: StepErrorText,
    pub holding: Option<bool>,
    pub gripper_gap_rad: Option<f32>,
    pub held: HeldUpdate,
}

impl FinishMacroSpec {
    pub fn done(parent: ActionId, prepared_boot: BootId) -> Self {
        Self {
            parent,
            prepared_boot,
            state: FinishState::Done,
            code: HolderLabel::default(),
            message: StepErrorText::empty(),
            holding: None,
            gripper_gap_rad: None,
            held: HeldUpdate::Unchanged,
        }
    }

    pub fn failed(parent: ActionId, prepared_boot: BootId, code: &str, message: &str) -> Self {
        Self {
            parent,
            prepared_boot,
            state: FinishState::Failed,
            code: HolderLabel::new(code),
            message: StepErrorText::new(message),
            holding: None,
            gripper_gap_rad: None,
            held: HeldUpdate::Unchanged,
        }
    }

    pub fn cancelled(parent: ActionId, prepared_boot: BootId, code: Option<&str>, message: Option<&str>) -> Self {
        Self {
            parent,
            prepared_boot,
            state: FinishState::Cancelled,
            code: code.map(HolderLabel::new).unwrap_or_default(),
            message: message.map(StepErrorText::new).unwrap_or_default(),
            holding: None,
            gripper_gap_rad: None,
            held: HeldUpdate::Unchanged,
        }
    }
}

/// Action-family broker command carried inside `CommandKind::Action(ActionCommand)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ActionCommand {
    /// A6 standalone prepared action staged in `plan` (reserves ACK/Refused + Terminal).
    InstallPrepared {
        plan: PlanHandle,
        kind: PreparedActionKind,
        prepared_boot: BootId,
        /// Original A1 admission `request_id` (`0` = fall back to broker `RequestId`).
        admission_request_id: u64,
        check_workspace_revision: bool,
        poses_revision: Option<u64>,
        calibration_revision: Option<u64>,
        gripper_calib_revision: Option<u64>,
        power_budget_revision: Option<u64>,
    },
    /// A7 macro parent start (reserves ACK/Refused + Parent Terminal independently).
    BeginMacro(BeginMacroSpec),
    /// A7 macro child install staged in `plan` (reserves ACK/Refused only; parent owns Terminal).
    InstallChild {
        plan: PlanHandle,
        prepared_boot: BootId,
        check_workspace_revision: bool,
        calibration_revision: Option<u64>,
        gripper_calib_revision: Option<u64>,
        power_budget_revision: Option<u64>,
    },
    /// A7 macro parent finish (reserves ACK/Refused only; triggers parent Terminal).
    FinishMacro(FinishMacroSpec),
}

impl ActionCommand {
    /// Whether this command starts a new top-level action/parent and reserves a Terminal reply.
    pub fn starts_action(&self) -> bool {
        matches!(self, Self::InstallPrepared { .. } | Self::BeginMacro(_))
    }

    /// The plan slot staged for this command, if any.
    pub fn plan(&self) -> Option<PlanHandle> {
        match self {
            Self::InstallPrepared { plan, .. } | Self::InstallChild { plan, .. } => Some(*plan),
            Self::BeginMacro(_) | Self::FinishMacro(_) => None,
        }
    }

    /// Whether this command installs a macro child (retires its slot when the child finishes,
    /// without superseding the parent's `Installed` terminal token).
    pub fn is_child(&self) -> bool {
        matches!(self, Self::InstallChild { .. })
    }
}

/// Submission wrapper for `Submit::Action(ActionSubmit<'a, P>)`.
pub enum ActionSubmit<'a, P> {
    InstallPrepared {
        write: &'a dyn Fn(&mut P),
        kind: PreparedActionKind,
        prepared_boot: BootId,
        admission_request_id: u64,
        check_workspace_revision: bool,
        poses_revision: Option<u64>,
        calibration_revision: Option<u64>,
        gripper_calib_revision: Option<u64>,
        power_budget_revision: Option<u64>,
    },
    BeginMacro(BeginMacroSpec),
    InstallChild {
        write: &'a dyn Fn(&mut P),
        prepared_boot: BootId,
        check_workspace_revision: bool,
        calibration_revision: Option<u64>,
        gripper_calib_revision: Option<u64>,
        power_budget_revision: Option<u64>,
    },
    FinishMacro(FinishMacroSpec),
}

impl<P> ActionSubmit<'_, P> {
    pub fn starts_action(&self) -> bool {
        matches!(self, Self::InstallPrepared { .. } | Self::BeginMacro(_))
    }
}

// ---------------------------------------------------------------------------------------
// Single-entry-point authority, freshness, and admission rules

/// Unified authority envelope carried from off-thread preparation to owner-side install on
/// BOTH Legacy and RT Executor owners.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActionStamp {
    pub arm: ArmId,
    pub boot: BootId,
    pub admission: AdmissionStamp,
    pub config_revision: u64,
    pub calibration_revision: Option<u64>,
    pub poses_revision: Option<u64>,
    pub sequences_revision: Option<u64>,
    pub grasp_setup_revision: Option<u64>,
    pub gripper_calib_revision: Option<u64>,
    pub power_budget_revision: Option<u64>,
    pub parent_generation: u64,
    pub deadline: Instant,
}

impl ActionStamp {
    /// Converts a broker [`Descriptor`] plus optional domain revisions into an [`ActionStamp`],
    /// preserving original connection identity (`d.stamp.client.0`), original single-use A1
    /// `admission_request_id` (falling back to broker `d.stamp.request.0` when `0`), and
    /// verifying `prepared_boot`.
    #[allow(clippy::too_many_arguments)]
    pub fn from_descriptor(
        d: &Descriptor,
        prepared_boot: BootId,
        admission_request_id: u64,
        calibration_revision: Option<u64>,
        poses_revision: Option<u64>,
        sequences_revision: Option<u64>,
        grasp_setup_revision: Option<u64>,
        gripper_calib_revision: Option<u64>,
        power_budget_revision: Option<u64>,
    ) -> Self {
        // Fail closed if descriptor boot and prepared_boot disagree or either is zero.
        let boot = if prepared_boot == d.stamp.boot && !prepared_boot.is_zero() {
            prepared_boot
        } else {
            BootId(0)
        };
        let admission = if admission_request_id != 0 {
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
        };
        Self {
            arm: d.stamp.arm,
            boot,
            admission,
            config_revision: d.stamp.authority.config_revision,
            calibration_revision,
            poses_revision,
            sequences_revision,
            grasp_setup_revision,
            gripper_calib_revision,
            power_budget_revision,
            parent_generation: d.stamp.authority.parent_generation,
            deadline: d.stamp.deadline,
        }
    }

    /// Derives a child [`ActionStamp`] for a macro sub-motion carrying a fresh single-use
    /// A1 `AdmissionStamp::for_macro_child` while preserving parent authority verbatim.
    pub fn for_child(self, parent: ActionId, deadline: Instant) -> Self {
        Self {
            admission: self.admission.for_macro_child(),
            parent_generation: parent.0,
            deadline,
            ..self
        }
    }

    /// Constructs the corresponding broker [`Authority`].
    pub fn broker_authority(&self) -> Authority {
        Authority {
            lease_generation: self.admission.lease_gen,
            epoch: self.admission.estop_epoch,
            stop_generation: self.admission.stop_gen,
            config_revision: self.config_revision,
            parent_generation: self.parent_generation,
        }
    }
}

/// Owner-side commit clock and target identity passed into the single authority checker.
#[derive(Debug, Clone, Copy)]
pub struct OwnerCheckCtx {
    pub arm: ArmId,
    pub boot: BootId,
    pub now: Instant,
    pub stop_generation: u64,
    /// True on RT executor commits where `commit_feedback_valid()` is enforced.
    pub require_rt_feedback: bool,
}

impl OwnerCheckCtx {
    pub fn from_commit_ctx(ctx: &CommitCtx) -> Self {
        Self {
            arm: ctx.arm,
            boot: ctx.boot,
            now: ctx.now,
            stop_generation: ctx.stop_generation,
            require_rt_feedback: true,
        }
    }

    pub fn for_legacy(inner: &SupervisorInner) -> Self {
        Self {
            arm: inner.core.id(),
            boot: inner.boot_id,
            now: Instant::now(),
            stop_generation: inner.current_stop_gen(),
            require_rt_feedback: false,
        }
    }
}

/// Zero-alloc static refusal code produced by the single owner-side checks, convertible to
/// rich [`ArmError`] on Legacy paths without allocating on RT paths.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActionRefusal {
    pub code: &'static str,
}

impl ActionRefusal {
    pub const fn new(code: &'static str) -> Self {
        Self { code }
    }

    pub fn from_safety(err: SafetyError) -> Self {
        Self {
            code: static_code(ArmError::Safety(err)),
        }
    }

    pub fn into_arm_error(self, inner: &SupervisorInner) -> ArmError {
        self.into_arm_error_with_epoch(inner.safety.epoch_id())
    }

    pub fn into_arm_error_with_epoch(self, expected_epoch: u64) -> ArmError {
        match self.code {
            "WRONG_TARGET" => ArmError::Rejected {
                status: 409,
                code: "WRONG_TARGET".into(),
                message: "Action was prepared for a different arm or daemon boot".into(),
            },
            "DEADLINE_EXPIRED" => ArmError::Rejected {
                status: 408,
                code: "DEADLINE_EXPIRED".into(),
                message: "Action deadline expired before installation".into(),
            },
            "STOPPED" => ArmError::Safety(SafetyError::Stopped),
            "LEASE_LOST" => ArmError::Safety(SafetyError::LeaseLost),
            "ESTOP_ACTIVE" => ArmError::Safety(SafetyError::EStopActive),
            "EPOCH_MISMATCH" => ArmError::Safety(SafetyError::EpochMismatch {
                expected: expected_epoch,
                received: 0,
            }),
            "PLAN_STALE" => ArmError::Motion(MotionError::PlanStale(
                "state or revision changed while planning".into(),
            )),
            "CALIBRATION_CHANGED" => ArmError::Rejected {
                status: 409,
                code: "CALIBRATION_CHANGED".into(),
                message: "Calibration changed while planning".into(),
            },
            "PARENT_REPLACED" => ArmError::Rejected {
                status: 409,
                code: "PARENT_REPLACED".into(),
                message: "Macro parent action ended or was replaced".into(),
            },
            "CONFIG_BUSY" => ArmError::Rejected {
                status: 409,
                code: "CONFIG_BUSY".into(),
                message: "Configuration change is in progress".into(),
            },
            "FEEDBACK_STALE" => ArmError::Rejected {
                status: 503,
                code: "FEEDBACK_STALE".into(),
                message: "Joint feedback is stale or invalid".into(),
            },
            "MOTION_BUSY" => ArmError::Motion(MotionError::MotionBusy(
                "a motion or macro sub-motion is running".into(),
            )),
            "STALE_ADMISSION" => ArmError::Safety(SafetyError::StampAlreadyConsumed),
            "TERMINALS_BUSY" => ArmError::Safety(SafetyError::TerminalsBusy),
            "TOOL_MISMATCH" => ArmError::Rejected {
                status: 409,
                code: "TOOL_MISMATCH".into(),
                message: "Active tool changed since preparation".into(),
            },
            other => ArmError::Rejected {
                status: 409,
                code: other.into(),
                message: format!("Action refused: {other}"),
            },
        }
    }
}

/// **Single entry point for authority checks** (boot, arm, deadline, stop generation, epoch,
/// lease generation, domain revisions, parent generation, config inhibition, RT feedback, and
/// A1 preflight admission including `ClosedConnections` / `ClosedBitset` / `ConsumedStamps`).
///
/// Called by EVERY owner-side install function in PR P (both Legacy and RT Executor).
pub fn check_action_authority(
    inner: &SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    check_workspace_rev: bool,
    require_single_use_admission: bool,
) -> Result<(), ActionRefusal> {
    let current = Authority {
        lease_generation: inner.lease_valid_at(ctx.now).map_or(0, |l| l.generation),
        epoch: inner.safety.epoch_id(),
        stop_generation: ctx.stop_generation,
        config_revision: if check_workspace_rev {
            ConfigAuthority::revision(inner, ConfigDomain::Workspace)
        } else {
            stamp.config_revision
        },
        parent_generation: inner
            .last_action
            .as_ref()
            .filter(|a| a.is_running())
            .map_or(0, |a| a.id),
    };
    let wire_stamp = Stamp {
        arm: stamp.arm,
        boot: stamp.boot,
        request: RequestId(stamp.admission.request_id.max(1)),
        client: ClientId(stamp.admission.connection_id),
        client_seq: 0,
        authority: stamp.broker_authority(),
        deadline: stamp.deadline,
    };
    if stamp.admission.estop_epoch != inner.safety.epoch_id()
        && (inner.e_stop_latched.load(Ordering::SeqCst)
            || matches!(inner.safety.state(), SafetyState::EStop))
    {
        return Err(ActionRefusal::new("ESTOP_ACTIVE"));
    }
    if let Some(code) = wire_stamp.check(ctx.arm, ctx.boot, ctx.now, &current) {
        return Err(ActionRefusal::new(code));
    }
    if inner.lease_enabled && stamp.admission.lease_gen == 0 {
        return Err(ActionRefusal::new("NO_LEASE"));
    }
    if let Some(cal_rev) = stamp.calibration_revision {
        if inner.calibration_revision != cal_rev {
            return Err(ActionRefusal::new("CALIBRATION_CHANGED"));
        }
    }
    if let Some(poses_rev) = stamp.poses_revision {
        if inner.poses.revision != poses_rev {
            return Err(ActionRefusal::new("PLAN_STALE"));
        }
    }
    if let Some(seq_rev) = stamp.sequences_revision {
        if inner.sequences.revision != seq_rev {
            return Err(ActionRefusal::new("PLAN_STALE"));
        }
    }
    if let Some(gs_rev) = stamp.grasp_setup_revision {
        if inner.grasp_setup_revision != gs_rev {
            return Err(ActionRefusal::new("PLAN_STALE"));
        }
    }
    if let Some(gc_rev) = stamp.gripper_calib_revision {
        if inner.gripper_calib_revision != gc_rev {
            return Err(ActionRefusal::new("PLAN_STALE"));
        }
    }
    if let Some(pb_rev) = stamp.power_budget_revision {
        if inner.power_budget_revision != pb_rev {
            return Err(ActionRefusal::new("PLAN_STALE"));
        }
    }
    if inner.core.motion_inhibited_by_config() {
        return Err(ActionRefusal::new("CONFIG_BUSY"));
    }
    if ctx.require_rt_feedback && !inner.commit_feedback_valid() {
        return Err(ActionRefusal::new("FEEDBACK_STALE"));
    }
    if stamp.admission.connection_id != 0 && inner.is_connection_closed(stamp.admission.connection_id)
    {
        return Err(ActionRefusal::new("LEASE_LOST"));
    }
    if require_single_use_admission {
        inner
            .preflight_admission(stamp.admission)
            .map_err(ActionRefusal::from_safety)?;
    }
    Ok(())
}

/// **Single entry point for start-position freshness** (finite full-DOF check across arm and
/// gripper against [`START_TOLERANCE`]), shared by BOTH Legacy and RT Executor owners.
pub fn check_start_fresh(commanded: &[f32], prepared_start: &[f32]) -> Result<(), ActionRefusal> {
    if prepared_start.len() != commanded.len()
        || prepared_start
            .iter()
            .zip(commanded)
            .any(|(s, c)| !s.is_finite() || !c.is_finite() || (s - c).abs() >= START_TOLERANCE)
    {
        return Err(ActionRefusal::new("PLAN_STALE"));
    }
    Ok(())
}

/// **Single entry point for single-use A1 admission consumption**: syncs `stop_generation`
/// and consumes `stamp.admission` via A1's [`SupervisorInner::check_admission`].
pub fn consume_action_admission<'g>(
    inner: &'g mut SupervisorInner,
    stamp: &ActionStamp,
    stop_generation: u64,
) -> Result<Admitted<'g>, ActionRefusal> {
    inner.sync_stop_generation(stop_generation);
    inner
        .check_admission(stamp.admission)
        .map_err(ActionRefusal::from_safety)
}

// ---------------------------------------------------------------------------------------
// Revision-tagged off-thread planning environment & snapshot

/// Slow-changing planning configuration and catalogs published off-thread via [`StateHub`]
/// (never touched by `armd-control`), tagged with the exact owner revisions it matches.
#[derive(Debug, Clone)]
pub struct ActionPrepEnv {
    pub config: ArmConfig,
    pub workspace_policy: Option<WorkspacePolicy>,
    pub power_budget: Option<PowerBudget>,
    pub poses: PoseList,
    pub sequences: SequenceStore,
    pub grasp_setup: Option<GraspSetup>,
    pub episodes: EpisodeStore,
    pub gripper_close_speed_limit: Option<f32>,
    pub gripper_ceiling_torque_estimate: Option<f32>,
    pub has_torque_feedback: bool,
    pub requires_real_jaw_curve: bool,
    pub driver_present: bool,
    pub follower: Option<String>,
    pub joint_zero_offsets: Vec<f32>,
    // Exact owner revisions captured alongside this environment.
    pub workspace_revision: u64,
    pub calibration_revision: u64,
    pub poses_revision: u64,
    pub sequences_revision: u64,
    pub grasp_setup_revision: u64,
    pub gripper_calib_revision: u64,
    pub power_budget_revision: u64,
    pub tool_digest_hash: u64,
}

impl ActionPrepEnv {
    /// Captures a coherent revision-tagged planning environment from `inner` (called off-RT
    /// at startup, before executor transfer, and after config mutations).
    pub fn from_inner(inner: &SupervisorInner) -> Self {
        let has_torque_feedback = inner
            .driver
            .as_ref()
            .is_none_or(|d| d.lock().is_ok_and(|drv| drv.capabilities().torque_feedback));
        Self {
            config: inner.config.clone(),
            workspace_policy: inner.workspace_policy().map(|(ws, _)| ws.clone()),
            power_budget: inner.power_budget().cloned(),
            poses: inner.poses.list.clone(),
            sequences: inner.sequences.clone(),
            grasp_setup: inner.grasp_setup(),
            episodes: inner.episodes.clone(),
            gripper_close_speed_limit: inner.gripper_close_speed_limit(),
            gripper_ceiling_torque_estimate: inner.gripper_ceiling_torque_estimate(),
            has_torque_feedback,
            requires_real_jaw_curve: inner.requires_real_jaw_curve(),
            driver_present: inner.driver.is_some(),
            follower: inner.follower.clone(),
            joint_zero_offsets: inner.joint_zero_offsets.clone(),
            workspace_revision: inner.workspace_revision,
            calibration_revision: inner.calibration_revision,
            poses_revision: inner.poses.revision,
            sequences_revision: inner.sequences.revision,
            grasp_setup_revision: inner.grasp_setup_revision,
            gripper_calib_revision: inner.gripper_calib_revision,
            power_budget_revision: inner.power_budget_revision,
            tool_digest_hash: tool_digest_hash(&inner.config.active_tool.config_digest),
        }
    }

    /// Verifies that every revision in this environment matches a live [`PublishedOwnerMeta`]
    /// sample (fail-closed: any mismatch refuses with `PLAN_STALE`).
    pub fn matches_owner_meta(&self, owner: &PublishedOwnerMeta) -> bool {
        self.workspace_revision == owner.authority.config_revision
            && self.calibration_revision == owner.calibration_revision
            && self.poses_revision == owner.poses_revision
            && self.sequences_revision == owner.sequences_revision
            && self.grasp_setup_revision == owner.grasp_setup_revision
            && self.gripper_calib_revision == owner.gripper_calib_revision
            && self.power_budget_revision == owner.power_budget_revision
            && self.tool_digest_hash == owner.tool_digest_hash
    }

    /// Planning velocity limits (`planning_limits()` equivalent), optionally scaled on non-gripper joints.
    pub fn planning_limits(&self, speed_scale: Option<f64>) -> Vec<JointLimit> {
        let scale = self.config.speed_cap.and_then(|c| c.joint_velocity_scale).unwrap_or(1.0);
        let gripper_idx = self.config.gripper_index();
        self.config
            .limits
            .iter()
            .enumerate()
            .map(|(i, l)| {
                let mut lim = l.clone();
                if Some(i) != gripper_idx {
                    if let Some(cap) = self.config.planned_max_velocity {
                        lim.max_velocity = lim.max_velocity.min(cap);
                    }
                    lim.max_velocity *= scale;
                    if let Some(s) = speed_scale {
                        lim.max_velocity *= s as f32;
                    }
                }
                lim
            })
            .collect()
    }

    /// Same-side approach plan computed off-lock from `self.config` (exact match to
    /// `SupervisorInner::approach_plan`).
    pub fn approach_plan(
        &self,
        use_: &str,
        request: backlash::Request,
    ) -> Result<backlash::Plan, Option<&'static str>> {
        let policy = match backlash::resolve(self.config.backlash_approach.as_ref(), use_, request) {
            backlash::Resolved::Direct(r) => return Err(r),
            backlash::Resolved::SameSide(p) => p,
        };
        let arm = self.config.arm_joint_indices();
        let joints = policy.joint_indices(&self.config.joint_names, &arm);
        let planning = self.planning_limits(None);
        let mut seg_limits = backlash::segment_limits(&planning, &joints, policy.speed_rad_s);
        if self.config.active_tool.commissioning_required {
            let gripper_idx = self.config.gripper_index();
            for (i, (seg, base)) in seg_limits.iter_mut().zip(&planning).enumerate() {
                if Some(i) != gripper_idx {
                    seg.max_velocity = seg.max_velocity.min(base.max_velocity * 0.2).min(0.2);
                }
            }
        }
        Ok(backlash::Plan {
            policy,
            joints,
            seg_limits,
            joint_names: self.config.joint_names.clone(),
            arm_dof: arm.len(),
        })
    }

    /// Joint-space IK seed poses besides the current joints (home + resting pose).
    pub fn goal_seed_poses(&self) -> Vec<[f64; 6]> {
        let arm_dof = self.config.arm_dof();
        let q6 = |q: &[f32]| crate::kinematics::q6(&q[..arm_dof.min(q.len())]);
        let mut seeds = vec![q6(&self.config.home_positions)];
        let rest = q6(self.config.resting_positions());
        if rest != seeds[0] {
            seeds.push(rest);
        }
        seeds
    }
}

/// Complete, revision-coherent snapshot used by off-thread preparation (`prepare_*`) and
/// macro orchestrators on BOTH Legacy and RT Executor owners.
#[derive(Debug, Clone)]
pub struct ActionSnapshot {
    pub env: Arc<ActionPrepEnv>,
    pub owner: PublishedOwnerMeta,
    pub safety_state: SafetyState,
    pub commanded_positions: Vec<f32>,
    pub commanded_velocities: Vec<f32>,
    pub measured_positions: Vec<f32>,
    pub measured_velocities: Vec<f32>,
    pub feedback_instants: Vec<Option<Instant>>,
    pub feedback_seqs: Vec<u64>,
    pub feedback_queue_bounds: Vec<Option<Instant>>,
    pub stamp: ActionStamp,
}

impl ActionSnapshot {
    /// Assembles a revision-coherent snapshot strictly from published [`StateHub`] state
    /// (zero `SupervisorLock` — used on `OWNER_EXECUTOR`). Fails closed with `PLAN_STALE`
    /// if `prep_env` revisions do not match `snap.rt.owner` revisions.
    pub fn from_published(
        snap: &Snapshot,
        env: Arc<ActionPrepEnv>,
        admission: AdmissionStamp,
        deadline: Instant,
    ) -> Result<Self, ArmError> {
        let owner = snap.rt.owner;
        if owner.boot.is_zero() {
            return Err(ArmError::Rejected {
                status: 503,
                code: "OBSERVATION_UNAVAILABLE".into(),
                message: "No live owner metadata published yet".into(),
            });
        }
        if !env.matches_owner_meta(&owner) {
            return Err(ArmError::Motion(MotionError::PlanStale(
                "published planning environment revision lags owner revision".into(),
            )));
        }
        let stamp = ActionStamp {
            arm: owner.arm,
            boot: owner.boot,
            admission,
            config_revision: env.workspace_revision,
            calibration_revision: Some(env.calibration_revision),
            poses_revision: Some(env.poses_revision),
            sequences_revision: Some(env.sequences_revision),
            grasp_setup_revision: Some(env.grasp_setup_revision),
            gripper_calib_revision: Some(env.gripper_calib_revision),
            power_budget_revision: Some(env.power_budget_revision),
            parent_generation: owner.authority.parent_generation,
            deadline,
        };
        Ok(Self {
            env,
            owner,
            safety_state: snap.obs.safety_state.clone(),
            commanded_positions: snap.rt.commanded_positions.clone(),
            commanded_velocities: snap.rt.commanded_velocities.clone(),
            measured_positions: snap.rt.positions.clone(),
            measured_velocities: snap.rt.velocities.clone(),
            feedback_instants: snap.rt.feedback_instants.clone(),
            feedback_seqs: snap.rt.feedback_seqs.clone(),
            feedback_queue_bounds: snap.rt.feedback_queue_bounds.clone(),
            stamp,
        })
    }

    /// Captures a snapshot from a short read/write guard of `SupervisorInner` (Legacy owner:
    /// zero planning under the lock; reflects direct mutations even without a tick loop).
    pub fn from_inner(inner: &SupervisorInner, admission: AdmissionStamp, deadline: Instant) -> Self {
        let env = Arc::new(ActionPrepEnv::from_inner(inner));
        let owner = inner.published_owner_meta();
        let stamp = ActionStamp {
            arm: owner.arm,
            boot: owner.boot,
            admission,
            config_revision: env.workspace_revision,
            calibration_revision: Some(env.calibration_revision),
            poses_revision: Some(env.poses_revision),
            sequences_revision: Some(env.sequences_revision),
            grasp_setup_revision: Some(env.grasp_setup_revision),
            gripper_calib_revision: Some(env.gripper_calib_revision),
            power_budget_revision: Some(env.power_budget_revision),
            parent_generation: owner.authority.parent_generation,
            deadline,
        };
        Self {
            env,
            owner,
            safety_state: inner.safety.state().clone(),
            commanded_positions: inner.commanded_positions.clone(),
            commanded_velocities: inner.commanded_velocities.clone(),
            measured_positions: inner.measured_positions.clone(),
            measured_velocities: inner.measured_velocities.clone(),
            feedback_instants: inner.core.feedback().instants().to_vec(),
            feedback_seqs: inner.core.feedback().seqs().to_vec(),
            feedback_queue_bounds: inner.joint_feedback_queue_bounds.clone(),
            stamp,
        }
    }

    /// Captures a snapshot from `sup` respecting the active owner mode:
    /// - `OWNER_EXECUTOR`: reads ONLY `StateHub::latest()` + `StateHub::prep_env()` (0 lock).
    /// - `OWNER_LEGACY`: takes a brief `read()` lock (never writes or plans under lock).
    pub async fn capture(
        sup: &ArmSupervisor,
        admission: AdmissionStamp,
        deadline: Instant,
    ) -> Result<Self, ArmError> {
        if sup.owner_mode() == OWNER_EXECUTOR {
            let snap = sup.hub().latest().ok_or_else(|| ArmError::Rejected {
                status: 503,
                code: "OBSERVATION_UNAVAILABLE".into(),
                message: "No published snapshot available yet".into(),
            })?;
            let env = sup.hub().prep_env().ok_or_else(|| ArmError::Rejected {
                status: 503,
                code: "OBSERVATION_UNAVAILABLE".into(),
                message: "No published planning environment available yet".into(),
            })?;
            Self::from_published(&snap, env, admission, deadline)
        } else {
            let inner = sup.inner().read().await;
            if inner.detached {
                return Err(ArmError::Rejected {
                    status: 503,
                    code: "OWNER_TRANSFERRED".into(),
                    message: "Ownership transferred to RT executor".into(),
                });
            }
            Ok(Self::from_inner(&inner, admission, deadline))
        }
    }

    /// Checks operational preconditions against the published snapshot (mirrors
    /// `SupervisorInner::action_preconditions` + `gravity_comp_motion_busy`).
    pub fn check_preconditions(&self) -> Result<(), ArmError> {
        self.check_preconditions_inner(false)
    }

    /// Checks operational preconditions for gripper actions (mirrors `action_preconditions`:
    /// gripper remains allowed while gravity compensation is active).
    pub fn check_gripper_preconditions(&self) -> Result<(), ArmError> {
        self.check_preconditions_inner(true)
    }

    fn check_preconditions_inner(&self, allow_gravity_comp: bool) -> Result<(), ArmError> {
        if self.owner.handover_settling {
            return Err(SafetyError::HandoverSettling.into());
        }
        match &self.safety_state {
            SafetyState::EStop => return Err(SafetyError::EStopActive.into()),
            SafetyState::Fault(r) => return Err(SafetyError::FaultActive(r.clone()).into()),
            s if !s.is_operational() => return Err(SafetyError::NotArmed(s.clone()).into()),
            _ => {}
        }
        if self.owner.pending_rearm || self.owner.pending_normal_unload {
            return Err(SafetyError::NotArmed(self.safety_state.clone()).into());
        }
        if self.owner.collision_hold {
            return Err(SafetyError::CollisionHold("collision hold active".into()).into());
        }
        if self.owner.config_inhibited {
            return Err(ArmError::Rejected {
                status: 409,
                code: "CONFIG_BUSY".into(),
                message: "Configuration change is in progress".into(),
            });
        }
        if self.owner.calibration_busy {
            return Err(MotionError::MotionBusy("a calibration apply is running".into()).into());
        }
        if self.owner.zero_check_persisting {
            return Err(
                MotionError::MotionBusy("gripper calibration is being persisted".into()).into(),
            );
        }
        if self.owner.policy_busy {
            return Err(MotionError::MotionBusy("a policy run is active".into()).into());
        }
        if self.owner.motion_active {
            return Err(MotionError::MotionBusy("a trajectory is running".into()).into());
        }
        if self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) {
            return Err(MotionError::MotionBusy("the arm is still moving".into()).into());
        }
        if !allow_gravity_comp && self.owner.gravity_comp_active {
            return Err(
                MotionError::MotionBusy("gravity compensation is active".into()).into(),
            );
        }
        Ok(())
    }

    /// Builds a [`PlanInput`] in `goal_frame` completely from this published snapshot (0 lock).
    pub fn plan_input_in(&self, goal_frame: GoalFrame) -> Result<PlanInput, ArmError> {
        if self.env.config.kinematics.is_none() || !self.env.config.has_cartesian_with_gripper() {
            return Err(MotionError::Unsupported(
                "Cartesian actions need a kinematics model and an arm with gripper".into(),
            )
            .into());
        }
        self.check_preconditions()?;
        let arm_dof = self.env.config.arm_dof();
        let gripper_idx = self.env.config.gripper_index().unwrap_or(arm_dof);
        Ok(PlanInput {
            q0: crate::kinematics::q6(
                &self.commanded_positions[..arm_dof.min(self.commanded_positions.len())],
            ),
            j7: self
                .commanded_positions
                .get(gripper_idx)
                .copied()
                .unwrap_or(0.0),
            limits: self.env.planning_limits(None),
            speed_cap: self.env.config.speed_cap,
            ws: self.env.workspace_policy.clone(),
            cam: self.env.config.camera_mount.clone(),
            arm: self.env.config.arm_model.clone(),
            epoch: self.owner.authority.epoch,
            workspace_revision: self.env.workspace_revision,
            power: self.env.power_budget.clone(),
            tcp_offset: self.env.config.tcp_offset,
            goal_frame,
        })
    }

    /// Builds a [`JointGoalSnapshot`] for off-lock quintic planning from this snapshot.
    pub fn joint_goal_snapshot(&self, speed_scale: f64) -> JointGoalSnapshot {
        JointGoalSnapshot {
            start: self.commanded_positions.clone(),
            v0: self.commanded_velocities.clone(),
            limits: self.env.planning_limits(Some(speed_scale)),
            arm_dof: self.env.config.arm_dof(),
            gripper_index: self.env.config.gripper_index(),
        }
    }

    /// Builds a [`ReplayInput`] from this snapshot without taking `SupervisorLock`
    /// (exact match to `SupervisorInner::replay_input`).
    pub fn replay_input(
        &self,
        request: ReplayRequest,
        epoch: u64,
    ) -> Result<ReplayInput, ArmError> {
        if self.env.config.active_tool.commissioning_required {
            return Err(SafetyError::CommissioningRequired(
                "replay (blocked while active tool requires commissioning; use jog/move_to/move_relative/gripper at speed_scale <= 0.2)".into(),
            )
            .into());
        }
        if self.owner.authority.epoch != epoch {
            return Err(SafetyError::EpochMismatch {
                expected: self.owner.authority.epoch,
                received: epoch,
            }
            .into());
        }
        let path = self.env.episodes.path_of(&request.episode)?;
        self.check_preconditions()?;
        Ok(ReplayInput {
            path,
            profile_id: self.env.config.profile_id.clone(),
            dof: self.env.config.dof,
            arm_dof: self.env.config.arm_dof(),
            gripper_idx: self.env.config.gripper_index(),
            joint_names: self.env.config.limits.iter().map(|l| l.name.clone()).collect(),
            limits: self.env.config.limits.clone(),
            speed_limits: self.env.planning_limits(None),
            ws: self
                .env
                .workspace_policy
                .clone()
                .map(|ws| (ws, self.env.config.camera_mount.clone())),
            arm: self.env.config.arm_model.clone(),
            start: self.commanded_positions.clone(),
            control_hz: self.env.config.control_frequency_hz,
            epoch,
            workspace_revision: self.env.workspace_revision,
            power: self.env.power_budget.clone(),
            store: self.env.episodes.clone(),
            zero_offsets: self.env.joint_zero_offsets.clone(),
            tcp_offset: self.env.config.tcp_offset,
            active_tool: self.env.config.active_tool.clone(),
            request,
        })
    }
}

// ---------------------------------------------------------------------------------------
// Off-thread prepared payloads (shared by Legacy and Executor)

/// Prepared linear Cartesian move (`move_to` / `move_relative`).
#[derive(Debug, Clone)]
pub struct PreparedCartesianAction {
    pub kind: PreparedActionKind,
    pub stamp: ActionStamp,
    pub start: Vec<f32>,
    pub input: PlanInput,
    pub direct: JointPath,
    pub pre: backlash::Pre,
    pub approach_plan: Option<backlash::Plan>,
    pub rules: Option<backlash::DescentRules>,
    pub tag: Option<String>,
    pub expected_duration_s: f64,
}

/// Resolves `(path_to_stage, approach_then)` for RT staging so `backlash::Pre::Ready(cp)` is
/// NEVER staged without its second-leg final approach segment; falls back cleanly to `direct`.
pub fn resolve_cartesian_rt_legs<'a>(
    direct: &'a JointPath,
    pre: &'a backlash::Pre,
    approach_plan: Option<&backlash::Plan>,
    power_budget: Option<&PowerBudget>,
    arm: &crate::chain::ArmModel,
) -> (&'a JointPath, Option<StagedQuinticLeg>) {
    if let (backlash::Pre::Ready(cp), Some(plan)) = (pre, approach_plan) {
        let dof = cp.path.dof;
        if dof > 0 && dof <= MAX_DOF && cp.target.len() == dof {
            let zeros = vec![0.0f32; dof];
            if let Ok(seg) = crate::trajectory::TrajectoryPlanner::plan(
                cp.path.last_slice(),
                &zeros,
                &cp.target,
                &zeros,
                Some(backlash::MIN_SEGMENT_S),
                &plan.seg_limits,
            ) {
                if let Ok((seg, power_scale)) = fit_quintic_power_off_lock(
                    seg,
                    cp.path.last_slice(),
                    &zeros,
                    &cp.target,
                    &plan.seg_limits,
                    power_budget,
                    arm,
                ) {
                    let mut target = [0.0f32; MAX_DOF];
                    target[..dof].copy_from_slice(&cp.target);
                    return (
                        &cp.path,
                        Some(StagedQuinticLeg {
                            target,
                            duration_s: seg.duration,
                            power_scale,
                        }),
                    );
                }
            }
        }
    }
    (direct, None)
}

impl PreparedCartesianAction {
    /// Stages this prepared Cartesian path into a broker [`PlanSlot`] (no IK on RT).
    pub fn to_plan_slot(&self, child: Option<ChildSpec>) -> Result<PlanSlot, &'static str> {
        let (path, approach_then) = resolve_cartesian_rt_legs(
            &self.direct,
            &self.pre,
            self.approach_plan.as_ref(),
            self.input.power.as_ref(),
            &self.input.arm,
        );
        if self.start.len() != path.dof || self.start.len() > MAX_DOF {
            return Err("INVALID_COMMAND");
        }
        let mut slot = PlanSlot::default();
        for (q, &s) in path.q.iter().zip(&path.s) {
            slot.path.push(&q[..path.dof], s)?;
        }
        let mut start = [0.0f32; MAX_DOF];
        start[..self.start.len()].copy_from_slice(&self.start);
        slot.primitive = Primitive::Action(StagedAction::Cartesian {
            child,
            kind: self.kind,
            dof: path.dof as u8,
            start,
            duration_s: path.duration,
            power_scale: self.input.power.as_ref().map(|_| path.power_scale),
            approach_then,
        });
        Ok(slot)
    }
}

/// Prepared joint move (`move_to` / `move_relative` `path: "joint"` or `move_to_pose`).
#[derive(Debug, Clone)]
pub struct PreparedJointAction {
    pub kind: PreparedActionKind,
    pub stamp: ActionStamp,
    pub start: Vec<f32>,
    pub target: Vec<f32>,
    pub limits: Vec<JointLimit>,
    pub speed_scale: f64,
    pub prepared_quintic: Option<PreparedQuintic>,
    pub approach_use: &'static str,
    pub same_side: Option<backlash::Request>,
    pub tag: Option<String>,
    pub expected_duration_s: f64,
    pub settle: bool,
}

impl PreparedJointAction {
    /// Stages this prepared joint move into a broker [`PlanSlot`] (zero-alloc on RT).
    pub fn to_plan_slot(&self, child: Option<ChildSpec>) -> Result<PlanSlot, &'static str> {
        let dof = self.start.len();
        if dof == 0 || dof > MAX_DOF || self.target.len() != dof || self.limits.len() != dof {
            return Err("INVALID_COMMAND");
        }
        let mut start = [0.0f32; MAX_DOF];
        let mut target = [0.0f32; MAX_DOF];
        let mut max_velocity = [0.0f32; MAX_DOF];
        start[..dof].copy_from_slice(&self.start);
        target[..dof].copy_from_slice(&self.target);
        for (i, l) in self.limits.iter().enumerate() {
            max_velocity[i] = l.max_velocity;
        }
        let (duration_s, power_scale) = match &self.prepared_quintic {
            Some(p) => (p.traj.duration, p.power_scale),
            None => (self.expected_duration_s, None),
        };
        let mut slot = PlanSlot::default();
        slot.set(Primitive::Action(StagedAction::Joint {
            child,
            kind: self.kind,
            dof: dof as u8,
            start,
            target,
            duration_s,
            max_velocity,
            power_scale,
            settle: self.settle,
            approach_then: None,
        }));
        Ok(slot)
    }
}

/// Prepared gripper action (`gripper` standalone or macro child).
#[derive(Debug, Clone)]
pub struct PreparedGripperAction {
    pub stamp: ActionStamp,
    pub start: Vec<f32>,
    pub request: GripperRequest,
    pub speed_scale: Option<f64>,
    pub baseline_cmd: GripperBaselineCmd,
    pub expected_duration_s: f64,
    pub timeout_s: f64,
    pub is_pos_force_close: bool,
    pub max_budget_s: Option<f64>,
    pub extend_pos_force_close_timeout: bool,
}

impl PreparedGripperAction {
    /// Stages this prepared gripper action into a broker [`PlanSlot`].
    pub fn to_plan_slot(&self, child: Option<ChildSpec>) -> Result<PlanSlot, &'static str> {
        let dof = self.start.len();
        if dof == 0 || dof > MAX_DOF {
            return Err("INVALID_COMMAND");
        }
        let mut start = [0.0f32; MAX_DOF];
        start[..dof].copy_from_slice(&self.start);
        let mut slot = PlanSlot::default();
        slot.set(Primitive::Action(StagedAction::Gripper {
            child,
            dof: dof as u8,
            start,
            command: self.request.command.into(),
            fraction: self.request.fraction,
            grasp: self.request.grasp,
            speed: self.request.speed,
            timeout_s: self.request.timeout_s,
            max_budget_s: self.max_budget_s,
            extend_pos_force_close_timeout: self.extend_pos_force_close_timeout,
            speed_scale: self.speed_scale,
        }));
        Ok(slot)
    }
}

/// Prepared replay action (`replay` standalone or macro child).
#[derive(Debug, Clone)]
pub struct PreparedReplayAction {
    pub stamp: ActionStamp,
    pub start: Vec<f32>,
    pub plan: ReplayPlan,
    pub tool_digest: u64,
    pub expected_duration_s: f64,
}

impl PreparedReplayAction {
    /// Stages this prepared replay into a broker [`PlanSlot`] (no file I/O on RT).
    pub fn to_plan_slot(&self, child: Option<ChildSpec>) -> Result<PlanSlot, &'static str> {
        let dof = self.start.len();
        if dof == 0 || dof > MAX_DOF {
            return Err("INVALID_COMMAND");
        }
        let episode = EpisodeRef::new(&self.plan.input.request.episode).ok_or("INVALID_COMMAND")?;
        let mut slot = PlanSlot::default();
        for (i, sample) in self.plan.samples.iter().enumerate() {
            slot.path.push(sample, i as f64)?;
        }
        let mut start = [0.0f32; MAX_DOF];
        start[..dof].copy_from_slice(&self.start);
        slot.primitive = Primitive::Action(StagedAction::Replay {
            child,
            dof: dof as u8,
            start,
            episode,
            speed: self.plan.input.request.speed as f32,
            gripper: self.plan.input.request.gripper,
            sample_dt: self.plan.sample_dt,
            duration_s: self.plan.duration,
            power_scale: self.plan.power_scale,
            tool_digest: self.tool_digest,
        });
        Ok(slot)
    }
}

// ---------------------------------------------------------------------------------------
// Off-thread preparation functions (pure snapshot -> prepared payload; 0 SupervisorLock)

/// Off-thread preparation for `/api/v1/action/gripper` from a published [`ActionSnapshot`].
/// Preserves exact wire validation order: `speed_scale` bounds & commissioning `0.2 + 1e-6`
/// cutoff, explicit `speed` precedence, `baseline: null` allowed vs non-null rejected,
/// `speed > 0.4 + 1e-6` commissioning check, gripper policy & torque-feedback check, and
/// pos-force wire quantization.
pub fn prepare_gripper(
    snap: &ActionSnapshot,
    body: &Value,
    baseline_cmd: GripperBaselineCmd,
) -> Result<PreparedGripperAction, ArmError> {
    if !body.is_object() {
        return Err(MotionError::InvalidCommand("Command body must be an object".into()).into());
    }
    let mut request = motion::parse_gripper(body)?;
    let speed_scale = motion::num(body, "speed_scale")?;
    if let Some(scale) = speed_scale {
        if !(0.05..=1.0).contains(&scale) {
            return Err(
                MotionError::InvalidCommand("speed_scale must be in [0.05, 1.0]".into()).into(),
            );
        }
        if snap.env.config.active_tool.commissioning_required && scale > 0.2 + 1e-6 {
            return Err(SafetyError::CommissioningRequired(format!(
                "gripper (speed_scale {scale:.2} > 0.2)"
            ))
            .into());
        }
        if body.get("speed").is_none_or(Value::is_null) {
            request.speed = (2.0 * scale).clamp(0.01, 2.0);
        }
    }
    if body.get("baseline").is_some_and(|v| !v.is_null()) {
        return Err(MotionError::InvalidCommand(
            "baseline is not a gripper action field; calibrate with /api/v1/action/gripper_calibrate".into(),
        )
        .into());
    }
    prepare_gripper_request(snap, request, speed_scale, baseline_cmd, None, false)
}

/// Off-thread preparation for a parsed [`GripperRequest`] (shared by standalone `gripper` and
/// macro gripper children).
pub fn prepare_gripper_request(
    snap: &ActionSnapshot,
    request: GripperRequest,
    speed_scale: Option<f64>,
    baseline_cmd: GripperBaselineCmd,
    max_budget_s: Option<f64>,
    extend_pos_force_close_timeout: bool,
) -> Result<PreparedGripperAction, ArmError> {
    let gripper_idx = snap.env.config.gripper_index().ok_or_else(|| {
        MotionError::Unsupported("Gripper actions need a profile with a gripper joint".into())
    })?;
    let policy = snap.env.config.gripper_policy.clone().ok_or_else(|| {
        MotionError::Unsupported("Gripper actions need a profile with a gripper policy".into())
    })?;
    if max_budget_s.is_none()
        && snap.env.config.active_tool.commissioning_required
        && request.speed > 0.4 + 1e-6
    {
        return Err(SafetyError::CommissioningRequired(format!(
            "gripper (speed {:.2} rad/s > 0.40 rad/s)",
            request.speed
        ))
        .into());
    }
    // Note: macro children run while parent macro is alive (motion_active is false between children).
    // Standalone gripper remains permitted while gravity compensation is active (`action_preconditions`).
    if max_budget_s.is_none() {
        snap.check_gripper_preconditions()?;
    }
    if policy.max_close_torque_nm.is_some() {
        let close_dir = if policy.closed_rad >= policy.open_rad { 1.0_f32 } else { -1.0_f32 };
        let cur = snap.commanded_positions.get(gripper_idx).copied().unwrap_or(0.0);
        let target = match request.command {
            GripperCommand::Open => policy.open_rad,
            GripperCommand::Close => policy.closed_rad,
            GripperCommand::Move => {
                policy.closed_rad + request.fraction.unwrap_or(0.0) as f32 * (policy.open_rad - policy.closed_rad)
            }
            GripperCommand::Seek => policy.closed_rad + close_dir * 0.6,
        };
        let is_closing = matches!(request.command, GripperCommand::Close | GripperCommand::Seek)
            || (target - cur) * close_dir > 1e-4;
        if is_closing && snap.env.driver_present && !snap.env.has_torque_feedback {
            return Err(MotionError::Unsupported(
                "Gripper close requires torque_feedback when max_close_torque_nm is configured"
                    .into(),
            )
            .into());
        }
    }
    let start_g = snap.commanded_positions.get(gripper_idx).copied().unwrap_or(0.0);
    let mut motion = GripperMotion::try_with_close_speed_limit(
        request.clone(),
        policy,
        snap.env.config.limits[gripper_idx].clone(),
        start_g,
        gripper_idx,
        snap.env.gripper_close_speed_limit,
    )?
    .with_ceiling_torque_estimate(snap.env.gripper_ceiling_torque_estimate);
    let is_pos_force_close = snap.env.gripper_close_speed_limit.is_some() && motion.is_closing();
    if is_pos_force_close && extend_pos_force_close_timeout && motion.request.timeout_s.is_some() {
        motion.use_default_timeout();
    }
    let expected_duration_s = motion.expected_duration();
    let timeout_s = motion.timeout_s;
    if let Some(budget_s) = max_budget_s {
        if !(budget_s.is_finite() && budget_s > 0.0) {
            return Err(MotionError::InvalidCommand(format!(
                "remaining macro budget exhausted ({budget_s:.3} s <= 0)"
            ))
            .into());
        }
        if is_pos_force_close && expected_duration_s > budget_s {
            return Err(MotionError::InvalidCommand(format!(
                "gripper motion needs {expected_duration_s:.1} s (deadline {timeout_s:.1} s), exceeding remaining macro budget {budget_s:.1} s"
            ))
            .into());
        }
    }
    let mut stamp = snap.stamp;
    stamp.parent_generation = 0;
    stamp.poses_revision = None;
    stamp.sequences_revision = None;
    stamp.grasp_setup_revision = None;
    stamp.calibration_revision = None;
    stamp.power_budget_revision = None;
    Ok(PreparedGripperAction {
        stamp,
        start: snap.commanded_positions.clone(),
        request,
        speed_scale,
        baseline_cmd,
        expected_duration_s,
        timeout_s,
        is_pos_force_close,
        max_budget_s,
        extend_pos_force_close_timeout,
    })
}

/// Off-thread preparation for `/api/v1/action/move_to_pose` from a published [`ActionSnapshot`].
/// Preserves exact commissioning, pose lookup, DOF/profile/joint_names compatibility,
/// `TOOL_MISMATCH` provenance check, gripper hold when `include_gripper == false`, and off-lock
/// quintic + power-fit + workspace sweep planning.
pub fn prepare_saved_pose(
    snap: &ActionSnapshot,
    body: &Value,
) -> Result<PreparedJointAction, ArmError> {
    if !body.is_object() {
        return Err(MotionError::InvalidCommand("Command body must be an object".into()).into());
    }
    let name = body
        .get("name")
        .and_then(Value::as_str)
        .ok_or_else(|| MotionError::InvalidCommand("name must be a string".into()))?;
    if !crate::poses::valid_name(name) {
        return Err(
            MotionError::InvalidCommand("name must match ^[A-Za-z0-9_-]{1,64}$".into()).into(),
        );
    }
    let include_gripper = motion::flag(body, "include_gripper", false)?;
    let speed_scale = crate::poses::speed_scale(body)?;
    let tag = motion::tag(body)?;
    let approach = backlash::parse_request(body)?;

    if snap.env.config.active_tool.commissioning_required {
        return Err(SafetyError::CommissioningRequired(
            "move_to_pose (blocked while active tool requires commissioning; use jog/move_to/move_relative/gripper at speed_scale <= 0.2)".into(),
        )
        .into());
    }
    let pose = snap
        .env
        .poses
        .poses
        .iter()
        .find(|p| p.name == name)
        .ok_or_else(|| ArmError::Rejected {
            status: 404,
            code: "POSE_NOT_FOUND".into(),
            message: format!("No pose named '{name}'"),
        })?;
    let cfg = &snap.env.config;
    let cfg_joint_names: Vec<String> = cfg.limits.iter().map(|l| l.name.clone()).collect();
    let effective_pid = pose.profile_id.as_ref().or(snap.env.poses.profile_id.as_ref());
    if let (Some(pose_pid), Some(ref cfg_pid)) = (effective_pid, &cfg.profile_id) {
        if pose_pid != cfg_pid {
            return Err(ArmError::Rejected {
                status: 400,
                code: "POSE_DOF_MISMATCH".into(),
                message: format!(
                    "Pose '{name}' profile_id '{pose_pid}' does not match active config profile_id '{cfg_pid}'"
                ),
            });
        }
    }
    let effective_arm_dof = pose.arm_dof.or(snap.env.poses.arm_dof);
    if cfg.arm_dof() == 5 && effective_arm_dof.is_none() {
        return Err(ArmError::Rejected {
            status: 400,
            code: "POSE_DOF_MISMATCH".into(),
            message: format!("Pose '{name}' missing arm_dof on 5-DOF profile"),
        });
    }
    if let Some(ad) = effective_arm_dof {
        if ad != cfg.arm_dof() {
            return Err(ArmError::Rejected {
                status: 400,
                code: "POSE_DOF_MISMATCH".into(),
                message: format!(
                    "Pose '{name}' arm_dof {ad} does not match active config arm_dof {}",
                    cfg.arm_dof()
                ),
            });
        }
    }
    if let Some(gj) = pose.gripper_joint {
        if Some(gj) != cfg.gripper_index() {
            return Err(ArmError::Rejected {
                status: 400,
                code: "POSE_DOF_MISMATCH".into(),
                message: format!(
                    "Pose '{name}' gripper_joint {gj} does not match active config gripper_joint {:?}",
                    cfg.gripper_index()
                ),
            });
        }
    }
    if let Some(ref jn) = pose.joint_names {
        if jn != &cfg_joint_names {
            return Err(ArmError::Rejected {
                status: 400,
                code: "POSE_DOF_MISMATCH".into(),
                message: format!(
                    "Pose '{name}' joint_names do not match active config joint_names"
                ),
            });
        }
    }
    if pose.positions.len() != cfg.dof {
        return Err(MotionError::InvalidCommand(format!(
            "positions must have {} entries",
            cfg.dof
        ))
        .into());
    }
    for (&p, l) in pose.positions.iter().zip(&cfg.limits) {
        if !p.is_finite() || p < l.min_position || p > l.max_position {
            return Err(MotionError::InvalidCommand(format!(
                "joint '{}' position {p} outside [{}, {}]",
                l.name, l.min_position, l.max_position
            ))
            .into());
        }
    }
    let gripper_idx = cfg.gripper_index();
    if include_gripper
        && gripper_idx.is_some()
        && !cfg
            .active_tool
            .matches_record(pose.tool_id.as_deref(), pose.tool_config_digest.as_deref())
    {
        return Err(ArmError::Rejected {
            status: 409,
            code: "TOOL_MISMATCH".into(),
            message: format!(
                "Pose '{name}' gripper position was saved with tool {:?} ({:?}), which does not match active tool '{}' ({}); pass include_gripper: false to move arm joints only",
                pose.tool_id,
                pose.tool_config_digest,
                cfg.active_tool.id,
                cfg.active_tool.config_digest
            ),
        });
    }
    snap.check_preconditions()?;
    let mut target = snap.commanded_positions.clone();
    for i in (0..cfg.dof).filter(|i| Some(*i) != gripper_idx) {
        target[i] = pose.positions[i];
    }
    if let (true, Some(g)) = (include_gripper, gripper_idx) {
        target[g] = pose.positions[g];
    }
    let limits = snap.env.planning_limits(Some(speed_scale));
    let zero_v = vec![0.0f32; cfg.dof];
    // Plan quintic + power-fit + swept workspace OFF-LOCK.
    let traj = TrajectoryPlanner::plan(
        &snap.commanded_positions,
        &snap.commanded_velocities,
        &target,
        &zero_v,
        None,
        &limits,
    )?;
    let (traj, power_scale) = fit_quintic_power_off_lock(
        traj,
        &snap.commanded_positions,
        &snap.commanded_velocities,
        &target,
        &limits,
        snap.env.power_budget.as_ref(),
        &cfg.arm_model,
    )?;
    if let Some(ws) = snap.env.workspace_policy.as_ref() {
        check_quintic_workspace_off_lock(
            &traj,
            &snap.commanded_positions,
            ws,
            &cfg.camera_mount,
            &cfg.arm_model,
            cfg.tcp_offset,
        )?;
    }
    let base_duration = traj.duration;
    let prepared_quintic = Some(PreparedQuintic {
        start: snap.commanded_positions.clone(),
        v0: snap.commanded_velocities.clone(),
        target: target.clone(),
        limits: limits.clone(),
        workspace_revision: snap.env.workspace_revision,
        tcp_offset: cfg.tcp_offset,
        power: snap.env.power_budget.clone(),
        checked_workspace: snap.env.workspace_policy.is_some(),
        traj,
        power_scale,
    });
    let use_same_side = !matches!(
        snap.env.approach_plan("move_to_pose", approach),
        Err(None)
    );
    let mut stamp = snap.stamp;
    stamp.parent_generation = 0;
    stamp.sequences_revision = None;
    stamp.grasp_setup_revision = None;
    stamp.gripper_calib_revision = None;
    stamp.calibration_revision = None;
    Ok(PreparedJointAction {
        kind: PreparedActionKind::MoveToPose,
        stamp,
        start: snap.commanded_positions.clone(),
        target,
        limits,
        speed_scale,
        prepared_quintic,
        approach_use: "move_to_pose",
        same_side: use_same_side.then_some(approach),
        tag,
        expected_duration_s: base_duration,
        settle: true,
    })
}

/// Off-thread preparation for `/api/v1/action/replay` from a published [`ActionSnapshot`].
pub fn prepare_replay(
    snap: &ActionSnapshot,
    body: &Value,
) -> Result<PreparedReplayAction, ArmError> {
    if !body.is_object() {
        return Err(MotionError::InvalidCommand("Command body must be an object".into()).into());
    }
    let request = replay::parse_replay(body)?;
    let input = snap.replay_input(request, snap.owner.authority.epoch)?;
    prepare_replay_from_input(snap, input)
}

/// Off-thread preparation from an already-captured [`ReplayInput`].
pub fn prepare_replay_from_input(
    snap: &ActionSnapshot,
    input: ReplayInput,
) -> Result<PreparedReplayAction, ArmError> {
    let plan = replay::plan(input)?;
    let tool_digest = snap.env.tool_digest_hash;
    // Estimate expected duration deterministically from start -> samples[0] approach + recorded / speed + 0.5 s.
    let mut first = plan.samples[0].clone();
    if let (false, Some(g)) = (plan.input.request.gripper, snap.env.config.gripper_index()) {
        if g < first.len() && g < snap.commanded_positions.len() {
            first[g] = snap.commanded_positions[g];
        }
    }
    let limits = snap.env.planning_limits(None);
    let zeros = vec![0.0f32; snap.env.config.dof];
    let approach_s = TrajectoryPlanner::plan(
        &snap.commanded_positions,
        &snap.commanded_velocities,
        &first,
        &zeros,
        None,
        &limits,
    )
    .map(|t| t.duration)
    .unwrap_or(0.0);
    let eff_speed = plan.input.request.speed * plan.power_scale.unwrap_or(1.0);
    let expected_duration_s = approach_s + plan.duration / eff_speed.max(1e-6) + 0.5;
    let mut stamp = snap.stamp;
    stamp.parent_generation = 0;
    stamp.poses_revision = None;
    stamp.sequences_revision = None;
    stamp.grasp_setup_revision = None;
    stamp.gripper_calib_revision = None;
    stamp.calibration_revision = None;
    Ok(PreparedReplayAction {
        stamp,
        start: snap.commanded_positions.clone(),
        plan,
        tool_digest,
        expected_duration_s,
    })
}

/// Result of off-thread `prepare_move_to` / `prepare_move_relative`.
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)]
pub enum PreparedMoveAction {
    Cartesian(PreparedCartesianAction),
    Joint(PreparedJointAction),
}

impl PreparedMoveAction {
    pub fn kind(&self) -> PreparedActionKind {
        match self {
            Self::Cartesian(c) => c.kind,
            Self::Joint(j) => j.kind,
        }
    }

    pub fn expected_duration_s(&self) -> f64 {
        match self {
            Self::Cartesian(c) => c.expected_duration_s,
            Self::Joint(j) => j.expected_duration_s,
        }
    }

    pub fn to_plan_slot(&self, child: Option<ChildSpec>) -> Result<PlanSlot, &'static str> {
        match self {
            Self::Cartesian(c) => c.to_plan_slot(child),
            Self::Joint(j) => j.to_plan_slot(child),
        }
    }
}

/// Off-thread preparation for `/api/v1/action/move_to` from a published [`ActionSnapshot`].
pub fn prepare_move_to(
    snap: &ActionSnapshot,
    body: &Value,
) -> Result<PreparedMoveAction, ArmError> {
    prepare_move_with_branch_hint(snap, body, false).map_err(|(e, _)| e)
}

/// Off-thread preparation for `/api/v1/action/move_relative` from a published [`ActionSnapshot`].
pub fn prepare_move_relative(
    snap: &ActionSnapshot,
    body: &Value,
) -> Result<PreparedMoveAction, ArmError> {
    prepare_move_with_branch_hint(snap, body, true).map_err(|(e, _)| e)
}

/// Off-thread preparation for `move_to` / `move_relative`, additionally reporting whether
/// `path: "joint"` is feasible when a straight line fails with `IK_BRANCH_FLIP` (so WS can
/// append `JOINT_PATH_HINT` without taking `SupervisorLock`).
pub fn prepare_move_with_branch_hint(
    snap: &ActionSnapshot,
    body: &Value,
    relative: bool,
) -> Result<PreparedMoveAction, (ArmError, bool)> {
    prepare_move_common(snap, body, relative)
}

fn joint_goal_feasible_off_lock(snap: &ActionSnapshot, q: &[f64; 6]) -> bool {
    let target = crate::supervisor::joint_goal_target_from(
        &snap.commanded_positions,
        q,
        snap.env.config.arm_dof(),
        snap.env.config.gripper_index(),
    );
    let limits = snap.env.planning_limits(None);
    if target
        .iter()
        .zip(&snap.env.config.limits)
        .any(|(t, l)| !t.is_finite() || *t < l.min_position || *t > l.max_position)
    {
        return false;
    }
    let zeros = vec![0.0f32; snap.env.config.dof];
    let Ok(traj) = TrajectoryPlanner::plan(
        &snap.commanded_positions,
        &zeros,
        &target,
        &zeros,
        None,
        &limits,
    ) else {
        return false;
    };
    if let Some(ws) = snap.env.workspace_policy.as_ref() {
        check_quintic_workspace_off_lock(
            &traj,
            &snap.commanded_positions,
            ws,
            &snap.env.config.camera_mount,
            &snap.env.config.arm_model,
            snap.env.config.tcp_offset,
        )
        .is_ok()
    } else {
        true
    }
}

fn prepare_move_common(
    snap: &ActionSnapshot,
    body: &Value,
    relative: bool,
) -> Result<PreparedMoveAction, (ArmError, bool)> {
    let no_hint = |e: ArmError| (e, false);
    if !body.is_object() {
        return Err(no_hint(
            MotionError::InvalidCommand("Command body must be an object".into()).into(),
        ));
    }
    let kind_name = if relative { "move_relative" } else { "move_to" };
    let (spec, mut speed, tag) = if relative {
        motion::parse_move_relative(body)
            .map_err(ArmError::from)
            .map_err(no_hint)?
    } else {
        motion::parse_move_to(body)
            .map_err(ArmError::from)
            .map_err(no_hint)?
    };
    let precise = motion::flag(body, "precise", false)
        .map_err(ArmError::from)
        .map_err(no_hint)?;
    let request = backlash::parse_request(body)
        .map_err(ArmError::from)
        .map_err(no_hint)?;
    let path_mode = motion::parse_path_mode(body)
        .map_err(ArmError::from)
        .map_err(no_hint)?;
    let same_side = precise || request != backlash::Request::Profile;
    if path_mode == PathMode::Joint && body.get("speed").is_some_and(|v| !v.is_null()) {
        return Err(no_hint(
            MotionError::InvalidCommand(
                "speed applies to path \"linear\"; path \"joint\" takes speed_scale (0.1..1.0)"
                    .into(),
            )
            .into(),
        ));
    }
    let linear_speed_scale = if path_mode == PathMode::Linear {
        if let Some(s) = motion::num(body, "speed_scale")
            .map_err(ArmError::from)
            .map_err(no_hint)?
        {
            if !(0.05..=1.0).contains(&s) {
                return Err(no_hint(
                    MotionError::InvalidCommand("speed_scale must be in [0.05, 1.0]".into()).into(),
                ));
            }
            if body.get("speed").is_none_or(Value::is_null) {
                speed.linear = (motion::MAX_LINEAR_MPS * s).min(motion::MAX_LINEAR_MPS);
                speed.angular = (motion::MAX_ANGULAR_RPS * s).min(motion::MAX_ANGULAR_RPS);
            }
            Some(s)
        } else {
            None
        }
    } else {
        None
    };
    let input = snap.plan_input_in(GoalFrame::Tcp).map_err(no_hint)?;
    let (goal, opts) = motion::resolve_goal_opts(&input, &spec);
    let seeds = motion::goal_seeds(&input, &goal, &snap.env.goal_seed_poses());
    let mut stamp = snap.stamp;
    stamp.parent_generation = 0;
    stamp.poses_revision = None;
    stamp.sequences_revision = None;
    stamp.grasp_setup_revision = None;
    stamp.gripper_calib_revision = None;
    stamp.calibration_revision = None;

    match path_mode {
        PathMode::Linear => {
            if snap.env.config.active_tool.commissioning_required {
                let scale_ok = linear_speed_scale.is_none_or(|s| s <= 0.2 + 1e-6);
                let speed_ok = speed.linear <= 0.2 * motion::MAX_LINEAR_MPS + 1e-6
                    && speed.angular <= 0.2 * motion::MAX_ANGULAR_RPS + 1e-6;
                if !scale_ok || !speed_ok {
                    return Err(no_hint(
                        SafetyError::CommissioningRequired(format!(
                            "{kind_name} (requires speed_scale <= 0.2 or speed <= {:.3} m/s / {:.2} rad/s)",
                            0.2 * motion::MAX_LINEAR_MPS,
                            0.2 * motion::MAX_ANGULAR_RPS
                        ))
                        .into(),
                    ));
                }
            }
            let approach = if same_side {
                snap.env.approach_plan("move_to_precise", request)
            } else {
                Err(None)
            };
            let direct = match motion::plan_cartesian_opts(&input, goal, speed, opts) {
                Ok(d) => d,
                Err(e) => {
                    let hint = matches!(&e, MotionError::IkBranchFlip { .. })
                        && motion::solve_goal_joints(&input, &goal, opts, &seeds)
                            .ok()
                            .is_some_and(|q| joint_goal_feasible_off_lock(snap, &q));
                    return Err((ArmError::from(e), hint));
                }
            };
            let pre = match &approach {
                Ok(plan) => backlash::plan_cartesian_pre(&input, &direct, plan, None, speed, opts),
                Err(Some(r)) => backlash::Pre::Direct((*r).to_string()),
                Err(None) => backlash::Pre::Off,
            };
            let expected_duration_s = direct.duration + 0.5;
            Ok(PreparedMoveAction::Cartesian(PreparedCartesianAction {
                kind: if relative {
                    PreparedActionKind::MoveRelativeLinear
                } else {
                    PreparedActionKind::MoveToLinear
                },
                stamp,
                start: snap.commanded_positions.clone(),
                input,
                direct,
                pre,
                approach_plan: approach.ok(),
                rules: None,
                tag,
                expected_duration_s,
            }))
        }
        PathMode::Joint => {
            let speed_scale = crate::poses::speed_scale(body).map_err(no_hint)?;
            if snap.env.config.active_tool.commissioning_required && speed_scale > 0.2 + 1e-6 {
                return Err(no_hint(
                    SafetyError::CommissioningRequired(format!(
                        "{kind_name} (speed_scale {speed_scale:.2} > 0.2)"
                    ))
                    .into(),
                ));
            }
            let q = motion::solve_goal_joints(&input, &goal, opts, &seeds)
                .map_err(ArmError::from)
                .map_err(no_hint)?;
            let jsnap = snap.joint_goal_snapshot(speed_scale);
            let target = crate::supervisor::joint_goal_target_from(
                &jsnap.start,
                &q,
                jsnap.arm_dof,
                jsnap.gripper_index,
            );
            let prepared_quintic = if same_side {
                None
            } else {
                crate::supervisor::prepare_quintic(&input, &jsnap, &q)
            };
            let expected_duration_s = prepared_quintic.as_ref().map_or_else(
                || {
                    let zeros = vec![0.0f32; snap.env.config.dof];
                    TrajectoryPlanner::plan(
                        &jsnap.start,
                        &jsnap.v0,
                        &target,
                        &zeros,
                        None,
                        &jsnap.limits,
                    )
                    .map_or(1.0, |t| t.duration)
                },
                |p| p.traj.duration,
            );
            Ok(PreparedMoveAction::Joint(PreparedJointAction {
                kind: if relative {
                    PreparedActionKind::MoveRelativeJoint
                } else {
                    PreparedActionKind::MoveToJoint
                },
                stamp,
                start: snap.commanded_positions.clone(),
                target,
                limits: jsnap.limits,
                speed_scale,
                prepared_quintic,
                approach_use: "move_to_precise",
                same_side: same_side.then_some(request),
                tag,
                expected_duration_s,
                settle: false,
            }))
        }
    }
}

/// Off-thread helper: stage a prepared Cartesian macro child into a [`PlanSlot`].
pub fn prepare_child_cartesian_slot(
    start: &[f32],
    path: &JointPath,
    power_scale: Option<f64>,
    child: ChildSpec,
) -> Result<PlanSlot, &'static str> {
    prepare_child_cartesian_slot_with_approach(start, path, power_scale, None, child)
}

/// Off-thread helper: stage a prepared Cartesian macro child with an optional second-leg
/// same-side approach quintic into a [`PlanSlot`].
pub fn prepare_child_cartesian_slot_with_approach(
    start: &[f32],
    path: &JointPath,
    power_scale: Option<f64>,
    approach_then: Option<StagedQuinticLeg>,
    child: ChildSpec,
) -> Result<PlanSlot, &'static str> {
    if start.len() != path.dof || start.len() > MAX_DOF {
        return Err("INVALID_COMMAND");
    }
    let mut slot = PlanSlot::default();
    for (q, &s) in path.q.iter().zip(&path.s) {
        slot.path.push(&q[..path.dof], s)?;
    }
    let mut start_arr = [0.0f32; MAX_DOF];
    start_arr[..start.len()].copy_from_slice(start);
    slot.primitive = Primitive::Action(StagedAction::Cartesian {
        child: Some(child),
        kind: PreparedActionKind::MoveToLinear,
        dof: path.dof as u8,
        start: start_arr,
        duration_s: path.duration,
        power_scale,
        approach_then,
    });
    Ok(slot)
}

/// Off-thread helper: stage a prepared joint-quintic macro child into a [`PlanSlot`].
pub fn prepare_child_joint_slot(
    start: &[f32],
    target: &[f32],
    max_velocity: &[f32],
    duration_s: f64,
    power_scale: Option<f64>,
    child: ChildSpec,
) -> Result<PlanSlot, &'static str> {
    let dof = start.len();
    if dof == 0 || dof > MAX_DOF || target.len() != dof || max_velocity.len() != dof {
        return Err("INVALID_COMMAND");
    }
    let mut s = [0.0f32; MAX_DOF];
    let mut t = [0.0f32; MAX_DOF];
    let mut mv = [0.0f32; MAX_DOF];
    s[..dof].copy_from_slice(start);
    t[..dof].copy_from_slice(target);
    mv[..dof].copy_from_slice(max_velocity);
    let mut slot = PlanSlot::default();
    slot.set(Primitive::Action(StagedAction::Joint {
        child: Some(child),
        kind: PreparedActionKind::MoveToPose,
        dof: dof as u8,
        start: s,
        target: t,
        duration_s,
        max_velocity: mv,
        power_scale,
        settle: true,
        approach_then: None,
    }));
    Ok(slot)
}

/// Off-thread helper: stage a prepared gripper macro child into a [`PlanSlot`].
#[allow(clippy::too_many_arguments)]
pub fn prepare_child_gripper_slot(
    start: &[f32],
    command: GripperCommand,
    fraction: Option<f64>,
    grasp: bool,
    speed: f64,
    timeout_s: Option<f64>,
    max_budget_s: Option<f64>,
    extend_pos_force_close_timeout: bool,
    child: ChildSpec,
) -> Result<PlanSlot, &'static str> {
    let dof = start.len();
    if dof == 0 || dof > MAX_DOF {
        return Err("INVALID_COMMAND");
    }
    let mut s = [0.0f32; MAX_DOF];
    s[..dof].copy_from_slice(start);
    let mut slot = PlanSlot::default();
    slot.set(Primitive::Action(StagedAction::Gripper {
        child: Some(child),
        dof: dof as u8,
        start: s,
        command: command.into(),
        fraction,
        grasp,
        speed,
        timeout_s,
        max_budget_s,
        extend_pos_force_close_timeout,
        speed_scale: None,
    }));
    Ok(slot)
}

/// Off-thread helper: stage a prepared replay macro child into a [`PlanSlot`].
pub fn prepare_child_replay_slot(
    start: &[f32],
    plan: &ReplayPlan,
    tool_digest: u64,
    child: ChildSpec,
) -> Result<PlanSlot, &'static str> {
    let dof = start.len();
    if dof == 0 || dof > MAX_DOF {
        return Err("INVALID_COMMAND");
    }
    let episode = EpisodeRef::new(&plan.input.request.episode).ok_or("INVALID_COMMAND")?;
    let mut slot = PlanSlot::default();
    for (i, sample) in plan.samples.iter().enumerate() {
        slot.path.push(sample, i as f64)?;
    }
    let mut s = [0.0f32; MAX_DOF];
    s[..dof].copy_from_slice(start);
    slot.primitive = Primitive::Action(StagedAction::Replay {
        child: Some(child),
        dof: dof as u8,
        start: s,
        episode,
        speed: plan.input.request.speed as f32,
        gripper: plan.input.request.gripper,
        sample_dt: plan.sample_dt,
        duration_s: plan.duration,
        power_scale: plan.power_scale,
        tool_digest,
    });
    Ok(slot)
}

fn fit_quintic_power_off_lock(
    traj: crate::trajectory::MultiJointQuinticTrajectory,
    start: &[f32],
    v0: &[f32],
    target: &[f32],
    limits: &[JointLimit],
    budget: Option<&PowerBudget>,
    arm: &crate::chain::ArmModel,
) -> Result<(crate::trajectory::MultiJointQuinticTrajectory, Option<f64>), ArmError> {
    crate::power::power_fit_quintic_with(
        budget,
        arm,
        arm.arm_dof(),
        traj,
        start,
        v0,
        target,
        limits,
        true,
    )
}

fn check_quintic_workspace_off_lock(
    traj: &crate::trajectory::MultiJointQuinticTrajectory,
    start: &[f32],
    ws: &WorkspacePolicy,
    cam: &crate::model::CameraMount,
    arm: &crate::chain::ArmModel,
    tcp_offset: [f64; 3],
) -> Result<(), ArmError> {
    let knots: Vec<f64> = (0..=20).map(|k| traj.duration * k as f64 / 20.0).collect();
    let at = |t: f64| traj.sample(t).positions;
    let travel = |t0: f64, t1: f64| traj.travel_bound(t0, t1);
    let eps = crate::kinematics::WORKSPACE_EPS;
    let samples = crate::supervisor::keepout_sweep_with_tcp_tol(
        arm,
        ws,
        cam,
        tcp_offset,
        eps,
        &at,
        &travel,
        &knots,
        crate::supervisor::MAX_KEEPOUT_EVALS_PER_PLAN,
    )?;
    crate::supervisor::check_path_with_tcp_tol(
        arm,
        ws,
        cam,
        tcp_offset,
        eps,
        start,
        samples.iter().map(Vec::as_slice),
    )?;
    Ok(())
}

// ---------------------------------------------------------------------------------------
// ONE owner-side install function per request kind (7 functions, Legacy + RT Executor)

/// Outcome of a standalone install (`ActionId`, expected duration s).
pub type InstalledAction = (u64, f64);

/// **1. ONE owner-side install for linear Cartesian moves** (`move_to`, `move_relative`).
/// Legacy passes `PreparedCartesianAction`; RT Executor passes a pinned `PlanSlotRef`.
pub fn install_move_cartesian(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    payload: CartesianInstallPayload,
) -> Result<InstalledAction, ArmError> {
    check_action_authority(inner, stamp, ctx, true, true)
        .map_err(|r| r.into_arm_error(inner))?;
    match payload {
        CartesianInstallPayload::Legacy(prep) => {
            check_start_fresh(&inner.commanded_positions, &prep.start)
                .map_err(|r| r.into_arm_error(inner))?;
            let epoch = inner.safety.epoch_id();
            let admitted = consume_action_admission(inner, stamp, ctx.stop_generation)
                .map_err(|r| r.into_arm_error_with_epoch(epoch))?;
            SupervisorInner::install_path_same_side_with_rules(
                admitted,
                &prep.input,
                prep.direct,
                prep.pre,
                prep.approach_plan.as_ref(),
                prep.rules.as_ref(),
                prep.kind.action_kind_str(),
                prep.tag,
            )
        }
        CartesianInstallPayload::Slot {
            kind,
            dof,
            start,
            slot,
            duration_s,
            power_scale,
            approach_then,
        } => install_move_cartesian_slot_rt(
            inner,
            stamp,
            ctx,
            kind,
            dof,
            &start,
            slot,
            duration_s,
            power_scale,
            approach_then,
        )
        .map_err(|code| ActionRefusal::new(code).into_arm_error(inner)),
    }
}

/// Payload for [`install_move_cartesian`].
pub enum CartesianInstallPayload {
    Legacy(Box<PreparedCartesianAction>),
    Slot {
        kind: PreparedActionKind,
        dof: usize,
        start: [f32; MAX_DOF],
        slot: PlanSlotRef<PlanSlot>,
        duration_s: f64,
        power_scale: Option<f64>,
        approach_then: Option<StagedQuinticLeg>,
    },
}

/// Single zero-alloc RT validator for an optional second-leg same-side approach quintic
/// (shared by `install_move_cartesian_slot_rt`, `install_move_joint_rt`, and
/// `install_macro_child_slot_rt`): enforces finite duration/target, safety motion validation,
/// and inline swept-workspace keepout BEFORE admission consumption.
fn validate_approach_leg_rt(
    inner: &mut SupervisorInner,
    epoch: u64,
    dof: usize,
    leg_start: &[f32],
    leg: StagedQuinticLeg,
) -> Result<(crate::trajectory::MultiJointQuinticTrajectory, StagedQuinticLeg), &'static str> {
    if !leg.duration_s.is_finite()
        || leg.duration_s <= 0.0
        || leg.target[..dof].iter().any(|v| !v.is_finite())
    {
        return Err("INVALID_COMMAND");
    }
    inner
        .safety
        .validate_motion_command(Some(epoch), &leg.target[..dof], &inner.config.limits)
        .map_err(|e| static_code(ArmError::Safety(e)))?;
    let zeros = [0.0f32; MAX_DOF];
    let seg = crate::trajectory::MultiJointQuinticTrajectory::new(
        &leg_start[..dof],
        &zeros[..dof],
        &leg.target[..dof],
        &zeros[..dof],
        leg.duration_s,
    )
    .map_err(|_| "INVALID_COMMAND")?;
    inner
        .check_workspace_quintic_inline(&seg)
        .map_err(|e| static_code(ArmError::Safety(e)))?;
    Ok((seg, leg))
}

#[allow(clippy::too_many_arguments)]
fn install_move_cartesian_slot_rt(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    kind: PreparedActionKind,
    dof: usize,
    start: &[f32; MAX_DOF],
    slot: PlanSlotRef<PlanSlot>,
    duration_s: f64,
    power_scale: Option<f64>,
    approach_then: Option<StagedQuinticLeg>,
) -> Result<InstalledAction, &'static str> {
    if dof == 0 || dof > MAX_DOF {
        return Err("INVALID_COMMAND");
    }
    let mut post_q = [0.0f32; MAX_DOF];
    let (post_dof, _) = inner.post_quiesce_commanded_positions(&mut post_q);
    check_start_fresh(&post_q[..post_dof], &start[..dof]).map_err(|r| r.code)?;
    inner.validate_prepared_joint_path(dof, &slot.path, duration_s)?;
    if inner.gravity_comp.is_some() {
        return Err("MOTION_BUSY");
    }
    inner.check_prepared_fresh_static(stamp.admission.estop_epoch)?;
    let last_slice = &slot.path.samples()[slot.path.len() - 1][..dof];
    let mut last_buf = [0.0f32; MAX_DOF];
    last_buf[..dof].copy_from_slice(last_slice);
    let then_traj = match approach_then {
        Some(leg) => Some(validate_approach_leg_rt(
            inner,
            stamp.admission.estop_epoch,
            dof,
            &last_buf[..dof],
            leg,
        )?),
        None => None,
    };
    let admitted = consume_action_admission(inner, stamp, ctx.stop_generation).map_err(|r| r.code)?;
    let (this, proof) = admitted.consume().map_err(|e| static_code(e.into()))?;
    this.check_terminal_fifo_for_new_action()
        .map_err(|e| static_code(e.into()))?;
    if let Some(old_id) = this.macro_run.as_ref().map(|m| m.id) {
        let _ = this.quiesce_macro_children_rt(old_id);
    }
    let duration = this
        .start_path_motion_slice_inner(&proof, stamp.admission.estop_epoch, &last_buf[..dof], duration_s)
        .map_err(static_code)?;
    let id = this.begin_action_rt(kind.action_kind_str());
    if let Some(s) = power_scale {
        this.note_action_power_scale(id, s);
    }
    let arm_dof = this.config.arm_dof();
    let base_ref = Reference::Path(motion::PathTrajectory::from_plan_slot(slot, dof, arm_dof, duration_s));
    let total_dur = if let Some((seg, leg)) = then_traj {
        let d2 = seg.duration;
        if let Some(s) = leg.power_scale {
            this.note_action_power_scale(id, s);
        }
        crate::rt_core::scratch::copy_into(&mut this.target_positions, &leg.target[..dof]);
        this.replace_active_trajectory(Some(ActiveTrajectory::with_approach(
            Some(id),
            base_ref,
            seg,
        )));
        if let Some(a) = this.last_action.as_mut().filter(|a| a.id == id) {
            a.phase = Some(HolderLabel::new("move"));
        }
        duration + 0.5 + d2
    } else {
        this.replace_active_trajectory(Some(ActiveTrajectory::new(Some(id), base_ref)));
        duration + 0.5
    };
    Ok((id, total_dur))
}

/// **2. ONE owner-side install for joint moves** (`move_to`/`move_relative` `path: "joint"`,
/// `move_to_pose` saved pose).
pub fn install_move_joint(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    payload: JointInstallPayload,
) -> Result<InstalledAction, ArmError> {
    check_action_authority(inner, stamp, ctx, true, true)
        .map_err(|r| r.into_arm_error(inner))?;
    match payload {
        JointInstallPayload::Legacy(prep) => {
            check_start_fresh(&inner.commanded_positions, &prep.start)
                .map_err(|r| r.into_arm_error(inner))?;
            if prep.kind == PreparedActionKind::MoveToPose {
                inner.check_commissioning_for_ordinary_motion("move_to_pose")?;
            }
            inner.check_prepared_fresh(stamp.admission.estop_epoch)?;
            let epoch = inner.safety.epoch_id();
            let admitted = consume_action_admission(inner, stamp, ctx.stop_generation)
                .map_err(|r| r.into_arm_error_with_epoch(epoch))?;
            let (this, proof) = admitted.consume()?;
            let (_, id, duration) = match prep.same_side {
                Some(req) => this.start_quintic_same_side(
                    &proof,
                    prep.target,
                    Some(stamp.admission.estop_epoch),
                    prep.kind.action_kind_str(),
                    prep.tag,
                    &prep.limits,
                    prep.approach_use,
                    req,
                )?,
                None => this.start_quintic_prepared(
                    &proof,
                    prep.target,
                    None,
                    Some(stamp.admission.estop_epoch),
                    prep.kind.action_kind_str(),
                    prep.tag,
                    &prep.limits,
                    prep.prepared_quintic,
                )?,
            };
            Ok((id, duration))
        }
        JointInstallPayload::Rt {
            kind,
            dof,
            start,
            target,
            duration_s,
            max_velocity,
            power_scale,
            settle,
            approach_then,
        } => install_move_joint_rt(
            inner,
            stamp,
            ctx,
            kind,
            dof,
            &start,
            &target,
            duration_s,
            max_velocity,
            power_scale,
            settle,
            approach_then,
        )
        .map_err(|code| ActionRefusal::new(code).into_arm_error(inner)),
    }
}

/// Payload for [`install_move_joint`].
pub enum JointInstallPayload {
    Legacy(Box<PreparedJointAction>),
    Rt {
        kind: PreparedActionKind,
        dof: usize,
        start: [f32; MAX_DOF],
        target: [f32; MAX_DOF],
        duration_s: f64,
        max_velocity: [f32; MAX_DOF],
        power_scale: Option<f64>,
        settle: bool,
        approach_then: Option<StagedQuinticLeg>,
    },
}

#[allow(clippy::too_many_arguments)]
fn install_move_joint_rt(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    kind: PreparedActionKind,
    dof: usize,
    start: &[f32; MAX_DOF],
    target: &[f32; MAX_DOF],
    duration_s: f64,
    max_velocity: [f32; MAX_DOF],
    power_scale: Option<f64>,
    settle: bool,
    approach_then: Option<StagedQuinticLeg>,
) -> Result<InstalledAction, &'static str> {
    if dof == 0 || dof > MAX_DOF || !duration_s.is_finite() || duration_s <= 0.0 {
        return Err("INVALID_COMMAND");
    }
    if target[..dof].iter().any(|v| !v.is_finite()) {
        return Err("INVALID_COMMAND");
    }
    let mut post_q = [0.0f32; MAX_DOF];
    let (post_dof, quiesce_macro_gripper) = inner.post_quiesce_commanded_positions(&mut post_q);
    check_start_fresh(&post_q[..post_dof], &start[..dof]).map_err(|r| r.code)?;
    if inner.config.active_tool.commissioning_required && kind == PreparedActionKind::MoveToPose {
        return Err("COMMISSIONING_REQUIRED");
    }
    if inner.gravity_comp.is_some() {
        return Err("MOTION_BUSY");
    }
    inner.check_prepared_fresh_static(stamp.admission.estop_epoch)?;
    let (mut limits_buf, lim_n) = inner
        .planning_limits_inline(None)
        .ok_or("INVALID_COMMAND")?;
    if lim_n != dof {
        return Err("INVALID_COMMAND");
    }
    for i in 0..dof {
        if max_velocity[i].is_finite() && max_velocity[i] > 0.0 {
            limits_buf[i].max_velocity = limits_buf[i].max_velocity.min(max_velocity[i]);
        }
    }
    let mut tgt = [0.0f32; MAX_DOF];
    tgt[..dof].copy_from_slice(&target[..dof]);
    inner
        .safety
        .validate_motion_command(Some(stamp.admission.estop_epoch), &tgt[..dof], &inner.config.limits)
        .map_err(|e| static_code(ArmError::Safety(e)))?;
    inner.torque_hold_filter(&mut tgt[..dof]);
    inner.power_hold_filter(&mut tgt[..dof]);
    let zeros = [0.0f32; MAX_DOF];
    let mut start_vel = [0.0f32; MAX_DOF];
    start_vel[..dof].copy_from_slice(&inner.commanded_velocities[..dof]);
    if quiesce_macro_gripper {
        if let Some(g) = inner.config.gripper_index().filter(|&g| g < dof) {
            start_vel[g] = 0.0;
        }
    }
    // Install with exact prepared timing verbatim (preserves ACK duration parity even within 1e-4 start drift).
    let traj = crate::trajectory::MultiJointQuinticTrajectory::new(
        &post_q[..dof],
        &start_vel[..dof],
        &tgt[..dof],
        &zeros[..dof],
        duration_s,
    )
    .map_err(|_| "INVALID_COMMAND")?;
    inner
        .check_workspace_quintic_inline(&traj)
        .map_err(|e| static_code(ArmError::Safety(e)))?;
    let then_traj = match approach_then {
        Some(leg) => Some(validate_approach_leg_rt(
            inner,
            stamp.admission.estop_epoch,
            dof,
            &tgt[..dof],
            leg,
        )?),
        None => None,
    };

    let admitted = consume_action_admission(inner, stamp, ctx.stop_generation).map_err(|r| r.code)?;
    let (this, _proof) = admitted.consume().map_err(|e| static_code(e.into()))?;
    this.check_terminal_fifo_for_new_action()
        .map_err(|e| static_code(e.into()))?;
    if let Some(old_id) = this.macro_run.as_ref().map(|m| m.id) {
        let _ = this.quiesce_macro_children_rt(old_id);
    }
    if this
        .config
        .gripper_index()
        .is_some_and(|g| g < dof && (tgt[g] - this.commanded_positions[g]).abs() > 1e-4)
    {
        this.gripper_hold_vel_ceiling = None;
    }
    this.release_servo_target();
    this.servo_settled_ticks.fill(0);
    this.servo_ws_min_depths = None;
    crate::rt_core::scratch::copy_into(&mut this.target_positions, &tgt[..dof]);
    this.safety
        .mark_motion_started()
        .map_err(|e| static_code(ArmError::Safety(e)))?;
    this.last_heartbeat = ctx.now;
    let id = this.begin_action_rt(kind.action_kind_str());
    if let Some(s) = power_scale {
        this.note_action_power_scale(id, s);
    }
    let mut caps = [0.0f32; MAX_DOF];
    for i in 0..dof {
        caps[i] = limits_buf[i].max_velocity;
    }
    let _ = settle;
    let total_dur = if let Some((seg, leg)) = then_traj {
        let d2 = seg.duration;
        if let Some(s) = leg.power_scale {
            this.note_action_power_scale(id, s);
        }
        crate::rt_core::scratch::copy_into(&mut this.target_positions, &leg.target[..dof]);
        let active = ActiveTrajectory::with_approach(Some(id), Reference::Quintic(traj), seg)
            .with_motion_max_velocity(caps);
        this.replace_active_trajectory(Some(active));
        if let Some(a) = this.last_action.as_mut().filter(|a| a.id == id) {
            a.phase = Some(HolderLabel::new("move"));
        }
        duration_s + d2
    } else {
        let active = ActiveTrajectory::quintic(traj, Some(id)).with_motion_max_velocity(caps);
        this.replace_active_trajectory(Some(active));
        duration_s
    };
    Ok((id, total_dur))
}

/// **3. ONE owner-side install for standalone gripper actions** (`gripper`).
pub fn install_gripper(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    payload: GripperInstallPayload,
) -> Result<InstalledAction, ArmError> {
    check_action_authority(inner, stamp, ctx, false, true)
        .map_err(|r| r.into_arm_error(inner))?;
    match payload {
        GripperInstallPayload::Legacy(prep) => {
            check_start_fresh(&inner.commanded_positions, &prep.start)
                .map_err(|r| r.into_arm_error(inner))?;
            if let Some(scale) = prep.speed_scale {
                if inner.config.active_tool.commissioning_required && scale > 0.2 + 1e-6 {
                    return Err(SafetyError::CommissioningRequired(format!(
                        "gripper (speed_scale {scale:.2} > 0.2)"
                    ))
                    .into());
                }
            }
            inner.check_prepared_fresh(stamp.admission.estop_epoch)?;
            let epoch = inner.safety.epoch_id();
            let admitted = consume_action_admission(inner, stamp, ctx.stop_generation)
                .map_err(|r| r.into_arm_error_with_epoch(epoch))?;
            SupervisorInner::start_gripper_with_baseline(
                admitted,
                prep.request,
                stamp.admission.estop_epoch,
                prep.baseline_cmd,
            )
        }
        GripperInstallPayload::Rt {
            dof,
            start,
            command,
            fraction,
            grasp,
            speed,
            timeout_s,
            speed_scale,
        } => install_gripper_rt(
            inner,
            stamp,
            ctx,
            dof,
            &start,
            command,
            fraction,
            grasp,
            speed,
            timeout_s,
            speed_scale,
        )
        .map_err(|code| ActionRefusal::new(code).into_arm_error(inner)),
    }
}

/// Payload for [`install_gripper`].
pub enum GripperInstallPayload {
    Legacy(Box<PreparedGripperAction>),
    Rt {
        dof: usize,
        start: [f32; MAX_DOF],
        command: GripperCommandKind,
        fraction: Option<f64>,
        grasp: bool,
        speed: f64,
        timeout_s: Option<f64>,
        speed_scale: Option<f64>,
    },
}

#[allow(clippy::too_many_arguments)]
fn install_gripper_rt(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    dof: usize,
    start: &[f32; MAX_DOF],
    command: GripperCommandKind,
    fraction: Option<f64>,
    grasp: bool,
    speed: f64,
    timeout_s: Option<f64>,
    speed_scale: Option<f64>,
) -> Result<InstalledAction, &'static str> {
    if dof == 0 || dof > MAX_DOF || !speed.is_finite() || speed <= 0.0 {
        return Err("INVALID_COMMAND");
    }
    let mut post_q = [0.0f32; MAX_DOF];
    let (post_dof, _) = inner.post_quiesce_commanded_positions(&mut post_q);
    check_start_fresh(&post_q[..post_dof], &start[..dof]).map_err(|r| r.code)?;
    let gripper_idx = inner.config.gripper_index().ok_or("UNSUPPORTED")?;
    let policy = inner
        .config
        .gripper_policy
        .as_ref()
        .ok_or("UNSUPPORTED")?
        .clone_without_jaw_opening();
    if inner.config.active_tool.commissioning_required
        && (speed_scale.is_some_and(|s| s > 0.2 + 1e-6) || speed > 0.4 + 1e-6)
    {
        return Err("COMMISSIONING_REQUIRED");
    }
    inner.check_prepared_fresh_static(stamp.admission.estop_epoch)?;
    let request = GripperRequest {
        command: command.into(),
        fraction,
        grasp,
        speed,
        timeout_s,
        tag: None,
    };
    if policy.max_close_torque_nm.is_some() {
        inner.check_gripper_close_torque_feedback_rt(
            request.command,
            request.fraction,
            &policy,
            gripper_idx,
        )?;
    }
    let close_cap = inner.gripper_close_speed_limit();
    let ceiling_est = inner.gripper_ceiling_torque_estimate();
    let lim = &inner.config.limits[gripper_idx];
    let limit_zero_alloc = JointLimit {
        name: String::new(),
        min_position: lim.min_position,
        max_position: lim.max_position,
        max_velocity: lim.max_velocity,
        max_acceleration: lim.max_acceleration,
    };
    let baseline_data = inner.gripper_baseline_data_rt();
    let mut motion = GripperMotion::try_with_close_speed_limit_static(
        request,
        policy,
        limit_zero_alloc,
        post_q[gripper_idx],
        gripper_idx,
        close_cap,
    )?
    .with_ceiling_torque_estimate(ceiling_est);
    let learn_samples = if motion.will_record_learning(LearnMode::Opening) {
        inner.take_move_samples_spare_rt()
    } else {
        None
    };
    motion = motion.with_baseline_data_and_samples(baseline_data, LearnMode::Opening, learn_samples);
    let duration = motion.expected_duration();
    let target_g = motion.target;
    let is_open = matches!(motion.request.command, GripperCommand::Open);
    let wire_vel_ceiling = motion.wire_velocity_ceiling();

    let admitted = consume_action_admission(inner, stamp, ctx.stop_generation).map_err(|r| r.code)?;
    let (this, _proof) = admitted.consume().map_err(|e| static_code(e.into()))?;
    this.check_terminal_fifo_for_new_action()
        .map_err(|e| static_code(e.into()))?;
    if let Some(old_id) = this.macro_run.as_ref().map(|m| m.id) {
        let _ = this.quiesce_macro_children_rt(old_id);
    }
    this.safety
        .mark_motion_started()
        .map_err(|e| static_code(ArmError::Safety(e)))?;
    this.releasing = is_open;
    this.gripper_hold_vel_ceiling = Some(wire_vel_ceiling);
    this.release_servo_target();
    this.servo_settled_ticks.fill(0);
    this.servo_ws_min_depths = None;
    crate::rt_core::scratch::copy_into(&mut this.target_positions, &this.commanded_positions);
    if gripper_idx < this.target_positions.len() {
        this.target_positions[gripper_idx] = target_g;
    }
    this.last_heartbeat = ctx.now;
    let id = this.begin_action_rt("gripper");
    let boxed = this.box_gripper_motion_rt(motion);
    this.replace_active_trajectory(Some(ActiveTrajectory::new(
        Some(id),
        Reference::Gripper(boxed),
    )));
    Ok((id, duration))
}

/// **4. ONE owner-side install for standalone replay actions** (`replay`).
pub fn install_replay(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    payload: ReplayInstallPayload,
) -> Result<InstalledAction, ArmError> {
    check_action_authority(inner, stamp, ctx, true, true)
        .map_err(|r| r.into_arm_error(inner))?;
    match payload {
        ReplayInstallPayload::Legacy(prep) => {
            check_start_fresh(&inner.commanded_positions, &prep.start)
                .map_err(|r| r.into_arm_error(inner))?;
            if tool_digest_hash(&inner.config.active_tool.config_digest) != prep.tool_digest {
                return Err(ActionRefusal::new("TOOL_MISMATCH").into_arm_error(inner));
            }
            inner.check_prepared_fresh(stamp.admission.estop_epoch)?;
            let epoch = inner.safety.epoch_id();
            let admitted = consume_action_admission(inner, stamp, ctx.stop_generation)
                .map_err(|r| r.into_arm_error_with_epoch(epoch))?;
            SupervisorInner::install_replay(admitted, prep.plan)
        }
        ReplayInstallPayload::Slot {
            dof,
            start,
            slot,
            kind,
            duration_s,
            power_scale,
        } => install_replay_slot_rt(inner, stamp, ctx, dof, &start, slot, kind, duration_s, power_scale)
            .map_err(|code| ActionRefusal::new(code).into_arm_error(inner)),
    }
}

/// Payload for [`install_replay`].
pub enum ReplayInstallPayload {
    Legacy(Box<PreparedReplayAction>),
    Slot {
        dof: usize,
        start: [f32; MAX_DOF],
        slot: PlanSlotRef<PlanSlot>,
        kind: PathKind,
        duration_s: f64,
        power_scale: Option<f64>,
    },
}

#[allow(clippy::too_many_arguments)]
fn install_replay_slot_rt(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    dof: usize,
    start: &[f32; MAX_DOF],
    slot: PlanSlotRef<PlanSlot>,
    kind: PathKind,
    duration_s: f64,
    power_scale: Option<f64>,
) -> Result<InstalledAction, &'static str> {
    if dof == 0 || dof > MAX_DOF {
        return Err("INVALID_COMMAND");
    }
    let mut post_q = [0.0f32; MAX_DOF];
    let (post_dof, _) = inner.post_quiesce_commanded_positions(&mut post_q);
    check_start_fresh(&post_q[..post_dof], &start[..dof]).map_err(|r| r.code)?;
    if inner.gravity_comp.is_some() {
        return Err("MOTION_BUSY");
    }
    let checked = inner.validate_prepared_replay(
        dof,
        &slot.path,
        kind,
        duration_s,
        stamp.admission.estop_epoch,
    )?;
    let prepared = inner
        .prepared_replay_from_slot_checked(
            dof,
            slot,
            checked,
            duration_s,
            power_scale,
            stamp.admission.estop_epoch,
        )
        .map_err(static_code)?;
    inner
        .check_terminal_fifo_for_new_action()
        .map_err(|e| static_code(e.into()))?;
    let admitted = consume_action_admission(inner, stamp, ctx.stop_generation).map_err(|r| r.code)?;
    SupervisorInner::install_prepared_replay(admitted, prepared).map_err(static_code)
}

/// **5. ONE owner-side install for `BeginMacro` (A7)**: validates authority, optional start
/// freshness, domain revisions, consumes A1 `Admitted<'g>`, detaches/quiesces any replaced
/// active macro, allocates parent `ActionId`, and records [`MacroRun`] with optional
/// owner-enforced whole-macro deadline.
pub fn install_begin_macro(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    spec: &BeginMacroSpec,
    legacy_tag: Option<String>,
    legacy_task: Option<String>,
) -> Result<u64, ActionRefusal> {
    check_action_authority(inner, stamp, ctx, spec.check_workspace_revision, true)?;
    if spec.check_start {
        let dof = usize::from(spec.dof);
        if dof == 0 || dof > MAX_DOF {
            return Err(ActionRefusal::new("INVALID_COMMAND"));
        }
        let mut post_q = [0.0f32; MAX_DOF];
        let (post_dof, _) = inner.post_quiesce_commanded_positions(&mut post_q);
        check_start_fresh(&post_q[..post_dof], &spec.start[..dof])?;
    }
    if spec.macro_deadline.is_some_and(|d| ctx.now > d) {
        return Err(ActionRefusal::new("DEADLINE_EXPIRED"));
    }
    if inner.rt_executor_owned {
        if inner.gravity_comp.is_some() {
            return Err(ActionRefusal::new("MOTION_BUSY"));
        }
        inner
            .check_prepared_fresh_static(stamp.admission.estop_epoch)
            .map_err(ActionRefusal::new)?;
    } else {
        inner
            .check_terminal_fifo_for_new_action()
            .map_err(|e| ActionRefusal::new(static_code(e.into())))?;
        if spec.check_start {
            inner
                .check_prepared_fresh(stamp.admission.estop_epoch)
                .map_err(|e| ActionRefusal::new(static_code(e)))?;
        } else {
            inner
                .action_preconditions()
                .map_err(|e| ActionRefusal::new(static_code(e)))?;
        }
        inner
            .gravity_comp_motion_busy()
            .map_err(|e| ActionRefusal::new(static_code(ArmError::Motion(e))))?;
    }
    let admitted = consume_action_admission(inner, stamp, ctx.stop_generation)?;
    let (this, proof) = admitted
        .consume()
        .map_err(|e| ActionRefusal::new(static_code(e.into())))?;
    Ok(install_begin_macro_committed(
        this,
        &proof,
        spec,
        legacy_tag,
        legacy_task,
    ))
}

/// Legacy adapter entry point for callers that already hold an A1 [`Admitted<'_>`] token
/// (e.g. `sequences::begin`, `grasp::begin`, `calib_hover::begin`, `measure_multiview::begin`).
/// Enforces the same start freshness, deadline, revision, and precondition checks and commits
/// through the single [`install_begin_macro_committed`] path.
pub fn install_begin_macro_admitted(
    admitted: Admitted<'_>,
    spec: &BeginMacroSpec,
    expected_workspace_revision: Option<u64>,
    legacy_tag: Option<String>,
    legacy_task: Option<String>,
) -> Result<u64, ArmError> {
    let (this, proof) = admitted.consume()?;
    install_begin_macro_with_proof(
        this,
        &proof,
        spec,
        expected_workspace_revision,
        legacy_tag,
        legacy_task,
    )
}

/// Same single owner-side `BeginMacro` commit path when the caller already consumed `Admitted`
/// to inspect `SupervisorInner` (e.g. computing approach estimate or reading `boot_id`/`dof`).
pub(crate) fn install_begin_macro_with_proof(
    this: &mut SupervisorInner,
    proof: &crate::supervisor::AdmittedProof,
    spec: &BeginMacroSpec,
    expected_workspace_revision: Option<u64>,
    legacy_tag: Option<String>,
    legacy_task: Option<String>,
) -> Result<u64, ArmError> {
    let now = Instant::now();
    if let Some(ws_rev) = expected_workspace_revision.filter(|_| spec.check_workspace_revision) {
        if this.workspace_revision != ws_rev {
            return Err(ActionRefusal::new("PLAN_STALE").into_arm_error(this));
        }
    }
    if let Some(cal_rev) = spec.calibration_revision {
        if this.calibration_revision != cal_rev {
            return Err(ActionRefusal::new("CALIBRATION_CHANGED").into_arm_error(this));
        }
    }
    if spec.check_start {
        let dof = usize::from(spec.dof);
        if dof == 0 || dof > MAX_DOF {
            return Err(MotionError::InvalidCommand("invalid start DOF".into()).into());
        }
        check_start_fresh(&this.commanded_positions, &spec.start[..dof])
            .map_err(|r| r.into_arm_error(this))?;
    }
    if spec.macro_deadline.is_some_and(|d| now > d) {
        return Err(ActionRefusal::new("DEADLINE_EXPIRED").into_arm_error(this));
    }
    this.check_terminal_fifo_for_new_action()?;
    if spec.check_start {
        this.check_prepared_fresh(proof.stamp().estop_epoch)?;
    } else {
        this.action_preconditions()?;
    }
    this.gravity_comp_motion_busy()?;
    Ok(install_begin_macro_committed(
        this,
        proof,
        spec,
        legacy_tag,
        legacy_task,
    ))
}

fn install_begin_macro_committed(
    this: &mut SupervisorInner,
    proof: &crate::supervisor::AdmittedProof,
    spec: &BeginMacroSpec,
    legacy_tag: Option<String>,
    legacy_task: Option<String>,
) -> u64 {
    if let Some(old_id) = this.macro_run.as_ref().map(|m| m.id) {
        let _ = this.quiesce_macro_children_rt(old_id);
    }
    let id = if this.rt_executor_owned {
        this.begin_action_rt(spec.kind.action_kind_str())
    } else {
        let tag = legacy_tag.or_else(|| {
            (!spec.tag.is_empty()).then(|| spec.tag.as_str().to_owned())
        });
        this.begin_action(spec.kind.action_kind_str(), tag)
    };
    let mut run = MacroRun::new(id, proof.stamp().estop_epoch, proof.stamp());
    if let Some(d) = spec.macro_deadline {
        let tc = if spec.timeout_code.is_empty() {
            "TIMEOUT"
        } else {
            spec.timeout_code.as_str()
        };
        run = run.with_deadline(Some(d), tc);
    }
    this.macro_run = Some(run);
    this.set_action_phase(id, "start", 0.0, None, None);
    if !this.rt_executor_owned {
        if spec.orientation_levelled {
            this.update_action(id, |a| a.orientation = Some("levelled".into()));
        }
        let task = legacy_task.or_else(|| {
            (!spec.task.is_empty()).then(|| spec.task.as_str().to_owned())
        });
        if let Some(task) = task {
            this.update_action(id, |a| a.task = Some(task));
        }
    }
    id
}

/// Payload for [`install_macro_child`] (A7).
pub enum MacroChildPayload {
    /// RT broker pinned slot (`StagedAction::{Cartesian, Joint, Gripper, Replay}`).
    Slot(PlanSlotRef<PlanSlot>),
    /// Legacy prepared Cartesian child (optional same-side approach + table clearance rules).
    LegacyCartesian {
        spec: ChildSpec,
        start: Vec<f32>,
        input: PlanInput,
        direct: JointPath,
        pre: backlash::Pre,
        approach_plan: Option<backlash::Plan>,
        rules: Option<backlash::DescentRules>,
    },
    /// Legacy prepared joint-quintic child (`sequence` pose step / `measure_multiview`).
    LegacyJoint {
        spec: ChildSpec,
        start: Vec<f32>,
        target: Vec<f32>,
        limits: Vec<JointLimit>,
    },
    /// Legacy prepared gripper child.
    LegacyGripper {
        spec: ChildSpec,
        start: Vec<f32>,
        request: GripperRequest,
        learn: LearnMode,
        max_budget_s: Option<f64>,
        extend_pos_force_close_timeout: bool,
    },
    /// Legacy prepared replay child.
    LegacyReplay {
        spec: ChildSpec,
        start: Vec<f32>,
        plan: ReplayPlan,
        tool_digest: u64,
    },
}

/// Result of [`install_macro_child`]: `(parent_id, child_ticket, expected_duration_s, extra_gripper_info)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InstalledMacroChild {
    pub parent: ActionId,
    pub ticket: ChildTicket,
    pub duration_s: f64,
    pub gripper_timeout_s: Option<f64>,
    pub is_pos_force_close: bool,
}

/// **6. ONE owner-side install for `InstallChild` (A7)**.
///
/// Enforces IN EXACT ORDER before ANY mutation:
/// 1. `check_action_authority` (arm, boot, child deadline, stop, epoch, lease, revisions,
///    `parent_generation`, `config_inhibited`, RT feedback, A1 `ClosedConnections`/`ClosedBitset`/`ConsumedStamps`).
/// 2. Parent identity (`spec.ticket.parent.0 == stamp.parent_generation != 0`) and live parent (`macro_alive`).
/// 3. Parent macro deadline (`run.deadline.is_some_and(|d| ctx.now > d)` -> `DEADLINE_EXPIRED` BEFORE A1 consumption!).
/// 4. Strict monotonic child sequence (`run.next_child_seq() == Some(spec.ticket.child_seq)` -> else `PLAN_STALE`, never wraps!).
/// 5. No child currently running or settling (`!inner.macro_child_running(parent_id)` -> else `MOTION_BUSY`).
/// 6. Start-position freshness (`check_start_fresh` -> else `PLAN_STALE`).
/// 7. Structural payload validation.
/// 8. Single-use A1 admission consumption (`consume_action_admission`).
/// 9. Child installation + contact guard + RT child deadline bounded by parent deadline.
pub fn install_macro_child(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    check_workspace_rev: bool,
    payload: MacroChildPayload,
) -> Result<InstalledMacroChild, ArmError> {
    check_action_authority(inner, stamp, ctx, check_workspace_rev, true)
        .map_err(|r| r.into_arm_error(inner))?;
    match payload {
        MacroChildPayload::Slot(slot) => {
            install_macro_child_slot_rt(inner, stamp, ctx, slot)
                .map_err(|code| ActionRefusal::new(code).into_arm_error(inner))
        }
        MacroChildPayload::LegacyCartesian {
            spec,
            start,
            input,
            direct,
            pre,
            approach_plan,
            rules,
        } => {
            let parent_id = precheck_macro_child(inner, stamp, ctx, &spec, &start)
                .map_err(|r| r.into_arm_error(inner))?;
            inner.guard_next_macro_child(parent_id, spec.contact);
            let admitted = match consume_action_admission(inner, stamp, ctx.stop_generation) {
                Ok(a) => a,
                Err(r) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(r.into_arm_error(inner));
                }
            };
            let dur = match SupervisorInner::install_macro_path_same_side_with_rules(
                admitted,
                parent_id,
                &input,
                direct,
                pre,
                approach_plan.as_ref(),
                rules.as_ref(),
            ) {
                Ok(d) => d,
                Err(e) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(e);
                }
            };
            let ticket = arm_child_deadline_clamped(inner, parent_id, &spec, ctx.now, Some((dur * 2.0 + 5.0).min(spec.timeout_s)))
                .ok_or_else(|| ActionRefusal::new("PARENT_REPLACED").into_arm_error(inner))?;
            Ok(InstalledMacroChild {
                parent: ActionId(parent_id),
                ticket,
                duration_s: dur,
                gripper_timeout_s: None,
                is_pos_force_close: false,
            })
        }
        MacroChildPayload::LegacyJoint {
            spec,
            start,
            target,
            limits,
        } => {
            let parent_id = precheck_macro_child(inner, stamp, ctx, &spec, &start)
                .map_err(|r| r.into_arm_error(inner))?;
            inner.guard_next_macro_child(parent_id, spec.contact);
            let admitted = match consume_action_admission(inner, stamp, ctx.stop_generation) {
                Ok(a) => a,
                Err(r) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(r.into_arm_error(inner));
                }
            };
            let dur = match SupervisorInner::install_macro_quintic(admitted, parent_id, target, &limits) {
                Ok(d) => d,
                Err(e) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(e);
                }
            };
            let ticket = arm_child_deadline_clamped(inner, parent_id, &spec, ctx.now, Some((dur * 2.0 + 5.0).min(spec.timeout_s)))
                .ok_or_else(|| ActionRefusal::new("PARENT_REPLACED").into_arm_error(inner))?;
            Ok(InstalledMacroChild {
                parent: ActionId(parent_id),
                ticket,
                duration_s: dur,
                gripper_timeout_s: None,
                is_pos_force_close: false,
            })
        }
        MacroChildPayload::LegacyGripper {
            spec,
            start,
            request,
            learn,
            max_budget_s,
            extend_pos_force_close_timeout,
        } => {
            let parent_id = precheck_macro_child(inner, stamp, ctx, &spec, &start)
                .map_err(|r| r.into_arm_error(inner))?;
            inner.guard_next_macro_child(parent_id, spec.contact);
            let admitted = match consume_action_admission(inner, stamp, ctx.stop_generation) {
                Ok(a) => a,
                Err(r) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(r.into_arm_error(inner));
                }
            };
            let (dur, timeout_s, is_pf) = match SupervisorInner::start_macro_gripper_learning_within(
                admitted,
                parent_id,
                request,
                learn,
                max_budget_s,
                extend_pos_force_close_timeout,
            ) {
                Ok(v) => v,
                Err(e) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(e);
                }
            };
            let explicit_timeout = if is_pf {
                Some(timeout_s + 10.0)
            } else {
                Some((dur + 10.0).min(spec.timeout_s))
            };
            let ticket = arm_child_deadline_clamped(inner, parent_id, &spec, ctx.now, explicit_timeout)
                .ok_or_else(|| ActionRefusal::new("PARENT_REPLACED").into_arm_error(inner))?;
            Ok(InstalledMacroChild {
                parent: ActionId(parent_id),
                ticket,
                duration_s: dur,
                gripper_timeout_s: Some(timeout_s),
                is_pos_force_close: is_pf,
            })
        }
        MacroChildPayload::LegacyReplay {
            spec,
            start,
            plan,
            tool_digest,
        } => {
            let parent_id = precheck_macro_child(inner, stamp, ctx, &spec, &start)
                .map_err(|r| r.into_arm_error(inner))?;
            if tool_digest_hash(&inner.config.active_tool.config_digest) != tool_digest {
                return Err(ActionRefusal::new("TOOL_MISMATCH").into_arm_error(inner));
            }
            inner.guard_next_macro_child(parent_id, spec.contact);
            let admitted = match consume_action_admission(inner, stamp, ctx.stop_generation) {
                Ok(a) => a,
                Err(r) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(r.into_arm_error(inner));
                }
            };
            let dur = match SupervisorInner::install_macro_replay(admitted, parent_id, plan) {
                Ok(d) => d,
                Err(e) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(e);
                }
            };
            let ticket = arm_child_deadline_clamped(inner, parent_id, &spec, ctx.now, Some((dur * 2.0 + 5.0).min(spec.timeout_s)))
                .ok_or_else(|| ActionRefusal::new("PARENT_REPLACED").into_arm_error(inner))?;
            Ok(InstalledMacroChild {
                parent: ActionId(parent_id),
                ticket,
                duration_s: dur,
                gripper_timeout_s: None,
                is_pos_force_close: false,
            })
        }
    }
}

/// **Shared parent-authority gate** enforced on BOTH `InstallChild` and `FinishMacro`:
/// verifies that the incoming request originates from the SAME client connection and authority
/// generations recorded when the parent macro began (preventing cross-client child/finish
/// hijacking even when leases are disabled).
fn check_macro_parent_authority(
    run: &crate::macro_action::MacroRun,
    stamp: &ActionStamp,
) -> Result<(), ActionRefusal> {
    if stamp.admission.connection_id != run.stamp.connection_id {
        return Err(ActionRefusal::new("LEASE_LOST"));
    }
    if stamp.admission.lease_gen != run.stamp.lease_gen {
        return Err(ActionRefusal::new("LEASE_LOST"));
    }
    if stamp.admission.estop_epoch != run.stamp.estop_epoch {
        return Err(ActionRefusal::new("EPOCH_MISMATCH"));
    }
    if stamp.admission.stop_gen != run.stamp.stop_gen {
        return Err(ActionRefusal::new("STOPPED"));
    }
    Ok(())
}

/// Shared parent liveness, parent authority, parent deadline, strict monotonic `child_seq`,
/// quiescence, and start-position freshness prechecks run BEFORE consuming A1 admission or
/// mutating state.
fn precheck_macro_child(
    inner: &SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    spec: &ChildSpec,
    start: &[f32],
) -> Result<u64, ActionRefusal> {
    let id = spec.ticket.parent.0;
    if id == 0 || id != stamp.parent_generation {
        return Err(ActionRefusal::new("PARENT_REPLACED"));
    }
    let Some(run) = inner
        .macro_run
        .as_ref()
        .filter(|m| m.id == id && inner.macro_alive(id))
    else {
        return Err(ActionRefusal::new("PARENT_REPLACED"));
    };
    check_macro_parent_authority(run, stamp)?;
    if run.deadline.is_some_and(|d| ctx.now > d) {
        return Err(ActionRefusal::new("DEADLINE_EXPIRED"));
    }
    let Some(expected_seq) = run.next_child_seq() else {
        return Err(ActionRefusal::new("PLAN_STALE"));
    };
    if spec.ticket.child_seq != expected_seq {
        return Err(ActionRefusal::new("PLAN_STALE"));
    }
    if inner.macro_child_running(id) {
        return Err(ActionRefusal::new("MOTION_BUSY"));
    }
    check_start_fresh(&inner.commanded_positions, start)?;
    Ok(id)
}

fn arm_child_deadline_clamped(
    inner: &mut SupervisorInner,
    parent_id: u64,
    spec: &ChildSpec,
    now: Instant,
    override_timeout_s: Option<f64>,
) -> Option<ChildTicket> {
    let raw_s = override_timeout_s.unwrap_or(spec.timeout_s);
    let clamped_s = if raw_s.is_finite() {
        raw_s.clamp(0.0, 1e6)
    } else {
        1e6
    };
    let mut deadline = now + Duration::from_secs_f64(clamped_s);
    if let Some(parent_dl) = inner
        .macro_run
        .as_ref()
        .filter(|m| m.id == parent_id)
        .and_then(|m| m.deadline)
    {
        deadline = deadline.min(parent_dl);
    }
    inner.arm_macro_child_bounded(
        parent_id,
        deadline,
        BoundedText::new(spec.timeout_code),
        StepErrorText::new("Sub-motion did not finish in time"),
    )
}

fn install_macro_child_slot_rt(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    slot: PlanSlotRef<PlanSlot>,
) -> Result<InstalledMacroChild, &'static str> {
    let Primitive::Action(staged) = slot.primitive else {
        return Err("INVALID_COMMAND");
    };
    let Some(spec) = staged.child().copied() else {
        return Err("INVALID_COMMAND");
    };
    let (dof, start_arr) = staged.dof_and_start();
    if dof == 0 || dof > MAX_DOF {
        return Err("INVALID_COMMAND");
    }
    let parent_id = precheck_macro_child(inner, stamp, ctx, &spec, &start_arr[..dof])
        .map_err(|r| r.code)?;

    match staged {
        StagedAction::Cartesian {
            duration_s,
            power_scale,
            approach_then,
            ..
        } => {
            inner.validate_prepared_joint_path(dof, &slot.path, duration_s)?;
            inner.check_prepared_fresh_static(stamp.admission.estop_epoch)?;
            let last_slice = &slot.path.samples()[slot.path.len() - 1][..dof];
            let mut last_buf = [0.0f32; MAX_DOF];
            last_buf[..dof].copy_from_slice(last_slice);
            let then_traj = match approach_then {
                Some(leg) => Some(validate_approach_leg_rt(
                    inner,
                    stamp.admission.estop_epoch,
                    dof,
                    &last_buf[..dof],
                    leg,
                )?),
                None => None,
            };
            inner.guard_next_macro_child(parent_id, spec.contact);
            let admitted = match consume_action_admission(inner, stamp, ctx.stop_generation) {
                Ok(a) => a,
                Err(r) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(r.code);
                }
            };
            let (this, proof) = admitted.consume().map_err(|e| static_code(e.into()))?;
            let duration = match this.start_path_motion_slice_inner(
                &proof,
                stamp.admission.estop_epoch,
                &last_buf[..dof],
                duration_s,
            ) {
                Ok(d) => d,
                Err(e) => {
                    this.guard_next_macro_child(parent_id, None);
                    return Err(static_code(e));
                }
            };
            if let Some(s) = power_scale {
                this.note_action_power_scale(parent_id, s);
            }
            let arm_dof = this.config.arm_dof();
            let base_ref =
                Reference::Path(motion::PathTrajectory::from_plan_slot(slot, dof, arm_dof, duration_s));
            let total_dur = if let Some((seg, leg)) = then_traj {
                let d2 = seg.duration;
                if let Some(s) = leg.power_scale {
                    this.note_action_power_scale(parent_id, s);
                }
                crate::rt_core::scratch::copy_into(&mut this.target_positions, &leg.target[..dof]);
                this.replace_active_trajectory(Some(ActiveTrajectory::with_approach(
                    Some(parent_id),
                    base_ref,
                    seg,
                )));
                duration + 0.5 + d2
            } else {
                this.replace_active_trajectory(Some(ActiveTrajectory::new(
                    Some(parent_id),
                    base_ref,
                )));
                duration + 0.5
            };
            this.set_macro_sub_running(parent_id);
            let ticket = arm_child_deadline_clamped(this, parent_id, &spec, ctx.now, None)
                .ok_or("PARENT_REPLACED")?;
            Ok(InstalledMacroChild {
                parent: ActionId(parent_id),
                ticket,
                duration_s: total_dur,
                gripper_timeout_s: None,
                is_pos_force_close: false,
            })
        }
        StagedAction::Joint {
            target,
            duration_s,
            max_velocity,
            power_scale,
            ..
        } => {
            if !duration_s.is_finite() || duration_s <= 0.0 || target[..dof].iter().any(|v| !v.is_finite()) {
                return Err("INVALID_COMMAND");
            }
            inner.check_prepared_fresh_static(stamp.admission.estop_epoch)?;
            let (mut limits_buf, lim_n) = inner
                .planning_limits_inline(None)
                .ok_or("INVALID_COMMAND")?;
            if lim_n != dof {
                return Err("INVALID_COMMAND");
            }
            for i in 0..dof {
                if max_velocity[i].is_finite() && max_velocity[i] > 0.0 {
                    limits_buf[i].max_velocity = limits_buf[i].max_velocity.min(max_velocity[i]);
                }
            }
            let mut tgt = [0.0f32; MAX_DOF];
            tgt[..dof].copy_from_slice(&target[..dof]);
            inner
                .safety
                .validate_motion_command(
                    Some(stamp.admission.estop_epoch),
                    &tgt[..dof],
                    &inner.config.limits,
                )
                .map_err(|e| static_code(ArmError::Safety(e)))?;
            inner.torque_hold_filter(&mut tgt[..dof]);
            inner.power_hold_filter(&mut tgt[..dof]);
            let zeros = [0.0f32; MAX_DOF];
            let traj = crate::trajectory::MultiJointQuinticTrajectory::new(
                &inner.commanded_positions[..dof],
                &inner.commanded_velocities[..dof],
                &tgt[..dof],
                &zeros[..dof],
                duration_s,
            )
            .map_err(|_| "INVALID_COMMAND")?;
            inner
                .check_workspace_quintic_inline(&traj)
                .map_err(|e| static_code(ArmError::Safety(e)))?;
            inner.guard_next_macro_child(parent_id, spec.contact);
            let admitted = match consume_action_admission(inner, stamp, ctx.stop_generation) {
                Ok(a) => a,
                Err(r) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(r.code);
                }
            };
            let (this, _proof) = admitted.consume().map_err(|e| static_code(e.into()))?;
            this.release_servo_target();
            this.servo_settled_ticks.fill(0);
            crate::rt_core::scratch::copy_into(&mut this.target_positions, &tgt[..dof]);
            if let Err(e) = this.safety.mark_motion_started() {
                this.guard_next_macro_child(parent_id, None);
                return Err(static_code(ArmError::Safety(e)));
            }
            this.last_heartbeat = ctx.now;
            if let Some(s) = power_scale {
                this.note_action_power_scale(parent_id, s);
            }
            let mut caps = [0.0f32; MAX_DOF];
            for i in 0..dof {
                caps[i] = limits_buf[i].max_velocity;
            }
            this.quintic_settle = None;
            this.replace_active_trajectory(Some(
                ActiveTrajectory::quintic(traj, Some(parent_id)).with_motion_max_velocity(caps),
            ));
            this.set_macro_sub_running(parent_id);
            let ticket = arm_child_deadline_clamped(this, parent_id, &spec, ctx.now, None)
                .ok_or("PARENT_REPLACED")?;
            Ok(InstalledMacroChild {
                parent: ActionId(parent_id),
                ticket,
                duration_s: duration_s + motion::SETTLE_MAX_S,
                gripper_timeout_s: None,
                is_pos_force_close: false,
            })
        }
        StagedAction::Gripper {
            command,
            fraction,
            grasp,
            speed,
            timeout_s,
            max_budget_s,
            extend_pos_force_close_timeout,
            ..
        } => {
            if !speed.is_finite() || speed <= 0.0 {
                return Err("INVALID_COMMAND");
            }
            let gripper_idx = inner.config.gripper_index().ok_or("UNSUPPORTED")?;
            let policy = inner
                .config
                .gripper_policy
                .as_ref()
                .ok_or("UNSUPPORTED")?
                .clone_without_jaw_opening();
            inner.check_prepared_fresh_static(stamp.admission.estop_epoch)?;
            let request = GripperRequest {
                command: command.into(),
                fraction,
                grasp,
                speed,
                timeout_s,
                tag: None,
            };
            if policy.max_close_torque_nm.is_some() {
                inner.check_gripper_close_torque_feedback_rt(
                    request.command,
                    request.fraction,
                    &policy,
                    gripper_idx,
                )?;
            }
            let close_cap = inner.gripper_close_speed_limit();
            let ceiling_est = inner.gripper_ceiling_torque_estimate();
            let lim = &inner.config.limits[gripper_idx];
            let limit_zero_alloc = JointLimit {
                name: String::new(),
                min_position: lim.min_position,
                max_position: lim.max_position,
                max_velocity: lim.max_velocity,
                max_acceleration: lim.max_acceleration,
            };
            let baseline_data = inner.gripper_baseline_data_rt();
            let mut motion = GripperMotion::try_with_close_speed_limit_static(
                request,
                policy,
                limit_zero_alloc,
                inner.commanded_positions[gripper_idx],
                gripper_idx,
                close_cap,
            )?
            .with_ceiling_torque_estimate(ceiling_est);
            let learn_samples = if motion.will_record_learning(LearnMode::Opening) {
                inner.take_move_samples_spare_rt()
            } else {
                None
            };
            motion = motion.with_baseline_data_and_samples(
                baseline_data,
                LearnMode::Opening,
                learn_samples,
            );
            let is_pf = close_cap.is_some() && motion.is_closing();
            if is_pf && extend_pos_force_close_timeout && motion.request.timeout_s.is_some() {
                motion.use_default_timeout();
            }
            let dur = motion.expected_duration();
            let mot_timeout_s = motion.timeout_s;
            if let Some(budget_s) = max_budget_s {
                if !(budget_s.is_finite() && budget_s > 0.0) || (is_pf && dur > budget_s) {
                    return Err("INVALID_COMMAND");
                }
            }
            let target_g = motion.target;
            let is_open = matches!(motion.request.command, GripperCommand::Open);
            let wire_vel = motion.wire_velocity_ceiling();
            inner.guard_next_macro_child(parent_id, spec.contact);
            let admitted = match consume_action_admission(inner, stamp, ctx.stop_generation) {
                Ok(a) => a,
                Err(r) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(r.code);
                }
            };
            let (this, _proof) = admitted.consume().map_err(|e| static_code(e.into()))?;
            if let Err(e) = this.safety.mark_motion_started() {
                this.guard_next_macro_child(parent_id, None);
                return Err(static_code(ArmError::Safety(e)));
            }
            this.releasing = is_open;
            this.gripper_hold_vel_ceiling = Some(wire_vel);
            this.release_servo_target();
            this.servo_settled_ticks.fill(0);
            this.servo_ws_min_depths = None;
            crate::rt_core::scratch::copy_into(&mut this.target_positions, &this.commanded_positions);
            if gripper_idx < this.target_positions.len() {
                this.target_positions[gripper_idx] = target_g;
            }
            this.last_heartbeat = ctx.now;
            let boxed = this.box_gripper_motion_rt(motion);
            this.replace_active_trajectory(Some(ActiveTrajectory::new(
                Some(parent_id),
                Reference::Gripper(boxed),
            )));
            this.set_macro_sub_running(parent_id);
            let override_s = is_pf.then_some(mot_timeout_s + 10.0);
            let ticket = arm_child_deadline_clamped(this, parent_id, &spec, ctx.now, override_s)
                .ok_or("PARENT_REPLACED")?;
            Ok(InstalledMacroChild {
                parent: ActionId(parent_id),
                ticket,
                duration_s: dur,
                gripper_timeout_s: Some(mot_timeout_s),
                is_pos_force_close: is_pf,
            })
        }
        StagedAction::Replay {
            episode,
            speed,
            gripper,
            sample_dt,
            duration_s,
            power_scale,
            tool_digest,
            ..
        } => {
            let kind = PathKind::Replay {
                episode,
                speed,
                gripper,
                sample_dt,
                tool_digest,
            };
            let checked = inner.validate_prepared_replay(
                dof,
                &slot.path,
                kind,
                duration_s,
                stamp.admission.estop_epoch,
            )?;
            let prepared = inner
                .prepared_replay_from_slot_checked(
                    dof,
                    slot,
                    checked,
                    duration_s,
                    power_scale,
                    stamp.admission.estop_epoch,
                )
                .map_err(static_code)?;
            inner.guard_next_macro_child(parent_id, spec.contact);
            let admitted = match consume_action_admission(inner, stamp, ctx.stop_generation) {
                Ok(a) => a,
                Err(r) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(r.code);
                }
            };
            let dur = match SupervisorInner::install_macro_prepared_replay(admitted, parent_id, prepared)
            {
                Ok(d) => d,
                Err(e) => {
                    inner.guard_next_macro_child(parent_id, None);
                    return Err(static_code(e));
                }
            };
            let ticket = arm_child_deadline_clamped(inner, parent_id, &spec, ctx.now, None)
                .ok_or("PARENT_REPLACED")?;
            Ok(InstalledMacroChild {
                parent: ActionId(parent_id),
                ticket,
                duration_s: dur,
                gripper_timeout_s: None,
                is_pos_force_close: false,
            })
        }
    }
}

/// **7. ONE owner-side install for `FinishMacro` (A7)**.
///
/// Stop/finish operation (never starts actuator motion, so exempt from A1 motion-start
/// consumption while strictly enforcing authority, finish submission deadline, and live
/// parent identity).
///
/// - Checks authority (`arm`, `boot`, finish submission `deadline`, `stop`, `epoch`, `lease`,
///   `parent_generation`, `ClosedConnections`/`ClosedBitset`).
/// - Refuses delayed/duplicate finishes from replaced/ended parents with `PARENT_REPLACED`
///   before ANY mutation to motion, `held`, or `last_action`.
/// - Normal finish (`FinishState::Done`): enforces physical quiescence — refuses `MOTION_BUSY`
///   if any child trajectory, settle, or deferred TX completion is still active; applies
///   optional `held` update only after passing the gate.
/// - Forced finish (`FinishState::Failed | Cancelled`): quiesces active child motion
///   (preserving gripper measured-position hold!); when a child was actively moving/settling
///   under a physical driver on RT, defers final terminal commit through `pending_deferred_done`
///   (`DeferredCompletionTarget::MacroFinish`) until Step 5's replacement hold output is accepted.
pub fn install_finish_macro(
    inner: &mut SupervisorInner,
    stamp: &ActionStamp,
    ctx: &OwnerCheckCtx,
    spec: &FinishMacroSpec,
    legacy_message: Option<String>,
) -> Result<ActionId, ActionRefusal> {
    check_action_authority(inner, stamp, ctx, false, false)?;
    let id = spec.parent.0;
    let Some(run) = inner
        .macro_run
        .as_ref()
        .filter(|m| id != 0 && id == stamp.parent_generation && m.id == id && inner.macro_alive(id))
    else {
        return Err(ActionRefusal::new("PARENT_REPLACED"));
    };
    check_macro_parent_authority(run, stamp)?;
    // Enforce parent whole-macro deadline precedence before allowing Done completion.
    if spec.state == FinishState::Done && run.deadline.is_some_and(|d| ctx.now > d) {
        return Err(ActionRefusal::new("DEADLINE_EXPIRED"));
    }
    let child_active = inner.macro_child_running(id);
    if spec.state == FinishState::Done && child_active {
        return Err(ActionRefusal::new("MOTION_BUSY"));
    }
    let code_opt = (!spec.code.is_empty()).then_some(spec.code.as_str());
    let clear_held = match spec.state {
        FinishState::Done => matches!(spec.held, HeldUpdate::Clear),
        FinishState::Failed | FinishState::Cancelled => {
            spec.held == HeldUpdate::Clear || spec.holding == Some(false)
        }
    };
    if spec.state == FinishState::Done {
        if let HeldUpdate::Set { label, spill_safe } = spec.held {
            if !inner.rt_executor_owned {
                inner.held = Some(HeldObject {
                    label: label.as_str().to_owned(),
                    spill_safe,
                });
            } else {
                inner.set_held_rt(label, spill_safe);
            }
        }
    }
    inner
        .enter_macro_finish_rt(
            id,
            spec.state.as_str(),
            code_opt,
            spec.message,
            legacy_message,
            spec.holding,
            spec.gripper_gap_rad,
            clear_held,
        )
        .map_err(ActionRefusal::new)?;
    Ok(ActionId(id))
}

// ---------------------------------------------------------------------------------------
// Single RT broker dispatcher called from `CoreOps::commit_action` in `ops.rs`

/// Dispatches one `CommandKind::Action(cmd)` on `armd-control` (zero-alloc on RT).
pub fn commit_action(
    inner: &mut SupervisorInner,
    d: &Descriptor,
    cmd: ActionCommand,
    slot: Option<PlanSlotRef<PlanSlot>>,
    ctx: &CommitCtx,
) -> Result<Committed, &'static str> {
    let owner_ctx = OwnerCheckCtx::from_commit_ctx(ctx);
    match cmd {
        ActionCommand::InstallPrepared {
            kind,
            prepared_boot,
            admission_request_id,
            check_workspace_revision,
            poses_revision,
            calibration_revision,
            gripper_calib_revision,
            power_budget_revision,
            ..
        } => {
            let slot = slot.ok_or("PLAN_RETIRED")?;
            let stamp = ActionStamp::from_descriptor(
                d,
                prepared_boot,
                admission_request_id,
                calibration_revision,
                poses_revision,
                None,
                None,
                gripper_calib_revision,
                power_budget_revision,
            );
            check_action_authority(inner, &stamp, &owner_ctx, check_workspace_revision, true)
                .map_err(|r| r.code)?;
            let Primitive::Action(staged) = slot.primitive else {
                return Err("INVALID_COMMAND");
            };
            if staged.child().is_some() {
                return Err("INVALID_COMMAND");
            }
            let (id, _) = match staged {
                StagedAction::Cartesian {
                    dof,
                    start,
                    duration_s,
                    power_scale,
                    approach_then,
                    ..
                } => install_move_cartesian_slot_rt(
                    inner,
                    &stamp,
                    &owner_ctx,
                    kind,
                    usize::from(dof),
                    &start,
                    slot,
                    duration_s,
                    power_scale,
                    approach_then,
                )?,
                StagedAction::Joint {
                    dof,
                    start,
                    target,
                    duration_s,
                    max_velocity,
                    power_scale,
                    settle,
                    approach_then,
                    ..
                } => install_move_joint_rt(
                    inner,
                    &stamp,
                    &owner_ctx,
                    kind,
                    usize::from(dof),
                    &start,
                    &target,
                    duration_s,
                    max_velocity,
                    power_scale,
                    settle,
                    approach_then,
                )?,
                StagedAction::Gripper {
                    dof,
                    start,
                    command,
                    fraction,
                    grasp,
                    speed,
                    timeout_s,
                    speed_scale,
                    ..
                } => install_gripper_rt(
                    inner,
                    &stamp,
                    &owner_ctx,
                    usize::from(dof),
                    &start,
                    command,
                    fraction,
                    grasp,
                    speed,
                    timeout_s,
                    speed_scale,
                )?,
                StagedAction::Replay {
                    dof,
                    start,
                    episode,
                    speed,
                    gripper,
                    sample_dt,
                    duration_s,
                    power_scale,
                    tool_digest,
                    ..
                } => install_replay_slot_rt(
                    inner,
                    &stamp,
                    &owner_ctx,
                    usize::from(dof),
                    &start,
                    slot,
                    PathKind::Replay {
                        episode,
                        speed,
                        gripper,
                        sample_dt,
                        tool_digest,
                    },
                    duration_s,
                    power_scale,
                )?,
            };
            Ok(Committed::for_action(d, Some(ActionId(id))))
        }
        ActionCommand::BeginMacro(spec) => {
            let stamp = ActionStamp::from_descriptor(
                d,
                spec.prepared_boot,
                spec.admission_request_id,
                spec.calibration_revision,
                spec.poses_revision,
                spec.sequences_revision,
                spec.grasp_setup_revision,
                None,
                spec.power_budget_revision,
            );
            let id = install_begin_macro(inner, &stamp, &owner_ctx, &spec, None, None)
                .map_err(|r| r.code)?;
            Ok(Committed::for_action(d, Some(ActionId(id))))
        }
        ActionCommand::InstallChild {
            prepared_boot,
            check_workspace_revision,
            calibration_revision,
            gripper_calib_revision,
            power_budget_revision,
            ..
        } => {
            let slot = slot.ok_or("PLAN_RETIRED")?;
            let stamp = ActionStamp::from_descriptor(
                d,
                prepared_boot,
                0,
                calibration_revision,
                None,
                None,
                None,
                gripper_calib_revision,
                power_budget_revision,
            );
            check_action_authority(inner, &stamp, &owner_ctx, check_workspace_revision, true)
                .map_err(|r| r.code)?;
            let installed = install_macro_child_slot_rt(inner, &stamp, &owner_ctx, slot)?;
            Ok(Committed::for_action(d, Some(installed.parent)))
        }
        ActionCommand::FinishMacro(spec) => {
            let stamp = ActionStamp::from_descriptor(
                d,
                spec.prepared_boot,
                0,
                None,
                None,
                None,
                None,
                None,
                None,
            );
            let parent = install_finish_macro(inner, &stamp, &owner_ctx, &spec, None)
                .map_err(|r| r.code)?;
            Ok(Committed::for_action(d, Some(parent)))
        }
    }
}

// ---------------------------------------------------------------------------------------
// Caller-owned bounded cross-family-safe ordered reply delivery

/// Immutable per-request metadata recorded at submit time (keyed by `RequestId`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SubmittedActionMeta {
    pub client: ClientId,
    pub expected_duration_s: f64,
    pub internal: bool,
    pub expects_terminal: bool,
}

/// Ordered delivery event emitted to a client writer (or internal orchestrator) preserving
/// exact `Broker::try_poll` cross-family FIFO order and bounded backpressure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ActionDeliveryEvent {
    Client {
        client: ClientId,
        reply: Reply,
        expected_duration_s: Option<f64>,
    },
    Internal {
        client: ClientId,
        reply: Reply,
    },
    OtherFamily {
        client: ClientId,
        reply: Reply,
    },
}

/// Caller-owned, zero-unbounded-buffer reply dispatcher over [`Broker::try_poll`].
///
/// - Never hijacks `Broker::poll` from other families or tests.
/// - Preserves exact FIFO order across ALL families (`Terminal(old)` -> `Ack(new)` -> `Terminal(new)`).
/// - When a client's downstream writer returns `DeliveryDecision::Backpressured`, the reply
///   remains in the bounded `Broker` reply ring (keeping its reservation intact so admission
///   backpressures cleanly with `RepliesFull`, with zero unbounded queues and zero drops).
/// - Binds prepared `expected_duration_s` immutably to `RequestId` at submission time.
#[derive(Debug, Default)]
pub struct ActionDelivery {
    meta: HashMap<RequestId, SubmittedActionMeta>,
}

impl ActionDelivery {
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers an action-family request immediately after `Broker::submit` returns `Ok(req)`.
    pub fn register(
        &mut self,
        request: RequestId,
        client: ClientId,
        expected_duration_s: f64,
        internal: bool,
        expects_terminal: bool,
    ) {
        self.meta.insert(
            request,
            SubmittedActionMeta {
                client,
                expected_duration_s,
                internal,
                expects_terminal,
            },
        );
    }

    /// Discards tracked metadata for a disconnected client so `meta` never leaks entries
    /// whose replies are abandoned inside [`Broker::try_poll`].
    pub fn disconnect(&mut self, client: ClientId) {
        self.meta.retain(|_, m| m.client != client);
    }

    /// Returns the number of tracked action requests still awaiting completion.
    pub fn tracked(&self) -> usize {
        self.meta.len()
    }

    /// Drains replies from `broker` in exact global FIFO order via [`Broker::try_poll`],
    /// stopping immediately on [`DeliveryDecision::Backpressured`] so bounded capacity and
    /// cross-family ordering stay intact.
    pub fn poll<P>(
        &mut self,
        broker: &mut Broker<P>,
        mut deliver: impl FnMut(ActionDeliveryEvent) -> DeliveryDecision,
    ) -> usize {
        let meta = &mut self.meta;
        broker.try_poll(|client, reply| {
            let req = reply.request();
            let event = match meta.get(&req).copied() {
                Some(m) if m.internal => ActionDeliveryEvent::Internal { client, reply },
                Some(m) => ActionDeliveryEvent::Client {
                    client,
                    reply,
                    expected_duration_s: matches!(reply, Reply::Ack { .. })
                        .then_some(m.expected_duration_s),
                },
                None => ActionDeliveryEvent::OtherFamily { client, reply },
            };
            let decision = deliver(event);
            if decision != DeliveryDecision::Backpressured {
                let done = match reply {
                    Reply::Refused { .. } | Reply::Terminal { .. } => true,
                    Reply::Ack { .. } => meta.get(&req).is_some_and(|m| !m.expects_terminal),
                };
                if done {
                    meta.remove(&req);
                }
            }
            decision
        })
    }
}
