use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use crate::model::JointLimit;

/// Stop policy strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StopPolicy {
    /// Return to stow/home position cleanly, then disable torque
    NormalUnload,
    /// Rigid lock at current position immediately; latch safety state
    EStopLatch,
    /// Over-temperature deceleration stop; latch fault state
    ThermalTrip,
    /// Communication/bus loss; lock current position and latch fault
    BusLossFallback,
}

/// Reasons for entering Fault state
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FaultReason {
    JointLimitExceeded {
        joint_idx: usize,
        joint_name: String,
        value: f32,
        min: f32,
        max: f32,
    },
    ThermalTrip {
        joint_idx: usize,
        temperature: f32,
        limit: f32,
    },
    BusLoss,
    WatchdogTimeout,
    /// Bounded CAN TX stalled: frames expired past the age limit or consecutive ticks deferred.
    CanTxStalled,
    /// J7 position-force feedback stayed stale past the configured bounded limit (recoverable).
    J7FeedbackStale,
    Other(String),
}

/// Declares [`FaultCode`] and [`FaultCode::ALL`] from one list, so `ALL` names every variant.
macro_rules! fault_codes {
    ($(#[$doc:meta])* pub enum FaultCode { $($(#[$vdoc:meta])* $variant:ident,)* }) => {
        $(#[$doc])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
        #[serde(rename_all = "SCREAMING_SNAKE_CASE")]
        pub enum FaultCode { $($(#[$vdoc])* $variant,)* }

        impl FaultCode {
            /// Every variant, in declaration order.
            pub const ALL: &'static [FaultCode] = &[$(FaultCode::$variant,)*];
        }
    };
}

fault_codes! {
    /// Machine-readable cause of the current Fault / EStop (`observation.fault.code`).
    pub enum FaultCode {
        Estop,
        BusLoss,
        ThermalTrip,
        JointLimit,
        WatchdogTimeout,
        FollowingError,
        ControlDeadline,
        InvalidTelemetry,
        DriverWriteFailed,
        CanTxStalled,
        /// J7 position-force feedback stayed stale past `damiao_gripper.feedback_stale_fault_ms`.
        J7FeedbackStale,
        HardwareFault,
        EnableFailed,
        ServoNoBrake,
        HandoverUnsettled,
        GravityCompFailed,
        /// Collision guard: more trips than `collision.max_auto_resumes` within the window.
        Collision,
        Other,
    }
}

impl FaultCode {
    /// Bitmask bit for this fault code (fits in `u32`).
    #[inline]
    pub const fn bit(self) -> u32 {
        1u32 << (self as u8)
    }

    /// Hint for client recovery policies; armd never clears a fault by itself.
    pub fn recoverable(self) -> bool {
        matches!(
            self,
            FaultCode::WatchdogTimeout
                | FaultCode::ControlDeadline
                | FaultCode::HandoverUnsettled
                | FaultCode::J7FeedbackStale
        )
    }

    /// Code implied by a reason when the trigger site gave none.
    pub fn from_reason(reason: &FaultReason) -> Self {
        match reason {
            FaultReason::JointLimitExceeded { .. } => FaultCode::JointLimit,
            FaultReason::ThermalTrip { .. } => FaultCode::ThermalTrip,
            FaultReason::BusLoss => FaultCode::BusLoss,
            FaultReason::WatchdogTimeout => FaultCode::WatchdogTimeout,
            FaultReason::CanTxStalled => FaultCode::CanTxStalled,
            FaultReason::J7FeedbackStale => FaultCode::J7FeedbackStale,
            FaultReason::Other(_) => FaultCode::Other,
        }
    }
}

/// Inline, allocation-free numeric diagnostics for [`FaultCode::J7FeedbackStale`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct J7StaleDetail {
    pub joint: u8,
    pub age_ms: Option<u64>,
    pub limit_ms: u64,
}

impl J7StaleDetail {
    /// Formats a human-readable J7 staleness message directly into a [`core::fmt::Write`] sink
    /// without heap allocation.
    pub fn write_message_into(&self, out: &mut impl core::fmt::Write) {
        let j = usize::from(self.joint) + 1;
        let _ = match self.age_ms {
            Some(age_ms) => write!(
                out,
                "Joint {j} position-force feedback stale for {age_ms} ms (limit {} ms); jaw holding last target",
                self.limit_ms
            ),
            None => write!(
                out,
                "Joint {j} position-force feedback unavailable/unbounded past limit {} ms; jaw holding last target",
                self.limit_ms
            ),
        };
    }
}

/// Inline, allocation-free numeric diagnostics for [`FaultCode::CanTxStalled`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxStallDetail {
    pub queued: usize,
    pub dropped: usize,
    pub age_ms: u64,
    pub limit_ms: u64,
    pub consecutive_ticks: u32,
    pub is_hold: bool,
}

impl TxStallDetail {
    /// Formats a human-readable stall message directly into a [`core::fmt::Write`] sink
    /// without heap allocation.
    pub fn write_message_into(&self, out: &mut impl core::fmt::Write) {
        let _ = if self.dropped > 0 {
            if self.is_hold {
                write!(
                    out,
                    "CAN TX hold stalled: {} frames expired after {} ms ({} frames queued, limit {} ms)",
                    self.dropped, self.age_ms, self.queued, self.limit_ms
                )
            } else {
                write!(
                    out,
                    "CAN TX stalled: {} frames expired after {} ms ({} frames queued, limit {} ms)",
                    self.dropped, self.age_ms, self.queued, self.limit_ms
                )
            }
        } else if self.is_hold {
            write!(
                out,
                "CAN TX hold stalled: {} frames queued with no hold sent for {} ms (limit {} ms)",
                self.queued, self.age_ms, self.limit_ms
            )
        } else {
            write!(
                out,
                "CAN TX stalled: {} frames queued across {} consecutive ticks (age {} ms, limit {} ms)",
                self.queued, self.consecutive_ticks, self.age_ms, self.limit_ms
            )
        };
    }

    /// Formats a human-readable stall message into `out` without allocating while `out` has capacity.
    pub fn write_message(&self, out: &mut String) {
        self.write_message_into(out);
    }
}

/// `observation.fault`: present whenever the safety state is Fault or EStop.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FaultInfo {
    pub code: FaultCode,
    pub message: BoundedText<SAFETY_LOG_TEXT_CAPACITY>,
    pub recoverable: bool,
}

impl FaultReason {
    /// Writes the human-readable fault message directly into a [`core::fmt::Write`] sink
    /// without heap allocation.
    pub fn write_message_into(&self, w: &mut impl core::fmt::Write) {
        match self {
            FaultReason::JointLimitExceeded { joint_name, value, min, max, .. } => {
                let _ = write!(w, "Joint '{joint_name}' at {value:.4} outside [{min:.4}, {max:.4}]");
            }
            FaultReason::ThermalTrip { joint_idx, temperature, limit } => {
                let _ = write!(w, "Joint {} temperature {temperature:.1} C above limit {limit:.1} C", joint_idx + 1);
            }
            FaultReason::BusLoss => {
                let _ = w.write_str("No fresh feedback from every joint (bus loss)");
            }
            FaultReason::WatchdogTimeout => {
                let _ = w.write_str("Control lease expired while torqued");
            }
            FaultReason::CanTxStalled => {
                let _ = w.write_str("CAN TX stalled (bounded transmit queue did not drain within age limit)");
            }
            FaultReason::J7FeedbackStale => {
                let _ = w.write_str("Joint 7 position-force feedback stale past bounded limit; jaw holding last target");
            }
            FaultReason::Other(m) => {
                let _ = w.write_str(m);
            }
        }
    }

    pub fn message(&self) -> String {
        let mut out = String::new();
        self.write_message_into(&mut out);
        out
    }
}

/// Safety state machine states
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SafetyState {
    Booting,
    Idle,
    Armed,
    Moving,
    EStop,
    Fault(FaultReason),
}

impl SafetyState {
    pub fn is_operational(&self) -> bool {
        matches!(self, SafetyState::Armed | SafetyState::Moving)
    }

    pub fn is_stopped(&self) -> bool {
        matches!(self, SafetyState::EStop | SafetyState::Fault(_))
    }
}

/// Safety errors
#[derive(Debug, Clone, PartialEq, Error, Serialize, Deserialize)]
pub enum SafetyError {
    #[error("E-Stop is latched, motion rejected")]
    EStopActive,

    #[error("Fault active: {0:?}")]
    FaultActive(FaultReason),

    #[error("Arm is not armed/torqued (current state: {0:?})")]
    NotArmed(SafetyState),

    #[error("Epoch mismatch: expected {expected}, received {received}. Command invalidated by generation change")]
    EpochMismatch { expected: u64, received: u64 },

    #[error("Actuator feedback unavailable: physical angles not received from hardware")]
    FeedbackUnavailable,

    #[error("Soft limit exceeded on joint '{joint}': value {value:.4} outside [{min:.4}, {max:.4}]")]
    SoftLimitExceeded {
        joint: String,
        value: f32,
        min: f32,
        max: f32,
    },

    #[error("Dimension mismatch: expected {expected} joints, got {received}")]
    DimensionMismatch { expected: usize, received: usize },

    #[error("Cannot clear fault while hardware fault condition persists: {0}")]
    ClearFaultRejected(String),

    #[error("Invalid non-finite float value (NaN or Inf) encountered")]
    InvalidValue,

    #[error("Arm is not at resting pose{}: (current: {current:?}, expected: {expected:?})", reason.as_deref().map(|r| format!(" ({r})")).unwrap_or_default())]
    ArmNotAtRestingPose {
        current: Vec<f32>,
        expected: Vec<f32>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        reason: Option<String>,
    },

    #[error("Motion forbidden in fault/estop state: {0}")]
    MotionForbiddenInFaultState(String),

    #[error("Arm is coming to a safe stop from previous session; motion gate locked")]
    HandoverSettling,

    #[error("Motion busy: an action trajectory is running")]
    MotionBusy,

    #[error("Collision hold active: {0}")]
    CollisionHold(String),

    #[error("Tool commissioning required before {0}; confirm commissioning or use low-speed debug (speed_scale <= 0.2)")]
    CommissioningRequired(String),

    #[error("Target outside the safe workspace: check point {point} {limit} {value:.3} m beyond bound {bound:.3} m")]
    WorkspaceViolation {
        point: usize,
        limit: String,
        value: f32,
        bound: f32,
        #[serde(default)]
        detail: Box<crate::kinematics::WorkspaceViolationDetail>,
    },

    #[error("OWNER_TRANSFERRED: control ownership transferred to RT executor; shell lease calls are refused")]
    OwnerTransferred,

    #[error("Stopped before motion began")]
    Stopped,

    #[error("Control lease is no longer current")]
    LeaseLost,

    #[error("Action terminal queue is full; wait for pending action completions to drain before starting a new action")]
    TerminalsBusy,

    #[error("Admission stamp has already been consumed or carries an invalid request id")]
    StampAlreadyConsumed,
}

impl SafetyError {
    pub fn from_workspace(f: crate::chain::WorkspaceFailure) -> Self {
        Self::WorkspaceViolation {
            point: f.point,
            limit: f.limit,
            value: f.value as f32,
            bound: f.bound as f32,
            detail: f.detail,
        }
    }

    pub fn with_workspace_phase(mut self, phase: &str) -> Self {
        if let Self::WorkspaceViolation { ref mut detail, .. } = self {
            detail.violation_phase = Some(phase.into());
        }
        self
    }

    pub fn workspace_detail(&self) -> Option<crate::kinematics::WorkspaceViolationDetail> {
        match self {
            Self::WorkspaceViolation { detail, .. } => Some((**detail).clone()),
            _ => None,
        }
    }
}

/// Capacity of [`SafetyManager`]'s inline off-thread fault reclamation queue.
pub const RETIRED_STATE_CAPACITY: usize = 16;
/// Capacity of [`SafetyManager`]'s preallocated `Arc<SafetyState>` fault slot pool.
pub const FAULT_ARC_POOL_CAPACITY: usize = 32;
/// Capacity of [`SafetyManager`]'s preallocated formatted-message `Arc<SafetyState>` fault slot pool.
pub const FORMATTED_FAULT_ARC_POOL_CAPACITY: usize = 32;
/// Capacity of [`SafetyManager`]'s preallocated `JointLimitExceeded` `Arc<SafetyState>` fault slot pool.
pub const JOINT_LIMIT_ARC_POOL_CAPACITY: usize = 8;
/// Capacity of [`SafetyManager`]'s bounded deferred safety log event ring.
pub const SAFETY_LOG_CAPACITY: usize = 64;
/// Maximum inline text bytes stored in a deferred safety log event's `FaultReason::Other`.
pub const SAFETY_LOG_TEXT_CAPACITY: usize = 320;
/// Maximum inline bytes stored for a joint name in a deferred safety log event.
pub const SAFETY_JOINT_NAME_CAPACITY: usize = 64;
/// Fallback message recorded when a latched fault's detail storage is unavailable.
pub const DETAIL_UNAVAILABLE: &str = "detail unavailable";

/// Preallocated static fault variants latched without heap allocation or pool consumption.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StaticFault {
    InvalidTelemetry = 0,
    UnexpectedTorqueLoss = 1,
    ControlDeadline = 2,
    HandoverSettlingTimeout = 3,
    DisableUnconfirmed = 4,
}

impl StaticFault {
    pub const COUNT: usize = 5;
    pub const ALL: [Self; Self::COUNT] = [
        Self::InvalidTelemetry,
        Self::UnexpectedTorqueLoss,
        Self::ControlDeadline,
        Self::HandoverSettlingTimeout,
        Self::DisableUnconfirmed,
    ];

    pub const fn code(self) -> FaultCode {
        match self {
            Self::InvalidTelemetry => FaultCode::InvalidTelemetry,
            Self::UnexpectedTorqueLoss => FaultCode::HardwareFault,
            Self::ControlDeadline => FaultCode::ControlDeadline,
            Self::HandoverSettlingTimeout => FaultCode::HandoverUnsettled,
            Self::DisableUnconfirmed => FaultCode::DriverWriteFailed,
        }
    }

    pub const fn message(self) -> &'static str {
        match self {
            Self::InvalidTelemetry => "Invalid hardware telemetry",
            Self::UnexpectedTorqueLoss => "Unexpected motor torque loss detected while operational",
            Self::ControlDeadline => "Control loop deadline exceeded",
            Self::HandoverSettlingTimeout => "Handover settling timeout",
            Self::DisableUnconfirmed => "Torque disable has not been confirmed by fresh hardware feedback",
        }
    }
}

/// Fault kind tracked in [`SafetyManager`]'s fixed-size latched-kind bitmask.
///
/// Severity order (highest to lowest, used for reported `fault.code` and displayed state):
/// `ThermalTrip` > `BusLoss` > `CollisionLatched` > `JointLimit` > `CanTxStalled`
/// > `Other` (non-recoverable) > `Estop` > `Other` (recoverable) / `J7FeedbackStale`
/// > `WatchdogTimeout`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FaultKind {
    ThermalTrip = 0,
    BusLoss = 1,
    CollisionLatched = 2,
    JointLimit = 3,
    CanTxStalled = 4,
    Other = 5,
    WatchdogTimeout = 6,
    Estop = 7,
    J7FeedbackStale = 8,
}

impl FaultKind {
    pub const COUNT: usize = 9;
    /// All known kinds in strict descending severity order.
    pub const ALL: [Self; Self::COUNT] = [
        Self::ThermalTrip,
        Self::BusLoss,
        Self::CollisionLatched,
        Self::JointLimit,
        Self::CanTxStalled,
        Self::Other,
        Self::Estop,
        Self::J7FeedbackStale,
        Self::WatchdogTimeout,
    ];
    /// Bitmask covering every known [`FaultKind`]; any bit outside this mask fails closed.
    pub const ALL_MASK: u16 = (1u16 << Self::COUNT) - 1;

    #[inline]
    pub const fn bit(self) -> u16 {
        1u16 << (self as u8)
    }

    #[inline]
    pub const fn from_bit_index(idx: u8) -> Option<Self> {
        match idx {
            0 => Some(Self::ThermalTrip),
            1 => Some(Self::BusLoss),
            2 => Some(Self::CollisionLatched),
            3 => Some(Self::JointLimit),
            4 => Some(Self::CanTxStalled),
            5 => Some(Self::Other),
            6 => Some(Self::WatchdogTimeout),
            7 => Some(Self::Estop),
            8 => Some(Self::J7FeedbackStale),
            _ => None,
        }
    }

    /// Default [`FaultCode`] associated with this kind.
    #[inline]
    pub const fn default_code(self) -> FaultCode {
        match self {
            Self::ThermalTrip => FaultCode::ThermalTrip,
            Self::BusLoss => FaultCode::BusLoss,
            Self::CollisionLatched => FaultCode::Collision,
            Self::JointLimit => FaultCode::JointLimit,
            Self::CanTxStalled => FaultCode::CanTxStalled,
            Self::Other => FaultCode::Other,
            Self::WatchdogTimeout => FaultCode::WatchdogTimeout,
            Self::Estop => FaultCode::Estop,
            Self::J7FeedbackStale => FaultCode::J7FeedbackStale,
        }
    }

    /// Severity rank for `(kind, code)` (strict total order across distinct `FaultKind`s so
    /// `highest_severity_kind()` and `latch()` state promotion always agree).
    #[inline]
    pub const fn severity(self, code: FaultCode) -> u8 {
        match self {
            Self::ThermalTrip => 9,
            Self::BusLoss => 8,
            Self::CollisionLatched => 7,
            Self::JointLimit => 6,
            Self::CanTxStalled => 5,
            Self::Other => {
                if matches!(code, FaultCode::ControlDeadline | FaultCode::HandoverUnsettled) {
                    2
                } else {
                    4
                }
            }
            Self::Estop => 3,
            Self::J7FeedbackStale => 1,
            Self::WatchdogTimeout => 0,
        }
    }

    /// True if this kind is inherently non-recoverable regardless of detail code.
    #[inline]
    pub const fn always_non_recoverable(self) -> bool {
        matches!(
            self,
            Self::ThermalTrip
                | Self::BusLoss
                | Self::CollisionLatched
                | Self::JointLimit
                | Self::CanTxStalled
                | Self::Estop
        )
    }

    #[inline]
    pub const fn from_reason(reason: &FaultReason) -> Self {
        match reason {
            FaultReason::JointLimitExceeded { .. } => Self::JointLimit,
            FaultReason::ThermalTrip { .. } => Self::ThermalTrip,
            FaultReason::BusLoss => Self::BusLoss,
            FaultReason::WatchdogTimeout => Self::WatchdogTimeout,
            FaultReason::CanTxStalled => Self::CanTxStalled,
            FaultReason::J7FeedbackStale => Self::J7FeedbackStale,
            FaultReason::Other(_) => Self::Other,
        }
    }

    #[inline]
    pub const fn from_code(code: FaultCode) -> Self {
        match code {
            FaultCode::ThermalTrip => Self::ThermalTrip,
            FaultCode::BusLoss => Self::BusLoss,
            FaultCode::Collision => Self::CollisionLatched,
            FaultCode::JointLimit => Self::JointLimit,
            FaultCode::CanTxStalled => Self::CanTxStalled,
            FaultCode::J7FeedbackStale => Self::J7FeedbackStale,
            FaultCode::WatchdogTimeout => Self::WatchdogTimeout,
            FaultCode::Estop => Self::Estop,
            _ => Self::Other,
        }
    }
}

/// Zero-allocation fault payload passed to [`SafetyManager::latch`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum FaultDetail {
    /// Hardware or operator emergency stop.
    Estop,
    /// CAN / actuator feedback lost on one or more joints.
    BusLoss,
    /// Control lease expired while torqued.
    WatchdogTimeout,
    /// Bounded CAN TX stalled (optional inline numeric diagnostics, 0 heap ops).
    CanTxStalled(Option<TxStallDetail>),
    /// J7 position-force feedback stayed stale past the configured limit (0 heap ops).
    J7FeedbackStale(Option<J7StaleDetail>),
    /// Joint temperature exceeded `max_temperature`.
    ThermalTrip { joint_idx: usize, temperature: f32, limit: f32 },
    /// Joint position exceeded configured/driver limits.
    JointLimitExceeded {
        joint_idx: usize,
        joint_name: BoundedText<SAFETY_JOINT_NAME_CAPACITY>,
        value: f32,
        min: f32,
        max: f32,
    },
    /// Preallocated static fault (0 pool consumption, 0 heap ops).
    Static(StaticFault),
    /// Dynamically formatted message stored inline (uses formatted slot pool when available).
    Formatted {
        code: FaultCode,
        message: BoundedText<SAFETY_LOG_TEXT_CAPACITY>,
    },
    /// Fallback when detail storage is unavailable.
    DetailUnavailable(FaultCode),
}

impl FaultDetail {
    /// Formats a dynamic fault message inline into a [`BoundedText`] without heap allocation.
    pub fn formatted(
        code: FaultCode,
        fmt_msg: impl FnOnce(&mut BoundedText<SAFETY_LOG_TEXT_CAPACITY>),
    ) -> Self {
        let mut message = BoundedText::<SAFETY_LOG_TEXT_CAPACITY>::empty();
        fmt_msg(&mut message);
        Self::Formatted { code, message }
    }

    /// Converts a `(FaultCode, &FaultReason)` pair into a bounded, heap-free [`FaultDetail`].
    pub fn from_reason(code: FaultCode, reason: &FaultReason) -> Self {
        match reason {
            FaultReason::ThermalTrip { joint_idx, temperature, limit } => {
                Self::ThermalTrip { joint_idx: *joint_idx, temperature: *temperature, limit: *limit }
            }
            FaultReason::BusLoss => Self::BusLoss,
            FaultReason::WatchdogTimeout => Self::WatchdogTimeout,
            FaultReason::CanTxStalled => Self::CanTxStalled(None),
            FaultReason::J7FeedbackStale => Self::J7FeedbackStale(None),
            FaultReason::JointLimitExceeded { joint_idx, joint_name, value, min, max } => {
                Self::JointLimitExceeded {
                    joint_idx: *joint_idx,
                    joint_name: BoundedText::new(joint_name),
                    value: *value,
                    min: *min,
                    max: *max,
                }
            }
            FaultReason::Other(msg) => Self::Formatted { code, message: BoundedText::new(msg) },
        }
    }

    /// Machine-readable [`FaultCode`] carried by this detail.
    pub const fn code(self, kind: FaultKind) -> FaultCode {
        match self {
            Self::Estop => FaultCode::Estop,
            Self::BusLoss => FaultCode::BusLoss,
            Self::WatchdogTimeout => FaultCode::WatchdogTimeout,
            Self::CanTxStalled(_) => FaultCode::CanTxStalled,
            Self::J7FeedbackStale(_) => FaultCode::J7FeedbackStale,
            Self::ThermalTrip { .. } => FaultCode::ThermalTrip,
            Self::JointLimitExceeded { .. } => FaultCode::JointLimit,
            Self::Static(sf) => sf.code(),
            Self::Formatted { code, .. } => code,
            Self::DetailUnavailable(code) => {
                if matches!(kind, FaultKind::Other) { code } else { kind.default_code() }
            }
        }
    }

    /// Stack-allocated [`BoundedFaultReason`] corresponding to this detail.
    pub fn bounded_reason(self, _kind: FaultKind) -> BoundedFaultReason {
        match self {
            Self::Estop => BoundedFaultReason::Other(BoundedText::new("E-stop latched")),
            Self::BusLoss => BoundedFaultReason::BusLoss,
            Self::WatchdogTimeout => BoundedFaultReason::WatchdogTimeout,
            Self::CanTxStalled(detail) => BoundedFaultReason::CanTxStalled(detail),
            Self::J7FeedbackStale(detail) => BoundedFaultReason::J7FeedbackStale(detail),
            Self::ThermalTrip { joint_idx, temperature, limit } => {
                BoundedFaultReason::ThermalTrip { joint_idx, temperature, limit }
            }
            Self::JointLimitExceeded { joint_idx, joint_name, value, min, max } => {
                BoundedFaultReason::JointLimitExceeded { joint_idx, joint_name, value, min, max }
            }
            Self::Static(sf) => BoundedFaultReason::Other(BoundedText::new(sf.message())),
            Self::Formatted { message, .. } => BoundedFaultReason::Other(message),
            Self::DetailUnavailable(_) => BoundedFaultReason::Other(BoundedText::new(DETAIL_UNAVAILABLE)),
        }
    }
}

/// Last bounded detail captured for one [`FaultKind`] latched since the last clear.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LatchedFaultDetail {
    pub code: FaultCode,
    pub reason: BoundedFaultReason,
}

/// Fixed-capacity UTF-8 text buffer stored inline without heap allocation.
/// When input exceeds `N` bytes, it is cut on a UTF-8 character boundary within `N` bytes and
/// [`Display`] appends `…` (U+2026).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BoundedText<const N: usize> {
    len: u16,
    truncated: bool,
    buf: [u8; N],
}

impl<const N: usize> BoundedText<N> {
    pub const fn empty() -> Self { Self { len: 0, truncated: false, buf: [0u8; N] } }

    pub fn new(s: &str) -> Self {
        let mut out = Self::empty();
        out.push_str(s);
        out
    }

    pub fn push_str(&mut self, s: &str) {
        if s.is_empty() { return; }
        let used = usize::from(self.len);
        let avail = N.saturating_sub(used);
        if avail == 0 {
            self.truncated = true;
            return;
        }
        let mut take = s.len().min(avail);
        while take > 0 && !s.is_char_boundary(take) { take -= 1; }
        if take < s.len() {
            self.truncated = true;
        }
        self.buf[used..used + take].copy_from_slice(&s.as_bytes()[..take]);
        self.len = (used + take) as u16;
    }

    pub fn as_str(&self) -> &str {
        std::str::from_utf8(&self.buf[..usize::from(self.len)]).unwrap_or_default()
    }

    pub fn is_empty(&self) -> bool { self.len == 0 }

    pub fn is_truncated(&self) -> bool { self.truncated }
}

impl<const N: usize> Default for BoundedText<N> { fn default() -> Self { Self::empty() } }
impl<const N: usize> std::ops::Deref for BoundedText<N> {
    type Target = str;
    fn deref(&self) -> &str { self.as_str() }
}
impl<const N: usize> AsRef<str> for BoundedText<N> {
    fn as_ref(&self) -> &str { self.as_str() }
}
impl<const N: usize> From<&str> for BoundedText<N> {
    fn from(s: &str) -> Self { Self::new(s) }
}
impl<const N: usize> From<String> for BoundedText<N> {
    fn from(s: String) -> Self { Self::new(&s) }
}
impl<const N: usize> PartialEq<str> for BoundedText<N> {
    fn eq(&self, other: &str) -> bool { self.as_str() == other }
}
impl<const N: usize> PartialEq<&str> for BoundedText<N> {
    fn eq(&self, other: &&str) -> bool { self.as_str() == *other }
}
impl<const N: usize> PartialEq<String> for BoundedText<N> {
    fn eq(&self, other: &String) -> bool { self.as_str() == other.as_str() }
}
impl<const N: usize> Serialize for BoundedText<N> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        if self.truncated {
            serializer.collect_str(self)
        } else {
            serializer.serialize_str(self.as_str())
        }
    }
}
impl<'de, const N: usize> Deserialize<'de> for BoundedText<N> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let s = String::deserialize(deserializer)?;
        Ok(Self::new(&s))
    }
}
impl<const N: usize> std::fmt::Debug for BoundedText<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { std::fmt::Debug::fmt(self.as_str(), f) }
}
impl<const N: usize> std::fmt::Display for BoundedText<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())?;
        if self.truncated {
            f.write_str("…")?;
        }
        Ok(())
    }
}
impl<const N: usize> core::fmt::Write for BoundedText<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result { self.push_str(s); Ok(()) }
}

/// Stack-allocated mirror of [`FaultReason`] for zero-alloc deferred safety logging.
#[derive(Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum BoundedFaultReason {
    JointLimitExceeded { joint_idx: usize, joint_name: BoundedText<SAFETY_JOINT_NAME_CAPACITY>, value: f32, min: f32, max: f32 },
    ThermalTrip { joint_idx: usize, temperature: f32, limit: f32 },
    BusLoss,
    WatchdogTimeout,
    CanTxStalled(Option<TxStallDetail>),
    J7FeedbackStale(Option<J7StaleDetail>),
    Other(BoundedText<SAFETY_LOG_TEXT_CAPACITY>),
}

impl BoundedFaultReason {
    pub fn from_reason(reason: &FaultReason) -> Self {
        match reason {
            FaultReason::JointLimitExceeded { joint_idx, joint_name, value, min, max } => Self::JointLimitExceeded {
                joint_idx: *joint_idx, joint_name: BoundedText::new(joint_name), value: *value, min: *min, max: *max,
            },
            FaultReason::ThermalTrip { joint_idx, temperature, limit } =>
                Self::ThermalTrip { joint_idx: *joint_idx, temperature: *temperature, limit: *limit },
            FaultReason::BusLoss => Self::BusLoss,
            FaultReason::WatchdogTimeout => Self::WatchdogTimeout,
            FaultReason::CanTxStalled => Self::CanTxStalled(None),
            FaultReason::J7FeedbackStale => Self::J7FeedbackStale(None),
            FaultReason::Other(m) => Self::Other(BoundedText::new(m)),
        }
    }

    /// Writes the human-readable fault message into `w` without heap allocation.
    pub fn write_message_into(&self, w: &mut impl core::fmt::Write) {
        match self {
            Self::JointLimitExceeded { joint_name, value, min, max, .. } => {
                let _ = write!(w, "Joint '{joint_name}' at {value:.4} outside [{min:.4}, {max:.4}]");
            }
            Self::ThermalTrip { joint_idx, temperature, limit } => {
                let _ = write!(w, "Joint {} temperature {temperature:.1} C above limit {limit:.1} C", joint_idx + 1);
            }
            Self::BusLoss => {
                let _ = w.write_str("No fresh feedback from every joint (bus loss)");
            }
            Self::WatchdogTimeout => {
                let _ = w.write_str("Control lease expired while torqued");
            }
            Self::CanTxStalled(Some(detail)) => {
                detail.write_message_into(w);
            }
            Self::CanTxStalled(None) => {
                let _ = w.write_str("CAN TX stalled (bounded transmit queue did not drain within age limit)");
            }
            Self::J7FeedbackStale(Some(detail)) => {
                detail.write_message_into(w);
            }
            Self::J7FeedbackStale(None) => {
                let _ = w.write_str("Joint 7 position-force feedback stale past bounded limit; jaw holding last target");
            }
            Self::Other(m) => {
                let _ = write!(w, "{m}");
            }
        }
    }
}

impl std::fmt::Debug for BoundedFaultReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::JointLimitExceeded { joint_idx, joint_name, value, min, max } => f
                .debug_struct("JointLimitExceeded")
                .field("joint_idx", joint_idx).field("joint_name", &joint_name.as_str())
                .field("value", value).field("min", min).field("max", max).finish(),
            Self::ThermalTrip { joint_idx, temperature, limit } => f
                .debug_struct("ThermalTrip")
                .field("joint_idx", joint_idx).field("temperature", temperature).field("limit", limit).finish(),
            Self::BusLoss => f.write_str("BusLoss"),
            Self::WatchdogTimeout => f.write_str("WatchdogTimeout"),
            Self::CanTxStalled(_) => f.write_str("CanTxStalled"),
            Self::J7FeedbackStale(_) => f.write_str("J7FeedbackStale"),
            Self::Other(m) => f.debug_tuple("Other").field(&m.as_str()).finish(),
        }
    }
}

/// Stack-allocated mirror of [`SafetyState`] for zero-alloc deferred safety logging.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum BoundedSafetyState {
    Booting, Idle, Armed, Moving, EStop, Fault(BoundedFaultReason),
}

impl BoundedSafetyState {
    pub fn from_state(state: &SafetyState) -> Self {
        match state {
            SafetyState::Booting => Self::Booting,
            SafetyState::Idle => Self::Idle,
            SafetyState::Armed => Self::Armed,
            SafetyState::Moving => Self::Moving,
            SafetyState::EStop => Self::EStop,
            SafetyState::Fault(reason) => Self::Fault(BoundedFaultReason::from_reason(reason)),
        }
    }
}

/// Bounded, heap-free safety log event deferred from the control thread to an off-thread consumer.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum SafetyLogEvent {
    EStop { from: BoundedSafetyState },
    Fault { from: BoundedSafetyState, reason: BoundedFaultReason, code: FaultCode },
    Cleared,
    GravityCompEstopExit,
    GravityCompHoldWriteFailed(BoundedText<160>),
    GravityCompRampComplete,
    WatchdogParkBlocked(BoundedText<160>),
    CollisionHoldReleased,
    CollisionTrip { joint: u8, residual: f32, threshold: f32, trips: u32, window_s: f64 },
    CollisionEscalated { trips: u32, window_s: f64 },
    StoppedHoldOutputFailed {
        partial: bool,
        suppressed: u64,
        message: BoundedText<160>,
        logged_at: std::time::Instant,
    },
    MitTrackingCancelled(BoundedText<SAFETY_LOG_TEXT_CAPACITY>),
    MitDeadlineNearMiss {
        count: u64,
        worst: std::time::Duration,
        deadline: std::time::Duration,
    },
}

impl SafetyLogEvent {
    /// Single authoritative formatting for rate-limited stopped-hold output failure logs
    /// (shared by Legacy direct logging and RT Executor off-thread deferred logging).
    pub fn emit_stopped_hold_output_failed(partial: bool, suppressed: u64, message: &str) {
        match (partial, suppressed) {
            (true, 0) => tracing::error!(
                target: "armd::supervisor",
                suppressed,
                "Stopped hold output partially failed: {message}"
            ),
            (true, n) => tracing::error!(
                target: "armd::supervisor",
                suppressed = n,
                "Stopped hold output partially failed (suppressed {n} events): {message}"
            ),
            (false, 0) => tracing::error!(
                target: "armd::supervisor",
                suppressed,
                "Stopped hold output failed: {message}"
            ),
            (false, n) => tracing::error!(
                target: "armd::supervisor",
                suppressed = n,
                "Stopped hold output failed (suppressed {n} events): {message}"
            ),
        }
    }

    /// Emits the exact tracing event on an off-RT thread.
    pub fn emit(&self) {
        match self {
            Self::EStop { from } => tracing::warn!(target: "armd::safety", "Safety: {:?} -> EStop", from),
            Self::Fault { from, reason, code } =>
                tracing::warn!(target: "armd::safety", "Safety: {:?} -> Fault({:?}) [{code:?}]", from, reason),
            Self::Cleared => tracing::info!(target: "armd::safety", "Safety: cleared -> Idle"),
            Self::GravityCompEstopExit => tracing::warn!(target: "armd::supervisor::motion", "Gravity compensation exit: ESTOP E-stop"),
            Self::GravityCompHoldWriteFailed(err) => tracing::error!(target: "armd::supervisor::motion", "Gravity comp hold write failed: {err}"),
            Self::GravityCompRampComplete => tracing::info!(target: "armd::gravity_comp", "Gravity compensation ramp complete"),
            Self::WatchdogParkBlocked(reason) => tracing::warn!(target: "armd::supervisor", "Watchdog: lease expired while torqued, not parking ({reason}); holding torque and latching WatchdogTimeout"),
            Self::CollisionHoldReleased => tracing::info!(target: "armd::collision", "Collision hold ended after quiet period"),
            Self::CollisionTrip { joint, residual, threshold, trips, window_s } => tracing::warn!(
                target: "armd::collision",
                "Collision on joint {}: residual {residual:.2} N·m above threshold {threshold:.2} N·m (trip {trips} within {window_s:.0} s); protective stop",
                usize::from(*joint) + 1
            ),
            Self::CollisionEscalated { trips, window_s } => tracing::error!(
                target: "armd::collision",
                "Collision guard: {trips} trips within {window_s:.0} s; latching Fault COLLISION"
            ),
            Self::StoppedHoldOutputFailed { partial, suppressed, message, .. } => {
                Self::emit_stopped_hold_output_failed(*partial, *suppressed, message.as_str());
            }
            Self::MitTrackingCancelled(reason) => {
                tracing::error!(target: "armd::mit", "MIT tracking cancelled: {reason}");
            }
            Self::MitDeadlineNearMiss { count, worst, deadline } => {
                tracing::warn!(
                    target: "armd::mit",
                    "MIT deadline near miss #{count}: {worst:?} of {deadline:?} (feedback age or output gap)"
                );
            }
        }
    }
}

/// Maximum per-joint resting-pose support violations captured inline without heap allocation.
pub const MAX_RESTING_SUPPORT_ITEMS: usize = 32;

/// One per-joint or total-deviation resting-pose support violation captured on the control thread.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RestingSupportJointItem {
    JointMoving { joint: u16, velocity: f32, limit: f32 },
    JointOffHome { joint: u16, deviation: f32, limit: f32 },
    TotalDeviation { total: f32, limit: f32 },
    JointTorque { joint: u16, torque: f32, limit: f32 },
    JointLoad { joint: u16, load: f32, limit: f32 },
}

impl std::fmt::Display for RestingSupportJointItem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match *self {
            Self::JointMoving { joint, velocity, limit } =>
                write!(f, "joint {} moving at {velocity:.4} rad/s (limit {limit})", usize::from(joint) + 1),
            Self::JointOffHome { joint, deviation, limit } =>
                write!(f, "joint {} off home by {deviation:.4} rad (limit {limit})", usize::from(joint) + 1),
            Self::TotalDeviation { total, limit } =>
                write!(f, "total deviation {total:.4} rad (limit {limit})"),
            Self::JointTorque { joint, torque, limit } =>
                write!(f, "joint {} torque {torque:.2} Nm (limit {limit})", usize::from(joint) + 1),
            Self::JointLoad { joint, load, limit } =>
                write!(f, "joint {} load {load:.2} (limit {limit})", usize::from(joint) + 1),
        }
    }
}

/// Bounded workspace violation captured during resting-pose support verification.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum WorkspaceSupportViolation {
    EvalFailed { limit: BoundedText<32>, reason: BoundedText<128> },
    Constraint {
        limit: BoundedText<160>,
        point: usize,
        cid: BoundedText<160>,
        gid: BoundedText<80>,
        depth: f64,
        value: f64,
        bound: f64,
    },
}

impl std::fmt::Display for WorkspaceSupportViolation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EvalFailed { limit, reason } =>
                write!(f, "measured pose workspace evaluation failed ({limit}): {reason}"),
            Self::Constraint { limit, point, cid, gid, depth, value, bound } => {
                write!(f, "measured pose violates workspace constraint '{limit}' (point {point} [{gid}]: ")?;
                if !depth.is_finite() {
                    write!(f, "invalid workspace check for {cid} ({gid})")?;
                } else {
                    write!(f, "new intrusion into {cid} by {depth:.3}m (geometry {gid})")?;
                }
                write!(f, ", value {value:.4} m vs bound {bound:.4} m)")
            }
        }
    }
}

/// Typed, bounded resting-pose support violation rendered off-thread to identical API text.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum RestingSupportViolation {
    FeedbackLengthMismatch,
    NoRestingPoseConfigured,
    DriverFeedbackUnavailable,
    DriverLacksVelocityFeedback,
    DriverLacksTorqueFeedback,
    DriverLacksLoadFeedback,
    FeedbackTooOld { max_age_ms: u64 },
    SupportTorqueIndexOutOfRange { index: usize },
    SupportLoadIndexOutOfRange { index: usize },
    Failures {
        items: [Option<RestingSupportJointItem>; MAX_RESTING_SUPPORT_ITEMS],
        len: u8,
        overflowed: u16,
        workspace: Option<WorkspaceSupportViolation>,
    },
}

impl std::fmt::Display for RestingSupportViolation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::FeedbackLengthMismatch => f.write_str("feedback length does not match dof"),
            Self::NoRestingPoseConfigured => f.write_str("no resting pose configured"),
            Self::DriverFeedbackUnavailable => f.write_str("driver feedback unavailable"),
            Self::DriverLacksVelocityFeedback => f.write_str("driver lacks velocity feedback"),
            Self::DriverLacksTorqueFeedback => f.write_str("driver lacks torque feedback for support torque limits"),
            Self::DriverLacksLoadFeedback => f.write_str("driver lacks load feedback for support load limits"),
            Self::FeedbackTooOld { max_age_ms } => write!(f, "feedback older than {max_age_ms} ms"),
            Self::SupportTorqueIndexOutOfRange { index } => write!(f, "support torque index {index} out of range"),
            Self::SupportLoadIndexOutOfRange { index } => write!(f, "support load index {index} out of range"),
            Self::Failures { items, len, overflowed, workspace } => {
                let mut first = true;
                for item in items.iter().take(usize::from(*len)).flatten() {
                    if !first { f.write_str("; ")?; }
                    first = false;
                    write!(f, "{item}")?;
                }
                if let Some(ws) = workspace {
                    if !first { f.write_str("; ")?; }
                    write!(f, "{ws}")?;
                }
                if *overflowed > 0 {
                    write!(f, " (+{overflowed} more)")?;
                }
                Ok(())
            }
        }
    }
}

/// Typed, bounded driver error captured during RT `clear_fault`, rendered off-thread to exact
/// `ArmDriverError::to_string()` text.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum ClearFaultDriverError {
    TransportTimeout,
    TransportWouldBlock,
    TransportNotOpen,
    TransportChecksumMismatch { expected: u8, actual: u8 },
    TransportFrameTooShort { expected: usize, actual: usize },
    JointCountMismatch { expected: usize, actual: usize },
    JointTimeout(u8),
    EstopInhibited,
    Protocol(BoundedText<256>),
    HardwareFault(BoundedText<256>),
    TransportFormatted(BoundedText<256>),
    Formatted(BoundedText<256>),
}

impl std::fmt::Display for ClearFaultDriverError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TransportTimeout => f.write_str("Transport error: Timeout error"),
            Self::TransportWouldBlock => f.write_str("Transport error: Operation would block"),
            Self::TransportNotOpen => f.write_str("Transport error: Device not open"),
            Self::TransportChecksumMismatch { expected, actual } =>
                write!(f, "Transport error: Checksum mismatch: expected 0x{expected:02X}, calculated 0x{actual:02X}"),
            Self::TransportFrameTooShort { expected, actual } =>
                write!(f, "Transport error: Frame too short: expected at least {expected} bytes, got {actual}"),
            Self::JointCountMismatch { expected, actual } =>
                write!(f, "Joint count mismatch: expected {expected}, got {actual}"),
            Self::JointTimeout(id) => write!(f, "Timeout waiting for response from joint {id}"),
            Self::EstopInhibited => f.write_str("E-stop requested during the target batch; remaining frames not sent"),
            Self::Protocol(msg) => write!(f, "Protocol error: {msg}"),
            Self::HardwareFault(msg) => write!(f, "Hardware fault: {msg}"),
            Self::TransportFormatted(msg) => write!(f, "Transport error: {msg}"),
            Self::Formatted(msg) => write!(f, "{msg}"),
        }
    }
}

/// Structured, allocation-free description of the first joint violating `JointLimit` clear bounds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JointLimitClearViolation {
    /// Telemetry length mismatch or missing configured limits.
    FeedbackIncomplete { measured_len: usize, dof: usize },
    /// Joint reported a non-finite position (`NaN` / `±Inf`).
    NonFinite {
        joint_idx: u16,
        joint_name: BoundedText<SAFETY_JOINT_NAME_CAPACITY>,
        value: f32,
    },
    /// Measured joint position lies outside `[min - margin, max + margin]`.
    OutOfBounds {
        joint_idx: u16,
        joint_name: BoundedText<SAFETY_JOINT_NAME_CAPACITY>,
        value: f32,
        min: f32,
        max: f32,
        margin: f32,
    },
}

impl std::fmt::Display for JointLimitClearViolation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::FeedbackIncomplete { measured_len, dof } => write!(
                f,
                "Measured joint positions incomplete ({measured_len} of {dof} joints); all joints must be back within limits before clearing JointLimit"
            ),
            Self::NonFinite { joint_idx, joint_name, value } => write!(
                f,
                "Joint '{joint_name}' (joint {}) position {value} rad is non-finite; restore valid feedback within limits before clearing JointLimit",
                joint_idx + 1
            ),
            Self::OutOfBounds { joint_idx, joint_name, value, min, max, margin } => write!(
                f,
                "Joint '{joint_name}' (joint {}) at {value:.4} rad is outside limits [{min:.4}, {max:.4}] rad (clear margin {margin:.2} rad); move it back within limits by hand before clearing JointLimit",
                joint_idx + 1
            ),
        }
    }
}

/// Typed, bounded `clear_fault` refusal carrying zero heap allocations on the control thread;
/// rendered off-thread to the exact `SafetyError::ClearFaultRejected(String)` text.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum ClearRefusal {
    EpochMismatch { expected: u64, received: u64 },
    EStopActive,
    ShutdownCommitted,
    DriverConfigError(BoundedText<256>),
    ConfigPersistenceFailed(BoundedText<256>),
    GravityCompUnconfirmed,
    DisablePending,
    MitLostUnsupported,
    MitModesUncertain,
    FreshFeedbackRequiredForRestingSupport,
    RestingPoseSupport(RestingSupportViolation),
    BudgetExpired,
    HardwareResetRequired,
    HardwareResetSent,
    FreshFeedbackRequired,
    J7FeedbackStale,
    TxQueueNotDrained,
    TxProbeRequired,
    TxProbeFeedbackRequired,
    TemperatureConfirmationRequired,
    TemperatureAboveThreshold,
    ThermalCooldownNotElapsed { elapsed_ms: u64, required_ms: u64 },
    WatchdogHeartbeatRequired,
    JointLimitsNotSatisfied(JointLimitClearViolation),
    EnableStateUnknown,
    ReclaimQueueFull,
    UnknownFaultKind,
    DriverError(ClearFaultDriverError),
}

/// Alias retained for compatibility with callers referencing [`ClearRefusal`] as `ClearFaultRefusal`.
pub type ClearFaultRefusal = ClearRefusal;

impl ClearRefusal {
    /// Renders the exact API error text off the RT control thread.
    pub fn render_message(&self) -> String { self.to_string() }

    /// Converts this refusal into the corresponding [`SafetyError`] off-thread.
    pub fn into_safety_error(self) -> SafetyError {
        match self {
            Self::EpochMismatch { expected, received } => SafetyError::EpochMismatch { expected, received },
            Self::EStopActive => SafetyError::EStopActive,
            other => SafetyError::ClearFaultRejected(other.render_message()),
        }
    }
}

impl std::fmt::Display for ClearRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EpochMismatch { expected, received } =>
                write!(f, "Epoch mismatch: expected {expected}, received {received}. Command invalidated by generation change"),
            Self::EStopActive => f.write_str("E-Stop is latched, motion rejected"),
            Self::ShutdownCommitted => f.write_str("Daemon shutdown is committed"),
            Self::DriverConfigError(err) => write!(f, "Driver limit configuration error: {err}"),
            Self::ConfigPersistenceFailed(err) => write!(f, "Configuration persistence reconciliation failed: {err}"),
            Self::GravityCompUnconfirmed => f.write_str("Gravity compensation: the switch back to POS_VEL is not confirmed yet"),
            Self::DisablePending => f.write_str("Torque disable is still unconfirmed"),
            Self::MitLostUnsupported => f.write_str("UNSUPPORTED: MIT lost recovery requires legacy path"),
            Self::MitModesUncertain => f.write_str("MIT tracking: the control mode of a joint is uncertain after a failed switch; check the arm and power-cycle the motors"),
            Self::FreshFeedbackRequiredForRestingSupport => f.write_str("Fresh feedback is required before checking resting pose support"),
            Self::RestingPoseSupport(violation) =>
                write!(f, "Cannot disable faulted supporting arm joint away from resting pose ({violation}); support the arm at rest first"),
            Self::BudgetExpired => f.write_str("Clear fault budget expired"),
            Self::HardwareResetRequired => f.write_str("Hardware fault bit active; motor reset is required before re-evaluating clear conditions"),
            Self::HardwareResetSent => f.write_str("Hardware reset sent; wait for fresh fault-free feedback before retrying"),
            Self::FreshFeedbackRequired => f.write_str("Fresh feedback is required before resetting a fault"),
            Self::J7FeedbackStale => f.write_str("Fresh J7 position-force feedback (<= 100 ms) is required before clearing J7_FEEDBACK_STALE"),
            Self::TxQueueNotDrained => f.write_str("CAN TX queue still has pending frames; wait for the bus to drain before clearing CAN_TX_STALLED"),
            Self::TxProbeRequired => f.write_str("CAN TX transmit probe is required before clearing CAN_TX_STALLED"),
            Self::TxProbeFeedbackRequired => f.write_str("CAN TX probe sent; wait for fresh feedback received after the probe before clearing CAN_TX_STALLED"),
            Self::TemperatureConfirmationRequired => f.write_str("Temperature must be confirmed below the reset threshold"),
            Self::TemperatureAboveThreshold => f.write_str("Temperature is above the reset threshold"),
            Self::ThermalCooldownNotElapsed { elapsed_ms, required_ms } =>
                write!(f, "Thermal cooldown period not elapsed ({elapsed_ms} ms of {required_ms} ms required below threshold)"),
            Self::WatchdogHeartbeatRequired => f.write_str("Fresh heartbeat from the active lease holder is required before clearing WatchdogTimeout"),
            Self::JointLimitsNotSatisfied(violation) => write!(f, "{violation}"),
            Self::EnableStateUnknown => f.write_str("Hardware enable state is unknown; reset remains latched"),
            Self::ReclaimQueueFull => f.write_str("Fault state reclamation queue is full; retry after off-thread drain"),
            Self::UnknownFaultKind => f.write_str("Unknown latched fault kind cannot be cleared (fail closed)"),
            Self::DriverError(err) => write!(f, "{err}"),
        }
    }
}

/// Snapshot of live supervisor and hardware readiness evaluated by [`SafetyManager::clear_check`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub struct ClearContext {
    pub expected_epoch: Option<u64>,
    pub pending_estop: bool,
    pub shutdown_committed: bool,
    pub driver_config_error: Option<BoundedText<256>>,
    pub config_reconcile_fault: Option<BoundedText<256>>,
    pub gravity_comp_unconfirmed: bool,
    pub disable_pending: bool,
    pub has_driver: bool,
    pub feedback_fresh: bool,
    /// True when J7 position-force feedback is provably fresh (<= 100 ms on both receive
    /// instant and receive-queue bound, finite position). Required to clear `J7FeedbackStale`.
    pub j7_feedback_fresh: bool,
    /// True when the driver's bounded TX carry-over queues (motion + stopped-hold) are empty.
    pub tx_queue_drained: bool,
    /// True when a CAN transmit probe (refresh/status frame sent with no backpressure or error
    /// after an empty queue) has been confirmed by fresh feedback received after that probe.
    pub tx_probe_confirmed: bool,
    /// True when a CAN transmit probe was sent cleanly and is awaiting post-probe fresh feedback.
    pub tx_probe_sent: bool,
    pub thermal_sensor_available: bool,
    pub thermal_below_threshold: bool,
    pub thermal_cooldown_elapsed_ms: u64,
    pub thermal_cooldown_required_ms: u64,
    pub has_active_lease: bool,
    pub lease_heartbeat_fresh: bool,
    pub joint_limit_violation: Option<JointLimitClearViolation>,
    pub enable_state_known: bool,
    /// True when the driver reports a latched motor fault bit that requires a motor reset
    /// before it can clear. `clear_check` permits this reset ONLY after every OTHER recorded
    /// kind's condition has passed, returning [`ClearRefusal::HardwareResetRequired`].
    pub hardware_reset_required: bool,
    /// True immediately after a permitted motor reset has been transmitted; `clear_check`
    /// returns [`ClearRefusal::HardwareResetSent`] until fresh fault-free feedback confirms recovery.
    pub hardware_reset_sent: bool,
    pub hardware_error: Option<ClearFaultDriverError>,
    pub resting_pose_violation: Option<RestingSupportViolation>,
    pub resting_feedback_missing: bool,
    pub mit_modes_uncertain: bool,
    pub mit_lost_unsupported: bool,
}

impl ClearContext {
    /// Context with every clear precondition satisfied (no driver, cool, within limits, no lease).
    pub const fn all_conditions_met() -> Self {
        Self {
            expected_epoch: None,
            pending_estop: false,
            shutdown_committed: false,
            driver_config_error: None,
            config_reconcile_fault: None,
            gravity_comp_unconfirmed: false,
            disable_pending: false,
            has_driver: false,
            feedback_fresh: true,
            j7_feedback_fresh: true,
            tx_queue_drained: true,
            tx_probe_confirmed: true,
            tx_probe_sent: false,
            thermal_sensor_available: true,
            thermal_below_threshold: true,
            thermal_cooldown_elapsed_ms: 0,
            thermal_cooldown_required_ms: 0,
            has_active_lease: false,
            lease_heartbeat_fresh: true,
            joint_limit_violation: None,
            enable_state_known: true,
            hardware_reset_required: false,
            hardware_reset_sent: false,
            hardware_error: None,
            resting_pose_violation: None,
            resting_feedback_missing: false,
            mit_modes_uncertain: false,
            mit_lost_unsupported: false,
        }
    }
}

/// Bounded RT error returned by `clear_fault_executor_bounded`; converted off-thread into [`SafetyError`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum ClearFaultError {
    EStopActive,
    EpochMismatch { expected: u64, received: u64 },
    FeedbackUnavailable,
    Rejected(ClearFaultRefusal),
}

impl std::fmt::Display for ClearFaultError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EStopActive => f.write_str("E-Stop is latched, motion rejected"),
            Self::EpochMismatch { expected, received } =>
                write!(f, "Epoch mismatch: expected {expected}, received {received}. Command invalidated by generation change"),
            Self::FeedbackUnavailable => f.write_str("Actuator feedback unavailable: physical angles not received from hardware"),
            Self::Rejected(r) => write!(f, "Cannot clear fault while hardware fault condition persists: {r}"),
        }
    }
}

impl std::error::Error for ClearFaultError {}
impl From<ClearFaultRefusal> for ClearFaultError {
    fn from(r: ClearFaultRefusal) -> Self {
        match r {
            ClearFaultRefusal::EpochMismatch { expected, received } => Self::EpochMismatch { expected, received },
            ClearFaultRefusal::EStopActive => Self::EStopActive,
            other => Self::Rejected(other),
        }
    }
}
impl From<ClearFaultError> for SafetyError {
    fn from(e: ClearFaultError) -> Self {
        match e {
            ClearFaultError::EStopActive => SafetyError::EStopActive,
            ClearFaultError::EpochMismatch { expected, received } => SafetyError::EpochMismatch { expected, received },
            ClearFaultError::FeedbackUnavailable => SafetyError::FeedbackUnavailable,
            ClearFaultError::Rejected(r) => r.into_safety_error(),
        }
    }
}

/// Emits a best-effort warning off-thread when bounded deferred safety log events were dropped.
/// The monotonic `total` counter remains authoritative even if this warning line itself is dropped.
pub fn emit_safety_log_dropped(delta: u64, total: u64) {
    if delta > 0 {
        tracing::warn!(
            target: "armd::safety",
            "Safety log: {delta} events dropped (safety log queue full; total {total})"
        );
    }
}

/// Emits a best-effort shutdown summary when any safety log events were dropped during the run.
pub fn emit_safety_log_shutdown_summary(total: u64) {
    if total > 0 {
        tracing::warn!(
            target: "armd::safety",
            "Safety log shutdown: {total} total events dropped"
        );
    }
}

/// Safety Manager tracking safety state machine and command generation ID
#[derive(Debug)]
pub struct SafetyManager {
    state_arc: Arc<SafetyState>,
    idle_arc: Arc<SafetyState>,
    armed_arc: Arc<SafetyState>,
    moving_arc: Arc<SafetyState>,
    estop_arc: Arc<SafetyState>,
    can_tx_stalled_arc: Arc<SafetyState>,
    j7_feedback_stale_arc: Arc<SafetyState>,
    bus_loss_arc: Arc<SafetyState>,
    watchdog_timeout_arc: Arc<SafetyState>,
    detail_unavailable_arc: Arc<SafetyState>,
    static_fault_arcs: [Arc<SafetyState>; StaticFault::COUNT],
    fault_arc_pool: [Option<Arc<SafetyState>>; FAULT_ARC_POOL_CAPACITY],
    fault_arc_pool_len: usize,
    formatted_fault_arc_pool: [Option<Arc<SafetyState>>; FORMATTED_FAULT_ARC_POOL_CAPACITY],
    formatted_fault_arc_pool_len: usize,
    joint_limit_arc_pool: [Option<Arc<SafetyState>>; JOINT_LIMIT_ARC_POOL_CAPACITY],
    joint_limit_arc_pool_len: usize,
    retired_state_arcs: [Option<Arc<SafetyState>>; RETIRED_STATE_CAPACITY],
    retired_head: usize,
    retired_len: usize,
    off_thread_reclaim: bool,
    deferred_logging: bool,
    /// Preallocated once at startup (`SAFETY_LOG_CAPACITY` slots) to keep `SafetyManager` small
    /// on debug stacks while performing zero heap allocations/frees on the RT thread.
    log_events: Box<[Option<SafetyLogEvent>]>,
    log_head: usize,
    log_len: usize,
    log_dropped: u64,
    log_dropped_forwarded: u64,
    epoch_id: u64,
    epoch_mirror: Arc<AtomicU64>,
    /// Monotonic counter incremented when `latch` records a new kind or distinct fault code.
    latch_count: u64,
    /// Code recorded by the trigger site of the current shown Fault.
    fault_code: Option<FaultCode>,
    /// Bitmask of every [`FaultKind`] latched since the last clear to `Idle`.
    latched_fault_kinds: u16,
    /// Bitmask of every [`FaultCode`] latched since the last clear to `Idle`.
    latched_fault_codes: u32,
    /// Sticky worst-of recoverability flag: set whenever ANY latched detail is non-recoverable,
    /// cleared only when the record is reset on clear.
    has_non_recoverable: bool,
    /// Fixed-size preallocated record of the representative detail per latched [`FaultKind`]
    /// (`FaultKind::COUNT` slots, allocated once at startup; zero RT heap ops).
    latched_fault_details: Box<[Option<LatchedFaultDetail>]>,
}

impl Clone for SafetyManager {
    fn clone(&self) -> Self {
        Self {
            state_arc: Arc::clone(&self.state_arc),
            idle_arc: Arc::clone(&self.idle_arc),
            armed_arc: Arc::clone(&self.armed_arc),
            moving_arc: Arc::clone(&self.moving_arc),
            estop_arc: Arc::clone(&self.estop_arc),
            can_tx_stalled_arc: Arc::clone(&self.can_tx_stalled_arc),
            j7_feedback_stale_arc: Arc::clone(&self.j7_feedback_stale_arc),
            bus_loss_arc: Arc::clone(&self.bus_loss_arc),
            watchdog_timeout_arc: Arc::clone(&self.watchdog_timeout_arc),
            detail_unavailable_arc: Arc::clone(&self.detail_unavailable_arc),
            static_fault_arcs: std::array::from_fn(|i| Arc::clone(&self.static_fault_arcs[i])),
            // Fresh unique pool slots so `Arc::get_mut` always succeeds on both instances.
            fault_arc_pool: std::array::from_fn(|_| Some(Arc::new(SafetyState::Idle))),
            fault_arc_pool_len: FAULT_ARC_POOL_CAPACITY,
            formatted_fault_arc_pool: std::array::from_fn(|_| {
                Some(Arc::new(SafetyState::Fault(FaultReason::Other(
                    String::with_capacity(SAFETY_LOG_TEXT_CAPACITY),
                ))))
            }),
            formatted_fault_arc_pool_len: FORMATTED_FAULT_ARC_POOL_CAPACITY,
            joint_limit_arc_pool: std::array::from_fn(|_| {
                Some(Arc::new(SafetyState::Fault(FaultReason::JointLimitExceeded {
                    joint_idx: 0,
                    joint_name: String::with_capacity(SAFETY_JOINT_NAME_CAPACITY),
                    value: 0.0,
                    min: 0.0,
                    max: 0.0,
                })))
            }),
            joint_limit_arc_pool_len: JOINT_LIMIT_ARC_POOL_CAPACITY,
            retired_state_arcs: self.retired_state_arcs.clone(),
            retired_head: self.retired_head,
            retired_len: self.retired_len,
            off_thread_reclaim: self.off_thread_reclaim,
            deferred_logging: self.deferred_logging,
            log_events: self.log_events.clone(),
            log_head: self.log_head,
            log_len: self.log_len,
            log_dropped: self.log_dropped,
            log_dropped_forwarded: self.log_dropped_forwarded,
            epoch_id: self.epoch_id,
            epoch_mirror: Arc::new(AtomicU64::new(self.epoch_id)),
            latch_count: self.latch_count,
            fault_code: self.fault_code,
            latched_fault_kinds: self.latched_fault_kinds,
            latched_fault_codes: self.latched_fault_codes,
            has_non_recoverable: self.has_non_recoverable,
            latched_fault_details: self.latched_fault_details.clone(),
        }
    }
}

impl Default for SafetyManager {
    fn default() -> Self {
        let idle_arc = Arc::new(SafetyState::Idle);
        Self {
            state_arc: Arc::clone(&idle_arc),
            idle_arc,
            armed_arc: Arc::new(SafetyState::Armed),
            moving_arc: Arc::new(SafetyState::Moving),
            estop_arc: Arc::new(SafetyState::EStop),
            can_tx_stalled_arc: Arc::new(SafetyState::Fault(FaultReason::CanTxStalled)),
            j7_feedback_stale_arc: Arc::new(SafetyState::Fault(FaultReason::J7FeedbackStale)),
            bus_loss_arc: Arc::new(SafetyState::Fault(FaultReason::BusLoss)),
            watchdog_timeout_arc: Arc::new(SafetyState::Fault(FaultReason::WatchdogTimeout)),
            detail_unavailable_arc: Arc::new(SafetyState::Fault(FaultReason::Other(
                DETAIL_UNAVAILABLE.into(),
            ))),
            static_fault_arcs: std::array::from_fn(|i| {
                Arc::new(SafetyState::Fault(FaultReason::Other(
                    StaticFault::ALL[i].message().into(),
                )))
            }),
            fault_arc_pool: std::array::from_fn(|_| Some(Arc::new(SafetyState::Idle))),
            fault_arc_pool_len: FAULT_ARC_POOL_CAPACITY,
            formatted_fault_arc_pool: std::array::from_fn(|_| {
                Some(Arc::new(SafetyState::Fault(FaultReason::Other(
                    String::with_capacity(SAFETY_LOG_TEXT_CAPACITY),
                ))))
            }),
            formatted_fault_arc_pool_len: FORMATTED_FAULT_ARC_POOL_CAPACITY,
            joint_limit_arc_pool: std::array::from_fn(|_| {
                Some(Arc::new(SafetyState::Fault(FaultReason::JointLimitExceeded {
                    joint_idx: 0,
                    joint_name: String::with_capacity(SAFETY_JOINT_NAME_CAPACITY),
                    value: 0.0,
                    min: 0.0,
                    max: 0.0,
                })))
            }),
            joint_limit_arc_pool_len: JOINT_LIMIT_ARC_POOL_CAPACITY,
            retired_state_arcs: std::array::from_fn(|_| None),
            retired_head: 0,
            retired_len: 0,
            off_thread_reclaim: false,
            deferred_logging: false,
            log_events: vec![None; SAFETY_LOG_CAPACITY].into_boxed_slice(),
            log_head: 0,
            log_len: 0,
            log_dropped: 0,
            log_dropped_forwarded: 0,
            epoch_id: 1,
            epoch_mirror: Arc::new(AtomicU64::new(1)),
            latch_count: 0,
            fault_code: None,
            latched_fault_kinds: 0,
            latched_fault_codes: 0,
            has_non_recoverable: false,
            latched_fault_details: vec![None; FaultKind::COUNT].into_boxed_slice(),
        }
    }
}

impl SafetyManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn state(&self) -> &SafetyState {
        &self.state_arc
    }

    /// Cheap reference-counted snapshot of the current safety state (0 heap allocations).
    pub fn state_arc(&self) -> Arc<SafetyState> {
        Arc::clone(&self.state_arc)
    }

    /// Enables bounded off-thread fault state reclamation and deferred safety log formatting
    /// (fail-closed when full) for RT executor mode.
    pub fn set_off_thread_reclaim(&mut self, enabled: bool) {
        self.off_thread_reclaim = enabled;
        self.deferred_logging = enabled;
    }

    /// Enables or disables deferring safety transition log formatting to an off-thread consumer.
    pub fn set_deferred_logging(&mut self, enabled: bool) {
        self.deferred_logging = enabled;
    }

    /// True when safety transition log formatting is deferred through bounded events.
    pub fn deferred_logging(&self) -> bool {
        self.deferred_logging
    }

    /// Number of queued deferred safety log events awaiting off-thread emission.
    pub fn log_events_len(&self) -> usize {
        self.log_len
    }

    /// Pops the oldest deferred [`SafetyLogEvent`] for off-thread emission (0 alloc, 0 drop).
    pub fn pop_log_event(&mut self) -> Option<SafetyLogEvent> {
        if self.log_len == 0 {
            return None;
        }
        let ev = self.log_events[self.log_head].take();
        self.log_head = (self.log_head + 1) % SAFETY_LOG_CAPACITY;
        self.log_len -= 1;
        ev
    }

    /// Monotonic total of deferred safety log events dropped because the inline ring was full
    /// (never reset).
    #[inline]
    pub fn log_dropped(&self) -> u64 {
        self.log_dropped
    }

    /// Returns newly dropped inline safety log events since the previous forward call, without
    /// ever resetting [`Self::log_dropped`].
    pub fn advance_log_dropped_forwarded(&mut self) -> u64 {
        let delta = self.log_dropped.saturating_sub(self.log_dropped_forwarded);
        self.log_dropped_forwarded = self.log_dropped;
        delta
    }

    /// Emits all queued deferred safety log events on a non-RT thread.
    pub fn drain_log_events_off_thread(&mut self) {
        let lost = self.advance_log_dropped_forwarded();
        emit_safety_log_dropped(lost, self.log_dropped);
        while let Some(ev) = self.pop_log_event() {
            ev.emit();
        }
    }

    fn queue_log_event(&mut self, ev: SafetyLogEvent) {
        if self.log_len >= SAFETY_LOG_CAPACITY {
            self.log_dropped = self.log_dropped.saturating_add(1);
            return;
        }
        let tail = (self.log_head + self.log_len) % SAFETY_LOG_CAPACITY;
        self.log_events[tail] = Some(ev);
        self.log_len += 1;
    }

    /// Number of preallocated clean `Arc<SafetyState>` fault slots currently available.
    pub fn fault_arc_pool_len(&self) -> usize {
        self.fault_arc_pool_len
    }

    /// Number of preallocated formatted-message `Arc<SafetyState>` fault slots currently available.
    pub fn formatted_fault_arc_pool_len(&self) -> usize {
        self.formatted_fault_arc_pool_len
    }

    /// Number of preallocated `JointLimitExceeded` `Arc<SafetyState>` fault slots currently available.
    pub fn joint_limit_arc_pool_len(&self) -> usize {
        self.joint_limit_arc_pool_len
    }

    /// Monotonic count of new/upgraded latches recorded by [`Self::latch`].
    #[inline]
    pub fn latch_count(&self) -> u64 {
        self.latch_count
    }

    /// Resets a uniquely-owned retired `SafetyState` off-thread for recycling into either the
    /// formatted-message pool (`Other(String::with_capacity(SAFETY_LOG_TEXT_CAPACITY))`), the
    /// joint-limit pool (`JointLimitExceeded`), or the general pool (`Idle`).
    pub fn reset_recycled_fault_arc(slot: &mut SafetyState) {
        match slot {
            SafetyState::Fault(FaultReason::Other(s)) => {
                if s.capacity() >= SAFETY_LOG_TEXT_CAPACITY && s.capacity() <= SAFETY_LOG_TEXT_CAPACITY * 2 {
                    s.clear();
                    return;
                }
                *slot = SafetyState::Fault(FaultReason::Other(String::with_capacity(SAFETY_LOG_TEXT_CAPACITY)));
            }
            SafetyState::Fault(FaultReason::JointLimitExceeded { joint_idx, joint_name, value, min, max }) => {
                if joint_name.capacity() >= SAFETY_JOINT_NAME_CAPACITY
                    && joint_name.capacity() <= SAFETY_JOINT_NAME_CAPACITY * 2
                {
                    joint_name.clear();
                    *joint_idx = 0;
                    *value = 0.0;
                    *min = 0.0;
                    *max = 0.0;
                    return;
                }
                *slot = SafetyState::Fault(FaultReason::JointLimitExceeded {
                    joint_idx: 0,
                    joint_name: String::with_capacity(SAFETY_JOINT_NAME_CAPACITY),
                    value: 0.0,
                    min: 0.0,
                    max: 0.0,
                });
            }
            _ => {
                *slot = SafetyState::Idle;
            }
        }
    }

    /// Emits or defers the gravity-compensation E-stop exit warning without RT heap allocation.
    pub fn log_gravity_comp_estop_exit(&mut self) {
        if self.deferred_logging {
            self.queue_log_event(SafetyLogEvent::GravityCompEstopExit);
        } else {
            tracing::warn!(target: "armd::supervisor::motion", "Gravity compensation exit: ESTOP E-stop");
        }
    }

    /// Emits or defers a gravity-compensation hold-write failure without RT heap allocation.
    pub fn log_gravity_comp_hold_write_failed(&mut self, msg: BoundedText<160>) {
        if self.deferred_logging {
            self.queue_log_event(SafetyLogEvent::GravityCompHoldWriteFailed(msg));
        } else {
            tracing::error!(target: "armd::supervisor::motion", "Gravity comp hold write failed: {msg}");
        }
    }

    /// Emits or defers the gravity-compensation ramp completion notice without RT heap allocation.
    pub fn log_gravity_comp_ramp_complete(&mut self) {
        if self.deferred_logging {
            self.queue_log_event(SafetyLogEvent::GravityCompRampComplete);
        } else {
            tracing::info!(target: "armd::gravity_comp", "Gravity compensation ramp complete");
        }
    }

    /// Emits or defers the collision hold release notice without RT heap allocation.
    pub fn log_collision_hold_released(&mut self) {
        if self.deferred_logging {
            self.queue_log_event(SafetyLogEvent::CollisionHoldReleased);
        } else {
            tracing::info!(target: "armd::collision", "Collision hold ended after quiet period");
        }
    }

    /// Emits or defers a collision protective-stop warning without RT heap allocation.
    pub fn log_collision_trip(&mut self, joint: u8, residual: f32, threshold: f32, trips: u32, window_s: f64) {
        if self.deferred_logging {
            self.queue_log_event(SafetyLogEvent::CollisionTrip { joint, residual, threshold, trips, window_s });
        } else {
            tracing::warn!(
                target: "armd::collision",
                "Collision on joint {}: residual {residual:.2} N·m above threshold {threshold:.2} N·m (trip {trips} within {window_s:.0} s); protective stop",
                usize::from(joint) + 1
            );
        }
    }

    /// Emits or defers a collision escalation error without RT heap allocation.
    pub fn log_collision_escalated(&mut self, trips: u32, window_s: f64) {
        if self.deferred_logging {
            self.queue_log_event(SafetyLogEvent::CollisionEscalated { trips, window_s });
        } else {
            tracing::error!(
                target: "armd::collision",
                "Collision guard: {trips} trips within {window_s:.0} s; latching Fault COLLISION"
            );
        }
    }

    /// Emits or defers the watchdog-park-blocked warning without RT heap allocation.
    pub fn log_watchdog_park_blocked(&mut self, reason: BoundedText<160>) {
        if self.deferred_logging {
            self.queue_log_event(SafetyLogEvent::WatchdogParkBlocked(reason));
        } else {
            tracing::warn!(
                target: "armd::supervisor",
                "Watchdog: lease expired while torqued, not parking ({reason}); holding torque and latching WatchdogTimeout"
            );
        }
    }

    /// Emits or defers an MIT tracking cancellation error without RT heap allocation.
    pub fn log_mit_tracking_cancelled(&mut self, reason: BoundedText<SAFETY_LOG_TEXT_CAPACITY>) {
        if self.deferred_logging {
            self.queue_log_event(SafetyLogEvent::MitTrackingCancelled(reason));
        } else {
            tracing::error!(target: "armd::mit", "MIT tracking cancelled: {reason}");
        }
    }

    /// Emits or defers an MIT deadline near-miss warning without RT heap allocation.
    pub fn log_mit_deadline_near_miss(
        &mut self,
        count: u64,
        worst: std::time::Duration,
        deadline: std::time::Duration,
    ) {
        if self.deferred_logging {
            self.queue_log_event(SafetyLogEvent::MitDeadlineNearMiss { count, worst, deadline });
        } else {
            tracing::warn!(
                target: "armd::mit",
                "MIT deadline near miss #{count}: {worst:?} of {deadline:?} (feedback age or output gap)"
            );
        }
    }

    /// Emits or defers a rate-limited stopped-hold output error with exact suppressed-event count
    /// (zero RT heap allocation).
    /// Returns `true` when emitted or queued/coalesced into the deferred ring, or `false` when
    /// the deferred ring is full of unrelated safety events so the caller retains the count.
    pub fn log_stopped_hold_output_failed(
        &mut self,
        partial: bool,
        suppressed: u64,
        message: BoundedText<160>,
        logged_at: std::time::Instant,
    ) -> bool {
        if !self.deferred_logging {
            SafetyLogEvent::emit_stopped_hold_output_failed(partial, suppressed, message.as_str());
            return true;
        }
        if self.log_len < SAFETY_LOG_CAPACITY {
            let tail = (self.log_head + self.log_len) % SAFETY_LOG_CAPACITY;
            self.log_events[tail] = Some(SafetyLogEvent::StoppedHoldOutputFailed {
                partial,
                suppressed,
                message,
                logged_at,
            });
            self.log_len += 1;
            return true;
        }
        // Ring is full: coalesce into an existing queued StoppedHoldOutputFailed slot if present
        // so suppressed counts are never silently dropped.
        for offset in 0..self.log_len {
            let idx = (self.log_head + offset) % SAFETY_LOG_CAPACITY;
            if let Some(SafetyLogEvent::StoppedHoldOutputFailed {
                partial: prev_partial,
                suppressed: prev_suppressed,
                message: prev_msg,
                logged_at: prev_at,
            }) = self.log_events[idx].as_mut()
            {
                *prev_partial = partial;
                *prev_suppressed = prev_suppressed
                    .saturating_add(1)
                    .saturating_add(suppressed);
                *prev_msg = message;
                *prev_at = logged_at;
                return true;
            }
        }
        false
    }

    /// True when `kind` has been latched since the last successful clear to `Idle`.
    #[inline]
    pub fn has_latched_fault_kind(&self, kind: FaultKind) -> bool {
        (self.latched_fault_kinds & kind.bit()) != 0
    }

    /// True when at least one fault kind is currently latched since the last clear.
    #[inline]
    pub fn has_latched_fault(&self) -> bool {
        self.latched_fault_kinds != 0
    }

    /// Bitmask of every [`FaultKind`] latched since the last clear to `Idle`.
    #[inline]
    pub fn latched_fault_kinds(&self) -> u16 {
        self.latched_fault_kinds
    }

    /// Representative inline detail recorded for `kind` since the last clear, if any.
    #[inline]
    pub fn latched_fault_detail(&self, kind: FaultKind) -> Option<&LatchedFaultDetail> {
        self.latched_fault_details[kind as usize].as_ref()
    }

    /// Sets an arbitrary bit in the latched-kind bitmask for fail-closed unknown-kind tests.
    #[doc(hidden)]
    pub fn record_unknown_kind_bit_for_test(&mut self, bit: u16) {
        self.latched_fault_kinds |= bit;
    }

    /// Empties every preallocated fault-slot pool for deterministic exhaustion tests.
    #[doc(hidden)]
    pub fn drain_fault_arc_pool_for_test(&mut self) -> Vec<Arc<SafetyState>> {
        let mut drained = Vec::with_capacity(
            self.fault_arc_pool_len + self.formatted_fault_arc_pool_len + self.joint_limit_arc_pool_len,
        );
        while self.fault_arc_pool_len > 0 {
            self.fault_arc_pool_len -= 1;
            if let Some(arc) = self.fault_arc_pool[self.fault_arc_pool_len].take() {
                drained.push(arc);
            }
        }
        while self.formatted_fault_arc_pool_len > 0 {
            self.formatted_fault_arc_pool_len -= 1;
            if let Some(arc) = self.formatted_fault_arc_pool[self.formatted_fault_arc_pool_len].take() {
                drained.push(arc);
            }
        }
        while self.joint_limit_arc_pool_len > 0 {
            self.joint_limit_arc_pool_len -= 1;
            if let Some(arc) = self.joint_limit_arc_pool[self.joint_limit_arc_pool_len].take() {
                drained.push(arc);
            }
        }
        drained
    }

    /// True when `code` has been latched since the last clear to `Idle`.
    #[inline]
    pub fn has_latched_fault_code(&self, code: FaultCode) -> bool {
        (self.latched_fault_codes & code.bit()) != 0
    }

    /// Most severe recorded [`FaultKind`] according to [`FaultKind::severity`].
    pub fn highest_severity_kind(&self) -> Option<FaultKind> {
        let mut best: Option<(FaultKind, u8)> = None;
        for kind in FaultKind::ALL {
            if self.has_latched_fault_kind(kind) {
                let code = self.detail_code_for(kind);
                let rank = kind.severity(code);
                if best.is_none_or(|(_, r)| rank > r) {
                    best = Some((kind, rank));
                }
            }
        }
        best.map(|(k, _)| k)
    }

    #[inline]
    fn detail_code_for(&self, kind: FaultKind) -> FaultCode {
        self.latched_fault_details[kind as usize]
            .map_or_else(|| kind.default_code(), |d| d.code)
    }

    /// Single recoverability rule: true ONLY when EVERY recorded fault kind and detail in the
    /// fixed-size record is recoverable (worst-of across the record, fail-closed on unknown bits).
    pub fn recoverable(&self) -> bool {
        if (self.latched_fault_kinds & !FaultKind::ALL_MASK) != 0 {
            return false;
        }
        if self.has_non_recoverable {
            return false;
        }
        for bit in 0..16u8 {
            if (self.latched_fault_kinds & (1u16 << bit)) != 0 {
                let Some(kind) = FaultKind::from_bit_index(bit) else {
                    return false;
                };
                if kind.always_non_recoverable() || !self.detail_code_for(kind).recoverable() {
                    return false;
                }
            }
        }
        match &*self.state_arc {
            SafetyState::EStop => false,
            SafetyState::Fault(reason) => {
                let code = self.fault_code.unwrap_or_else(|| FaultCode::from_reason(reason));
                code.recoverable()
            }
            _ => true,
        }
    }

    fn emit_or_queue_fault_log(&mut self, reason: &BoundedFaultReason, code: FaultCode) {
        if self.deferred_logging {
            let from = BoundedSafetyState::from_state(&self.state_arc);
            self.queue_log_event(SafetyLogEvent::Fault { from, reason: *reason, code });
        } else {
            tracing::warn!("Safety: {:?} -> Fault({:?}) [{code:?}]", *self.state_arc, reason);
        }
    }

    /// Returns a clean, uniquely owned `SafetyState::Idle`, pre-sized empty `Other(String)`, or
    /// pre-sized `JointLimitExceeded` `Arc` recycled off-thread back into the matching bounded
    /// fault slot pool; returns `Err(arc)` without dropping if rejected.
    pub fn return_fault_arc(&mut self, arc: Arc<SafetyState>) -> Result<(), Arc<SafetyState>> {
        if Arc::strong_count(&arc) != 1 || Arc::weak_count(&arc) != 0 {
            return Err(arc);
        }
        match &*arc {
            SafetyState::Idle if self.fault_arc_pool_len < FAULT_ARC_POOL_CAPACITY => {
                self.fault_arc_pool[self.fault_arc_pool_len] = Some(arc);
                self.fault_arc_pool_len += 1;
                Ok(())
            }
            SafetyState::Fault(FaultReason::Other(s))
                if s.is_empty()
                    && s.capacity() >= SAFETY_LOG_TEXT_CAPACITY
                    && self.formatted_fault_arc_pool_len < FORMATTED_FAULT_ARC_POOL_CAPACITY =>
            {
                self.formatted_fault_arc_pool[self.formatted_fault_arc_pool_len] = Some(arc);
                self.formatted_fault_arc_pool_len += 1;
                Ok(())
            }
            SafetyState::Fault(FaultReason::JointLimitExceeded { joint_name, .. })
                if joint_name.is_empty()
                    && joint_name.capacity() >= SAFETY_JOINT_NAME_CAPACITY
                    && self.joint_limit_arc_pool_len < JOINT_LIMIT_ARC_POOL_CAPACITY =>
            {
                self.joint_limit_arc_pool[self.joint_limit_arc_pool_len] = Some(arc);
                self.joint_limit_arc_pool_len += 1;
                Ok(())
            }
            _ => Err(arc),
        }
    }

    /// True when the inline fault reclamation ring is at capacity.
    pub fn retired_state_arcs_full(&self) -> bool {
        self.retired_len >= RETIRED_STATE_CAPACITY
    }

    /// Number of retired heap-bearing fault states awaiting off-thread drop.
    pub fn retired_state_arcs_len(&self) -> usize {
        self.retired_len
    }

    /// Pops the oldest retired `Fault` state arc for off-thread drop (0 alloc, 0 drop).
    pub fn pop_retired_state_arc(&mut self) -> Option<Arc<SafetyState>> {
        if self.retired_len == 0 {
            return None;
        }
        let arc = self.retired_state_arcs[self.retired_head].take();
        self.retired_head = (self.retired_head + 1) % RETIRED_STATE_CAPACITY;
        self.retired_len -= 1;
        arc
    }

    /// Drops all queued retired fault states on a non-RT thread (legacy/pre-handoff cleanup),
    /// recycling uniquely owned slots (no outstanding strong or weak handles) back to the pools.
    pub fn drain_retired_state_arcs_off_thread(&mut self) {
        while let Some(mut arc) = self.pop_retired_state_arc() {
            if let Some(slot) = Arc::get_mut(&mut arc) {
                Self::reset_recycled_fault_arc(slot);
                if let Err(rejected) = self.return_fault_arc(arc) {
                    drop(rejected);
                }
                continue;
            }
            drop(arc);
        }
    }

    fn is_cached_fault_arc(&self, arc: &Arc<SafetyState>) -> bool {
        Arc::ptr_eq(arc, &self.can_tx_stalled_arc)
            || Arc::ptr_eq(arc, &self.j7_feedback_stale_arc)
            || Arc::ptr_eq(arc, &self.bus_loss_arc)
            || Arc::ptr_eq(arc, &self.watchdog_timeout_arc)
            || Arc::ptr_eq(arc, &self.detail_unavailable_arc)
            || self.static_fault_arcs.iter().any(|a| Arc::ptr_eq(arc, a))
    }

    /// Acquires a preallocated or cached `Arc<SafetyState>` for `detail` with zero heap allocation.
    fn acquire_state_arc(&mut self, kind: FaultKind, detail: FaultDetail) -> Option<Arc<SafetyState>> {
        match detail {
            FaultDetail::Estop => Some(Arc::clone(&self.estop_arc)),
            FaultDetail::BusLoss => Some(Arc::clone(&self.bus_loss_arc)),
            FaultDetail::WatchdogTimeout => Some(Arc::clone(&self.watchdog_timeout_arc)),
            FaultDetail::CanTxStalled(_) => Some(Arc::clone(&self.can_tx_stalled_arc)),
            FaultDetail::J7FeedbackStale(_) => Some(Arc::clone(&self.j7_feedback_stale_arc)),
            FaultDetail::Static(sf) => Some(Arc::clone(&self.static_fault_arcs[sf as usize])),
            FaultDetail::DetailUnavailable(_) => {
                if kind == FaultKind::Estop {
                    Some(Arc::clone(&self.estop_arc))
                } else {
                    Some(Arc::clone(&self.detail_unavailable_arc))
                }
            }
            FaultDetail::ThermalTrip { joint_idx, temperature, limit } => {
                while self.fault_arc_pool_len > 0 {
                    self.fault_arc_pool_len -= 1;
                    if let Some(mut arc) = self.fault_arc_pool[self.fault_arc_pool_len].take() {
                        if let Some(slot) = Arc::get_mut(&mut arc) {
                            *slot = SafetyState::Fault(FaultReason::ThermalTrip {
                                joint_idx,
                                temperature,
                                limit,
                            });
                            return Some(arc);
                        }
                    }
                }
                None
            }
            FaultDetail::JointLimitExceeded { joint_idx, joint_name, value, min, max } => {
                while self.joint_limit_arc_pool_len > 0 {
                    self.joint_limit_arc_pool_len -= 1;
                    if let Some(mut arc) = self.joint_limit_arc_pool[self.joint_limit_arc_pool_len].take() {
                        if let Some(SafetyState::Fault(FaultReason::JointLimitExceeded {
                            joint_idx: dst_idx,
                            joint_name: dst_name,
                            value: dst_val,
                            min: dst_min,
                            max: dst_max,
                        })) = Arc::get_mut(&mut arc)
                        {
                            if dst_name.capacity() >= joint_name.as_str().len() {
                                dst_name.clear();
                                dst_name.push_str(joint_name.as_str());
                                *dst_idx = joint_idx;
                                *dst_val = value;
                                *dst_min = min;
                                *dst_max = max;
                                return Some(arc);
                            }
                        }
                    }
                }
                None
            }
            FaultDetail::Formatted { message, .. } => {
                while self.formatted_fault_arc_pool_len > 0 {
                    self.formatted_fault_arc_pool_len -= 1;
                    if let Some(mut arc) = self.formatted_fault_arc_pool[self.formatted_fault_arc_pool_len].take() {
                        if let Some(SafetyState::Fault(FaultReason::Other(s))) = Arc::get_mut(&mut arc) {
                            if s.capacity() >= message.as_str().len() {
                                s.clear();
                                s.push_str(message.as_str());
                                return Some(arc);
                            }
                        }
                    }
                }
                None
            }
        }
    }

    /// Replaces `state_arc` with `next`. When `off_thread_reclaim` is active and the current state
    /// is a non-cached `Fault(_)`, queues the replaced `Arc` into `retired_state_arcs` without
    /// overwriting any occupied slot; returns `false` (fail-closed, keeping the current `Fault`
    /// latched) if full.
    fn try_replace_state_arc(&mut self, next: Arc<SafetyState>) -> bool {
        let needs_reclaim = matches!(*self.state_arc, SafetyState::Fault(_))
            && !self.is_cached_fault_arc(&self.state_arc);
        if self.off_thread_reclaim && needs_reclaim {
            if self.retired_len >= RETIRED_STATE_CAPACITY {
                return false;
            }
            let prev = std::mem::replace(&mut self.state_arc, next);
            let tail = (self.retired_head + self.retired_len) % RETIRED_STATE_CAPACITY;
            self.retired_state_arcs[tail] = Some(prev);
            self.retired_len += 1;
            return true;
        }
        let mut prev = std::mem::replace(&mut self.state_arc, next);
        if !self.off_thread_reclaim && needs_reclaim {
            if let Some(slot) = Arc::get_mut(&mut prev) {
                Self::reset_recycled_fault_arc(slot);
                let _ = self.return_fault_arc(prev);
            }
        }
        true
    }

    pub fn epoch_id(&self) -> u64 {
        self.epoch_id
    }

    /// Lock-free atomic mirror of [`Self::epoch_id`], kept in sync on every epoch bump.
    pub fn epoch_mirror(&self) -> Arc<AtomicU64> {
        Arc::clone(&self.epoch_mirror)
    }

    #[inline]
    fn bump_epoch(&mut self) -> u64 {
        self.epoch_id = self.epoch_id.wrapping_add(1);
        self.epoch_mirror.store(self.epoch_id, Ordering::SeqCst);
        self.epoch_id
    }

    /// Single authoritative fault/E-stop latch used by every detector.
    ///
    /// Always records `kind`, its bounded inline detail, and worst-of recoverability in the
    /// fixed-size record — even when another fault is already active and even when every
    /// preallocated `Arc<SafetyState>` pool slot is exhausted — and never allocates on the heap.
    /// Every latch (new kind or repeat of the same kind, including E-stop) advances `epoch_id`,
    /// while repeat detection of an already-recorded kind with the same code avoids Arc/log churn.
    pub fn latch(&mut self, kind: FaultKind, detail: FaultDetail) -> u64 {
        let code = detail.code(kind);
        let bounded_reason = detail.bounded_reason(kind);
        let already_latched = self.has_latched_fault_kind(kind);
        let prev_detail = self.latched_fault_details[kind as usize];
        let prev_highest = self.highest_severity_kind();
        let prev_highest_rank = prev_highest.map_or(0, |k| k.severity(self.detail_code_for(k)));

        // 1. Always record kind, code, worst-of recoverability, and inline detail (0 heap ops).
        self.latched_fault_kinds |= kind.bit();
        self.latched_fault_codes |= code.bit();
        if kind.always_non_recoverable() || !code.recoverable() {
            self.has_non_recoverable = true;
        }
        let keep_prev_other_detail = kind == FaultKind::Other
            && prev_detail.is_some_and(|d| !d.code.recoverable() && code.recoverable());
        if !keep_prev_other_detail {
            self.latched_fault_details[kind as usize] = Some(LatchedFaultDetail {
                code,
                reason: bounded_reason,
            });
        }

        // 2. EVERY latch (a new kind or a repeat of the same kind, including E-stop) advances epoch.
        self.latch_count = self.latch_count.wrapping_add(1);
        self.bump_epoch();

        // Repeat detection of an already-recorded kind with the same code needs no Arc/log churn.
        if already_latched && prev_detail.is_some_and(|d| d.code == code) {
            return self.epoch_id;
        }

        // 3. Determine whether this latch upgrades the displayed state by severity precedence.
        let new_rank = kind.severity(code);
        let upgrades_shown = prev_highest.is_none()
            || new_rank > prev_highest_rank
            || (prev_highest == Some(kind) && kind == FaultKind::Other && !keep_prev_other_detail);

        if kind == FaultKind::Estop {
            if upgrades_shown && *self.state_arc != SafetyState::EStop {
                if self.deferred_logging {
                    let from = BoundedSafetyState::from_state(&self.state_arc);
                    self.queue_log_event(SafetyLogEvent::EStop { from });
                } else {
                    tracing::warn!("Safety: {:?} -> EStop", *self.state_arc);
                }
            }
        } else {
            self.emit_or_queue_fault_log(&bounded_reason, code);
        }

        if upgrades_shown && self.can_accept_fault_transition() {
            if let Some(next_arc) = self.acquire_state_arc(kind, detail) {
                self.fault_code = Some(code);
                let replaced = self.try_replace_state_arc(next_arc);
                debug_assert!(replaced);
            } else if !self.state_arc.is_stopped() {
                self.fault_code = Some(code);
                let replaced = self.try_replace_state_arc(Arc::clone(&self.detail_unavailable_arc));
                debug_assert!(replaced);
            }
        }

        self.epoch_id
    }

    /// Single authoritative clear check evaluated FIRST on every clear path, before any motor
    /// reset or other side effect.
    ///
    /// Evaluates universal supervisor blockers and the explicit per-kind condition table over
    /// EVERY recorded bit in `latched_fault_kinds` BEFORE permitting a motor fault reset
    /// ([`ClearRefusal::HardwareResetRequired`]). Any unknown kind bit refuses to clear
    /// (fail closed). Performs zero heap allocations.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_check(&self, ctx: &ClearContext) -> Result<(), ClearRefusal> {
        if let Some(received) = ctx.expected_epoch {
            if received != self.epoch_id {
                return Err(ClearRefusal::EpochMismatch {
                    expected: self.epoch_id,
                    received,
                });
            }
        }
        if ctx.pending_estop {
            return Err(ClearRefusal::EStopActive);
        }
        if ctx.shutdown_committed {
            return Err(ClearRefusal::ShutdownCommitted);
        }
        if let Some(err) = ctx.driver_config_error {
            return Err(ClearRefusal::DriverConfigError(err));
        }
        if let Some(err) = ctx.config_reconcile_fault {
            return Err(ClearRefusal::ConfigPersistenceFailed(err));
        }
        if ctx.gravity_comp_unconfirmed {
            return Err(ClearRefusal::GravityCompUnconfirmed);
        }
        if ctx.disable_pending {
            return Err(ClearRefusal::DisablePending);
        }
        if ctx.mit_lost_unsupported {
            return Err(ClearRefusal::MitLostUnsupported);
        }
        if ctx.mit_modes_uncertain {
            return Err(ClearRefusal::MitModesUncertain);
        }
        if ctx.resting_feedback_missing {
            return Err(ClearRefusal::FreshFeedbackRequiredForRestingSupport);
        }
        if let Some(violation) = ctx.resting_pose_violation {
            return Err(ClearRefusal::RestingPoseSupport(violation));
        }
        if !self.can_accept_fault_transition() {
            return Err(ClearRefusal::ReclaimQueueFull);
        }
        if (self.latched_fault_kinds & !FaultKind::ALL_MASK) != 0 {
            return Err(ClearRefusal::UnknownFaultKind);
        }

        // Per-kind condition table evaluated over EVERY recorded kind BEFORE any motor reset
        // exception (`hardware_reset_required`) or hardware-error check: a motor reset is ONLY
        // permitted after every OTHER recorded kind's condition has passed.
        const CHECK_ORDER: [FaultKind; FaultKind::COUNT] = [
            FaultKind::BusLoss,
            FaultKind::CanTxStalled,
            FaultKind::ThermalTrip,
            FaultKind::WatchdogTimeout,
            FaultKind::JointLimit,
            FaultKind::J7FeedbackStale,
            FaultKind::CollisionLatched,
            FaultKind::Estop,
            FaultKind::Other,
        ];
        let mut checked_any = false;
        for kind in CHECK_ORDER {
            if !self.has_latched_fault_kind(kind) {
                continue;
            }
            checked_any = true;
            match kind {
                FaultKind::BusLoss => {
                    if !ctx.feedback_fresh {
                        return Err(ClearRefusal::FreshFeedbackRequired);
                    }
                }
                FaultKind::CanTxStalled => {
                    if !ctx.feedback_fresh {
                        return Err(ClearRefusal::FreshFeedbackRequired);
                    }
                    if !ctx.tx_queue_drained {
                        return Err(ClearRefusal::TxQueueNotDrained);
                    }
                }
                FaultKind::J7FeedbackStale => {
                    if !ctx.j7_feedback_fresh {
                        return Err(ClearRefusal::J7FeedbackStale);
                    }
                    if ctx.has_driver && !ctx.feedback_fresh {
                        return Err(ClearRefusal::FreshFeedbackRequired);
                    }
                }
                FaultKind::ThermalTrip => {
                    if ctx.has_driver && !ctx.feedback_fresh {
                        return Err(ClearRefusal::FreshFeedbackRequired);
                    }
                    if !ctx.thermal_sensor_available || !ctx.thermal_below_threshold {
                        return Err(if ctx.has_driver {
                            ClearRefusal::TemperatureConfirmationRequired
                        } else {
                            ClearRefusal::TemperatureAboveThreshold
                        });
                    }
                    if ctx.thermal_cooldown_elapsed_ms < ctx.thermal_cooldown_required_ms {
                        return Err(ClearRefusal::ThermalCooldownNotElapsed {
                            elapsed_ms: ctx.thermal_cooldown_elapsed_ms,
                            required_ms: ctx.thermal_cooldown_required_ms,
                        });
                    }
                }
                FaultKind::WatchdogTimeout => {
                    if ctx.has_driver && !ctx.feedback_fresh {
                        return Err(ClearRefusal::FreshFeedbackRequired);
                    }
                    if ctx.has_active_lease && !ctx.lease_heartbeat_fresh {
                        return Err(ClearRefusal::WatchdogHeartbeatRequired);
                    }
                }
                FaultKind::JointLimit => {
                    if ctx.has_driver && !ctx.feedback_fresh {
                        return Err(ClearRefusal::FreshFeedbackRequired);
                    }
                    if let Some(violation) = ctx.joint_limit_violation {
                        return Err(ClearRefusal::JointLimitsNotSatisfied(violation));
                    }
                }
                FaultKind::CollisionLatched | FaultKind::Estop | FaultKind::Other => {
                    if ctx.has_driver && !ctx.feedback_fresh {
                        return Err(ClearRefusal::FreshFeedbackRequired);
                    }
                }
            }
        }
        // Exhaustive bit-level fail-closed guard over all 16 bits.
        for bit in 0..16u8 {
            if (self.latched_fault_kinds & (1u16 << bit)) != 0
                && FaultKind::from_bit_index(bit).is_none()
            {
                return Err(ClearRefusal::UnknownFaultKind);
            }
        }
        if !checked_any && self.state_arc.is_stopped() && ctx.has_driver && !ctx.feedback_fresh {
            return Err(ClearRefusal::FreshFeedbackRequired);
        }
        // Hardware-fault reset, CAN TX probe, and status checks run ONLY after every other recorded kind passed.
        if checked_any || self.state_arc.is_stopped() {
            if let Some(err) = ctx.hardware_error {
                return Err(ClearRefusal::DriverError(err));
            }
            if ctx.has_driver && self.has_latched_fault_kind(FaultKind::CanTxStalled) && !ctx.tx_probe_confirmed {
                if ctx.tx_probe_sent {
                    return Err(ClearRefusal::TxProbeFeedbackRequired);
                }
                return Err(ClearRefusal::TxProbeRequired);
            }
            if ctx.hardware_reset_sent {
                return Err(ClearRefusal::HardwareResetSent);
            }
            if ctx.hardware_reset_required {
                return Err(ClearRefusal::HardwareResetRequired);
            }
            if ctx.has_driver && !ctx.enable_state_known {
                return Err(ClearRefusal::EnableStateUnknown);
            }
        }
        Ok(())
    }

    /// Evaluates [`Self::clear_check`] FIRST and, only after it passes, resets the entire fault
    /// record and transitions back to `Idle` all at once (0 heap allocations).
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_with_check(&mut self, ctx: &ClearContext) -> Result<SafetyState, ClearRefusal> {
        self.clear_check(ctx)?;
        if self.state_arc.is_stopped() || self.has_latched_fault() {
            if !self.try_replace_state_arc(Arc::clone(&self.idle_arc)) {
                return Err(ClearRefusal::ReclaimQueueFull);
            }
            if self.deferred_logging {
                self.queue_log_event(SafetyLogEvent::Cleared);
            } else {
                tracing::info!("Safety: cleared -> Idle");
            }
            self.fault_code = None;
            self.latched_fault_kinds = 0;
            self.latched_fault_codes = 0;
            self.has_non_recoverable = false;
            self.latched_fault_details.fill(None);
            self.bump_epoch();
            Ok(SafetyState::Idle)
        } else {
            Ok((*self.state_arc).clone())
        }
    }

    /// Advance epoch and latch E-Stop immediately (compatibility wrapper over [`Self::latch`]).
    pub fn trigger_e_stop(&mut self) -> u64 {
        self.latch(FaultKind::Estop, FaultDetail::Estop)
    }

    /// Machine-readable cause of the most severe recorded Fault / EStop; `None` when healthy.
    pub fn fault_code(&self) -> Option<FaultCode> {
        if let Some(kind) = self.highest_severity_kind() {
            return Some(self.detail_code_for(kind));
        }
        match &*self.state_arc {
            SafetyState::EStop => Some(FaultCode::Estop),
            SafetyState::Fault(reason) => Some(self.fault_code.unwrap_or_else(|| FaultCode::from_reason(reason))),
            _ => None,
        }
    }

    /// Inline detail of the latched [`FaultKind::CanTxStalled`] fault, if any.
    pub fn tx_stall_detail(&self) -> Option<TxStallDetail> {
        match self.latched_fault_details[FaultKind::CanTxStalled as usize] {
            Some(LatchedFaultDetail {
                reason: BoundedFaultReason::CanTxStalled(detail),
                ..
            }) => detail,
            _ => None,
        }
    }

    /// Structured `observation.fault` for the most severe recorded kind, with `recoverable`
    /// computed by [`Self::recoverable`] (0 heap allocations).
    pub fn fault(&self) -> Option<FaultInfo> {
        let mut message = BoundedText::empty();
        let code = if let Some(kind) = self.highest_severity_kind() {
            if let Some(detail) = self.latched_fault_details[kind as usize] {
                detail.reason.write_message_into(&mut message);
                detail.code
            } else {
                message.push_str(DETAIL_UNAVAILABLE);
                kind.default_code()
            }
        } else {
            match &*self.state_arc {
                SafetyState::EStop => {
                    message.push_str("E-stop latched");
                    FaultCode::Estop
                }
                SafetyState::Fault(reason) => {
                    reason.write_message_into(&mut message);
                    self.fault_code.unwrap_or_else(|| FaultCode::from_reason(reason))
                }
                _ => return None,
            }
        };
        Some(FaultInfo {
            code,
            message,
            recoverable: self.recoverable(),
        })
    }

    /// Compatibility wrapper over [`Self::latch`] for preallocated static faults.
    pub fn trigger_static_fault(&mut self, fault: StaticFault) -> u64 {
        self.latch(FaultKind::Other, FaultDetail::Static(fault))
    }

    /// Compatibility wrapper over [`Self::latch`] for dynamically formatted faults.
    pub fn trigger_fault_formatted(
        &mut self,
        code: FaultCode,
        kind: FaultKind,
        fmt_msg: impl FnOnce(&mut BoundedText<SAFETY_LOG_TEXT_CAPACITY>),
    ) -> u64 {
        self.latch(kind, FaultDetail::formatted(code, fmt_msg))
    }

    /// Compatibility wrapper over [`Self::latch`]; the code follows from the reason.
    pub fn trigger_fault(&mut self, reason: FaultReason) -> u64 {
        let code = FaultCode::from_reason(&reason);
        self.trigger_fault_coded(code, reason)
    }

    /// True when a fault transition can be accepted without overflowing the bounded off-thread
    /// fault reclamation queue.
    pub fn can_accept_fault_transition(&self) -> bool {
        !(self.off_thread_reclaim
            && matches!(*self.state_arc, SafetyState::Fault(_))
            && !self.is_cached_fault_arc(&self.state_arc)
            && self.retired_len >= RETIRED_STATE_CAPACITY)
    }

    /// Compatibility wrapper over [`Self::latch`]: skips `make_reason` when reclamation is full or
    /// the pool is empty while still recording the fault kind via [`Self::latch`].
    pub fn trigger_fault_coded_lazy(
        &mut self,
        code: FaultCode,
        make_reason: impl FnOnce() -> FaultReason,
    ) -> u64 {
        let kind = FaultKind::from_code(code);
        if !self.can_accept_fault_transition() || self.fault_arc_pool_len == 0 {
            let before = self.epoch_id;
            let ep = self.latch(kind, FaultDetail::DetailUnavailable(code));
            if !self.can_accept_fault_transition() && ep == before {
                self.bump_epoch();
            }
            return self.epoch_id;
        }
        self.trigger_fault_coded(code, make_reason())
    }

    /// Fallible compatibility wrapper over [`Self::latch`] returning `Err((epoch, reason))` when
    /// off-thread reclamation is full or the matching preallocated pool is empty so callers can
    /// drop heap-bearing `reason` off the RT thread.
    pub fn try_trigger_fault_coded(
        &mut self,
        code: FaultCode,
        reason: FaultReason,
    ) -> Result<u64, (u64, FaultReason)> {
        let kind = match FaultKind::from_code(code) {
            FaultKind::Other => FaultKind::from_reason(&reason),
            specific => specific,
        };
        let detail = FaultDetail::from_reason(code, &reason);
        let needs_pool = match detail {
            FaultDetail::ThermalTrip { .. } => self.fault_arc_pool_len == 0,
            FaultDetail::JointLimitExceeded { .. } => self.joint_limit_arc_pool_len == 0,
            FaultDetail::Formatted { .. } => self.formatted_fault_arc_pool_len == 0,
            _ => false,
        };
        let reclaim_full = !self.can_accept_fault_transition();
        let ep = self.latch(kind, detail);
        if reclaim_full || needs_pool {
            return Err((ep, reason));
        }
        Ok(ep)
    }

    /// Compatibility wrapper over [`Self::latch`] with an explicit [`FaultCode`].
    pub fn trigger_fault_coded(&mut self, code: FaultCode, reason: FaultReason) -> u64 {
        let kind = match FaultKind::from_code(code) {
            FaultKind::Other => FaultKind::from_reason(&reason),
            specific => specific,
        };
        let detail = FaultDetail::from_reason(code, &reason);
        self.latch(kind, detail)
    }

    /// Attempt to clear E-Stop or Fault back to Idle without heap allocation (evaluating
    /// [`Self::clear_check`] with all external conditions met).
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_fault_bounded(&mut self) -> Result<SafetyState, ClearRefusal> {
        let ctx = ClearContext::all_conditions_met();
        self.clear_with_check(&ctx)
    }

    /// Attempt to clear E-Stop or Fault back to Idle.
    pub fn clear_fault(&mut self) -> Result<SafetyState, SafetyError> {
        self.clear_fault_bounded()
            .map_err(|r| SafetyError::ClearFaultRejected(r.render_message()))
    }

    /// Set torque enabled/disabled
    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        if self.state_arc.is_stopped() {
            if !enable {
                return Err(SafetyError::MotionForbiddenInFaultState(
                    "Cannot disable torque in EStop or Fault. Call clear_fault first.".into(),
                ));
            }
            if let SafetyState::Fault(ref reason) = *self.state_arc {
                return Err(SafetyError::FaultActive(reason.clone()));
            }
            return Err(SafetyError::EStopActive);
        }

        if enable {
            let _ = self.try_replace_state_arc(Arc::clone(&self.armed_arc));
            Ok(SafetyState::Armed)
        } else {
            let _ = self.try_replace_state_arc(Arc::clone(&self.idle_arc));
            Ok(SafetyState::Idle)
        }
    }

    /// Zero-allocation RT equivalent of [`Self::set_torque`], returning static protocol error codes.
    pub fn set_torque_rt(&mut self, enable: bool) -> Result<(), &'static str> {
        if self.state_arc.is_stopped() {
            if !enable {
                return Err("MOTION_FORBIDDEN_IN_FAULT_STATE");
            }
            if matches!(*self.state_arc, SafetyState::Fault(_)) {
                return Err("FAULT_ACTIVE");
            }
            return Err("ESTOP_ACTIVE");
        }
        if enable {
            let _ = self.try_replace_state_arc(Arc::clone(&self.armed_arc));
        } else {
            let _ = self.try_replace_state_arc(Arc::clone(&self.idle_arc));
        }
        Ok(())
    }

    /// Notify that motion has started
    pub fn mark_motion_started(&mut self) -> Result<(), SafetyError> {
        if *self.state_arc != SafetyState::Armed && *self.state_arc != SafetyState::Moving {
            return Err(SafetyError::NotArmed((*self.state_arc).clone()));
        }
        let _ = self.try_replace_state_arc(Arc::clone(&self.moving_arc));
        Ok(())
    }

    /// Notify that motion has completed
    pub fn mark_motion_finished(&mut self) {
        if *self.state_arc == SafetyState::Moving {
            let _ = self.try_replace_state_arc(Arc::clone(&self.armed_arc));
        }
    }

    /// Validate an incoming motion command against single controller authority (epoch_id),
    /// current safety state, and soft limits.
    pub fn validate_motion_command(
        &self,
        cmd_epoch_id: Option<u64>,
        target_positions: &[f32],
        limits: &[JointLimit],
    ) -> Result<(), SafetyError> {
        // 1. Safety state check
        match &*self.state_arc {
            SafetyState::EStop => return Err(SafetyError::EStopActive),
            SafetyState::Fault(reason) => return Err(SafetyError::FaultActive(reason.clone())),
            SafetyState::Booting | SafetyState::Idle => {
                return Err(SafetyError::NotArmed((*self.state_arc).clone()));
            }
            SafetyState::Armed | SafetyState::Moving => {}
        }

        // 2. Epoch generation ID validation (single authority & epoch latch)
        if let Some(cmd_epoch) = cmd_epoch_id {
            if cmd_epoch != self.epoch_id {
                return Err(SafetyError::EpochMismatch {
                    expected: self.epoch_id,
                    received: cmd_epoch,
                });
            }
        }

        // 3. Dimensionality check
        if target_positions.len() != limits.len() {
            return Err(SafetyError::DimensionMismatch {
                expected: limits.len(),
                received: target_positions.len(),
            });
        }

        // 4. Soft limit check and finite check
        for (&target, limit) in target_positions.iter().zip(limits.iter()) {
            if !target.is_finite() {
                return Err(SafetyError::InvalidValue);
            }
            if target < limit.min_position || target > limit.max_position {
                return Err(SafetyError::SoftLimitExceeded {
                    joint: limit.name.clone(),
                    value: target,
                    min: limit.min_position,
                    max: limit.max_position,
                });
            }
        }

        Ok(())
    }

    /// Zero-allocation variant of [`Self::validate_motion_command`] for RT servo target intake.
    /// Returns `Err((protocol_code, offending_joint_index))`.
    pub fn validate_motion_command_rt(
        &self,
        cmd_epoch_id: Option<u64>,
        target_positions: &[f32],
        limits: &[JointLimit],
    ) -> Result<(), (&'static str, Option<usize>)> {
        match &*self.state_arc {
            SafetyState::EStop => return Err(("ESTOP_ACTIVE", None)),
            SafetyState::Fault(_) => return Err(("FAULT_ACTIVE", None)),
            SafetyState::Booting | SafetyState::Idle => return Err(("NOT_ARMED", None)),
            SafetyState::Armed | SafetyState::Moving => {}
        }
        if let Some(cmd_epoch) = cmd_epoch_id {
            if cmd_epoch != self.epoch_id {
                return Err(("EPOCH_MISMATCH", None));
            }
        }
        if target_positions.len() != limits.len() {
            return Err(("SAFETY_ERROR", None));
        }
        for (idx, (&target, limit)) in target_positions.iter().zip(limits.iter()).enumerate() {
            if !target.is_finite() {
                return Err(("SAFETY_ERROR", None));
            }
            if target < limit.min_position || target > limit.max_position {
                return Err(("SOFT_LIMIT_EXCEEDED", Some(idx)));
            }
        }
        Ok(())
    }

    /// Check motion limits across positions, velocities, and torques with NaN/Inf protection
    pub fn check_motion_limits(
        &self,
        positions: &[f32],
        velocities: &[f32],
        torques: &[f32],
        limits: &[JointLimit],
    ) -> Result<(), SafetyError> {
        for (&p, limit) in positions.iter().zip(limits.iter()) {
            if !p.is_finite() {
                return Err(SafetyError::InvalidValue);
            }
            if p < limit.min_position || p > limit.max_position {
                return Err(SafetyError::SoftLimitExceeded {
                    joint: limit.name.clone(),
                    value: p,
                    min: limit.min_position,
                    max: limit.max_position,
                });
            }
        }
        for (&v, limit) in velocities.iter().zip(limits.iter()) {
            if !v.is_finite() {
                return Err(SafetyError::InvalidValue);
            }
            if v.abs() > limit.max_velocity {
                return Err(SafetyError::SoftLimitExceeded {
                    joint: limit.name.clone(),
                    value: v,
                    min: -limit.max_velocity,
                    max: limit.max_velocity,
                });
            }
        }
        for &tau in torques {
            if !tau.is_finite() {
                return Err(SafetyError::InvalidValue);
            }
        }
        Ok(())
    }
}
