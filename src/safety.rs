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
    /// Hint for client recovery policies; armd never clears a fault by itself.
    pub fn recoverable(self) -> bool {
        matches!(self, FaultCode::WatchdogTimeout | FaultCode::ControlDeadline | FaultCode::HandoverUnsettled)
    }

    /// Code implied by a reason when the trigger site gave none.
    pub fn from_reason(reason: &FaultReason) -> Self {
        match reason {
            FaultReason::JointLimitExceeded { .. } => FaultCode::JointLimit,
            FaultReason::ThermalTrip { .. } => FaultCode::ThermalTrip,
            FaultReason::BusLoss => FaultCode::BusLoss,
            FaultReason::WatchdogTimeout => FaultCode::WatchdogTimeout,
            FaultReason::Other(_) => FaultCode::Other,
        }
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
            FaultReason::Other(m) => {
                let _ = w.write_str(m);
            }
        }
    }

    pub fn message(&self) -> String {
        match self {
            FaultReason::JointLimitExceeded { joint_name, value, min, max, .. } =>
                format!("Joint '{joint_name}' at {value:.4} outside [{min:.4}, {max:.4}]"),
            FaultReason::ThermalTrip { joint_idx, temperature, limit } =>
                format!("Joint {} temperature {temperature:.1} C above limit {limit:.1} C", joint_idx + 1),
            FaultReason::BusLoss => "No fresh feedback from every joint (bus loss)".into(),
            FaultReason::WatchdogTimeout => "Control lease expired while torqued".into(),
            FaultReason::Other(m) => m.clone(),
        }
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
pub const FORMATTED_FAULT_ARC_POOL_CAPACITY: usize = 16;
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

/// Discriminant of [`FaultReason`] tracked in [`SafetyManager`]'s fixed-size latched-kind bitmask.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FaultKind {
    JointLimitExceeded = 0,
    ThermalTrip = 1,
    BusLoss = 2,
    WatchdogTimeout = 3,
    Other = 4,
}

impl FaultKind {
    pub const COUNT: usize = 5;
    pub const ALL: [Self; Self::COUNT] = [
        Self::JointLimitExceeded,
        Self::ThermalTrip,
        Self::BusLoss,
        Self::WatchdogTimeout,
        Self::Other,
    ];

    #[inline]
    pub const fn bit(self) -> u8 {
        1u8 << (self as u8)
    }

    #[inline]
    pub const fn from_reason(reason: &FaultReason) -> Self {
        match reason {
            FaultReason::JointLimitExceeded { .. } => Self::JointLimitExceeded,
            FaultReason::ThermalTrip { .. } => Self::ThermalTrip,
            FaultReason::BusLoss => Self::BusLoss,
            FaultReason::WatchdogTimeout => Self::WatchdogTimeout,
            FaultReason::Other(_) => Self::Other,
        }
    }

    #[inline]
    pub const fn from_code(code: FaultCode) -> Self {
        match code {
            FaultCode::JointLimit => Self::JointLimitExceeded,
            FaultCode::ThermalTrip => Self::ThermalTrip,
            FaultCode::BusLoss => Self::BusLoss,
            FaultCode::WatchdogTimeout => Self::WatchdogTimeout,
            _ => Self::Other,
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
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BoundedText<const N: usize> {
    len: u16,
    buf: [u8; N],
}

impl<const N: usize> BoundedText<N> {
    pub const fn empty() -> Self { Self { len: 0, buf: [0u8; N] } }

    pub fn new(s: &str) -> Self {
        let mut out = Self::empty();
        out.push_str(s);
        out
    }

    pub fn push_str(&mut self, s: &str) {
        let used = usize::from(self.len);
        let avail = N.saturating_sub(used);
        if avail == 0 { return; }
        let mut take = s.len().min(avail);
        while take > 0 && !s.is_char_boundary(take) { take -= 1; }
        self.buf[used..used + take].copy_from_slice(&s.as_bytes()[..take]);
        self.len = (used + take) as u16;
    }

    pub fn as_str(&self) -> &str {
        std::str::from_utf8(&self.buf[..usize::from(self.len)]).unwrap_or_default()
    }

    pub fn is_empty(&self) -> bool { self.len == 0 }
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
        serializer.serialize_str(self.as_str())
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
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { f.write_str(self.as_str()) }
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
            FaultReason::Other(m) => Self::Other(BoundedText::new(m)),
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
}

impl SafetyLogEvent {
    /// Emits the exact tracing event on an off-RT thread.
    pub fn emit(&self) {
        match self {
            Self::EStop { from } => tracing::warn!(target: "armd::safety", "Safety: {:?} -> EStop", from),
            Self::Fault { from, reason, code } =>
                tracing::warn!(target: "armd::safety", "Safety: {:?} -> Fault({:?}) [{code:?}]", from, reason),
            Self::Cleared => tracing::info!(target: "armd::safety", "Safety: cleared -> Idle"),
            Self::GravityCompEstopExit => tracing::warn!(target: "armd::supervisor::motion", "Gravity compensation exit: ESTOP E-stop"),
            Self::GravityCompHoldWriteFailed(err) => tracing::error!(target: "armd::supervisor::motion", "Gravity comp hold write failed: {err}"),
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
            Self::Failures { items, len, workspace } => {
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
        }
    }
}

/// Typed, bounded RT `clear_fault` refusal carrying zero heap allocations on the control thread;
/// rendered off-thread to the exact existing `SafetyError::ClearFaultRejected(String)` text.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum ClearFaultRefusal {
    ShutdownCommitted,
    DriverConfigError(BoundedText<256>),
    GravityCompUnconfirmed,
    DisablePending,
    MitLostUnsupported,
    FreshFeedbackRequiredForRestingSupport,
    RestingPoseSupport(RestingSupportViolation),
    BudgetExpired,
    HardwareResetSent,
    FreshFeedbackRequired,
    TemperatureConfirmationRequired,
    EnableStateUnknown,
    TemperatureAboveThreshold,
    ReclaimQueueFull,
    DriverError(ClearFaultDriverError),
}

impl ClearFaultRefusal {
    /// Renders the exact API error text off the RT control thread.
    pub fn render_message(&self) -> String { self.to_string() }
}

impl std::fmt::Display for ClearFaultRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ShutdownCommitted => f.write_str("Daemon shutdown is committed"),
            Self::DriverConfigError(err) => write!(f, "Driver limit configuration error: {err}"),
            Self::GravityCompUnconfirmed => f.write_str("Gravity compensation: the switch back to POS_VEL is not confirmed yet"),
            Self::DisablePending => f.write_str("Torque disable is still unconfirmed"),
            Self::MitLostUnsupported => f.write_str("UNSUPPORTED: MIT lost recovery requires legacy path"),
            Self::FreshFeedbackRequiredForRestingSupport => f.write_str("Fresh feedback is required before checking resting pose support"),
            Self::RestingPoseSupport(violation) =>
                write!(f, "Cannot disable faulted supporting arm joint away from resting pose ({violation}); support the arm at rest first"),
            Self::BudgetExpired => f.write_str("Clear fault budget expired"),
            Self::HardwareResetSent => f.write_str("Hardware reset sent; wait for fresh fault-free feedback before retrying"),
            Self::FreshFeedbackRequired => f.write_str("Fresh feedback is required before resetting a fault"),
            Self::TemperatureConfirmationRequired => f.write_str("Temperature must be confirmed below the reset threshold"),
            Self::EnableStateUnknown => f.write_str("Hardware enable state is unknown; reset remains latched"),
            Self::TemperatureAboveThreshold => f.write_str("Temperature is above the reset threshold"),
            Self::ReclaimQueueFull => f.write_str("Fault state reclamation queue is full; retry after off-thread drain"),
            Self::DriverError(err) => write!(f, "{err}"),
        }
    }
}

/// Bounded RT error returned by `clear_fault_executor_bounded`; converted off-thread into [`SafetyError`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline stack storage avoids RT heap allocation
pub enum ClearFaultError {
    EStopActive,
    FeedbackUnavailable,
    Rejected(ClearFaultRefusal),
}

impl std::fmt::Display for ClearFaultError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EStopActive => f.write_str("E-Stop is latched, motion rejected"),
            Self::FeedbackUnavailable => f.write_str("Actuator feedback unavailable: physical angles not received from hardware"),
            Self::Rejected(r) => write!(f, "Cannot clear fault while hardware fault condition persists: {r}"),
        }
    }
}

impl std::error::Error for ClearFaultError {}
impl From<ClearFaultRefusal> for ClearFaultError {
    fn from(r: ClearFaultRefusal) -> Self { Self::Rejected(r) }
}
impl From<ClearFaultError> for SafetyError {
    fn from(e: ClearFaultError) -> Self {
        match e {
            ClearFaultError::EStopActive => SafetyError::EStopActive,
            ClearFaultError::FeedbackUnavailable => SafetyError::FeedbackUnavailable,
            ClearFaultError::Rejected(r) => SafetyError::ClearFaultRejected(r.render_message()),
        }
    }
}

/// Emits a warning off-thread when bounded deferred safety log events were dropped.
pub fn emit_safety_log_dropped(dropped: u64) {
    if dropped > 0 {
        tracing::warn!(
            target: "armd::safety",
            "Safety log: {dropped} events dropped (safety log queue full)"
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
    bus_loss_arc: Arc<SafetyState>,
    watchdog_timeout_arc: Arc<SafetyState>,
    detail_unavailable_arc: Arc<SafetyState>,
    static_fault_arcs: [Arc<SafetyState>; StaticFault::COUNT],
    fault_arc_pool: [Option<Arc<SafetyState>>; FAULT_ARC_POOL_CAPACITY],
    fault_arc_pool_len: usize,
    formatted_fault_arc_pool: [Option<Arc<SafetyState>>; FORMATTED_FAULT_ARC_POOL_CAPACITY],
    formatted_fault_arc_pool_len: usize,
    retired_state_arcs: [Option<Arc<SafetyState>>; RETIRED_STATE_CAPACITY],
    retired_head: usize,
    retired_len: usize,
    off_thread_reclaim: bool,
    deferred_logging: bool,
    log_events: [Option<SafetyLogEvent>; SAFETY_LOG_CAPACITY],
    log_head: usize,
    log_len: usize,
    log_dropped: u64,
    epoch_id: u64,
    /// Code recorded by the trigger site of the current Fault.
    fault_code: Option<FaultCode>,
    /// Bitmask of every [`FaultKind`] latched since the last clear to `Idle`.
    latched_fault_kinds: u8,
    /// Fixed-size inline record of the last detail per latched [`FaultKind`].
    latched_fault_details: [Option<LatchedFaultDetail>; FaultKind::COUNT],
}

impl Clone for SafetyManager {
    fn clone(&self) -> Self {
        Self {
            state_arc: Arc::clone(&self.state_arc),
            idle_arc: Arc::clone(&self.idle_arc),
            armed_arc: Arc::clone(&self.armed_arc),
            moving_arc: Arc::clone(&self.moving_arc),
            estop_arc: Arc::clone(&self.estop_arc),
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
            retired_state_arcs: self.retired_state_arcs.clone(),
            retired_head: self.retired_head,
            retired_len: self.retired_len,
            off_thread_reclaim: self.off_thread_reclaim,
            deferred_logging: self.deferred_logging,
            log_events: self.log_events,
            log_head: self.log_head,
            log_len: self.log_len,
            log_dropped: self.log_dropped,
            epoch_id: self.epoch_id,
            fault_code: self.fault_code,
            latched_fault_kinds: self.latched_fault_kinds,
            latched_fault_details: self.latched_fault_details,
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
            retired_state_arcs: std::array::from_fn(|_| None),
            retired_head: 0,
            retired_len: 0,
            off_thread_reclaim: false,
            deferred_logging: false,
            log_events: [None; SAFETY_LOG_CAPACITY],
            log_head: 0,
            log_len: 0,
            log_dropped: 0,
            epoch_id: 1,
            fault_code: None,
            latched_fault_kinds: 0,
            latched_fault_details: [None; FaultKind::COUNT],
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

    /// Returns and resets the number of deferred safety log events dropped because the ring was full.
    pub fn take_log_dropped(&mut self) -> u64 {
        std::mem::replace(&mut self.log_dropped, 0)
    }

    /// Emits all queued deferred safety log events on a non-RT thread.
    pub fn drain_log_events_off_thread(&mut self) {
        let lost = self.take_log_dropped();
        emit_safety_log_dropped(lost);
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

    /// Resets a uniquely-owned retired `SafetyState` off-thread for recycling into either the
    /// formatted-message pool (`Other(String::with_capacity(SAFETY_LOG_TEXT_CAPACITY))`) or the
    /// general pool (`Idle`).
    pub fn reset_recycled_fault_arc(slot: &mut SafetyState) {
        if let SafetyState::Fault(FaultReason::Other(s)) = slot {
            if s.capacity() >= SAFETY_LOG_TEXT_CAPACITY && s.capacity() <= SAFETY_LOG_TEXT_CAPACITY * 2 {
                s.clear();
                return;
            }
            *slot = SafetyState::Fault(FaultReason::Other(String::with_capacity(SAFETY_LOG_TEXT_CAPACITY)));
        } else {
            *slot = SafetyState::Idle;
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
    pub fn latched_fault_kinds(&self) -> u8 {
        self.latched_fault_kinds
    }

    /// Last inline detail recorded for `kind` since the last clear, if any.
    #[inline]
    pub fn latched_fault_detail(&self, kind: FaultKind) -> Option<&LatchedFaultDetail> {
        self.latched_fault_details[kind as usize].as_ref()
    }

    /// Empties the preallocated fault-slot pool for deterministic exhaustion tests.
    #[doc(hidden)]
    pub fn drain_fault_arc_pool_for_test(&mut self) -> Vec<Arc<SafetyState>> {
        let mut drained = Vec::with_capacity(self.fault_arc_pool_len);
        while self.fault_arc_pool_len > 0 {
            self.fault_arc_pool_len -= 1;
            if let Some(arc) = self.fault_arc_pool[self.fault_arc_pool_len].take() {
                drained.push(arc);
            }
        }
        drained
    }

    fn record_latched_fault(&mut self, kind: FaultKind, detail: LatchedFaultDetail) {
        self.latched_fault_kinds |= kind.bit();
        self.latched_fault_details[kind as usize] = Some(detail);
    }

    fn any_latched_non_recoverable(&self) -> bool {
        self.latched_fault_details
            .iter()
            .flatten()
            .any(|d| !d.code.recoverable())
    }

    fn emit_or_queue_fault_log(&mut self, reason: &BoundedFaultReason, code: FaultCode) {
        if self.deferred_logging {
            let from = BoundedSafetyState::from_state(&self.state_arc);
            self.queue_log_event(SafetyLogEvent::Fault { from, reason: *reason, code });
        } else {
            tracing::warn!("Safety: {:?} -> Fault({:?}) [{code:?}]", *self.state_arc, reason);
        }
    }

    /// Returns a clean, uniquely owned `SafetyState::Idle` or pre-sized empty `Other(String)`
    /// `Arc` recycled off-thread back into the matching bounded fault slot pool; returns
    /// `Err(arc)` without dropping if rejected.
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
        Arc::ptr_eq(arc, &self.bus_loss_arc)
            || Arc::ptr_eq(arc, &self.watchdog_timeout_arc)
            || Arc::ptr_eq(arc, &self.detail_unavailable_arc)
            || self.static_fault_arcs.iter().any(|a| Arc::ptr_eq(arc, a))
    }

    fn allocate_fault_arc(&mut self, reason: FaultReason) -> Result<Arc<SafetyState>, FaultReason> {
        match reason {
            FaultReason::BusLoss => Ok(Arc::clone(&self.bus_loss_arc)),
            FaultReason::WatchdogTimeout => Ok(Arc::clone(&self.watchdog_timeout_arc)),
            other => {
                while self.fault_arc_pool_len > 0 {
                    self.fault_arc_pool_len -= 1;
                    if let Some(mut arc) = self.fault_arc_pool[self.fault_arc_pool_len].take() {
                        if let Some(slot) = Arc::get_mut(&mut arc) {
                            *slot = SafetyState::Fault(other);
                            return Ok(arc);
                        }
                    }
                }
                Err(other)
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

    /// Advance epoch and latch E-Stop immediately
    pub fn trigger_e_stop(&mut self) -> u64 {
        let prev_not_estop = *self.state_arc != SafetyState::EStop;
        if !self.deferred_logging {
            if prev_not_estop {
                tracing::warn!("Safety: {:?} -> EStop", *self.state_arc);
            }
            let _ = self.try_replace_state_arc(Arc::clone(&self.estop_arc));
        } else {
            let prev = prev_not_estop.then(|| BoundedSafetyState::from_state(&self.state_arc));
            let replaced = self.try_replace_state_arc(Arc::clone(&self.estop_arc));
            if replaced {
                if let Some(from) = prev {
                    self.queue_log_event(SafetyLogEvent::EStop { from });
                }
            }
        }
        self.epoch_id = self.epoch_id.wrapping_add(1);
        self.epoch_id
    }

    /// Structured cause of the current Fault / EStop; None in any other state.
    /// The `code` of [`Self::fault`] without building its message.
    pub fn fault_code(&self) -> Option<FaultCode> {
        match &*self.state_arc {
            SafetyState::EStop => Some(FaultCode::Estop),
            SafetyState::Fault(reason) => Some(self.fault_code.unwrap_or_else(|| FaultCode::from_reason(reason))),
            _ => None,
        }
    }

    pub fn fault(&self) -> Option<FaultInfo> {
        let mut message = BoundedText::empty();
        let code = match &*self.state_arc {
            SafetyState::EStop => {
                message.push_str("E-stop latched");
                FaultCode::Estop
            }
            SafetyState::Fault(reason) => {
                reason.write_message_into(&mut message);
                self.fault_code.unwrap_or_else(|| FaultCode::from_reason(reason))
            }
            _ => return None,
        };
        Some(FaultInfo {
            code,
            message,
            recoverable: code.recoverable() && !self.any_latched_non_recoverable(),
        })
    }

    /// Advance epoch and latch a preallocated static fault with zero heap allocations and zero
    /// pool consumption.
    pub fn trigger_static_fault(&mut self, fault: StaticFault) -> u64 {
        let code = fault.code();
        let bounded = BoundedFaultReason::Other(BoundedText::new(fault.message()));
        self.record_latched_fault(FaultKind::Other, LatchedFaultDetail { code, reason: bounded });
        self.emit_or_queue_fault_log(&bounded, code);
        if self.can_accept_fault_transition() {
            self.fault_code = Some(code);
            let replaced = self.try_replace_state_arc(Arc::clone(&self.static_fault_arcs[fault as usize]));
            debug_assert!(replaced);
        }
        self.epoch_id = self.epoch_id.wrapping_add(1);
        self.epoch_id
    }

    /// Advance epoch and latch a dynamically formatted fault message completely inline on the RT
    /// thread (0 heap alloc, 0 heap free) using the pre-sized formatted-message slot pool.
    pub fn trigger_fault_formatted(
        &mut self,
        code: FaultCode,
        kind: FaultKind,
        fmt_msg: impl FnOnce(&mut BoundedText<SAFETY_LOG_TEXT_CAPACITY>),
    ) -> u64 {
        let mut msg = BoundedText::<SAFETY_LOG_TEXT_CAPACITY>::empty();
        fmt_msg(&mut msg);
        let bounded = BoundedFaultReason::Other(msg);
        self.record_latched_fault(kind, LatchedFaultDetail { code, reason: bounded });
        self.emit_or_queue_fault_log(&bounded, code);
        if self.can_accept_fault_transition() {
            let mut next_arc = None;
            while self.formatted_fault_arc_pool_len > 0 {
                self.formatted_fault_arc_pool_len -= 1;
                if let Some(mut arc) = self.formatted_fault_arc_pool[self.formatted_fault_arc_pool_len].take() {
                    if let Some(SafetyState::Fault(FaultReason::Other(s))) = Arc::get_mut(&mut arc) {
                        if s.capacity() >= msg.as_str().len() {
                            s.clear();
                            s.push_str(msg.as_str());
                            next_arc = Some(arc);
                            break;
                        }
                    }
                }
            }
            if let Some(arc) = next_arc {
                self.fault_code = Some(code);
                let replaced = self.try_replace_state_arc(arc);
                debug_assert!(replaced);
            } else if !matches!(*self.state_arc, SafetyState::Fault(_)) {
                self.fault_code = Some(code);
                let replaced = self.try_replace_state_arc(Arc::clone(&self.detail_unavailable_arc));
                debug_assert!(replaced);
            }
        }
        self.epoch_id = self.epoch_id.wrapping_add(1);
        self.epoch_id
    }

    /// Advance epoch and latch Fault immediately; the code follows from the reason.
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

    /// Advance epoch and latch Fault lazily: when off-thread reclamation is saturated or the
    /// bounded fault slot pool is empty, skips `make_reason` (0 heap alloc / 0 drop on RT) while
    /// STILL latching the fault kind with `"detail unavailable"` (and moving a non-faulted state
    /// to the preallocated `"detail unavailable"` fault Arc without calling `Arc::new`).
    pub fn trigger_fault_coded_lazy(
        &mut self,
        code: FaultCode,
        make_reason: impl FnOnce() -> FaultReason,
    ) -> u64 {
        if !self.can_accept_fault_transition() || self.fault_arc_pool_len == 0 {
            let kind = FaultKind::from_code(code);
            let reason = BoundedFaultReason::Other(BoundedText::new(DETAIL_UNAVAILABLE));
            self.record_latched_fault(kind, LatchedFaultDetail { code, reason });
            self.emit_or_queue_fault_log(&reason, code);
            if !matches!(*self.state_arc, SafetyState::Fault(_)) {
                self.fault_code = Some(code);
                let replaced = self.try_replace_state_arc(Arc::clone(&self.detail_unavailable_arc));
                debug_assert!(replaced);
            }
            self.epoch_id = self.epoch_id.wrapping_add(1);
            return self.epoch_id;
        }
        self.trigger_fault_coded(code, make_reason())
    }

    /// Fallible fault latch that ALWAYS records the fault kind in the fixed-size latched record
    /// (and never falls back to `Arc::new`). When off-thread reclamation is saturated or the
    /// preallocated fault-slot pool is empty, keeps the current `Fault` Arc (or switches a
    /// non-faulted state to the preallocated `"detail unavailable"` fault Arc) and returns
    /// `Err((epoch, reason))` so the caller does not drop heap-bearing `reason` on the RT thread.
    pub fn try_trigger_fault_coded(
        &mut self,
        code: FaultCode,
        reason: FaultReason,
    ) -> Result<u64, (u64, FaultReason)> {
        let kind = FaultKind::from_reason(&reason);
        let bounded_reason = BoundedFaultReason::from_reason(&reason);
        self.record_latched_fault(kind, LatchedFaultDetail { code, reason: bounded_reason });
        if !self.can_accept_fault_transition() {
            self.emit_or_queue_fault_log(&bounded_reason, code);
            self.epoch_id = self.epoch_id.wrapping_add(1);
            return Err((self.epoch_id, reason));
        }
        let next_arc = match self.allocate_fault_arc(reason) {
            Ok(arc) => arc,
            Err(reason) => {
                self.emit_or_queue_fault_log(&bounded_reason, code);
                if !matches!(*self.state_arc, SafetyState::Fault(_)) {
                    self.fault_code = Some(code);
                    let replaced = self.try_replace_state_arc(Arc::clone(&self.detail_unavailable_arc));
                    debug_assert!(replaced);
                }
                self.epoch_id = self.epoch_id.wrapping_add(1);
                return Err((self.epoch_id, reason));
            }
        };
        self.emit_or_queue_fault_log(&bounded_reason, code);
        self.fault_code = Some(code);
        let replaced = self.try_replace_state_arc(next_arc);
        debug_assert!(replaced);
        self.epoch_id = self.epoch_id.wrapping_add(1);
        Ok(self.epoch_id)
    }

    /// [`Self::trigger_fault`] with the code recorded by the trigger site.
    pub fn trigger_fault_coded(&mut self, code: FaultCode, reason: FaultReason) -> u64 {
        match self.try_trigger_fault_coded(code, reason) {
            Ok(ep) => ep,
            Err((ep, _)) => ep,
        }
    }

    /// Attempt to clear E-Stop or Fault back to Idle without heap allocation.
    /// Epoch is incremented to guarantee that any commands issued prior to fault clear are invalidated.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_fault_bounded(&mut self) -> Result<SafetyState, ClearFaultRefusal> {
        match &*self.state_arc {
            SafetyState::EStop | SafetyState::Fault(_) => {
                if !self.try_replace_state_arc(Arc::clone(&self.idle_arc)) {
                    return Err(ClearFaultRefusal::ReclaimQueueFull);
                }
                if self.deferred_logging {
                    self.queue_log_event(SafetyLogEvent::Cleared);
                } else {
                    tracing::info!("Safety: cleared -> Idle");
                }
                self.fault_code = None;
                self.latched_fault_kinds = 0;
                self.latched_fault_details = [None; FaultKind::COUNT];
                self.epoch_id = self.epoch_id.wrapping_add(1);
                Ok(SafetyState::Idle)
            }
            other => Ok(other.clone()),
        }
    }

    /// Attempt to clear E-Stop or Fault back to Idle.
    /// Epoch is incremented to guarantee that any commands issued prior to fault clear are invalidated.
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
