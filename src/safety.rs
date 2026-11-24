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
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct FaultInfo {
    pub code: FaultCode,
    pub message: String,
    pub recoverable: bool,
}

crate::fieldwise_clone!(FaultInfo { code, message, recoverable });

impl FaultReason {
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

    #[error("{message}")]
    TorqueOffWouldDrop {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        joint: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        position: Option<f32>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        rest: Option<f32>,
        message: String,
    },

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

/// Safety Manager tracking safety state machine and command generation ID
#[derive(Debug, Clone)]
pub struct SafetyManager {
    state_arc: Arc<SafetyState>,
    idle_arc: Arc<SafetyState>,
    armed_arc: Arc<SafetyState>,
    moving_arc: Arc<SafetyState>,
    estop_arc: Arc<SafetyState>,
    retired_state_arcs: [Option<Arc<SafetyState>>; RETIRED_STATE_CAPACITY],
    retired_head: usize,
    retired_len: usize,
    off_thread_reclaim: bool,
    epoch_id: u64,
    /// Code recorded by the trigger site of the current Fault.
    fault_code: Option<FaultCode>,
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
            retired_state_arcs: std::array::from_fn(|_| None),
            retired_head: 0,
            retired_len: 0,
            off_thread_reclaim: false,
            epoch_id: 1,
            fault_code: None,
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

    /// Enables bounded off-thread fault state reclamation (fail-closed when full) for RT executor mode.
    pub fn set_off_thread_reclaim(&mut self, enabled: bool) {
        self.off_thread_reclaim = enabled;
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

    /// Drops all queued retired fault states on a non-RT thread (legacy/pre-handoff cleanup).
    pub fn drain_retired_state_arcs_off_thread(&mut self) {
        while let Some(arc) = self.pop_retired_state_arc() {
            drop(arc);
        }
    }

    /// Replaces `state_arc` with `next`. When `off_thread_reclaim` is active and the current state
    /// is a `Fault(_)`, queues the replaced `Arc` into `retired_state_arcs` without overwriting any
    /// occupied slot; returns `false` (fail-closed, keeping the current `Fault` latched) if full.
    fn try_replace_state_arc(&mut self, next: Arc<SafetyState>) -> bool {
        if self.off_thread_reclaim && matches!(*self.state_arc, SafetyState::Fault(_)) {
            if self.retired_len >= RETIRED_STATE_CAPACITY {
                return false;
            }
            let prev = std::mem::replace(&mut self.state_arc, next);
            let tail = (self.retired_head + self.retired_len) % RETIRED_STATE_CAPACITY;
            self.retired_state_arcs[tail] = Some(prev);
            self.retired_len += 1;
            return true;
        }
        self.state_arc = next;
        true
    }

    pub fn epoch_id(&self) -> u64 {
        self.epoch_id
    }

    /// Advance epoch and latch E-Stop immediately
    pub fn trigger_e_stop(&mut self) -> u64 {
        if *self.state_arc != SafetyState::EStop {
            tracing::warn!("Safety: {:?} -> EStop", *self.state_arc);
        }
        let _ = self.try_replace_state_arc(Arc::clone(&self.estop_arc));
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
        let (code, message) = match &*self.state_arc {
            SafetyState::EStop => (FaultCode::Estop, "E-stop latched".to_string()),
            SafetyState::Fault(reason) => (self.fault_code.unwrap_or_else(|| FaultCode::from_reason(reason)), reason.message()),
            _ => return None,
        };
        Some(FaultInfo { code, message, recoverable: code.recoverable() })
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
            && self.retired_len >= RETIRED_STATE_CAPACITY)
    }

    /// Advance epoch and latch Fault lazily: when off-thread reclamation is saturated while already
    /// faulted, fails closed without invoking `make_reason` (0 heap alloc / 0 drop on RT).
    pub fn trigger_fault_coded_lazy(
        &mut self,
        code: FaultCode,
        make_reason: impl FnOnce() -> FaultReason,
    ) -> u64 {
        if !self.can_accept_fault_transition() {
            self.epoch_id = self.epoch_id.wrapping_add(1);
            return self.epoch_id;
        }
        self.trigger_fault_coded(code, make_reason())
    }

    /// Fallible fault latch that returns `Err((epoch, reason))` without dropping `reason` when
    /// off-thread reclamation is saturated while already faulted.
    pub fn try_trigger_fault_coded(
        &mut self,
        code: FaultCode,
        reason: FaultReason,
    ) -> Result<u64, (u64, FaultReason)> {
        if !self.can_accept_fault_transition() {
            self.epoch_id = self.epoch_id.wrapping_add(1);
            return Err((self.epoch_id, reason));
        }
        tracing::warn!("Safety: {:?} -> Fault({:?}) [{code:?}]", *self.state_arc, reason);
        self.fault_code = Some(code);
        let replaced = self.try_replace_state_arc(Arc::new(SafetyState::Fault(reason)));
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

    /// Attempt to clear E-Stop or Fault back to Idle.
    /// Epoch is incremented to guarantee that any commands issued prior to fault clear are invalidated.
    pub fn clear_fault(&mut self) -> Result<SafetyState, SafetyError> {
        match &*self.state_arc {
            SafetyState::EStop | SafetyState::Fault(_) => {
                if !self.try_replace_state_arc(Arc::clone(&self.idle_arc)) {
                    return Err(SafetyError::ClearFaultRejected(
                        "Fault state reclamation queue is full; retry after off-thread drain".into(),
                    ));
                }
                tracing::info!("Safety: cleared -> Idle");
                self.fault_code = None;
                self.epoch_id = self.epoch_id.wrapping_add(1);
                Ok(SafetyState::Idle)
            }
            other => Ok(other.clone()),
        }
    }

    /// Set torque enabled/disabled
    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        if self.state_arc.is_stopped() {
            if !enable {
                // Disabling torque while stopped keeps the E-stop / Fault latch intact;
                // rest-pose gating is enforced by the supervisor before reaching here.
                return Ok((*self.state_arc).clone());
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
