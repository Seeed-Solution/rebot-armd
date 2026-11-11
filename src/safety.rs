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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FaultInfo {
    pub code: FaultCode,
    pub message: String,
    pub recoverable: bool,
}

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

    #[error("Arm is not at resting pose (current: {current:?}, expected: {expected:?})")]
    ArmNotAtRestingPose { current: Vec<f32>, expected: Vec<f32> },

    #[error("Motion forbidden in fault/estop state: {0}")]
    MotionForbiddenInFaultState(String),

    #[error("Arm is coming to a safe stop from previous session; motion gate locked")]
    HandoverSettling,

    #[error("Motion busy: an action trajectory is running")]
    MotionBusy,

    #[error("{0}")]
    CollisionHold(String),

    #[error("Target outside the safe workspace: check point {point} {limit} {value:.3} m beyond bound {bound:.3} m")]
    WorkspaceViolation { point: usize, limit: String, value: f32, bound: f32 },
}

/// Safety Manager tracking safety state machine and command generation ID
#[derive(Debug, Clone)]
pub struct SafetyManager {
    state: SafetyState,
    epoch_id: u64,
    /// Code recorded by the trigger site of the current Fault.
    fault_code: Option<FaultCode>,
}

impl Default for SafetyManager {
    fn default() -> Self {
        Self {
            state: SafetyState::Idle,
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
        &self.state
    }

    pub fn epoch_id(&self) -> u64 {
        self.epoch_id
    }

    /// Advance epoch and latch E-Stop immediately
    pub fn trigger_e_stop(&mut self) -> u64 {
        if self.state != SafetyState::EStop {
            tracing::warn!("Safety: {:?} -> EStop", self.state);
        }
        self.state = SafetyState::EStop;
        self.epoch_id = self.epoch_id.wrapping_add(1);
        self.epoch_id
    }

    /// Structured cause of the current Fault / EStop; None in any other state.
    pub fn fault(&self) -> Option<FaultInfo> {
        let (code, message) = match &self.state {
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

    /// [`Self::trigger_fault`] with the code recorded by the trigger site.
    pub fn trigger_fault_coded(&mut self, code: FaultCode, reason: FaultReason) -> u64 {
        tracing::warn!("Safety: {:?} -> Fault({:?}) [{code:?}]", self.state, reason);
        self.fault_code = Some(code);
        self.state = SafetyState::Fault(reason);
        self.epoch_id = self.epoch_id.wrapping_add(1);
        self.epoch_id
    }

    /// Attempt to clear E-Stop or Fault back to Idle.
    /// Epoch is incremented to guarantee that any commands issued prior to fault clear are invalidated.
    pub fn clear_fault(&mut self) -> Result<SafetyState, SafetyError> {
        match &self.state {
            SafetyState::EStop | SafetyState::Fault(_) => {
                tracing::info!("Safety: {:?} cleared -> Idle", self.state);
                self.state = SafetyState::Idle;
                self.fault_code = None;
                self.epoch_id = self.epoch_id.wrapping_add(1);
                Ok(self.state.clone())
            }
            _ => Ok(self.state.clone()),
        }
    }

    /// Set torque enabled/disabled
    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        if self.state.is_stopped() {
            if !enable {
                return Err(SafetyError::MotionForbiddenInFaultState(
                    "Cannot disable torque in EStop or Fault. Call clear_fault first.".into(),
                ));
            }
            if let SafetyState::Fault(ref reason) = self.state {
                return Err(SafetyError::FaultActive(reason.clone()));
            }
            return Err(SafetyError::EStopActive);
        }

        if enable {
            self.state = SafetyState::Armed;
        } else {
            self.state = SafetyState::Idle;
        }
        Ok(self.state.clone())
    }

    /// Notify that motion has started
    pub fn mark_motion_started(&mut self) -> Result<(), SafetyError> {
        if self.state != SafetyState::Armed && self.state != SafetyState::Moving {
            return Err(SafetyError::NotArmed(self.state.clone()));
        }
        self.state = SafetyState::Moving;
        Ok(())
    }

    /// Notify that motion has completed
    pub fn mark_motion_finished(&mut self) {
        if self.state == SafetyState::Moving {
            self.state = SafetyState::Armed;
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
        match &self.state {
            SafetyState::EStop => return Err(SafetyError::EStopActive),
            SafetyState::Fault(reason) => return Err(SafetyError::FaultActive(reason.clone())),
            SafetyState::Booting | SafetyState::Idle => {
                return Err(SafetyError::NotArmed(self.state.clone()));
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
