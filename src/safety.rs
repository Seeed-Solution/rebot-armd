use serde::{Deserialize, Serialize};
use thiserror::Error;
use crate::model::JointLimit;

/// Stop policy strategy (review)
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

    #[error("Target outside the safe workspace: check point {point} {limit} {value:.3} m beyond bound {bound:.3} m")]
    WorkspaceViolation { point: usize, limit: String, value: f32, bound: f32 },
}

/// Safety Manager tracking safety state machine and command generation ID
#[derive(Debug, Clone)]
pub struct SafetyManager {
    state: SafetyState,
    epoch_id: u64,
}

impl Default for SafetyManager {
    fn default() -> Self {
        Self {
            state: SafetyState::Idle,
            epoch_id: 1,
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

    /// Advance epoch and latch Fault immediately
    pub fn trigger_fault(&mut self, reason: FaultReason) -> u64 {
        tracing::warn!("Safety: {:?} -> Fault({:?})", self.state, reason);
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

        // 2. Epoch generation ID validation (review: single authority & epoch latch)
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
