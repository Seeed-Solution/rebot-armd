use serde::{Deserialize, Serialize};
use crate::safety::SafetyState;
use crate::driver::{DriverCapabilities, EnableState};

/// Limits for a single joint
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct JointLimit {
    pub name: String,
    pub min_position: f32,
    pub max_position: f32,
    pub max_velocity: f32,
    pub max_acceleration: f32,
}

impl JointLimit {
    pub fn new(
        name: impl Into<String>,
        min_position: f32,
        max_position: f32,
        max_velocity: f32,
        max_acceleration: f32,
    ) -> Self {
        Self {
            name: name.into(),
            min_position,
            max_position,
            max_velocity,
            max_acceleration,
        }
    }
}

/// Dynamic state of a single joint
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct JointState {
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub temperature: f32,
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            torque: 0.0,
            temperature: 25.0,
        }
    }
}

/// Hardware-specific evidence required before automatic torque release.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RestingPosePolicy {
    pub position_tolerances: Vec<f32>,
    pub max_total_deviation: f32,
    pub max_velocity: f32,
    pub support_torque_limits: Vec<(usize, f32)>,
    pub feedback_max_age_ms: u64,
}

impl RestingPosePolicy {
    pub fn rebot_dm() -> Self {
        Self {
            position_tolerances: vec![0.15, 0.08, 0.08, 0.08, 0.08, 0.08],
            max_total_deviation: 0.22,
            max_velocity: 0.015,
            support_torque_limits: vec![(1, 1.5), (2, 4.5)],
            feedback_max_age_ms: 100,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FollowingErrorPolicy {
    pub max_error_rad: f32,
    pub persistence_ms: u64,
}
impl Default for FollowingErrorPolicy {
    fn default() -> Self { Self { max_error_rad: 0.5, persistence_ms: 500 } }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum GravityModel { RebotDevArm }

/// Estimated Damiao POS_VEL lag gain: tracking error ~= v / POSVEL_LAG_GAIN rad at
/// v rad/s. Estimate from the 2026-11 tracking-fault analysis; needs verification
/// on hardware (record |commanded - measured| during presets to fit it).
pub const POSVEL_LAG_GAIN: f32 = 2.2;

/// Robotic arm hardware/control configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArmConfig {
    pub dof: usize,
    pub joint_names: Vec<String>,
    pub limits: Vec<JointLimit>,
    pub home_positions: Vec<f32>,
    pub lease_timeout_ms: u64,
    pub max_temperature: f32,
    pub control_frequency_hz: u32,
    /// None forbids automatic hardware torque release without a calibrated profile.
    #[serde(default)]
    pub resting_pose: Option<RestingPosePolicy>,
    #[serde(default)]
    pub following_error: FollowingErrorPolicy,
    /// Soft jerk preference; braking/position constraints take precedence.
    #[serde(default)]
    pub max_jerk: Option<f32>,
    #[serde(default)]
    pub gravity_model: Option<GravityModel>,
    /// Joint speed cap (rad/s) for planned motion (jog, home, presets, park) and
    /// servo stepping. Applies to every joint except the 7-DOF gripper (index 6).
    /// Keeps POS_VEL lag below the following-error bound. None = no extra cap.
    #[serde(default)]
    pub planned_max_velocity: Option<f32>,
}

impl Default for ArmConfig {
    fn default() -> Self {
        let dof = 6;
        let joint_names = vec![
            "joint_1".to_string(),
            "joint_2".to_string(),
            "joint_3".to_string(),
            "joint_4".to_string(),
            "joint_5".to_string(),
            "joint_6".to_string(),
        ];

        let limits = vec![
            JointLimit::new("joint_1", -3.14159, 3.14159, 2.0, 5.0),
            JointLimit::new("joint_2", -2.0944, 2.0944, 2.0, 5.0),
            JointLimit::new("joint_3", -2.61799, 2.61799, 2.0, 5.0),
            JointLimit::new("joint_4", -3.14159, 3.14159, 3.0, 8.0),
            JointLimit::new("joint_5", -2.0944, 2.0944, 3.0, 8.0),
            JointLimit::new("joint_6", -6.28318, 6.28318, 4.0, 10.0),
        ];

        let home_positions = vec![0.0; dof];

        Self {
            dof,
            joint_names,
            limits,
            home_positions,
            lease_timeout_ms: 2500,
            max_temperature: 70.0,
            control_frequency_hz: 50,
            resting_pose: None,
            following_error: FollowingErrorPolicy::default(),
            max_jerk: None,
            gravity_model: None,
            planned_max_velocity: None,
        }
    }
}

impl ArmConfig {
    /// Validate configuration invariants and auto-normalize joint names
    pub fn validate(&mut self) -> Result<(), String> {
        if self.dof == 0 {
            return Err("DOF cannot be zero".into());
        }
        if self.limits.len() != self.dof {
            return Err(format!("Limits length {} does not match DOF {}", self.limits.len(), self.dof));
        }
        if self.home_positions.len() != self.dof {
            return Err(format!("Home positions length {} does not match DOF {}", self.home_positions.len(), self.dof));
        }
        if self.joint_names.len() != self.dof {
            self.joint_names.resize_with(self.dof, || "joint".to_string());
        }
        if self.control_frequency_hz == 0 || self.control_frequency_hz > 1000
            || !self.max_temperature.is_finite() || self.max_temperature <= 5.0
            || !self.following_error.max_error_rad.is_finite() || self.following_error.max_error_rad <= 0.0
            || self.following_error.persistence_ms == 0
            || self.max_jerk.is_some_and(|j| !j.is_finite() || j <= 0.0) {
            return Err("Invalid control, temperature, following-error or jerk configuration".into());
        }
        if let Some(cap) = self.planned_max_velocity {
            if !cap.is_finite() || cap <= 0.0 {
                return Err("planned_max_velocity must be positive finite".into());
            }
            if cap / POSVEL_LAG_GAIN >= self.following_error.max_error_rad {
                return Err(format!(
                    "planned_max_velocity {cap} rad/s implies an expected lag {:.3} rad at or above the following-error bound {} rad",
                    cap / POSVEL_LAG_GAIN, self.following_error.max_error_rad));
            }
        }
        if let Some(policy) = &self.resting_pose {
            if policy.position_tolerances.is_empty() || policy.position_tolerances.len() > self.dof
                || policy.position_tolerances.iter().any(|v| !v.is_finite() || *v <= 0.0)
                || !policy.max_total_deviation.is_finite() || policy.max_total_deviation <= 0.0
                || !policy.max_velocity.is_finite() || policy.max_velocity <= 0.0
                || policy.feedback_max_age_ms == 0
                || policy.support_torque_limits.iter().any(|(i, t)| *i >= self.dof || !t.is_finite() || *t < 0.0) {
                return Err("Invalid resting-pose policy".into());
            }
        }
        for (i, lim) in self.limits.iter().enumerate() {
            if !lim.min_position.is_finite() || !lim.max_position.is_finite() || lim.min_position >= lim.max_position {
                return Err(format!("Joint {} min position >= max position", i));
            }
            if lim.max_velocity <= 0.0 || !lim.max_velocity.is_finite() || lim.max_acceleration <= 0.0 || !lim.max_acceleration.is_finite() {
                return Err(format!("Joint {} max_velocity must be positive finite", i));
            }
        }
        if self.home_positions.iter().zip(&self.limits).any(|(p, l)| !p.is_finite() || *p < l.min_position || *p > l.max_position) {
            return Err("Home positions must be finite and within joint limits".into());
        }
        Ok(())
    }

    /// Factory for reBot Arm B601-DM with 6 arm joints + 1 gripper joint (7 DOF total)
    pub fn rebot_dm() -> Self {
        let dof = 7;
        let joint_names = vec![
            "joint_1".to_string(),
            "joint_2".to_string(),
            "joint_3".to_string(),
            "joint_4".to_string(),
            "joint_5".to_string(),
            "joint_6".to_string(),
            "gripper".to_string(),
        ];

        let limits = vec![
            JointLimit::new("joint_1", -3.14159, 3.14159, 2.0, 5.0),
            JointLimit::new("joint_2", -2.0944, 2.0944, 2.0, 5.0),
            JointLimit::new("joint_3", -2.61799, 2.61799, 2.0, 5.0),
            JointLimit::new("joint_4", -3.14159, 3.14159, 3.0, 8.0),
            JointLimit::new("joint_5", -2.0944, 2.0944, 3.0, 8.0),
            JointLimit::new("joint_6", -6.28318, 6.28318, 4.0, 10.0),
            // Measured on hardware 2026-11-06: closed +0.03 rad, mechanical full open -5.87 rad.
            JointLimit::new("gripper", -5.75, 0.10, 2.0, 5.0),
        ];

        let home_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0];

        Self {
            dof,
            joint_names,
            limits,
            home_positions,
            lease_timeout_ms: 2500,
            max_temperature: 70.0,
            control_frequency_hz: 50,
            resting_pose: Some(RestingPosePolicy::rebot_dm()),
            following_error: FollowingErrorPolicy::default(),
            max_jerk: Some(50.0),
            gravity_model: Some(GravityModel::RebotDevArm),
            planned_max_velocity: Some(0.7),
        }
    }
}

/// Full arm telemetry / observation
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ArmObservation {
    pub timestamp: f64,
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    pub torques: Vec<f32>,
    pub temperatures: Vec<f32>,
    pub is_torqued: bool,
    pub safety_state: SafetyState,
    pub epoch_id: u64,
    #[serde(default)]
    pub gravity_compensated: bool,
    #[serde(default)]
    pub gravity_torques: Vec<f32>,
    #[serde(default)]
    pub driver_capabilities: DriverCapabilities,
    #[serde(default)]
    pub enable_state: EnableState,
    #[serde(default)]
    pub feedback_age_ms: Vec<Option<u64>>,
    #[serde(default)]
    pub disable_pending: bool,
    #[serde(default)]
    pub feedback_valid: bool,
    /// Reference positions sent to the motors this tick (rad).
    #[serde(default)]
    pub commanded_positions: Vec<f32>,
}

/// Field description in the observation schema
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SchemaField {
    pub name: String,
    pub dtype: String,
    pub shape: Vec<usize>,
    pub unit: Option<String>,
}

/// Observation schema for clients
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObservationSchema {
    pub dof: usize,
    pub joint_names: Vec<String>,
    pub fields: Vec<SchemaField>,
    pub limits: Vec<JointLimit>,
}

impl ObservationSchema {
    pub fn from_config(config: &ArmConfig) -> Self {
        Self {
            dof: config.dof,
            joint_names: config.joint_names.clone(),
            fields: vec![
                SchemaField {
                    name: "timestamp".to_string(),
                    dtype: "float64".to_string(),
                    shape: vec![1],
                    unit: Some("seconds".to_string()),
                },
                SchemaField {
                    name: "positions".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("radians".to_string()),
                },
                SchemaField {
                    name: "commanded_positions".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("radians".to_string()),
                },
                SchemaField {
                    name: "velocities".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("rad/s".to_string()),
                },
                SchemaField {
                    name: "torques".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("Nm".to_string()),
                },
                SchemaField {
                    name: "temperatures".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![config.dof],
                    unit: Some("celsius".to_string()),
                },
                SchemaField {
                    name: "is_torqued".to_string(),
                    dtype: "bool".to_string(),
                    shape: vec![1],
                    unit: None,
                },
                SchemaField {
                    name: "epoch_id".to_string(),
                    dtype: "uint64".to_string(),
                    shape: vec![1],
                    unit: None,
                },
                SchemaField {
                    name: "gravity_compensated".to_string(),
                    dtype: "bool".to_string(),
                    shape: vec![1],
                    unit: None,
                },
                SchemaField {
                    name: "gravity_torques".to_string(),
                    dtype: "float32".to_string(),
                    shape: vec![6],
                    unit: Some("Nm".to_string()),
                },
            ],
            limits: config.limits.clone(),
        }
    }
}
