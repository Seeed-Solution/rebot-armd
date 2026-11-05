use serde::{Deserialize, Serialize};
use crate::safety::SafetyState;

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
        for (i, lim) in self.limits.iter().enumerate() {
            if lim.min_position >= lim.max_position {
                return Err(format!("Joint {} min position >= max position", i));
            }
            if lim.max_velocity <= 0.0 || !lim.max_velocity.is_finite() {
                return Err(format!("Joint {} max_velocity must be positive finite", i));
            }
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
            JointLimit::new("gripper", -0.05, 0.16, 2.0, 5.0),
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
