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
            lease_timeout_ms: 500,
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
            ],
            limits: config.limits.clone(),
        }
    }
}
