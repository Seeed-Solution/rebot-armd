use thiserror::Error;
use crate::model::JointLimit;

#[derive(Debug, Clone, PartialEq, Error)]
pub enum TrajectoryError {
    #[error("Duration must be strictly positive, got {0:.4}s")]
    InvalidDuration(f64),

    #[error("Dimension mismatch: start has {start} joints, target has {target}")]
    DimensionMismatch { start: usize, target: usize },

    #[error("Empty joint list")]
    EmptyJoints,

    #[error("Invalid non-finite float value (NaN or Inf) encountered")]
    InvalidValue,

    #[error("Trajectory constraints unattainable: limits exceeded after maximum iterations")]
    ConstraintsUnattainable,
}

/// 1D Quintic polynomial: s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuinticPolynomial {
    pub a0: f64,
    pub a1: f64,
    pub a2: f64,
    pub a3: f64,
    pub a4: f64,
    pub a5: f64,
    pub duration: f64,
    pub p0: f64,
    pub p1: f64,
    pub v0: f64,
    pub v1: f64,
    pub a_start: f64,
    pub a_end: f64,
}

impl QuinticPolynomial {
    /// Construct a quintic polynomial given boundary conditions at t=0 and t=T:
    /// s(0) = p0, s'(0) = v0, s''(0) = a0
    /// s(T) = p1, s'(T) = v1, s''(T) = a1
    pub fn new(
        p0: f64,
        v0: f64,
        a0: f64,
        p1: f64,
        v1: f64,
        a1: f64,
        t_duration: f64,
    ) -> Result<Self, TrajectoryError> {
        if !p0.is_finite()
            || !v0.is_finite()
            || !a0.is_finite()
            || !p1.is_finite()
            || !v1.is_finite()
            || !a1.is_finite()
        {
            return Err(TrajectoryError::InvalidValue);
        }
        if !t_duration.is_finite() || t_duration <= 1e-9 {
            return Err(TrajectoryError::InvalidDuration(t_duration));
        }

        let t = t_duration;
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;

        let c0 = p0;
        let c1 = v0;
        let c2 = 0.5 * a0;

        let h = p1 - p0 - v0 * t - 0.5 * a0 * t2;
        let g = v1 - v0 - a0 * t;
        let f = a1 - a0;

        let c3 = (10.0 * h - 4.0 * g * t + 0.5 * f * t2) / t3;
        let c4 = (-15.0 * h + 7.0 * g * t - f * t2) / t4;
        let c5 = (6.0 * h - 3.0 * g * t + 0.5 * f * t2) / t5;

        Ok(Self {
            a0: c0,
            a1: c1,
            a2: c2,
            a3: c3,
            a4: c4,
            a5: c5,
            duration: t,
            p0,
            p1,
            v0,
            v1,
            a_start: a0,
            a_end: a1,
        })
    }

    /// Sample position at time t
    pub fn position(&self, t: f64) -> f64 {
        if t <= 0.0 {
            return self.p0;
        }
        if t >= self.duration {
            return self.p1;
        }
        self.a0
            + t * (self.a1
                + t * (self.a2 + t * (self.a3 + t * (self.a4 + t * self.a5))))
    }

    /// Sample velocity at time t
    pub fn velocity(&self, t: f64) -> f64 {
        if t <= 0.0 {
            return self.v0;
        }
        if t >= self.duration {
            return self.v1;
        }
        self.a1
            + t * (2.0 * self.a2
                + t * (3.0 * self.a3 + t * (4.0 * self.a4 + 5.0 * self.a5 * t)))
    }

    /// Sample acceleration at time t
    pub fn acceleration(&self, t: f64) -> f64 {
        if t <= 0.0 {
            return self.a_start;
        }
        if t >= self.duration {
            return self.a_end;
        }
        2.0 * self.a2
            + t * (6.0 * self.a3 + t * (12.0 * self.a4 + 20.0 * self.a5 * t))
    }
}

/// Trajectory point sample across all joints
#[derive(Debug, Clone, PartialEq)]
pub struct TrajectoryPoint {
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    pub accelerations: Vec<f32>,
}

/// Multi-joint quintic polynomial trajectory
#[derive(Debug, Clone)]
pub struct MultiJointQuinticTrajectory {
    pub polynomials: Vec<QuinticPolynomial>,
    pub duration: f64,
    pub elapsed: f64,
}

impl MultiJointQuinticTrajectory {
    pub fn new(
        start_pos: &[f32],
        start_vel: &[f32],
        target_pos: &[f32],
        target_vel: &[f32],
        duration: f64,
    ) -> Result<Self, TrajectoryError> {
        let dof = start_pos.len();
        if dof == 0 {
            return Err(TrajectoryError::EmptyJoints);
        }
        if target_pos.len() != dof || start_vel.len() != dof || target_vel.len() != dof {
            return Err(TrajectoryError::DimensionMismatch {
                start: dof,
                target: target_pos.len(),
            });
        }
        for &val in start_pos.iter().chain(start_vel).chain(target_pos).chain(target_vel) {
            if !val.is_finite() {
                return Err(TrajectoryError::InvalidValue);
            }
        }
        if !duration.is_finite() || duration <= 1e-9 {
            return Err(TrajectoryError::InvalidDuration(duration));
        }

        let mut polynomials = Vec::with_capacity(dof);
        for i in 0..dof {
            // zero start and end acceleration to guarantee jerk-bounded smooth motion
            let poly = QuinticPolynomial::new(
                start_pos[i] as f64,
                start_vel[i] as f64,
                0.0,
                target_pos[i] as f64,
                target_vel[i] as f64,
                0.0,
                duration,
            )?;
            polynomials.push(poly);
        }

        Ok(Self {
            polynomials,
            duration,
            elapsed: 0.0,
        })
    }

    /// Advance elapsed time and return current sample point
    pub fn step(&mut self, dt: f64) -> TrajectoryPoint {
        self.elapsed += dt;
        self.sample(self.elapsed)
    }

    /// Sample at specific elapsed time
    pub fn sample(&self, t: f64) -> TrajectoryPoint {
        let dof = self.polynomials.len();
        let mut positions = Vec::with_capacity(dof);
        let mut velocities = Vec::with_capacity(dof);
        let mut accelerations = Vec::with_capacity(dof);

        for poly in &self.polynomials {
            positions.push(poly.position(t) as f32);
            velocities.push(poly.velocity(t) as f32);
            accelerations.push(poly.acceleration(t) as f32);
        }

        TrajectoryPoint {
            positions,
            velocities,
            accelerations,
        }
    }

    pub fn is_finished(&self) -> bool {
        self.elapsed >= self.duration
    }
}

/// Trajectory planner that automatically calculates duration if not provided
pub struct TrajectoryPlanner;

impl TrajectoryPlanner {
    pub fn plan(
        start_pos: &[f32],
        start_vel: &[f32],
        target_pos: &[f32],
        target_vel: &[f32],
        requested_duration: Option<f64>,
        limits: &[JointLimit],
    ) -> Result<MultiJointQuinticTrajectory, TrajectoryError> {
        let dof = start_pos.len();
        if dof == 0 {
            return Err(TrajectoryError::EmptyJoints);
        }
        if target_pos.len() != dof || start_vel.len() != dof || target_vel.len() != dof {
            return Err(TrajectoryError::DimensionMismatch {
                start: dof,
                target: target_pos.len(),
            });
        }

        // Check for non-finite values (NaN / Inf)
        for &val in start_pos.iter().chain(start_vel).chain(target_pos).chain(target_vel) {
            if !val.is_finite() {
                return Err(TrajectoryError::InvalidValue);
            }
        }

        if let Some(d) = requested_duration {
            if !d.is_finite() || d <= 1e-9 {
                return Err(TrajectoryError::InvalidDuration(d));
            }
        }

        // Calculate minimum duration based on max velocity and max acceleration limits
        // For rest-to-rest quintic polynomial:
        // v_peak = 1.875 * |delta_p| / T  => T >= 1.875 * |delta_p| / v_max
        // a_peak = 5.7735 * |delta_p| / T^2 => T >= sqrt(5.7735 * |delta_p| / a_max)
        let mut t_min: f64 = 0.0;
        for i in 0..dof {
            let delta_p = (target_pos[i] - start_pos[i]).abs() as f64;
            if i < limits.len() {
                let v_max = limits[i].max_velocity as f64;
                let a_max = limits[i].max_acceleration as f64;

                if v_max > 1e-6 {
                    let t_v = 1.875 * delta_p / v_max;
                    if t_v > t_min {
                        t_min = t_v;
                    }
                }
                if a_max > 1e-6 {
                    let t_a = (5.7735 * delta_p / a_max).sqrt();
                    if t_a > t_min {
                        t_min = t_a;
                    }
                }
            }
        }

        // Final duration: initial estimate
        let mut duration = requested_duration.unwrap_or(t_min).max(t_min).max(0.04);

        // Iterative multi-point verification for physical limits under non-zero boundary velocities
        const MAX_ITERATIONS: usize = 10;
        const NUM_SAMPLES: usize = 20;

        for _ in 0..MAX_ITERATIONS {
            let traj = MultiJointQuinticTrajectory::new(
                start_pos,
                start_vel,
                target_pos,
                target_vel,
                duration,
            )?;

            let mut exceeds_limits = false;
            for sample_idx in 0..=NUM_SAMPLES {
                let t = duration * (sample_idx as f64) / (NUM_SAMPLES as f64);
                let point = traj.sample(t);

                for i in 0..dof {
                    if i < limits.len() {
                        let v_max = limits[i].max_velocity * 1.001;
                        let a_max = limits[i].max_acceleration * 1.001;

                        // Check position soft limits
                        if point.positions[i] < limits[i].min_position - 1e-4
                            || point.positions[i] > limits[i].max_position + 1e-4
                        {
                            exceeds_limits = true;
                            break;
                        }
                        if limits[i].max_velocity > 1e-6 && point.velocities[i].abs() > v_max {
                            exceeds_limits = true;
                            break;
                        }
                        if limits[i].max_acceleration > 1e-6 && point.accelerations[i].abs() > a_max {
                            exceeds_limits = true;
                            break;
                        }
                    }
                }
                if exceeds_limits {
                    break;
                }
            }

            if !exceeds_limits {
                return Ok(traj);
            }

            duration *= 1.2;
        }

        Err(TrajectoryError::ConstraintsUnattainable)
    }
}
