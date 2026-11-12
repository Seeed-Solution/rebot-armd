//! Supply power budget: an estimate of the motor supply current from motor feedback.
//!
//! There is no current or voltage sensor. The supply current is estimated each control
//! tick from the measured joint torque τ (N·m, output side) and velocity ω (rad/s) with
//! motor parameters from the datasheets (not measured on this arm):
//!
//! ```text
//! P_j    = max(0, τ_j·ω_j) / efficiency + k_cu · (τ_j / kt_j)² · r_phase_j + idle_w_j
//! regen  = Σ max(0, −τ_j·ω_j) · efficiency          (not credited: the supply cannot sink it)
//! I_bus  = (Σ P_j + base_w) / v_bus
//! ```
//!
//! `kt_j` is the torque constant at the output (N·m per A of phase current, gearbox
//! included), `k_cu · I² · r_phase` the copper loss (k_cu = 1.5 for a three-phase motor with
//! `I` the phase current amplitude and `r_phase` the per-phase resistance), `efficiency` the
//! remaining drive / gearbox loss on the mechanical power. Copper loss is counted in full
//! while regenerating (conservative).
//!
//! Filtering: `fast_a` is a first-order low-pass of the per-tick value with
//! [`FAST_FILTER_S`] (the governor input), `bus_a` one with `filter_s` (display, default
//! 0.25 s). Peaks are kept in fixed bucket windows (5 s and 60 s) of `fast_a`.
//!
//! Per tick the estimate is O(joints) and allocates nothing.

use serde::{Deserialize, Serialize};

use crate::supervisor::SupervisorInner;

/// Most joints the per-tick arrays hold.
pub const MAX_JOINTS: usize = 8;
/// Time constant (s) of the fast filter (the governor input). At 50 Hz this is 2 ticks.
pub const FAST_FILTER_S: f32 = 0.04;
/// Peak window shown with the meter (`observation.power.peak_a`), s.
pub const PEAK_WINDOW_S: f64 = 5.0;

fn default_idle_w() -> f32 { 0.5 }

/// Per-joint motor parameters of `power_budget.joints` (one entry per joint, gripper included).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct JointPower {
    /// Torque constant at the joint output (N·m per A phase current, gearbox included).
    pub kt_nm_per_a: f32,
    /// Phase resistance (Ω).
    pub r_phase_ohm: f32,
    /// Driver / controller power with the motor idle (W).
    #[serde(default = "default_idle_w")]
    pub idle_w: f32,
    /// Viscous friction (N·m per rad/s) added by the planning model. Default 0.
    #[serde(default)]
    pub viscous_nm_s: f32,
    /// Coulomb friction (N·m) added by the planning model. Default 0.
    #[serde(default)]
    pub coulomb_nm: f32,
}

impl JointPower {
    /// Damiao DM-J4340P-2EC (B601-DM J1-J3), datasheet: 40 N·m peak at 19.85 A phase
    /// (40:1, 2.015 N·m/A at the output), phase resistance ≈ 0.72 Ω.
    pub const DM_J4340P: Self = Self { kt_nm_per_a: 2.015, r_phase_ohm: 0.72, idle_w: 0.5, viscous_nm_s: 0.0, coulomb_nm: 0.0 };
    /// Damiao DM-J4310-2EC (B601-DM J4-J7), datasheet: 12.5 N·m at 20 A phase with OC 0.98
    /// (10:1, 0.625 N·m/A at the output), phase resistance ≈ 0.58 Ω.
    pub const DM_J4310: Self = Self { kt_nm_per_a: 0.625, r_phase_ohm: 0.58, idle_w: 0.5, viscous_nm_s: 0.0, coulomb_nm: 0.0 };
}

/// Profile section `power_budget`. Absent: no estimate, no governor, no planning check.
/// Missing fields take the defaults below (the 24 V 6.5 A Mean Well LRS-150-24 in use on the
/// B601-DM; `joints` has no default and must list every joint).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct PowerBudget {
    pub enabled: bool,
    /// Rated continuous output current of the supply (A).
    pub supply_rated_a: f32,
    /// Current at which the supply's overload protection trips (A); informational and the
    /// upper bound of the governor's hard limit.
    pub supply_trip_a: f32,
    /// Bus voltage used to convert power to current (V).
    pub v_bus: f32,
    /// Load on the same supply besides the motors (W), e.g. the reCamera.
    pub base_w: f32,
    /// Copper-loss factor: P_cu = k_cu · (τ / kt)² · r_phase.
    pub k_cu: f32,
    /// Drive and gearbox efficiency on the mechanical power (0, 1].
    pub efficiency: f32,
    /// Time constant (s) of the displayed estimate `bus_a`.
    pub filter_s: f32,
    pub joints: Vec<JointPower>,
}

impl Default for PowerBudget {
    fn default() -> Self {
        Self {
            enabled: true,
            supply_rated_a: 6.5,
            supply_trip_a: 8.5,
            v_bus: 24.0,
            base_w: 4.0,
            k_cu: 1.5,
            efficiency: 0.8,
            filter_s: 0.25,
            joints: Vec::new(),
        }
    }
}

/// One estimate (no allocation).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Estimate {
    /// Supply current (A).
    pub bus_a: f32,
    /// Mechanical power fed back by the motors (W), not credited in `bus_a`.
    pub regen_w: f32,
    /// Per joint share of `bus_a` (A), idle power included; the base load is not in it.
    pub joint_a: [f32; MAX_JOINTS],
    pub n: usize,
}

impl PowerBudget {
    /// B601-DM: J1-J3 DM-J4340P, J4-J7 DM-J4310.
    pub fn rebot_dm() -> Self {
        let mut joints = vec![JointPower::DM_J4340P; 3];
        joints.extend([JointPower::DM_J4310; 4]);
        Self { joints, ..Default::default() }
    }

    pub fn validate(&self, dof: usize) -> Result<(), String> {
        let pos = |v: f32| v.is_finite() && v > 0.0;
        let nonneg = |v: f32| v.is_finite() && v >= 0.0;
        if !pos(self.supply_rated_a) || !pos(self.supply_trip_a) || self.supply_trip_a < self.supply_rated_a {
            return Err("power_budget: supply_rated_a must be positive and supply_trip_a at least supply_rated_a".into());
        }
        if !(5.0..=60.0).contains(&self.v_bus) {
            return Err("power_budget.v_bus must be in [5, 60] V".into());
        }
        if !nonneg(self.base_w) || !(0.5..=3.0).contains(&self.k_cu) || !(self.efficiency > 0.3 && self.efficiency <= 1.0) {
            return Err("power_budget: base_w must be >= 0, k_cu in [0.5, 3], efficiency in (0.3, 1]".into());
        }
        if !(0.0..=5.0).contains(&self.filter_s) {
            return Err("power_budget.filter_s must be in [0, 5] s".into());
        }
        if self.joints.len() != dof || dof > MAX_JOINTS {
            return Err(format!("power_budget.joints needs {dof} entries (one per joint, at most {MAX_JOINTS})"));
        }
        for (j, p) in self.joints.iter().enumerate() {
            if !pos(p.kt_nm_per_a) || !nonneg(p.r_phase_ohm) || !nonneg(p.idle_w) || !nonneg(p.viscous_nm_s) || !nonneg(p.coulomb_nm) {
                return Err(format!("power_budget.joints[{j}]: kt_nm_per_a must be positive; r_phase_ohm, idle_w, viscous_nm_s, coulomb_nm >= 0"));
            }
        }
        Ok(())
    }

    /// Supply power drawn by joint `j` (W) and mechanical power it feeds back (W).
    #[inline]
    pub fn joint_power_w(&self, j: usize, tau: f32, omega: f32) -> (f32, f32) {
        let p = &self.joints[j];
        let (tau, omega) = (if tau.is_finite() { tau } else { 0.0 }, if omega.is_finite() { omega } else { 0.0 });
        let mech = tau * omega;
        let i = tau / p.kt_nm_per_a;
        let draw = mech.max(0.0) / self.efficiency + self.k_cu * i * i * p.r_phase_ohm + p.idle_w;
        (draw, (-mech).max(0.0) * self.efficiency)
    }

    /// Estimate from per-joint torques and velocities (missing entries count as 0).
    pub fn estimate(&self, torques: &[f32], velocities: &[f32]) -> Estimate {
        let n = self.joints.len().min(MAX_JOINTS);
        let mut joint_a = [0.0f32; MAX_JOINTS];
        let (mut total, mut regen) = (self.base_w, 0.0f32);
        for j in 0..n {
            let (draw, back) = self.joint_power_w(j, torques.get(j).copied().unwrap_or(0.0), velocities.get(j).copied().unwrap_or(0.0));
            joint_a[j] = draw / self.v_bus;
            total += draw;
            regen += back;
        }
        Estimate { bus_a: total / self.v_bus, regen_w: regen, joint_a, n }
    }

    /// Supply current with every joint at rest and torque-free (A): idle and base load.
    pub fn idle_a(&self) -> f32 {
        (self.base_w + self.joints.iter().map(|p| p.idle_w).sum::<f32>()) / self.v_bus
    }
}

/// Maximum over a sliding window kept in `N` buckets of `bucket_s` (no allocation).
#[derive(Debug, Clone, Copy)]
pub struct PeakWindow<const N: usize> {
    buckets: [f32; N],
    bucket_s: f64,
    head: usize,
    head_start: f64,
    started: bool,
}

impl<const N: usize> PeakWindow<N> {
    pub const fn new(bucket_s: f64) -> Self { Self { buckets: [0.0; N], bucket_s, head: 0, head_start: 0.0, started: false } }

    pub fn push(&mut self, t: f64, v: f32) {
        if !self.started {
            self.started = true;
            self.head_start = t;
        }
        let k = ((t - self.head_start) / self.bucket_s).floor();
        if k >= 1.0 {
            for _ in 0..(k as usize).min(N) {
                self.head = (self.head + 1) % N;
                self.buckets[self.head] = 0.0;
            }
            self.head_start += k * self.bucket_s;
        }
        if v.is_finite() && v > self.buckets[self.head] { self.buckets[self.head] = v; }
    }

    /// Maximum over the last (N - 1) to N buckets.
    pub fn max(&self) -> f32 { self.buckets.iter().copied().fold(0.0, f32::max) }
}

/// Runtime estimate kept in the supervisor.
#[derive(Debug, Clone)]
pub struct PowerMonitor {
    /// An estimate from fresh, valid torque feedback exists this tick.
    pub valid: bool,
    /// Unfiltered value of this tick (A).
    pub raw_a: f32,
    /// Fast filter (A), the governor input.
    pub fast_a: f32,
    /// Display filter (A).
    pub bus_a: f32,
    pub regen_w: f32,
    pub joint_a: [f32; MAX_JOINTS],
    pub peak_5s: PeakWindow<10>,
    pub peak_60s: PeakWindow<60>,
    /// Highest `fast_a` since start and its observation-clock time.
    pub max_a: f32,
    pub max_at_s: f64,
    /// Time with `fast_a` above the supply rating (s).
    pub over_rated_s: f64,
    primed: bool,
}

impl Default for PowerMonitor {
    fn default() -> Self {
        Self {
            valid: false, raw_a: 0.0, fast_a: 0.0, bus_a: 0.0, regen_w: 0.0, joint_a: [0.0; MAX_JOINTS],
            peak_5s: PeakWindow::new(PEAK_WINDOW_S / 10.0), peak_60s: PeakWindow::new(1.0),
            max_a: 0.0, max_at_s: 0.0, over_rated_s: 0.0, primed: false,
        }
    }
}

impl PowerMonitor {
    /// Feed one estimate taken `dt` s after the previous one at observation time `now_s`.
    pub fn update(&mut self, budget: &PowerBudget, e: &Estimate, dt: f32, now_s: f64) {
        let lp = |y: f32, x: f32, tc: f32| if tc <= 0.0 { x } else { y + (x - y) * (dt / (tc + dt)) };
        if self.primed {
            self.fast_a = lp(self.fast_a, e.bus_a, FAST_FILTER_S);
            self.bus_a = lp(self.bus_a, e.bus_a, budget.filter_s);
            self.regen_w = lp(self.regen_w, e.regen_w, budget.filter_s);
            for j in 0..e.n { self.joint_a[j] = lp(self.joint_a[j], e.joint_a[j], budget.filter_s); }
        } else {
            self.primed = true;
            (self.fast_a, self.bus_a, self.regen_w, self.joint_a) = (e.bus_a, e.bus_a, e.regen_w, e.joint_a);
        }
        self.valid = true;
        self.raw_a = e.bus_a;
        self.peak_5s.push(now_s, self.fast_a);
        self.peak_60s.push(now_s, self.fast_a);
        if self.fast_a > self.max_a { self.max_a = self.fast_a; self.max_at_s = now_s; }
        if self.fast_a > budget.supply_rated_a { self.over_rated_s += f64::from(dt); }
    }

    /// No estimate this tick (stale or invalid feedback): the filters restart from the next one.
    pub fn invalidate(&mut self) {
        self.valid = false;
        self.primed = false;
    }
}

fn r3(v: f32) -> f32 { (v * 1000.0).round() / 1000.0 }

/// `observation.power`: null when the profile has no enabled `power_budget`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PowerObservation {
    /// An estimate from fresh, valid torque feedback exists; the numbers below are 0 otherwise.
    pub valid: bool,
    /// Estimated supply current (A), filtered (`filter_s`).
    pub bus_a: f32,
    /// Highest fast-filtered estimate over the last ~5 s (A).
    pub peak_a: f32,
    /// Supply rating (A).
    pub rated_a: f32,
    /// `bus_a / rated_a`.
    pub ratio: f32,
    /// Mechanical power fed back by the motors (W); not credited, raises the bus voltage.
    pub regen_w: f32,
    /// Per joint share of `bus_a` (A).
    pub per_joint_a: Vec<f32>,
    /// Speed scale applied to motions this tick by the governor (1 = none).
    pub scale: f32,
    /// The governor is slowing or holding motion.
    pub limiting: bool,
    /// A power hold is in force (`motion_limit.reason` = `power`).
    pub hold: bool,
}

impl SupervisorInner {
    /// The enabled power budget of the profile.
    pub fn power_budget(&self) -> Option<&PowerBudget> { self.config.power_budget.as_ref().filter(|p| p.enabled) }

    /// One control tick, after this tick's feedback. `dt` is the control-loop step (s).
    pub(crate) fn power_tick(&mut self, dt: f64) {
        let Some(budget) = self.config.power_budget.as_ref().filter(|p| p.enabled) else { return; };
        let fresh = self.driver.is_some() && self.collision.torque_feedback && self.feedback_numeric_valid
            && self.hardware_feedback_fresh(std::time::Duration::from_millis(100));
        if !fresh {
            self.power.invalidate();
            return;
        }
        let e = budget.estimate(&self.torques, &self.measured_velocities);
        let now = self.start_time.elapsed().as_secs_f64();
        self.power.update(budget, &e, dt as f32, now);
    }

    /// `observation.power`.
    pub fn power_observation(&self) -> Option<PowerObservation> {
        let budget = self.power_budget()?;
        let m = &self.power;
        let n = budget.joints.len().min(MAX_JOINTS);
        let v = |x: f32| if m.valid { r3(x) } else { 0.0 };
        Some(PowerObservation {
            valid: m.valid,
            bus_a: v(m.bus_a),
            peak_a: v(m.peak_5s.max()),
            rated_a: budget.supply_rated_a,
            ratio: v(m.bus_a / budget.supply_rated_a),
            regen_w: v(m.regen_w),
            per_joint_a: m.joint_a[..n].iter().map(|a| v(*a)).collect(),
            scale: 1.0,
            limiting: false,
            hold: false,
        })
    }

    /// `GET /api/v1/diagnostics/power`.
    pub fn power_diagnostics(&self) -> serde_json::Value {
        let Some(budget) = self.power_budget() else {
            return serde_json::json!({"enabled": false});
        };
        let m = &self.power;
        serde_json::json!({
            "enabled": true,
            "estimate_only": true,
            "observation": self.power_observation(),
            "fast_a": r3(m.fast_a),
            "raw_a": r3(m.raw_a),
            "peak_5s_a": r3(m.peak_5s.max()),
            "peak_60s_a": r3(m.peak_60s.max()),
            "max_a": r3(m.max_a),
            "max_at_s": m.max_at_s,
            "over_rated_s": m.over_rated_s,
            "idle_a": r3(budget.idle_a()),
            "budget": budget,
        })
    }
}
