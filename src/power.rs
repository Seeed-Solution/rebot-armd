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
//!
//! Governor (`governor`), on every motion in position control (servo streams, jog, actions,
//! replay, park). With r = fast_a / supply_rated_a:
//!
//! - speed scale target: 1 for r <= soft, falling linearly to `floor_scale` at r = hard; the
//!   applied scale follows the target at most `down_per_s` / `up_per_s` per second.
//!   [`SupervisorInner::motion_time_scale`] is the one value every reference reads: planned
//!   references (joint quintics, Cartesian paths, replay) advance their time at dt x scale;
//!   servo streams scale the velocity cap (never below what the joint can brake to in one
//!   tick, so the stepper stays feasible) and keep the acceleration limit.
//! - hold: r >= hard for `persist_ms` while a motion is running and a heavy joint (at least
//!   [`HEAVY_SHARE`] of the joint load above idle) is commanded in the direction of its torque
//!   (pushing): the running action fails with `POWER_LIMIT`, every motion ends and the measured
//!   pose is held with torque on (`motion_limit.reason` = `power`). While held, servo and jog
//!   targets that move a joint that was pushing further in its push direction are clamped to
//!   the hold position; targets that keep it or move it the other way (yielding, which lowers
//!   its torque) pass. Holding still above hard does not trip (stopping cannot lower a static
//!   load); it is logged once.
//! - recovery: the hold ends once `bus_a` stays below soft for `recover_ms`, or on torque
//!   off, stop, Fault or gravity compensation.
//! - gravity compensation: estimated and published, never scaled or held; r >= hard is logged
//!   once per session.

use serde::{Deserialize, Serialize};

use crate::model::MotionLimitReason;
use crate::supervisor::SupervisorInner;

/// Most joints the per-tick arrays hold.
pub const MAX_JOINTS: usize = 8;
/// Time constant (s) of the fast filter (the governor input). At 50 Hz this is 2 ticks.
pub const FAST_FILTER_S: f32 = 0.04;
/// A joint counts as heavy for the hold rule at this share of the joints' load above idle.
pub const HEAVY_SHARE: f32 = 0.25;

/// Section `power_budget.governor`.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct GovernorPolicy {
    pub enabled: bool,
    /// Fraction of `supply_rated_a` where slowing starts.
    pub soft: f32,
    /// Fraction of `supply_rated_a` where the scale reaches `floor_scale` and the hold applies;
    /// `hard x supply_rated_a` must stay below `supply_trip_a`.
    pub hard: f32,
    /// Lowest speed scale before the hold.
    pub floor_scale: f32,
    /// Time above hard (control-loop time) before the hold (ms).
    pub persist_ms: u64,
    /// Time below soft that ends a hold (ms).
    pub recover_ms: u64,
    /// Largest scale change per second, down / up.
    pub down_per_s: f32,
    pub up_per_s: f32,
}

impl Default for GovernorPolicy {
    fn default() -> Self {
        Self { enabled: true, soft: 0.8, hard: 0.95, floor_scale: 0.25, persist_ms: 100, recover_ms: 300, down_per_s: 5.0, up_per_s: 0.5 }
    }
}

impl GovernorPolicy {
    /// Scale target for the load ratio `r` (estimate / rating).
    pub fn target_scale(&self, r: f32) -> f32 {
        if !r.is_finite() { return self.floor_scale; }
        if r <= self.soft { return 1.0; }
        if r >= self.hard { return self.floor_scale; }
        1.0 - (r - self.soft) / (self.hard - self.soft) * (1.0 - self.floor_scale)
    }

    pub fn validate(&self, budget: &PowerBudget) -> Result<(), String> {
        if !(self.soft > 0.2 && self.soft < self.hard && self.hard <= 1.2) {
            return Err("power_budget.governor: need 0.2 < soft < hard <= 1.2".into());
        }
        if self.hard * budget.supply_rated_a >= budget.supply_trip_a {
            return Err(format!("power_budget.governor.hard x supply_rated_a ({:.2} A) must be below supply_trip_a ({} A)",
                self.hard * budget.supply_rated_a, budget.supply_trip_a));
        }
        if !(0.05..=1.0).contains(&self.floor_scale) || self.persist_ms == 0 || self.persist_ms > 2000 || self.recover_ms > 10_000 {
            return Err("power_budget.governor: floor_scale in [0.05, 1], persist_ms in 1..=2000, recover_ms <= 10000".into());
        }
        if !(self.down_per_s > 0.0 && self.down_per_s <= 100.0 && self.up_per_s > 0.0 && self.up_per_s <= 100.0) {
            return Err("power_budget.governor: down_per_s and up_per_s in (0, 100]".into());
        }
        Ok(())
    }
}

/// A power hold in force.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerHold {
    /// Per joint: sign of the torque of a joint that was pushing at the trip (+1 / -1), 0 = free.
    pub push: [f32; MAX_JOINTS],
    /// Commanded (held) position per joint at the trip.
    pub hold_q: [f32; MAX_JOINTS],
    /// Estimate at the trip (A).
    pub bus_a: f32,
    pub since_s: f64,
}

/// Governor runtime state.
#[derive(Debug, Clone)]
pub struct Governor {
    /// Scale applied this tick.
    pub scale: f32,
    pub hold: Option<PowerHold>,
    over_s: f64,
    below_s: f64,
    /// Scaling is logged (a transition is logged once).
    limiting: bool,
    static_warned: bool,
    gc_warned: bool,
    pub holds_total: u64,
    /// Time with scale below 1 (s).
    pub limited_s: f64,
}

impl Default for Governor {
    fn default() -> Self {
        Self { scale: 1.0, hold: None, over_s: 0.0, below_s: 0.0, limiting: false, static_warned: false, gc_warned: false, holds_total: 0, limited_s: 0.0 }
    }
}

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
    pub governor: GovernorPolicy,
    /// Time scaling of planned motions (`src/power_plan.rs`).
    pub planning: crate::power_plan::PlanningPolicy,
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
            governor: GovernorPolicy::default(),
            planning: crate::power_plan::PlanningPolicy::default(),
        }
    }
}

/// The scalar part of the budget the governor needs, copied each tick without allocation
/// (the supervisor is borrowed mutably while it runs).
#[derive(Debug, Clone, Copy)]
pub(crate) struct PolicyView {
    pub supply_rated_a: f32,
    pub v_bus: f32,
    pub idle_w: [f32; MAX_JOINTS],
    pub governor: GovernorPolicy,
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
        self.governor.validate(self)?;
        self.planning.validate(self.governor.soft)?;
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

    pub(crate) fn clone_policy(&self) -> PolicyView {
        let mut idle_w = [0.0; MAX_JOINTS];
        for (o, p) in idle_w.iter_mut().zip(&self.joints) { *o = p.idle_w; }
        PolicyView { supply_rated_a: self.supply_rated_a, v_bus: self.v_bus, idle_w, governor: self.governor }
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
    pub gov: Governor,
    primed: bool,
}

impl Default for PowerMonitor {
    fn default() -> Self {
        Self {
            valid: false, raw_a: 0.0, fast_a: 0.0, bus_a: 0.0, regen_w: 0.0, joint_a: [0.0; MAX_JOINTS],
            peak_5s: PeakWindow::new(PEAK_WINDOW_S / 10.0), peak_60s: PeakWindow::new(1.0),
            max_a: 0.0, max_at_s: 0.0, over_rated_s: 0.0, gov: Governor::default(), primed: false,
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
        let budget = budget.clone_policy();
        self.governor_tick(&budget, &e, dt);
    }

    /// Speed scale every motion reference applies this tick (1 = unchanged). Gravity
    /// compensation is never scaled.
    pub fn motion_time_scale(&self) -> f64 {
        if self.gravity_comp.is_some() { 1.0 } else { f64::from(self.power.gov.scale) }
    }

    fn governor_tick(&mut self, b: &PolicyView, e: &Estimate, dt: f64) {
        let g = b.governor;
        if !g.enabled { self.power.gov.scale = 1.0; return; }
        let rated = b.supply_rated_a;
        let r = self.power.fast_a / rated;
        let operational = self.is_torqued && self.safety.state().is_operational() && !self.disable_pending && !self.pending_rearm;
        let gc = self.gravity_comp.is_some();
        if gc {
            self.power.gov.scale = 1.0;
            self.power.gov.over_s = 0.0;
            if r >= g.hard && !self.power.gov.gc_warned {
                tracing::warn!("Power estimate {:.2} A ({:.0}% of {rated} A rated) during gravity compensation; not limited (hand guiding)", self.power.fast_a, r * 100.0);
                self.power.gov.gc_warned = true;
            }
        } else {
            self.power.gov.gc_warned = false;
        }
        // hold end
        if self.power.gov.hold.is_some() {
            if !operational || gc {
                tracing::info!("Power hold ended (stop, torque off or gravity compensation)");
                self.end_power_hold();
            } else if self.power.bus_a < g.soft * rated {
                self.power.gov.below_s += dt;
                if self.power.gov.below_s >= g.recover_ms as f64 / 1000.0 - 1e-6 {
                    tracing::info!("Power hold ended: estimate {:.2} A below {:.0}% of rated for {} ms", self.power.bus_a, g.soft * 100.0, g.recover_ms);
                    self.end_power_hold();
                }
            } else {
                self.power.gov.below_s = 0.0;
            }
        }
        if gc { return; }
        // scale, rate limited
        let target = g.target_scale(r);
        let s = self.power.gov.scale;
        let dtf = dt as f32;
        let next = if target < s { (s - g.down_per_s * dtf).max(target) } else { (s + g.up_per_s * dtf).min(target) };
        self.power.gov.scale = next.clamp(g.floor_scale, 1.0);
        let moving = self.active_trajectory.is_some() || self.servo_target.is_some();
        let limiting = self.power.gov.scale < 0.999;
        if limiting { self.power.gov.limited_s += dt; }
        if limiting != self.power.gov.limiting {
            if limiting {
                tracing::warn!("Power governor: estimate {:.2} A ({:.0}% of {rated} A rated), slowing motion (scale {:.2})", self.power.fast_a, r * 100.0, self.power.gov.scale);
            } else {
                tracing::info!("Power governor: estimate {:.2} A, full speed again", self.power.fast_a);
            }
            self.power.gov.limiting = limiting;
        }
        if limiting && moving { self.note_motion_limit(MotionLimitReason::Power, None); }
        // hold rule: above hard while a heavy joint pushes
        let n = e.n.min(self.commanded_velocities.len()).min(self.torques.len());
        let idle_a = |j: usize| b.idle_w[j] / b.v_bus;
        let dyn_sum: f32 = (0..n).map(|j| (e.joint_a[j] - idle_a(j)).max(0.0)).sum();
        let mut push = [0.0f32; MAX_JOINTS];
        let mut worst: Option<(usize, f32)> = None;
        for j in 0..n {
            let load = (e.joint_a[j] - idle_a(j)).max(0.0);
            let (tau, v) = (self.torques[j], self.commanded_velocities[j]);
            if dyn_sum > 0.0 && load >= HEAVY_SHARE * dyn_sum && tau * v > 0.0 && v.abs() > 1e-3 {
                push[j] = tau.signum();
                if worst.is_none_or(|(_, w)| load > w) { worst = Some((j, load)); }
            }
        }
        let over = operational && r >= g.hard;
        if over && moving && worst.is_some() {
            self.power.gov.over_s += dt;
        } else {
            self.power.gov.over_s = 0.0;
        }
        if over && worst.is_none() && !self.power.gov.static_warned && self.power.gov.hold.is_none() {
            tracing::warn!("Power estimate {:.2} A ({:.0}% of {rated} A rated) with no joint pushing; not holding (stopping does not lower a static load)",
                self.power.fast_a, r * 100.0);
            self.power.gov.static_warned = true;
        } else if r < g.soft {
            self.power.gov.static_warned = false;
        }
        if self.power.gov.over_s >= g.persist_ms as f64 / 1000.0 - 1e-6 {
            if let Some((j, _)) = worst { self.power_trip(j, push, rated, g.persist_ms); }
        }
    }

    fn end_power_hold(&mut self) {
        self.power.gov.hold = None;
        self.power.gov.below_s = 0.0;
        if self.motion_limit.as_ref().is_some_and(|l| l.reason == MotionLimitReason::Power) { self.motion_limit = None; }
    }

    /// Trip: fail the running action with `POWER_LIMIT`, end every motion and hold the
    /// measured pose with torque on (gripper keeps its command).
    fn power_trip(&mut self, joint: usize, push: [f32; MAX_JOINTS], rated: f32, persist_ms: u64) {
        let gov = &mut self.power.gov;
        gov.over_s = 0.0;
        gov.holds_total += 1;
        let a = self.power.fast_a;
        let message = format!("Power limit: estimated supply current {a:.2} A ({:.0}% of {rated} A rated) for {persist_ms} ms, joint {} pushing",
            a / rated * 100.0, joint + 1);
        if self.power.gov.hold.is_some() {
            tracing::debug!("{message}; holding the new measured position");
        } else {
            tracing::warn!("{message}; motion stopped, holding position");
        }
        if let Some(id) = self.last_action.as_ref().filter(|a| a.is_running()).map(|a| a.id) {
            self.finish_action(id, "failed", Some("POWER_LIMIT"), Some(message), None, None, None);
        }
        self.macro_run = None;
        self.hold_measured_pose();
        self.note_motion_limit(MotionLimitReason::Power, Some(joint));
        let mut hold_q = [0.0f32; MAX_JOINTS];
        for (o, q) in hold_q.iter_mut().zip(&self.commanded_positions) { *o = *q; }
        let mut merged = push;
        if let Some(h) = self.power.gov.hold { for j in 0..MAX_JOINTS { if merged[j] == 0.0 { merged[j] = h.push[j]; } } }
        self.power.gov.hold = Some(PowerHold { push: merged, hold_q, bus_a: a, since_s: self.start_time.elapsed().as_secs_f64() });
    }

    /// While a power hold is in force: a servo / jog target that moves a joint that was
    /// pushing further in its push direction is clamped to the hold position.
    pub(crate) fn power_hold_filter(&mut self, targets: &mut [f32]) {
        let Some(h) = self.power.gov.hold else { return; };
        let mut clamped = None;
        for (j, t) in targets.iter_mut().enumerate().take(MAX_JOINTS) {
            if h.push[j] != 0.0 && (*t - h.hold_q[j]) * h.push[j] > 0.0 {
                *t = h.hold_q[j];
                clamped.get_or_insert(j);
            }
        }
        if let Some(j) = clamped { self.note_motion_limit(MotionLimitReason::Power, Some(j)); }
    }

    /// Record the power-budget time scale of a planned motion on action `id` (the smallest
    /// of its sub-motions for a macro action).
    pub(crate) fn note_action_power_scale(&mut self, id: u64, scale: f64) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
            let s = r3(scale as f32);
            a.power_scale = Some(a.power_scale.map_or(s, |old| old.min(s)));
        }
    }

    /// Fit a joint quintic from `start` / `v0` to `target` into the power budget
    /// (`src/power_plan.rs`): returns the trajectory to run (re-planned longer when slowed)
    /// and its scale (None without a planning budget). `refuse`: return POWER_BUDGET when even
    /// `min_scale` does not fit; otherwise run at `min_scale` (park).
    pub(crate) fn power_fit_quintic(&self, traj: crate::trajectory::MultiJointQuinticTrajectory, start: &[f32], v0: &[f32], target: &[f32],
        limits: &[crate::model::JointLimit], refuse: bool) -> Result<(crate::trajectory::MultiJointQuinticTrajectory, Option<f64>), crate::supervisor::ArmError> {
        use crate::power_plan::{plan_scale, quintic_samples};
        let Some(b) = self.power_budget().filter(|b| b.planning.enabled) else { return Ok((traj, None)); };
        let t0 = std::time::Instant::now();
        let (p, arm, dof) = (b.planning, &self.config.arm_model, self.config.arm_dof());
        let fit = |t: &crate::trajectory::MultiJointQuinticTrajectory| plan_scale(b, arm, &quintic_samples(&p, t, dof));
        let mut s = match fit(&traj) {
            Ok(f) if f.scale >= 1.0 => return Ok((traj, Some(1.0))),
            Ok(f) => f.scale,
            Err(e) if refuse => return Err(e.into()),
            Err(_) => p.min_scale,
        };
        let full = traj.duration;
        let zeros = vec![0.0; target.len()];
        // from rest the longer quintic is an exact time scaling of this one: no re-check
        if v0.iter().all(|v| v.abs() < 1e-6) {
            let t2 = crate::trajectory::TrajectoryPlanner::plan(start, v0, target, &zeros, Some(full / s), limits)?;
            let scale = full / t2.duration;
            tracing::debug!("Power planning (joint move): scale {scale:.3}, {:.2} s -> {:.2} s, {} µs", full, t2.duration, t0.elapsed().as_micros());
            return Ok((t2, Some(scale)));
        }
        // with a start velocity the longer quintic is not an exact time scaling: re-check it
        let mut best = None;
        for _ in 0..6 {
            let t2 = crate::trajectory::TrajectoryPlanner::plan(start, v0, target, &zeros, Some(full / s), limits)?;
            let ok = fit(&t2).is_ok_and(|f| f.scale >= 0.999);
            best = Some(t2);
            if ok || s <= p.min_scale + 1e-9 { break; }
            s = (s * 0.9).max(p.min_scale);
        }
        let t2 = best.expect("at least one plan");
        let scale = full / t2.duration;
        tracing::debug!("Power planning (joint move): scale {scale:.3}, {:.2} s -> {:.2} s, {} µs", full, t2.duration, t0.elapsed().as_micros());
        Ok((t2, Some(scale)))
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
            scale: r3(m.gov.scale),
            limiting: m.gov.scale < 0.999 || m.gov.hold.is_some(),
            hold: m.gov.hold.is_some(),
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
            "governor": {
                "scale": r3(m.gov.scale),
                "limited_s": m.gov.limited_s,
                "holds_total": m.gov.holds_total,
                "hold": m.gov.hold.map(|h| serde_json::json!({"bus_a": r3(h.bus_a), "since_s": h.since_s,
                    "push": h.push[..budget.joints.len().min(MAX_JOINTS)].to_vec()})),
            },
            "idle_a": r3(budget.idle_a()),
            "budget": budget,
        })
    }
}
