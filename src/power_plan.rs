//! Power-budget time scaling of planned motions (`power_budget.planning`).
//!
//! Before a planned motion runs (joint moves / jog, home, park, move_to_pose, Cartesian
//! actions and their grasp / place sub-motions, replay) its trajectory is sampled and, per
//! sample, the joint torques are computed with the rigid-body model (`src/dynamics.rs`):
//!
//! ```text
//! tau_k(s) = s² · tau_dyn,k + g(q_k) + viscous · s · q'_k + coulomb · sign(q'_k)
//! omega_k(s) = s · q'_k
//! ```
//!
//! for the motion slowed uniformly by the time scale s (duration / s; tau_dyn = M q'' + C q'
//! at s = 1). The supply current per sample is the power model of `src/power.rs`. The plan
//! keeps s = 1 when the peak is at or below the limit L, otherwise the largest s in
//! [`min_scale`, 1] whose peak fits is found by bisection (the lower end is always one that
//! fits, so the result fits). L = `fraction` x `supply_rated_a`, or the static load at the
//! start pose plus [`START_MARGIN`] x `supply_rated_a` when that is higher (a move from a pose
//! that already exceeds the budget is refused only if it adds load). If even `min_scale` does not fit the motion is refused with
//! `POWER_BUDGET`; park is never refused (it gets `min_scale` then, and the governor still
//! applies).
//!
//! Runs where the plan is made: off the supervisor lock for Cartesian paths and replay
//! (blocking planner thread), under the API command's lock for joint quintics (like their
//! workspace check), never in the control tick.
use serde::{Deserialize, Serialize};

use crate::chain::{ArmModel, MAX_CHAIN_JOINTS};
use crate::motion::MotionError;
use crate::power::{PowerBudget, MAX_JOINTS};

/// When the static load at the start pose is above the planning limit, the limit becomes that
/// load plus this fraction of the supply rating (moves that do not add load are not refused).
pub const START_MARGIN: f32 = 0.05;

/// Section `power_budget.planning`.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct PlanningPolicy {
    pub enabled: bool,
    /// Planning limit as a fraction of `supply_rated_a` (at most `governor.soft`, so that a
    /// plan that fits does not make the governor slow it).
    pub fraction: f32,
    /// Slowest time scale a plan may get (duration up to 1 / min_scale).
    pub min_scale: f64,
    /// Sample spacing (s at full speed) and the most samples per motion.
    pub sample_dt_s: f64,
    pub max_samples: usize,
}

impl Default for PlanningPolicy {
    fn default() -> Self { Self { enabled: true, fraction: 0.75, min_scale: 0.2, sample_dt_s: 0.02, max_samples: 400 } }
}

impl PlanningPolicy {
    pub fn validate(&self, soft: f32) -> Result<(), String> {
        if !(self.fraction >= 0.1 && self.fraction <= soft) {
            return Err(format!("power_budget.planning.fraction must be in [0.1, governor.soft = {soft}]"));
        }
        if !(0.05..=1.0).contains(&self.min_scale) || !(0.005..=0.2).contains(&self.sample_dt_s) || !(10..=5000).contains(&self.max_samples) {
            return Err("power_budget.planning: min_scale in [0.05, 1], sample_dt_s in [0.005, 0.2], max_samples in 10..=5000".into());
        }
        Ok(())
    }
}

/// Trajectory samples at full speed: position, velocity, acceleration of the arm joints.
#[derive(Debug, Clone, Default)]
pub struct Samples {
    pub t: Vec<f64>,
    pub q: Vec<[f64; MAX_CHAIN_JOINTS]>,
    pub qd: Vec<[f64; MAX_CHAIN_JOINTS]>,
    pub qdd: Vec<[f64; MAX_CHAIN_JOINTS]>,
}

impl Samples {
    /// Number of samples for a motion of `duration` s under `p`.
    pub fn count(p: &PlanningPolicy, duration: f64) -> usize {
        ((duration / p.sample_dt_s).ceil() as usize).clamp(2, p.max_samples)
    }

    /// Positions `pos(t)` at `n + 1` evenly spaced times over `[0, duration]`, derivatives by
    /// central differences (one-sided at the ends).
    pub fn from_positions(duration: f64, n: usize, dof: usize, pos: impl Fn(f64) -> [f64; MAX_CHAIN_JOINTS]) -> Self {
        let n = n.max(2);
        let h = duration / n as f64;
        let t: Vec<f64> = (0..=n).map(|k| k as f64 * h).collect();
        let q: Vec<_> = t.iter().map(|&x| pos(x)).collect();
        let mut qd = vec![[0.0; MAX_CHAIN_JOINTS]; n + 1];
        let mut qdd = vec![[0.0; MAX_CHAIN_JOINTS]; n + 1];
        for k in 0..=n {
            let (a, b) = (k.saturating_sub(1), (k + 1).min(n));
            for j in 0..dof {
                qd[k][j] = (q[b][j] - q[a][j]) / ((b - a) as f64 * h);
                let c = k.clamp(1, n - 1);
                qdd[k][j] = (q[c + 1][j] - 2.0 * q[c][j] + q[c - 1][j]) / (h * h);
            }
        }
        Self { t, q, qd, qdd }
    }
}

/// Result of [`plan_scale`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanScale {
    /// Time scale (1 = as planned; the duration becomes duration / scale).
    pub scale: f64,
    /// Estimated peak supply current at `scale` and at full speed (A).
    pub peak_a: f32,
    pub peak_full_a: f32,
    /// Limit the plan was fitted to (A).
    pub limit_a: f32,
    pub samples: usize,
}

/// Torques split for the time scaling (per sample, arm joints).
struct Pre {
    tau_d: Vec<[f32; MAX_JOINTS]>,
    tau_g: Vec<[f32; MAX_JOINTS]>,
    qd: Vec<[f32; MAX_JOINTS]>,
    arm: usize,
}

impl Pre {
    fn new(arm: &ArmModel, s: &Samples) -> Self {
        let n = arm.arm_dof().min(MAX_JOINTS);
        let zero = [0.0; MAX_CHAIN_JOINTS];
        let f = |v: [f64; MAX_CHAIN_JOINTS]| -> [f32; MAX_JOINTS] { std::array::from_fn(|j| if j < n { v[j] as f32 } else { 0.0 }) };
        let mut pre = Self { tau_d: Vec::with_capacity(s.q.len()), tau_g: Vec::with_capacity(s.q.len()), qd: Vec::with_capacity(s.q.len()), arm: n };
        for k in 0..s.q.len() {
            pre.tau_d.push(f(arm.inverse_dynamics(&s.q[k], &s.qd[k], &s.qdd[k], false)));
            pre.tau_g.push(f(arm.inverse_dynamics(&s.q[k], &zero, &zero, true)));
            pre.qd.push(f(s.qd[k]));
        }
        pre
    }

    /// Estimated supply current at sample `k` slowed by `s` (A).
    fn current(&self, b: &PowerBudget, k: usize, s: f32) -> f32 {
        let mut w = b.base_w;
        for (j, p) in b.joints.iter().enumerate().take(MAX_JOINTS) {
            let (tau, omega) = if j < self.arm {
                let qd = self.qd[k][j];
                let sign = if qd.abs() > 1e-6 { qd.signum() } else { 0.0 };
                (s * s * self.tau_d[k][j] + self.tau_g[k][j] + p.viscous_nm_s * s * qd + p.coulomb_nm * sign, s * qd)
            } else { (0.0, 0.0) };
            w += b.joint_power_w(j, tau, omega).0;
        }
        w / b.v_bus
    }

    /// (peak current, sample) at scale `s`.
    fn peak(&self, b: &PowerBudget, s: f32) -> (f32, usize) {
        (0..self.tau_d.len()).map(|k| (self.current(b, k, s), k)).fold((0.0, 0), |m, x| if x.0 > m.0 { x } else { m })
    }
}

/// Fit the samples (full speed) of a motion into the planning budget. `Ok` with `scale` 1 when
/// it fits as planned; `Err(POWER_BUDGET)` when even `min_scale` does not fit.
pub fn plan_scale(b: &PowerBudget, arm: &ArmModel, samples: &Samples) -> Result<PlanScale, MotionError> {
    let p = b.planning;
    let pre = Pre::new(arm, samples);
    let static0 = pre.current(b, 0, 0.0);
    let limit = (p.fraction * b.supply_rated_a).max(static0 + START_MARGIN * b.supply_rated_a);
    let (full, _) = pre.peak(b, 1.0);
    let n = samples.q.len();
    if full <= limit {
        return Ok(PlanScale { scale: 1.0, peak_a: full, peak_full_a: full, limit_a: limit, samples: n });
    }
    let (at_min, k) = pre.peak(b, p.min_scale as f32);
    if at_min > limit {
        let stat = pre.current(b, k, 0.0);
        return Err(MotionError::PowerBudget(format!(
            "the motion needs an estimated {at_min:.2} A of supply current even slowed to {:.0}% speed (t = {:.2} s of {:.2} s at full speed; static holding load there {stat:.2} A), above the planning limit {limit:.2} A ({:.0}% of the {} A supply rating); estimate from the rigid-body model, not measured",
            p.min_scale * 100.0, samples.t[k], samples.t.last().copied().unwrap_or(0.0), p.fraction * 100.0, b.supply_rated_a)));
    }
    let (mut lo, mut hi) = (p.min_scale, 1.0f64);
    for _ in 0..30 {
        let mid = 0.5 * (lo + hi);
        if pre.peak(b, mid as f32).0 <= limit { lo = mid; } else { hi = mid; }
        if hi - lo < 1e-3 { break; }
    }
    let (peak, _) = pre.peak(b, lo as f32);
    Ok(PlanScale { scale: lo, peak_a: peak, peak_full_a: full, limit_a: limit, samples: n })
}

/// Samples of a joint quintic (analytic velocities and accelerations).
pub fn quintic_samples(p: &PlanningPolicy, traj: &crate::trajectory::MultiJointQuinticTrajectory, dof: usize) -> Samples {
    let n = Samples::count(p, traj.duration);
    let mut s = Samples::default();
    for k in 0..=n {
        let t = traj.duration * k as f64 / n as f64;
        let pt = traj.sample(t);
        let f = |v: &[f32]| -> [f64; MAX_CHAIN_JOINTS] { std::array::from_fn(|j| if j < dof.min(v.len()) { f64::from(v[j]) } else { 0.0 }) };
        s.t.push(t);
        s.q.push(f(&pt.positions));
        s.qd.push(f(&pt.velocities));
        s.qdd.push(f(&pt.accelerations));
    }
    s
}

/// Zero-allocation equivalent of [`plan_scale`] on [`quintic_samples`] for a joint quintic
/// trajectory: computes identical inverse-dynamics currents directly from the polynomial
/// coefficients on the stack (no `Samples` or `Pre` `Vec` allocations).
pub fn plan_scale_quintic(
    b: &PowerBudget,
    arm: &ArmModel,
    dof: usize,
    traj: &crate::trajectory::MultiJointQuinticTrajectory,
) -> Result<PlanScale, MotionError> {
    let p = b.planning;
    let n = Samples::count(&p, traj.duration);
    let arm_n = arm.arm_dof().min(MAX_JOINTS);
    let sample_dof = dof.min(traj.polynomials.len()).min(MAX_CHAIN_JOINTS);
    let zero = [0.0; MAX_CHAIN_JOINTS];

    let sample_torques = |k: usize| -> ([f32; MAX_JOINTS], [f32; MAX_JOINTS], [f32; MAX_JOINTS]) {
        let t = traj.duration * k as f64 / n as f64;
        let mut q = [0.0f64; MAX_CHAIN_JOINTS];
        let mut qd = [0.0f64; MAX_CHAIN_JOINTS];
        let mut qdd = [0.0f64; MAX_CHAIN_JOINTS];
        for (j, poly) in traj.polynomials.iter().enumerate().take(sample_dof) {
            q[j] = f64::from(poly.position(t) as f32);
            qd[j] = f64::from(poly.velocity(t) as f32);
            qdd[j] = f64::from(poly.acceleration(t) as f32);
        }
        let to_f32 = |v: [f64; MAX_CHAIN_JOINTS]| -> [f32; MAX_JOINTS] {
            std::array::from_fn(|j| if j < arm_n { v[j] as f32 } else { 0.0 })
        };
        let tau_d = to_f32(arm.inverse_dynamics(&q, &qd, &qdd, false));
        let tau_g = to_f32(arm.inverse_dynamics(&q, &zero, &zero, true));
        let qd32 = to_f32(qd);
        (tau_d, tau_g, qd32)
    };

    let current_at = |tau_d: &[f32; MAX_JOINTS], tau_g: &[f32; MAX_JOINTS], qd: &[f32; MAX_JOINTS], s: f32| -> f32 {
        let mut w = b.base_w;
        for (j, jp) in b.joints.iter().enumerate().take(MAX_JOINTS) {
            let (tau, omega) = if j < arm_n {
                let qdj = qd[j];
                let sign = if qdj.abs() > 1e-6 { qdj.signum() } else { 0.0 };
                (s * s * tau_d[j] + tau_g[j] + jp.viscous_nm_s * s * qdj + jp.coulomb_nm * sign, s * qdj)
            } else { (0.0, 0.0) };
            w += b.joint_power_w(j, tau, omega).0;
        }
        w / b.v_bus
    };

    let mut static0 = 0.0f32;
    let mut full = 0.0f32;
    let mut at_min = 0.0f32;
    let mut at_min_k = 0usize;
    let mut at_min_stat = 0.0f32;
    let min_s = p.min_scale as f32;
    for k in 0..=n {
        let (tau_d, tau_g, qd) = sample_torques(k);
        if k == 0 { static0 = current_at(&tau_d, &tau_g, &qd, 0.0); }
        let c_full = current_at(&tau_d, &tau_g, &qd, 1.0);
        if c_full > full { full = c_full; }
        let c_min = current_at(&tau_d, &tau_g, &qd, min_s);
        if c_min > at_min {
            at_min = c_min;
            at_min_k = k;
            at_min_stat = current_at(&tau_d, &tau_g, &qd, 0.0);
        }
    }
    let limit = (p.fraction * b.supply_rated_a).max(static0 + START_MARGIN * b.supply_rated_a);
    let total = n + 1;
    if full <= limit {
        return Ok(PlanScale { scale: 1.0, peak_a: full, peak_full_a: full, limit_a: limit, samples: total });
    }
    if at_min > limit {
        let tk = traj.duration * at_min_k as f64 / n as f64;
        return Err(MotionError::PowerBudget(format!(
            "the motion needs an estimated {at_min:.2} A of supply current even slowed to {:.0}% speed (t = {:.2} s of {:.2} s at full speed; static holding load there {at_min_stat:.2} A), above the planning limit {limit:.2} A ({:.0}% of the {} A supply rating); estimate from the rigid-body model, not measured",
            p.min_scale * 100.0, tk, traj.duration, p.fraction * 100.0, b.supply_rated_a)));
    }
    let peak_for = |s: f32| -> f32 {
        let mut mx = 0.0f32;
        for k in 0..=n {
            let (tau_d, tau_g, qd) = sample_torques(k);
            let c = current_at(&tau_d, &tau_g, &qd, s);
            if c > mx { mx = c; }
        }
        mx
    };
    let (mut lo, mut hi) = (p.min_scale, 1.0f64);
    for _ in 0..30 {
        let mid = 0.5 * (lo + hi);
        if peak_for(mid as f32) <= limit { lo = mid; } else { hi = mid; }
        if hi - lo < 1e-3 { break; }
    }
    let peak = peak_for(lo as f32);
    Ok(PlanScale { scale: lo, peak_a: peak, peak_full_a: full, limit_a: limit, samples: total })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ArmConfig;
    use crate::trajectory::TrajectoryPlanner;

    #[test]
    fn plan_scale_quintic_matches_reference_plan_scale() {
        let cfg: ArmConfig = serde_json::from_reader(
            std::fs::File::open(concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/profile/rebot-b601dm.json")).unwrap(),
        )
        .unwrap();
        let mut budget = cfg.power_budget.clone().expect("device profile has power budget");
        let dof = cfg.dof;
        let arm_dof = cfg.arm_dof();
        let start = cfg.home_positions.clone();
        let mut target = start.clone();
        target[1] += 0.5;
        target[2] -= 0.4;
        let zeros = vec![0.0; dof];
        let traj = TrajectoryPlanner::plan(&start, &zeros, &target, &zeros, Some(0.5), &cfg.limits).unwrap();

        // 1. Fits at full speed (scale == 1.0)
        let r_ref = plan_scale(&budget, &cfg.arm_model, &quintic_samples(&budget.planning, &traj, arm_dof)).unwrap();
        let r_inline = plan_scale_quintic(&budget, &cfg.arm_model, arm_dof, &traj).unwrap();
        assert_eq!(r_inline, r_ref);

        // 2. Needs bisection slowdown (tighter supply rating)
        budget.supply_rated_a = 1.5;
        let r_ref2 = plan_scale(&budget, &cfg.arm_model, &quintic_samples(&budget.planning, &traj, arm_dof));
        let r_inline2 = plan_scale_quintic(&budget, &cfg.arm_model, arm_dof, &traj);
        match (r_ref2, r_inline2) {
            (Ok(a), Ok(b)) => assert_eq!(a, b),
            (Err(MotionError::PowerBudget(a)), Err(MotionError::PowerBudget(b))) => assert_eq!(a, b),
            (a, b) => panic!("mismatch: ref={a:?} inline={b:?}"),
        }
    }
}
