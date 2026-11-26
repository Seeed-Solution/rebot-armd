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
        if !c0.is_finite()
            || !c1.is_finite()
            || !c2.is_finite()
            || !c3.is_finite()
            || !c4.is_finite()
            || !c5.is_finite()
        {
            return Err(TrajectoryError::InvalidValue);
        }

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

    /// Certify that the executed position curve `self.position(t)` stays within `[lo, hi]`
    /// for every `t in [0, self.duration]` (zero heap allocations, bounded work).
    ///
    /// Derives Bernstein control points directly from the stored power coefficients `a0..=a5`
    /// that `self.position(t)` evaluates, accounting for conversion/Horner rounding, handling
    /// exact boundary touches (constant holds and monotonic approaches ending on `lo`/`hi`),
    /// and failing closed on non-finite coefficients or uncertified boundary extrema.
    pub fn certify_within(&self, lo: f64, hi: f64) -> Result<(), f64> {
        self.certify_bounds(Some(lo), Some(hi))
    }

    /// Certify that the executed `f32` position curve `self.position(t) as f32` stays within
    /// `[lo, hi]` for every `t in [0, self.duration]` (zero heap allocations, bounded work).
    ///
    /// Uses the strict IEEE-754 `f32` rounding preimage (`0.25 * ulp` inside the half-ULP
    /// boundary around `lo` and `hi`): every `f64` inside `[lo_pre, hi_pre]` rounds to a value in
    /// `[lo, hi]` in `f32`, while genuine excursions (even `1e-22` past `0.0f32`) remain rejected.
    pub fn certify_f32_within(&self, lo: f32, hi: f32) -> Result<(), f64> {
        if !lo.is_finite() || !hi.is_finite() || lo > hi {
            return Err(f64::NAN);
        }
        let lo_f64 = f64::from(lo);
        let hi_f64 = f64::from(hi);
        let lo_pre = lo_f64 - 0.25 * (lo_f64 - f64::from(lo.next_down()));
        let hi_pre = hi_f64 + 0.25 * (f64::from(hi.next_up()) - hi_f64);
        self.certify_bounds(Some(lo_pre), Some(hi_pre))
    }

    /// Certify that `self.position(t)` satisfies optional lower (`lo`) and/or upper (`hi`) bounds
    /// for every `t in [0, self.duration]` (zero heap allocations, bounded work).
    pub fn certify_bounds(&self, lo_opt: Option<f64>, hi_opt: Option<f64>) -> Result<(), f64> {
        if lo_opt.is_none() && hi_opt.is_none() {
            return Ok(());
        }
        if lo_opt.is_some_and(|v| !v.is_finite())
            || hi_opt.is_some_and(|v| !v.is_finite())
            || matches!((lo_opt, hi_opt), (Some(l), Some(h)) if l > h)
            || !self.duration.is_finite()
            || self.duration <= 0.0
            || !self.p0.is_finite()
            || !self.p1.is_finite()
            || !self.a0.is_finite()
            || !self.a1.is_finite()
            || !self.a2.is_finite()
            || !self.a3.is_finite()
            || !self.a4.is_finite()
            || !self.a5.is_finite()
        {
            return Err(f64::NAN);
        }
        let check_val = |v: f64| -> bool {
            lo_opt.is_none_or(|lo| v >= lo) && hi_opt.is_none_or(|hi| v <= hi)
        };
        if !check_val(self.p0) {
            return Err(self.p0);
        }
        if !check_val(self.p1) {
            return Err(self.p1);
        }
        if !check_val(self.a0) {
            return Err(self.a0);
        }

        // Fast-path: exact constant polynomial (covers constant holds directly on a bound).
        if self.a1 == 0.0 && self.a2 == 0.0 && self.a3 == 0.0 && self.a4 == 0.0 && self.a5 == 0.0 {
            return Ok(());
        }

        let t = self.duration;
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;
        let c0 = self.a0;
        let c1 = self.a1 * t;
        let c2 = self.a2 * t2;
        let c3 = self.a3 * t3;
        let c4 = self.a4 * t4;
        let c5 = self.a5 * t5;
        if !c1.is_finite() || !c2.is_finite() || !c3.is_finite() || !c4.is_finite() || !c5.is_finite() {
            return Err(f64::NAN);
        }
        let mag = c0.abs() + c1.abs() + c2.abs() + c3.abs() + c4.abs() + c5.abs();
        let round_eps = 64.0 * f64::EPSILON * mag.max(1e-30);

        // Exact local Taylor check at t = 0: if the curve starts on a bound and its first
        // non-zero derivative points outward, it exits [lo, hi] immediately for small t > 0.
        if let Some(first_nz) = [self.a1, self.a2, self.a3, self.a4, self.a5]
            .into_iter()
            .find(|&c| c != 0.0)
        {
            if let Some(hi) = hi_opt {
                if self.a0 >= hi && first_nz > 0.0 {
                    return Err(hi);
                }
            }
            if let Some(lo) = lo_opt {
                if self.a0 <= lo && first_nz < 0.0 {
                    return Err(lo);
                }
            }
        }

        // Exact shared numerical contract between evaluator (`Self::position`) and certificate:
        // verified rest-to-rest minimum-jerk S-curves evaluate via the symmetric cancellation-free
        // form clamped to `[min(p0, p1), max(p0, p1)]`, so endpoint checks certify the full curve.
        if self.is_exact_rest_to_rest() {
            return Ok(());
        }

        // Exact monomial-to-Bernstein conversion of the executed polynomial on u = t / T in [0, 1].
        let b0 = c0;
        let b1 = c0 + c1 * 0.2;
        let b2 = c0 + c1 * 0.4 + c2 * 0.1;
        let b3 = c0 + c1 * 0.6 + c2 * 0.3 + c3 * 0.1;
        let b4 = c0 + c1 * 0.8 + c2 * 0.6 + c3 * 0.4 + c4 * 0.2;
        let b5 = c0 + c1 + c2 + c3 + c4 + c5;
        let root = [b0, b1, b2, b3, b4, b5];
        if !root.iter().all(|v| v.is_finite()) {
            return Err(f64::NAN);
        }
        if !check_val(b0) {
            return Err(b0);
        }
        if lo_opt.is_some_and(|lo| b5 < lo - round_eps) || hi_opt.is_some_and(|hi| b5 > hi + round_eps) {
            return Err(b5);
        }
        if (b5 - self.p1).abs() > round_eps && !check_val(b5) {
            return Err(b5);
        }

        // Conservative first-derivative monotonicity proof (zero slack).
        let is_strictly_monotonic = |b: &[f64; 6]| -> bool {
            let inc = b.windows(2).all(|w| w[1] - w[0] >= round_eps);
            let dec = b.windows(2).all(|w| w[0] - w[1] >= round_eps);
            inc || dec
        };

        // Evaluate derivative p'(u*T) directly from the executed power coefficients a1..=a5.
        let poly_deriv = |u: f64| -> f64 {
            let tau = u * t;
            self.a1
                + tau
                    * (2.0 * self.a2
                        + tau * (3.0 * self.a3 + tau * (4.0 * self.a4 + 5.0 * self.a5 * tau)))
        };

        const MAX_STACK: usize = 32;
        const MAX_DEPTH: u8 = 14;
        const MAX_SPLITS: usize = 64;
        let mut stack: [([f64; 6], f64, f64, u8); MAX_STACK] = [([0.0; 6], 0.0, 0.0, 0); MAX_STACK];
        let mut sp = 1usize;
        stack[0] = (root, 0.0, 1.0, 0);
        let mut splits = 0usize;

        while sp > 0 {
            sp -= 1;
            let (b, u0, u1, depth) = stack[sp];
            let (mut bmin, mut bmax) = (f64::INFINITY, f64::NEG_INFINITY);
            for &v in &b {
                if v < bmin { bmin = v; }
                if v > bmax { bmax = v; }
            }
            if lo_opt.is_none_or(|lo| bmin >= lo + round_eps)
                && hi_opt.is_none_or(|hi| bmax <= hi - round_eps)
            {
                continue;
            }
            if u0 > 0.0 {
                let v0 = self.position(u0 * t);
                if !check_val(v0) {
                    return Err(v0);
                }
            }
            if u1 < 1.0 {
                let v1 = self.position(u1 * t);
                if !check_val(v1) {
                    return Err(v1);
                }
            }
            let u_mid = 0.5 * (u0 + u1);
            let v_mid = self.position(u_mid * t);
            if !check_val(v_mid) {
                return Err(v_mid);
            }
            if is_strictly_monotonic(&b) {
                continue;
            }
            // Conservative second-derivative convexity/concavity proof: handles exact interior
            // tangent touches (e.g. p(t) = -(t - 0.5)^2 <= 0) via unique stationary-point bisection.
            let dd = [
                b[2] - 2.0 * b[1] + b[0],
                b[3] - 2.0 * b[2] + b[1],
                b[4] - 2.0 * b[3] + b[2],
                b[5] - 2.0 * b[4] + b[3],
            ];
            let strictly_concave = dd.iter().all(|&v| v <= -round_eps);
            let strictly_convex = dd.iter().all(|&v| v >= round_eps);
            if strictly_concave || strictly_convex {
                // On a strictly concave (resp. convex) interval, p'(u) is strictly decreasing
                // (resp. increasing), so any interior extremum is unique and bracketed by bisection.
                let (mut a, mut c) = (u0, u1);
                for _ in 0..56 {
                    let m = 0.5 * (a + c);
                    let dm = poly_deriv(m);
                    if (strictly_concave && dm > 0.0) || (strictly_convex && dm < 0.0) {
                        a = m;
                    } else {
                        c = m;
                    }
                }
                let u_star = 0.5 * (a + c);
                for u_chk in [a, u_star, c] {
                    if u_chk > 0.0 && u_chk < 1.0 {
                        let val = self.position(u_chk * t);
                        if !check_val(val) {
                            return Err(val);
                        }
                    }
                }
                // Verify the remaining non-endpoint bound (min for concave, max for convex) is
                // already certified by the checked endpoints u0, u1.
                continue;
            }
            if depth >= MAX_DEPTH || splits >= MAX_SPLITS || sp + 2 > MAX_STACK {
                let worst = match (lo_opt, hi_opt) {
                    (Some(lo), Some(hi)) => if (hi - bmax) < (bmin - lo) { bmax } else { bmin },
                    (Some(_), None) => bmin,
                    (None, Some(_)) => bmax,
                    (None, None) => bmax,
                };
                return Err(worst);
            }
            splits += 1;
            let mut d = b;
            let mut left = [0.0f64; 6];
            let mut right = [0.0f64; 6];
            left[0] = d[0];
            right[5] = d[5];
            for k in 1..6 {
                for j in 0..(6 - k) {
                    d[j] = 0.5 * (d[j] + d[j + 1]);
                }
                left[k] = d[0];
                right[5 - k] = d[5 - k];
            }
            stack[sp] = (right, u_mid, u1, depth + 1);
            sp += 1;
            stack[sp] = (left, u0, u_mid, depth + 1);
            sp += 1;
        }
        Ok(())
    }

    /// True when stored coefficients and boundary metadata match an untampered rest-to-rest
    /// minimum-jerk quintic (`v0 = v1 = a0 = a1 = 0`). Shared by [`Self::position`] and
    /// [`Self::certify_bounds`] so evaluator and certificate uphold the exact same contract.
    #[inline]
    fn is_exact_rest_to_rest(&self) -> bool {
        if self.v0 != 0.0
            || self.v1 != 0.0
            || self.a_start != 0.0
            || self.a_end != 0.0
            || self.a0 != self.p0
            || self.a1 != 0.0
            || self.a2 != 0.0
            || !self.duration.is_finite()
            || self.duration <= 0.0
        {
            return false;
        }
        let t = self.duration;
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;
        let h = self.p1 - self.p0;
        let h_eps = 64.0 * f64::EPSILON * h.abs().max(1e-30);
        (self.a3 * t3 - 10.0 * h).abs() <= h_eps
            && (self.a4 * t4 + 15.0 * h).abs() <= h_eps
            && (self.a5 * t5 - 6.0 * h).abs() <= h_eps
    }

    /// Sample position at time t
    pub fn position(&self, t: f64) -> f64 {
        if t <= 0.0 {
            return self.p0;
        }
        if t >= self.duration {
            return self.p1;
        }
        if self.is_exact_rest_to_rest() {
            let u = t / self.duration;
            let s = if u <= 0.5 {
                let u2 = u * u;
                u2 * u * (10.0 + u * (-15.0 + 6.0 * u))
            } else {
                let w = 1.0 - u;
                let w2 = w * w;
                1.0 - w2 * w * (1.0 + u * (3.0 + 6.0 * u))
            };
            let val = self.p0 + (self.p1 - self.p0) * s.clamp(0.0, 1.0);
            return val.clamp(self.p0.min(self.p1), self.p0.max(self.p1));
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

    /// Upper bound on |velocity| over `[t0, t1]` (within the duration): the exact Taylor
    /// expansion of the quartic velocity at the midpoint, every term taken by magnitude.
    pub fn speed_bound(&self, t0: f64, t1: f64) -> f64 {
        let (t, h) = (0.5 * (t0 + t1), 0.5 * (t1 - t0).abs());
        let d = [
            self.a1 + t * (2.0 * self.a2 + t * (3.0 * self.a3 + t * (4.0 * self.a4 + 5.0 * self.a5 * t))),
            2.0 * self.a2 + t * (6.0 * self.a3 + t * (12.0 * self.a4 + 20.0 * self.a5 * t)),
            6.0 * self.a3 + t * (24.0 * self.a4 + 60.0 * self.a5 * t),
            24.0 * self.a4 + 120.0 * self.a5 * t,
            120.0 * self.a5,
        ];
        let mut bound = 0.0;
        let mut hk = 1.0;
        let mut fact = 1.0;
        for (k, dk) in d.iter().enumerate() {
            if k > 0 { hk *= h; fact *= k as f64; }
            bound += dk.abs() * hk / fact;
        }
        bound
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
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TrajectoryPoint {
    pub positions: Vec<f32>,
    pub velocities: Vec<f32>,
    pub accelerations: Vec<f32>,
}

/// Maximum joint count stored inline in [`JointPolynomials`] without heap allocation.
pub const INLINE_POLYS: usize = crate::model::MAX_MOTION_DOF;

/// Fixed-storage polynomial array for up to [`INLINE_POLYS`] joints (zero allocations and
/// zero frees on RT jog and replay approach commits), falling back to `Vec` above that.
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)]
pub enum JointPolynomials {
    Inline {
        data: [QuinticPolynomial; INLINE_POLYS],
        len: usize,
    },
    Heap(Vec<QuinticPolynomial>),
}

impl JointPolynomials {
    pub fn as_slice(&self) -> &[QuinticPolynomial] {
        match self {
            Self::Inline { data, len } => &data[..*len],
            Self::Heap(v) => v.as_slice(),
        }
    }

    pub fn as_mut_slice(&mut self) -> &mut [QuinticPolynomial] {
        match self {
            Self::Inline { data, len } => &mut data[..*len],
            Self::Heap(v) => v.as_mut_slice(),
        }
    }

    /// True when stored on the heap (`dof > INLINE_POLYS`).
    pub fn is_heap(&self) -> bool {
        matches!(self, Self::Heap(_))
    }
}

impl std::ops::Deref for JointPolynomials {
    type Target = [QuinticPolynomial];
    fn deref(&self) -> &[QuinticPolynomial] { self.as_slice() }
}

impl std::ops::DerefMut for JointPolynomials {
    fn deref_mut(&mut self) -> &mut [QuinticPolynomial] { self.as_mut_slice() }
}

impl<'a> IntoIterator for &'a JointPolynomials {
    type Item = &'a QuinticPolynomial;
    type IntoIter = std::slice::Iter<'a, QuinticPolynomial>;
    fn into_iter(self) -> Self::IntoIter { self.as_slice().iter() }
}

impl<'a> IntoIterator for &'a mut JointPolynomials {
    type Item = &'a mut QuinticPolynomial;
    type IntoIter = std::slice::IterMut<'a, QuinticPolynomial>;
    fn into_iter(self) -> Self::IntoIter { self.as_mut_slice().iter_mut() }
}

/// Multi-joint quintic polynomial trajectory
#[derive(Debug, Clone)]
pub struct MultiJointQuinticTrajectory {
    pub polynomials: JointPolynomials,
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

        let polynomials = if dof <= INLINE_POLYS {
            let zero = QuinticPolynomial {
                a0: 0.0, a1: 0.0, a2: 0.0, a3: 0.0, a4: 0.0, a5: 0.0,
                duration, p0: 0.0, p1: 0.0, v0: 0.0, v1: 0.0, a_start: 0.0, a_end: 0.0,
            };
            let mut data = [zero; INLINE_POLYS];
            for i in 0..dof {
                data[i] = QuinticPolynomial::new(
                    start_pos[i] as f64,
                    start_vel[i] as f64,
                    0.0,
                    target_pos[i] as f64,
                    target_vel[i] as f64,
                    0.0,
                    duration,
                )?;
            }
            JointPolynomials::Inline { data, len: dof }
        } else {
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
            JointPolynomials::Heap(polynomials)
        };

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

    /// [`Self::step`] into `out`, reusing its vectors (no allocation once they have the
    /// capacity: the control tick's form).
    pub fn step_into(&mut self, dt: f64, out: &mut TrajectoryPoint) {
        self.elapsed += dt;
        self.sample_into(self.elapsed, out);
    }

    /// Positions and velocities of [`Self::step`] into `positions` / `velocities` (reused).
    pub fn step_pv_into(&mut self, dt: f64, positions: &mut Vec<f32>, velocities: &mut Vec<f32>) {
        self.elapsed += dt;
        let t = self.elapsed;
        positions.clear();
        velocities.clear();
        for poly in &self.polynomials {
            positions.push(poly.position(t) as f32);
            velocities.push(poly.velocity(t) as f32);
        }
    }

    /// [`Self::sample`] into `out`, reusing its vectors.
    pub fn sample_into(&self, t: f64, out: &mut TrajectoryPoint) {
        out.positions.clear();
        out.velocities.clear();
        out.accelerations.clear();
        for poly in &self.polynomials {
            out.positions.push(poly.position(t) as f32);
            out.velocities.push(poly.velocity(t) as f32);
            out.accelerations.push(poly.acceleration(t) as f32);
        }
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

    /// Sample positions at time `t` into `out` without allocating (`out.len() >= dof`).
    pub fn sample_positions_into(&self, t: f64, out: &mut [f32]) {
        for (dst, poly) in out.iter_mut().zip(self.polynomials.iter()) {
            *dst = poly.position(t) as f32;
        }
    }

    /// Upper bound on how far each joint travels during `[t0, t1]` (rad) into `out` without allocating.
    pub fn travel_bound_into(&self, t0: f64, t1: f64, out: &mut [f64]) {
        let (a, b) = (t0.clamp(0.0, self.duration), t1.clamp(0.0, self.duration));
        let dt = (b - a).abs();
        for (dst, poly) in out.iter_mut().zip(self.polynomials.iter()) {
            *dst = poly.speed_bound(a, b) * dt;
        }
    }

    /// Upper bound on how far each joint travels during `[t0, t1]` (rad).
    pub fn travel_bound(&self, t0: f64, t1: f64) -> Vec<f64> {
        let (a, b) = (t0.clamp(0.0, self.duration), t1.clamp(0.0, self.duration));
        self.polynomials.iter().map(|p| p.speed_bound(a, b) * (b - a).abs()).collect()
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

                for (i, poly) in traj.polynomials.iter().enumerate() {
                    if i < limits.len() {
                        let v_max = limits[i].max_velocity * 1.001;
                        let a_max = limits[i].max_acceleration * 1.001;
                        let pos = poly.position(t) as f32;
                        let vel = poly.velocity(t) as f32;
                        let acc = poly.acceleration(t) as f32;

                        // Check position soft limits
                        if pos < limits[i].min_position - 1e-4
                            || pos > limits[i].max_position + 1e-4
                        {
                            exceeds_limits = true;
                            break;
                        }
                        if limits[i].max_velocity > 1e-6 && vel.abs() > v_max {
                            exceeds_limits = true;
                            break;
                        }
                        if limits[i].max_acceleration > 1e-6 && acc.abs() > a_max {
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
