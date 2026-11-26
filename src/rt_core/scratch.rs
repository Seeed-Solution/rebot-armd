//! Preallocated buffers of the control tick (armd 1.6 RT core, design section 5: no
//! allocation or deallocation on the RT thread in a steady-state tick).
//!
//! The tick reads the driver into these buffers (`ArmDriver::*_into`), builds its
//! temporaries in them and copies state with [`copy_into`] instead of `clone()`, so a
//! steady-state tick reuses the same memory every period. Every buffer is created with the
//! capacity of the arm's profile (dof joints); a buffer only grows (allocates) when a
//! longer value arrives, which a profile-sized arm does not produce.

use std::time::{Duration, Instant};

use crate::driver::JointTarget;
use crate::model::{JointLimit, JointState};

/// Copies `src` into `dst`, reusing `dst`'s memory (no allocation while it has the capacity).
#[inline]
pub fn copy_into<T: Copy>(dst: &mut Vec<T>, src: &[T]) {
    dst.clear();
    dst.extend_from_slice(src);
}

/// Sets every element of `dst` to `v` and its length to `n` (no allocation while `dst` has
/// the capacity): the in-place form of `*dst = vec![v; n]`.
#[inline]
pub fn fill_n<T: Copy>(dst: &mut Vec<T>, v: T, n: usize) {
    dst.clear();
    dst.resize(n, v);
}

/// Copies joint limits into `dst` without allocating once `dst` holds limits with names of
/// the same length (`JointLimit::name` is a `String`; `clone_from` reuses its buffer).
pub fn copy_limits(dst: &mut Vec<JointLimit>, src: &[JointLimit]) {
    dst.truncate(src.len());
    let have = dst.len();
    for (d, s) in dst.iter_mut().zip(src) { d.clone_from(s); }
    dst.extend(src[have..].iter().cloned());
}

/// `*dst = src.cloned()` reusing `dst`'s memory when both are `Some` (`clone_from`).
#[inline]
pub fn set_opt<T: Clone>(dst: &mut Option<T>, src: Option<&T>) {
    match (dst.as_mut(), src) {
        (Some(d), Some(s)) => d.clone_from(s),
        (_, s) => *dst = s.cloned(),
    }
}

/// [`set_opt`] only when the values differ (no copy at all while unchanged; for values
/// whose `clone_from` is not field-wise).
#[inline]
pub fn set_opt_eq<T: Clone + PartialEq>(dst: &mut Option<T>, src: Option<&T>) {
    if dst.as_ref() != src { set_opt(dst, src); }
}

/// `*dst = src.map(str::to_owned)` reusing `dst`'s buffer.
#[inline]
pub fn set_opt_str(dst: &mut Option<String>, src: Option<&str>) {
    match (dst.as_mut(), src) {
        (Some(d), Some(s)) => { if d != s { d.clear(); d.push_str(s); } }
        (_, s) => *dst = s.map(str::to_owned),
    }
}

/// `*dst = src.to_owned()` reusing `dst`'s buffer.
#[inline]
pub fn set_str(dst: &mut String, src: &str) {
    if dst != src { dst.clear(); dst.push_str(src); }
}

/// Per-joint feedback timing read from the driver (`record_driver_feedback_timing`).
#[derive(Debug, Default)]
pub struct FeedbackScratch {
    pub instants: Vec<Option<Instant>>,
    pub seqs: Vec<u64>,
    pub bounds: Vec<Option<Instant>>,
}

/// Per-joint hard lead bounds `(upper [(val, rate); 2], upper_len, lower [(val, rate); 2], lower_len)`
/// used by position-lead tracking with zero heap allocation per tick.
pub type TrackingBoundsEntry = ([(f64, f64); 2], u8, [(f64, f64); 2], u8);

/// Preallocated scratch buffers for position-lead tracking and residual lead ramp-out on
/// `armd-control`, sized off-thread from `dof`.
#[derive(Debug, Default)]
pub struct TrackingScratch {
    pub base_lead: Vec<f32>,
    pub zero_target: Vec<f32>,
    pub seeded_q: Vec<f32>,
    pub nom_q_prev: Vec<f64>,
    pub nom_v_prev: Vec<f64>,
    pub v_nom: Vec<f64>,
    pub a_nom: Vec<f64>,
    pub out_lim: Vec<(f64, f64)>,
    pub window: Vec<(f64, f64)>,
    pub lead_prev: Vec<f64>,
    pub bounds: Vec<TrackingBoundsEntry>,
    pub cand_lead: Vec<f32>,
    pub cand_zero: Vec<f32>,
    pub out_positions: Vec<f32>,
    pub out_velocities: Vec<f32>,
}

impl TrackingScratch {
    /// Preallocates all tracking scratch vectors with capacity for `dof` joints.
    pub fn with_dof(dof: usize) -> Self {
        let n = dof.max(1);
        Self {
            base_lead: Vec::with_capacity(n),
            zero_target: Vec::with_capacity(n),
            seeded_q: Vec::with_capacity(n),
            nom_q_prev: Vec::with_capacity(n),
            nom_v_prev: Vec::with_capacity(n),
            v_nom: Vec::with_capacity(n),
            a_nom: Vec::with_capacity(n),
            out_lim: Vec::with_capacity(n),
            window: Vec::with_capacity(n),
            lead_prev: Vec::with_capacity(n),
            bounds: Vec::with_capacity(n),
            cand_lead: Vec::with_capacity(n),
            cand_zero: Vec::with_capacity(n),
            out_positions: Vec::with_capacity(n),
            out_velocities: Vec::with_capacity(n),
        }
    }
}

/// The tick's buffers.
#[derive(Debug, Default)]
pub struct TickScratch {
    pub states: Vec<JointState>,
    pub ages: Vec<Option<Duration>>,
    pub fb: FeedbackScratch,
    pub rx: Vec<Option<Instant>>,
    pub rx_vals: Vec<Instant>,
    pub recovered: Vec<usize>,
    pub loads: Vec<f32>,
    pub status: Vec<u8>,
    pub before_velocities: Vec<f32>,
    pub next_positions: Vec<f32>,
    pub next_velocities: Vec<f32>,
    pub reached: Vec<bool>,
    pub lead_capped: Vec<bool>,
    pub step_limits: Vec<JointLimit>,
    pub fallback_limits: Vec<JointLimit>,
    pub fallback_prev_confirmed: Vec<f32>,
    pub fallback_prev_confirmed_valid: bool,
    pub targets: Vec<JointTarget>,
    pub servo_target: Vec<f32>,
    pub hold: Vec<f32>,
    pub point: crate::trajectory::TrajectoryPoint,
    pub step_out: crate::motion::StepOut,
    pub tracking: TrackingScratch,
}

impl TickScratch {
    /// Buffers with the capacity of a `dof`-joint arm.
    pub fn with_dof(dof: usize) -> Self {
        let n = dof.max(1);
        Self {
            states: Vec::with_capacity(n),
            ages: Vec::with_capacity(n),
            fb: FeedbackScratch { instants: Vec::with_capacity(n), seqs: Vec::with_capacity(n), bounds: Vec::with_capacity(n) },
            rx: Vec::with_capacity(n),
            rx_vals: Vec::with_capacity(n),
            recovered: Vec::with_capacity(n),
            loads: Vec::with_capacity(n),
            status: Vec::with_capacity(n),
            before_velocities: Vec::with_capacity(n),
            next_positions: Vec::with_capacity(n),
            next_velocities: Vec::with_capacity(n),
            reached: Vec::with_capacity(n),
            lead_capped: Vec::with_capacity(n),
            step_limits: Vec::with_capacity(n),
            fallback_limits: Vec::with_capacity(n),
            fallback_prev_confirmed: Vec::with_capacity(n),
            fallback_prev_confirmed_valid: false,
            targets: Vec::with_capacity(n),
            servo_target: Vec::with_capacity(n),
            hold: Vec::with_capacity(n),
            point: crate::trajectory::TrajectoryPoint {
                positions: Vec::with_capacity(n),
                velocities: Vec::with_capacity(n),
                accelerations: Vec::with_capacity(n),
            },
            step_out: crate::motion::StepOut { positions: Vec::with_capacity(n), velocities: Vec::with_capacity(n), ..Default::default() },
            tracking: TrackingScratch::with_dof(n),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn helpers_reuse_memory() {
        let mut v: Vec<f32> = Vec::with_capacity(7);
        let p = v.as_ptr();
        copy_into(&mut v, &[1.0; 7]);
        fill_n(&mut v, 0.0, 7);
        copy_into(&mut v, &[2.0; 3]);
        assert_eq!((v.as_ptr(), v.as_slice()), (p, &[2.0f32; 3][..]));
        let src = vec![JointLimit::new("j1", -1.0, 1.0, 2.0, 3.0), JointLimit::new("j2", -1.0, 1.0, 2.0, 3.0)];
        let mut dst = Vec::new();
        copy_limits(&mut dst, &src);
        assert_eq!(dst, src);
        let mut src2 = src.clone();
        src2[1].max_velocity = 0.5;
        let names = dst[1].name.as_ptr();
        copy_limits(&mut dst, &src2);
        assert_eq!((dst.as_slice(), dst[1].name.as_ptr()), (src2.as_slice(), names));
    }
}
