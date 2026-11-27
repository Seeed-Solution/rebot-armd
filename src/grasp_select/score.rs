//! Ranking of fully checked candidates: contact quality, robustness under coherent
//! perturbations of the evidence, arm margins, and tie-breaks. All weights are fixed
//! starting values (design "Scoring"); every component is returned.

use super::candidates::{Chord, DirClass, Hypothesis};
use super::feasibility::{self, Checked, Work};
use super::geometry as g;
use super::snapshot::{PosePreferences, Side, Snapshot, Target, OVER_TILT_PENALTY_PER_RAD, SIDE_MISMATCH_PENALTY, TABLE_ERROR_M};
use crate::kinematics::{self, Pose};

/// Joint-limit margin that counts as full (normalized).
pub const FULL_JOINT_MARGIN: f64 = 0.2;
/// Jacobian singular value that counts as full (m/rad).
pub const FULL_SIGMA: f64 = 0.05;
/// Opening reserve per side that counts as full (m).
pub const FULL_RESERVE_M: f64 = 0.01;
pub const W_DURATION_PER_S: f64 = 0.02;
pub const W_TRAVEL_PER_RAD: f64 = 0.01;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Scores {
    pub contact: f64,
    pub robustness: f64,
    pub margins: f64,
    /// Duration and joint travel (subtracted).
    pub tie_break: f64,
    /// Vertical weight, preferred tilt and side (subtracted).
    pub preference: f64,
    pub total: f64,
}

/// Worst credible values under the perturbations.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Robustness {
    pub worst_opening_reserve_m: f64,
    pub worst_pad_overlap_m: f64,
    pub worst_table_clearance_m: f64,
    /// Perturbed contact and pre-grasp poses with an IK solution / tried.
    pub ik_ok: usize,
    pub ik_tried: usize,
}

/// Contact quality of a chord taken by `hy`: opposition, pad overlap, opening reserve and the
/// moment arm to the estimated centre of mass. For a nearly round footprint the moment term
/// (which relies on the uncertain long axis) gets less weight.
pub fn contact_quality(s: &Snapshot, t: &Target, ch: &Chord, hy: &Hypothesis) -> f64 {
    let overlap = (hy.pad_overlap_m / s.tool.pad_length_m).clamp(0.0, 1.0);
    let reserve = (hy.opening_reserve_m / FULL_RESERVE_M).clamp(0.0, 1.0);
    let opp = ch.opposition.clamp(0.0, 1.0) * ch.pad_support.clamp(0.0, 1.0);
    let w_moment = if t.axis_ambiguous { 0.1 } else { 0.2 };
    let central = 1.0 - 2.0 * (ch.station - 0.5).abs();
    (0.35 * opp + 0.25 * overlap + 0.2 * reserve + w_moment * ch.moment_q + (0.2 - w_moment) * central) / 1.0
}

/// Preference penalty of `hy` (vertical weight per radian, preferred tilt, side).
pub fn preference_penalty(p: &PosePreferences, hy: &Hypothesis) -> f64 {
    let tilt = hy.tilt_deg.to_radians();
    let mut pen = p.vertical_weight * tilt;
    if let Some(m) = p.preferred_max_tilt_deg {
        if hy.tilt_deg > m { pen += OVER_TILT_PENALTY_PER_RAD * (hy.tilt_deg - m).to_radians(); }
    }
    let side = hy.class.side();
    if p.approach_side != Side::Any && hy.class != DirClass::Vertical && side != p.approach_side { pen += SIDE_MISMATCH_PENALTY; }
    pen
}

/// Robustness of a checked candidate: the object shifted by the scale error (radially), the
/// hand-eye error (tangentially) and the contour error (along the jaws), the table moved by
/// ±[`TABLE_ERROR_M`], the chord scaled by ±3 %; contact and pre-grasp IK at the shifted poses.
pub fn robustness(s: &Snapshot, t: &Target, hy: &Hypothesis, c: &Checked, work: &mut Work) -> Robustness {
    robustness_budget(s, t, hy, c, work, &|| false).expect("unbounded robustness never expires")
}

/// [`robustness`] checking `out_of_budget()` across every perturbed IK solve; returns `None` if
/// the budget or cancellation flag trips.
pub fn robustness_budget(
    s: &Snapshot,
    t: &Target,
    hy: &Hypothesis,
    c: &Checked,
    work: &mut Work,
    out_of_budget: &dyn Fn() -> bool,
) -> Option<Robustness> {
    if out_of_budget() {
        return None;
    }
    let rho = g::unit2([hy.g[0], hy.g[1]]);
    let lam = g::perp2(rho);
    let y = kinematics::column(&hy.r, 1);
    let yh = g::unit2([y[0], y[1]]);
    let shifts = [g::scale2(rho, t.scale_error_m), g::scale2(rho, -t.scale_error_m), g::scale2(lam, t.hand_eye_error_m), g::scale2(lam, -t.hand_eye_error_m)];
    let w_hi = hy.width_m * (1.0 + super::snapshot::SCALE_ERROR);
    let worst_reserve = shifts.iter().map(|sh| (s.tool.open_width_m - w_hi) / 2.0 - g::dot2(*sh, yh).abs() - t.contour_error_m)
        .fold(f64::INFINITY, f64::min);
    let cos_t = -hy.a[2];
    let pad = s.tool.pad_length_m;
    let top_lo = t.table_z + t.height[0] - TABLE_ERROR_M;
    let worst_overlap = if cos_t < 0.2 { if hy.g[2] <= top_lo { pad } else { 0.0 } } else { ((top_lo - hy.g[2]) / cos_t).clamp(0.0, pad) };
    let worst_table = c.margins.table_clearance_m - TABLE_ERROR_M;
    let mut ok = 0;
    let mut tried = 0;
    for sh in &shifts {
        let d = [sh[0], sh[1], 0.0];
        for (pose, seed) in [(hy.grasp_pose(), c.q_grasp), (hy.pregrasp_pose(), c.q_pre)] {
            if out_of_budget() {
                return None;
            }
            tried += 1;
            let p = Pose::new(pose.r, kinematics::add(&pose.p, &d));
            match feasibility::endpoint_ik_budget(&s.input, &p, &[seed], work, out_of_budget) {
                Ok(q) => {
                    if feasibility::joint_margin(&s.input, &q) > 0.0 { ok += 1; }
                }
                Err(feasibility::EndpointIkError::BudgetExhausted) => return None,
                Err(_) => {}
            }
        }
    }
    if out_of_budget() {
        return None;
    }
    Some(Robustness { worst_opening_reserve_m: worst_reserve, worst_pad_overlap_m: worst_overlap, worst_table_clearance_m: worst_table, ik_ok: ok, ik_tried: tried })
}

/// The scores of a checked candidate.
pub fn score(s: &Snapshot, t: &Target, ch: &Chord, hy: &Hypothesis, c: &Checked, r: &Robustness) -> Scores {
    let contact = contact_quality(s, t, ch, hy);
    let robustness = 0.35 * (r.worst_opening_reserve_m / FULL_RESERVE_M).clamp(0.0, 1.0)
        + 0.25 * (r.worst_pad_overlap_m / s.tool.pad_length_m).clamp(0.0, 1.0)
        + 0.15 * (r.worst_table_clearance_m / 0.005).clamp(0.0, 1.0)
        + 0.25 * if r.ik_tried > 0 { r.ik_ok as f64 / r.ik_tried as f64 } else { 0.0 };
    let margins = 0.5 * (c.margins.joint_limit / FULL_JOINT_MARGIN).clamp(0.0, 1.0) + 0.5 * (c.margins.sigma_min / FULL_SIGMA).clamp(0.0, 1.0);
    let tie_break = W_DURATION_PER_S * c.duration_s + W_TRAVEL_PER_RAD * c.joint_travel_rad;
    let preference = preference_penalty(&s.prefs, hy);
    let total = contact + robustness + 0.5 * margins - tie_break - preference;
    Scores { contact, robustness, margins, tie_break, preference, total }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hy(class: DirClass, tilt: f64) -> Hypothesis {
        Hypothesis {
            index: 0, chord: 0, class, tilt_deg: tilt, deep: false, half_turn: false, a: [0.0, 0.0, -1.0], r: kinematics::IDENTITY,
            g: [0.4, 0.0, 0.02], insertion_m: 0.05, width_m: 0.03, pad_overlap_m: 0.02, opening_reserve_m: 0.01, table_clearance_m: 0.005, jaw_elevation_deg: 0.0,
        }
    }

    #[test]
    fn preference_penalties() {
        let p = PosePreferences::default();
        assert_eq!(preference_penalty(&p, &hy(DirClass::Vertical, 0.0)), 0.0);
        let tilted = preference_penalty(&p, &hy(DirClass::Outward, 30.0));
        assert!((tilted - 0.3 * 30f64.to_radians()).abs() < 1e-12);
        let q = PosePreferences { preferred_max_tilt_deg: Some(20.0), approach_side: Side::Inward, ..p };
        let pen = preference_penalty(&q, &hy(DirClass::Outward, 30.0));
        assert!((pen - tilted - OVER_TILT_PENALTY_PER_RAD * 10f64.to_radians() - SIDE_MISMATCH_PENALTY).abs() < 1e-12);
    }
}
