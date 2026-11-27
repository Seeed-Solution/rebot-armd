//! Shared grasp admission of both strategies: the jaw predicate (dry run, live planning and
//! fresh-evidence revalidation) and the top-down candidate (the legacy route as one candidate
//! of the adaptive search). On 6-DOF arms the top-down candidate runs through the same
//! whole-route preflight as every adaptive candidate, first and with its own budget, so the
//! adaptive strategy accepts every target the legacy strategy accepts on the same snapshot.

use std::time::{Duration, Instant};

use serde_json::{json, Value};

use super::candidates::{self, CheapStats, Chord, DirClass, Hypothesis, Rejection, MIN_OPENING_RESERVE_M};
use super::feasibility::{self, Work};
use super::geometry as g;
use super::snapshot::{self, Snapshot, Target};
use super::{score, AdaptiveRoute, Candidate};
use crate::grasp::PendingAdaptive;
use crate::kinematics;
use crate::supervisor::ArmError;

/// Hypothesis index of the top-down candidate (ids `<snapshot>-top`).
pub const TOP_DOWN_INDEX: usize = 99_999;
/// Wall budget reserved for the top-down candidate, outside the request's search budget.
pub const TOP_DOWN_BUDGET_MS: u64 = 1500;
/// Floating-point slack of the jaw predicate (an exact-threshold width passes).
const JAW_EPS_M: f64 = 1e-9;

/// The jaw predicate: `width` along the closing axis is finite, at most `max_grasp_width_m`,
/// and `width + 2·hard_error + 2·MIN_OPENING_RESERVE_M <= open_width`. Returns the per-side
/// reserve `(open_width - width - 2·hard_error) / 2`.
pub fn jaw_check(width_m: f64, hard_error_m: f64, open_width_m: f64, max_grasp_width_m: f64) -> Result<f64, Rejection> {
    if !width_m.is_finite() || width_m <= 0.0 {
        return Err(Rejection::new("geometry", "NO_WIDTH_EVIDENCE", "no finite object width along the jaw closing axis"));
    }
    if width_m > max_grasp_width_m + JAW_EPS_M {
        return Err(Rejection::new("geometry", "TOO_WIDE", format!("chord {width_m:.3} m exceeds max_grasp_width_m {max_grasp_width_m:.3} m")).with(width_m, max_grasp_width_m));
    }
    let reserve = (open_width_m - width_m - 2.0 * hard_error_m) / 2.0;
    if reserve.is_nan() || reserve < MIN_OPENING_RESERVE_M - JAW_EPS_M {
        return Err(Rejection::new("geometry", "OPENING", format!(
            "open jaws clear the error-shifted object by {reserve:.4} m per side (width {width_m:.4} m, hard error {hard_error_m:.4} m, opening {open_width_m:.4} m)"))
            .with(reserve, MIN_OPENING_RESERVE_M));
    }
    Ok(reserve)
}

/// The structured reason of a refusal: the candidate (None for a refusal before any
/// candidate), the phase, the code, the message, and the failing value with its bound.
#[derive(Debug, Clone, PartialEq)]
pub struct BestRefusal {
    pub candidate_id: Option<String>,
    pub phase: String,
    pub code: String,
    pub message: String,
    pub value: Option<f64>,
    pub bound: Option<f64>,
}

impl BestRefusal {
    pub fn from_rejection(candidate_id: Option<String>, r: &Rejection) -> Self {
        Self { candidate_id, phase: r.phase.to_string(), code: r.code.clone(), message: r.message.clone(), value: r.value, bound: r.bound }
    }

    pub fn to_json(&self) -> Value {
        json!({"candidate_id": self.candidate_id, "phase": self.phase, "code": self.code, "message": self.message,
            "value": self.value, "bound": self.bound})
    }
}

/// The jaw fit a legacy grasp was admitted with; the live measured width is checked against
/// the same predicate ([`JawAdmission::recheck`]).
#[derive(Debug, Clone, PartialEq)]
pub struct JawAdmission {
    pub candidate_id: String,
    pub width_m: f64,
    pub opening_reserve_m: f64,
    pub hard_error_m: f64,
    pub open_width_m: f64,
    pub max_grasp_width_m: f64,
    /// Whether the top-down candidate's whole-route preflight was part of the admission
    /// (false: jaw predicate only, see [`legacy_admission`]).
    pub full_route_checked: bool,
    /// Set when the planning check was deferred to the live measured width (and why).
    pub deferred: Option<String>,
}

impl JawAdmission {
    /// The jaw predicate on a fresh width with the admitted error and opening.
    pub fn recheck(&self, width_m: f64, max_grasp_width_m: f64) -> Result<f64, Rejection> {
        jaw_check(width_m, self.hard_error_m, self.open_width_m, max_grasp_width_m)
    }

    pub fn to_json(&self) -> Value {
        json!({"candidate_id": self.candidate_id, "width_m": self.width_m, "opening_reserve_m": self.opening_reserve_m,
            "hard_error_m": self.hard_error_m, "open_width_m": self.open_width_m, "max_grasp_width_m": self.max_grasp_width_m,
            "bound_m": MIN_OPENING_RESERVE_M, "full_route_checked": self.full_route_checked,
            "status": if self.deferred.is_some() { "deferred" } else { "checked" }, "deferred_reason": self.deferred})
    }
}

/// What the top-down evaluation needs besides the snapshot.
pub(super) struct Ctx<'a> {
    pub policy: &'a crate::model::GripperPolicy,
    pub limit: &'a crate::model::JointLimit,
    pub close_speed_limit: Option<f32>,
    pub has_gravity_model: bool,
}

/// A refused top-down candidate: the hypothesis when one was built, and why.
pub(super) struct Refused {
    pub hy: Option<Box<Hypothesis>>,
    pub rejection: Rejection,
}

impl Refused {
    fn bare(rejection: Rejection) -> Self { Self { hy: None, rejection } }
}

/// The jaw closing axis of the top-down grasp: across the object's short side when the
/// legacy grasp aligns J6 and the long axis is known, else the jaw axis of the look-down pose.
fn closing_axis(s: &Snapshot, t: &Target, grasp_r: &kinematics::Mat3) -> g::V2 {
    if s.setup.align_j6 && !t.axis_ambiguous {
        return g::perp2(t.axis);
    }
    let y = kinematics::column(grasp_r, 1);
    g::unit2([y[0], y[1]])
}

/// The top-down candidate's chord and hypothesis (cheap checks, jaw predicate included).
pub(super) fn top_down_hypothesis(s: &Snapshot, t: &Target, chord_index: usize, cheap: &mut CheapStats) -> Result<(Chord, Hypothesis), Refused> {
    let td = crate::grasp::top_down_pose(&s.input, &s.setup, &s.cam, &s.envelope, &s.box_norm).map_err(|e| {
        let api = crate::api::rest::ApiError::from(e);
        Refused::bare(Rejection::new("acquisition", api.code.as_deref().unwrap_or(&api.error), api.message))
    })?;
    let u = closing_axis(s, t, &td.grasp.r);
    let chord = candidates::chord_through(t, [td.target[0], td.target[1]], u, &s.tool).map_err(Refused::bare)?;
    let hy = candidates::make(s, t, &chord, chord_index, (DirClass::Vertical, 0.0, [1.0, 0.0]), false, false, TOP_DOWN_INDEX, cheap)
        .map_err(|b| { let (hy, rejection) = *b; Refused { hy: Some(Box::new(hy)), rejection } })?;
    Ok((chord, hy))
}

/// The candidate id of the top-down candidate.
pub fn top_down_id(s: &Snapshot) -> String { format!("{}-top", s.id) }

/// The top-down candidate through the whole-route preflight (6-DOF), with the reserved
/// budget. `acq`: (acquisition passed, its end joints).
pub(super) fn evaluate_top_down(
    s: &Snapshot,
    t: &Target,
    chord: &Chord,
    hy: &Hypothesis,
    ctx: &Ctx,
    acq: (bool, Option<[f64; 6]>),
    work: &mut Work,
) -> Result<(Candidate, AdaptiveRoute), Refused> {
    let deadline = Instant::now() + Duration::from_millis(TOP_DOWN_BUDGET_MS);
    let refused = |rejection: Rejection| Refused { hy: Some(Box::new(hy.clone())), rejection };
    let canonical = super::canonical_seed(s, t);
    let mut seeds = vec![s.input.q0];
    seeds.extend(acq.1);
    seeds.push(canonical);
    let q_g = feasibility::endpoint_ik(&s.input, &hy.grasp_pose(), &seeds, work).map_err(|e| refused(e.into_rejection("contact pose")))?;
    let mut pre_seeds = vec![q_g, s.input.q0];
    pre_seeds.extend(acq.1);
    pre_seeds.push(canonical);
    let q_pre = feasibility::endpoint_ik(&s.input, &hy.pregrasp_pose(), &pre_seeds, work).map_err(|e| refused(e.into_rejection("pre-grasp pose")))?;
    let over = || Instant::now() > deadline;
    let (checked, mut route) = super::preflight_candidate_from_start(
        s, t, hy, q_pre, acq.0, ctx.policy, ctx.limit, ctx.close_speed_limit, ctx.has_gravity_model, work, &over,
    ).map_err(refused)?;
    let rob = score::robustness(s, t, hy, &checked, work);
    let scores = score::score(s, t, chord, hy, &checked, &rob);
    let vis = super::visible(s, t, &hy.pregrasp_pose());
    let id = top_down_id(s);
    route.candidate_id = id.clone();
    route.robustness = rob;
    route.scores = scores;
    route.visible_at_pregrasp = vis;
    let cand = Candidate { id, hy: hy.clone(), chord: chord.clone(), checked, robustness: rob, scores, visible_at_pregrasp: vis };
    Ok((cand, route))
}

/// The refusal of a legacy grasp as [`ArmError::Detailed`] (422): `[phase] message` with the
/// structured refusal in the details.
fn legacy_refusal(_s: &Snapshot, _t: Option<&Target>, r: &Rejection, id: Option<String>) -> ArmError {
    let refusal = BestRefusal::from_rejection(id, r);
    let details = json!({"failing_phase": r.phase, "outcome": "infeasible", "best_refusal": refusal.to_json()});
    ArmError::Detailed { status: 422, code: r.code.clone(), message: format!("[{}] {}", r.phase, r.message), details: Box::new(details) }
}

/// Legacy admission: the jaw predicate on the top-down candidate's chord (the width along the
/// closing axis through the box-centre target, the snapshot's hard error and jaw opening).
///
/// The whole-route preflight of the top-down candidate is not part of the legacy admission
/// yet: with the default height interval its cheap checks (pad overlap above the table
/// clearance) refuse most legacy grasps the legacy descent performs today. The adaptive
/// strategy evaluates the top-down candidate first through the whole route.
///
/// Requests with only `box_norm` (no contour, oriented box or confident angle) are not refused
/// at planning: the box footprint is an upper bound of the width along the jaws, and the legacy
/// grasp measures the width at close range, where [`JawAdmission::recheck`] applies the same
/// predicate (`deferred` in the reply).
pub fn legacy_admission(pa: &PendingAdaptive) -> Result<JawAdmission, ArmError> {
    let s = &pa.snapshot;
    let id = top_down_id(s);
    let deferred = |hard_error_m: f64, why: String| JawAdmission {
        candidate_id: id.clone(), width_m: f64::NAN, opening_reserve_m: f64::NAN, hard_error_m,
        open_width_m: s.tool.open_width_m, max_grasp_width_m: s.tool.max_grasp_width_m, full_route_checked: false, deferred: Some(why),
    };
    let t = match snapshot::target(s) {
        Ok(t) => t,
        Err(m) if !has_footprint_evidence(s) => return Ok(deferred(fallback_hard_error(s), format!("box-only evidence: {m}"))),
        Err(m) => return Err(legacy_refusal(s, None, &Rejection::new("evidence", "NO_WIDTH_EVIDENCE", format!("evidence: {m}")), None)),
    };
    if t.source == snapshot::FootprintSource::Box {
        return Ok(deferred(t.hard_error_m, "box-only evidence: the width is checked on the measured width at close range".into()));
    }
    let refuse = |r: &Rejection| legacy_refusal(s, Some(&t), r, Some(id.clone()));
    let td = crate::grasp::top_down_pose(&s.input, &s.setup, &s.cam, &s.envelope, &s.box_norm)?;
    let u = closing_axis(s, &t, &td.grasp.r);
    let chord = candidates::chord_through(&t, [td.target[0], td.target[1]], u, &s.tool).map_err(|r| refuse(&r))?;
    let reserve = jaw_check(chord.width_m, t.hard_error_m, s.tool.open_width_m, s.tool.max_grasp_width_m).map_err(|r| refuse(&r))?;
    Ok(JawAdmission {
        candidate_id: id,
        width_m: chord.width_m,
        opening_reserve_m: reserve,
        hard_error_m: t.hard_error_m,
        open_width_m: s.tool.open_width_m,
        max_grasp_width_m: s.tool.max_grasp_width_m,
        full_route_checked: false,
        deferred: None,
    })
}

/// Whether the request carries footprint evidence beyond the box (contour, oriented box or a
/// confident angle).
fn has_footprint_evidence(s: &Snapshot) -> bool {
    let e = &s.evidence;
    e.contour_norm.is_some() || e.obb_corners_norm.is_some() || e.angle_conf.is_some_and(|c| c >= s.setup.align_min_angle_conf) && e.angle_deg.is_some()
}

/// The fallback hard error at the box-centre target when no footprint could be projected
/// (scale and hand-eye terms at the camera-to-target distance plus the contour term).
fn fallback_hard_error(s: &Snapshot) -> f64 {
    let pose_end = s.input.arm.fk(&s.input.q0);
    let Ok(td) = crate::grasp::top_down_pose(&s.input, &s.setup, &s.cam, &s.envelope, &s.box_norm) else { return f64::NAN };
    let dist = kinematics::norm(&kinematics::sub(&s.cam.origin(&pose_end), &td.target));
    let scale = snapshot::SCALE_ERROR * td.target[0].hypot(td.target[1]);
    let he = snapshot::HAND_EYE_ERROR_DEG.to_radians().tan() * dist;
    scale.max(he) + snapshot::CONTOUR_ERROR_PX * dist / s.cam.fx.min(s.cam.fy)
}
