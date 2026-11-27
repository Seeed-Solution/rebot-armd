//! Shared grasp admission of both strategies: the jaw predicate (dry run, live planning and
//! fresh-evidence revalidation) and the shared legacy top-down route builder + checker.
//!
//! One function ([`plan_legacy_top_down`]) builds and checks the legacy vertical route (box-centre
//! contact, `grasp_height_m`, `approach_height_m` pregrasp, camera-down / look-down approach,
//! transition, nominal descent, table clearance, and the shared jaw rule without `PAD_OVERLAP`).
//! Legacy admission, the adaptive `-top` candidate, and the nested `legacy` verdict in adaptive
//! replies all come from that single call.

use std::time::{Duration, Instant};

use serde_json::{json, Map, Value};

use super::candidates::{self, Chord, DirClass, Hypothesis, Rejection, MIN_OPENING_RESERVE_M};
use super::feasibility::{self, Work};
use super::geometry as g;
use super::margin::ErrorMargin;
use super::snapshot::{self, FootprintSource, Snapshot, Target};
use super::{score, AdaptiveRoute, Candidate};
use crate::api::rest::ApiError;
use crate::grasp::{
    table_violation, top_down_pose, transition_estimate, TopDown, TABLE_CLEARANCE_M,
};
use crate::kinematics::{self, Mat3};
use crate::motion::{self, GoalFrame, Speed};
use crate::supervisor::ArmError;

/// Hypothesis index of the top-down candidate (ids `<snapshot>-top`).
pub const TOP_DOWN_INDEX: usize = 99_999;
/// Wall budget reserved for the top-down candidate, outside the request's search budget.
pub const TOP_DOWN_BUDGET_MS: u64 = 1500;
/// Floating-point slack of the jaw predicate (an exact-threshold width passes).
const JAW_EPS_M: f64 = 1e-9;

/// The jaw predicate: `width` along the closing axis is finite, at most `max_grasp_width_m`
/// (when the cap is enabled), and `width + 2·hard_error + 2·MIN_OPENING_RESERVE_M <= open_width`.
/// The cap is optional independently of the opening reserve. Returns the per-side reserve
/// `(open_width - width - 2·hard_error) / 2`.
pub fn jaw_check(
    width_m: f64,
    hard_error_m: f64,
    open_width_m: f64,
    max_grasp_width_m: Option<f64>,
) -> Result<f64, Rejection> {
    if !width_m.is_finite() || width_m <= 0.0 {
        return Err(Rejection::new(
            "geometry",
            "NO_WIDTH_EVIDENCE",
            "no finite object width along the jaw closing axis",
        ));
    }
    if let Some(cap) = max_grasp_width_m {
        if !cap.is_finite() || width_m > cap + JAW_EPS_M {
            return Err(Rejection::new(
                "geometry",
                "TOO_WIDE",
                format!("chord {width_m:.3} m exceeds max_grasp_width_m {cap:.3} m"),
            )
            .with(width_m, cap));
        }
    }
    let reserve = (open_width_m - width_m - 2.0 * hard_error_m) / 2.0;
    if !reserve.is_finite() || reserve < MIN_OPENING_RESERVE_M - JAW_EPS_M {
        return Err(Rejection::new(
            "geometry",
            "OPENING",
            format!(
                "open jaws clear the error-shifted object by {reserve:.4} m per side (width {width_m:.4} m, hard error {hard_error_m:.4} m, opening {open_width_m:.4} m)"
            ),
        )
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
        Self {
            candidate_id,
            phase: r.phase.to_string(),
            code: r.code.clone(),
            message: r.message.clone(),
            value: r.value,
            bound: r.bound,
        }
    }

    pub fn to_json(&self) -> Value {
        json!({
            "candidate_id": self.candidate_id,
            "phase": self.phase,
            "code": self.code,
            "message": self.message,
            "value": self.value,
            "bound": self.bound,
        })
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
    /// Whether the top-down candidate's full adaptive preflight was part of the admission
    /// (false for legacy vertical admission).
    pub full_route_checked: bool,
    /// Set when the planning check was deferred to the live measured width (and why).
    pub deferred: Option<String>,
    /// The error budget behind `hard_error_m` (active fallback, reported measured budget).
    pub error_margin: ErrorMargin,
}

impl JawAdmission {
    /// The jaw predicate on a fresh width with the admitted error, actual insertion opening and
    /// optional width cap.
    pub fn recheck_opening(
        &self,
        width_m: f64,
        open_width_m: f64,
        max_grasp_width_m: Option<f64>,
    ) -> Result<f64, Rejection> {
        jaw_check(width_m, self.hard_error_m, open_width_m, max_grasp_width_m)
    }

    /// The jaw predicate on a fresh width with the admitted error and full opening.
    pub fn recheck(&self, width_m: f64, max_grasp_width_m: Option<f64>) -> Result<f64, Rejection> {
        self.recheck_opening(width_m, self.open_width_m, max_grasp_width_m)
    }

    pub fn to_json(&self) -> Value {
        json!({
            "candidate_id": self.candidate_id,
            "width_m": self.width_m,
            "opening_reserve_m": self.opening_reserve_m,
            "hard_error_m": self.hard_error_m,
            "open_width_m": self.open_width_m,
            "max_grasp_width_m": self.max_grasp_width_m,
            "bound_m": MIN_OPENING_RESERVE_M,
            "full_route_checked": self.full_route_checked,
            "status": if self.deferred.is_some() { "deferred" } else { "checked" },
            "deferred_reason": self.deferred,
            "error_margin": self.error_margin.to_json(),
        })
    }
}

/// The nested `legacy` verdict in adaptive dry-run replies, built directly from the same
/// [`plan_legacy_top_down`] evaluation that drives both legacy admission and adaptive `-top`.
#[derive(Debug, Clone, PartialEq)]
pub struct LegacyVerdict {
    pub reachable: bool,
    pub code: Option<String>,
    pub message: Option<String>,
    pub admission: Option<Map<String, Value>>,
}

impl LegacyVerdict {
    pub fn from_ok(tr: &TopDownRoute) -> Self {
        let mut m = Map::new();
        m.insert("jaw".into(), tr.jaw.to_json());
        Self { reachable: true, code: None, message: None, admission: Some(m) }
    }

    pub fn from_err(e: ArmError) -> Self {
        let admission = match &e {
            ArmError::Detailed { details, .. } => {
                let mut m = Map::new();
                for k in ["best_refusal", "failing_phase", "error_margin"] {
                    if let Some(v) = details.get(k) {
                        m.insert(k.into(), v.clone());
                    }
                }
                (!m.is_empty()).then_some(m)
            }
            _ => None,
        };
        let api = ApiError::from(e);
        Self {
            reachable: false,
            code: api.code.or(Some(api.error)),
            message: Some(api.message),
            admission,
        }
    }

    pub fn to_json(&self) -> Value {
        let mut lv = json!({
            "reachable": self.reachable,
            "code": self.code,
            "message": self.message,
        });
        for (k, x) in self.admission.iter().flatten() {
            lv[k] = x.clone();
        }
        lv
    }
}

/// Verified legacy top-down route produced by [`plan_legacy_top_down`].
#[derive(Debug, Clone)]
pub struct TopDownRoute {
    pub td: TopDown,
    pub approach_duration: f64,
    pub transition_duration: f64,
    pub q_acq: [f64; 6],
    pub q_tool: [f64; 6],
    pub q_grasp: [f64; 6],
    /// Actual closing orientation (with J6 aligned to short side when `align_j6` is on, the long
    /// axis is known, and the J6 turn fits inside J6's planning range; else `td.grasp.r`).
    pub grasp_r: Mat3,
    pub closing_axis: g::V2,
    pub jaw: JawAdmission,
    /// Present only when a box-only request failed the upper-bound jaw check and was deferred for
    /// legacy live measurement (adaptive refuses `-top` with this rejection instead of deferring).
    pub box_refusal: Option<Rejection>,
}

/// Outcome of the single shared [`plan_legacy_top_down`] call.
pub struct TopDownOutcome {
    pub route: Result<TopDownRoute, ArmError>,
    /// Result of the legacy approach (acquisition) prefix, reused by adaptive without replanning.
    pub acq: Result<[f64; 6], Rejection>,
    /// Structured rejection of the legacy top-down route (when `route` is `Err`).
    pub rejection: Option<Rejection>,
}

/// What the top-down adaptive preflight needs besides the snapshot.
pub(super) struct Ctx<'a> {
    pub policy: &'a crate::model::GripperPolicy,
    pub limit: &'a crate::model::JointLimit,
    pub close_speed_limit: Option<f32>,
    pub has_gravity_model: bool,
}

/// The candidate id of the top-down candidate.
pub fn top_down_id(s: &Snapshot) -> String {
    format!("{}-top", s.id)
}

fn q6_of(q: &[f32; 7]) -> [f64; 6] {
    std::array::from_fn(|i| f64::from(q[i]))
}

fn arm_error_to_rejection(phase: &'static str, e: &ArmError) -> Rejection {
    match e {
        ArmError::Motion(me) => feasibility::motion_rejection(phase, me),
        ArmError::Detailed { code, message, details, .. } => {
            if let Some(br) = details.get("best_refusal") {
                let mut r = Rejection::new(
                    phase,
                    br.get("code").and_then(Value::as_str).unwrap_or(code),
                    br.get("message").and_then(Value::as_str).unwrap_or(message),
                );
                r.value = br.get("value").and_then(Value::as_f64);
                r.bound = br.get("bound").and_then(Value::as_f64);
                return r;
            }
            Rejection::new(phase, code, message)
        }
        other => Rejection::new(phase, "INVALID_COMMAND", format!("{other}")),
    }
}

/// Resolve the actual top-down closing orientation and joint targets after checking whether the
/// requested `align_j6` roll fits inside the roll joint's planning limits (matching `align_turn`).
fn resolve_top_down_alignment(
    s: &Snapshot,
    t: Option<&Target>,
    grasp_r: &Mat3,
    mut q_tool: [f64; 6],
    mut q_grasp: [f64; 6],
) -> (Mat3, g::V2, [f64; 6], [f64; 6]) {
    let mut r = *grasp_r;
    if let Some(t) = t.filter(|t| s.setup.align_j6 && !t.axis_ambiguous) {
        let arm_dof = s.input.arm.arm_dof();
        if arm_dof >= 6 {
            let roll_idx = arm_dof - 1;
            let sign = s.input.arm.joint_map().get(roll_idx).map_or(1.0, |m| m.sign);
            let lim = &s.input.limits[roll_idx];
            if let Some((lo, hi)) = lim.effective_planning_range() {
                let (lo, hi) = (f64::from(lo) + 0.1, f64::from(hi) - 0.1);
                let room = |turn: f64| {
                    let qt = q_tool[roll_idx] + sign * turn;
                    let qg = q_grasp[roll_idx] + sign * turn;
                    qt >= lo && qt <= hi && qg >= lo && qg <= hi
                };
                let jaw = kinematics::column(&r, 1);
                let t0 = crate::grasp::roll_across_long_side(jaw, [t.axis[0], t.axis[1], 0.0]);
                let pi = std::f64::consts::PI;
                if let Some(turn) = [t0, t0 + pi, t0 - pi]
                    .into_iter()
                    .filter(|turn| room(*turn))
                    .min_by(|a, b| a.abs().total_cmp(&b.abs()))
                {
                    if turn.abs() >= 1e-3 {
                        r = kinematics::mat_mul(&r, &kinematics::axis_r(&[1.0, 0.0, 0.0], turn));
                        q_tool[roll_idx] += sign * turn;
                        q_grasp[roll_idx] += sign * turn;
                    }
                }
            }
        }
    }
    let y = kinematics::column(&r, 1);
    (r, g::unit2([y[0], y[1]]), q_tool, q_grasp)
}

/// The refusal of a legacy grasp as [`ArmError::Detailed`] (422): `[phase] message` with the
/// structured refusal in the details.
fn legacy_refusal(s: &Snapshot, t: Option<&Target>, r: &Rejection, id: Option<String>) -> ArmError {
    let refusal = BestRefusal::from_rejection(id, r);
    let margin = t.map(|t| t.margin.clone()).unwrap_or_else(|| fallback_margin(s));
    let details = json!({
        "failing_phase": r.phase,
        "outcome": "infeasible",
        "best_refusal": refusal.to_json(),
        "error_margin": margin.to_json(),
    });
    ArmError::Detailed {
        status: 422,
        code: r.code.clone(),
        message: format!("[{}] {}", r.phase, r.message),
        details: Box::new(details),
    }
}

/// The single shared builder and checker of the legacy vertical route (box-centre contact,
/// `grasp_height_m`, `approach_height_m` pregrasp, camera-down / look-down approach, transition,
/// nominal descent, table clearance, and shared jaw rule without `PAD_OVERLAP`).
///
/// Called by legacy `grasp::plan()`, by adaptive `select_adaptive_route()` for the `-top`
/// candidate, and directly for the nested `legacy` verdict in adaptive dry-run replies.
pub fn plan_legacy_top_down(
    s: &Snapshot,
    target_res: Option<Result<&Target, &str>>,
    work: &mut Work,
) -> TopDownOutcome {
    let setup = &s.setup;
    let is_5dof = s.input.arm.arm_dof() == 5;
    let pose_end = s.input.arm.fk(&s.input.q0);
    let td = match top_down_pose(&s.input, setup, &s.cam, &s.envelope, &s.box_norm) {
        Ok(td) => td,
        Err(e) => {
            let r = arm_error_to_rejection("acquisition", &e);
            return TopDownOutcome { route: Err(e), acq: Err(r.clone()), rejection: Some(r) };
        }
    };
    let TopDown { target, goal, tool, camera_down, .. } = td;
    // Envelope stays above the table on the approach and (camera-down) the transition.
    if camera_down {
        let tool_end = motion::tcp_to_end(&tool, &s.input.tcp_offset);
        if let Some(e) = table_violation(&goal, &tool_end, &s.envelope, setup.table_z_m) {
            let r = feasibility::motion_rejection("acquisition", &e);
            return TopDownOutcome { route: Err(e.into()), acq: Err(r.clone()), rejection: Some(r) };
        }
    }
    let goal_end = if camera_down { goal } else { motion::tcp_to_end(&goal, &s.input.tcp_offset) };
    if let Some(e) = table_violation(&pose_end, &goal_end, &s.envelope, setup.table_z_m) {
        let r = feasibility::motion_rejection("acquisition", &e);
        return TopDownOutcome { route: Err(e.into()), acq: Err(r.clone()), rejection: Some(r) };
    }
    let mut cur = s.input.clone();
    cur.j7 = s.open_rad;
    cur.goal_frame = if camera_down { GoalFrame::EndLink } else { GoalFrame::Tcp };
    let before = motion::ik_iterations();
    work.path_checks += 1;
    let path = match motion::plan_cartesian_ex(
        &cur,
        goal,
        Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS },
        is_5dof,
    ) {
        Ok(p) => {
            work.ik_iterations += motion::ik_iterations() - before;
            work.path_samples += p.q.len();
            p
        }
        Err(e) => {
            work.ik_iterations += motion::ik_iterations() - before;
            let r = feasibility::motion_rejection("acquisition", &e);
            return TopDownOutcome { route: Err(e.into()), acq: Err(r.clone()), rejection: Some(r) };
        }
    };
    if let Err(r) = feasibility::grasp_table_clear(s, "acquisition", &path) {
        let e = crate::backlash::check_table_clear(
            &s.input.arm,
            &s.envelope,
            setup.table_z_m + TABLE_CLEARANCE_M,
            path.q.iter().map(|q| q.as_slice()),
        )
        .unwrap_err()
        .into_motion_error();
        return TopDownOutcome { route: Err(e.into()), acq: Err(r.clone()), rejection: Some(r) };
    }
    let q_acq = q6_of(&path.last());
    cur.q0 = q_acq;
    cur.goal_frame = GoalFrame::Tcp;
    let (q_tool, transition_duration) = if camera_down {
        let before = motion::ik_iterations();
        work.path_checks += 1;
        let trans_path = match motion::plan_cartesian_ex(
            &cur,
            tool,
            Speed { linear: setup.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS },
            is_5dof,
        ) {
            Ok(p) => {
                work.ik_iterations += motion::ik_iterations() - before;
                work.path_samples += p.q.len();
                p
            }
            Err(e) => {
                work.ik_iterations += motion::ik_iterations() - before;
                let r = feasibility::motion_rejection("transition", &e);
                return TopDownOutcome { route: Err(e.into()), acq: Ok(q_acq), rejection: Some(r) };
            }
        };
        if let Err(r) = feasibility::grasp_table_clear(s, "transition", &trans_path) {
            let e = crate::backlash::check_table_clear(
                &s.input.arm,
                &s.envelope,
                setup.table_z_m + TABLE_CLEARANCE_M,
                trans_path.q.iter().map(|q| q.as_slice()),
            )
            .unwrap_err()
            .into_motion_error();
            return TopDownOutcome { route: Err(e.into()), acq: Ok(q_acq), rejection: Some(r) };
        }
        let qt = q6_of(&trans_path.last());
        cur.q0 = qt;
        (qt, transition_estimate(&goal, &target, setup))
    } else {
        (q_acq, 0.0)
    };
    let before = motion::ik_iterations();
    work.path_checks += 1;
    let descend_path = match motion::plan_cartesian_ex(
        &cur,
        td.grasp,
        Speed { linear: setup.descend_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS },
        is_5dof,
    ) {
        Ok(p) => {
            work.ik_iterations += motion::ik_iterations() - before;
            work.path_samples += p.q.len();
            p
        }
        Err(e) => {
            work.ik_iterations += motion::ik_iterations() - before;
            let r = feasibility::motion_rejection("insertion", &e);
            return TopDownOutcome { route: Err(e.into()), acq: Ok(q_acq), rejection: Some(r) };
        }
    };
    let q_grasp = q6_of(&descend_path.last());

    // Target footprint and shared jaw rule on the exact box-centre contact along the actual
    // closing axis (checking whether `align_j6` fits in J6 planning limits).
    let owned_target;
    let t_res: Result<&Target, String> = match target_res {
        Some(Ok(t)) => Ok(t),
        Some(Err(m)) => Err(m.to_string()),
        None => match snapshot::target(s) {
            Ok(t) => {
                owned_target = t;
                Ok(&owned_target)
            }
            Err(m) => Err(m),
        },
    };
    let id = top_down_id(s);
    let make_deferred = |margin: ErrorMargin, why: String| JawAdmission {
        candidate_id: id.clone(),
        width_m: f64::NAN,
        opening_reserve_m: f64::NAN,
        hard_error_m: margin.hard_error_m,
        open_width_m: s.tool.open_width_m,
        max_grasp_width_m: s.tool.max_grasp_width_m,
        full_route_checked: false,
        deferred: Some(why),
        error_margin: margin,
    };
    let t = match t_res {
        Ok(t) => t,
        Err(m) if !has_footprint_evidence(s) => {
            let (grasp_r, closing_axis, q_tool, q_grasp) =
                resolve_top_down_alignment(s, None, &td.grasp.r, q_tool, q_grasp);
            let rej = Rejection::new("evidence", "NO_WIDTH_EVIDENCE", format!("evidence: {m}"));
            let jaw = make_deferred(fallback_margin(s), format!("box-only evidence: {m}"));
            return TopDownOutcome {
                route: Ok(TopDownRoute {
                    td,
                    approach_duration: path.duration,
                    transition_duration,
                    q_acq,
                    q_tool,
                    q_grasp,
                    grasp_r,
                    closing_axis,
                    jaw,
                    box_refusal: Some(rej),
                }),
                acq: Ok(q_acq),
                rejection: None,
            };
        }
        Err(m) => {
            let r = Rejection::new("evidence", "NO_WIDTH_EVIDENCE", format!("evidence: {m}"));
            let e = legacy_refusal(s, None, &r, None);
            return TopDownOutcome { route: Err(e), acq: Ok(q_acq), rejection: Some(r) };
        }
    };
    let (grasp_r, closing_axis, q_tool, q_grasp) =
        resolve_top_down_alignment(s, Some(t), &td.grasp.r, q_tool, q_grasp);
    let p_contact = [td.target[0], td.target[1]];
    let jaw_res = candidates::cut_width_near(&t.poly, p_contact, closing_axis)
        .ok_or_else(|| {
            Rejection::new(
                "geometry",
                "NO_CROSS_SECTION",
                "the closing line through the top-down target misses the footprint",
            )
        })
        .and_then(|w| {
            jaw_check(w, t.hard_error_m, s.tool.open_width_m, s.tool.max_grasp_width_cap)
                .map(|reserve| (w, reserve))
        });
    match (t.source, jaw_res) {
        (_, Ok((width_m, opening_reserve_m))) => {
            let jaw = JawAdmission {
                candidate_id: id,
                width_m,
                opening_reserve_m,
                hard_error_m: t.hard_error_m,
                open_width_m: s.tool.open_width_m,
                max_grasp_width_m: s.tool.max_grasp_width_m,
                full_route_checked: false,
                deferred: None,
                error_margin: t.margin.clone(),
            };
            TopDownOutcome {
                route: Ok(TopDownRoute {
                    td,
                    approach_duration: path.duration,
                    transition_duration,
                    q_acq,
                    q_tool,
                    q_grasp,
                    grasp_r,
                    closing_axis,
                    jaw,
                    box_refusal: None,
                }),
                acq: Ok(q_acq),
                rejection: None,
            }
        }
        (FootprintSource::Box, Err(rej)) => {
            // Axis-aligned box is an upper bound: defer legacy admission to live close-range
            // measurement (which must resolve before descent or return NO_WIDTH_EVIDENCE).
            let jaw = make_deferred(
                t.margin.clone(),
                "box-only evidence: the width is checked on the measured width at close range".into(),
            );
            TopDownOutcome {
                route: Ok(TopDownRoute {
                    td,
                    approach_duration: path.duration,
                    transition_duration,
                    q_acq,
                    q_tool,
                    q_grasp,
                    grasp_r,
                    closing_axis,
                    jaw,
                    box_refusal: Some(rej),
                }),
                acq: Ok(q_acq),
                rejection: None,
            }
        }
        (_, Err(rej)) => {
            let e = legacy_refusal(s, Some(t), &rej, Some(id));
            TopDownOutcome { route: Err(e), acq: Ok(q_acq), rejection: Some(rej) }
        }
    }
}

/// Build the `-top` adaptive candidate directly from the verified [`TopDownRoute`] (exact legacy
/// contact point, height, pregrasp, and closing orientation; no `PAD_OVERLAP` check) and run
/// Phase B whole-route preflight (`separation`, `lift`, carried-object keepout, payload torque,
/// `withdrawal`, parent budget) within the reserved top-down deadline.
#[allow(clippy::too_many_arguments)]
pub(super) fn evaluate_top_down_from_legacy(
    s: &Snapshot,
    t: &Target,
    tr: &TopDownRoute,
    chord_index: usize,
    ctx: &Ctx,
    acq_ok: bool,
    work: &mut Work,
    out_of_budget: &dyn Fn() -> bool,
) -> Result<(Chord, Candidate, AdaptiveRoute), Box<(Option<Hypothesis>, Rejection)>> {
    let contact_xy = [tr.td.target[0], tr.td.target[1]];
    let tangent = g::perp2(tr.closing_axis);
    let (lo, hi) = g::extent_along(&t.poly, t.centroid, tangent);
    let len = (hi - lo).max(1e-6);
    let d = g::cross2(tr.closing_axis, g::sub2(t.centroid, contact_xy)).abs();
    let moment_q = 1.0 - (d / (len / 2.0)).min(1.0);
    let chord = Chord {
        station: 0.5,
        axis_index: 0,
        centre: contact_xy,
        dir: tr.closing_axis,
        width_m: tr.jaw.width_m,
        tangent,
        opposition: 1.0,
        moment_q,
        pad_support: 1.0,
    };
    let tool = &s.tool;
    let finger_low = candidates::gripper_lattice(tool, tool.open_width_m)
        .iter()
        .map(|p| kinematics::mat_vec(&tr.grasp_r, p)[2])
        .fold(0.0, f64::min);
    let env_low = candidates::lowest_end(&tr.grasp_r, &s.envelope, &s.input.tcp_offset);
    let gz = tr.td.grasp.p[2];
    let top_lo = t.table_z + t.height[0];
    let pad_overlap_m = (top_lo - gz).clamp(0.0, tool.pad_length_m);
    let insertion_m = (tr.td.tool.p[2] - gz).max(0.0);
    let hy = Hypothesis {
        index: TOP_DOWN_INDEX,
        chord: chord_index,
        class: DirClass::Vertical,
        tilt_deg: 0.0,
        deep: false,
        half_turn: false,
        a: [0.0, 0.0, -1.0],
        r: tr.grasp_r,
        g: tr.td.grasp.p,
        insertion_m,
        width_m: tr.jaw.width_m,
        pad_overlap_m,
        opening_reserve_m: tr.jaw.opening_reserve_m,
        table_clearance_m: gz + finger_low.min(env_low) - t.table_z,
        jaw_elevation_deg: 0.0,
    };
    if let Some(rej) = &tr.box_refusal {
        return Err(Box::new((Some(hy), rej.clone())));
    }
    if out_of_budget() {
        return Err(Box::new((
            Some(hy),
            Rejection::new("budget", "BUDGET_EXHAUSTED", "top-down reservation exceeded"),
        )));
    }
    let (checked, mut route) = super::preflight_candidate_from_start(
        s,
        t,
        &hy,
        &chord,
        tr.q_tool,
        acq_ok,
        ctx.policy,
        ctx.limit,
        ctx.close_speed_limit,
        ctx.has_gravity_model,
        work,
        out_of_budget,
    )
    .map_err(|r| Box::new((Some(hy.clone()), r)))?;
    let rob = score::robustness(s, t, &hy, &checked, work);
    if out_of_budget() {
        return Err(Box::new((
            Some(hy),
            Rejection::new("budget", "BUDGET_EXHAUSTED", "top-down reservation exceeded"),
        )));
    }
    let scores = score::score(s, t, &chord, &hy, &checked, &rob);
    let vis = super::visible(s, t, &hy.pregrasp_pose());
    let id = top_down_id(s);
    route.candidate_id = id.clone();
    route.robustness = rob;
    route.scores = scores;
    route.visible_at_pregrasp = vis;
    let cand = Candidate {
        id,
        hy,
        chord: chord.clone(),
        checked,
        robustness: rob,
        scores,
        visible_at_pregrasp: vis,
    };
    Ok((chord, cand, route))
}

/// Top-down budget check closure helper (deadline + cooperative cancellation).
pub(super) fn top_down_budget_fn<'a>(
    s: &'a Snapshot,
    deadline: Instant,
) -> impl Fn() -> bool + 'a {
    move || {
        Instant::now() > deadline
            || s.budget
                .cancel
                .as_ref()
                .is_some_and(|c| c.load(std::sync::atomic::Ordering::Relaxed))
    }
}

/// Top-down deadline from `start`.
pub(super) fn top_down_deadline(start: Instant) -> Instant {
    start + Duration::from_millis(TOP_DOWN_BUDGET_MS)
}

/// Whether the request carries footprint evidence beyond the box (contour, oriented box or a
/// confident angle).
fn has_footprint_evidence(s: &Snapshot) -> bool {
    let e = &s.evidence;
    e.contour_norm.is_some()
        || e.obb_corners_norm.is_some()
        || e.angle_conf.is_some_and(|c| c >= s.setup.align_min_angle_conf)
            && e.angle_deg.is_some()
}

/// The margin at the box-centre target when no footprint could be projected (scale and
/// hand-eye terms at the camera-to-target distance plus the contour term; NaN hard error when
/// the box centre ray misses the grasp plane).
fn fallback_margin(s: &Snapshot) -> ErrorMargin {
    let pose_end = s.input.arm.fk(&s.input.q0);
    let Ok(td) = top_down_pose(&s.input, &s.setup, &s.cam, &s.envelope, &s.box_norm) else {
        return super::margin::margin(&s.calib, f64::NAN, f64::NAN, f64::NAN, f64::NAN);
    };
    let o = s.cam.origin(&pose_end);
    let dist = kinematics::norm(&kinematics::sub(&o, &td.target));
    let scale = snapshot::SCALE_ERROR * td.target[0].hypot(td.target[1]);
    let he = snapshot::HAND_EYE_ERROR_DEG.to_radians().tan() * dist;
    let contour = snapshot::CONTOUR_ERROR_PX * dist / s.cam.fx.min(s.cam.fy);
    let cam_tan =
        (o[0] - td.target[0]).hypot(o[1] - td.target[1]) / (o[2] - td.target[2]).abs().max(1e-3);
    super::margin::margin(&s.calib, scale.max(he) + contour, contour, dist, cam_tan)
}
