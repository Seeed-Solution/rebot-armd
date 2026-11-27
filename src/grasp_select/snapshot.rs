//! The immutable planning snapshot of an adaptive grasp selection: the target evidence of
//! one detection bound to its capture pose, the camera, table, tool, opening, workspace and
//! the measured start joints. Built under the supervisor lock, planned without it.

use serde_json::Value;

use super::geometry::{self as g, V2};
use crate::grasp::{self, CamModel, GraspSetup, PendingGrasp};
use crate::kinematics::{self, Pose, Vec3};
use crate::motion::{GoalFrame, MotionError, PlanInput};
use crate::supervisor::{ArmError, SupervisorInner};

/// Evidence of the target beyond the request's `box_norm`, from the request's `evidence`
/// object (fields of the same detection as `box_norm`, rebot-vision names).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Evidence {
    /// Exterior contour, normalised like `box_norm` (validated by [`grasp::parse_contour`]).
    pub contour_norm: Option<Vec<V2>>,
    /// Reason a supplied contour was not used.
    pub contour_error: Option<String>,
    /// Oriented box corners, normalised.
    pub obb_corners_norm: Option<[V2; 4]>,
    pub angle_deg: Option<f64>,
    pub angle_conf: Option<f64>,
    /// Object height interval above the table (m).
    pub height_m: Option<[f64; 2]>,
    /// Upper bound of the payload mass (kg).
    pub payload_kg: Option<f64>,
}

fn num(v: &Value, key: &str) -> Result<Option<f64>, String> {
    match v.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(x) => x.as_f64().filter(|f| f.is_finite()).map(Some).ok_or_else(|| format!("evidence.{key} must be a finite number")),
    }
}

/// `evidence`: `{contour?, obb_corners_norm?, angle_deg?, angle_conf?, height_m?: h | [lo, hi], payload_kg?}`.
/// A malformed contour is not an error in adaptive mode: it is reported and the next evidence is used.
pub fn parse_evidence(body: &Value) -> Result<Evidence, String> {
    parse_evidence_mode(body, true)
}

/// Legacy evidence parser: validates `evidence` (when present) and all fields used for jaw
/// admission (`contour`, `obb_corners_norm`, `angle_deg`, `angle_conf`), while keeping
/// compatibility for unused adaptive-only sibling fields (`height_m`, `payload_kg`) so a
/// malformed sibling never erases valid footprint evidence.
pub fn parse_evidence_for_legacy(body: &Value) -> Result<Evidence, String> {
    parse_evidence_mode(body, false)
}

fn parse_height_m(e: &Value) -> Result<Option<[f64; 2]>, String> {
    match e.get("height_m") {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Array(a)) if a.len() == 2 => {
            let (lo, hi) = (a[0].as_f64(), a[1].as_f64());
            match (lo, hi) {
                (Some(lo), Some(hi)) if lo > 0.0 && hi >= lo && hi <= 0.5 => Ok(Some([lo, hi])),
                _ => Err("evidence.height_m must be h or [lo, hi] with 0 < lo <= hi <= 0.5 m".into()),
            }
        }
        Some(v) => match v.as_f64() {
            Some(h) if h > 0.0 && h <= 0.5 => Ok(Some([h, h])),
            _ => Err("evidence.height_m must be h or [lo, hi] with 0 < lo <= hi <= 0.5 m".into()),
        },
    }
}

fn parse_payload_kg(e: &Value) -> Result<Option<f64>, String> {
    let p = num(e, "payload_kg")?;
    if p.is_some_and(|m| !(0.0..=5.0).contains(&m)) {
        return Err("evidence.payload_kg must be in [0, 5] kg".into());
    }
    Ok(p)
}

fn parse_evidence_mode(body: &Value, strict_adaptive: bool) -> Result<Evidence, String> {
    let Some(e) = body.get("evidence").filter(|e| !e.is_null()) else { return Ok(Evidence::default()) };
    if !e.is_object() { return Err("evidence must be an object".into()); }
    let mut out = Evidence::default();
    if let Some(c) = e.get("contour").filter(|c| !c.is_null()) {
        match grasp::parse_contour(c, false, [1.0, 1.0]) {
            Ok(p) => out.contour_norm = Some(p),
            Err(why) if strict_adaptive => out.contour_error = Some(why),
            Err(why) => return Err(format!("evidence.contour: {why}")),
        }
    }
    if let Some(o) = e.get("obb_corners_norm").filter(|o| !o.is_null()) {
        let bad = || "evidence.obb_corners_norm must be 4 [x, y] pairs in [0, 1]".to_string();
        let a = o.as_array().filter(|a| a.len() == 4).ok_or_else(bad)?;
        let mut c = [[0.0; 2]; 4];
        for (dst, v) in c.iter_mut().zip(a) {
            let q = v.as_array().filter(|q| q.len() == 2).ok_or_else(bad)?;
            for (d, x) in dst.iter_mut().zip(q) {
                *d = x.as_f64().filter(|x| (-0.01..=1.01).contains(x)).ok_or_else(bad)?.clamp(0.0, 1.0);
            }
        }
        if g::signed_area(&c).abs() < 1e-8 { return Err("evidence.obb_corners_norm has no area".into()); }
        out.obb_corners_norm = Some(c);
    }
    out.angle_deg = num(e, "angle_deg")?.map(|a| (a + 90.0).rem_euclid(180.0) - 90.0);
    out.angle_conf = num(e, "angle_conf")?.map(|c| c.clamp(0.0, 1.0));
    if strict_adaptive {
        out.height_m = parse_height_m(e)?;
        out.payload_kg = parse_payload_kg(e)?;
    } else {
        out.height_m = parse_height_m(e).ok().flatten();
        out.payload_kg = parse_payload_kg(e).ok().flatten();
    }
    Ok(out)
}

/// Approach side preference of `pose_preferences`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Side { #[default] Any, Outward, Inward, Lateral }

/// Request `pose_preferences` (soft unless under `hard`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosePreferences {
    /// Score penalty per radian of tilt from vertical (0..2, default [`DEFAULT_VERTICAL_WEIGHT`]).
    pub vertical_weight: f64,
    pub approach_side: Side,
    /// Tilt beyond this adds [`OVER_TILT_PENALTY_PER_RAD`] per radian.
    pub preferred_max_tilt_deg: Option<f64>,
    /// `hard.max_tilt_deg`: candidates tilted more are not considered.
    pub hard_max_tilt_deg: Option<f64>,
}

pub const DEFAULT_VERTICAL_WEIGHT: f64 = 0.3;
pub const OVER_TILT_PENALTY_PER_RAD: f64 = 0.5;
pub const SIDE_MISMATCH_PENALTY: f64 = 0.2;

impl Default for PosePreferences {
    fn default() -> Self { Self { vertical_weight: DEFAULT_VERTICAL_WEIGHT, approach_side: Side::Any, preferred_max_tilt_deg: None, hard_max_tilt_deg: None } }
}

pub fn parse_preferences(body: &Value) -> Result<PosePreferences, String> {
    let mut p = PosePreferences::default();
    let Some(o) = body.get("pose_preferences").filter(|o| !o.is_null()) else { return Ok(p) };
    let o = o.as_object().ok_or("pose_preferences must be an object")?;
    for k in o.keys() {
        if !["vertical_weight", "approach_side", "preferred_max_tilt_deg", "hard"].contains(&k.as_str()) {
            return Err(format!("pose_preferences: unknown field {k}"));
        }
    }
    let tilt = |v: &Value, what: &str| v.as_f64().filter(|t| (0.0..=90.0).contains(t)).ok_or(format!("{what} must be in [0, 90] deg"));
    if let Some(v) = o.get("vertical_weight").filter(|v| !v.is_null()) {
        p.vertical_weight = v.as_f64().filter(|w| (0.0..=2.0).contains(w)).ok_or("pose_preferences.vertical_weight must be in [0, 2]")?;
    }
    if let Some(v) = o.get("approach_side").filter(|v| !v.is_null()) {
        p.approach_side = match v.as_str() {
            Some("any") => Side::Any, Some("outward") => Side::Outward, Some("inward") => Side::Inward, Some("lateral") => Side::Lateral,
            _ => return Err("pose_preferences.approach_side must be \"any\", \"outward\", \"inward\" or \"lateral\"".into()),
        };
    }
    if let Some(v) = o.get("preferred_max_tilt_deg").filter(|v| !v.is_null()) {
        p.preferred_max_tilt_deg = Some(tilt(v, "pose_preferences.preferred_max_tilt_deg")?);
    }
    if let Some(h) = o.get("hard").filter(|h| !h.is_null()) {
        let h = h.as_object().ok_or("pose_preferences.hard must be an object")?;
        for k in h.keys() { if k != "max_tilt_deg" { return Err(format!("pose_preferences.hard: unknown field {k}")); } }
        if let Some(v) = h.get("max_tilt_deg").filter(|v| !v.is_null()) {
            p.hard_max_tilt_deg = Some(tilt(v, "pose_preferences.hard.max_tilt_deg")?);
        }
    }
    Ok(p)
}

/// Finger, pad and palm volumes in the jaw frame (x along the approach, tips at x = 0, y the
/// closing direction, z = x × y). armd's tool definitions name finger geometries but carry no
/// commissioned volumes yet: these are assumed B601-DM values ([`ToolVolumes::source`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToolVolumes {
    pub finger_length_m: f64,
    pub finger_thickness_m: f64,
    /// Finger extent along jaw z.
    pub finger_width_m: f64,
    /// Pad length back from the tips.
    pub pad_length_m: f64,
    /// Palm block behind the finger roots (depth along -x).
    pub palm_depth_m: f64,
    /// Jaw opening at the gripper policy's open position.
    pub open_width_m: f64,
    /// Widest object the grasp accepts (`max_grasp_width_m`, clamped to the calibrated opening;
    /// 0.0 when the cap is disabled on an uncalibrated policy).
    pub max_grasp_width_m: f64,
    /// Optional width cap enforced independently of the opening reserve (`None`: cap disabled).
    pub max_grasp_width_cap: Option<f64>,
    pub source: &'static str,
}

impl ToolVolumes {
    pub fn b601dm_assumed(open_width_m: f64, max_grasp_width_m: f64) -> Self {
        Self::b601dm_assumed_with_cap(open_width_m, max_grasp_width_m, (max_grasp_width_m > 0.0).then_some(max_grasp_width_m))
    }

    pub fn b601dm_assumed_with_cap(open_width_m: f64, max_grasp_width_m: f64, max_grasp_width_cap: Option<f64>) -> Self {
        Self {
            finger_length_m: 0.055, finger_thickness_m: 0.008, finger_width_m: 0.018, pad_length_m: 0.025,
            palm_depth_m: 0.03, open_width_m, max_grasp_width_m, max_grasp_width_cap, source: "assumed_b601dm",
        }
    }
}

/// Wall-time budget and cooperative cancellation of one selection.
#[derive(Debug, Clone)]
pub struct Budget {
    pub wall_ms: u64,
    pub cancel: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
}

pub const DEFAULT_BUDGET_MS: u64 = 2000;
pub const MAX_BUDGET_MS: u64 = 10_000;

/// Everything a selection reads, fixed at capture.
#[derive(Debug, Clone)]
pub struct Snapshot {
    /// Content hash of the inputs (hex): candidate ids are `<id>-<n>`.
    pub id: String,
    /// TCP-frame plan input with the measured start joints.
    pub input: PlanInput,
    pub setup: GraspSetup,
    pub cam: CamModel,
    pub camera_down: bool,
    pub label: String,
    pub box_norm: [f64; 4],
    pub frame_id: Option<u64>,
    pub evidence: Evidence,
    pub open_rad: f32,
    pub closed_rad: f32,
    /// End_link points kept above the table ([`grasp::envelope_with_contact`]).
    pub envelope: Vec<Vec3>,
    pub tool: ToolVolumes,
    /// Derated holding-torque limits of the arm joints and their source.
    pub torque_limits: Option<(Vec<f64>, &'static str)>,
    pub prefs: PosePreferences,
    pub budget: Budget,
    /// Residuals of the calibration in effect (reported margin, [`super::margin`]).
    pub calib: super::margin::CalibQuality,
}

/// Request fields of the adaptive strategy (besides the legacy grasp fields).
pub fn parse_budget(body: &Value) -> Result<u64, String> {
    match body.get("budget_ms") {
        None | Some(Value::Null) => Ok(DEFAULT_BUDGET_MS),
        Some(v) => v.as_u64().filter(|b| (1..=MAX_BUDGET_MS).contains(b)).ok_or(format!("budget_ms must be an integer in [1, {MAX_BUDGET_MS}]")),
    }
}

fn invalid(m: String) -> ArmError { MotionError::InvalidCommand(m).into() }

/// Under the lock: bind the parsed request (`pending`) and the request's evidence to the
/// supervisor state.
pub fn capture(inner: &SupervisorInner, pending: &PendingGrasp, body: &Value) -> Result<Snapshot, ArmError> {
    capture_mode(inner, pending, body, true)
}

/// [`capture`] with `strict` = the adaptive rules (6-DOF only, malformed `evidence`,
/// `pose_preferences` or `budget_ms` refused). Legacy requests (`strict` false) capture on any
/// arm and ignore those adaptive-only fields when malformed.
pub fn capture_mode(inner: &SupervisorInner, pending: &PendingGrasp, body: &Value, strict: bool) -> Result<Snapshot, ArmError> {
    let policy = inner.config.gripper_policy.clone()
        .ok_or_else(|| ArmError::from(MotionError::Unsupported("Grasp needs a gripper policy".into())))?;
    capture_from_parts(
        pending,
        body,
        &policy,
        inner.requires_real_jaw_curve(),
        inner.config.gravity_comp.as_ref(),
        inner.config.damiao_ranges.as_deref(),
        inner.calibration.as_ref(),
        strict,
    )
}

/// Zero-lock capture from a published [`crate::rt_core::requests::actions::ActionPrepEnv`]
/// (used by both live `ActionSnapshot` and lease-free `ReadOnlySnapshot` on `OWNER_EXECUTOR`).
pub fn capture_from_env(
    env: &crate::rt_core::requests::actions::ActionPrepEnv,
    pending: &PendingGrasp,
    body: &Value,
) -> Result<Snapshot, ArmError> {
    capture_from_env_mode(env, pending, body, true)
}

/// [`capture_from_env`] with the `strict` rules of [`capture_mode`].
pub fn capture_from_env_mode(
    env: &crate::rt_core::requests::actions::ActionPrepEnv,
    pending: &PendingGrasp,
    body: &Value,
    strict: bool,
) -> Result<Snapshot, ArmError> {
    let policy = env.config.gripper_policy.clone()
        .ok_or_else(|| ArmError::from(MotionError::Unsupported("Grasp needs a gripper policy".into())))?;
    capture_from_parts(
        pending,
        body,
        &policy,
        env.requires_real_jaw_curve,
        env.config.gravity_comp.as_ref(),
        env.config.damiao_ranges.as_deref(),
        env.calibration.as_ref(),
        strict,
    )
}

fn capture_from_parts(
    pending: &PendingGrasp,
    body: &Value,
    policy: &crate::model::GripperPolicy,
    requires_real_jaw_curve: bool,
    gravity_comp: Option<&crate::model::GravityCompPolicy>,
    damiao_ranges: Option<&[crate::model::DamiaoRange]>,
    calibration: Option<&crate::calibration::CalibrationFile>,
    strict: bool,
) -> Result<Snapshot, ArmError> {
    let (evidence, prefs, wall_ms) = if strict {
        (parse_evidence(body).map_err(invalid)?, parse_preferences(body).map_err(invalid)?, parse_budget(body).map_err(invalid)?)
    } else {
        (parse_evidence_for_legacy(body).map_err(invalid)?, parse_preferences(body).unwrap_or_default(), parse_budget(body).unwrap_or(DEFAULT_BUDGET_MS))
    };
    let request = &pending.request;
    let mut input = pending.input.clone();
    if strict && input.arm.arm_dof() != 6 {
        return Err(MotionError::Unsupported("strategy \"adaptive\" needs a 6-DOF arm (the grasp orientation is fully constrained)".into()).into());
    }
    input.goal_frame = GoalFrame::Tcp;
    let setup = request.setup.clone();
    let cam = CamModel::assumed(&input.arm, &setup, &input.cam);
    let camera_down = grasp::camera_down_mode(&setup, &input.cam).map_err(invalid)?;
    let envelope = grasp::envelope_with_contact(input.ws.as_ref(), &input.cam, &cam.rec, pending.physical_contact_end_m);
    let open_width = f64::from(policy.opening_m(policy.open_rad));
    let (max_w, max_w_cap) = if policy.is_calibrated() {
        let calib = open_width - setup.pre_open_margin_m;
        let w = if setup.max_grasp_width_m > 0.0 { setup.max_grasp_width_m.min(calib) } else { calib };
        (w, Some(w))
    } else if setup.max_grasp_width_m > 0.0 {
        let check = crate::model::check_grasp_width(policy, setup.max_grasp_width_m, requires_real_jaw_curve);
        let w = check.effective_max_m(setup.max_grasp_width_m);
        (w, Some(w))
    } else {
        (0.0, None)
    };
    let torque_limits = gravity_comp
        .filter(|gc| gc.torque_limits_nm.len() >= 6)
        .map(|gc| (gc.torque_limits_nm.iter().take(6).map(|t| f64::from(*t)).collect(), "gravity_comp.torque_limits_nm"))
        .or_else(|| damiao_ranges.filter(|r| r.len() >= 6)
            .map(|r| (r.iter().take(6).map(|d| 0.5 * f64::from(d.t_max)).collect(), "damiao_ranges.t_max x 0.5")));
    let mut snap = Snapshot {
        id: String::new(), input, setup, cam, camera_down, label: request.label.clone(), box_norm: request.box_norm,
        frame_id: request.frame_id, evidence, open_rad: pending.open_rad, closed_rad: policy.closed_rad, envelope,
        tool: ToolVolumes::b601dm_assumed_with_cap(open_width, max_w, max_w_cap), torque_limits, prefs,
        budget: Budget { wall_ms, cancel: None },
        calib: super::margin::quality(calibration),
    };
    snap.id = snapshot_id(&snap);
    Ok(snap)
}

/// FNV-1a over the inputs that decide the selection (not the wall search budget).
pub fn snapshot_id(s: &Snapshot) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut eat = |x: f64| for b in x.to_bits().to_le_bytes() { h ^= u64::from(b); h = h.wrapping_mul(0x0100_0000_01b3); };
    s.input.q0.iter().for_each(|v| eat(*v));
    eat(f64::from(s.input.j7));
    s.input.tcp_offset.iter().for_each(|v| eat(*v));
    s.box_norm.iter().for_each(|v| eat(*v));
    for p in s.evidence.contour_norm.iter().flatten() { eat(p[0]); eat(p[1]); }
    for p in s.evidence.obb_corners_norm.iter().flatten() { eat(p[0]); eat(p[1]); }
    for v in [s.evidence.angle_deg, s.evidence.angle_conf, s.evidence.payload_kg].into_iter().flatten() { eat(v); }
    for v in s.evidence.height_m.iter().flatten() { eat(*v); }
    let st = &s.setup;
    for v in [
        st.table_z_m, st.approach_height_m, st.grasp_height_m, st.lift_height_m, st.camera_tilt_deg, st.fx_px, st.fy_px,
        st.approach_speed_mps, st.descend_speed_mps, st.max_grasp_width_m,
        st.timeout_s, st.phase_timeout_s, st.settle_delay_s, st.lost_timeout_s,
        st.post_lift_settle_ms as f64, st.descend_contact_stop_nm.unwrap_or(-1.0),
        f64::from(u8::from(st.align_j6)), st.align_min_angle_conf,
    ] { eat(v); }
    for p in &s.envelope { p.iter().for_each(|v| eat(*v)); }
    for v in [s.tool.open_width_m, s.tool.max_grasp_width_m, f64::from(s.open_rad), f64::from(s.closed_rad)] { eat(v); }
    if let Some((lim, _)) = &s.torque_limits {
        eat(lim.len() as f64);
        for v in lim { eat(*v); }
    } else {
        eat(-1.0);
    }
    eat(s.prefs.vertical_weight);
    eat(s.prefs.preferred_max_tilt_deg.unwrap_or(-1.0));
    eat(s.prefs.hard_max_tilt_deg.unwrap_or(-1.0));
    eat(s.prefs.approach_side as u8 as f64);
    eat(s.input.workspace_revision as f64);
    if let Some(ws) = &s.input.ws {
        eat(f64::from(ws.floor_z_m));
        for k in &ws.keepout_boxes { k.min_m.iter().chain(&k.max_m).for_each(|v| eat(f64::from(*v))); }
    }
    format!("{h:016x}")
}

/// Where the footprint came from (best first).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FootprintSource { Contour, Obb, Angle, Box }

impl FootprintSource {
    pub fn name(self) -> &'static str {
        match self { Self::Contour => "contour", Self::Obb => "obb", Self::Angle => "angle", Self::Box => "box" }
    }
}

/// The target as the planner sees it: a footprint on the table, a height interval and the
/// uncertainty of its position.
#[derive(Debug, Clone)]
pub struct Target {
    /// Counter-clockwise base-frame footprint, projected at the mid-height plane.
    pub poly: Vec<V2>,
    /// Base-frame footprint projected onto the table plane (`table_z`).
    pub poly_lo: Vec<V2>,
    /// Base-frame footprint projected onto the object top plane (`table_z + height[1]`).
    pub poly_hi: Vec<V2>,
    pub source: FootprintSource,
    /// Why better evidence was not used (contour rejected, ray missed, ...).
    pub fallback_reason: Option<String>,
    pub table_z: f64,
    /// Object height interval above the table (m) and its source.
    pub height: [f64; 2],
    pub height_source: &'static str,
    pub centroid: V2,
    pub axis: V2,
    /// Major / minor radius of gyration.
    pub aspect: f64,
    /// Long axis ambiguous (box only, or nearly round): stations are taken along both axes.
    pub axis_ambiguous: bool,
    /// Distance between the projections at the table and at the top (half), m.
    pub projection_spread_m: f64,
    /// Position error of the whole object used by the hard clearance checks (m): the larger
    /// of the scale and hand-eye errors (they may be one error) plus the contour error.
    pub hard_error_m: f64,
    pub scale_error_m: f64,
    pub hand_eye_error_m: f64,
    pub contour_error_m: f64,
    /// Camera centre to target distance at capture (m).
    pub view_distance_m: f64,
    /// The active (fallback) and the reported measured error budget.
    pub margin: super::margin::ErrorMargin,
}

/// Fractional scale error of the image-to-base mapping (user report: 12 mm at 0.40 m).
pub const SCALE_ERROR: f64 = 0.03;
/// Hand-eye rotation error (deg).
pub const HAND_EYE_ERROR_DEG: f64 = 0.5;
/// Contour location error (px).
pub const CONTOUR_ERROR_PX: f64 = 3.0;
/// Table height error (m).
pub const TABLE_ERROR_M: f64 = 0.005;
/// Height interval without evidence or prior (m).
pub const DEFAULT_HEIGHT_M: [f64; 2] = [0.02, 0.06];

/// The target from the snapshot's evidence: contour, else OBB, else a confident angle with the
/// box, else the box.
pub fn target(s: &Snapshot) -> Result<Target, String> {
    let st = &s.setup;
    let pose_end = s.input.arm.fk(&s.input.q0);
    let (height, height_source) = match (s.evidence.height_m, st.height_priors.iter().find(|(k, _)| k.eq_ignore_ascii_case(&s.label))) {
        (Some(h), _) => ([h[0], h[1]], "evidence"),
        (None, Some((_, h))) => ([0.8 * h, 1.2 * h], "height_prior"),
        _ => (DEFAULT_HEIGHT_M, "default"),
    };
    let table = st.table_z_m;
    let z_mid = table + 0.5 * (height[0] + height[1]) / 2.0;
    let project = |px: &[V2], z: f64| -> Option<Vec<V2>> {
        px.iter().map(|p| s.cam.plane_point(&pose_end, *p, z).map(|q| [q[0], q[1]])).collect()
    };
    let to_px = |n: &[V2]| -> Vec<V2> { n.iter().map(|p| s.cam.to_px(*p)).collect() };
    let b = s.box_norm;
    let box_px = to_px(&[[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]]]);
    let mut fallback: Option<String> = s.evidence.contour_error.clone().map(|e| format!("contour: {e}"));
    let mut pick: Option<(Vec<V2>, FootprintSource)> = None;
    if let Some(c) = &s.evidence.contour_norm {
        match project(&to_px(c), z_mid) {
            Some(p) => pick = (p.len() >= 3).then(|| (to_px(c), FootprintSource::Contour)),
            None => fallback = Some("contour: a vertex ray misses the plane".into()),
        }
    }
    if pick.is_none() {
        if let Some(o) = &s.evidence.obb_corners_norm { pick = Some((to_px(o), FootprintSource::Obb)); }
    }
    if pick.is_none() {
        if let (Some(a), Some(conf)) = (s.evidence.angle_deg, s.evidence.angle_conf) {
            if conf >= st.align_min_angle_conf {
                pick = oriented_rect_px(&box_px, a).map(|r| (r, FootprintSource::Angle));
            }
        }
    }
    let (px, source) = pick.unwrap_or((box_px, FootprintSource::Box));
    let mut poly = project(&px, z_mid).ok_or("the box rays do not meet the table plane in front of the camera")?;
    g::ensure_ccw(&mut poly);
    if g::signed_area(&poly) < 1e-8 { return Err("the projected footprint has no area".into()); }
    let lo = project(&px, table).ok_or("the footprint rays do not meet the table plane")?;
    let hi = project(&px, table + height[1]).ok_or("the footprint rays do not meet the object's top plane")?;
    let spread = lo.iter().zip(&hi).map(|(a, b)| g::norm2(g::sub2(*a, *b))).fold(0.0, f64::max) / 2.0;
    let c = g::centroid(&poly);
    let (axis, aspect) = g::principal_axis(&poly);
    let cam_o = s.cam.origin(&pose_end);
    let dist = kinematics::norm(&kinematics::sub(&cam_o, &[c[0], c[1], z_mid]));
    let scale_e = SCALE_ERROR * c[0].hypot(c[1]);
    let he_e = HAND_EYE_ERROR_DEG.to_radians().tan() * dist;
    let contour_e = CONTOUR_ERROR_PX * dist / s.cam.fx.min(s.cam.fy);
    let cam_tan = (cam_o[0] - c[0]).hypot(cam_o[1] - c[1]) / (cam_o[2] - z_mid).abs().max(1e-3);
    let margin = super::margin::margin(&s.calib, scale_e.max(he_e) + contour_e, contour_e, dist, cam_tan);
    Ok(Target {
        poly, poly_lo: lo, poly_hi: hi, source, fallback_reason: if source == FootprintSource::Contour { None } else { fallback }, table_z: table, height, height_source,
        centroid: c, axis, aspect, axis_ambiguous: source == FootprintSource::Box || aspect < 1.2,
        projection_spread_m: spread, hard_error_m: scale_e.max(he_e) + contour_e, scale_error_m: scale_e, hand_eye_error_m: he_e,
        contour_error_m: contour_e, view_distance_m: dist, margin,
    })
}

/// The rectangle with its long side at image angle `angle_deg` that the axis-aligned box
/// `box_px` (4 corners) circumscribes; None when the angle makes it ill-conditioned (near 45°
/// for a square-ish box).
fn oriented_rect_px(box_px: &[V2], angle_deg: f64) -> Option<Vec<V2>> {
    let (bw, bh) = (box_px[1][0] - box_px[0][0], box_px[2][1] - box_px[1][1]);
    let a = angle_deg.to_radians();
    let (c, s) = (a.cos().abs(), a.sin().abs());
    // bw = L c + W s, bh = L s + W c
    let det = c * c - s * s;
    if det.abs() < 0.2 { return None; }
    let l = (bw * c - bh * s) / det;
    let w = (bh * c - bw * s) / det;
    if !(l > 0.0 && w > 0.0) { return None; }
    let (l, w) = if l >= w { (l, w) } else { (w, l) };
    let ctr = [(box_px[0][0] + box_px[2][0]) / 2.0, (box_px[0][1] + box_px[2][1]) / 2.0];
    let u = [a.cos(), a.sin()];
    let v = g::perp2(u);
    Some([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]].iter()
        .map(|k| g::add2(ctr, g::add2(g::scale2(u, k[0] * l / 2.0), g::scale2(v, k[1] * w / 2.0)))).collect())
}

/// Pose of the camera at capture (end_link).
pub fn capture_pose(s: &Snapshot) -> Pose { s.input.arm.fk(&s.input.q0) }

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn evidence_and_preferences_parse() {
        let e = parse_evidence(&json!({"evidence": {"contour": [[0.1, 0.1], [0.2, 0.1], [0.2, 0.2]], "height_m": [0.03, 0.04], "angle_deg": 100.0}})).unwrap();
        assert_eq!(e.contour_norm.as_ref().map(Vec::len), Some(3));
        assert_eq!(e.height_m, Some([0.03, 0.04]));
        assert!((e.angle_deg.unwrap() + 80.0).abs() < 1e-9);
        let bad = parse_evidence(&json!({"evidence": {"contour": [[0.1, 0.1]]}})).unwrap();
        assert!(bad.contour_norm.is_none() && bad.contour_error.is_some());
        assert!(parse_evidence(&json!({"evidence": {"height_m": [0.05, 0.01]}})).is_err());
        assert!(parse_evidence(&json!({"evidence": 3})).is_err());
        let p = parse_preferences(&json!({"pose_preferences": {"vertical_weight": 1.0, "approach_side": "outward", "preferred_max_tilt_deg": 30, "hard": {"max_tilt_deg": 60}}})).unwrap();
        assert_eq!((p.vertical_weight, p.approach_side, p.preferred_max_tilt_deg, p.hard_max_tilt_deg), (1.0, Side::Outward, Some(30.0), Some(60.0)));
        assert!(parse_preferences(&json!({"pose_preferences": {"tilt": 3}})).is_err());
        assert!(parse_preferences(&json!({"pose_preferences": {"hard": {"max_tilt_deg": 120}}})).is_err());
        assert_eq!(parse_budget(&json!({})), Ok(DEFAULT_BUDGET_MS));
        assert!(parse_budget(&json!({"budget_ms": 0})).is_err());
    }

    #[test]
    fn oriented_rect_recovers_a_rotated_box() {
        // a 100 x 20 px rectangle at 20 deg circumscribed by its axis-aligned box
        let a = 20f64.to_radians();
        let (bw, bh) = (100.0 * a.cos() + 20.0 * a.sin(), 100.0 * a.sin() + 20.0 * a.cos());
        let b = [[0.0, 0.0], [bw, 0.0], [bw, bh], [0.0, bh]];
        let r = oriented_rect_px(&b, 20.0).expect("well conditioned");
        let l = g::norm2(g::sub2(r[1], r[0]));
        let w = g::norm2(g::sub2(r[2], r[1]));
        assert!((l - 100.0).abs() < 1e-6 && (w - 20.0).abs() < 1e-6, "{l} {w}");
        assert!(oriented_rect_px(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]], 45.0).is_none());
    }
}
