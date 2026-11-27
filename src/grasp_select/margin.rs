//! Calibration-derived position error budget of a grasp target.
//!
//! The budget is computed from the residuals stored in `calibration.json` and reported, but the
//! admission keeps using the fallback constants of [`super::snapshot`] until the E4 calibration
//! campaign shows which residuals bound the error across the workspace and establishes the
//! factor that inflates an RMS residual to a bound (`error_margin_source` stays `fallback`).

use serde_json::{json, Value};

use crate::calibration::CalibrationFile;

/// Residuals of the calibration in effect at capture (m, deg), and why they are incomplete.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CalibQuality {
    /// Larger of the A→B / B→A kinematic held-out errors.
    pub kin_heldout_m: Option<f64>,
    pub hand_eye_m: Option<f64>,
    pub hand_eye_deg: Option<f64>,
    /// Board-origin height range of the table fit.
    pub plane_spread_m: Option<f64>,
    /// Fingertip height spread of a passed, valid hover acceptance.
    pub hover_height_spread_m: Option<f64>,
    /// Why the measured budget cannot be formed (None: all residuals present).
    pub problem: Option<String>,
}

fn finite_max(v: &Value) -> Option<f64> {
    match v {
        Value::Array(a) => a.iter().filter_map(Value::as_f64).filter(|x| x.is_finite() && *x >= 0.0).reduce(f64::max),
        _ => v.as_f64().filter(|x| x.is_finite() && *x >= 0.0),
    }
}

/// The residuals of `cal` (None: no calibration file).
pub fn quality(cal: Option<&CalibrationFile>) -> CalibQuality {
    let Some(cal) = cal else { return CalibQuality { problem: Some("no calibration".into()), ..Default::default() } };
    let rms = cal.rms.as_ref();
    let rms_f = |k: &str| rms.and_then(|r| r.get(k)).and_then(finite_max);
    let kin_mm = cal.kinematics.as_ref().and_then(|k| k.fit.as_ref()).and_then(|f| f.get("heldout_mm")).and_then(finite_max)
        .or_else(|| rms_f("kin_heldout_mm"));
    let hover = cal.acceptance.as_ref().filter(|a| a.passed && a.invalidated.is_none())
        .and_then(|a| a.summary.get("height_spread_m")).and_then(finite_max);
    let mut q = CalibQuality {
        kin_heldout_m: kin_mm.map(|mm| mm / 1000.0),
        hand_eye_m: rms_f("hand_eye_m"),
        hand_eye_deg: rms_f("hand_eye_deg"),
        plane_spread_m: rms_f("plane_spread_m"),
        hover_height_spread_m: hover,
        problem: None,
    };
    let mut missing = Vec::new();
    if q.kin_heldout_m.is_none() { missing.push("kinematic held-out residual"); }
    if q.hand_eye_m.is_none() || q.hand_eye_deg.is_none() { missing.push("hand-eye residuals"); }
    if q.plane_spread_m.is_none() && q.hover_height_spread_m.is_none() { missing.push("table residual"); }
    q.problem = if cal.stale.is_some() {
        Some("stale: measured before a joint zero change".into())
    } else if !missing.is_empty() {
        Some(format!("partial: no {}", missing.join(", ")))
    } else {
        None
    };
    q
}

/// The fallback budget (active) and the measured one (reported) of one target.
#[derive(Debug, Clone, PartialEq)]
pub struct ErrorMargin {
    /// Active hard error (m): the fallback formula.
    pub hard_error_m: f64,
    /// Measured budget (m) when every residual is present and current.
    pub measured_budget_m: Option<f64>,
    pub kinematic_m: Option<f64>,
    pub hand_eye_m: Option<f64>,
    pub contour_m: f64,
    pub table_xy_m: Option<f64>,
    /// Why the measured budget is not active.
    pub reason: String,
}

/// The margin of a target `view_distance_m` from the camera, `cam_tan` = horizontal / vertical
/// camera-to-target distance (projects a table height error into XY), with the fallback
/// `hard_error_m` and contour term.
pub fn margin(q: &CalibQuality, hard_error_m: f64, contour_m: f64, view_distance_m: f64, cam_tan: f64) -> ErrorMargin {
    let hand_eye = match (q.hand_eye_m, q.hand_eye_deg) {
        (Some(m), Some(deg)) => Some(m + view_distance_m * deg.to_radians().tan()),
        _ => None,
    };
    let dz = match (q.plane_spread_m, q.hover_height_spread_m) {
        (Some(a), Some(b)) => Some(a.max(b)),
        (a, b) => a.or(b),
    };
    let table_xy = dz.map(|dz| dz * cam_tan.abs());
    let measured = match (&q.problem, q.kin_heldout_m, hand_eye, table_xy) {
        (None, Some(k), Some(he), Some(txy)) => Some(k.max(he) + contour_m + txy),
        _ => None,
    };
    let reason = match &q.problem {
        Some(p) => format!("fallback: {p}"),
        None => "measured budget reported only: the fallback constants stay active until E4 calibration validates the residuals (no RMS inflation factor established)".into(),
    };
    ErrorMargin { hard_error_m, measured_budget_m: measured, kinematic_m: q.kin_heldout_m, hand_eye_m: hand_eye, contour_m, table_xy_m: table_xy, reason }
}

impl ErrorMargin {
    pub fn to_json(&self) -> Value {
        json!({
            "error_margin_source": "fallback",
            "error_margin_reason": self.reason,
            "hard_error_m": self.hard_error_m,
            "measured_budget_m": self.measured_budget_m,
            "measured_components": {"kinematic_m": self.kinematic_m, "hand_eye_m": self.hand_eye_m, "contour_m": self.contour_m, "table_xy_m": self.table_xy_m},
        })
    }
}
