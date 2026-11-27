//! Calibration-derived position error budget of a grasp target.
//!
//! The budget is computed from the residuals stored in `calibration.json` and reported, but the
//! admission keeps using the fallback constants of [`super::snapshot`] until the E4 calibration
//! campaign shows which residuals bound the error across the workspace and establishes the
//! factor that inflates an RMS residual to a bound (`error_margin_source` stays `fallback`).

use serde_json::{json, Value};

use crate::calibration::{
    acceptance_valid_for_with_legacy, file_fingerprints, legacy_file_fingerprint, CalibrationFile,
};
use crate::model::CameraMount;
use crate::tool::ActiveToolInfo;

/// Live fingerprint & active-tool context required to verify stored hover acceptance records.
pub struct AcceptanceContext<'a> {
    pub camera_mount: &'a CameraMount,
    pub joint_zero_offsets: &'a [f32],
    pub active_tool: &'a ActiveToolInfo,
}

/// Residuals of the calibration in effect at capture (m, deg), and why they are incomplete,
/// invalid, stale, or mismatched.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CalibQuality {
    /// Larger of the A→B / B→A kinematic held-out errors.
    pub kin_heldout_m: Option<f64>,
    pub hand_eye_m: Option<f64>,
    pub hand_eye_deg: Option<f64>,
    /// Board-origin height range of the table fit.
    pub plane_spread_m: Option<f64>,
    /// Fingertip height spread of a passed, fingerprint- and tool-valid hover acceptance.
    pub hover_height_spread_m: Option<f64>,
    /// Why the measured budget cannot be formed (None: all residuals valid and current).
    pub problem: Option<String>,
}

enum Field {
    Absent,
    Valid(f64),
    Invalid(&'static str),
}

fn parse_scalar(v: Option<&Value>, max: f64, err: &'static str) -> Field {
    match v {
        None | Some(Value::Null) => Field::Absent,
        Some(x) => match x.as_f64() {
            Some(n) if n.is_finite() && n >= 0.0 && n <= max => Field::Valid(n),
            _ => Field::Invalid(err),
        },
    }
}

fn parse_deg(v: Option<&Value>, err: &'static str) -> Field {
    match v {
        None | Some(Value::Null) => Field::Absent,
        Some(x) => match x.as_f64() {
            Some(n) if n.is_finite() && (0.0..89.0).contains(&n) => Field::Valid(n),
            _ => Field::Invalid(err),
        },
    }
}

fn parse_heldout_mm(v: Option<&Value>) -> Field {
    match v {
        None | Some(Value::Null) => Field::Absent,
        Some(Value::Array(a)) => {
            if a.is_empty() || a.len() > 2 {
                return Field::Invalid("kinematic held-out residual array must have 1 or 2 values");
            }
            let mut max_v: f64 = 0.0;
            for item in a {
                match item.as_f64() {
                    Some(n) if n.is_finite() && (0.0..=1000.0).contains(&n) => max_v = max_v.max(n),
                    _ => return Field::Invalid("kinematic held-out residual array contains invalid entry"),
                }
            }
            Field::Valid(max_v)
        }
        Some(x) => match x.as_f64() {
            Some(n) if n.is_finite() && (0.0..=1000.0).contains(&n) => Field::Valid(n),
            _ => Field::Invalid("kinematic held-out residual must be a finite number in [0, 1000] mm"),
        },
    }
}

/// The residuals of `cal` without active fingerprint/tool context (hover acceptance is never
/// trusted without [`AcceptanceContext`]).
pub fn quality(cal: Option<&CalibrationFile>) -> CalibQuality {
    quality_for(cal, None)
}

/// The residuals of `cal`, validating shapes/ranges and checking stored hover acceptance against
/// the active calibration fingerprint and tool via [`acceptance_valid_for_with_legacy`].
pub fn quality_for(cal: Option<&CalibrationFile>, ctx: Option<&AcceptanceContext<'_>>) -> CalibQuality {
    let Some(cal) = cal else {
        return CalibQuality { problem: Some("no calibration".into()), ..Default::default() };
    };
    let rms = cal.rms.as_ref();
    let kin_raw = cal
        .kinematics
        .as_ref()
        .and_then(|k| k.fit.as_ref())
        .and_then(|f| f.get("heldout_mm"))
        .or_else(|| rms.and_then(|r| r.get("kin_heldout_mm")));
    let kin_f = parse_heldout_mm(kin_raw);
    let he_m_f = parse_scalar(rms.and_then(|r| r.get("hand_eye_m")), 1.0, "hand_eye_m must be a finite scalar in [0, 1] m");
    let he_deg_f = parse_deg(rms.and_then(|r| r.get("hand_eye_deg")), "hand_eye_deg must be a finite scalar in [0, 89) deg");
    let plane_f = parse_scalar(rms.and_then(|r| r.get("plane_spread_m")), 1.0, "plane_spread_m must be a finite scalar in [0, 1] m");

    let mut invalid: Vec<&'static str> = Vec::new();
    let mut take = |f: Field| -> Option<f64> {
        match f {
            Field::Valid(v) => Some(v),
            Field::Invalid(why) => {
                invalid.push(why);
                None
            }
            Field::Absent => None,
        }
    };
    let kin_heldout_m = take(kin_f).map(|mm| mm / 1000.0);
    let hand_eye_m = take(he_m_f);
    let hand_eye_deg = take(he_deg_f);
    let plane_spread_m = take(plane_f);

    let mut hover_mismatch: Option<&'static str> = None;
    let hover_height_spread_m = if let Some(acc) = cal.acceptance.as_ref() {
        let valid = match ctx {
            Some(c) => {
                let (_, fp) = file_fingerprints(
                    Some(cal),
                    c.camera_mount,
                    c.joint_zero_offsets,
                    Some(&c.active_tool.config_digest),
                );
                let legacy_fp = legacy_file_fingerprint(Some(cal), c.camera_mount, c.joint_zero_offsets);
                acceptance_valid_for_with_legacy(cal, &fp, Some(&legacy_fp), c.active_tool)
            }
            None => false,
        };
        if valid {
            take(parse_scalar(
                acc.summary.get("height_spread_m"),
                1.0,
                "acceptance.summary.height_spread_m must be a finite scalar in [0, 1] m",
            ))
        } else {
            hover_mismatch = Some(if ctx.is_some() {
                "hover acceptance invalid or mismatched for active calibration/tool"
            } else {
                "hover acceptance unverified without active calibration/tool context"
            });
            None
        }
    } else {
        None
    };

    let mut missing = Vec::new();
    if kin_heldout_m.is_none() {
        missing.push("kinematic held-out residual");
    }
    if hand_eye_m.is_none() || hand_eye_deg.is_none() {
        missing.push("hand-eye residuals");
    }
    if plane_spread_m.is_none() && hover_height_spread_m.is_none() {
        missing.push("table residual");
    }

    let problem = if cal.stale.is_some() {
        Some("stale: measured before a joint zero change".into())
    } else if !invalid.is_empty() {
        Some(format!("invalid: {}", invalid.join("; ")))
    } else if let Some(hm) = hover_mismatch.filter(|_| plane_spread_m.is_none() && hover_height_spread_m.is_none()) {
        Some(format!("mismatched: {hm}"))
    } else if !missing.is_empty() {
        Some(format!("partial: no {}", missing.join(", ")))
    } else {
        None
    };

    CalibQuality {
        kin_heldout_m,
        hand_eye_m,
        hand_eye_deg,
        plane_spread_m,
        hover_height_spread_m,
        problem,
    }
}

/// The fallback budget (active) and the measured one (reported) of one target.
#[derive(Debug, Clone, PartialEq)]
pub struct ErrorMargin {
    /// Active hard error (m): the fallback formula.
    pub hard_error_m: f64,
    /// Measured budget (m) when every residual is present, valid, and current.
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
pub fn margin(
    q: &CalibQuality,
    hard_error_m: f64,
    contour_m: f64,
    view_distance_m: f64,
    cam_tan: f64,
) -> ErrorMargin {
    let geom_ok = view_distance_m.is_finite()
        && view_distance_m >= 0.0
        && cam_tan.is_finite()
        && contour_m.is_finite()
        && contour_m >= 0.0;
    let hand_eye = if geom_ok {
        match (q.hand_eye_m, q.hand_eye_deg) {
            (Some(m), Some(deg)) => {
                let v = m + view_distance_m * deg.to_radians().tan();
                v.is_finite().then_some(v)
            }
            _ => None,
        }
    } else {
        None
    };
    let dz = match (q.plane_spread_m, q.hover_height_spread_m) {
        (Some(a), Some(b)) => Some(a.max(b)),
        (a, b) => a.or(b),
    };
    let table_xy = if geom_ok {
        dz.and_then(|dz| {
            let v = dz * cam_tan.abs();
            v.is_finite().then_some(v)
        })
    } else {
        None
    };
    let measured = if geom_ok {
        match (&q.problem, q.kin_heldout_m, hand_eye, table_xy) {
            (None, Some(k), Some(he), Some(txy)) => {
                let b = k.max(he) + contour_m + txy;
                b.is_finite().then_some(b)
            }
            _ => None,
        }
    } else {
        None
    };
    let reason = if !geom_ok {
        "fallback: non-finite target geometry".into()
    } else {
        match &q.problem {
            Some(p) => format!("fallback: {p}"),
            None => "measured budget reported only: the fallback constants stay active until E4 calibration validates the residuals (no RMS inflation factor established)".into(),
        }
    };
    ErrorMargin {
        hard_error_m,
        measured_budget_m: measured,
        kinematic_m: q.kin_heldout_m,
        hand_eye_m: hand_eye,
        contour_m,
        table_xy_m: table_xy,
        reason,
    }
}

impl ErrorMargin {
    pub fn to_json(&self) -> Value {
        json!({
            "error_margin_source": "fallback",
            "error_margin_reason": self.reason,
            "hard_error_m": self.hard_error_m,
            "measured_budget_m": self.measured_budget_m,
            "measured_components": {
                "kinematic_m": self.kinematic_m,
                "hand_eye_m": self.hand_eye_m,
                "contour_m": self.contour_m,
                "table_xy_m": self.table_xy_m,
            },
        })
    }
}
