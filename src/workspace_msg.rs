//! Canonical workspace refusal formatter owned by [`crate::chain::WorkspaceFailure`].
//! Used by `SafetyError`, `MotionError`, and zero-allocation RT outputs.

use crate::chain::WorkspaceFailure;
use crate::kinematics as k;

/// Single witness rendered by [`WorkspaceFailure::write_witness`].
#[derive(Debug, Clone, Copy)]
pub enum WorkspaceWitness<'a> {
    Constraint { cid: &'a str, gid: &'a str, limit: &'a str, value: f64, bound: f64, prev_depth: f64, depth: f64, start_outside: bool },
    SweepBudget { max_budget: usize, blocking_cid: Option<&'a str>, blocking_gid: Option<&'a str>, keepout_name: Option<&'a str>, start_outside: bool },
    Invalid { reason: &'a str },
}

pub(crate) fn box_edge_info(cid: &str) -> Option<(&'static str, &'static str)> {
    match cid {
        "box_max_x" => Some(("front edge", "move back")), "box_min_x" => Some(("back edge", "move forward")),
        "box_max_y" => Some(("left edge", "move right")), "box_min_y" => Some(("right edge", "move left")),
        "box_max_z" => Some(("top edge", "move down")), "box_min_z" => Some(("bottom edge", "move up")),
        _ => None,
    }
}

fn recovery_direction(cid: Option<&str>) -> &'static str {
    match cid {
        Some("box_max_x") => "back (-X)", Some("box_min_x") => "forward (+X)",
        Some("box_max_y") => "right (-Y)", Some("box_min_y") => "left (+Y)",
        Some("box_max_z") => "down (-Z)", Some("box_min_z") | Some("floor_z") => "up (+Z)",
        Some("max_reach") => "inward toward the base",
        Some(c) if c.starts_with("self_collision:") => "away from self-collision",
        _ => "inward toward the workspace",
    }
}

fn write_deepening(w: &mut impl core::fmt::Write, deepening: bool, delta_m: f64) {
    if !deepening { return; }
    let mm = delta_m * 1000.0;
    if mm >= 1.0 && ((mm - mm.round()).abs() < 0.05 || mm >= 10.0) { let _ = write!(w, "; this move would go {:.0} mm deeper", mm.round()); }
    else { let _ = write!(w, "; this move would go {:.1} mm deeper", mm.max(0.1)); }
}

impl WorkspaceFailure {
    /// Canonical, zero-allocation workspace refusal formatter owned by [`WorkspaceFailure`].
    pub fn write_witness(w: &mut impl core::fmt::Write, witness: WorkspaceWitness<'_>) {
        match witness {
            WorkspaceWitness::Invalid { reason } => { let _ = w.write_str(reason); }
            WorkspaceWitness::SweepBudget { max_budget, blocking_cid, blocking_gid, keepout_name, start_outside } => {
                if let Some(name) = keepout_name {
                    let gid = blocking_gid.unwrap_or("end_origin");
                    let (act, tail) = if start_outside {
                        ("press Park, take a shorter jog away from obstacle", ", or move the obstacle")
                    } else { ("take a shorter jog or move the obstacle", "") };
                    let _ = write!(w, "path check near keep-out box '{name}' ({gid}) needs more than {max_budget} FK evaluations; {act} '{name}'{tail}");
                } else {
                    let kind = if start_outside { "return path" } else { "path" };
                    let dir = recovery_direction(blocking_cid);
                    let _ = write!(w, "{kind} could not be checked (needs more than {max_budget} FK evaluations");
                    if let (Some(cid), Some(gid)) = (blocking_cid, blocking_gid) { let _ = write!(w, " near {cid} on {gid}"); }
                    let _ = write!(w, "); press Park or make a short jog {dir}");
                }
            }
            WorkspaceWitness::Constraint { cid, gid, limit, value, bound, prev_depth, depth, start_outside } => {
                if !depth.is_finite() || !value.is_finite() || !bound.is_finite() {
                    let _ = write!(w, "invalid workspace check for {cid} ({gid})");
                    return;
                }
                let depth = depth.max(0.0);
                let prev = if prev_depth.is_finite() { prev_depth.max(0.0) } else { 0.0 };
                let (deepening, delta) = (prev > k::WORKSPACE_EPS, (depth - prev).max(0.0));
                if let Some((edge, recover)) = box_edge_info(cid) {
                    let part = if gid == "camera_mount" { "camera" } else if gid.starts_with("capsule:") { "arm" } else { "gripper" };
                    let esc = if start_outside || deepening { "press Park" } else { "press Home" };
                    if depth >= 0.001 { let _ = write!(w, "{part} ({gid}) is {:.1} cm", depth * 100.0); }
                    else { let _ = write!(w, "{part} ({gid}) is {:.1} mm", (depth * 1000.0).max(0.1)); }
                    let _ = write!(w, " beyond the workspace {edge} ({cid} {value:.3} m, limit {bound:.3} m; previous depth {prev:.3} m, new depth {depth:.3} m");
                    write_deepening(w, deepening, delta);
                    let _ = write!(w, "); {recover} or {esc}");
                } else if let Some(name) = cid.strip_prefix("keepout:") {
                    let verb = if deepening { "inside" } else { "enters" };
                    let _ = write!(w, "{gid} {verb} keepout '{name}' (keep-out obstacle depth {value:.3} m, limit {bound:.3} m; previous depth {prev:.3} m, new depth {depth:.3} m");
                    write_deepening(w, deepening, delta);
                    let _ = w.write_str(")");
                } else {
                    let (label, dir_word) = match cid { "max_reach" => ("reach", "over"), "floor_z" => ("floor z", "below"), _ => (limit, "under") };
                    let _ = write!(w, "{gid} {label} {value:.3} m (limit {bound:.3} m, new depth {depth:.3} m {dir_word}, previous depth {prev:.3} m");
                    write_deepening(w, deepening, delta);
                    let _ = w.write_str(")");
                }
            }
        }
    }

    /// Formats the user-facing message for `SafetyError::WorkspaceViolation` and `MotionError::WorkspaceViolation`.
    pub fn format_error_message(
        path_s: Option<f64>, _point: usize, limit: &str, value: f64, bound: f64, detail: &k::WorkspaceViolationDetail,
    ) -> String {
        let body = detail.reason.as_deref().filter(|s| !s.is_empty()).map_or_else(|| {
            let mut buf = String::new();
            Self::write_witness(&mut buf, WorkspaceWitness::Constraint {
                cid: detail.constraint_id.as_deref().unwrap_or(limit), gid: detail.geometry_id.as_deref().unwrap_or("end_origin"),
                limit, value, bound, prev_depth: 0.0, depth: (value - bound).abs(), start_outside: false,
            });
            buf
        }, str::to_string);
        match path_s {
            Some(s) if !detail.constraint_id.as_deref().is_some_and(|cid| box_edge_info(cid).is_some() || cid == "sweep_budget") => {
                format!("Path leaves the safe workspace at s={s:.3}: {body}")
            }
            _ => body,
        }
    }
}

/// Decodes a compact [`crate::rt_core::mailbox::RefusalDetail::WorkspaceBoxEdge`] payload off-RT.
#[allow(clippy::too_many_arguments)]
pub fn decode_workspace_box_edge_refusal(
    edge_idx: u8, geom_kind: u8, capsule_len: u8, capsule_id: &[u8; 32],
    depth_bits: u64, prev_depth_bits: u64, value_bits: u64, bound_bits: u64, start_outside: bool,
) -> Option<(&'static str, String, String)> {
    let (cid, limit) = match edge_idx {
        0 => ("box_min_x", "box_x_min"), 1 => ("box_max_x", "box_x_max"), 2 => ("box_min_y", "box_y_min"),
        3 => ("box_max_y", "box_y_max"), 4 => ("box_min_z", "box_z_min"), 5 => ("box_max_z", "box_z_max"),
        _ => return None,
    };
    let gid = match geom_kind {
        0 => "end_origin".into(), 254 => "camera_mount".into(),
        255 => format!("capsule:{}", core::str::from_utf8(&capsule_id[..usize::from(capsule_len).min(capsule_id.len())]).unwrap_or("link")),
        n => format!("end_point:{n}"),
    };
    let (depth, prev_depth, value, bound) = (f64::from_bits(depth_bits), f64::from_bits(prev_depth_bits), f64::from_bits(value_bits), f64::from_bits(bound_bits));
    if ![depth, prev_depth, value, bound].iter().all(|v| v.is_finite()) { return None; }
    let mut reason = String::new();
    WorkspaceFailure::write_witness(&mut reason, WorkspaceWitness::Constraint { cid, gid: &gid, limit, value, bound, prev_depth, depth, start_outside });
    Some((cid, gid, reason))
}

/// Attaches structured workspace box-edge fields to a JSON rejection reply off the RT thread.
#[cfg(not(target_arch = "wasm32"))]
pub fn attach_workspace_box_edge_fields(reply: &mut serde_json::Value, detail: crate::rt_core::mailbox::RefusalDetail, phase: &str) -> bool {
    let crate::rt_core::mailbox::RefusalDetail::WorkspaceBoxEdge {
        edge_idx, geom_kind, capsule_len, capsule_id, depth_bits, prev_depth_bits, value_bits, bound_bits, start_outside,
    } = detail else { return false; };
    let Some((cid, gid, reason)) = decode_workspace_box_edge_refusal(
        edge_idx, geom_kind, capsule_len, &capsule_id, depth_bits, prev_depth_bits, value_bits, bound_bits, start_outside,
    ) else { return false; };
    for (k, v) in [("constraint_id", cid), ("geometry_id", gid.as_str()), ("violation_phase", phase), ("reason", reason.as_str()), ("message", reason.as_str())] {
        reply[k] = serde_json::json!(v);
    }
    true
}
