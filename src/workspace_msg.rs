//! Canonical workspace refusal formatter owned by [`crate::chain::WorkspaceFailure`].
//! Used by `SafetyError`, `MotionError`, and zero-allocation RT outputs.

use crate::chain::WorkspaceFailure;
use crate::kinematics as k;

/// Single witness rendered by [`WorkspaceFailure::write_witness`].
#[derive(Debug, Clone, Copy)]
pub enum WorkspaceWitness<'a> {
    Constraint {
        cid: &'a str, gid: &'a str, limit: &'a str,
        value: f64, bound: f64, prev_depth: f64, depth: f64,
        start_outside: bool, conservative_bound: bool,
    },
    SweepBudget {
        max_budget: usize, blocking_cid: Option<&'a str>, blocking_gid: Option<&'a str>,
        keepout_name: Option<&'a str>, start_outside: bool,
    },
    Invalid { reason: &'a str },
}

/// Zero-allocation, compact (128-byte) `Copy` workspace refusal payload carried across the RT
/// mailbox for EVERY constraint kind (preserving full 64-byte capsule/keep-out IDs) and rendered
/// off-RT by [`WorkspaceFailure::write_witness`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WorkspaceRefusalRt {
    pub value_bits: u64,
    pub bound_bits: u64,
    pub prev_depth_bits: u64,
    pub depth_bits: u64,
    pub max_budget: u32,
    /// 0=Constraint, 1=ConservativeBound, 2=SweepBudget, 3=Invalid
    pub kind: u8,
    /// 0..=5=box_{min,max}_{x,y,z}, 6=floor_z, 7=max_reach, 8=keepout, 9=self_collision, 255=none
    pub constraint_tag: u8,
    /// 0=end_origin, 1..=252=end_point:N, 253=tcp_origin, 254=camera_mount, 255=capsule
    pub geom_kind: u8,
    pub start_outside: bool,
    pub name1_len: u8,
    pub name2_len: u8,
    /// Packed UTF-8 bytes: `name1` (`0..name1_len`, e.g. capsule id or invalid reason) followed by
    /// `name2` (`name1_len..name1_len+name2_len`, e.g. keep-out name or 2nd self-collision capsule).
    pub names: [u8; 82],
}

impl WorkspaceRefusalRt {
    pub(crate) fn pack_names(n1: &str, n2: &str) -> (u8, u8, [u8; 82]) {
        let mut buf = [0u8; 82];
        let mut l1 = n1.len().min(buf.len());
        while l1 > 0 && !n1.is_char_boundary(l1) { l1 -= 1; }
        buf[..l1].copy_from_slice(&n1.as_bytes()[..l1]);
        let rem = buf.len() - l1;
        let mut l2 = n2.len().min(rem);
        while l2 > 0 && !n2.is_char_boundary(l2) { l2 -= 1; }
        buf[l1..l1 + l2].copy_from_slice(&n2.as_bytes()[..l2]);
        (l1 as u8, l2 as u8, buf)
    }

    pub(crate) fn unpack_names(&self) -> (&str, &str) {
        let l1 = usize::from(self.name1_len).min(self.names.len());
        let l2 = usize::from(self.name2_len).min(self.names.len().saturating_sub(l1));
        let n1 = core::str::from_utf8(&self.names[..l1]).unwrap_or("?");
        let n2 = core::str::from_utf8(&self.names[l1..l1 + l2]).unwrap_or("?");
        (n1, n2)
    }
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

fn write_depth_value(w: &mut impl core::fmt::Write, depth_m: f64) {
    if depth_m > 0.0 && depth_m < 0.001 {
        let _ = write!(w, "{:.1} mm", (depth_m * 1000.0).max(0.1));
    } else {
        let _ = write!(w, "{depth_m:.3} m");
    }
}

fn write_coord_pair(w: &mut impl core::fmt::Write, value: f64, bound: f64, sub_mm: bool) {
    if sub_mm { let _ = write!(w, "{value:.4} m, limit {bound:.4} m"); }
    else { let _ = write!(w, "{value:.3} m, limit {bound:.3} m"); }
}

fn write_deepening(w: &mut impl core::fmt::Write, deepening: bool, delta_m: f64) {
    if !deepening { return; }
    let mm = delta_m * 1000.0;
    if mm >= 1.0 && ((mm - mm.round()).abs() < 0.05 || mm >= 10.0) {
        let _ = write!(w, "; this move would go {:.0} mm deeper", mm.round());
    } else {
        let _ = write!(w, "; this move would go {:.1} mm deeper", mm.max(0.1));
    }
}

impl WorkspaceFailure {
    /// Canonical, zero-allocation workspace refusal formatter owned by [`WorkspaceFailure`].
    pub fn write_witness(w: &mut impl core::fmt::Write, witness: WorkspaceWitness<'_>) {
        match witness {
            WorkspaceWitness::Invalid { reason } => { let _ = w.write_str(reason); }
            WorkspaceWitness::SweepBudget { max_budget, blocking_cid, blocking_gid, keepout_name, start_outside } => {
                if let Some(name) = keepout_name {
                    let act = if start_outside {
                        "press Park, take a shorter jog away from the obstacle, or move the obstacle"
                    } else {
                        "take a shorter jog or move the obstacle"
                    };
                    let gid = blocking_gid.unwrap_or("end_origin");
                    let _ = write!(w, "path check near keep-out box '{name}' needs more than {max_budget} FK evaluations; {act} ({gid})");
                } else {
                    let dir = recovery_direction(blocking_cid);
                    let _ = write!(w, "return path could not be checked (needs more than {max_budget} FK evaluations); press Park or make a short jog {dir}");
                    if let (Some(cid), Some(gid)) = (blocking_cid, blocking_gid) {
                        let _ = write!(w, " (near {cid} on {gid})");
                    }
                }
            }
            WorkspaceWitness::Constraint { cid, gid, limit, value, bound, prev_depth, depth, start_outside, conservative_bound } => {
                if !depth.is_finite() || !value.is_finite() || !bound.is_finite() {
                    let _ = write!(w, "invalid workspace check for {cid} ({gid})");
                    return;
                }
                let depth = depth.max(0.0);
                let prev = if prev_depth.is_finite() { prev_depth.max(0.0) } else { 0.0 };
                let sub_mm = depth > 0.0 && depth < 0.001;
                let (deepening, delta) = (!conservative_bound && prev > k::WORKSPACE_EPS, (depth - prev).max(0.0));
                let depth_lbl = if conservative_bound { "bound depth" } else { "new depth" };
                if let Some((edge, recover)) = box_edge_info(cid) {
                    let part = if gid == "camera_mount" { "camera" } else if gid.starts_with("capsule:") { "arm" } else { "gripper" };
                    let esc = if start_outside || prev > k::WORKSPACE_EPS { "press Park" } else { "press Home" };
                    if conservative_bound {
                        let _ = write!(w, "{part} ({gid}) sweep could not verify workspace {edge} ({cid} conservative bound ");
                    } else if depth >= 0.001 {
                        let _ = write!(w, "{part} ({gid}) is {:.1} cm beyond the workspace {edge} ({cid} ", depth * 100.0);
                    } else {
                        let _ = write!(w, "{part} ({gid}) is {:.1} mm beyond the workspace {edge} ({cid} ", (depth * 1000.0).max(0.1));
                    }
                    write_coord_pair(w, value, bound, sub_mm);
                    let _ = w.write_str("; previous depth ");
                    write_depth_value(w, prev);
                    let _ = write!(w, ", {depth_lbl} ");
                    write_depth_value(w, depth);
                    write_deepening(w, deepening, delta);
                    let _ = write!(w, "); {recover} or {esc}");
                } else if let Some(name) = cid.strip_prefix("keepout:") {
                    let head = if conservative_bound { "sweep could not verify keepout" } else if deepening { "inside keepout" } else { "enters keepout" };
                    let val_lbl = if conservative_bound { "keep-out obstacle conservative bound" } else { "keep-out obstacle depth" };
                    let _ = write!(w, "{gid} {head} '{name}' ({val_lbl} ");
                    write_coord_pair(w, value, bound, sub_mm);
                    let _ = w.write_str("; previous depth ");
                    write_depth_value(w, prev);
                    let _ = write!(w, ", {depth_lbl} ");
                    write_depth_value(w, depth);
                    write_deepening(w, deepening, delta);
                    let _ = w.write_str(")");
                } else {
                    let (label, dir_word) = match cid { "max_reach" => ("reach", "over"), "floor_z" => ("floor z", "below"), _ => (limit, "under") };
                    if conservative_bound {
                        let _ = write!(w, "{gid} sweep could not verify {label} (conservative bound ");
                        write_coord_pair(w, value, bound, sub_mm);
                    } else if sub_mm {
                        let _ = write!(w, "{gid} {label} {value:.4} m (limit {bound:.4} m");
                    } else {
                        let _ = write!(w, "{gid} {label} {value:.3} m (limit {bound:.3} m");
                    }
                    let _ = write!(w, ", {depth_lbl} ");
                    write_depth_value(w, depth);
                    let _ = write!(w, " {dir_word}, previous depth ");
                    write_depth_value(w, prev);
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
                limit, value, bound, prev_depth: 0.0, depth: (value - bound).abs(), start_outside: false, conservative_bound: false,
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

/// Decodes a zero-alloc [`WorkspaceRefusalRt`] mailbox payload off the RT thread into
/// `(constraint_id, geometry_id, reason)` using [`WorkspaceFailure::write_witness`].
pub fn decode_workspace_refusal(r: &WorkspaceRefusalRt) -> (String, String, String) {
    let (n1, n2) = r.unpack_names();
    let gid = match r.geom_kind {
        0 => "end_origin".to_string(),
        253 => "tcp_origin".to_string(),
        254 => "camera_mount".to_string(),
        255 => format!("capsule:{n1}"),
        n => format!("end_point:{n}"),
    };
    let (cid, limit, keepout_name): (String, String, Option<&str>) = match r.constraint_tag {
        0 => ("box_min_x".into(), "box_x_min".into(), None),
        1 => ("box_max_x".into(), "box_x_max".into(), None),
        2 => ("box_min_y".into(), "box_y_min".into(), None),
        3 => ("box_max_y".into(), "box_y_max".into(), None),
        4 => ("box_min_z".into(), "box_z_min".into(), None),
        5 => ("box_max_z".into(), "box_z_max".into(), None),
        6 => ("floor_z".into(), "floor".into(), None),
        7 => ("max_reach".into(), "reach".into(), None),
        8 => (format!("keepout:{n2}"), format!("keepout '{n2}'"), Some(n2)),
        9 => (format!("self_collision:{n1}:{n2}"), format!("self_collision '{n1}' vs '{n2}'"), None),
        _ => (String::new(), String::new(), None),
    };
    let mut reason = String::new();
    match r.kind {
        0 | 1 => {
            WorkspaceFailure::write_witness(&mut reason, WorkspaceWitness::Constraint {
                cid: &cid, gid: &gid, limit: &limit,
                value: f64::from_bits(r.value_bits), bound: f64::from_bits(r.bound_bits),
                prev_depth: f64::from_bits(r.prev_depth_bits), depth: f64::from_bits(r.depth_bits),
                start_outside: r.start_outside, conservative_bound: r.kind == 1,
            });
            (cid, gid, reason)
        }
        2 => {
            let b_cid = (!cid.is_empty()).then_some(cid.as_str());
            WorkspaceFailure::write_witness(&mut reason, WorkspaceWitness::SweepBudget {
                max_budget: r.max_budget as usize, blocking_cid: b_cid, blocking_gid: b_cid.map(|_| gid.as_str()),
                keepout_name, start_outside: r.start_outside,
            });
            ("sweep_budget".into(), gid, reason)
        }
        _ => {
            let msg = if r.geom_kind == 255 { n2 } else { n1 };
            WorkspaceFailure::write_witness(&mut reason, WorkspaceWitness::Invalid { reason: msg });
            ("invalid".into(), gid, reason)
        }
    }
}

/// Attaches structured workspace refusal fields to a JSON rejection reply off the RT thread.
#[cfg(not(target_arch = "wasm32"))]
pub fn attach_workspace_refusal_fields(reply: &mut serde_json::Value, detail: crate::rt_core::mailbox::RefusalDetail, phase: &str) -> bool {
    let crate::rt_core::mailbox::RefusalDetail::Workspace(ref r) = detail else { return false; };
    let (cid, gid, reason) = decode_workspace_refusal(r);
    for (k, v) in [("constraint_id", cid.as_str()), ("geometry_id", gid.as_str()), ("violation_phase", phase), ("reason", reason.as_str()), ("message", reason.as_str())] {
        reply[k] = serde_json::json!(v);
    }
    true
}
