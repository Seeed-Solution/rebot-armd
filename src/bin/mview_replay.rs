//! Re-run the multi-view reconstruction of a `measure_multiview` debug bundle on the host:
//! `cargo run --release --features mview-replay --bin mview-replay -- <bundle dir> [--points] [--json <file>] [--pose-refine auto|aruco|off]`.
//! Prints the stage counts and times, the per-view rotation refinement, the triangulated points
//! in the target box with their heights above the table, the measurement, and the run's own
//! stats from the bundle. `--json` writes the points (base frame, reference pixel, object flag)
//! and the refined cameras for offline analysis.
use serde_json::{json, Value};

fn pose_raw_value(
    mode: armd::measure_multiview::PoseRefine,
    pose: &Option<armd::mview_stereo::PoseRefinement>,
) -> Value {
    let Some(p) = pose else {
        return json!({"mode": mode.name(), "applied": false});
    };
    let views: Vec<Value> = p
        .views
        .iter()
        .map(|v| {
            json!({
                "status": v.status,
                "correction_mrad": v.correction_mrad,
                "rot_mrad": v.rot_mrad,
                "n_points": v.n_points,
                "n_marker_corners": v.n_marker_corners,
                "rms_before_px": v.rms_before_px,
                "rms_after_px": v.rms_after_px,
            })
        })
        .collect();
    json!({
        "mode": mode.name(),
        "applied": p.views.iter().any(|v| v.status == "refined"),
        "n_points": p.n_corrs,
        "n_marker_points": p.n_marker_corrs,
        "ms": p.ms,
        "views": views,
    })
}

#[cfg(feature = "mview-profile")]
fn build_profile_doc(
    bundle: &str,
    backend: &armd::mview_stereo::BackendDiagnostics,
    c: &armd::measure_multiview::Computed,
) -> Value {
    let pose_val = c
        .pose
        .as_ref()
        .and_then(|p| p.profile.as_ref())
        .map_or(Value::Null, |pp| {
            let denom = pp.total_ms;
            let share = |v: f64| if denom > 0.0 { v / denom } else { 0.0 };
            let integral_ms = pp.pass1.integral_ms + pp.pass2.integral_ms;
            let coarse_score_ms = pp.pass1.coarse_score_ms + pp.pass2.coarse_score_ms;
            let peak_select_ms = pp.pass1.peak_select_ms + pp.pass2.peak_select_ms;
            let refine_ms = pp.pass1.refine_ms + pp.pass2.refine_ms;
            let residual_ms = pp.pass1.residual_ms + pp.pass2.residual_ms;
            json!({
                "prep_ms": pp.prep_ms,
                "pass1_match_ms": pp.pass1_match_ms,
                "pass1_solve_ms": pp.pass1_solve_ms,
                "pass2_match_ms": pp.pass2_match_ms,
                "pass2_solve_ms": pp.pass2_solve_ms,
                "total_ms": pp.total_ms,
                "pass1": pp.pass1,
                "pass2": pp.pass2,
                "shares": {
                    "prep": share(pp.prep_ms),
                    "pass1_match": share(pp.pass1_match_ms),
                    "pass1_solve": share(pp.pass1_solve_ms),
                    "pass2_match": share(pp.pass2_match_ms),
                    "pass2_solve": share(pp.pass2_solve_ms),
                    "integral": share(integral_ms),
                    "coarse_score": share(coarse_score_ms),
                    "peak_select": share(peak_select_ms),
                    "refine": share(refine_ms),
                    "residual": share(residual_ms),
                },
            })
        });
    json!({
        "bundle": bundle,
        "backend": backend,
        "pose": pose_val,
        "reconstruction": {
            "stats": c.rec.stats,
            "ncc_profile": c.rec.profile,
        },
    })
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let opt = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned();
    let json_out = opt("--json");
    let mode = opt("--pose-refine").map(|m| armd::measure_multiview::PoseRefine::parse(&m).unwrap_or_else(|| { eprintln!("--pose-refine auto|aruco|off"); std::process::exit(2); }));
    let Some(dir) = args.iter().enumerate().find(|(i, a)| !a.starts_with("--") && (*i == 0 || !["--json", "--pose-refine"].contains(&args[i - 1].as_str()))).map(|(_, a)| a.clone()) else {
        eprintln!("usage: mview_replay <bundle dir> [--points] [--json <file>] [--pose-refine auto|aruco|off]");
        std::process::exit(2);
    };
    armd::mview_stereo::reset_kernel_counts();
    let (b, c) = match armd::measure_multiview::replay_bundle_with(std::path::Path::new(&dir), mode) {
        Ok(r) => r,
        Err(e) => { eprintln!("{e}"); std::process::exit(1); }
    };
    let backend_diag = armd::mview_stereo::backend_diagnostics();
    println!("ncc backend {}", serde_json::to_string(&backend_diag).unwrap());
    println!("views {}, reference {}, box {:?}, region {:?}, table_z {:.4} m", b.views.len(), b.reference, b.ref_box_px, c.region_px, b.table_z_m);
    println!("replay stats {}", serde_json::to_string(&c.rec.stats).unwrap());
    println!("device stats {}", b.doc["stats"]);
    let mode_eff = mode.unwrap_or(b.pose_refine);
    let (pose, _) = armd::measure_multiview::pose_value(mode_eff, &c.pose);
    let pose_raw = pose_raw_value(mode_eff, &c.pose);
    println!("pose refine {pose}");
    println!("in box {}", armd::measure_multiview::box_heights(&c.rec.points, b.table_z_m));
    println!("outcomes {}", armd::measure_multiview::outcome_counts(&c.rec, b.ref_box_px));
    match &c.measure {
        Ok(m) => println!("measurement height {:.1} mm ({}), top {:.4} m, {} points ({} edge, {} top), width {:.1} mm, length {:.1} mm, yaw {:.1} deg, rms {:.2} px",
            m.height_m * 1000.0, m.top_method, m.top_z_m, m.n_points, m.n_edge_points, m.n_top_points, m.width_m * 1000.0, m.length_m * 1000.0, m.yaw_deg, m.reproj_rms_px),
        Err(w) => println!("measurement: {w}"),
    }
    println!("table check n {} median {:?} mm |median| {:?} mm; table plane {}", c.table.n_points, c.table.median_mm, c.table.median_abs_mm, armd::measure_multiview::plane_value(&c.plane));
    if args.iter().any(|a| a == "--points") {
        for q in c.rec.points.iter().filter(|q| q.in_box) {
            println!("point ref_px {:?} h {:+.1} mm edge {} consistent {} reproj {:.2} px", q.ref_px, (q.x[2] - b.table_z_m) * 1000.0, q.edge, q.consistent, q.reproj_rms_px);
        }
    }
    #[cfg(feature = "mview-profile")]
    let profile_doc = {
        let doc = build_profile_doc(&dir, &backend_diag, &c);
        println!("profile {}", serde_json::to_string(&doc).unwrap());
        doc
    };
    if let Some(path) = json_out {
        let obj: Vec<usize> = c.measure.as_ref().map(|m| m.points.clone()).unwrap_or_default();
        let pts: Vec<serde_json::Value> = c.rec.points.iter().enumerate().map(|(i, q)| json!({
            "x": q.x, "h_mm": (q.x[2] - b.table_z_m) * 1000.0, "ref_px": q.ref_px, "in_box": q.in_box, "edge": q.edge,
            "consistent": q.consistent, "rms": q.reproj_rms_px, "object": obj.contains(&i),
        })).collect();
        let m = c.measure.as_ref().map_or_else(|w| json!(w), |m| json!({"h_mm": m.height_m * 1000.0, "w_mm": m.width_m * 1000.0, "l_mm": m.length_m * 1000.0,
            "n": m.n_points, "n_top": m.n_top_points, "rms": m.reproj_rms_px, "top_method": m.top_method}));
        let cams: Vec<serde_json::Value> = c.pose.as_ref().map(|p| p.cams.clone()).unwrap_or_else(|| b.views.iter().map(|v| v.cam).collect())
            .iter().map(|k| json!({"r": k.r, "c": k.c})).collect();
        #[cfg_attr(not(feature = "mview-profile"), allow(unused_mut))]
        let mut doc = json!({
            "table_z": b.table_z_m,
            "box": b.ref_box_px,
            "measure": m,
            "pose_refine": pose_raw,
            "cams": cams,
            "points": pts,
            "stats": c.rec.stats,
            "backend": backend_diag,
        });
        #[cfg(feature = "mview-profile")]
        {
            doc["profile"] = profile_doc;
        }
        std::fs::write(&path, serde_json::to_vec(&doc).unwrap()).unwrap_or_else(|e| { eprintln!("{path}: {e}"); std::process::exit(1); });
    }
}
