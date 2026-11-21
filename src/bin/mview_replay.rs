//! Re-run the multi-view reconstruction of a `measure_multiview` debug bundle on the host:
//! `cargo run --release --features mview-replay --bin mview-replay -- <bundle dir> [--points]`.
//! Prints the stage counts and times, the triangulated points in the target box with their
//! heights above the table, the measurement, and the run's own stats from the bundle.
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(dir) = args.iter().find(|a| !a.starts_with("--")) else {
        eprintln!("usage: mview_replay <bundle dir> [--points]");
        std::process::exit(2);
    };
    let (b, c) = match armd::measure_multiview::replay_bundle(std::path::Path::new(dir)) {
        Ok(r) => r,
        Err(e) => { eprintln!("{e}"); std::process::exit(1); }
    };
    println!("views {}, reference {}, box {:?}, region {:?}, table_z {:.4} m", b.views.len(), b.reference, b.ref_box_px, c.region_px, b.table_z_m);
    println!("replay stats {}", serde_json::to_string(&c.rec.stats).unwrap());
    println!("device stats {}", b.doc["stats"]);
    println!("in box {}", armd::measure_multiview::box_heights(&c.rec.points, b.table_z_m));
    println!("outcomes {}", armd::measure_multiview::outcome_counts(&c.rec, b.ref_box_px));
    match &c.measure {
        Ok(m) => println!("measurement height {:.1} mm, top {:.4} m, {} points ({} edge, {} top), width {:.1} mm, length {:.1} mm, yaw {:.1} deg",
            m.height_m * 1000.0, m.top_z_m, m.n_points, m.n_edge_points, m.n_top_points, m.width_m * 1000.0, m.length_m * 1000.0, m.yaw_deg),
        Err(w) => println!("measurement: {w}"),
    }
    println!("table check n {} median {:?} mm |median| {:?} mm", c.table.n_points, c.table.median_mm, c.table.median_abs_mm);
    if args.iter().any(|a| a == "--points") {
        for q in c.rec.points.iter().filter(|q| q.in_box) {
            println!("point ref_px {:?} h {:+.1} mm edge {} consistent {} reproj {:.2} px", q.ref_px, (q.x[2] - b.table_z_m) * 1000.0, q.edge, q.consistent, q.reproj_rms_px);
        }
    }
}
