//! Motion-stereo geometry (src/mview_stereo.rs) on rendered images: a textured table with a box,
//! three views at a known lateral baseline, Gaussian intensity noise (sigma 2 grey levels) and one
//! of two pixel-noise models:
//! - random: Gaussian noise of 1 px per axis added to every matched pixel of every view before the
//!   triangulation (`reconstruct_perturbed`; localisation noise; the acceptance case, height
//!   error < 2 mm);
//! - systematic: each view's image rendered with its principal point shifted by 1 px in a random
//!   direction (a 1 px pose / calibration error per view; reported: it biases table and object
//!   points alike, which `table_check` shows).
mod render;

use armd::kinematics;
use armd::mview_stereo::{self, Params, View};
use render::{BoxObj, Noise, Rng, Scene};

const SIZE: [f64; 2] = [640.0, 360.0];
const FX: f64 = 804.0;

struct Case { height: f64, baseline: f64, distance: f64, seed: u64, px_noise: f64, shift_px: f64 }

struct Out { m: mview_stereo::ObjectMeasure, t: mview_stereo::TableCheck, stats: mview_stereo::Stats, truth: BoxObj, ms: f64 }

/// Views at chord offsets -B/2, 0, +B/2 around the box's top centre, start camera `distance` from
/// it at 55 deg elevation from the base side.
fn run(case: &Case) -> Out {
    let b = BoxObj { xy: [0.30, 0.01], yaw: 20f64.to_radians(), size: [0.050, 0.030, case.height] };
    let scene = Scene::new(0.0, vec![b]);
    let pivot = [b.xy[0], b.xy[1], case.height * 0.5];
    let el = 55f64.to_radians();
    let c0 = [pivot[0] - case.distance * el.cos(), pivot[1], pivot[2] + case.distance * el.sin()];
    let cam0 = render::look_at(c0, pivot, FX, SIZE);
    let mut rng = Rng(case.seed * 7919 + 1);
    let offsets = [-case.baseline / 2.0, 0.0, case.baseline / 2.0];
    let views: Vec<View> = offsets.iter().enumerate().map(|(i, &o)| {
        let cam = render::orbit(&cam0, pivot, o);
        let a = 2.0 * std::f64::consts::PI * rng.next_f64();
        let noise = Noise { shift_px: [case.shift_px * a.cos(), case.shift_px * a.sin()], sigma: 2.0 };
        View { img: render::render(&scene, &cam, SIZE[0] as usize, SIZE[1] as usize, 2, noise, case.seed * 31 + i as u64), cam }
    }).collect();
    let bx = render::box_px(&scene, &b, &views[1].cam);
    let (w, h) = (bx[2] - bx[0], bx[3] - bx[1]);
    let region = [bx[0] - 0.25 * w - 12.0, bx[1] - 0.25 * h - 12.0, bx[2] + 0.25 * w + 12.0, bx[3] + 0.25 * h + 12.0];
    let p = Params::new(0.0);
    let t0 = std::time::Instant::now();
    let mut nrng = Rng(case.seed * 104_729 + 3);
    let sd = case.px_noise;
    let rec = mview_stereo::reconstruct_perturbed(&views, 1, region, bx, &p, |_, q| [q[0] + sd * nrng.gauss(), q[1] + sd * nrng.gauss()]);
    let m = mview_stereo::measure(&rec.points, &p).unwrap_or_else(|e| panic!("{e}: {:?}", rec.stats));
    let ms = t0.elapsed().as_secs_f64() * 1000.0;
    Out { t: mview_stereo::table_check(&rec.points, &p), m, stats: rec.stats, truth: b, ms }
}

fn yaw_err(a: f64, b: f64) -> f64 { ((a - b + 90.0).rem_euclid(180.0) - 90.0).abs() }

fn table_row(height: f64, seed: u64, o: &Out) -> f64 {
    let err = (o.m.height_m - height) * 1000.0;
    println!("| {:.0} | {seed} | {:.2} | {:+.2} | {:+.1} | {:+.1} | {:.1} | {} | {:.2} | {} | {:.2} | {:.2} | {} | {} | {} | {} | {} | {:.0} |",
        height * 1000.0, o.m.height_m * 1000.0, err, (o.m.length_m - o.truth.size[0]) * 1000.0, (o.m.width_m - o.truth.size[1]) * 1000.0,
        yaw_err(o.m.yaw_deg, o.truth.yaw.to_degrees()), o.m.n_points, o.m.reproj_rms_px, o.t.n_points,
        o.t.median_abs_mm.unwrap_or(f64::NAN), o.t.p90_abs_mm.unwrap_or(f64::NAN),
        o.stats.features, o.stats.no_peak, o.stats.ambiguous, o.stats.lr_rejected, o.stats.reproj_rejected, o.ms);
    err
}

const HEADER: &str = "| height mm | seed | measured mm | err mm | length err mm | width err mm | yaw err deg | n_points | reproj rms px | table n | table median mm | table p90 mm | features | no_peak | ambiguous | lr_rej | reproj_rej | compute ms |";

#[test]
fn heights_20_40_80_mm_within_2_mm_at_8_cm_baseline_35_cm_with_1_px_noise() {
    println!("random 1 px noise\n{HEADER}");
    let mut worst: f64 = 0.0;
    for height in [0.020, 0.040, 0.080] {
        for seed in 1..=3 {
            let o = run(&Case { height, baseline: 0.08, distance: 0.35, seed, px_noise: 1.0, shift_px: 0.0 });
            let err = table_row(height, seed, &o);
            worst = worst.max(err.abs());
            assert!(err.abs() < 2.0, "height {height}: error {err:.2} mm");
            assert!(o.m.n_points >= 20, "{:?}", o.stats);
            let c = kinematics::norm(&kinematics::sub(&[o.m.centroid_m[0], o.m.centroid_m[1], 0.0], &[o.truth.xy[0], o.truth.xy[1], 0.0]));
            assert!(c < 0.005, "centroid off by {c:.4} m");
            assert!(yaw_err(o.m.yaw_deg, o.truth.yaw.to_degrees()) < 10.0, "yaw {}", o.m.yaw_deg);
            assert!(o.t.n_points >= 20 && o.t.median_mm.unwrap().abs() < 1.0 && o.t.median_abs_mm.unwrap() < 3.0, "{:?}", o.t);
        }
    }
    println!("worst |height error| {worst:.2} mm");
}

#[test]
fn a_1_px_pose_error_per_view_biases_object_and_table_alike() {
    // reported, bounded loosely: the table check shows the same bias as the object height
    println!("systematic 1 px shift per view\n{HEADER}");
    for height in [0.020, 0.040, 0.080] {
        for seed in 1..=3 {
            let o = run(&Case { height, baseline: 0.08, distance: 0.35, seed, px_noise: 0.0, shift_px: 1.0 });
            let err = table_row(height, seed, &o);
            assert!(err.abs() < 5.0, "height {height}: error {err:.2} mm");
            // the bias is common to table and object: relative to the triangulated table it cancels
            let rel = err - o.t.median_mm.unwrap();
            println!("  height - table median: error {rel:+.2} mm");
            // 2 mm: the top-cluster estimate moved the 80 mm / seed 3 case from -1.46 to -1.55 mm
            assert!(rel.abs() < 2.0, "height {height}: relative error {rel:.2} mm");
        }
    }
}

#[test]
fn without_noise_the_height_error_is_sub_millimetre() {
    for height in [0.020, 0.080] {
        let o = run(&Case { height, baseline: 0.08, distance: 0.35, seed: 4, px_noise: 0.0, shift_px: 0.0 });
        let err = (o.m.height_m - height) * 1000.0;
        assert!(err.abs() < 1.0, "height {height}: {err:.2} mm ({:?})", o.stats);
        assert!(o.m.reproj_rms_px < 0.5, "{}", o.m.reproj_rms_px);
    }
}

#[test]
fn triangulated_points_lie_on_the_rendered_surfaces() {
    // epipolar matching check: every accepted point is within 3 mm of the table or the box surface
    let height = 0.040;
    let b = BoxObj { xy: [0.30, 0.01], yaw: 20f64.to_radians(), size: [0.050, 0.030, height] };
    let scene = Scene::new(0.0, vec![b]);
    let pivot = [0.30, 0.01, 0.02];
    let cam0 = render::look_at([0.10, 0.01, 0.31], pivot, FX, SIZE);
    let views: Vec<View> = [-0.04, 0.0, 0.04].iter().enumerate().map(|(i, &o)| {
        let cam = render::orbit(&cam0, pivot, o);
        View { img: render::render(&scene, &cam, 640, 360, 2, Noise { sigma: 2.0, ..Noise::default() }, 50 + i as u64), cam }
    }).collect();
    let bx = render::box_px(&scene, &b, &views[1].cam);
    let region = [bx[0] - 40.0, bx[1] - 40.0, bx[2] + 40.0, bx[3] + 40.0];
    let rec = mview_stereo::reconstruct(&views, 1, region, bx, &Params::new(0.0));
    assert!(rec.points.len() > 100, "{:?}", rec.stats);
    let (c, s) = (b.yaw.cos(), b.yaw.sin());
    let mut worst: f64 = 0.0;
    for q in &rec.points {
        let rel = [q.x[0] - b.xy[0], q.x[1] - b.xy[1]];
        let (u, v) = (c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1]);
        let (du, dv) = ((u.abs() - b.size[0] / 2.0).max(0.0), (v.abs() - b.size[1] / 2.0).max(0.0));
        let outside = du.hypot(dv);
        // distance to the box surface (outside: to the closest face; inside the footprint: to the top or a side)
        let d_box = if outside > 0.0 { outside.hypot((q.x[2] - height).max(0.0)) } else {
            (q.x[2] - height).abs().min(b.size[0] / 2.0 - u.abs()).min(b.size[1] / 2.0 - v.abs())
        };
        let d = d_box.min(q.x[2].abs());
        worst = worst.max(d);
        assert!(d < 0.003, "point {:?} is {:.1} mm from every surface", q.x, d * 1000.0);
    }
    println!("{} points, worst surface distance {:.2} mm, {:?}", rec.points.len(), worst * 1000.0, rec.stats);
}

#[test]
fn repeated_texture_along_the_epipolar_lines_is_rejected_as_ambiguous() {
    // a dot pattern of 4 mm period: each patch matches at every period along the epipolar line,
    // so the ratio test must reject the matches instead of returning false depths
    let mut scene = Scene::new(0.0, vec![]);
    scene.periodic = true;
    let pivot = [0.30, 0.0, 0.0];
    let cam0 = render::look_at([0.10, 0.0, 0.29], pivot, FX, SIZE);
    let views: Vec<View> = [-0.04, 0.0, 0.04].iter().enumerate().map(|(i, &o)| {
        let cam = render::orbit(&cam0, pivot, o);
        View { img: render::render(&scene, &cam, 640, 360, 2, Noise { sigma: 0.5, ..Noise::default() }, 70 + i as u64), cam }
    }).collect();
    let rec = mview_stereo::reconstruct(&views, 1, [200.0, 120.0, 440.0, 240.0], [280.0, 150.0, 360.0, 210.0], &Params::new(0.0));
    let bad = rec.points.iter().filter(|q| q.x[2].abs() > 0.003).count();
    println!("{:?}, {bad} points off the table", rec.stats);
    assert!(rec.stats.features > 50, "{:?}", rec.stats);
    assert!(rec.stats.ambiguous > rec.stats.corners / 2, "{:?}", rec.stats);
    assert!(bad * 10 <= rec.stats.features.max(1), "{bad} false points: {:?}", rec.stats);
}

/// Compute time of one measurement (3 and 5 views) and of a 640x640 snapshot decode. On the
/// host: `cargo test --release --test test_mview_stereo bench -- --ignored --nocapture`.
#[test]
#[ignore]
fn bench_compute_and_decode() {
    let b = BoxObj { xy: [0.30, 0.01], yaw: 0.3, size: [0.05, 0.03, 0.04] };
    let scene = Scene::new(0.0, vec![b]);
    let pivot = [0.30, 0.01, 0.02];
    let el = 55f64.to_radians();
    let cam0 = render::look_at([pivot[0] - 0.35 * el.cos(), pivot[1], pivot[2] + 0.35 * el.sin()], pivot, FX, SIZE);
    use armd::measure_multiview::{view_offsets, Axes};
    for (n, axes, motion) in [(3usize, Axes::X, Motion::Orbit), (5, Axes::X, Motion::Orbit), (5, Axes::Xy, Motion::Translate), (5, Axes::Xy, Motion::Orbit)] {
        let offsets = view_offsets(n, 0.08, axes);
        let views: Vec<View> = offsets.iter().enumerate().map(|(i, o)| {
            let cam = match motion { Motion::Orbit => render::orbit_xy(&cam0, pivot, *o), Motion::Translate => render::translate(&cam0, *o) };
            View { img: render::render(&scene, &cam, 640, 360, 2, Noise { sigma: 2.0, ..Noise::default() }, 90 + i as u64), cam }
        }).collect();
        let reference = offsets.iter().position(|o| o[0] == 0.0 && o[1] == 0.0).unwrap();
        let bx = render::box_px(&scene, &b, &views[reference].cam);
        let (w, h) = (bx[2] - bx[0], bx[3] - bx[1]);
        let region = [bx[0] - 0.25 * w - 12.0, bx[1] - 0.25 * h - 12.0, bx[2] + 0.25 * w + 12.0, bx[3] + 0.25 * h + 12.0];
        let p = Params::new(0.0);
        let mut times = Vec::new();
        for _ in 0..5 {
            let t0 = std::time::Instant::now();
            let rec = mview_stereo::reconstruct(&views, reference, region, bx, &p);
            let m = mview_stereo::measure(&rec.points, &p).unwrap();
            times.push(t0.elapsed().as_secs_f64() * 1000.0);
            assert!((m.height_m - 0.04).abs() < 0.002);
        }
        times.sort_by(f64::total_cmp);
        println!("{n} views {} {motion:?}: compute median {:.1} ms (min {:.1}, max {:.1})", axes.name(), times[2], times[0], times[4]);
        let rec = mview_stereo::reconstruct(&views, reference, region, bx, &p);
        let st = rec.stats;
        println!("  stages ms: pyramid {:.1}, features {:.1}, sweep {:.1}, refine {:.1}, triangulate {:.2}, total {:.1}; features {} (corners {}, edges {}), points {}",
            st.pyramid_ms, st.features_ms, st.sweep_ms, st.refine_ms, st.triangulate_ms, st.total_ms, st.features, st.corners, st.edges, st.points);
    }
    // snapshot decode: the frame letterboxed into 640x640, JPEG quality 90
    let img = render::render(&scene, &cam0, 640, 360, 1, Noise { sigma: 2.0, ..Noise::default() }, 7);
    let mut sq = vec![0u8; 640 * 640];
    sq[140 * 640..500 * 640].copy_from_slice(&img.px);
    let mut jpeg = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg, 90).encode(&sq, 640, 640, jpeg_encoder::ColorType::Luma).unwrap();
    let mut rgb = Vec::with_capacity(640 * 640 * 3);
    for v in &sq { rgb.extend_from_slice(&[*v, *v, *v]); }
    let mut jpeg_rgb = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg_rgb, 90).encode(&rgb, 640, 640, jpeg_encoder::ColorType::Rgb).unwrap();
    for (what, data) in [("luma", &jpeg), ("ycbcr 4:2:0", &jpeg_rgb)] {
        let t0 = std::time::Instant::now();
        for _ in 0..20 { armd::measure_multiview::decode_frame(data, SIZE).unwrap(); }
        println!("decode 640x640 {what} ({} KB): {:.1} ms", data.len() / 1024, t0.elapsed().as_secs_f64() * 1000.0 / 20.0);
    }
}

#[derive(Clone, Copy, Debug)]
enum Motion { Orbit, Translate }

/// Plain (uniform) box cases: start camera 0.35 m from the box centre at 55 deg elevation,
/// offsets in the image plane, reference = the start view (offset [0, 0], index 0).
fn plain_case(height: f64, plain_table: bool, motion: Motion, offsets: &[[f64; 2]], seed: u64, px_noise: f64) -> (Result<mview_stereo::ObjectMeasure, &'static str>, mview_stereo::Stats, mview_stereo::TableCheck) {
    let b = BoxObj { xy: [0.30, 0.01], yaw: 20f64.to_radians(), size: [0.050, 0.030, height] };
    let mut scene = Scene::new(0.0, vec![b]);
    scene.plain_boxes = true;
    scene.plain_table = plain_table;
    let pivot = [b.xy[0], b.xy[1], height * 0.5];
    let el = 55f64.to_radians();
    let cam0 = render::look_at([pivot[0] - 0.35 * el.cos(), pivot[1], pivot[2] + 0.35 * el.sin()], pivot, FX, SIZE);
    let views: Vec<View> = offsets.iter().enumerate().map(|(i, o)| {
        let cam = match motion { Motion::Orbit => render::orbit_xy(&cam0, pivot, *o), Motion::Translate => render::translate(&cam0, *o) };
        View { img: render::render(&scene, &cam, 640, 360, 2, Noise { sigma: 2.0, ..Noise::default() }, seed * 31 + i as u64), cam }
    }).collect();
    let bx = render::box_px(&scene, &b, &views[0].cam);
    let (w, h) = (bx[2] - bx[0], bx[3] - bx[1]);
    let region = [bx[0] - 0.25 * w - 12.0, bx[1] - 0.25 * h - 12.0, bx[2] + 0.25 * w + 12.0, bx[3] + 0.25 * h + 12.0];
    let p = Params::new(0.0);
    let mut nrng = Rng(seed * 104_729 + 3);
    let rec = mview_stereo::reconstruct_perturbed(&views, 0, region, bx, &p, |_, q| [q[0] + px_noise * nrng.gauss(), q[1] + px_noise * nrng.gauss()]);
    if std::env::var("MV_POINTS").is_ok() {
        // outcome per feature class: edge / corner, true surface height band (render ray cast)
        let mut tally: std::collections::BTreeMap<(bool, &str, &str), usize> = Default::default();
        for (f, o) in &rec.outcomes {
            let z = render::hit(&scene, &views[0].cam, [f.x as f64, f.y as f64]).map_or(-1.0, |x| x[2]);
            let band = if z < 0.002 { "table" } else if z > height - 0.002 { "top" } else { "side" };
            *tally.entry((f.normal.is_some(), band, o)).or_default() += 1;
        }
        println!("{tally:?}");
    }
    (mview_stereo::measure(&rec.points, &p), rec.stats, mview_stereo::table_check(&rec.points, &p))
}

const X3: [[f64; 2]; 3] = [[0.0, 0.0], [-0.04, 0.0], [0.04, 0.0]];
const XY5: [[f64; 2]; 5] = [[0.0, 0.0], [0.04, 0.0], [0.0, 0.04], [-0.04, 0.0], [0.0, -0.04]];

#[test]
fn plain_boxes_are_measured_from_their_edges() {
    for px_noise in [0.0, 1.0] {
        println!("plain (uniform grey) 50 x 30 mm box, pixel noise {px_noise} px per axis on the matches\n| table | motion | views | height mm | measured mm | err mm | n_points | edge points | top points | why | features (corners/edges) | edge_no_view | ambiguous | lr_rej | table n |");
        let mut worst: f64 = 0.0;
        for (plain_table, motion, offs, name) in [
            (false, Motion::Orbit, &X3[..], "x3"), (false, Motion::Translate, &XY5[..], "xy5"), (false, Motion::Orbit, &XY5[..], "xy5"),
            (true, Motion::Orbit, &X3[..], "x3"), (true, Motion::Translate, &XY5[..], "xy5"), (true, Motion::Orbit, &XY5[..], "xy5"),
        ] {
            for height in [0.020, 0.040, 0.080] {
                let (m, st, t) = plain_case(height, plain_table, motion, offs, 5, px_noise);
                if std::env::var("MV_DEBUG").is_ok() { println!("{st:?}"); }
                let (meas, err, n, ne, nt, why) = match &m {
                    Ok(m) => (format!("{:.2}", m.height_m * 1000.0), format!("{:+.2}", (m.height_m - height) * 1000.0), m.n_points, m.n_edge_points, m.n_top_points, "-"),
                    Err(w) => ("-".into(), "-".into(), 0, 0, 0, *w),
                };
                println!("| {} | {motion:?} | {name} | {:.0} | {meas} | {err} | {n} | {ne} | {nt} | {why} | {} ({}/{}) | {} | {} | {} | {} |",
                    if plain_table { "plain" } else { "textured" }, height * 1000.0, st.features, st.corners, st.edges, st.edge_no_view, st.ambiguous, st.lr_rejected, t.n_points);
                // without pixel noise every case measures; with it a case may end too_few_points
                if px_noise == 0.0 { assert!(m.is_ok(), "{motion:?} {name} {height}: {st:?}"); }
                let Ok(m) = m else { continue };
                worst = worst.max((m.height_m - height).abs());
                assert!((m.height_m - height).abs() < 0.002, "{motion:?} {name} {height}: {m:?}");
            }
        }
        println!("worst |height error| {:.2} mm", worst * 1000.0);
    }
}

/// A scene like the real-arm run of 2026-11-22 (an elongated object filling ~80 % of the image
/// height at Z ~0.47 m, fx 804, 3 views orbit x and 5 views orbit xy, JPEG q85), written as replay
/// bundles to `$MVIEW_BUNDLE_OUT/{x3,xy5}` for `src/bin/mview_replay.rs` (host and riscv / qemu).
#[test]
#[ignore]
fn write_realistic_bundles() {
    let Some(out) = std::env::var_os("MVIEW_BUNDLE_OUT") else { return };
    // long side along base x (away from the arm): it spans the image height, foreshortened
    let b = BoxObj { xy: [0.62, 0.08], yaw: 0.0, size: [0.22, 0.05, 0.035] };
    let scene = Scene::new(0.0098, vec![b]);
    let pivot = [b.xy[0], b.xy[1], 0.0098 + 0.02];
    let el = 41f64.to_radians();
    let cam0 = render::look_at([pivot[0] - 0.47 * el.cos(), pivot[1], pivot[2] + 0.47 * el.sin()], pivot, FX, SIZE);
    use armd::measure_multiview::{view_offsets, write_replay_bundle, Axes};
    for (name, n, axes) in [("x3", 3usize, Axes::X), ("xy5", 5, Axes::Xy)] {
        let offsets = view_offsets(n, 0.08, axes);
        let cams: Vec<mview_stereo::Camera> = offsets.iter().map(|o| render::orbit_xy(&cam0, pivot, *o)).collect();
        let jpegs: Vec<Vec<u8>> = cams.iter().enumerate().map(|(i, c)| {
            let img = render::render(&scene, c, 640, 360, 2, Noise { sigma: 2.0, ..Noise::default() }, 300 + i as u64);
            let mut sq = vec![0u8; 640 * 640];
            sq[140 * 640..500 * 640].copy_from_slice(&img.px);
            let mut j = Vec::new();
            jpeg_encoder::Encoder::new(&mut j, 85).encode(&sq, 640, 640, jpeg_encoder::ColorType::Luma).unwrap();
            j
        }).collect();
        let reference = offsets.iter().position(|o| o[0] == 0.0 && o[1] == 0.0).unwrap();
        let bx = render::box_px(&scene, &b, &cams[reference]);
        println!("{name}: box {bx:?} ({:.0} % of the height)", (bx[3] - bx[1]) / 3.6);
        write_replay_bundle(&std::path::Path::new(&out).join(name), &jpegs, &cams, SIZE, reference, bx, 0.0098).unwrap();
    }
}

/// Thresholds against a scene like the real-arm run (2026-11-22): the object fills ~80 % of the
/// image height at Z ~0.47 m, dim light (grey levels compressed to 20 + 0.35 x), a per-view gain
/// change of up to 8 % (gloss), JPEG q75, 3 views orbit x; a plain or weakly textured object on a
/// noise-textured or periodic (board-like) table. Prints the stage counts for `ncc_min` 0.8 / 0.7.
#[test]
#[ignore]
fn thresholds_on_a_dim_glossy_scene() {
    use armd::measure_multiview::{decode_frame, view_offsets, Axes};
    println!("| table | object | ncc_min | height err mm | n_points | in box n / above 5 mm | no_peak | ambiguous | refine | lr | reproj |");
    for (periodic, plain) in [(false, false), (false, true), (true, false), (true, true)] {
        let b = BoxObj { xy: [0.62, 0.08], yaw: 0.0, size: [0.22, 0.05, 0.035] };
        let mut scene = Scene::new(0.0098, vec![b]);
        scene.periodic = periodic;
        scene.plain_boxes = plain;
        let pivot = [b.xy[0], b.xy[1], 0.0098 + 0.02];
        let el = 41f64.to_radians();
        let cam0 = render::look_at([pivot[0] - 0.47 * el.cos(), pivot[1], pivot[2] + 0.47 * el.sin()], pivot, FX, SIZE);
        let offsets = view_offsets(3, 0.08, Axes::X);
        let views: Vec<View> = offsets.iter().enumerate().map(|(i, o)| {
            let cam = render::orbit_xy(&cam0, pivot, *o);
            let img = render::render(&scene, &cam, 640, 360, 2, Noise { sigma: 2.0, ..Noise::default() }, 400 + i as u64);
            let gain = [1.0, 1.08, 0.94][i];
            let px: Vec<u8> = img.px.iter().map(|&v| (20.0 + 0.35 * gain * f64::from(v)).round().clamp(0.0, 255.0) as u8).collect();
            let mut j = Vec::new();
            jpeg_encoder::Encoder::new(&mut j, 75).encode(&px, 640, 360, jpeg_encoder::ColorType::Luma).unwrap();
            View { img: decode_frame(&j, SIZE).unwrap(), cam }
        }).collect();
        let bx = render::box_px(&scene, &b, &views[1].cam);
        let region = armd::measure_multiview::feature_region(bx);
        for ncc_min in [0.8, 0.7] {
            let mut p = Params::new(0.0098);
            p.ncc_min = ncc_min;
            let rec = mview_stereo::reconstruct(&views, 1, region, bx, &p);
            let st = rec.stats;
            let ib = armd::measure_multiview::box_heights(&rec.points, 0.0098);
            let m = mview_stereo::measure(&rec.points, &p);
            println!("| {} | {} | {ncc_min} | {} | {} | {} / {} | {} | {} | {} | {} | {} |", if periodic { "periodic" } else { "noise" }, if plain { "plain" } else { "textured" },
                m.as_ref().map_or_else(|w| w.to_string(), |m| format!("{:+.2}", (m.height_m - 0.035) * 1000.0)), m.as_ref().map_or(0, |m| m.n_points),
                ib["n"], ib["n_above"], st.no_peak, st.ambiguous, st.refine_rejected, st.lr_rejected, st.reproj_rejected);
            if std::env::var("MV_DEBUG").is_ok() { println!("  {}", armd::measure_multiview::outcome_counts(&rec, bx)); }
        }
    }
}
