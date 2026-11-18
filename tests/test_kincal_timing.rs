//! Cost of FK and IK per call on the built-in model, with and without a kinematic correction.
//! The corrected model precomputes its joint origins when the correction is set
//! (`ArmModel::with_correction`), so a correction must not make FK / IK slower.
//!
//! Numbers are meaningful in a release build:
//! `cargo test --release --test test_kincal_timing -- --nocapture`. The debug build only
//! prints them; the release build also checks the ratio.
use std::hint::black_box;
use std::time::Instant;

use armd::chain::ArmModel;
use armd::kinematics::{self as k, Correction, JointCorrection};

fn correction() -> Correction {
    let mut c = [JointCorrection::default(); 6];
    for (i, j) in c.iter_mut().enumerate().skip(1).take(4) {
        let s = i as f64;
        j.offset_rad = 0.01 * s;
        j.scale = 0.002 * s;
        j.tilt_rad = [0.003 * s, -0.002 * s];
        j.origin_m = [0.001 * s, -0.0005 * s, 0.0007];
    }
    c
}

fn poses(n: usize) -> Vec<[f64; 6]> {
    (0..n).map(|i| {
        let t = i as f64 * 0.37;
        [0.6 * t.sin(), -0.5 + 0.4 * (1.3 * t).cos(), 0.9 + 0.5 * (0.7 * t).sin(), 0.3 * (2.1 * t).cos(), -0.4 + 0.3 * t.sin(), 1.2 * (0.5 * t).cos()]
    }).collect()
}

/// ns per call of `f` over `qs`, best of 5 rounds.
fn ns_per_call(qs: &[[f64; 6]], rounds: usize, mut f: impl FnMut(&[f64; 6])) -> f64 {
    let mut best = f64::INFINITY;
    for _ in 0..5 {
        let t = Instant::now();
        for _ in 0..rounds { for q in qs { f(q); } }
        best = best.min(t.elapsed().as_nanos() as f64 / (rounds * qs.len()) as f64);
    }
    best
}

#[test]
fn corrected_fk_and_ik_cost_the_same_as_uncorrected() {
    let plain = ArmModel::REBOT_DEV_ARM;
    let corr = plain.with_correction(Some(&correction())).unwrap();
    let qs = poses(256);
    let fk_plain = ns_per_call(&qs, 200, |q| { black_box(plain.fk(black_box(q))); });
    let fk_corr = ns_per_call(&qs, 200, |q| { black_box(corr.fk(black_box(q))); });
    // IK: from a seed 0.05 rad off to the FK pose of each sample (same goals for both models
    // through their own FK, so both solve a reachable target)
    let (lo, hi) = ([-3.1, -2.0, -2.6, -3.1, -2.0, -6.2], [3.1, 2.0, 2.6, 3.1, 2.0, 6.2]);
    let ik = |m: &ArmModel| {
        let goals: Vec<([f64; 6], k::Pose)> = qs.iter().map(|q| (q.map(|v| v + 0.05), m.fk(q))).collect();
        let mut it = goals.iter().cycle();
        ns_per_call(&qs[..64], 5, |_| {
            let (seed, goal) = it.next().unwrap();
            black_box(m.solve_dls(black_box(goal), seed, &lo, &hi, k::PAR.w));
        })
    };
    let (ik_plain, ik_corr) = (ik(&plain), ik(&corr));
    let release = !cfg!(debug_assertions);
    println!("kinematics timing ({}): FK {fk_plain:.0} ns/call uncorrected, {fk_corr:.0} ns/call corrected ({:.2}x); \
        IK {ik_plain:.0} ns/call uncorrected, {ik_corr:.0} ns/call corrected ({:.2}x)",
        if release { "release" } else { "debug, not checked" }, fk_corr / fk_plain, ik_corr / ik_plain);
    if release {
        assert!(fk_corr <= fk_plain * 1.25, "corrected FK {fk_corr:.0} ns vs {fk_plain:.0} ns");
        assert!(ik_corr <= ik_plain * 1.25, "corrected IK {ik_corr:.0} ns vs {ik_plain:.0} ns");
    }
}
