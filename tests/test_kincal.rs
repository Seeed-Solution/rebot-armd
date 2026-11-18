// The limits use the same rounded angle values as the profile.
#![allow(clippy::approx_constant)]
//! Kinematic fine calibration of the built-in chain (src/kinematics.rs `frames_corrected`,
//! `ArmModel::with_correction`): zero correction = the URDF chain bit for bit, agreement with
//! the reference implementation kincal2.mjs `fkP` (fixture), exact Jacobian and DLS round trips.
use armd::chain::ArmModel;
use armd::kinematics::{self as k, Correction, JointCorrection, Mat3};
use serde_json::Value;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn uni(&mut self, lo: f64, hi: f64) -> f64 { lo + (hi - lo) * self.next() }
}

/// Model-angle limits of the B601-DM profile.
const LIM: [[f64; 2]; 6] = [[-3.14159, 3.14159], [-2.0944, 2.0944], [-2.61799, 2.61799], [-3.14159, 3.14159], [-2.0944, 2.0944], [-6.28318, 6.28318]];

fn max_diff_m(a: &Mat3, b: &Mat3) -> f64 {
    (0..3).flat_map(|i| (0..3).map(move |j| (i, j))).map(|(i, j)| (a[i][j] - b[i][j]).abs()).fold(0.0, f64::max)
}

/// A correction near the bounds, like the one fitted on the real B601-DM (J2..J5 only).
fn real_like() -> Correction {
    let d = std::f64::consts::PI / 180.0;
    let mut c = [JointCorrection::default(); 6];
    c[1] = JointCorrection { offset_rad: 1.2 * d, scale: 0.012, tilt_rad: [2.5 * d, -0.8 * d], origin_m: [0.004, -0.006, 0.002] };
    c[2] = JointCorrection { offset_rad: -2.0 * d, scale: -0.02, tilt_rad: [0.5 * d, 1.1 * d], origin_m: [-0.012, 0.003, 0.008] };
    c[3] = JointCorrection { offset_rad: 3.1 * d, scale: 0.03, tilt_rad: [-1.5 * d, 0.7 * d], origin_m: [0.009, 0.011, -0.004] };
    c[4] = JointCorrection { offset_rad: -4.0 * d, scale: -0.05, tilt_rad: [2.9 * d, -2.9 * d], origin_m: [-0.02, 0.015, 0.025] };
    c
}

fn random_correction(rng: &mut Rng) -> Correction {
    let d = std::f64::consts::PI / 180.0;
    std::array::from_fn(|_| JointCorrection {
        offset_rad: rng.uni(-10.0, 10.0) * d, scale: rng.uni(-0.08, 0.08),
        tilt_rad: [rng.uni(-3.0, 3.0) * d, rng.uni(-3.0, 3.0) * d],
        origin_m: [rng.uni(-0.03, 0.03), rng.uni(-0.03, 0.03), rng.uni(-0.03, 0.03)],
    })
}

#[test]
fn zero_correction_is_the_urdf_chain_bit_for_bit() {
    let zero = [JointCorrection::default(); 6];
    let mut rng = Rng(11);
    for _ in 0..500 {
        let q: [f64; 6] = std::array::from_fn(|i| rng.uni(LIM[i][0], LIM[i][1]));
        assert_eq!(k::frames_corrected(&q, Some(&zero)), k::frames(&q), "{q:?}");
        assert_eq!(k::fk_corrected(&q, Some(&zero)), k::fk(&q));
        let goal = k::fk(&std::array::from_fn(|i| q[i] * 0.9));
        let lo = LIM.map(|l| l[0]);
        let hi = LIM.map(|l| l[1]);
        assert_eq!(k::solve_dls_corrected(&goal, &q, &lo, &hi, k::PAR.w, k::PAR.iters, Some(&zero)), k::solve_dls(&goal, &q, &lo, &hi, k::PAR.w));
    }
    // ArmModel: a zero correction is no correction
    let arm = ArmModel::REBOT_DEV_ARM.with_correction(Some(&zero)).unwrap();
    assert!(arm.correction().is_none());
    let q = [0.3, -0.7, 1.1, 0.2, -0.4, 1.9];
    assert_eq!(arm.fk(&q), ArmModel::REBOT_DEV_ARM.fk(&q));
    // a correction of one joint leaves the arithmetic of the others unchanged: J6 only
    // changes nothing before joint 6's origin
    let mut c6 = zero;
    c6[5].offset_rad = 0.05;
    let (a, b) = (k::frames_corrected(&q, Some(&c6)), k::frames(&q));
    assert_eq!(a.origins, b.origins);
    assert_eq!(a.axes, b.axes);
}

#[test]
fn corrected_fk_matches_kincal2_fkp() {
    let path = format!("{}/tests/fixtures/kincal_fkp_samples.json", env!("CARGO_MANIFEST_DIR"));
    let fx: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let samples = fx["samples"].as_array().unwrap();
    assert!(samples.len() >= 40);
    let (mut worst_p, mut worst_r) = (0.0f64, 0.0f64);
    for s in samples {
        let q: [f64; 6] = serde_json::from_value(s["q"].clone()).unwrap();
        let c: Correction = serde_json::from_value(s["joints"].clone()).unwrap();
        let r: Mat3 = serde_json::from_value(s["R"].clone()).unwrap();
        let p: [f64; 3] = serde_json::from_value(s["p"].clone()).unwrap();
        let got = k::fk_corrected(&q, Some(&c));
        worst_p = worst_p.max(k::norm(&k::sub(&got.p, &p)));
        worst_r = worst_r.max(max_diff_m(&got.r, &r));
    }
    println!("fkP fixture: {} samples, worst position {worst_p:.3e} m, worst rotation entry {worst_r:.3e}", samples.len());
    assert!(worst_p < 1e-12 && worst_r < 1e-12, "{worst_p} {worst_r}");
}

#[test]
fn corrected_axes_are_the_jacobian() {
    let mut rng = Rng(5);
    for _ in 0..50 {
        let c = random_correction(&mut rng);
        let q: [f64; 6] = std::array::from_fn(|i| rng.uni(LIM[i][0] * 0.8, LIM[i][1] * 0.8));
        let f = k::frames_corrected(&q, Some(&c));
        for j in 0..6 {
            let h = 1e-6;
            let (mut qp, mut qm) = (q, q);
            qp[j] += h;
            qm[j] -= h;
            let (tp, tm) = (k::fk_corrected(&qp, Some(&c)), k::fk_corrected(&qm, Some(&c)));
            let dp = k::sub(&tp.p, &tm.p).map(|v| v / (2.0 * h));
            let dr = k::log_v(&k::mul_t(&tp.r, &tm.r)).map(|v| v / (2.0 * h));
            let lin = k::cross(&f.axes[j], &k::sub(&f.end.p, &f.origins[j]));
            assert!(k::norm(&k::sub(&dp, &lin)) < 1e-7, "joint {j}: {dp:?} vs {lin:?}");
            assert!(k::norm(&k::sub(&dr, &f.axes[j])) < 1e-7, "joint {j}: {dr:?} vs {:?}", f.axes[j]);
        }
    }
}

/// Smallest singular value of the w-scaled numeric Jacobian of the corrected chain.
fn sigma_min(q: &[f64; 6], c: &Correction, w: f64) -> f64 {
    let t0 = k::fk_corrected(q, Some(c));
    let mut jac = [[0.0f64; 6]; 6];
    for j in 0..6 {
        let mut qq = *q;
        qq[j] += 1e-6;
        let t1 = k::fk_corrected(&qq, Some(c));
        let dp = k::sub(&t1.p, &t0.p);
        let dr = k::log_v(&k::mul_t(&t1.r, &t0.r));
        jac[j] = [dp[0], dp[1], dp[2], dr[0] * w, dr[1] * w, dr[2] * w].map(|v| v / 1e-6);
    }
    let mut a = [[0.0f64; 6]; 6];
    for i in 0..6 { for kk in 0..6 { a[i][kk] = (0..6).map(|r| jac[i][r] * jac[kk][r]).sum(); } }
    for _ in 0..30 {
        for p in 0..6 {
            for r in (p + 1)..6 {
                if a[p][r].abs() < 1e-15 { continue; }
                let th = (a[r][r] - a[p][p]) / (2.0 * a[p][r]);
                let t = (if th == 0.0 { 1.0 } else { th.signum() }) / (th.abs() + (th * th + 1.0).sqrt());
                let cc = 1.0 / (t * t + 1.0).sqrt();
                let s = t * cc;
                for kk in 0..6 { let (x, y) = (a[kk][p], a[kk][r]); a[kk][p] = cc * x - s * y; a[kk][r] = s * x + cc * y; }
                for kk in 0..6 { let (x, y) = (a[p][kk], a[r][kk]); a[p][kk] = cc * x - s * y; a[r][kk] = s * x + cc * y; }
            }
        }
    }
    (0..6).map(|i| a[i][i]).fold(f64::INFINITY, f64::min).max(0.0).sqrt()
}

/// FK -> IK -> FK on 200 random in-limit poses with a non-zero correction. (1) armd's solve
/// (`ArmModel::solve_dls`, motor angles, [`k::PAR`]): ok, < 0.1 mm and within armd's stop
/// tolerance (`PAR.tol_rot / 10` = 0.05 deg). (2) The same DLS run to a tighter stop tolerance
/// (`solve_dls_par`): < 0.1 mm and < 0.01 deg, i.e. it converges on the corrected chain.
#[test]
fn corrected_ik_round_trip_200_random_poses() {
    let off = k::B601DM_MODEL_OFFSETS_RAD;
    let tight = k::IkParams { tol_pos: 1e-5, tol_rot: 1e-3_f64.to_radians(), iters: 30, ..k::PAR };
    let mut rng = Rng(0x6b696e63);
    for (name, corr) in [("real-like", real_like()), ("random", random_correction(&mut Rng(99))), ("none", [JointCorrection::default(); 6])] {
        let arm = ArmModel::REBOT_DEV_ARM.with_correction(Some(&corr)).unwrap();
        let lo: [f64; 6] = std::array::from_fn(|i| LIM[i][0] + 0.02 - off[i]);
        let hi: [f64; 6] = std::array::from_fn(|i| LIM[i][1] - 0.02 - off[i]);
        let (lo_m, hi_m) = (LIM.map(|l| l[0] + 0.02), LIM.map(|l| l[1] - 0.02));
        let (mut bad, mut bad_t, mut over_001, mut skipped, mut n) = (0, 0, 0, 0, 0);
        let (mut worst, mut worst_t, mut max_it, mut max_it_t) = ((0.0f64, 0.0f64), (0.0f64, 0.0f64), 0, 0);
        while n < 200 {
            let q: [f64; 6] = std::array::from_fn(|i| rng.uni(lo[i] + 0.15, hi[i] - 0.15));
            let qm: [f64; 6] = std::array::from_fn(|i| q[i] + off[i]);
            if sigma_min(&qm, &corr, 0.2) < 0.02 { skipped += 1; continue; }
            n += 1;
            let goal = arm.fk(&q);
            let q0: [f64; 6] = std::array::from_fn(|i| q[i] + rng.uni(-0.1, 0.1));
            // (1) armd's solve
            let r = arm.solve_dls(&goal, &q0, &lo, &hi, k::PAR.w);
            let got = arm.fk(&r.q);
            let (mm, deg) = (k::norm(&k::sub(&got.p, &goal.p)) * 1000.0, k::angle_between(&got.r, &goal.r).to_degrees());
            if !r.ok || mm >= 0.1 || deg > k::PAR.tol_rot.to_degrees() / 10.0 { bad += 1; }
            if deg >= 0.01 { over_001 += 1; }
            worst = (worst.0.max(mm), worst.1.max(deg));
            max_it = max_it.max(r.iterations);
            // (2) tighter stop tolerance, model angles
            let q0m: [f64; 6] = std::array::from_fn(|i| q0[i] + off[i]);
            let t = k::solve_dls_par(&goal, &q0m, &lo_m, &hi_m, k::PAR.w, &tight, Some(&corr));
            let got = k::fk_corrected(&t.q, Some(&corr));
            let (mm, deg) = (k::norm(&k::sub(&got.p, &goal.p)) * 1000.0, k::angle_between(&got.r, &goal.r).to_degrees());
            if !t.ok || mm >= 0.1 || deg >= 0.01 { bad_t += 1; }
            worst_t = (worst_t.0.max(mm), worst_t.1.max(deg));
            max_it_t = max_it_t.max(t.iterations);
        }
        println!("corrected DLS round trip ({name}): 200 cases, skipped near-singular {skipped}; armd PAR: fail {bad}, worst {:.5} mm / {:.5} deg, \
            {over_001} cases >= 0.01 deg, max iters {max_it}; tight stop: fail {bad_t}, worst {:.6} mm / {:.6} deg, max iters {max_it_t}",
            worst.0, worst.1, worst_t.0, worst_t.1);
        assert_eq!((bad, bad_t), (0, 0), "{name}");
    }
}

#[test]
fn correction_bounds_and_profiles() {
    let d = std::f64::consts::PI / 180.0;
    assert!(k::validate_correction(&real_like()).is_ok());
    let mut edge = [JointCorrection::default(); 6];
    edge[0] = JointCorrection { offset_rad: 10.0 * d, scale: -0.08, tilt_rad: [3.0 * d, -3.0 * d], origin_m: [0.03, -0.03, 0.03] };
    assert!(k::validate_correction(&edge).is_ok(), "the bounds themselves are allowed");
    for (j, f, patch) in [
        (1, "offset_rad", JointCorrection { offset_rad: 10.01 * d, ..Default::default() }),
        (2, "scale", JointCorrection { scale: 0.081, ..Default::default() }),
        (3, "tilt_rad", JointCorrection { tilt_rad: [0.0, -3.01 * d], ..Default::default() }),
        (4, "origin_m", JointCorrection { origin_m: [0.0, 0.0, 0.0301], ..Default::default() }),
        (5, "offset_rad", JointCorrection { offset_rad: f64::NAN, ..Default::default() }),
    ] {
        let mut c = [JointCorrection::default(); 6];
        c[j] = patch;
        let e = k::validate_correction(&c).unwrap_err();
        assert!(e.contains(f) && e.contains(&format!("J{}", j + 1)), "{e}");
        assert!(ArmModel::REBOT_DEV_ARM.with_correction(Some(&c)).is_err());
    }
    // serde: the documented shape, missing fields are 0, unknown fields and wrong lengths fail
    let c: Correction = serde_json::from_str(r#"[{"offset_rad":0.01,"scale":0,"tilt_rad":[0,0],"origin_m":[0,0,0]},{},{},{},{},{}]"#).unwrap();
    assert_eq!(c[0].offset_rad, 0.01);
    assert!(serde_json::from_str::<Correction>(r#"[{"offset":0.01},{},{},{},{},{}]"#).is_err());
    assert!(serde_json::from_str::<Correction>(r#"[{},{},{},{},{}]"#).is_err());
    // URDF chain profiles: no correction
    let mut so = armd::ArmConfig::rebot_dm();
    so.init_arm_model_in_memory(None).unwrap();
    let mut cfg: armd::ArmConfig = serde_json::from_str(include_str!("../profiles/so_arm101.json")).unwrap();
    cfg.init_arm_model_in_memory(Some(include_str!("../assets/so101_new_calib.urdf"))).unwrap();
    let e = cfg.arm_model.with_correction(Some(&real_like())).unwrap_err();
    assert!(e.contains("URDF"), "{e}");
    assert!(cfg.arm_model.with_correction(None).is_ok());
    // validate() keeps an installed correction
    so.arm_model = so.arm_model.with_correction(Some(&real_like())).unwrap();
    so.validate().unwrap();
    assert_eq!(so.arm_model.correction(), Some(&real_like()));
}
