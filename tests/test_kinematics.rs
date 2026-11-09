//! Rust kinematics (src/kinematics.rs) vs the companion FK fixture and the cockpit's JS VIK.
use std::time::Instant;
use armd::kinematics::{self, Mat3, Pose};
use serde_json::Value;

fn load(name: &str) -> Value {
    let path = format!("{}/tests/fixtures/{name}", env!("CARGO_MANIFEST_DIR"));
    serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
}
fn v3(v: &Value) -> [f64; 3] { [v[0].as_f64().unwrap(), v[1].as_f64().unwrap(), v[2].as_f64().unwrap()] }
fn v6(v: &Value) -> [f64; 6] {
    let a: Vec<f64> = v.as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
    a.try_into().unwrap()
}
fn m3(v: &Value) -> Mat3 { [v3(&v[0]), v3(&v[1]), v3(&v[2])] }
fn max_diff_m(a: &Mat3, b: &Mat3) -> f64 {
    (0..3).flat_map(|i| (0..3).map(move |j| (i, j))).map(|(i, j)| (a[i][j] - b[i][j]).abs()).fold(0.0, f64::max)
}
fn max_diff_v(a: &[f64], b: &[f64]) -> f64 { a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max) }

const LIM: [[f64; 2]; 6] = [[-3.14159, 3.14159], [-2.0944, 2.0944], [-2.61799, 2.61799], [-3.14159, 3.14159], [-2.0944, 2.0944], [-6.28318, 6.28318]];
fn limits(margin: f64) -> ([f64; 6], [f64; 6]) {
    (LIM.map(|l| l[0] + margin), LIM.map(|l| l[1] - margin))
}

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

#[test]
fn fk_matches_companion_fixture() {
    let fx = load("fk_samples.json");
    let samples = fx["samples"].as_array().unwrap();
    let mut worst = 0.0f64;
    for s in samples {
        let pose = kinematics::fk(&v6(&s["q"]));
        let t = &s["T"];
        let r: Mat3 = [0, 1, 2].map(|i| [0, 1, 2].map(|j| t[i][j].as_f64().unwrap()));
        let p = [0, 1, 2].map(|i| t[i][3].as_f64().unwrap());
        worst = worst.max(max_diff_v(&pose.p, &p)).max(max_diff_m(&pose.r, &r));
    }
    println!("FK vs companion fixture: {} samples, max abs diff {worst:.3e}", samples.len());
    assert!(worst < 1e-6, "{worst}");
}

#[test]
fn fk_frames_match_js_vik() {
    let fx = load("vik_samples.json");
    let mut worst = 0.0f64;
    for s in fx["fk"].as_array().unwrap() {
        let f = kinematics::frames(&v6(&s["q"]));
        worst = worst.max(max_diff_m(&f.end.r, &m3(&s["r"]))).max(max_diff_v(&f.end.p, &v3(&s["p"])));
        for j in 0..6 {
            worst = worst.max(max_diff_v(&f.axes[j], &v3(&s["axes"][j]))).max(max_diff_v(&f.origins[j], &v3(&s["origins"][j])));
        }
    }
    println!("FK/frames vs JS VIK: max abs diff {worst:.3e}");
    assert!(worst < 1e-12, "{worst}");
}

#[test]
fn dls_solve_matches_js_vik() {
    let fx = load("vik_samples.json");
    let lo = v6(&fx["lo"]);
    let hi = v6(&fx["hi"]);
    let (mut worst_q, mut same_ok, mut same_it, mut n) = (0.0f64, 0, 0, 0);
    for s in fx["solve"].as_array().unwrap() {
        let goal = Pose::new(m3(&s["goal"]["r"]), v3(&s["goal"]["p"]));
        let r = kinematics::solve_dls(&goal, &v6(&s["q0"]), &lo, &hi, kinematics::PAR.w);
        n += 1;
        if r.ok == s["ok"].as_bool().unwrap() { same_ok += 1; }
        if r.iterations as u64 == s["it"].as_u64().unwrap() { same_it += 1; }
        let dq = max_diff_v(&r.q, &v6(&s["q"]));
        worst_q = worst_q.max(dq);
        if s["ok"].as_bool().unwrap() {
            assert!((r.ep - s["ep"].as_f64().unwrap()).abs() < 1e-9 && (r.er - s["er"].as_f64().unwrap()).abs() < 1e-9);
        }
    }
    println!("IK cross-check vs JS VIK.solve: {n} cases, ok agree {same_ok}/{n}, iterations agree {same_it}/{n}, max |q_rust - q_js| {worst_q:.3e} rad");
    assert_eq!(same_ok, n);
    assert_eq!(same_it, n);
    assert!(worst_q < 1e-9, "{worst_q}");
}

#[test]
fn cam_mount_level_and_rotation_helpers_match_js() {
    let fx = load("vik_samples.json");
    for s in fx["cam_mount"].as_array().unwrap() {
        let r = kinematics::cam_mount(s["flip"].as_bool().unwrap(), s["rot_deg"].as_f64().unwrap(), 0.0);
        let d = max_diff_m(&r, &m3(&s["r"]));
        assert!(d < 1e-12, "camMount {s}: {d}");
    }
    let rec = kinematics::cam_mount(false, 0.0, 0.0);
    let level_cos = fx["level_cos"].as_f64().unwrap();
    for s in fx["level"].as_array().unwrap() {
        let p = kinematics::level(&kinematics::fk(&v6(&s["q"])), &rec, level_cos);
        assert!(max_diff_m(&p.r, &m3(&s["r"])) < 1e-12 && max_diff_v(&p.p, &v3(&s["p"])) < 1e-12);
    }
    for s in fx["rot"].as_array().unwrap() {
        let r = m3(&s["r"]);
        assert!(max_diff_v(&kinematics::log_r(&r), &v3(&s["log"])) < 1e-12);
        assert!(max_diff_m(&kinematics::exp_r(&kinematics::log_r(&r)), &r) < 1e-9);
        assert!(max_diff_m(&kinematics::quat_to_r(&kinematics::r_to_quat(&r)), &r) < 1e-12);
        let rpy = kinematics::r_to_rpy(&r);
        assert!(max_diff_m(&kinematics::rpy_to_r(rpy[0], rpy[1], rpy[2]), &r) < 1e-9);
    }
    // tilt rotates about x_cam: positive tilt lifts the optical axis toward -y_cam
    let tilted = kinematics::cam_mount(false, 0.0, 10.0);
    let z = kinematics::column(&tilted, 2);
    let y0 = kinematics::column(&rec, 1);
    assert!(kinematics::dot(&z, &y0) < -0.17 && (z[0] - 10f64.to_radians().cos()).abs() < 1e-12);
}

#[test]
fn home_pose_matches_spec_numbers() {
    let home = kinematics::fk(&[0.0, 0.0, 0.0, 0.0, 0.0, std::f64::consts::FRAC_PI_2]);
    assert!(max_diff_v(&home.p, &[0.260, 0.0, 0.192]) < 1.5e-3, "{:?}", home.p);
    assert!(home.r[0][0] > 0.999, "tool x forward");
    let inspect = kinematics::fk(&[0.0, -1.167, -1.565, 1.435, 0.0, std::f64::consts::FRAC_PI_2]);
    assert!(max_diff_v(&inspect.p, &[0.250, 0.0, 0.300]) < 1.5e-3, "{:?}", inspect.p);
    assert!(max_diff_v(&kinematics::column(&inspect.r, 0), &[0.509, 0.0, -0.861]) < 2e-3);
}

/// Smallest singular value of the w-scaled Jacobian (numeric), as scripts/view_ik_selftest.mjs.
fn sigma_min(q: &[f64; 6], w: f64) -> f64 {
    let t0 = kinematics::fk(q);
    let mut jac = [[0.0f64; 6]; 6];
    for j in 0..6 {
        let mut qq = *q;
        qq[j] += 1e-6;
        let t1 = kinematics::fk(&qq);
        let dp = kinematics::sub(&t1.p, &t0.p);
        let dr = kinematics::log_v(&kinematics::mul_t(&t1.r, &t0.r));
        jac[j] = [dp[0], dp[1], dp[2], dr[0] * w, dr[1] * w, dr[2] * w].map(|v| v / 1e-6);
    }
    let mut a = [[0.0f64; 6]; 6];
    for i in 0..6 { for k in 0..6 { a[i][k] = (0..6).map(|r| jac[i][r] * jac[k][r]).sum(); } }
    for _ in 0..30 {
        for p in 0..6 {
            for r in (p + 1)..6 {
                if a[p][r].abs() < 1e-15 { continue; }
                let th = (a[r][r] - a[p][p]) / (2.0 * a[p][r]);
                let t = (if th == 0.0 { 1.0 } else { th.signum() }) / (th.abs() + (th * th + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..6 { let (x, y) = (a[k][p], a[k][r]); a[k][p] = c * x - s * y; a[k][r] = s * x + c * y; }
                for k in 0..6 { let (x, y) = (a[p][k], a[r][k]); a[p][k] = c * x - s * y; a[r][k] = s * x + c * y; }
            }
        }
    }
    (0..6).map(|i| a[i][i]).fold(f64::INFINITY, f64::min).max(0.0).sqrt()
}

#[test]
fn dls_round_trip_200_random_configurations() {
    let (lo, hi) = limits(0.02);
    let mut rng = Rng(0x7a11e109);
    let (mut bad, mut worst_mm, mut worst_deg, mut skipped, mut max_it) = (0, 0.0f64, 0.0f64, 0, 0);
    let mut n = 0;
    while n < 200 {
        let q: [f64; 6] = std::array::from_fn(|i| rng.uni(lo[i] + 0.15, hi[i] - 0.15));
        if sigma_min(&q, 0.2) < 0.02 { skipped += 1; continue; }
        n += 1;
        let goal = kinematics::fk(&q);
        let q0: [f64; 6] = std::array::from_fn(|i| q[i] + rng.uni(-0.1, 0.1));
        let r = kinematics::solve_dls(&goal, &q0, &lo, &hi, kinematics::PAR.w);
        let got = kinematics::fk(&r.q);
        let mm = kinematics::norm(&kinematics::sub(&got.p, &goal.p)) * 1000.0;
        let deg = kinematics::angle_between(&got.r, &goal.r).to_degrees();
        if !r.ok || mm > 1.0 || deg > 0.5 { bad += 1; }
        worst_mm = worst_mm.max(mm);
        worst_deg = worst_deg.max(deg);
        max_it = max_it.max(r.iterations);
    }
    println!("DLS round trip: 200 cases, fail {bad}, worst {worst_mm:.4} mm / {worst_deg:.4} deg, max iters {max_it}, skipped near-singular {skipped}");
    assert_eq!(bad, 0);
}

#[test]
fn unreachable_and_invalid_goals_return_not_ok() {
    let (lo, hi) = limits(0.02);
    let q0 = [0.0, 0.0, 0.0, 0.0, 0.0, std::f64::consts::FRAC_PI_2];
    let mut far = kinematics::fk(&q0);
    far.p[0] += 1.0;
    let r = kinematics::solve_dls(&far, &q0, &lo, &hi, 0.2);
    assert!(!r.ok && r.ep > 0.1);
    assert!(r.q.iter().zip(lo.iter().zip(&hi)).all(|(q, (l, h))| *q >= *l && *q <= *h), "IK clamps to limits");
    let mut nan = kinematics::fk(&q0);
    nan.p[2] = f64::NAN;
    let r = kinematics::solve_dls(&nan, &q0, &lo, &hi, 0.2);
    assert!(!r.ok && r.q == q0);
}

/// Per-call FK / IK time. Run on the device:  cargo test --release --test test_kinematics fk_ik_timing -- --nocapture
#[test]
fn fk_ik_timing() {
    let (lo, hi) = limits(0.02);
    let q = [0.1, -1.0, -1.4, 1.3, 0.1, 1.5];
    let n = 2000;
    let t = Instant::now();
    let mut acc = 0.0;
    for i in 0..n {
        let mut qq = q;
        qq[0] += i as f64 * 1e-6;
        acc += kinematics::fk(&qq).p[0];
    }
    let fk_us = t.elapsed().as_secs_f64() * 1e6 / n as f64;
    let goal = kinematics::fk(&[0.12, -0.98, -1.42, 1.31, 0.08, 1.52]);
    let m = 300;
    let t = Instant::now();
    let mut iters = 0;
    for _ in 0..m {
        let r = kinematics::solve_dls(&goal, &q, &lo, &hi, 0.2);
        iters += r.iterations;
        acc += r.q[0];
    }
    let ik_us = t.elapsed().as_secs_f64() * 1e6 / m as f64;
    println!("timing: fk {fk_us:.2} us/call, solve_dls {ik_us:.1} us/call ({:.1} iterations avg) [checksum {acc:.3}]", iters as f64 / m as f64);
    assert!(acc.is_finite());
}
