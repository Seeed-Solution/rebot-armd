//! Acceptance gate for the URDF chain (src/chain.rs): built from the B601-DM URDF it must
//! reproduce the built-in hand-coded model (src/kinematics.rs FK / DLS IK, src/gravity.rs).
use std::time::Instant;

use armd::chain::{ArmModel, ChainJointSpec, ChainSpec, KinematicChain};
use armd::kinematics::{self, Pose};

const FK_TOL_M: f64 = 1e-9;
const ROT_TOL: f64 = 1e-9;
const IK_TOL_RAD: f64 = 1e-9;
const GRAVITY_TOL_NM: f64 = 1e-6;

fn dm_spec() -> ChainSpec {
    ChainSpec {
        base_link: "base_link".into(),
        tip_link: "end_link".into(),
        joints: (1..=6).map(|i| ChainJointSpec { urdf_joint: format!("joint{i}"), sign: 1.0, offset_rad: 0.0 }).collect(),
    }
}

fn dm_chain() -> KinematicChain {
    KinematicChain::from_urdf_str(include_str!("../assets/reBot-DevArm_gripper.urdf"), &dm_spec()).unwrap()
}

/// Deterministic uniform numbers in [0, 1).
struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn q(&mut self, span: f64) -> [f64; 6] { std::array::from_fn(|_| (self.next() * 2.0 - 1.0) * span) }
}

fn rot_dev(a: &Pose, b: &Pose) -> f64 {
    a.r.iter().flatten().zip(b.r.iter().flatten()).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max)
}
fn pos_dev(a: &kinematics::Vec3, b: &kinematics::Vec3) -> f64 {
    (0..3).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}

#[test]
fn dm_chain_fk_matches_builtin() {
    let chain = dm_chain();
    assert_eq!(chain.dof(), 6);
    let mut rng = Lcg(1);
    let (mut dp, mut dr, mut dj) = (0.0f64, 0.0f64, 0.0f64);
    for _ in 0..5000 {
        let q = rng.q(std::f64::consts::PI);
        let a = kinematics::frames(&q);
        let b = chain.frames(&q);
        dp = dp.max(pos_dev(&a.end.p, &b.end.p));
        dr = dr.max(rot_dev(&a.end, &b.end));
        for j in 0..6 {
            dj = dj.max(pos_dev(&a.axes[j], &b.axes[j])).max(pos_dev(&a.origins[j], &b.origins[j]));
        }
    }
    eprintln!("FK max deviation: position {dp:.3e} m, rotation {dr:.3e}, joint axes/origins {dj:.3e}");
    assert!(dp < FK_TOL_M && dr < ROT_TOL && dj < FK_TOL_M, "{dp:e} {dr:e} {dj:e}");
}

#[test]
#[allow(clippy::approx_constant)] // URDF joint limits as written
fn dm_chain_ik_matches_builtin() {
    let chain = dm_chain();
    let model = ArmModel::Chain(chain.into());
    let lo = [-2.8, -3.14, -3.14, -1.87, -1.57, -3.14];
    let hi = [2.8, 0.0, 0.0, 1.57, 1.57, 3.14];
    let mut rng = Lcg(7);
    let (mut dq, mut dep, mut n_ok, mut n) = (0.0f64, 0.0f64, 0, 0);
    for _ in 0..2000 {
        let target: [f64; 6] = std::array::from_fn(|i| lo[i] + rng.next() * (hi[i] - lo[i]));
        let goal = kinematics::fk(&target);
        let pert = rng.q(0.3);
        let q0: [f64; 6] = std::array::from_fn(|i| (target[i] + pert[i]).clamp(lo[i], hi[i]));
        let a = kinematics::solve_dls(&goal, &q0, &lo, &hi, kinematics::PAR.w);
        let b = model.solve_dls(&goal, &q0, &lo, &hi, kinematics::PAR.w);
        assert_eq!(a.ok, b.ok, "ok flag differs for target {target:?} from {q0:?}");
        assert_eq!(a.iterations, b.iterations, "iterations differ for target {target:?}");
        dq = dq.max((0..6).map(|i| (a.q[i] - b.q[i]).abs()).fold(0.0, f64::max));
        dep = dep.max((a.ep - b.ep).abs()).max((a.er - b.er).abs());
        n += 1;
        if a.ok { n_ok += 1; }
    }
    eprintln!("IK max deviation: q {dq:.3e} rad, ep/er {dep:.3e} ({n_ok}/{n} solved)");
    assert!(dq < IK_TOL_RAD && dep < FK_TOL_M, "{dq:e} {dep:e}");
    assert!(n_ok > n / 2, "sweep should mostly converge: {n_ok}/{n}");
}

#[test]
fn dm_chain_gravity_matches_builtin() {
    let chain = dm_chain();
    let model = ArmModel::Chain(chain.clone().into());
    let mut rng = Lcg(3);
    let (mut d64, mut d32) = (0.0f64, 0.0f64);
    for _ in 0..5000 {
        let q = rng.q(std::f64::consts::PI);
        let a = armd::gravity::gravity_torques_f64(&q);
        let b = chain.gravity_torques(&q);
        d64 = d64.max((0..6).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max));
        let q32: [f32; 6] = q.map(|v| v as f32);
        let a32 = armd::compute_gravity_torques(&q32).unwrap();
        let b32 = model.gravity_torques(&q32).unwrap();
        d32 = d32.max((0..6).map(|i| f64::from((a32[i] - b32[i]).abs())).fold(0.0, f64::max));
    }
    eprintln!("gravity max deviation: f64 {d64:.3e} N·m, f32 path {d32:.3e} N·m");
    assert!(d64 < GRAVITY_TOL_NM, "{d64:e}");
    // f32 outputs may differ by one rounding step of values up to ~20 N·m.
    assert!(d32 < 4e-6, "{d32:e}");
    // Mass: every link but base_link (0.8366 kg) moves.
    let total = 0.1613 + 1.3266 + 0.8353 + 0.52 + 0.383 + 0.3663 + 0.55 + 0.03 + 0.03;
    assert!((chain.moving_mass() - total).abs() < 1e-12);
}

#[test]
fn dm_chain_shoulder_and_camera_match_builtin() {
    let model = ArmModel::Chain(dm_chain().into());
    let s = model.shoulder_rz();
    // SHOULDER_RZ is the rounded URDF value (0.020084 - 0.00008416 = 0.01999984 m).
    assert!((s[0] - kinematics::SHOULDER_RZ[0]).abs() < 1e-6 && (s[1] - kinematics::SHOULDER_RZ[1]).abs() < 1e-9, "{s:?}");
    for (flip, rot, tilt) in [(false, 0.0, 0.0), (true, 30.0, -10.0)] {
        let a = kinematics::cam_mount(flip, rot, tilt);
        let b = model.cam_mount(flip, rot, tilt);
        let d = a.iter().flatten().zip(b.iter().flatten()).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max);
        assert!(d < 1e-9, "{d:e}");
    }
}

#[test]
fn chain_sign_and_offset_map_armd_angles() {
    let mut spec = dm_spec();
    spec.joints[1].sign = -1.0;
    spec.joints[2].offset_rad = 0.25;
    let chain = KinematicChain::from_urdf_str(include_str!("../assets/reBot-DevArm_gripper.urdf"), &spec).unwrap();
    let q = [0.1, 0.4, -0.3, 0.2, -0.5, 0.6];
    let urdf_q = [0.1, -0.4, -0.05, 0.2, -0.5, 0.6];
    assert!(pos_dev(&chain.fk(&q).p, &kinematics::fk(&urdf_q).p) < 1e-12);
    // Jacobian columns are per armd radian: finite difference check.
    let jac = chain.jacobian(&q);
    for j in 0..6 {
        let mut qp = q;
        qp[j] += 1e-6;
        let d = kinematics::sub(&chain.fk(&qp).p, &chain.fk(&q).p);
        for a in 0..3 { assert!((d[a] / 1e-6 - jac[j][a]).abs() < 1e-5, "joint {j} axis {a}"); }
    }
    // Gravity torque is per armd joint direction.
    let tau = chain.gravity_torques(&q);
    let tau_ref = chain_reference_flip(&q);
    assert!((tau[1] + tau_ref).abs() < 1e-9, "sign flips the J2 torque: {} vs {}", tau[1], tau_ref);
}

/// J2 torque of the unmodified DM chain at the URDF angles the flipped spec maps to.
fn chain_reference_flip(q: &[f64; 6]) -> f64 {
    let urdf_q = [q[0], -q[1], q[2] + 0.25, q[3], q[4], q[5]];
    dm_chain().gravity_torques(&urdf_q)[1]
}

#[test]
fn chain_spec_errors_are_reported() {
    let urdf = include_str!("../assets/reBot-DevArm_gripper.urdf");
    let mut s = dm_spec();
    s.joints.swap(0, 1);
    assert!(KinematicChain::from_urdf_str(urdf, &s).unwrap_err().contains("in order"));
    let mut s = dm_spec();
    s.tip_link = "nope".into();
    assert!(KinematicChain::from_urdf_str(urdf, &s).is_err());
    let mut s = dm_spec();
    s.tip_link = "left_finger".into();
    assert!(KinematicChain::from_urdf_str(urdf, &s).unwrap_err().contains("Prismatic"));
    let mut s = dm_spec();
    s.joints[0].sign = 2.0;
    assert!(KinematicChain::from_urdf_str(urdf, &s).is_err());
}

/// Per-tick gravity cost, built-in vs chain. Run with
/// `cargo test --release --test test_chain -- --ignored --nocapture gravity_cost`.
#[test]
#[ignore]
fn gravity_cost() {
    let model = ArmModel::Chain(dm_chain().into());
    let mut rng = Lcg(11);
    let qs: Vec<[f32; 6]> = (0..1000).map(|_| rng.q(3.0).map(|v| v as f32)).collect();
    let rounds = 200;
    let mut sink = 0.0f32;
    let t = Instant::now();
    for _ in 0..rounds { for q in &qs { sink += armd::compute_gravity_torques(std::hint::black_box(q)).unwrap()[1]; } }
    let builtin = t.elapsed().as_nanos() as f64 / (rounds * qs.len()) as f64;
    let t = Instant::now();
    for _ in 0..rounds { for q in &qs { sink += model.gravity_torques(std::hint::black_box(q)).unwrap()[1]; } }
    let chain = t.elapsed().as_nanos() as f64 / (rounds * qs.len()) as f64;
    eprintln!("gravity per call: built-in {builtin:.0} ns, chain {chain:.0} ns, ratio {:.2} (sink {sink})", chain / builtin);
    assert!(chain < 2.0 * builtin);
}
