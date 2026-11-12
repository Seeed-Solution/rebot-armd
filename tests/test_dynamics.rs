//! Rigid-body inverse dynamics (src/dynamics.rs): RNEA against the existing gravity model,
//! the mass matrix (symmetric, positive definite, matches the kinetic energy) and the
//! power balance d(KE)/dt = q'ᵀ (tau - g).
use armd::chain::{ArmModel, ChainJointSpec, ChainSpec, KinematicChain, MAX_CHAIN_JOINTS};
use armd::dynamics::{kinetic_energy, rebot_dm_chain, rnea};

const Z: [f64; MAX_CHAIN_JOINTS] = [0.0; MAX_CHAIN_JOINTS];

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn v(&mut self, span: f64) -> [f64; MAX_CHAIN_JOINTS] {
        std::array::from_fn(|i| if i < 6 { (self.next() * 2.0 - 1.0) * span } else { 0.0 })
    }
}

fn mass_matrix(c: &KinematicChain, q: &[f64]) -> [[f64; 6]; 6] {
    let mut m = [[0.0; 6]; 6];
    for i in 0..6 {
        let mut e = Z;
        e[i] = 1.0;
        let col = rnea(c, q, &Z, &e, false);
        for r in 0..6 { m[r][i] = col[r]; }
    }
    m
}

/// Cholesky succeeds (positive definite).
fn pos_def(m: &[[f64; 6]; 6]) -> bool {
    let mut l = [[0.0f64; 6]; 6];
    for i in 0..6 {
        for j in 0..=i {
            let s: f64 = (0..j).map(|k| l[i][k] * l[j][k]).sum();
            if i == j {
                let d = m[i][i] - s;
                if d <= 0.0 { return false; }
                l[i][j] = d.sqrt();
            } else {
                l[i][j] = (m[i][j] - s) / l[j][j];
            }
        }
    }
    true
}

#[test]
fn rnea_at_rest_equals_the_gravity_model() {
    let mut rng = Lcg(7);
    let builtin = ArmModel::REBOT_DEV_ARM;
    let mut dev = 0.0f64;
    for _ in 0..5000 {
        let q = rng.v(std::f64::consts::PI);
        let tau = builtin.inverse_dynamics(&q, &Z, &Z, true);
        // gravity.rs takes model (URDF) angles; q is in motor angles
        let off = armd::kinematics::B601DM_MODEL_OFFSETS_RAD;
        let g = armd::gravity::gravity_torques_f64(&std::array::from_fn(|i| q[i] + off[i]));
        let chain_g = rebot_dm_chain(&armd::kinematics::B601DM_MODEL_OFFSETS_RAD).gravity_torques(&q);
        for j in 0..6 { dev = dev.max((tau[j] - g[j]).abs()).max((tau[j] - chain_g[j]).abs()); }
    }
    eprintln!("RNEA(q, 0, 0) vs gravity.rs / chain gravity: max deviation {dev:.3e} N·m over 5000 poses");
    assert!(dev < 1e-9, "{dev:e}");
}

#[test]
fn rnea_matches_a_urdf_chain_profile() {
    // the RobStride URDF (profiles/rebot_b601_rs.json chain) through the same chain code
    let spec = ChainSpec {
        base_link: "base_link".into(), tip_link: "gripper_end".into(),
        joints: (1..=6).map(|i| ChainJointSpec { urdf_joint: format!("joint{i}"), sign: 1.0, offset_rad: 0.0 }).collect(),
    };
    let chain = KinematicChain::from_urdf_str(include_str!("../assets/ReBot_Arm_RS.urdf"), &spec).unwrap();
    let mut rng = Lcg(9);
    let mut dev = 0.0f64;
    for _ in 0..1000 {
        let q = rng.v(2.0);
        let tau = rnea(&chain, &q, &Z, &Z, true);
        let g = chain.gravity_torques(&q);
        for j in 0..chain.dof() { dev = dev.max((tau[j] - g[j]).abs()); }
    }
    assert!(dev < 1e-9, "{dev:e}");
}

#[test]
fn mass_matrix_is_symmetric_positive_definite_and_gives_the_kinetic_energy() {
    let c = rebot_dm_chain(&armd::kinematics::B601DM_MODEL_OFFSETS_RAD);
    let mut rng = Lcg(3);
    let (mut asym, mut ke_dev, mut lin_dev) = (0.0f64, 0.0f64, 0.0f64);
    for _ in 0..500 {
        let q = rng.v(std::f64::consts::PI);
        let qd = rng.v(2.0);
        let qdd = rng.v(5.0);
        let m = mass_matrix(c, &q);
        for i in 0..6 { for j in 0..6 { asym = asym.max((m[i][j] - m[j][i]).abs()); } }
        assert!(pos_def(&m), "M not positive definite at {q:?}");
        let quad: f64 = (0..6).map(|i| (0..6).map(|j| qd[i] * m[i][j] * qd[j]).sum::<f64>()).sum::<f64>() * 0.5;
        ke_dev = ke_dev.max((quad - kinetic_energy(c, &q, &qd)).abs());
        // linear in q'': tau(q, q', q'') - tau(q, q', 0) = M q''
        let a = rnea(c, &q, &qd, &qdd, true);
        let b = rnea(c, &q, &qd, &Z, true);
        for i in 0..6 {
            let mq: f64 = (0..6).map(|j| m[i][j] * qdd[j]).sum();
            lin_dev = lin_dev.max((a[i] - b[i] - mq).abs());
        }
    }
    eprintln!("M: asymmetry {asym:.3e}, |½q'ᵀMq' - KE| {ke_dev:.3e} J, |Δtau - M q''| {lin_dev:.3e} N·m");
    assert!(asym < 1e-10 && ke_dev < 1e-10 && lin_dev < 1e-9);
    let m0 = mass_matrix(c, &[0.0, -1.0, -1.5, 1.4, 0.0, 1.57, 0.0, 0.0]);
    eprintln!("M diagonal at INSPECT: {:?} kg·m²", (0..6).map(|i| (m0[i][i] * 1e4).round() / 1e4).collect::<Vec<_>>());
}

/// Along q(t) = q0 + A sin(w t): d(KE)/dt = q'ᵀ (tau - g), and the time-scaling identity the
/// planner relies on: tau_dyn(q, s q', s² q'') = s² tau_dyn(q, q', q'').
#[test]
fn power_balance_and_time_scaling() {
    let c = rebot_dm_chain(&armd::kinematics::B601DM_MODEL_OFFSETS_RAD);
    let mut rng = Lcg(11);
    let (mut worst, mut scale_dev) = (0.0f64, 0.0f64);
    for _ in 0..50 {
        let q0 = rng.v(2.0);
        let amp = rng.v(0.8);
        let w: [f64; MAX_CHAIN_JOINTS] = std::array::from_fn(|i| 1.0 + i as f64 * 0.37);
        let at = |t: f64| -> ([f64; 8], [f64; 8], [f64; 8]) {
            (std::array::from_fn(|i| q0[i] + amp[i] * (w[i] * t).sin()),
             std::array::from_fn(|i| amp[i] * w[i] * (w[i] * t).cos()),
             std::array::from_fn(|i| -amp[i] * w[i] * w[i] * (w[i] * t).sin()))
        };
        for k in 0..20 {
            let t = 0.1 * k as f64;
            let (q, qd, qdd) = at(t);
            let tau = rnea(c, &q, &qd, &qdd, false);
            let p: f64 = (0..6).map(|i| qd[i] * tau[i]).sum();
            let h = 1e-5;
            let (qa, qda, _) = at(t + h);
            let (qb, qdb, _) = at(t - h);
            let dke = (kinetic_energy(c, &qa, &qda) - kinetic_energy(c, &qb, &qdb)) / (2.0 * h);
            worst = worst.max((p - dke).abs() / (1e-3 + p.abs()));
            let s = 0.37;
            let sc = rnea(c, &q, &qd.map(|v| v * s), &qdd.map(|v| v * s * s), false);
            for i in 0..6 { scale_dev = scale_dev.max((sc[i] - s * s * tau[i]).abs()); }
        }
    }
    eprintln!("power balance: worst relative error {worst:.2e}; time scaling deviation {scale_dev:.2e} N·m");
    assert!(worst < 1e-5 && scale_dev < 1e-10);
}

/// Per-call cost (printed): one RNEA of the 6-joint chain.
#[test]
fn rnea_cost() {
    let c = rebot_dm_chain(&armd::kinematics::B601DM_MODEL_OFFSETS_RAD);
    let mut rng = Lcg(5);
    let qs: Vec<_> = (0..1000).map(|_| (rng.v(2.0), rng.v(1.0), rng.v(3.0))).collect();
    let t0 = std::time::Instant::now();
    let mut acc = 0.0;
    for _ in 0..100 { for (q, qd, qdd) in &qs { acc += rnea(c, q, qd, qdd, true)[1]; } }
    let per = t0.elapsed().as_secs_f64() / 100_000.0;
    eprintln!("RNEA: {:.2} µs per call ({acc:.1})", per * 1e6);
    assert!(per < 200e-6);
}
