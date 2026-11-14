//! Rigid-body inverse dynamics of the arm chain: recursive Newton-Euler (RNEA),
//!
//! ```text
//! tau = M(q) q'' + C(q, q') q' + g(q)
//! ```
//!
//! from the URDF inertials (mass, centre of mass, inertia tensor per link, lumped per moving
//! segment by `chain.rs`). Joint friction is not part of the rigid-body model; the power
//! planner adds the profile's per-joint viscous / Coulomb terms (default 0).
//!
//! The built-in B601-DM model (`ArmModel::RebotDevArm`) uses a chain built once from the
//! embedded `assets/reBot-DevArm_gripper.urdf` (`base_link -> end_link`, joints 1..6); its
//! gravity term matches `gravity.rs` (`tests/test_dynamics.rs`).
//!
//! Computed in the base frame with forward velocity / acceleration propagation and a backward
//! force / moment pass; gravity enters as an upward acceleration of the base. O(n), no
//! allocation.
use std::sync::OnceLock;

use crate::chain::{ArmModel, ChainJointSpec, ChainSpec, KinematicChain, G, MAX_CHAIN_JOINTS};
use crate::kinematics::{self as k, Vec3};

/// Joint torques (N·m, armd joint direction) for positions `q`, velocities `qd` and
/// accelerations `qdd` (each at least `chain.dof()` entries); `gravity` adds g(q).
pub fn rnea(chain: &KinematicChain, q: &[f64], qd: &[f64], qdd: &[f64], gravity: bool) -> [f64; MAX_CHAIN_JOINTS] {
    let f = chain.frames(q);
    let n = f.n;
    let mut force = [[0.0f64; 3]; MAX_CHAIN_JOINTS];
    let mut moment = [[0.0f64; 3]; MAX_CHAIN_JOINTS];
    let mut rc = [[0.0f64; 3]; MAX_CHAIN_JOINTS];
    let (mut w, mut wd) = ([0.0f64; 3], [0.0f64; 3]);
    let mut acc: Vec3 = if gravity { [0.0, 0.0, G] } else { [0.0; 3] };
    let mut o_prev = [0.0f64; 3];
    for j in 0..n {
        let a = f.axes[j];
        let r = k::sub(&f.origins[j], &o_prev);
        acc = k::add(&acc, &k::add(&k::cross(&wd, &r), &k::cross(&w, &k::cross(&w, &r))));
        let wj = [a[0] * qd[j], a[1] * qd[j], a[2] * qd[j]];
        let wd_new = k::add(&k::add(&wd, &[a[0] * qdd[j], a[1] * qdd[j], a[2] * qdd[j]]), &k::cross(&w, &wj));
        w = k::add(&w, &wj);
        wd = wd_new;
        let (m, com, inertia) = chain.segment(j);
        let rot = f.seg_r[j];
        rc[j] = k::mat_vec(&rot, &com);
        let ac = k::add(&acc, &k::add(&k::cross(&wd, &rc[j]), &k::cross(&w, &k::cross(&w, &rc[j]))));
        force[j] = [m * ac[0], m * ac[1], m * ac[2]];
        let iw = k::mat_mul(&k::mat_mul(&rot, &inertia), &k::transpose(&rot));
        moment[j] = k::add(&k::mat_vec(&iw, &wd), &k::cross(&w, &k::mat_vec(&iw, &w)));
        o_prev = f.origins[j];
    }
    let mut tau = [0.0f64; MAX_CHAIN_JOINTS];
    let (mut f_next, mut n_next) = ([0.0f64; 3], [0.0f64; 3]);
    for j in (0..n).rev() {
        let lever = if j + 1 < n { k::sub(&f.origins[j + 1], &f.origins[j]) } else { [0.0; 3] };
        let nj = k::add(&k::add(&moment[j], &k::cross(&rc[j], &force[j])), &k::add(&n_next, &k::cross(&lever, &f_next)));
        f_next = k::add(&force[j], &f_next);
        n_next = nj;
        tau[j] = k::dot(&f.axes[j], &nj);
    }
    tau
}

/// Kinetic energy (J) at `q`, `qd`, from the segment velocities (independent of [`rnea`]).
pub fn kinetic_energy(chain: &KinematicChain, q: &[f64], qd: &[f64]) -> f64 {
    let f = chain.frames(q);
    let (mut w, mut v) = ([0.0f64; 3], [0.0f64; 3]);
    let mut o_prev = [0.0f64; 3];
    let mut e = 0.0;
    for j in 0..f.n {
        v = k::add(&v, &k::cross(&w, &k::sub(&f.origins[j], &o_prev)));
        let a = f.axes[j];
        w = k::add(&w, &[a[0] * qd[j], a[1] * qd[j], a[2] * qd[j]]);
        let (m, com, inertia) = chain.segment(j);
        let rot = f.seg_r[j];
        let rcj = k::mat_vec(&rot, &com);
        let vc = k::add(&v, &k::cross(&w, &rcj));
        let iw = k::mat_mul(&k::mat_mul(&rot, &inertia), &k::transpose(&rot));
        e += 0.5 * m * k::dot(&vc, &vc) + 0.5 * k::dot(&w, &k::mat_vec(&iw, &w));
        o_prev = f.origins[j];
    }
    e
}

/// Chain of the built-in B601-DM model, built once from the embedded URDF.
/// The built-in B601-DM chain from the embedded URDF with the given motor -> model offsets
/// (`model_offsets_rad`). Built once per distinct offset set and kept for the process life
/// (in practice one set per profile).
pub fn rebot_dm_chain(offsets: &[f64; 6]) -> &'static KinematicChain {
    static CHAINS: OnceLock<std::sync::Mutex<Vec<([u64; 6], &'static KinematicChain)>>> = OnceLock::new();
    let key = offsets.map(f64::to_bits);
    let mut chains = CHAINS.get_or_init(Default::default).lock().unwrap_or_else(|e| e.into_inner());
    if let Some((_, c)) = chains.iter().find(|(k, _)| *k == key) {
        return c;
    }
    let spec = ChainSpec {
        base_link: "base_link".into(),
        tip_link: "end_link".into(),
        tip_rpy_rad: [0.0; 3],
        canonical_end_frame: false,
        joints: (1..=6).map(|i| ChainJointSpec { urdf_joint: format!("joint{i}"), sign: 1.0, offset_rad: offsets[i - 1] }).collect(),
    };
    let chain: &'static KinematicChain = Box::leak(Box::new(
        KinematicChain::from_urdf_str(include_str!("../assets/reBot-DevArm_gripper.urdf"), &spec)
            .expect("embedded B601-DM URDF builds a chain"),
    ));
    chains.push((key, chain));
    chain
}

impl ArmModel {
    /// The chain the dynamics use (the built-in model's chain from the embedded URDF).
    pub fn dynamics_chain(&self) -> &KinematicChain {
        match self { Self::RebotDevArm(off) => rebot_dm_chain(off), Self::Chain(c) => c }
    }

    /// [`rnea`] for this model.
    pub fn inverse_dynamics(&self, q: &[f64], qd: &[f64], qdd: &[f64], gravity: bool) -> [f64; MAX_CHAIN_JOINTS] {
        rnea(self.dynamics_chain(), q, qd, qdd, gravity)
    }
}
