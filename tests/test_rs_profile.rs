//! profiles/rebot_b601_rs.json: the B601-RS URDF chain against independent references
//! (tests/fixtures/gen_rs_reference.py: FK from the MuJoCo model rs_arm.xml, gravity as dV/dq
//! of ReBot_Arm_RS.urdf). The profile is not verified on hardware.
use armd::ArmConfig;

const PROFILE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/profiles/rebot_b601_rs.json");
// MJCF quaternions and URDF rpy are both rounded to ~1e-6 rad.
const FK_TOL: f64 = 1e-5;
const GRAVITY_TOL_NM: f64 = 1e-5;

fn profile() -> ArmConfig {
    let mut c = ArmConfig::from_profile_file(PROFILE).unwrap();
    c.validate().unwrap();
    c
}

#[test]
fn rs_profile_loads() {
    let c = profile();
    let chain = c.arm_model.chain().expect("URDF chain");
    assert_eq!((chain.robot.as_str(), chain.dof(), c.dof, c.gripper_index()), ("ReBot_Arm_RS", 6, 6, None));
    let caps = armd::model::Capabilities::from_config(&c);
    assert!(caps.workspace_check && !caps.cartesian && !caps.gripper && caps.residual_torques);
    let sup = armd::SupervisorInner::new(c.clone());
    assert!(sup.gravity_compensated && sup.workspace_policy().is_some());
    // Home (all zero, arm stretched forward) is inside the workspace.
    let (ws, cam) = sup.workspace_policy().unwrap();
    assert_eq!(c.arm_model.workspace_violation(&c.home_positions, ws, cam).total, 0.0);
}

#[test]
fn rs_chain_matches_mjcf_fk_and_urdf_energy() {
    let c = profile();
    let fixture: serde_json::Value = serde_json::from_str(include_str!("fixtures/rs_reference.json")).unwrap();
    let (mut dp, mut dr, mut dg) = (0.0f64, 0.0f64, 0.0f64);
    for s in fixture["samples"].as_array().unwrap() {
        let q: Vec<f64> = s["q"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
        let pose = c.arm_model.fk(&q);
        for i in 0..3 {
            dp = dp.max((pose.p[i] - s["p"][i].as_f64().unwrap()).abs());
            for j in 0..3 { dr = dr.max((pose.r[i][j] - s["r"][i][j].as_f64().unwrap()).abs()); }
        }
        let tau = c.arm_model.chain().unwrap().gravity_torques(&q);
        for i in 0..6 { dg = dg.max((tau[i] - s["gravity_nm"][i].as_f64().unwrap()).abs()); }
    }
    eprintln!("RS vs reference: FK position {dp:.3e} m, rotation {dr:.3e}, gravity {dg:.3e} N·m");
    assert!(dp < FK_TOL && dr < FK_TOL && dg < GRAVITY_TOL_NM, "{dp:e} {dr:e} {dg:e}");
}

#[test]
fn rs_and_dm_chains_differ() {
    // The RS link lengths are not the DM ones: same joint vector, different tip.
    let rs = profile();
    let q = [0.0f64; 6];
    let d = armd::kinematics::sub(&rs.arm_model.fk(&q).p, &armd::kinematics::fk(&q).p);
    assert!(armd::kinematics::norm(&d) > 0.01, "{d:?}");
}
