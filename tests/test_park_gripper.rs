//! Park with an object in the gripper: the joint quintic to home (J7 = closed) must not squeeze
//! it, and at the resting pose the jaw opens to drop it, closes, then torque goes off.

use armd::model::ArmConfig;
use armd::supervisor::GRIPPER_STALL_LATCH_NM;
use armd::{StopPolicy, SupervisorInner};

const DT: f64 = 0.02;
/// Object in the jaw: J7 stops here while closing (policy closed 0.0, open -5.7).
const OBJ: f32 = -2.0;
/// Mock squeeze torque per rad of command past the contact (see `mock_gripper_object`).
const MOCK_K: f32 = 2.0;

fn sim_holding(q: [f32; 7]) -> SupervisorInner {
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.commanded_positions = q.to_vec();
    inner.measured_positions = q.to_vec();
    inner.target_positions = q.to_vec();
    inner.mock_gripper_object = Some((OBJ, false));
    inner
}

struct Run { max_tau7: f32, max_open: f32, min_after_open: f32, torque_off: bool, ticks: usize }

fn park(inner: &mut SupervisorInner) -> Run {
    inner.apply_stop_policy(StopPolicy::NormalUnload);
    assert!(inner.pending_normal_unload, "park planned");
    let mut r = Run { max_tau7: 0.0, max_open: f32::INFINITY, min_after_open: f32::NEG_INFINITY, torque_off: false, ticks: 0 };
    for k in 0..3000 {
        let obs = inner.tick(DT);
        r.max_tau7 = r.max_tau7.max(obs.torques[6].abs());
        let q7 = obs.positions[6];
        if q7 < r.max_open { r.max_open = q7; }
        if r.max_open < OBJ - 1.0 { r.min_after_open = r.min_after_open.max(q7); }
        if !obs.is_torqued { r.torque_off = true; r.ticks = k; break; }
    }
    r
}

#[test]
fn park_holding_an_object_does_not_squeeze_and_releases_at_rest() {
    let mut q = armd::ArmConfig::rebot_dm().home_positions.clone();
    q[0] = 0.4;
    q[1] = 0.5;
    q[6] = -3.0; // open, the jaw closes onto the object at -2.0 during the park
    let mut arm: [f32; 7] = [0.0; 7];
    arm.copy_from_slice(&q);
    let mut inner = sim_holding(arm);
    let r = park(&mut inner);
    eprintln!("max |tau7| {:.2} N·m, most open {:.2} rad, closed back to {:.2}, torque off after {:.1} s",
        r.max_tau7, r.max_open, r.min_after_open, r.ticks as f64 * DT);
    // stall latch fires at 0.444 N·m after 6 ticks; the freeze keeps the push at contact level
    assert!(r.max_tau7 < GRIPPER_STALL_LATCH_NM * 1.5 + 0.5 * MOCK_K, "squeezed: {:.2} N·m", r.max_tau7);
    assert!(r.max_open <= OBJ - 1.9, "opened to drop the object: {:.2}", r.max_open);
    assert!(inner.mock_gripper_object.is_none(), "object released");
    assert!(r.min_after_open > -0.05, "closed again after the release: {:.2}", r.min_after_open);
    assert!(r.torque_off, "torque off at the resting pose");
}

#[test]
fn park_with_an_empty_gripper_does_not_open() {
    let mut q = armd::ArmConfig::rebot_dm().home_positions.clone();
    q[0] = 0.4;
    q[6] = -3.0;
    let mut arm: [f32; 7] = [0.0; 7];
    arm.copy_from_slice(&q);
    let mut inner = sim_holding(arm);
    inner.mock_gripper_object = None;
    let r = park(&mut inner);
    assert!(r.max_open >= -3.0 - 1e-4, "no release move: {:.2}", r.max_open);
    assert!(r.torque_off);
    assert!(inner.park_release.is_none() && !inner.park_gripper_contact);
}
