//! Accessor matrix of the `ArmDriver` capability extensions (S4 step 1, Fable spec §3.3).
//!
//! A driver that implements an `*Ext` trait but does not override its accessor pair silently
//! reports "unsupported" (`NoExt`). This table pins which extensions each follower exposes.

use std::net::UdpSocket;

use armd::driver::{
    ArmDriver, DamiaoArmDriver, FashionStarArmDriver, FeetechArmDriver, RobStrideArmDriver,
};
use armd::sim_driver::SimArmDriver;
use armd::transport::{MockCanTransport, MockTransport};

/// Extension columns, in order.
const COLS: [&str; 9] = [
    "tx", "mode_switch", "commissioning", "joint_mode", "mit", "pos_force", "bus_recovery", "shadow", "joint_zero",
];

fn shared(d: &dyn ArmDriver) -> [bool; 9] {
    [
        d.tx_ext().is_some(),
        d.mode_switch_ext().is_some(),
        d.commissioning_ext().is_some(),
        d.joint_mode_ext().is_some(),
        d.mit_ext().is_some(),
        d.pos_force_ext().is_some(),
        d.bus_recovery_ext().is_some(),
        d.shadow_ext().is_some(),
        d.joint_zero_ext().is_some(),
    ]
}

fn exclusive(d: &mut dyn ArmDriver) -> [bool; 9] {
    [
        d.tx_ext_mut().is_some(),
        d.mode_switch_ext_mut().is_some(),
        d.commissioning_ext_mut().is_some(),
        d.joint_mode_ext_mut().is_some(),
        d.mit_ext_mut().is_some(),
        d.pos_force_ext_mut().is_some(),
        d.bus_recovery_ext_mut().is_some(),
        d.shadow_ext_mut().is_some(),
        d.joint_zero_ext_mut().is_some(),
    ]
}

pub fn assert_matrix(name: &str, d: &mut dyn ArmDriver, want: [bool; 9]) {
    let got = shared(d);
    let got_mut = exclusive(d);
    for i in 0..COLS.len() {
        assert_eq!(got[i], want[i], "{name}: {}_ext()", COLS[i]);
        assert_eq!(got_mut[i], want[i], "{name}: {}_ext_mut()", COLS[i]);
    }
}

const T: bool = true;
const F: bool = false;

#[test]
fn follower_accessor_matrix() {
    //                                    tx mode comm jmode mit pf  bus shadow jzero
    let damiao = [T, T, T, T, T, T, T, T, F];
    let sim = [F, F, T, T, F, F, F, T, F];
    let legacy = [F, F, F, F, F, F, F, T, F];

    let mut dm = DamiaoArmDriver::new(MockCanTransport::new(), (1..=7).collect());
    assert_matrix("Damiao", &mut dm, damiao);
    let mut ft = FeetechArmDriver::new(MockTransport::new(), vec![1, 2, 3, 4, 5, 6]);
    assert_matrix("Feetech", &mut ft, legacy);
    let mut fs = FashionStarArmDriver::new(MockTransport::new(), vec![1, 2, 3, 4, 5, 6]);
    assert_matrix("FashionStar", &mut fs, legacy);
    let mut rs = RobStrideArmDriver::new(MockCanTransport::new(), (1..=6).collect());
    assert_matrix("RobStride", &mut rs, legacy);
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    let mut sm = SimArmDriver::connect(&format!("udp://{}", sock.local_addr().unwrap()), 7).unwrap();
    assert_matrix("Sim", &mut sm, sim);
}

#[test]
fn getters_fall_back_to_the_default_bodies() {
    // A follower without the cluster answers through `NoExt`: today's trait defaults.
    let mut ft: Box<dyn ArmDriver> = Box::new(FeetechArmDriver::new(MockTransport::new(), vec![1, 2, 3, 4, 5, 6]));
    assert_eq!(ft.tx().release_budget_consumed(), (0, 0));
    assert!(ft.tx().tx_probe_confirmed());
    assert_eq!(ft.tx().last_target_tx_sent_mask(), u64::MAX);
    assert!(ft.pos_force().pos_force_feedback_fresh());
    assert!(ft.joint_mode().joint_hold_mode_verified(0));
    assert_eq!(ft.mode_switch().mode_switch_remaining_rx_budget(17), 17);
    assert!(ft.mit().mit_feedback_after_recovery(6));
    assert!(ft.joint_zero().joint_zero_offsets().is_none());
    let mut out = vec![3usize];
    ft.joint_mode_mut().take_recovery_writes_into(&mut out);
    assert!(out.is_empty());
    assert!(ft.commissioning_mut().set_zero_position(0).is_err());
}

#[test]
fn temporary_call_site_bridge_is_gone() {
    // S4 PR 1a added src/driver/bridge.rs (old method names on dyn ArmDriver); PR 1d removed it.
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    assert!(!root.join("src/driver/bridge.rs").exists(), "src/driver/bridge.rs must not come back");
    let driver = std::fs::read_to_string(root.join("src/driver.rs")).unwrap();
    assert!(!driver.lines().any(|l| l.trim() == "mod bridge;"), "src/driver.rs must not declare mod bridge");
}
