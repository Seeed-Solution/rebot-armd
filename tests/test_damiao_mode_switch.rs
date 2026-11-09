//! Damiao per-joint control-mode switch (POS_VEL <-> MIT) used by gravity compensation.
//! Frame layouts follow Damiao's DM_CAN reference library; the motors' answers here are
//! simulated by MockCanTransport and are not a hardware verification.
use armd::driver::{ArmDriver, DamiaoArmDriver, DamiaoControlMode, JointMode, JointTarget};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::transport::{CanFrame, MockCanTransport};

fn feedback_frame(id: u8, state: u8, position: f32) -> CanFrame {
    let limits = DamiaoLimits::default();
    let p = damiao::float_to_uint(position, limits.p_min, limits.p_max, 16);
    let mut data = [0u8; 8];
    data[0] = (state << 4) | (id & 0x0f);
    data[1] = (p >> 8) as u8;
    data[2] = p as u8;
    CanFrame::new_standard(0x10 + id as u16, &data).unwrap()
}

fn reply(id: u16, op: u8, rid: u8, value: [u8; 4]) -> CanFrame {
    let [lo, hi] = id.to_le_bytes();
    CanFrame::new_standard(0, &[lo, hi, op, rid, value[0], value[1], value[2], value[3]]).unwrap()
}

/// Motor model on the mock bus: answers range reads with `ranges` and acknowledges mode
/// writes when `ack_modes` is set.
fn answer(can: &mut MockCanTransport, id: u16, ranges: [f32; 3], ack_modes: bool) {
    for (rid, v) in [(damiao::RID_PMAX, ranges[0]), (damiao::RID_VMAX, ranges[1]), (damiao::RID_TMAX, ranges[2])] {
        can.respond_to_transmit(damiao::build_read_register_frame(id, rid), reply(id, damiao::REG_READ, rid, v.to_le_bytes()));
    }
    if ack_modes {
        for mode in [damiao::CTRL_MODE_MIT, damiao::CTRL_MODE_POS_VEL] {
            can.respond_to_transmit(damiao::build_set_control_mode_frame(id, mode),
                reply(id, damiao::REG_WRITE, damiao::RID_CTRL_MODE, mode.to_le_bytes()));
        }
    }
}

const DEFAULT_RANGES: [f32; 3] = [12.5, 30.0, 18.0];

fn enabled_driver(ids: &[u16]) -> DamiaoArmDriver<MockCanTransport> {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), ids.to_vec());
    for &id in ids {
        driver.can_mut().respond_to_transmit(damiao::build_enable_frame(id), feedback_frame(id as u8, 1, 0.2));
    }
    driver.set_torque(true).unwrap();
    driver.can_mut().take_tx_frames();
    driver
}

fn mit_hold(p: f32) -> JointTarget { JointTarget::mit(p, 0.0, 30.0, 1.0, -2.0) }

#[test]
fn register_frames_use_the_dm_can_layout() {
    let w = damiao::build_set_control_mode_frame(0x102, damiao::CTRL_MODE_MIT);
    assert_eq!(w.id, 0x7FF);
    assert_eq!(w.data, [0x02, 0x01, 0x55, 10, 1, 0, 0, 0]);
    let r = damiao::build_read_register_frame(3, damiao::RID_TMAX);
    assert_eq!(r.data, [3, 0, 0x33, 23, 0, 0, 0, 0]);
    let parsed = damiao::parse_register_reply(&reply(3, 0x33, 23, 18.0f32.to_le_bytes())).unwrap();
    assert_eq!((parsed.motor_id, parsed.write, parsed.rid), (3, false, 23));
    assert_eq!(parsed.as_f32(), 18.0);
    // An enabled motor's feedback frame is not mistaken for a reply.
    assert!(damiao::parse_register_reply(&feedback_frame(3, 1, 0.0)).is_none());
}

#[test]
fn switch_to_mit_writes_ctrl_mode_then_holds_in_the_new_mode() {
    let mut driver = enabled_driver(&[1, 2]);
    for id in [1, 2] { answer(driver.can_mut(), id, DEFAULT_RANGES, true); }
    assert!(driver.supports_mit_mode_switch());
    driver.set_joint_modes(&[JointMode::Mit, JointMode::Position], &[mit_hold(0.2), JointTarget::position(0.2)]).unwrap();
    let tx = driver.can_mut().take_tx_frames();
    // range readback for the joint entering MIT, then the mode write, then the MIT hold frame
    let reads: Vec<u8> = tx.iter().filter(|f| f.id == 0x7FF && f.data[2] == 0x33).map(|f| f.data[3]).collect();
    assert_eq!(reads, vec![21, 22, 23]);
    let w = tx.iter().position(|f| f.id == 0x7FF && f.data[2] == 0x55).expect("mode write");
    assert_eq!(tx[w].data[..5], [1, 0, 0x55, 10, 1]);
    assert_eq!(tx[w + 1].id, 1, "the MIT hold frame must directly follow the mode write");
    assert!(!tx.iter().any(|f| f.id == 0x7FF && f.data[0] == 2 && f.data[2] == 0x55), "joint 2 keeps POS_VEL");
    assert_eq!(driver.joint_control_modes(), &[Some(DamiaoControlMode::Mit), Some(DamiaoControlMode::PosVel)]);
    assert!(!driver.joint_modes_uncertain());

    // Writes now go out per joint: MIT for joint 1, POS_VEL for joint 2.
    driver.write_joint_targets(&[mit_hold(0.25), JointTarget::position(0.3)]).unwrap();
    let tx = driver.can_mut().take_tx_frames();
    assert_eq!(tx.iter().map(|f| f.id).collect::<Vec<_>>(), vec![1, 0x102]);
    let expected = damiao::build_mit_frame(1, 0.25, 0.0, 30.0, 1.0, -2.0, &DamiaoLimits::default());
    assert_eq!(tx[0].data, expected.data);

    // and back
    driver.set_joint_modes(&[JointMode::Position, JointMode::Position], &[JointTarget::position(0.25), JointTarget::position(0.3)]).unwrap();
    let tx = driver.can_mut().take_tx_frames();
    assert!(!tx.iter().any(|f| f.data[2] == 0x33), "ranges are read once");
    let w = tx.iter().position(|f| f.id == 0x7FF && f.data[2] == 0x55).unwrap();
    assert_eq!(tx[w].data[..5], [1, 0, 0x55, 10, 2]);
    assert_eq!(tx[w + 1].id, 0x101);
    assert_eq!(driver.joint_control_modes(), &[Some(DamiaoControlMode::PosVel); 2]);
}

#[test]
fn range_mismatch_refuses_mit_without_touching_the_mode() {
    let mut driver = enabled_driver(&[1]);
    answer(driver.can_mut(), 1, [12.5, 10.0, 28.0], true);
    let err = driver.set_joint_modes(&[JointMode::Mit], &[mit_hold(0.2)]).unwrap_err().to_string();
    assert!(err.contains("TMAX 28.000"), "{err}");
    let tx = driver.can_mut().take_tx_frames();
    assert!(!tx.iter().any(|f| f.id == 0x7FF && f.data[2] == 0x55), "no mode write after a failed readback");
    assert_eq!(driver.joint_control_modes(), &[Some(DamiaoControlMode::PosVel)]);
}

#[test]
fn silent_motor_refuses_mit() {
    let mut driver = enabled_driver(&[1]);
    let err = driver.set_joint_modes(&[JointMode::Mit], &[mit_hold(0.2)]).unwrap_err().to_string();
    assert!(err.contains("did not answer"), "{err}");
    assert_eq!(driver.joint_control_modes(), &[Some(DamiaoControlMode::PosVel)]);
}

#[test]
fn unacknowledged_switch_is_reverted_with_dual_hold() {
    let mut driver = enabled_driver(&[1]);
    answer(driver.can_mut(), 1, DEFAULT_RANGES, false);
    // only the revert to POS_VEL is acknowledged
    driver.can_mut().respond_to_transmit(damiao::build_set_control_mode_frame(1, damiao::CTRL_MODE_POS_VEL),
        reply(1, damiao::REG_WRITE, damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL.to_le_bytes()));
    let err = driver.set_joint_modes(&[JointMode::Mit], &[mit_hold(0.2)]).unwrap_err().to_string();
    assert!(err.contains("not confirmed") && !err.contains("still unconfirmed"), "{err}");
    let tx = driver.can_mut().take_tx_frames();
    let revert = tx.iter().rposition(|f| f.id == 0x7FF && f.data[2] == 0x55).unwrap();
    assert_eq!(tx[revert].data[4], 2);
    assert_eq!(tx[revert + 1].id, 0x101, "revert holds with a POS_VEL frame ...");
    assert_eq!(tx[revert + 2].id, 1, "... and an MIT frame");
    assert!(!driver.joint_modes_uncertain());
}

#[test]
fn unconfirmed_mode_gets_both_frames_and_enable_restores_pos_vel() {
    let mut driver = enabled_driver(&[1]);
    answer(driver.can_mut(), 1, DEFAULT_RANGES, false);
    assert!(driver.set_joint_modes(&[JointMode::Mit], &[mit_hold(0.2)]).unwrap_err().to_string().contains("still unconfirmed"));
    assert!(driver.joint_modes_uncertain());
    driver.can_mut().take_tx_frames();
    driver.write_joint_targets(&[mit_hold(0.2)]).unwrap();
    let ids: Vec<u32> = driver.can_mut().take_tx_frames().iter().map(|f| f.id).collect();
    assert_eq!(ids, vec![0x101, 1]);

    // A later enable handshake first restores POS_VEL (refused while unconfirmed).
    driver.set_torque(false).unwrap();
    driver.set_pending_rearm(true);
    assert!(driver.set_torque(true).is_err(), "enable must not proceed with an unconfirmed mode");
    driver.can_mut().respond_to_transmit(damiao::build_set_control_mode_frame(1, damiao::CTRL_MODE_POS_VEL),
        reply(1, damiao::REG_WRITE, damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL.to_le_bytes()));
    driver.set_torque(true).unwrap();
    assert_eq!(driver.joint_control_modes(), &[Some(DamiaoControlMode::PosVel)]);
}

#[test]
fn register_reply_is_not_parsed_as_feedback() {
    let mut driver = enabled_driver(&[2]);
    let before = driver.read_joint_states().unwrap()[0].position;
    driver.can_mut().provide_rx_frame(reply(2, damiao::REG_WRITE, damiao::RID_CTRL_MODE, [1, 0, 0, 0]));
    let after = driver.read_joint_states().unwrap()[0].position;
    assert_eq!(before, after);
    assert_eq!(driver.motor_status().unwrap(), vec![1], "status must not drop to 0 (disabled)");
}

#[test]
fn mit_base_mode_driver_does_not_offer_the_switch() {
    let driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]).with_control_mode(DamiaoControlMode::Mit);
    assert!(!driver.supports_mit_mode_switch());
}
