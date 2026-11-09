//! Per-joint Damiao mapping ranges. The B601-DM motors report (registers 21 / 22 / 23,
//! read on hardware 2026-11-09) PMAX 12.5, VMAX 10, TMAX 28 on J1-J3 and PMAX 12.5,
//! VMAX 30, TMAX 10 on J4-J7; the driver used to decode all of them with 12.5 / 30 / 18.
use armd::driver::{ArmDriver, DamiaoArmDriver, JointMode, JointTarget};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::transport::{CanFrame, MockCanTransport};
use armd::model::{ArmConfig, DamiaoRange};

const J1_J3: [f32; 3] = [12.5, 10.0, 28.0];
const J4_J7: [f32; 3] = [12.5, 30.0, 10.0];

fn lim(r: [f32; 3]) -> DamiaoLimits { DamiaoLimits::symmetric(r[0], r[1], r[2]) }

/// Feedback frame with raw fields: position 0x8000, velocity 3000, torque 2600, state 1.
fn raw_feedback(id: u8) -> CanFrame {
    CanFrame::new_standard(0x10 + id as u16, &[0x10 | id, 0x80, 0x00, 0xBB, 0x8A, 0x28, 30, 32]).unwrap()
}

fn close(a: f32, b: f32) -> bool { (a - b).abs() < 1e-4 }

#[test]
fn decode_vectors_for_both_motor_types() {
    let f = raw_feedback(2);
    let j2 = damiao::unpack_feedback_frame(&f, &lim(J1_J3)).unwrap();
    assert!(close(j2.velocity, 4.65201) && close(j2.torque, 7.55556), "{j2:?}");
    let j5 = damiao::unpack_feedback_frame(&raw_feedback(5), &lim(J4_J7)).unwrap();
    assert!(close(j5.velocity, 13.95604) && close(j5.torque, 2.69841), "{j5:?}");
    // The old single range read the same J2 frame 3x too fast and at 18/28 of the torque,
    // and the J5 torque at 18/10 of the real value.
    let old = damiao::unpack_feedback_frame(&f, &DamiaoLimits::default()).unwrap();
    assert!(close(old.velocity, 13.95604) && close(old.torque, 4.85714), "{old:?}");
    assert!(close(old.velocity / j2.velocity, 3.0) && close(old.torque / j2.torque, 18.0 / 28.0));
    let old5 = damiao::unpack_feedback_frame(&raw_feedback(5), &DamiaoLimits::default()).unwrap();
    assert!(close(old5.torque / j5.torque, 1.8));
    // Position range is the same on both motor types.
    assert!(close(j2.position, j5.position) && close(j2.position, old.position));
}

fn rebot_dm_driver() -> DamiaoArmDriver<MockCanTransport> {
    let limits = ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(DamiaoRange::limits).collect();
    DamiaoArmDriver::new(MockCanTransport::new(), (1..=7).collect()).with_joint_limits(limits).unwrap()
}

#[test]
fn driver_decodes_each_joint_with_its_own_range() {
    let mut driver = rebot_dm_driver();
    let frames: Vec<CanFrame> = (1..=7).map(raw_feedback).collect();
    driver.can_mut().provide_rx_frames(&frames);
    let states = driver.read_joint_states().unwrap();
    for (i, s) in states.iter().enumerate() {
        let (v, t) = if i < 3 { (4.65201, 7.55556) } else { (13.95604, 2.69841) };
        assert!(close(s.velocity, v) && close(s.torque, t), "joint {}: {s:?}", i + 1);
    }
}

#[test]
fn profile_without_ranges_keeps_the_single_default_range() {
    assert!(ArmConfig::default().damiao_ranges.is_none());
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![2, 5]);
    assert_eq!(driver.joint_limits(), &[DamiaoLimits::default(); 2]);
    driver.can_mut().provide_rx_frames(&[raw_feedback(2), raw_feedback(5)]);
    let states = driver.read_joint_states().unwrap();
    assert!(states.iter().all(|s| close(s.velocity, 13.95604) && close(s.torque, 4.85714)));
}

#[test]
fn joint_limits_length_must_match_the_motor_count() {
    let r = DamiaoArmDriver::new(MockCanTransport::new(), vec![1, 2]).with_joint_limits(vec![lim(J1_J3)]);
    assert!(r.is_err());
}

#[test]
fn rebot_dm_profile_ranges_validate() {
    let mut c = ArmConfig::rebot_dm();
    c.validate().unwrap();
    let r = c.damiao_ranges.clone().unwrap();
    assert_eq!(r[..3], [DamiaoRange::REBOT_DM_J1_J3; 3]);
    assert_eq!(r[3..], [DamiaoRange::REBOT_DM_J4_J7; 4]);
    let mut short = ArmConfig::rebot_dm();
    short.damiao_ranges.as_mut().unwrap().pop();
    assert!(short.validate().is_err());
    // A gravity-comp torque clamp above a joint's TMAX could never be encoded.
    let mut over = ArmConfig::rebot_dm();
    over.gravity_comp.as_mut().unwrap().torque_limits_nm[4] = 10.5;
    assert!(over.validate().unwrap_err().contains("TMAX"));
}

fn reply(id: u16, op: u8, rid: u8, value: [u8; 4]) -> CanFrame {
    let [lo, hi] = id.to_le_bytes();
    CanFrame::new_standard(0x10 + id, &[lo, hi, op, rid, value[0], value[1], value[2], value[3]]).unwrap()
}

fn answer_ranges(can: &mut MockCanTransport, id: u16, r: [f32; 3]) {
    for (rid, v) in [(damiao::RID_PMAX, r[0]), (damiao::RID_VMAX, r[1]), (damiao::RID_TMAX, r[2])] {
        can.respond_to_transmit(damiao::build_read_register_frame(id, rid), reply(id, damiao::REG_READ, rid, v.to_le_bytes()));
    }
    for mode in [damiao::CTRL_MODE_MIT, damiao::CTRL_MODE_POS_VEL] {
        can.respond_to_transmit(damiao::build_set_control_mode_frame(id, mode),
            reply(id, damiao::REG_WRITE, damiao::RID_CTRL_MODE, mode.to_le_bytes()));
    }
}

fn enabled(mut driver: DamiaoArmDriver<MockCanTransport>, ids: &[u16]) -> DamiaoArmDriver<MockCanTransport> {
    for &id in ids {
        driver.can_mut().respond_to_transmit(damiao::build_enable_frame(id), raw_feedback(id as u8));
    }
    driver.set_torque(true).unwrap();
    driver.can_mut().take_tx_frames();
    driver
}

#[test]
fn mit_range_check_and_encoding_are_per_joint() {
    let driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![2, 5]).with_joint_limits(vec![lim(J1_J3), lim(J4_J7)]).unwrap();
    let mut driver = enabled(driver, &[2, 5]);
    answer_ranges(driver.can_mut(), 2, J1_J3);
    answer_ranges(driver.can_mut(), 5, J4_J7);
    let hold = [JointTarget::mit(0.1, 0.0, 30.0, 1.0, 5.0), JointTarget::mit(0.1, 0.0, 10.0, 0.3, 1.0)];
    driver.set_joint_modes(&[JointMode::Mit, JointMode::Mit], &hold).unwrap();
    driver.can_mut().take_tx_frames();
    driver.write_joint_targets(&hold).unwrap();
    let tx = driver.can_mut().take_tx_frames();
    assert_eq!(tx[0].data, damiao::build_mit_frame(2, 0.1, 0.0, 30.0, 1.0, 5.0, &lim(J1_J3)).data);
    assert_eq!(tx[1].data, damiao::build_mit_frame(5, 0.1, 0.0, 10.0, 0.3, 1.0, &lim(J4_J7)).data);
    assert_ne!(tx[0].data, damiao::build_mit_frame(2, 0.1, 0.0, 30.0, 1.0, 5.0, &DamiaoLimits::default()).data);
}

#[test]
fn mit_range_check_rejects_a_joint_reporting_the_other_motor_type() {
    let driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![2, 5]).with_joint_limits(vec![lim(J1_J3), lim(J4_J7)]).unwrap();
    let mut driver = enabled(driver, &[2, 5]);
    answer_ranges(driver.can_mut(), 2, J1_J3);
    answer_ranges(driver.can_mut(), 5, J1_J3);
    let hold = [JointTarget::mit(0.1, 0.0, 30.0, 1.0, 0.0), JointTarget::mit(0.1, 0.0, 10.0, 0.3, 0.0)];
    let err = driver.set_joint_modes(&[JointMode::Mit, JointMode::Mit], &hold).unwrap_err().to_string();
    assert!(err.contains("Joint 2") && err.contains("TMAX 28.000") && err.contains("10.000"), "{err}");
}
