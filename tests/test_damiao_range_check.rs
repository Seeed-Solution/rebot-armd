//! Startup check of the Damiao mapping ranges (registers 10, 21-23) against the
//! profile. The motors are simulated by MockCanTransport; register replies use the
//! layout read on the B601-DM: `[id, 0, 0x33, rid, value LE]` on 0x10 + id.
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, DamiaoArmDriver, DamiaoControlMode, JointMode, JointTarget, RangeCheckStatus};
use armd::model::{ArmConfig, DamiaoRange};
use armd::protocol::damiao;
use armd::transport::{CanFrame, MockCanTransport};
use armd::ArmSupervisor;

fn reply(id: u16, rid: u8, value: [u8; 4]) -> CanFrame {
    CanFrame::new_standard(0x10 + id, &[id as u8, 0, damiao::REG_READ, rid, value[0], value[1], value[2], value[3]]).unwrap()
}

/// Motor `id` answers register reads with CTRL_MODE `mode` and `ranges`.
fn motor(can: &mut MockCanTransport, id: u16, mode: u32, ranges: [f32; 3]) {
    can.respond_to_transmit(damiao::build_read_register_frame(id, damiao::RID_CTRL_MODE), reply(id, damiao::RID_CTRL_MODE, mode.to_le_bytes()));
    for (rid, v) in [(damiao::RID_PMAX, ranges[0]), (damiao::RID_VMAX, ranges[1]), (damiao::RID_TMAX, ranges[2])] {
        can.respond_to_transmit(damiao::build_read_register_frame(id, rid), reply(id, rid, v.to_le_bytes()));
    }
}

fn feedback(id: u8, state: u8) -> CanFrame {
    CanFrame::new_standard(0x10 + id as u16, &[(state << 4) | id, 0x80, 0x00, 0x80, 0x08, 0x00, 30, 30]).unwrap()
}

const J1_J3: [f32; 3] = [12.5, 10.0, 28.0];
const J4_J7: [f32; 3] = [12.5, 30.0, 10.0];

fn profile_ranges(i: u16) -> [f32; 3] { if i <= 3 { J1_J3 } else { J4_J7 } }

fn rebot_dm_driver() -> DamiaoArmDriver<MockCanTransport> {
    let limits = ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(DamiaoRange::limits).collect();
    let mut d = DamiaoArmDriver::new(MockCanTransport::new(), (1..=7).collect()).with_joint_limits(limits).unwrap();
    for id in 1..=7u16 {
        d.can_mut().respond_to_transmit(damiao::build_enable_frame(id), feedback(id as u8, 1));
    }
    d
}

fn sent_enable(tx: &[CanFrame]) -> bool { tx.iter().any(|f| f.data == damiao::CMD_ENABLE) }

#[test]
fn matching_motors_pass_without_enabling_anything() {
    let mut d = rebot_dm_driver();
    for id in 1..=7 { motor(d.can_mut(), id, damiao::CTRL_MODE_POS_VEL, profile_ranges(id)); }
    let check = d.check_mapping_ranges().clone();
    assert_eq!(check.status, RangeCheckStatus::Passed, "{}", check.message);
    assert_eq!(check.joints.len(), 7);
    assert!(check.joints.iter().all(|j| j.ctrl_mode == Some(2) && j.reported == Some(j.configured)));
    let tx = d.can_mut().take_tx_frames();
    // only read-only frames on 0x7FF: 4 register reads per motor, plus the 0xCC status
    // refresh the feedback poll sends to motors without recent feedback
    assert!(tx.iter().all(|f| f.id == 0x7FF && (f.data[2] == damiao::REG_READ || f.data[2] == 0xCC)), "{tx:?}");
    assert_eq!(tx.iter().filter(|f| f.data[2] == damiao::REG_READ).count(), 28);
    assert!(!sent_enable(&tx));
    assert_eq!(d.range_check().unwrap().status, RangeCheckStatus::Passed);
    d.set_torque(true).unwrap();
    // ranges verified at startup: the first switch to MIT does not read them again
    for id in 1..=7u16 {
        for mode in [damiao::CTRL_MODE_MIT, damiao::CTRL_MODE_POS_VEL] {
            d.can_mut().respond_to_transmit(damiao::build_set_control_mode_frame(id, mode),
                CanFrame::new_standard(0x10 + id, &{ let m = mode.to_le_bytes(); [id as u8, 0, damiao::REG_WRITE, damiao::RID_CTRL_MODE, m[0], m[1], m[2], m[3]] }).unwrap());
        }
    }
    d.can_mut().take_tx_frames();
    let mut modes = vec![JointMode::Mit; 6];
    modes.push(JointMode::Position);
    let hold: Vec<JointTarget> = (0..7).map(|i| if i < 6 { JointTarget::mit(0.0, 0.0, 30.0, 1.0, 0.0) } else { JointTarget::position(0.0) }).collect();
    d.set_joint_modes(&modes, &hold).unwrap();
    assert!(!d.can_mut().take_tx_frames().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ));
}

#[test]
fn mismatch_refuses_to_arm() {
    let mut d = rebot_dm_driver();
    for id in 1..=7 {
        // J2 still carries the old single range
        let r = if id == 2 { [12.5, 30.0, 18.0] } else { profile_ranges(id) };
        motor(d.can_mut(), id, damiao::CTRL_MODE_POS_VEL, r);
    }
    let check = d.check_mapping_ranges().clone();
    assert_eq!(check.status, RangeCheckStatus::Mismatch);
    assert_eq!(check.joints[1].status, RangeCheckStatus::Mismatch);
    assert!(check.message.contains("J2 reports PMAX/VMAX/TMAX 12.500/30.000/18.000 (configured 12.500/10.000/28.000)"), "{}", check.message);
    d.can_mut().take_tx_frames();
    let err = d.set_torque(true).unwrap_err().to_string();
    assert!(err.contains("mapping range mismatch"), "{err}");
    assert!(!sent_enable(&d.can_mut().take_tx_frames()), "no enable frame after a mismatch");
    assert!(!d.is_enabled());
}

#[test]
fn silent_motor_is_unverified_within_the_time_budget() {
    let mut d = rebot_dm_driver();
    for id in 1..=7 { if id != 4 { motor(d.can_mut(), id, damiao::CTRL_MODE_POS_VEL, profile_ranges(id)); } }
    let t = Instant::now();
    let check = d.check_mapping_ranges().clone();
    let one_silent = t.elapsed();
    assert_eq!(check.status, RangeCheckStatus::Unverified);
    assert_eq!(check.joints[3].status, RangeCheckStatus::Unverified);
    assert!(check.joints[3].reported.is_none() && check.joints[3].ctrl_mode.is_none());
    assert!(check.message.contains("[4]"), "{}", check.message);
    assert!(one_silent < Duration::from_millis(60), "{one_silent:?}");
    // arming still works with the configured ranges (the check is retried before enable)
    d.set_torque(true).unwrap();
    assert!(d.is_enabled());
    assert_eq!(d.range_check().unwrap().status, RangeCheckStatus::Unverified);

    // all seven silent: the worst case stays below 200 ms
    let mut all = rebot_dm_driver();
    let t = Instant::now();
    assert_eq!(all.check_mapping_ranges().status, RangeCheckStatus::Unverified);
    assert!(t.elapsed() < Duration::from_millis(200), "{:?}", t.elapsed());
}

#[test]
fn a_motor_answering_later_turns_the_check_into_passed_at_enable() {
    let mut d = rebot_dm_driver();
    for id in 1..=6 { motor(d.can_mut(), id, damiao::CTRL_MODE_POS_VEL, profile_ranges(id)); }
    assert_eq!(d.check_mapping_ranges().status, RangeCheckStatus::Unverified);
    motor(d.can_mut(), 7, damiao::CTRL_MODE_POS_VEL, J4_J7);
    d.set_torque(true).unwrap();
    assert_eq!(d.range_check().unwrap().status, RangeCheckStatus::Passed);
}

#[test]
fn joint_left_in_mit_is_restored_before_enable() {
    let mut d = rebot_dm_driver();
    for id in 1..=7 {
        let mode = if id == 3 { damiao::CTRL_MODE_MIT } else { damiao::CTRL_MODE_POS_VEL };
        motor(d.can_mut(), id, mode, profile_ranges(id));
    }
    let check = d.check_mapping_ranges().clone();
    assert_eq!(check.status, RangeCheckStatus::Passed);
    assert_eq!(check.joints[2].ctrl_mode, Some(1));
    assert_eq!(d.joint_control_modes()[2], None);
    d.can_mut().respond_to_transmit(damiao::build_set_control_mode_frame(3, damiao::CTRL_MODE_POS_VEL),
        CanFrame::new_standard(0x13, &[3, 0, damiao::REG_WRITE, damiao::RID_CTRL_MODE, 2, 0, 0, 0]).unwrap());
    d.can_mut().take_tx_frames();
    d.set_torque(true).unwrap();
    let tx = d.can_mut().take_tx_frames();
    let write = tx.iter().position(|f| f.id == 0x7FF && f.data[..5] == [3, 0, damiao::REG_WRITE, damiao::RID_CTRL_MODE, 2]).expect("POS_VEL write");
    let enable = tx.iter().position(|f| f.data == damiao::CMD_ENABLE).expect("enable");
    assert!(write < enable);
    assert_eq!(d.joint_control_modes()[2], Some(DamiaoControlMode::PosVel));
}

#[tokio::test]
async fn gravity_comp_needs_a_passed_range_check_and_the_observation_publishes_it() {
    let mut d = rebot_dm_driver();
    for id in 1..=6 { motor(d.can_mut(), id, damiao::CTRL_MODE_POS_VEL, profile_ranges(id)); }
    d.check_mapping_ranges();
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(d));
    let obs = sup.get_observation().await;
    let check = obs.driver_range_check.expect("published");
    assert_eq!(check.status, RangeCheckStatus::Unverified);
    let json = serde_json::to_value(sup.get_observation().await).unwrap();
    assert_eq!(json["driver_range_check"]["status"], "unverified");
    assert_eq!(json["driver_range_check"]["joints"][6]["reported"], serde_json::Value::Null);
    assert!(obs.gravity_comp.supported);
    let blocker = sup.inner().read().await.gravity_comp_blocker().unwrap();
    assert_eq!(blocker.code, "DRIVER_RANGE_UNVERIFIED");
    assert_eq!(obs.gravity_comp.blocker.unwrap().code, "DRIVER_RANGE_UNVERIFIED");
}
