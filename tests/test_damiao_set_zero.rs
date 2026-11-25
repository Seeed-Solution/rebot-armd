//! Damiao set-zero sequence of the motor zero tool. The motors' answers are simulated by
//! MockCanTransport; this checks the frames armd sends, not the motors' behaviour.
use armd::driver::{ArmDriver, DamiaoArmDriver};
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

fn ctrl_mode_reply(id: u16, mode: u32) -> CanFrame {
    let [lo, hi] = id.to_le_bytes();
    let [m0, m1, m2, m3] = mode.to_le_bytes();
    CanFrame::new_standard(0x10 + id, &[lo, hi, damiao::REG_READ, damiao::RID_CTRL_MODE, m0, m1, m2, m3]).unwrap()
}

#[test]
fn set_zero_needs_a_confirmed_disabled_motor_and_sends_the_fe_frame() {
    let ids = [1u16, 2, 3];
    let mut d = DamiaoArmDriver::new(MockCanTransport::new(), ids.to_vec());
    assert!(d.motor_zero_support().supported);
    assert_eq!(d.motor_zero_support().persist.as_deref(), Some("motor_flash"));
    assert!(!d.motor_zero_support().persist_verified);
    // no feedback yet: not confirmed disabled
    assert!(d.set_zero_position(1).is_err());
    // disable: every motor answers state 0 (post-settle 0xCC refresh confirms)
    for &id in &ids {
        let fb = feedback_frame(id as u8, 0, 0.4);
        d.can_mut().respond_to_transmit(damiao::build_disable_frame(id), fb);
        d.can_mut().respond_to_transmit(damiao::build_refresh_frame(id), fb);
    }
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..ids.len() {
        arm_joints[i] = i;
    }
    let mut rest_positions = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
    for slot in &mut rest_positions[..ids.len()] {
        *slot = 0.4;
    }
    d.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: ids.len(),
        rest_positions,
        rest_pose_tolerance_rad: 0.05,
    });
    d.read_joint_states().unwrap();
    for step in 0..3 {
        if step > 0 {
            std::thread::sleep(std::time::Duration::from_millis(11));
        }
        for &id in &ids {
            d.can_mut().provide_rx_frame(feedback_frame(id as u8, 0, 0.4));
        }
        d.read_joint_states().unwrap();
    }
    d.set_torque(false).unwrap();
    d.can_mut().take_tx_frames();
    d.set_zero_position(1).unwrap();
    let tx = d.can_mut().take_tx_frames();
    assert_eq!(tx, vec![damiao::build_set_zero_frame(2)], "only the set-zero frame of joint 2");
    assert_eq!(tx[0].data, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]);
    assert!(d.set_zero_position(5).is_err());
}

#[test]
fn set_zero_is_refused_while_enabled_or_when_a_motor_reports_enabled() {
    let ids = [1u16, 2];
    let mut d = DamiaoArmDriver::new(MockCanTransport::new(), ids.to_vec());
    for &id in &ids {
        let fb = feedback_frame(id as u8, 1, 0.2);
        d.can_mut().respond_to_transmit(damiao::build_read_register_frame(id, damiao::RID_CTRL_MODE), ctrl_mode_reply(id, damiao::CTRL_MODE_POS_VEL));
        d.can_mut().respond_to_transmit(damiao::build_enable_frame(id), fb);
        d.can_mut().respond_to_transmit(damiao::build_refresh_frame(id), fb);
    }
    d.set_torque(true).unwrap();
    d.can_mut().take_tx_frames();
    assert!(d.set_zero_position(0).unwrap_err().to_string().contains("torque off"));
    assert!(d.can_mut().take_tx_frames().is_empty());
    // driver flag off but the motor still reports state 1 (enabled)
    d.set_torque_flag(false);
    d.can_mut().provide_rx_frame(feedback_frame(1, 1, 0.2));
    let _ = d.read_joint_states();
    assert!(d.set_zero_position(0).unwrap_err().to_string().contains("not confirmed disabled"));
    assert!(!d.can_mut().take_tx_frames().iter().any(|f| f.data == damiao::CMD_SET_ZERO));
}

/// Accepted disable transmits `DAMIAO_DISABLE_REPEATS` bounded disable frames per motor spaced by
/// `DAMIAO_DISABLE_REPEAT_INTERVAL`, independent of motor feedback (no 150 ms confirmation wait).
#[test]
fn disable_confirmation_is_time_bounded_and_resends_to_unconfirmed_motors() {
    let ids = [1u16, 2, 3];
    let mut d = DamiaoArmDriver::new(MockCanTransport::new(), ids.to_vec());
    for &id in &ids[..2] {
        let fb = feedback_frame(id as u8, 0, 0.1);
        d.can_mut().respond_to_transmit(damiao::build_disable_frame(id), fb);
        d.can_mut().respond_to_transmit(damiao::build_refresh_frame(id), fb);
    }
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..ids.len() {
        arm_joints[i] = i;
    }
    let mut rest_positions = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
    for slot in &mut rest_positions[..ids.len()] {
        *slot = 0.1;
    }
    d.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: ids.len(),
        rest_positions,
        rest_pose_tolerance_rad: 0.05,
    });
    d.read_joint_states().unwrap();
    for step in 0..3 {
        if step > 0 {
            std::thread::sleep(std::time::Duration::from_millis(11));
        }
        for &id in &ids {
            d.can_mut().provide_rx_frame(feedback_frame(id as u8, 0, 0.1));
        }
        d.read_joint_states().unwrap();
    }
    let _ = d.can_mut().take_tx_frames();
    let t0 = std::time::Instant::now();
    d.set_torque(false).unwrap();
    let waited = t0.elapsed();
    let tx = d.can_mut().take_tx_frames();
    let sent = |id: u16| tx.iter().filter(|f| **f == damiao::build_disable_frame(id)).count();
    let expected_min = armd::driver::DAMIAO_DISABLE_REPEAT_INTERVAL
        * u32::from(armd::driver::DAMIAO_DISABLE_REPEATS - 1);
    assert!(
        waited >= expected_min && waited < std::time::Duration::from_millis(120),
        "expected bounded repeats without 150 ms confirmation wait, waited {waited:?}"
    );
    let r = usize::from(armd::driver::DAMIAO_DISABLE_REPEATS);
    assert_eq!(
        (sent(1), sent(2), sent(3)),
        (r, r, r),
        "every motor receives exactly DAMIAO_DISABLE_REPEATS disable frames"
    );
}
