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

#[test]
fn set_zero_needs_a_confirmed_disabled_motor_and_sends_the_fe_frame() {
    let ids = [1u16, 2, 3];
    let mut d = DamiaoArmDriver::new(MockCanTransport::new(), ids.to_vec());
    assert!(d.motor_zero_support().supported);
    assert_eq!(d.motor_zero_support().persist.as_deref(), Some("motor_flash"));
    assert!(!d.motor_zero_support().persist_verified);
    // no feedback yet: not confirmed disabled
    assert!(d.set_zero_position(1).is_err());
    // disable: every motor answers state 0
    for &id in &ids { d.can_mut().respond_to_transmit(damiao::build_disable_frame(id), feedback_frame(id as u8, 0, 0.4)); }
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
    for &id in &ids { d.can_mut().respond_to_transmit(damiao::build_enable_frame(id), feedback_frame(id as u8, 1, 0.2)); }
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
