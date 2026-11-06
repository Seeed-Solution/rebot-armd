use std::thread;
use std::time::Duration;

use armd::driver::{
    ArmDriver, DamiaoArmDriver, DamiaoControlMode, DriverCapabilities, EnableState,
    FashionStarArmDriver, FeetechArmDriver, RobStrideArmDriver,
};
use armd::protocol::damiao;
use armd::transport::{CanFrame, CanTransport, MockCanTransport, MockTransport, Transport};

fn feedback(state: u8, mos: u8, rotor: u8) -> CanFrame {
    // Midrange encoded position, velocity and torque; physical temperature bytes.
    CanFrame::new_standard(0x11, &[state << 4 | 1, 0x80, 0, 0x80, 0x08, 0, mos, rotor]).unwrap()
}

#[test]
fn capabilities_follow_active_protocol_and_do_not_claim_estimates_as_measurements() {
    let dm = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]).with_vlim(1.2);
    assert_eq!(
        dm.capabilities(),
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
        }
    );
    assert_eq!(dm.max_command_velocity(0), Some(1.2));
    assert_eq!(dm.max_command_velocity(1), None);
    let mit = DamiaoArmDriver::new(MockCanTransport::new(), vec![1])
        .with_control_mode(DamiaoControlMode::Mit);
    assert!(mit.capabilities().torque_feedforward);
    assert_eq!(mit.max_command_velocity(0), None);

    let rs = RobStrideArmDriver::new(MockCanTransport::new(), vec![1]);
    assert!(rs.capabilities().torque_feedforward);
    assert!(rs.capabilities().torque_feedback);
    assert!(!rs.capabilities().enable_feedback);
    let ft = FeetechArmDriver::new(MockTransport::new(), vec![1]);
    assert!(ft.capabilities().velocity_feedback);
    assert!(ft.capabilities().temperature_feedback);
    assert!(
        !ft.capabilities().torque_feedback,
        "load-based estimate is not measured Nm"
    );
    let fs = FashionStarArmDriver::new(MockTransport::new(), vec![1]);
    assert_eq!(fs.capabilities(), DriverCapabilities::default());
}

#[test]
fn unconfirmed_enable_is_unknown_for_non_damiao_drivers() {
    let mut drivers: Vec<Box<dyn ArmDriver>> = vec![
        Box::new(RobStrideArmDriver::new(MockCanTransport::new(), vec![1])),
        Box::new(FeetechArmDriver::new(MockTransport::new(), vec![1])),
        Box::new(FashionStarArmDriver::new(MockTransport::new(), vec![1])),
    ];
    for driver in &mut drivers {
        assert_eq!(driver.feedback_ages(), vec![None]);
        driver.set_torque(true).unwrap();
        assert_eq!(driver.enable_state(), EnableState::Unknown);
        assert!(!driver.is_enabled());
    }
}

#[test]
fn generic_mocks_do_not_automatically_acknowledge_damiao_enable() {
    let mut can = MockCanTransport::new();
    can.transmit(&damiao::build_enable_frame(1)).unwrap();
    assert!(can.receive().is_err());
    let mut serial = MockTransport::new();
    serial.write_all(b"t0018FFFFFFFFFFFFFFFC\r").unwrap();
    let mut bytes = [0; 64];
    assert_eq!(serial.read(&mut bytes).unwrap(), 0);
}

#[test]
fn damiao_requires_new_enable_feedback_and_exposes_real_sample_age() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver.can_mut().provide_rx_frame(feedback(1, 30, 32));
    assert!(
        driver.set_torque(true).is_err(),
        "queued pre-enable feedback must not confirm enable"
    );
    assert_eq!(driver.enable_state(), EnableState::Unknown);
    driver
        .can_mut()
        .respond_to_transmit(damiao::build_enable_frame(1), feedback(1, 30, 32));
    driver.set_torque(true).unwrap();
    assert_eq!(driver.enable_state(), EnableState::Enabled);
    let before = driver.feedback_ages()[0].unwrap();
    thread::sleep(Duration::from_millis(5));
    driver.read_joint_states().unwrap();
    assert!(
        driver.feedback_ages()[0].unwrap() > before,
        "cache reads must not renew samples"
    );
    thread::sleep(Duration::from_millis(260));
    assert_eq!(driver.enable_state(), EnableState::Unknown);
    assert!(!driver.is_enabled());
}

#[test]
fn damiao_uses_hotter_mos_or_rotor_temperature() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver.can_mut().provide_rx_frame(feedback(1, 40, 75));
    assert_eq!(driver.read_joint_states().unwrap()[0].temperature, 75.0);
    driver.can_mut().provide_rx_frame(feedback(1, 80, 42));
    assert_eq!(driver.read_joint_states().unwrap()[0].temperature, 80.0);
}

#[test]
fn damiao_disable_needs_feedback_and_failure_does_not_fabricate_disabled_status() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver
        .can_mut()
        .respond_to_transmit(damiao::build_enable_frame(1), feedback(1, 30, 32));
    driver.set_torque(true).unwrap();
    driver
        .can_mut()
        .set_error_on_next_tx(Some("bus offline".into()));
    assert!(driver.set_torque(false).is_err());
    assert_eq!(
        driver.motor_status(),
        Some(vec![1]),
        "a failed write must not rewrite observed state"
    );
    assert_eq!(driver.enable_state(), EnableState::Unknown);

    driver.set_torque(false).unwrap();
    assert_eq!(
        driver.enable_state(),
        EnableState::Unknown,
        "successful send alone is not disable confirmation"
    );
    driver.can_mut().provide_rx_frame(feedback(0, 30, 32));
    driver.read_joint_states().unwrap();
    assert_eq!(driver.enable_state(), EnableState::Disabled);
}

#[test]
fn clear_fault_write_does_not_erase_observed_hardware_faults() {
    let mut dm = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    dm.can_mut().provide_rx_frame(feedback(0xB, 75, 40));
    dm.read_joint_states().unwrap();
    assert!(dm.check_hardware_status().is_err());
    dm.clear_fault().unwrap();
    assert!(dm.check_hardware_status().is_err());
    dm.can_mut().provide_rx_frame(feedback(0, 40, 40));
    dm.read_joint_states().unwrap();
    assert!(dm.check_hardware_status().is_ok());

    let mut rs = RobStrideArmDriver::new(MockCanTransport::new(), vec![1]);
    let fault_id = armd::protocol::robstride::build_feedback_extended_id(3, 1, 0xFD);
    let clear_id = armd::protocol::robstride::build_feedback_extended_id(0, 1, 0xFD);
    let data = [128, 0, 128, 0, 128, 0, 0, 250];
    rs.can_mut()
        .provide_rx_frame(CanFrame::new_extended(fault_id, &data).unwrap());
    rs.read_joint_states().unwrap();
    assert!(rs.check_hardware_status().is_err());
    rs.clear_fault().unwrap();
    assert!(rs.check_hardware_status().is_err());
    rs.can_mut()
        .provide_rx_frame(CanFrame::new_extended(clear_id, &data).unwrap());
    rs.read_joint_states().unwrap();
    assert!(rs.check_hardware_status().is_ok());
}

#[test]
fn queued_disabled_feedback_cannot_acknowledge_a_later_disable_command() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver
        .can_mut()
        .respond_to_transmit(damiao::build_enable_frame(1), feedback(1, 30, 32));
    driver.set_torque(true).unwrap();
    driver.can_mut().provide_rx_frame(feedback(0, 30, 32));
    driver.set_torque(false).unwrap();
    assert_eq!(
        driver.enable_state(),
        EnableState::Unknown,
        "pre-command state=0 is not a disable ACK"
    );
    driver.can_mut().provide_rx_frame(feedback(0, 30, 32));
    driver.read_joint_states().unwrap();
    assert_eq!(driver.enable_state(), EnableState::Disabled);
}

#[test]
fn overloaded_rx_still_sends_disable_but_requires_a_new_feedback_boundary() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver
        .can_mut()
        .respond_to_transmit(damiao::build_enable_frame(1), feedback(1, 30, 32));
    driver.set_torque(true).unwrap();
    driver.can_mut().take_tx_frames();
    for _ in 0..205 {
        driver.can_mut().provide_rx_frame(feedback(0, 30, 32));
    }
    driver.set_torque(false).unwrap();
    assert!(driver
        .can_mut()
        .take_tx_frames()
        .iter()
        .any(|f| *f == damiao::build_disable_frame(1)));
    assert_eq!(driver.enable_state(), EnableState::Unknown);
    driver.read_joint_states().unwrap();
    assert_eq!(
        driver.enable_state(),
        EnableState::Unknown,
        "draining old frames cannot confirm disable"
    );
    driver.can_mut().provide_rx_frame(feedback(0, 30, 32));
    driver.read_joint_states().unwrap();
    assert_eq!(driver.enable_state(), EnableState::Disabled);
}

#[test]
fn genuine_response_to_disable_confirms_disabled_state() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver
        .can_mut()
        .respond_to_transmit(damiao::build_enable_frame(1), feedback(1, 30, 32));
    driver
        .can_mut()
        .respond_to_transmit(damiao::build_disable_frame(1), feedback(0, 30, 32));
    driver.set_torque(true).unwrap();
    driver.set_torque(false).unwrap();
    assert_eq!(driver.enable_state(), EnableState::Disabled);
}
