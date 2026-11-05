use std::thread;
use std::time::Duration;

use armd::driver::{ArmDriver, DamiaoArmDriver, DamiaoControlMode, JointTarget};
use armd::protocol::damiao::{self, DamiaoLimits, CMD_ENABLE};
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
fn posvel_allows_micro_motion_and_respects_configured_limit() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]).with_vlim(0.02);

    for velocity in [0.0, 0.005, 0.02] {
        driver
            .write_joint_targets(&[JointTarget {
                position: 0.1,
                velocity: Some(velocity),
                torque: None,
                kp: None,
                kd: None,
            }])
            .unwrap();
        let frame = driver.can_mut().take_tx_frames().pop().unwrap();
        let sent = f32::from_le_bytes(frame.data[4..8].try_into().unwrap());
        let expected = velocity.abs().max(0.01).min(0.02);
        assert!((sent - expected).abs() < 1e-6, "velocity={velocity}, sent={sent}");
    }
}

#[test]
fn queued_feedback_is_consumed_before_recovery_check() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver.can_mut().provide_rx_frame(feedback_frame(1, 1, 0.0));
    driver.set_torque(true).unwrap();
    let enable_frames = driver.can_mut().take_tx_frames();
    assert_eq!(enable_frames.len(), 1);
    assert_eq!(enable_frames[0].data, CMD_ENABLE);

    // Make the previous sample stale, then queue a fresh sample before polling.
    thread::sleep(Duration::from_millis(50));
    driver.can_mut().provide_rx_frame(feedback_frame(1, 1, 0.25));
    let _ = driver.read_joint_states().unwrap();
    assert!(driver.can_mut().take_tx_frames().is_empty());
}

#[test]
fn recovery_defers_when_rx_budget_is_exhausted() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver.can_mut().provide_rx_frame(feedback_frame(1, 1, 0.0));
    driver.set_torque(true).unwrap();
    let _ = driver.can_mut().take_tx_frames();

    thread::sleep(Duration::from_millis(50));
    // First 64 frames are unrelated bus traffic and must not refresh motor 1.
    for _ in 0..64 {
        driver
            .can_mut()
            .provide_rx_frame(CanFrame::new_standard(0x500, &[0u8; 8]).unwrap());
    }
    // The valid motor-1 feedback is deliberately beyond the first poll budget.
    driver.can_mut().provide_rx_frame(feedback_frame(1, 1, 0.3));

    // The bounded receive budget is exhausted before the queue is known empty;
    // recovery must be deferred rather than injecting a stale hold frame.
    let _ = driver.read_joint_states().unwrap();
    assert!(driver.can_mut().take_tx_frames().is_empty());

    // The next poll consumes the remaining feedback, observes an explicit
    // timeout, and still must not recover because that feedback is fresh.
    let _ = driver.read_joint_states().unwrap();
    assert!(driver.can_mut().take_tx_frames().is_empty());
}

#[test]
fn stale_mit_recovery_uses_mit_frame_id() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1])
        .with_control_mode(DamiaoControlMode::Mit);
    driver.can_mut().provide_rx_frame(feedback_frame(1, 1, 0.0));
    driver.set_torque(true).unwrap();
    let _ = driver.can_mut().take_tx_frames();
    driver
        .write_joint_targets(&[JointTarget {
            position: 0.4,
            velocity: Some(0.0),
            torque: Some(0.7),
            kp: Some(12.0),
            kd: Some(0.8),
        }])
        .unwrap();
    let expected = driver.can_mut().take_tx_frames().pop().unwrap();

    thread::sleep(Duration::from_millis(50));
    let _ = driver.read_joint_states().unwrap();
    let frames = driver.can_mut().take_tx_frames();
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].id, 1, "MIT recovery must not use POS_VEL id 0x101");
    assert_eq!(frames[0].dlc, 8);
    assert_eq!(frames[0].data, expected.data, "recovery should hold the last MIT reference");
}

#[test]
fn stale_posvel_gripper_recovery_uses_posforce_frame() {
    let motor_ids: Vec<u16> = (1..=7).collect();
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), motor_ids);
    for id in 1..=7 {
        driver.can_mut().provide_rx_frame(feedback_frame(id, 1, 0.0));
    }
    driver.set_torque(true).unwrap();
    let _ = driver.can_mut().take_tx_frames();

    thread::sleep(Duration::from_millis(50));
    let _ = driver.read_joint_states().unwrap();
    let frames = driver.can_mut().take_tx_frames();
    assert_eq!(frames.len(), 7);
    let gripper = frames.iter().find(|frame| frame.id == 0x307).unwrap();
    assert_eq!(gripper.dlc, 8);
}
