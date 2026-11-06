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
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]).with_vlim(2.5);

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
        let expected = 2.5f32.clamp(0.5, 4.0);
        assert!((sent - expected).abs() < 1e-6, "velocity={velocity}, sent={sent}");
    }
}

#[test]
fn queued_feedback_is_consumed_before_recovery_check() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver.can_mut().respond_to_transmit(damiao::build_enable_frame(1), feedback_frame(1, 1, 0.0));
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
    driver.can_mut().respond_to_transmit(damiao::build_enable_frame(1), feedback_frame(1, 1, 0.0));
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
    driver.can_mut().respond_to_transmit(damiao::build_enable_frame(1), feedback_frame(1, 1, 0.0));
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
    let _ = driver.can_mut().take_tx_frames();

    thread::sleep(Duration::from_millis(50));
    let _ = driver.read_joint_states().unwrap();
    let frames = driver.can_mut().take_tx_frames();
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].id, 1, "MIT recovery must not use POS_VEL id 0x101");
    assert_eq!(frames[0].dlc, 8);
    let limits = DamiaoLimits::default();
    let measured_zero = damiao::uint_to_float(
        damiao::float_to_uint(0.0, limits.p_min, limits.p_max, 16), limits.p_min, limits.p_max, 16,
    );
    let expected = damiao::build_mit_frame(1, measured_zero, 0.0, 40.0, 1.0, 0.0, &limits);
    assert_eq!(frames[0].data, expected.data, "recovery holds measured pose, never replays the old moving reference");
}

#[test]
fn stale_posvel_gripper_recovery_uses_posvel_frame() {
    let motor_ids: Vec<u16> = (1..=7).collect();
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), motor_ids);
    for id in 1..=7 {
        driver.can_mut().respond_to_transmit(damiao::build_enable_frame(id as u16), feedback_frame(id, 1, 0.0));
    }
    driver.set_torque(true).unwrap();
    let _ = driver.can_mut().take_tx_frames();

    thread::sleep(Duration::from_millis(50));
    let _ = driver.read_joint_states().unwrap();
    let frames = driver.can_mut().take_tx_frames();
    assert_eq!(frames.len(), 7);
    let gripper = frames.iter().find(|frame| frame.id == 0x107).unwrap();
    assert_eq!(gripper.dlc, 8);
}

#[test]
fn test_damiao_set_torque_fast_path_and_rearm_handshake() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver.can_mut().respond_to_transmit(damiao::build_enable_frame(1), feedback_frame(1, 1, 0.0));

    // 1. Initial arming: pending_rearm is true, must execute full handshake
    driver.set_torque(true).unwrap();
    let tx1 = driver.can_mut().take_tx_frames();
    assert_eq!(tx1.len(), 1);
    assert_eq!(tx1[0].data, CMD_ENABLE);

    // 2. Fast-path: motor enabled, feedback valid, state == 1, pending_rearm is false
    driver.set_torque(true).unwrap();
    let tx2 = driver.can_mut().take_tx_frames();
    assert!(tx2.is_empty(), "Fast path must NOT re-transmit enable frames");

    // 3. Set torque flag to false (e.g. from Supervisor clear_fault Branch B): sets pending_rearm = true
    driver.set_torque_flag(false);
    assert!(!driver.is_enabled());

    // The registered mock response is produced after each enable transmission.

    // 4. Arming again: pending_rearm was set to true, fast-path is bypassed and full handshake executes
    driver.set_torque(true).unwrap();
    let tx3 = driver.can_mut().take_tx_frames();
    assert_eq!(tx3.len(), 1, "Must broadcast enable frame after pending_rearm was set");
    assert_eq!(tx3[0].data, CMD_ENABLE);
}

#[test]
fn test_damiao_write_targets_retries_on_would_block() {
    use armd::transport::TransportError;

    struct FlakyCanTransport {
        would_block_count: usize,
        tx_frames: Vec<CanFrame>,
    }

    impl armd::transport::CanTransport for FlakyCanTransport {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            if self.would_block_count > 0 {
                self.would_block_count -= 1;
                return Err(TransportError::WouldBlock);
            }
            self.tx_frames.push(*frame);
            Ok(())
        }

        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            Err(TransportError::Timeout)
        }

        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
    }

    // A: 2 WouldBlock occurrences: recovered within 3 retries -> succeeds
    let mut driver = DamiaoArmDriver::new(
        FlakyCanTransport {
            would_block_count: 2,
            tx_frames: Vec::new(),
        },
        vec![1],
    );

    let res = driver.write_joint_targets(&[JointTarget::position(0.5)]);
    assert!(res.is_ok(), "Write joint targets must succeed after transient WouldBlock retry");
    assert_eq!(driver.can_mut().tx_frames.len(), 1);

    // B: 4 WouldBlock occurrences: exceeds 3 retries -> returns error
    let mut driver_failing = DamiaoArmDriver::new(
        FlakyCanTransport {
            would_block_count: 4,
            tx_frames: Vec::new(),
        },
        vec![1],
    );

    let res_fail = driver_failing.write_joint_targets(&[JointTarget::position(0.5)]);
    assert!(res_fail.is_err(), "Exceeding max retries must return error");
}

#[test]
fn expired_feedback_does_not_replay_old_motion_on_recovery() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1]);
    driver.can_mut().respond_to_transmit(damiao::build_enable_frame(1), feedback_frame(1, 1, 0.0));
    driver.set_torque(true).unwrap();
    driver.write_joint_targets(&[JointTarget::position(0.8)]).unwrap();
    driver.can_mut().take_tx_frames();
    thread::sleep(Duration::from_millis(260));
    driver.read_joint_states().unwrap();
    assert!(driver.can_mut().take_tx_frames().is_empty(), "unknown feedback must not resend cached target");
    driver.can_mut().provide_rx_frame(feedback_frame(1, 1, 0.1));
    driver.read_joint_states().unwrap();
    assert!(driver.can_mut().take_tx_frames().is_empty(), "first restored sample must not resume motion");
}
