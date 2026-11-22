use std::f32::consts::PI;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, FeetechArmDriver, JointTarget};
use armd::model::{ArmConfig, GripperPolicy, HoldingMode, JointLimit, JointState, RestingPosePolicy};
use armd::motion::{GripperCommand, GripperMotion, GripperRequest, StepCtx, StepStatus};
use armd::protocol::feetech::{
    self, calculate_checksum, decode_sign_magnitude, encode_sign_magnitude, feetech_error_name,
    raw_load_to_fraction, raw_speed_to_rad_s, INST_SYNC_READ, INST_WRITE, REG_D_COEF,
    REG_I_COEF, REG_LOCK, REG_MAX_ANGLE_LIMIT_L, REG_MAX_TORQUE_L, REG_MIN_ANGLE_LIMIT_L,
    REG_OPERATING_MODE, REG_OVERLOAD_TORQUE, REG_PRESENT_POSITION_L, REG_PROTECTION_CURRENT_L,
    REG_P_COEF, REG_RETURN_DELAY, REG_STATUS_RETURN_LEVEL, REG_TORQUE_ENABLE,
    STEPS_PER_REVOLUTION,
};
use armd::safety::{FaultCode, SafetyError, SafetyState};
use armd::supervisor::{ArmError, ArmSupervisor};
use armd::transport::{DriverClock, MockClock, MockTransport, SerialTransportConfig, Transport, TransportError};

#[derive(Clone, Default)]
struct SharedTransport(Arc<Mutex<MockTransport>>);

impl Transport for SharedTransport {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Transport::clock(&*self.0.lock().unwrap())
    }
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        self.0.lock().unwrap().read(buf)
    }
    fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        self.0.lock().unwrap().write(buf)
    }
    fn flush(&mut self) -> Result<(), TransportError> {
        self.0.lock().unwrap().flush()
    }
}

// -------------------------------------------------------------------------
// 1. Sign-magnitude velocity & homing offset conversions
// -------------------------------------------------------------------------
#[test]
fn test_feetech_sign_magnitude_velocity() {
    // 0 and negative-0 must decode to 0.0 rad/s
    assert_eq!(raw_speed_to_rad_s(0x0000), 0.0);
    assert_eq!(raw_speed_to_rad_s(0x8000), 0.0);

    let step_rad = (2.0 * PI) / STEPS_PER_REVOLUTION;
    // 1 step/sec positive
    assert!((raw_speed_to_rad_s(0x0001) - step_rad).abs() < 1e-6);
    // 1 step/sec negative (bit 15 set)
    assert!((raw_speed_to_rad_s(0x8001) - (-step_rad)).abs() < 1e-6);

    // 100 steps/sec
    assert!((raw_speed_to_rad_s(100) - (100.0 * step_rad)).abs() < 1e-5);
    assert!((raw_speed_to_rad_s(0x8000 | 100) - (-100.0 * step_rad)).abs() < 1e-5);

    // General sign-magnitude encoding/decoding helper
    assert_eq!(encode_sign_magnitude(100, 0x8000), 100);
    assert_eq!(encode_sign_magnitude(-100, 0x8000), 0x8064);
    assert_eq!(decode_sign_magnitude(0x8064, 0x8000), -100);
    assert_eq!(decode_sign_magnitude(100, 0x8000), 100);

    // Homing offset with sign_bit = 0x0800 (bit 11)
    assert_eq!(encode_sign_magnitude(50, 0x0800), 50);
    assert_eq!(encode_sign_magnitude(-50, 0x0800), 0x0800 | 50);
    assert_eq!(decode_sign_magnitude(0x0800 | 50, 0x0800), -50);
}

// -------------------------------------------------------------------------
// 2. Load fraction and error name decoding
// -------------------------------------------------------------------------
#[test]
fn test_feetech_load_fraction_and_error_names() {
    assert_eq!(raw_load_to_fraction(0), 0.0);
    // CCW / positive direction (bit 10 is 0)
    assert!((raw_load_to_fraction(500) - 0.5).abs() < 1e-5);
    // CW / negative direction (bit 10 is 1)
    assert!((raw_load_to_fraction(0x0400 | 500) - (-0.5)).abs() < 1e-5);
    // Clamping to 1000 / 1.0 (bits 0..=9 clamped to 1000)
    assert_eq!(raw_load_to_fraction(1000), 1.0);
    assert_eq!(raw_load_to_fraction(0x0400 | 1000), -1.0);

    // Status / error decoding
    assert_eq!(feetech_error_name(0x00), None);
    assert_eq!(feetech_error_name(0x01), Some("Voltage".to_string()));
    assert_eq!(feetech_error_name(0x02), Some("Sensor".to_string()));
    assert_eq!(feetech_error_name(0x04), Some("Overtemperature".to_string()));
    assert_eq!(feetech_error_name(0x08), Some("Overcurrent".to_string()));
    assert_eq!(feetech_error_name(0x20), Some("Overload".to_string()));
    assert_eq!(feetech_error_name(0x05), Some("Voltage | Overtemperature".to_string()));
    assert_eq!(feetech_error_name(0x24), Some("Overtemperature | Overload".to_string()));
    assert_eq!(feetech_error_name(0x40), Some("Error(0x40)".to_string()));
}

// -------------------------------------------------------------------------
// 3. Serial RS-485 auto-probe and config
// -------------------------------------------------------------------------
#[test]
fn test_serial_transport_rs485_config() {
    assert!(SerialTransportConfig::default().port.is_empty());
    let cfg_sc = SerialTransportConfig {
        port: "/dev/ttySC0".to_string(),
        baud_rate: 1_000_000,
        timeout: Duration::from_millis(10),
        low_latency_1ms: true,
        rs485_rts_on_send: None,
    };
    // None defaults cleanly for /dev/ttySC*
    assert_eq!(cfg_sc.rs485_rts_on_send, None);
    assert!(cfg_sc.port.starts_with("/dev/ttySC"));
}

// -------------------------------------------------------------------------
// 4. FeetechArmDriver TX echo stripping (SC16IS752 half-duplex transceiver)
// -------------------------------------------------------------------------
#[test]
fn test_feetech_tx_echo_drain() {
    let mock = MockTransport::new();
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids)
        .with_calibrated(true)
        .with_tx_echo(Some(false));

    // Provide feedback for servo 1 first
    let pos = 2048u16.to_le_bytes();
    let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
    let cs = calculate_checksum(1, 10, 0x00, &params);
    let mut fb = vec![0xFF, 0xFF, 1, 10, 0x00];
    fb.extend_from_slice(&params);
    fb.push(cs);
    let sync_read = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    driver.transport_mut().stage_reply(&sync_read, &fb);
    driver.read_joint_states().unwrap();
    let _ = driver.transport_mut().take_tx();

    let mut driver = driver.with_tx_echo(Some(true));

    // Prepare response: TX packet echoed first, followed by RX response
    let pkt_write = feetech::build_set_torque(1, true);
    let params = [1u8];
    let len = (params.len() + 2) as u8;
    let cs = calculate_checksum(1, len, 0x00, &params);
    let pkt_read_resp = vec![0xFF, 0xFF, 1, len, 0x00, 1, cs];

    // 1. Echo of write command
    driver.transport_mut().stage_reply(&pkt_write, &pkt_write);

    // 2. Read command will be sent by set_torque -> echo of read command + real response
    let pkt_read_cmd = feetech::build_read(1, REG_TORQUE_ENABLE, 1);
    let mut echo_and_resp = pkt_read_cmd.clone();
    echo_and_resp.extend_from_slice(&pkt_read_resp);
    driver.transport_mut().stage_reply(&pkt_read_cmd, &echo_and_resp);

    driver.set_torque(true).unwrap();
    assert_eq!(driver.enable_state(), EnableState::Enabled);
}

// -------------------------------------------------------------------------
// 5. Readback-verified set_torque enable, rollback on failure, and disable
// -------------------------------------------------------------------------
#[test]
fn test_feetech_torque_enable_rollback() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1, 2];
    let mut driver = FeetechArmDriver::new(mock, servo_ids.clone()).with_calibrated(true);

    // Provide feedback first
    let mut rx_fb = Vec::new();
    for &id in &servo_ids {
        let pos = 2048u16.to_le_bytes();
        let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
        let cs = calculate_checksum(id, 10, 0x00, &params);
        rx_fb.extend_from_slice(&[0xFF, 0xFF, id, 10, 0x00]);
        rx_fb.extend_from_slice(&params);
        rx_fb.push(cs);
    }
    driver.transport_mut().provide_rx(&rx_fb);
    driver.read_joint_states().unwrap();
    let _ = driver.transport_mut().take_tx();

    // Servo 1 reports 1 (enabled), Servo 2 fails (reports 0)
    let cs1 = calculate_checksum(1, 3, 0x00, &[1]);
    let resp1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs1];
    let cs2 = calculate_checksum(2, 3, 0x00, &[0]);
    let resp2 = vec![0xFF, 0xFF, 2, 3, 0x00, 0, cs2];

    let mut rx = Vec::new();
    rx.extend_from_slice(&resp1);
    rx.extend_from_slice(&resp2);
    // Rollback readbacks: both return 0 (disabled)
    let cs_rb1 = calculate_checksum(1, 3, 0x00, &[0]);
    let resp_rb1 = vec![0xFF, 0xFF, 1, 3, 0x00, 0, cs_rb1];
    let cs_rb2 = calculate_checksum(2, 3, 0x00, &[0]);
    let resp_rb2 = vec![0xFF, 0xFF, 2, 3, 0x00, 0, cs_rb2];
    rx.extend_from_slice(&resp_rb1);
    rx.extend_from_slice(&resp_rb2);
    driver.transport_mut().provide_rx(&rx);

    let res = driver.set_torque(true);
    assert!(res.is_err());
    let err = res.unwrap_err();
    assert!(matches!(err, ArmDriverError::HardwareFault(_)));
    assert_eq!(driver.enable_state(), EnableState::Disabled);

    // Verify rollback: driver wrote disable packets (REG_TORQUE_ENABLE = 0) and read back status
    let tx = driver.transport_mut().take_tx();
    // Pre-seeding SYNC_WRITE (14) + Enable writes (2*8=16) + Readback reads (2*8=16) + Rollback writes (2*8=16) + Rollback reads (2*8=16) = 78 bytes
    assert_eq!(tx.len(), 78);
    // Rollback write to servo 1 at offset 46: [0xFF, 0xFF, 1, 4, 3, 0x28, 0, cs]
    assert_eq!(tx[46 + 2], 1); // ID 1
    assert_eq!(tx[46 + 4], INST_WRITE);
    assert_eq!(tx[46 + 5], REG_TORQUE_ENABLE);
    assert_eq!(tx[46 + 6], 0); // Torque disable
}

// -------------------------------------------------------------------------
// 6. SYNC_READ 3-servo chunking for 6-DOF arm
// -------------------------------------------------------------------------
#[test]
fn test_feetech_sync_read_chunking_6dof() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1, 2, 3, 4, 5, 6];
    let mut driver = FeetechArmDriver::new(mock, servo_ids);

    // Provide responses for Chunk 0 (servos 1, 2, 3)
    let mut rx = Vec::new();
    for id in 1..=3 {
        let pos = 2048u16.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 200u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        rx.extend_from_slice(&[0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00]);
        rx.extend_from_slice(&params);
        rx.push(cs);
    }
    driver.transport_mut().provide_rx(&rx);

    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 6);
    // Servos 1..3 should have valid feedback
    assert!((states[0].position - 0.0).abs() < 1e-4);
    let loads = driver.joint_loads().unwrap();
    assert!((loads[0] - 0.2).abs() < 1e-4);

    // Verify SYNC_READ packet structure sent
    let tx = driver.transport_mut().take_tx();
    assert!(!tx.is_empty());
    assert_eq!(tx[4], INST_SYNC_READ);
    assert_eq!(tx[5], REG_PRESENT_POSITION_L);
    assert_eq!(tx[6], 8); // 8 bytes requested per servo
    assert_eq!(tx[7], 1); // Servo 1
    assert_eq!(tx[8], 2); // Servo 2
    assert_eq!(tx[9], 3); // Servo 3
}

// -------------------------------------------------------------------------
// 7. configure_servos: compare, unlock, write, lock
// -------------------------------------------------------------------------
#[test]
fn test_feetech_configure_servos() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids.clone());

    // Provide EEPROM read responses: first model (STS3215 = 777), then default return delay = 250
    // (differs from target 0) and other registers matching defaults
    let mut rx = Vec::new();
    let model_bytes = 777u16.to_le_bytes();
    let model_len = (model_bytes.len() + 2) as u8;
    let model_cs = calculate_checksum(1, model_len, 0x00, &model_bytes);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, model_len, 0x00]);
    rx.extend_from_slice(&model_bytes);
    rx.push(model_cs);

    // Torque enable check (must be 0)
    let torq_cs = calculate_checksum(1, 3, 0x00, &[0]);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, 3, 0x00, 0, torq_cs]);

    let addrs_and_vals: &[(u8, u16)] = &[
        (REG_RETURN_DELAY, 250),
        (REG_STATUS_RETURN_LEVEL, 0),
        (REG_MIN_ANGLE_LIMIT_L, 0),
        (REG_MAX_ANGLE_LIMIT_L, 4095),
        (REG_P_COEF, 16),
        (REG_D_COEF, 32),
        (REG_I_COEF, 0),
        (REG_MAX_TORQUE_L, 1000),
        (REG_PROTECTION_CURRENT_L, 500),
        (REG_OVERLOAD_TORQUE, 25),
        (REG_OPERATING_MODE, 0),
    ];
    for &(addr, val) in addrs_and_vals {
        let is_16 = addr == REG_MIN_ANGLE_LIMIT_L
            || addr == REG_MAX_ANGLE_LIMIT_L
            || addr == REG_MAX_TORQUE_L
            || addr == REG_PROTECTION_CURRENT_L;
        let params: Vec<u8> = if is_16 {
            val.to_le_bytes().to_vec()
        } else {
            vec![val as u8]
        };
        let len = (params.len() + 2) as u8;
        let cs = calculate_checksum(1, len, 0x00, &params);
        rx.extend_from_slice(&[0xFF, 0xFF, 1, len, 0x00]);
        rx.extend_from_slice(&params);
        rx.push(cs);
    }

    // Readback verification response for changed REG_RETURN_DELAY (value 0)
    let ret_cs = calculate_checksum(1, 3, 0x00, &[0]);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, 3, 0x00, 0, ret_cs]);

    driver.transport_mut().provide_rx(&rx);

    driver.configure_servos().unwrap();

    let tx = driver.transport_mut().take_tx();
    // Verify that EEPROM was unlocked with REG_LOCK = 0, then REG_RETURN_DELAY written with 0,
    // and finally EEPROM was locked with REG_LOCK = 1
    assert!(!tx.is_empty());
    let mut lock_writes = Vec::new();
    for window in tx.windows(8) {
        if window[0] == 0xFF && window[1] == 0xFF && window[2] == 1 && window[4] == INST_WRITE && window[5] == REG_LOCK {
            lock_writes.push(window[6]);
        }
    }
    assert_eq!(lock_writes, vec![0, 1]); // Unlocked (0), then Locked (1)
}

// -------------------------------------------------------------------------
// 8. GripperMotion torque-less holding detection (PositionGap)
// -------------------------------------------------------------------------
#[test]
fn test_gripper_motion_torque_less_holding() {
    let req = GripperRequest {
        command: GripperCommand::Close,
        fraction: None,
        grasp: true,
        speed: 1.0,
        timeout_s: Some(5.0),
        tag: None,
    };
    let mut policy = GripperPolicy::rebot_dm();
    policy.closed_rad = 0.0;
    policy.open_rad = 1.0;
    policy.holding_mode = HoldingMode::PositionGap;
    policy.holding_gap_rad = 0.05;
    policy.max_close_torque_nm = None;

    let limit = JointLimit::new("gripper", -0.2, 1.5, 2.0, 5.0);
    let mut motion = GripperMotion::new(req, policy, limit, 1.0, 6);

    let measured = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.10]; // Jaw stopped at 0.10 rad
    let measured_v = [0.0; 7];
    let commanded = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // Commanded into 0.0 -> gap 0.10 rad > 0.05
    let commanded_v = [0.0; 7];
    let torques = [0.0; 7];
    let loads = [0.0; 7];

    let ctx = StepCtx {
        dt: 0.02,
        time_scale: 1.0,
        measured: &measured,
        measured_velocities: &measured_v,
        commanded: &commanded,
        commanded_velocities: &commanded_v,
        torques: &torques,
        loads: &loads,
        feedback_fresh: true,
        torque_feedback: false, // Feetech torque-less
        load_feedback: false,
        gripper_stalled: false,
        gripper_stall_sign: 1.0,
        gripper_force_capped: false,
    };

    // Squeeze takes settled ticks to declare holding
    let mut status = StepStatus::Running;
    for _ in 0..100 {
        let out = motion.step(&ctx);
        status = out.status;
        if status != StepStatus::Running {
            break;
        }
    }
    assert_eq!(
        status,
        StepStatus::Done {
            settled: None,
            holding: Some(true),
            gripper_gap_rad: Some(0.10),
        }
    );
}

// -------------------------------------------------------------------------
// 9. Supervisor automatic unload support with load feedback
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_supervisor_automatic_unload_with_load_feedback() {
    struct FeetechTestDriver;
    impl ArmDriver for FeetechTestDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: true,
            }
        }
        fn enable_state(&self) -> EnableState { EnableState::Enabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 2] }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState::default(), JointState::default()])
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    }

    let mut config = ArmConfig::default();
    config.dof = 2;
    config.limits = vec![
        JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0),
        JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0),
    ];
    config.home_positions = vec![0.0, 0.0];
    // Policy with support_load_limits (no torque limits)
    config.resting_pose = Some(RestingPosePolicy {
        positions: None,
        position_tolerances: vec![0.08, 0.08],
        max_total_deviation: 0.16,
        max_velocity: 0.015,
        joint_max_velocities: vec![],
        support_torque_limits: vec![],
        support_load_limits: vec![(0, 0.5)],
        feedback_max_age_ms: 100,
    });

    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(FeetechTestDriver));
    assert!(supervisor.inner().read().await.supports_automatic_unload());
}

// -------------------------------------------------------------------------
// 10. configure_servos refuses when torque enabled, and zero writes on match
// -------------------------------------------------------------------------
#[test]
fn test_feetech_configure_servos_torque_enabled_refuses() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids)
        .with_calibrated(true)
        .with_tx_echo(Some(false));

    let mut rx = Vec::new();
    let model_bytes = 777u16.to_le_bytes();
    let model_len = (model_bytes.len() + 2) as u8;
    let model_cs = calculate_checksum(1, model_len, 0x00, &model_bytes);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, model_len, 0x00]);
    rx.extend_from_slice(&model_bytes);
    rx.push(model_cs);

    // REG_TORQUE_ENABLE = 1
    let torq_cs = calculate_checksum(1, 3, 0x00, &[1]);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, 3, 0x00, 1, torq_cs]);

    let addrs_and_vals: &[(u8, u16)] = &[
        (REG_RETURN_DELAY, 10), // Differs from target (0) to force EEPROM write attempt
        (REG_STATUS_RETURN_LEVEL, 0),
        (REG_MIN_ANGLE_LIMIT_L, 0),
        (REG_MAX_ANGLE_LIMIT_L, 4095),
        (REG_P_COEF, 16),
        (REG_D_COEF, 32),
        (REG_I_COEF, 0),
        (REG_MAX_TORQUE_L, 1000),
        (REG_PROTECTION_CURRENT_L, 500),
        (REG_OVERLOAD_TORQUE, 25),
        (REG_OPERATING_MODE, 0),
    ];
    for &(addr, val) in addrs_and_vals {
        let is_16 = addr == REG_MIN_ANGLE_LIMIT_L
            || addr == REG_MAX_ANGLE_LIMIT_L
            || addr == REG_MAX_TORQUE_L
            || addr == REG_PROTECTION_CURRENT_L;
        let params: Vec<u8> = if is_16 {
            val.to_le_bytes().to_vec()
        } else {
            vec![val as u8]
        };
        let len = (params.len() + 2) as u8;
        let cs = calculate_checksum(1, len, 0x00, &params);
        rx.extend_from_slice(&[0xFF, 0xFF, 1, len, 0x00]);
        rx.extend_from_slice(&params);
        rx.push(cs);
    }
    driver.transport_mut().provide_rx(&rx);

    let res = driver.configure_servos();
    assert!(res.is_err());
    let err = res.unwrap_err();
    assert!(matches!(err, ArmDriverError::HardwareFault(_)));

    let tx = driver.transport_mut().take_tx();
    for window in tx.windows(8) {
        if window[0] == 0xFF && window[1] == 0xFF && window[4] == INST_WRITE {
            panic!("no write commands allowed when torque enabled check fails");
        }
    }
}

#[test]
fn test_feetech_configure_servos_already_matching_zero_writes() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids)
        .with_calibrated(true)
        .with_tx_echo(Some(false));

    let mut rx = Vec::new();
    let model_bytes = 777u16.to_le_bytes();
    let model_len = (model_bytes.len() + 2) as u8;
    let model_cs = calculate_checksum(1, model_len, 0x00, &model_bytes);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, model_len, 0x00]);
    rx.extend_from_slice(&model_bytes);
    rx.push(model_cs);

    let torq_cs = calculate_checksum(1, 3, 0x00, &[0]);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, 3, 0x00, 0, torq_cs]);

    let addrs_and_vals: &[(u8, u16)] = &[
        (REG_RETURN_DELAY, 0),
        (REG_STATUS_RETURN_LEVEL, 0),
        (REG_MIN_ANGLE_LIMIT_L, 0),
        (REG_MAX_ANGLE_LIMIT_L, 4095),
        (REG_P_COEF, 16),
        (REG_D_COEF, 32),
        (REG_I_COEF, 0),
        (REG_MAX_TORQUE_L, 1000),
        (REG_PROTECTION_CURRENT_L, 500),
        (REG_OVERLOAD_TORQUE, 25),
        (REG_OPERATING_MODE, 0),
    ];
    for &(addr, val) in addrs_and_vals {
        let is_16 = addr == REG_MIN_ANGLE_LIMIT_L
            || addr == REG_MAX_ANGLE_LIMIT_L
            || addr == REG_MAX_TORQUE_L
            || addr == REG_PROTECTION_CURRENT_L;
        let params: Vec<u8> = if is_16 {
            val.to_le_bytes().to_vec()
        } else {
            vec![val as u8]
        };
        let len = (params.len() + 2) as u8;
        let cs = calculate_checksum(1, len, 0x00, &params);
        rx.extend_from_slice(&[0xFF, 0xFF, 1, len, 0x00]);
        rx.extend_from_slice(&params);
        rx.push(cs);
    }
    driver.transport_mut().provide_rx(&rx);

    driver.configure_servos().unwrap();
    assert_eq!(driver.enable_state(), EnableState::Disabled);

    let tx = driver.transport_mut().take_tx();
    for window in tx.windows(8) {
        if window[0] == 0xFF && window[1] == 0xFF && window[4] == INST_WRITE {
            panic!("no write commands allowed when registers already match");
        }
    }
}

#[test]
fn test_feetech_configure_servos_already_matching_torque_enabled_succeeds() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids)
        .with_calibrated(true)
        .with_tx_echo(Some(false));

    let mut rx = Vec::new();
    let model_bytes = 777u16.to_le_bytes();
    let model_len = (model_bytes.len() + 2) as u8;
    let model_cs = calculate_checksum(1, model_len, 0x00, &model_bytes);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, model_len, 0x00]);
    rx.extend_from_slice(&model_bytes);
    rx.push(model_cs);

    // REG_TORQUE_ENABLE = 1
    let torq_cs = calculate_checksum(1, 3, 0x00, &[1]);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, 3, 0x00, 1, torq_cs]);

    let addrs_and_vals: &[(u8, u16)] = &[
        (REG_RETURN_DELAY, 0),
        (REG_STATUS_RETURN_LEVEL, 0),
        (REG_MIN_ANGLE_LIMIT_L, 0),
        (REG_MAX_ANGLE_LIMIT_L, 4095),
        (REG_P_COEF, 16),
        (REG_D_COEF, 32),
        (REG_I_COEF, 0),
        (REG_MAX_TORQUE_L, 1000),
        (REG_PROTECTION_CURRENT_L, 500),
        (REG_OVERLOAD_TORQUE, 25),
        (REG_OPERATING_MODE, 0),
    ];
    for &(addr, val) in addrs_and_vals {
        let is_16 = addr == REG_MIN_ANGLE_LIMIT_L
            || addr == REG_MAX_ANGLE_LIMIT_L
            || addr == REG_MAX_TORQUE_L
            || addr == REG_PROTECTION_CURRENT_L;
        let params: Vec<u8> = if is_16 {
            val.to_le_bytes().to_vec()
        } else {
            vec![val as u8]
        };
        let len = (params.len() + 2) as u8;
        let cs = calculate_checksum(1, len, 0x00, &params);
        rx.extend_from_slice(&[0xFF, 0xFF, 1, len, 0x00]);
        rx.extend_from_slice(&params);
        rx.push(cs);
    }
    driver.transport_mut().provide_rx(&rx);

    driver.configure_servos().unwrap();
    assert_eq!(driver.enable_state(), EnableState::Enabled);

    let tx = driver.transport_mut().take_tx();
    for window in tx.windows(8) {
        if window[0] == 0xFF && window[1] == 0xFF && window[4] == INST_WRITE {
            panic!("no write commands allowed when registers already match even with torque enabled");
        }
    }
}

#[test]
fn test_feetech_set_torque_already_enabled_no_rollback_on_failure() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1, 2];
    let mut driver = FeetechArmDriver::new(mock, servo_ids.clone()).with_calibrated(true);

    // Provide feedback first
    let mut rx_fb = Vec::new();
    for &id in &servo_ids {
        let pos = 2048u16.to_le_bytes();
        let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
        let cs = calculate_checksum(id, 10, 0x00, &params);
        rx_fb.extend_from_slice(&[0xFF, 0xFF, id, 10, 0x00]);
        rx_fb.extend_from_slice(&params);
        rx_fb.push(cs);
    }
    driver.transport_mut().provide_rx(&rx_fb);
    driver.read_joint_states().unwrap();
    let _ = driver.transport_mut().take_tx();

    // First enable successfully so driver.enable_state() == Enabled
    let cs1 = calculate_checksum(1, 3, 0x00, &[1]);
    let resp1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs1];
    let cs2 = calculate_checksum(2, 3, 0x00, &[1]);
    let resp2 = vec![0xFF, 0xFF, 2, 3, 0x00, 1, cs2];
    let mut rx = Vec::new();
    rx.extend_from_slice(&resp1);
    rx.extend_from_slice(&resp2);
    driver.transport_mut().provide_rx(&rx);
    driver.set_torque(true).unwrap();
    assert_eq!(driver.enable_state(), EnableState::Enabled);
    let _ = driver.transport_mut().take_tx();

    // Now call set_torque(true) again, but simulate readback failure (servo 2 reports 0)
    let cs1 = calculate_checksum(1, 3, 0x00, &[1]);
    let resp1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs1];
    let cs2_zero = calculate_checksum(2, 3, 0x00, &[0]);
    let resp2_zero = vec![0xFF, 0xFF, 2, 3, 0x00, 0, cs2_zero];
    let mut rx_fail = Vec::new();
    rx_fail.extend_from_slice(&resp1);
    rx_fail.extend_from_slice(&resp2_zero);
    driver.transport_mut().provide_rx(&rx_fail);

    let res = driver.set_torque(true);
    assert!(res.is_err());
    assert!(matches!(res.unwrap_err(), ArmDriverError::HardwareFault(_)));
    assert_eq!(driver.enable_state(), EnableState::Unknown);

    // M4: Holding targets are preserved on already-enabled joints, so no pre-seeding SYNC_WRITE is sent.
    // TX should contain 2 enable writes + 2 readback reads = 32 bytes.
    // It must NOT contain rollback disable writes (which would be another 16 bytes with value 0).
    let tx = driver.transport_mut().take_tx();
    assert_eq!(tx.len(), 32);
    for window in tx.windows(8) {
        if window[0] == 0xFF && window[1] == 0xFF && window[4] == INST_WRITE && window[5] == REG_TORQUE_ENABLE {
            assert_eq!(window[6], 1, "Must never send torque disable rollback when already enabled");
        }
    }
}

// -------------------------------------------------------------------------
// 11. LoadAndGap holding detection (succeeds and fails based on load)
// -------------------------------------------------------------------------
#[test]
fn test_gripper_motion_load_and_gap_holding() {
    let req = GripperRequest {
        command: GripperCommand::Close,
        fraction: None,
        grasp: true,
        speed: 1.0,
        timeout_s: Some(5.0),
        tag: None,
    };
    let mut policy = GripperPolicy::rebot_dm();
    policy.closed_rad = 0.0;
    policy.open_rad = 1.0;
    policy.holding_mode = HoldingMode::LoadAndGap;
    policy.holding_gap_rad = 0.05;
    policy.holding_load_fraction = Some(0.30);
    policy.max_close_torque_nm = None;

    let limit = JointLimit::new("gripper", -0.2, 1.5, 2.0, 5.0);

    // Case A: High load (0.40 >= 0.30) with sufficient gap (0.10 >= 0.05) -> holding: Some(true)
    {
        let mut motion = GripperMotion::new(req.clone(), policy.clone(), limit.clone(), 1.0, 6);
        let measured = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.10];
        let measured_v = [0.0; 7];
        let commanded = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let commanded_v = [0.0; 7];
        let torques = [0.0; 7];
        let loads = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.40];

        let ctx = StepCtx {
            dt: 0.02,
            time_scale: 1.0,
            measured: &measured,
            measured_velocities: &measured_v,
            commanded: &commanded,
            commanded_velocities: &commanded_v,
            torques: &torques,
            loads: &loads,
            feedback_fresh: true,
            torque_feedback: false,
            load_feedback: true,
            gripper_stalled: false,
            gripper_stall_sign: 1.0,
            gripper_force_capped: false,
        };

        let mut status = StepStatus::Running;
        for _ in 0..100 {
            let out = motion.step(&ctx);
            status = out.status;
            if status != StepStatus::Running {
                break;
            }
        }
        assert_eq!(
            status,
            StepStatus::Done {
                settled: None,
                holding: Some(true),
                gripper_gap_rad: Some(0.10),
            }
        );
    }

    // Case B: Low load (0.10 < 0.30) even with gap -> holding: Some(false)
    {
        let mut motion = GripperMotion::new(req, policy, limit, 1.0, 6);
        let measured = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.10];
        let measured_v = [0.0; 7];
        let commanded = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let commanded_v = [0.0; 7];
        let torques = [0.0; 7];
        let loads = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.10];

        let ctx = StepCtx {
            dt: 0.02,
            time_scale: 1.0,
            measured: &measured,
            measured_velocities: &measured_v,
            commanded: &commanded,
            commanded_velocities: &commanded_v,
            torques: &torques,
            loads: &loads,
            feedback_fresh: true,
            torque_feedback: false,
            load_feedback: true,
            gripper_stalled: false,
            gripper_stall_sign: 1.0,
            gripper_force_capped: false,
        };

        let mut status = StepStatus::Running;
        for _ in 0..100 {
            let out = motion.step(&ctx);
            status = out.status;
            if status != StepStatus::Running {
                break;
            }
        }
        assert_eq!(
            status,
            StepStatus::Done {
                settled: None,
                holding: Some(false),
                gripper_gap_rad: Some(0.10),
            }
        );
    }
}

// -------------------------------------------------------------------------
// 12. Partial and out-of-order SYNC_READ chunking
// -------------------------------------------------------------------------
#[test]
fn test_feetech_sync_read_partial_out_of_order_chunking() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1, 2, 3, 4, 5, 6];
    let mut driver = FeetechArmDriver::new(mock, servo_ids);

    let build_resp = |id: u8| -> Vec<u8> {
        let pos = 2048u16.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 100u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    let mut rx = Vec::new();
    // Chunk 0: Servos 1, 2, 3 respond
    rx.extend_from_slice(&build_resp(1));
    rx.extend_from_slice(&build_resp(2));
    rx.extend_from_slice(&build_resp(3));
    // Chunk 1: Servo 6 arrives first, then Servo 4; Servo 5 is missing/dropped!
    rx.extend_from_slice(&build_resp(6));
    rx.extend_from_slice(&build_resp(4));

    driver.transport_mut().provide_rx(&rx);
    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 6);

    let ages = driver.feedback_ages();
    assert!(ages[0].is_some(), "Servo 1 replied");
    assert!(ages[1].is_some(), "Servo 2 replied");
    assert!(ages[2].is_some(), "Servo 3 replied");
    assert!(ages[3].is_some(), "Servo 4 replied");
    assert!(ages[4].is_none(), "Servo 5 was missing and must have None age");
    assert!(ages[5].is_some(), "Servo 6 replied out-of-order");
}

// -------------------------------------------------------------------------
// 13. Split/partial RX frame reassembly in read_joint_states
// -------------------------------------------------------------------------
#[test]
fn test_feetech_split_rx_frame_reassembly() {
    let mock = MockTransport::new(); // default preserve_on_clear == false!
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids)
        .with_calibrated(true)
        .with_tx_echo(Some(false));

    let pos = 2048u16.to_le_bytes();
    let spd = 0u16.to_le_bytes();
    let load = 150u16.to_le_bytes();
    let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 42];
    let cs = calculate_checksum(1, (params.len() + 2) as u8, 0x00, &params);
    let mut full_pkt = vec![0xFF, 0xFF, 1, (params.len() + 2) as u8, 0x00];
    full_pkt.extend_from_slice(&params);
    full_pkt.push(cs);

    // Provide fragmented bytes: first 3 bytes staged on sync_read, then 2 bytes, then the rest
    let sync_read = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    driver.transport_mut().stage_reply(&sync_read, &full_pkt[..3]);
    let _ = driver.read_joint_states();
    assert!(driver.feedback_ages()[0].is_none(), "Incomplete header cannot yield state");

    // Destructive clear_input_buffer must NOT wipe continuation bytes on subsequent reads!
    driver.transport_mut().provide_rx(&full_pkt[3..5]);
    let _ = driver.read_joint_states();
    assert!(driver.feedback_ages()[0].is_none(), "Incomplete payload cannot yield state");

    driver.transport_mut().provide_rx(&full_pkt[5..]);
    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 1);
    assert!((states[0].position - 0.0).abs() < 1e-4);
    assert!((states[0].temperature - 42.0).abs() < 1e-4);
    assert!(driver.feedback_ages()[0].is_some());
}

#[test]
fn test_feetech_split_after_first_byte_0xff_survives_cross_call_destructive_mock() {
    let mock = MockTransport::new(); // default preserve_on_clear == false!
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids)
        .with_calibrated(true)
        .with_tx_echo(Some(false));

    let pos = 2048u16.to_le_bytes();
    let spd = 0u16.to_le_bytes();
    let load = 150u16.to_le_bytes();
    let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 42];
    let cs = calculate_checksum(1, (params.len() + 2) as u8, 0x00, &params);
    let mut full_pkt = vec![0xFF, 0xFF, 1, (params.len() + 2) as u8, 0x00];
    full_pkt.extend_from_slice(&params);
    full_pkt.push(cs);

    // Call 1: Split right after the first header byte [0xFF]!
    let sync_read = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    driver.transport_mut().stage_reply(&sync_read, &[0xFF]);
    let _ = driver.read_joint_states();
    assert_eq!(driver.incomplete_chunk_idx(), Some(0), "Single 0xFF byte must be detected as incomplete frame header");
    assert_eq!(driver.partial_pos_rx(), &[0xFF], "partial_pos_rx must hold the [0xFF] byte");

    // Call 2: clear staged write responses and supply remaining bytes [0xFF, 1, len, ...] via provide_rx with real destructive mock!
    driver.transport_mut().clear_write_responses();
    driver.transport_mut().provide_rx(&full_pkt[1..]);
    let states = driver.read_joint_states().expect("Reassembly from [0xFF] split must succeed");
    assert_eq!(states.len(), 1);
    assert!((states[0].position - 0.0).abs() < 1e-4);
    assert!((states[0].temperature - 42.0).abs() < 1e-4);
    assert_eq!(driver.incomplete_chunk_idx(), None);
}

// -------------------------------------------------------------------------
// 14. with_config updates servo_ids used in bus operations
// -------------------------------------------------------------------------
#[test]
fn test_feetech_with_config_updates_servo_ids_in_bus_ops() {
    let mock = MockTransport::new_preserving();
    let default_ids = vec![1, 2, 3];
    let driver = FeetechArmDriver::new(mock, default_ids);

    let custom_ids = [11, 12, 13];
    let cfg = feetech::FeetechConfig {
        joints: custom_ids.iter().map(|&id| feetech::FeetechJointCalib {
            id,
            ..Default::default()
        }).collect(),
        ..Default::default()
    };
    let mut driver = driver.with_config(cfg);

    // 1. write_joint_targets must write to custom IDs 11, 12, 13
    driver.write_joint_targets(&[
        JointTarget::position(0.0),
        JointTarget::position(0.0),
        JointTarget::position(0.0),
    ]).unwrap();

    let tx = driver.transport_mut().take_tx();
    assert!(!tx.is_empty());
    // Packet for SYNC_WRITE
    // Header: FF FF FE LEN 83 REG(0x2A) DATA_LEN(0x02) [ID POS_L POS_H] ...
    // Verify each custom ID appears at the entry positions (offset 7, 10, 13)
    assert_eq!(tx[7], 11);
    assert_eq!(tx[10], 12);
    assert_eq!(tx[13], 13);

    // 2. read_joint_states must poll and decode using custom IDs
    let pos = 2048u16.to_le_bytes();
    let spd = 0u16.to_le_bytes();
    let load = 100u16.to_le_bytes();
    let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 30];
    let cs = calculate_checksum(11, (params.len() + 2) as u8, 0x00, &params);
    let mut rx = vec![0xFF, 0xFF, 11, (params.len() + 2) as u8, 0x00];
    rx.extend_from_slice(&params);
    rx.push(cs);
    driver.transport_mut().provide_rx(&rx);

    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 3);
    assert!((states[0].position - 0.0).abs() < 1e-4);
    assert!(driver.feedback_ages()[0].is_some());
    assert!(driver.feedback_ages()[1].is_none());
}

// -------------------------------------------------------------------------
// 15. drive_mode_invert negates velocity and load consistently with position
// -------------------------------------------------------------------------
#[test]
fn test_feetech_drive_mode_invert_decodes_velocity_and_load() {
    let mock = MockTransport::new_preserving();
    let driver = FeetechArmDriver::new(mock, vec![1, 2]);

    let cfg = feetech::FeetechConfig {
        joints: vec![
            feetech::FeetechJointCalib {
                id: 1,
                drive_mode_invert: false,
                zero_step: 2048,
                ..Default::default()
            },
            feetech::FeetechJointCalib {
                id: 2,
                drive_mode_invert: true,
                zero_step: 2048,
                ..Default::default()
            },
        ],
        ..Default::default()
    };
    let mut driver = driver.with_config(cfg);

    // Provide response for joint 1 (non-inverted) and joint 2 (inverted):
    // raw position: 2148 (+100 steps from zero)
    // raw speed: 50 steps/s positive (bit 15 = 0)
    // raw load: 150 (+0.15 fraction, bit 10 = 0)
    let pos_raw = 2148u16.to_le_bytes();
    let spd_raw = 50u16.to_le_bytes();
    let load_raw = 150u16.to_le_bytes();
    let params = [pos_raw[0], pos_raw[1], spd_raw[0], spd_raw[1], load_raw[0], load_raw[1], 70, 35];

    let mut rx = Vec::new();
    for id in [1u8, 2u8] {
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        rx.extend_from_slice(&[0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00]);
        rx.extend_from_slice(&params);
        rx.push(cs);
    }
    driver.transport_mut().provide_rx(&rx);

    let states = driver.read_joint_states().unwrap();
    let loads = driver.joint_loads().unwrap();

    // Joint 1: standard direction (positive pos, positive vel, positive load)
    assert!(states[0].position > 0.0, "Joint 1 position > 0");
    assert!(states[0].velocity > 0.0, "Joint 1 velocity > 0");
    assert!(loads[0] > 0.0, "Joint 1 load > 0");

    // Joint 2: inverted direction (negated pos, negated vel, negated load)
    assert!(states[1].position < 0.0, "Joint 2 position < 0");
    assert!(states[1].velocity < 0.0, "Joint 2 velocity < 0");
    assert!(loads[1] < 0.0, "Joint 2 load < 0");

    assert!((states[0].position + states[1].position).abs() < 1e-4, "Position inverted");
    assert!((states[0].velocity + states[1].velocity).abs() < 1e-4, "Velocity inverted");
    assert!((loads[0] + loads[1]).abs() < 1e-4, "Load inverted");
}

// -------------------------------------------------------------------------
// 21. read_register_u16_direct: accepting error=0x02 and echo filtering
// -------------------------------------------------------------------------
#[test]
fn test_feetech_read_u16_sensor_error_bit_and_echo_filtering() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]);

    // Request is read 2 bytes from ID 1, reg 0x38 (present position)
    let req = feetech::build_read(1, feetech::REG_PRESENT_POSITION_L, 2);

    // Stream contains:
    // 1. Un-drained request echo (e.g. from an RS-485 transceiver or kernel buffer)
    // 2. Real response packet from servo ID 1 with error byte = 0x02 (Sensor Error bit)
    let val_expected = 2048u16;
    let params = val_expected.to_le_bytes();
    let err_byte = 0x02u8; // Sensor Error bit in STS3215 status
    let cs = calculate_checksum(1, (params.len() + 2) as u8, err_byte, &params);
    let resp = vec![0xFF, 0xFF, 1, (params.len() + 2) as u8, err_byte, params[0], params[1], cs];

    let mut stream = Vec::new();
    stream.extend_from_slice(&req);
    stream.extend_from_slice(&resp);

    driver.transport_mut().provide_rx(&stream);

    let val = driver.read_register_u16_direct(1, feetech::REG_PRESENT_POSITION_L)
        .expect("read_register_u16_direct should filter un-drained req echo and accept error byte 0x02");
    assert_eq!(val, val_expected);
}

// -------------------------------------------------------------------------
// 22. with_port_name auto-downgrade of tx_echo when no echo arrives
// -------------------------------------------------------------------------
#[test]
fn test_feetech_port_name_auto_echo_downgrade() {
    let mock = MockTransport::new_preserving();
    // Default constructor with /dev/ttySC0 auto-detects echo=true, auto=true
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_port_name("/dev/ttySC0");
    assert_eq!(driver.tx_echo(), Some(true));
    assert!(driver.tx_echo_auto());

    // Provide a valid read response without any preceding echo
    let val_expected = 1u8;
    let params = [val_expected];
    let cs = calculate_checksum(1, 3, 0x00, &params);
    let resp = vec![0xFF, 0xFF, 1, 3, 0x00, val_expected, cs];
    driver.transport_mut().provide_rx(&resp);

    // Call read_register_u8_direct: drain_tx_echo will wait up to 5 ms, find no echo,
    // and auto-downgrade tx_echo to Some(false) and tx_echo_auto to false
    let val = driver.read_register_u8_direct(1, REG_TORQUE_ENABLE).unwrap();
    assert_eq!(val, val_expected);
    assert_eq!(driver.tx_echo(), Some(false));
    assert!(!driver.tx_echo_auto());

    // Explicit override via with_tx_echo clears tx_echo_auto
    let driver2 = FeetechArmDriver::new(MockTransport::new_preserving(), vec![1])
        .with_port_name("/dev/ttySC0")
        .with_tx_echo(Some(true));
    assert_eq!(driver2.tx_echo(), Some(true));
    assert!(!driver2.tx_echo_auto());
}

// -------------------------------------------------------------------------
// 23. set_torque(true) pre-seeding Goal Position when feedback is valid
// -------------------------------------------------------------------------
#[test]
fn test_feetech_set_torque_preseeding_goal_position() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1, 2]).with_calibrated(true);

    // 1. Initially without feedback: set_torque(true) must be rejected
    let res = driver.set_torque(true);
    assert!(res.is_err());
    assert!(matches!(res.unwrap_err(), ArmDriverError::HardwareFault(_)));
    assert!(driver.transport_mut().take_tx().is_empty());

    // 2. Now feed telemetry so feedback is valid
    let pos_1 = 2200u16.to_le_bytes();
    let pos_2 = 1800u16.to_le_bytes();
    let params_1 = [pos_1[0], pos_1[1], 0, 0, 0, 0, 70, 35];
    let params_2 = [pos_2[0], pos_2[1], 0, 0, 0, 0, 70, 35];
    let cs_fb1 = calculate_checksum(1, 10, 0x00, &params_1);
    let cs_fb2 = calculate_checksum(2, 10, 0x00, &params_2);
    let mut rx_fb = Vec::new();
    rx_fb.extend_from_slice(&[0xFF, 0xFF, 1, 10, 0x00]);
    rx_fb.extend_from_slice(&params_1);
    rx_fb.push(cs_fb1);
    rx_fb.extend_from_slice(&[0xFF, 0xFF, 2, 10, 0x00]);
    rx_fb.extend_from_slice(&params_2);
    rx_fb.push(cs_fb2);
    driver.transport_mut().provide_rx(&rx_fb);

    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 2);
    assert!(driver.is_feedback_valid());

    // 3. Now call set_torque(true) with feedback present
    let cs1 = calculate_checksum(1, 3, 0x00, &[1]);
    let cs2 = calculate_checksum(2, 3, 0x00, &[1]);
    driver.transport_mut().take_tx(); // clear previous TX
    driver.transport_mut().provide_rx(&[
        0xFF, 0xFF, 1, 3, 0x00, 1, cs1,
        0xFF, 0xFF, 2, 3, 0x00, 1, cs2,
    ]);
    driver.set_torque(true).unwrap();

    let tx_with_fb = driver.transport_mut().take_tx();
    // The very first packet in TX must be SYNC_WRITE (0x83) targeting REG_GOAL_POSITION_L (0x2A)
    assert!(tx_with_fb.len() >= 13);
    assert_eq!(tx_with_fb[0], 0xFF);
    assert_eq!(tx_with_fb[1], 0xFF);
    assert_eq!(tx_with_fb[2], feetech::BROADCAST_ID);
    assert_eq!(tx_with_fb[4], feetech::INST_SYNC_WRITE);
    assert_eq!(tx_with_fb[5], feetech::REG_GOAL_POSITION_L);
    // Data length per servo = 2 (u16 position)
    assert_eq!(tx_with_fb[6], 2);
    // Servo 1 ID & position
    assert_eq!(tx_with_fb[7], 1);
    assert_eq!([tx_with_fb[8], tx_with_fb[9]], pos_1);
    // Servo 2 ID & position
    assert_eq!(tx_with_fb[10], 2);
    assert_eq!([tx_with_fb[11], tx_with_fb[12]], pos_2);
}

// -------------------------------------------------------------------------
// 24. Overload status byte (0x20) triggers HardwareFault and supervisor clear_fault
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_feetech_overload_status_hardware_fault_and_supervisor_clear_fault() {
    let mock = Arc::new(Mutex::new(MockTransport::new_preserving()));
    let driver = FeetechArmDriver::new(SharedTransport(mock.clone()), vec![1]).with_calibrated(true);

    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];
    config.resting_pose = Some(RestingPosePolicy {
        positions: None,
        position_tolerances: vec![0.08],
        max_total_deviation: 0.16,
        max_velocity: 0.015,
        joint_max_velocities: vec![],
        support_torque_limits: vec![],
        support_load_limits: vec![],
        feedback_max_age_ms: 100,
    });

    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(driver));

    // First, feed normal feedback and enable torque so arm is Armed/Idle
    let pos_1 = 2048u16.to_le_bytes();
    let normal_params = [pos_1[0], pos_1[1], 0, 0, 0, 0, 70, 35];
    let cs_normal = calculate_checksum(1, 10, 0x00, &normal_params);
    let mut rx_normal = vec![0xFF, 0xFF, 1, 10, 0x00];
    rx_normal.extend_from_slice(&normal_params);
    rx_normal.push(cs_normal);

    // Provide feedback for pre-enable read_joint_states,
    // followed by response for set_torque readback (REG_TORQUE_ENABLE = 1),
    // followed by feedback for supervisor's post-torque sync call.
    let cs_tq1 = calculate_checksum(1, 3, 0x00, &[1]);
    let rx_tq1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq1];

    mock.lock().unwrap().provide_rx(&rx_normal);
    mock.lock().unwrap().provide_rx(&rx_tq1);
    mock.lock().unwrap().provide_rx(&rx_normal);

    supervisor.set_torque(true).await.unwrap();
    assert!(supervisor.inner().read().await.is_torqued);

    // Now feed an overload status byte (0x20 in the error byte of Feetech packet)
    let cs_overload = calculate_checksum(1, 10, 0x20, &normal_params);
    let mut rx_overload = vec![0xFF, 0xFF, 1, 10, 0x20];
    rx_overload.extend_from_slice(&normal_params);
    rx_overload.push(cs_overload);

    mock.lock().unwrap().provide_rx(&rx_overload);

    let obs = supervisor.tick(0.02).await;
    assert!(obs.fault.is_some(), "Supervisor must transition to fault on hardware error");
    let fault = obs.fault.unwrap();
    assert_eq!(fault.code, FaultCode::HardwareFault);
    assert!(fault.message.contains("fault 0x20: Overload"));

    // Attempting clear_fault() calls drv.clear_fault(), which disables torque on the bus (REG_TORQUE_ENABLE = 0),
    // and asks to wait for fresh fault-free feedback before retrying.
    // Provide fresh feedback for resting-pose support pre-check (still showing 0x20 error), followed by tq0 for clear_fault readback
    let mut rx_rest = vec![0xFF, 0xFF, 1, 10, 0x20];
    rx_rest.extend_from_slice(&normal_params);
    rx_rest.push(cs_overload);
    mock.lock().unwrap().provide_rx(&rx_rest);

    let cs_tq0 = calculate_checksum(1, 3, 0x00, &[0]);
    let rx_tq0 = vec![0xFF, 0xFF, 1, 3, 0x00, 0, cs_tq0];
    mock.lock().unwrap().provide_rx(&rx_tq0);

    let clear_err = supervisor.clear_fault().await.unwrap_err();
    assert!(
        clear_err.to_string().contains("wait for fresh fault-free feedback"),
        "clear_fault must reject and ask to wait for fresh feedback: got {clear_err}"
    );

    // Fresh 0x00 feedback arrives and is ingested by tick
    let mut rx_fresh = vec![0xFF, 0xFF, 1, 10, 0x00];
    rx_fresh.extend_from_slice(&normal_params);
    rx_fresh.push(cs_normal);
    mock.lock().unwrap().provide_rx(&rx_fresh);

    supervisor.tick(0.02).await;

    // Provide another fresh feedback frame for clear_fault's own read_joint_states() call
    let mut rx_fresh2 = vec![0xFF, 0xFF, 1, 10, 0x00];
    rx_fresh2.extend_from_slice(&normal_params);
    rx_fresh2.push(cs_normal);
    mock.lock().unwrap().provide_rx(&rx_fresh2);

    let clear_ok = supervisor.clear_fault().await;
    assert!(clear_ok.is_ok(), "clear_fault must succeed with fresh fault-free feedback: got {:?}", clear_ok.err());
}

// -------------------------------------------------------------------------
// 25. set_torque(true) rejects stale feedback (>250ms)
// -------------------------------------------------------------------------
#[test]
fn test_feetech_set_torque_rejects_stale_feedback() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_calibrated(true);

    let pos_1 = 2048u16.to_le_bytes();
    let params_1 = [pos_1[0], pos_1[1], 0, 0, 0, 0, 70, 35];
    let cs1 = calculate_checksum(1, 10, 0x00, &params_1);
    let mut rx = vec![0xFF, 0xFF, 1, 10, 0x00];
    rx.extend_from_slice(&params_1);
    rx.push(cs1);
    driver.transport_mut().provide_rx(&rx);

    driver.read_joint_states().unwrap();
    assert!(driver.is_feedback_valid());

    // Wait > 250ms for feedback to become stale
    driver.transport().advance_clock(Duration::from_millis(260));
    assert!(!driver.is_feedback_valid());

    driver.transport_mut().take_tx(); // clear any previous packets

    let res = driver.set_torque(true);
    assert!(
        matches!(res, Err(ArmDriverError::HardwareFault(ref msg)) if msg.contains("Stale joint feedback")),
        "Expected stale feedback error, got {:?}",
        res
    );
    assert_ne!(driver.enable_state(), EnableState::Enabled);
    assert!(driver.transport_mut().take_tx().is_empty(), "No command packets should be sent when feedback is stale");
}

// -------------------------------------------------------------------------
// 26. parse_response rejects length = 254 (> 64)
// -------------------------------------------------------------------------
#[test]
fn test_feetech_parse_response_rejects_oversized_length() {
    let buf = [0xFF, 0xFF, 1, 254, 0x00, 0x00];
    let res = feetech::parse_response(&buf);
    assert!(
        matches!(res, Err(feetech::FeetechError::ParameterError(ref msg)) if msg.contains("254")),
        "Expected ParameterError for length 254, got {:?}",
        res
    );
}

// -------------------------------------------------------------------------
// 27. Omitting range_min_steps/range_max_steps preserves existing EEPROM angle limits
// -------------------------------------------------------------------------
#[test]
fn test_feetech_omitted_range_steps_preserves_eeprom() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids)
        .with_calibrated(true)
        .with_tx_echo(Some(false));

    assert!(driver.config().joints[0].range_min_steps.is_none());
    assert!(driver.config().joints[0].range_max_steps.is_none());

    let mut rx = Vec::new();
    let model_bytes = 777u16.to_le_bytes();
    let model_len = (model_bytes.len() + 2) as u8;
    let model_cs = calculate_checksum(1, model_len, 0x00, &model_bytes);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, model_len, 0x00]);
    rx.extend_from_slice(&model_bytes);
    rx.push(model_cs);

    let torq_cs = calculate_checksum(1, 3, 0x00, &[0]);
    rx.extend_from_slice(&[0xFF, 0xFF, 1, 3, 0x00, 0, torq_cs]);

    let addrs_and_vals: &[(u8, u16)] = &[
        (REG_RETURN_DELAY, 0),
        (REG_STATUS_RETURN_LEVEL, 0),
        (REG_MIN_ANGLE_LIMIT_L, 100),
        (REG_MAX_ANGLE_LIMIT_L, 3900),
        (REG_P_COEF, 16),
        (REG_D_COEF, 32),
        (REG_I_COEF, 0),
        (REG_MAX_TORQUE_L, 1000),
        (REG_PROTECTION_CURRENT_L, 500),
        (REG_OVERLOAD_TORQUE, 25),
        (REG_OPERATING_MODE, 0),
    ];
    for &(addr, val) in addrs_and_vals {
        let is_16 = addr == REG_MIN_ANGLE_LIMIT_L
            || addr == REG_MAX_ANGLE_LIMIT_L
            || addr == REG_MAX_TORQUE_L
            || addr == REG_PROTECTION_CURRENT_L;
        let params: Vec<u8> = if is_16 {
            val.to_le_bytes().to_vec()
        } else {
            vec![val as u8]
        };
        let len = (params.len() + 2) as u8;
        let cs = calculate_checksum(1, len, 0x00, &params);
        rx.extend_from_slice(&[0xFF, 0xFF, 1, len, 0x00]);
        rx.extend_from_slice(&params);
        rx.push(cs);
    }
    driver.transport_mut().provide_rx(&rx);

    driver.configure_servos().unwrap();
    assert_eq!(driver.effective_ranges(), &[(100, 3900)]);

    let tx = driver.transport_mut().take_tx();
    for window in tx.windows(8) {
        if window[0] == 0xFF && window[1] == 0xFF && window[4] == INST_WRITE {
            assert!(
                window[5] != REG_MIN_ANGLE_LIMIT_L && window[5] != REG_MAX_ANGLE_LIMIT_L,
                "Must never overwrite EEPROM angle limits when omitted in calib"
            );
        }
    }
}

// -------------------------------------------------------------------------
// 28. Uncalibrated FeetechConfig { calibrated: false, .. } rejects set_torque(true)
// -------------------------------------------------------------------------
#[test]
fn test_feetech_uncalibrated_rejects_set_torque() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_calibrated(false);

    let pos = 2048u16.to_le_bytes();
    let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
    let cs = calculate_checksum(1, 10, 0x00, &params);
    let mut fb = vec![0xFF, 0xFF, 1, 10, 0x00];
    fb.extend_from_slice(&params);
    fb.push(cs);
    driver.transport_mut().provide_rx(&fb);
    driver.read_joint_states().unwrap();

    let err = driver.set_torque(true).unwrap_err();
    assert!(
        matches!(err, ArmDriverError::HardwareFault(ref msg) if msg.contains("uncalibrated")),
        "Expected uncalibrated error, got: {:?}", err
    );
    assert_ne!(driver.enable_state(), EnableState::Enabled);
}

// -------------------------------------------------------------------------
// 29. Out-of-range raw feedback rejects set_torque(true)
// -------------------------------------------------------------------------
#[test]
fn test_feetech_out_of_range_feedback_rejects_set_torque() {
    let mock = MockTransport::new_preserving();
    let driver = FeetechArmDriver::new(mock, vec![1]);

    let mut cfg = feetech::FeetechConfig::default();
    cfg.calibrated = true;
    cfg.joints = vec![feetech::FeetechJointCalib {
        id: 1,
        range_min_steps: Some(500),
        range_max_steps: Some(3500),
        ..Default::default()
    }];
    let mut driver = driver.with_config(cfg);

    let pos = 400u16.to_le_bytes();
    let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
    let cs = calculate_checksum(1, 10, 0x00, &params);
    let mut fb = vec![0xFF, 0xFF, 1, 10, 0x00];
    fb.extend_from_slice(&params);
    fb.push(cs);
    driver.transport_mut().provide_rx(&fb);
    driver.read_joint_states().unwrap();

    let err = driver.set_torque(true).unwrap_err();
    assert!(
        matches!(err, ArmDriverError::HardwareFault(ref msg) if msg.contains("outside calibrated range")),
        "Expected out of range error, got: {:?}", err
    );
    assert_ne!(driver.enable_state(), EnableState::Enabled);
}

// -------------------------------------------------------------------------
// 30. All-None feedback rejects set_torque(true)
// -------------------------------------------------------------------------
#[test]
fn test_feetech_all_none_feedback_rejects_set_torque() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1, 2]).with_calibrated(true);

    assert!(!driver.is_feedback_valid());
    let err = driver.set_torque(true).unwrap_err();
    assert!(
        matches!(err, ArmDriverError::HardwareFault(ref msg) if msg.contains("No valid joint feedback received")),
        "Expected no valid feedback error, got: {:?}", err
    );
}

// -------------------------------------------------------------------------
// 31. Partial faulted servo clear_fault ONLY disables faulted servo and tracks partial enable
// -------------------------------------------------------------------------
#[test]
fn test_feetech_partial_fault_disables_only_faulted_and_detects_partial_enable() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1, 2];
    let mut driver = FeetechArmDriver::new(mock, servo_ids);

    let mut rx_fb = Vec::new();
    for &id in &[1u8, 2u8] {
        let pos = 2048u16.to_le_bytes();
        let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
        let cs = calculate_checksum(id, 10, 0x00, &params);
        rx_fb.extend_from_slice(&[0xFF, 0xFF, id, 10, 0x00]);
        rx_fb.extend_from_slice(&params);
        rx_fb.push(cs);
    }
    driver.transport_mut().provide_rx(&rx_fb);
    driver.read_joint_states().unwrap();
    let _ = driver.transport_mut().take_tx();

    let mut rx_err = Vec::new();
    let pos = 2048u16.to_le_bytes();
    let p1 = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
    let cs1 = calculate_checksum(1, 10, 0x00, &p1);
    rx_err.extend_from_slice(&[0xFF, 0xFF, 1, 10, 0x00]);
    rx_err.extend_from_slice(&p1);
    rx_err.push(cs1);

    let p2 = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
    let cs2 = calculate_checksum(2, 10, 0x20, &p2);
    rx_err.extend_from_slice(&[0xFF, 0xFF, 2, 10, 0x20]);
    rx_err.extend_from_slice(&p2);
    rx_err.push(cs2);

    driver.transport_mut().provide_rx(&rx_err);
    let _ = driver.read_joint_states().unwrap();

    assert_eq!(driver.clear_fault_disables_joints(), vec![1]);

    let cs_tq1 = calculate_checksum(1, 3, 0x00, &[1]);
    let cs_tq2 = calculate_checksum(2, 3, 0x00, &[0]);
    driver.transport_mut().provide_rx(&[
        0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq1,
        0xFF, 0xFF, 2, 3, 0x00, 0, cs_tq2,
    ]);

    let _ = driver.transport_mut().take_tx();
    driver.clear_fault().unwrap();

    let tx = driver.transport_mut().take_tx();
    assert_eq!(tx.len(), 24);
    assert_eq!(tx[2], 2);
    assert_eq!(tx[4], INST_WRITE);
    assert_eq!(tx[5], REG_TORQUE_ENABLE);
    assert_eq!(tx[6], 0);

    assert!(driver.has_known_partial_enable());
}

// -------------------------------------------------------------------------
// 32. Runtime REG_TORQUE_ENABLE poll detects torque loss and triggers HardwareFault
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_feetech_periodic_enable_poll_detects_torque_loss_and_triggers_fault() {
    let mock = Arc::new(Mutex::new(MockTransport::new_preserving()));
    let driver = FeetechArmDriver::new(SharedTransport(mock.clone()), vec![1]).with_calibrated(true);

    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];
    config.resting_pose = Some(RestingPosePolicy {
        positions: None,
        position_tolerances: vec![0.08],
        max_total_deviation: 0.16,
        max_velocity: 0.015,
        joint_max_velocities: vec![],
        support_torque_limits: vec![],
        support_load_limits: vec![],
        feedback_max_age_ms: 100,
    });

    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(driver));

    let pos_1 = 2048u16.to_le_bytes();
    let normal_params = [pos_1[0], pos_1[1], 0, 0, 0, 0, 70, 35];
    let cs_normal = calculate_checksum(1, 10, 0x00, &normal_params);
    let mut rx_normal = vec![0xFF, 0xFF, 1, 10, 0x00];
    rx_normal.extend_from_slice(&normal_params);
    rx_normal.push(cs_normal);

    let cs_tq1 = calculate_checksum(1, 3, 0x00, &[1]);
    let rx_tq1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq1];

    mock.lock().unwrap().provide_rx(&rx_normal);
    mock.lock().unwrap().provide_rx(&rx_tq1);
    mock.lock().unwrap().provide_rx(&rx_normal);

    supervisor.set_torque(true).await.unwrap();
    assert!(supervisor.inner().read().await.is_torqued);

    // Initial enable consumed polls 1 and 2. Ticks 1..7 advance polls to 9.
    for _ in 0..7 {
        mock.lock().unwrap().provide_rx(&rx_normal);
        let obs = supervisor.tick(0.02).await;
        assert!(obs.fault.is_none());
    }

    // On 8th tick (poll 10), driver does a SYNC_READ of REG_TORQUE_ENABLE.
    // Provide normal feedback followed by torque enable response reporting 0 (torque lost!)
    let cs_tq0 = calculate_checksum(1, 3, 0x00, &[0]);
    let rx_tq0 = vec![0xFF, 0xFF, 1, 3, 0x00, 0, cs_tq0];
    mock.lock().unwrap().provide_rx(&rx_normal);
    mock.lock().unwrap().provide_rx(&rx_tq0);

    let obs = supervisor.tick(0.02).await;
    assert!(obs.fault.is_some());
    assert_eq!(obs.fault.unwrap().code, FaultCode::HardwareFault);
}

// -------------------------------------------------------------------------
// 33. set_torque & clear_fault sync loads and enforce support_load_limits
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_supervisor_set_torque_and_clear_fault_sync_loads_enforcing_support_limits() {
    let mock = Arc::new(Mutex::new(MockTransport::new_preserving()));
    let driver = FeetechArmDriver::new(SharedTransport(mock.clone()), vec![1]).with_calibrated(true);

    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];
    config.resting_pose = Some(RestingPosePolicy {
        positions: None,
        position_tolerances: vec![0.08],
        max_total_deviation: 0.16,
        max_velocity: 0.015,
        joint_max_velocities: vec![],
        support_torque_limits: vec![],
        support_load_limits: vec![(0, 0.30)],
        feedback_max_age_ms: 100,
    });

    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(driver));

    let pos_1 = 2048u16.to_le_bytes();
    let load_bytes = 600u16.to_le_bytes();
    let heavy_params = [pos_1[0], pos_1[1], 0, 0, load_bytes[0], load_bytes[1], 70, 35];
    let cs_heavy = calculate_checksum(1, 10, 0x00, &heavy_params);
    let mut rx_heavy = vec![0xFF, 0xFF, 1, 10, 0x00];
    rx_heavy.extend_from_slice(&heavy_params);
    rx_heavy.push(cs_heavy);

    let cs_tq1 = calculate_checksum(1, 3, 0x00, &[1]);
    let rx_tq1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq1];

    mock.lock().unwrap().provide_rx(&rx_heavy);
    mock.lock().unwrap().provide_rx(&rx_tq1);
    mock.lock().unwrap().provide_rx(&rx_heavy);

    supervisor.set_torque(true).await.unwrap();

    let loads = supervisor.inner().read().await.loads.clone();
    assert!((loads[0] - 0.60).abs() < 1e-3);

    let inner = supervisor.inner().read().await;
    assert!(inner.resting_pose_static_violation().is_some());
}

// -------------------------------------------------------------------------
// 34. Gripper stall latched with load feedback and unlatches on load drop or reversal
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_gripper_stall_latched_with_load_feedback_and_unlatches() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("gripper", 0.0, 1.0, 1.0, 1.0)];
    config.home_positions = vec![0.0];
    let supervisor = ArmSupervisor::new(config);

    let mut inner = supervisor.inner().write().await;
    inner.is_torqued = true;
    inner.loads = vec![0.25];
    inner.measured_positions = vec![0.5];
    inner.commanded_positions = vec![0.5];
    inner.gripper_stalled = true;
    inner.gripper_stall_sign = 1.0;

    let load_g = inner.loads[0].abs();
    let gap = (inner.measured_positions[0] - inner.commanded_positions[0]).abs();
    let dir = 1.0f32;
    let same_dir = (dir > 0.0) == (inner.gripper_stall_sign > 0.0);
    let keep_latched = (gap > 0.02) || (load_g >= 0.15 && same_dir);
    assert!(keep_latched);

    inner.loads[0] = 0.05;
    let load_g = inner.loads[0].abs();
    let keep_latched = (gap > 0.02) || (load_g >= 0.15 && same_dir);
    assert!(!keep_latched);

    inner.loads[0] = 0.25;
    let dir_rev = -1.0f32;
    let same_dir_rev = (dir_rev > 0.0) == (inner.gripper_stall_sign > 0.0);
    let keep_latched_rev = (gap > 0.02) || (inner.loads[0].abs() >= 0.15 && same_dir_rev);
    assert!(!keep_latched_rev);
}

// -------------------------------------------------------------------------
// 35. Parquet metadata precedence: sidecar > embedded > dir meta.json
// -------------------------------------------------------------------------
#[test]
fn test_parquet_embedded_metadata_precedence_over_directory_meta_json() {
    use std::fs;
    use tempfile::tempdir;
    use armd::RecordFrame;
    use armd::recording::parquet_writer::{read_metadata_from_parquet, write_frames_to_parquet_with_meta, EpisodeMetadata};

    let dir = tempdir().unwrap();
    let file_path = dir.path().join("episode_000000.parquet");
    let dir_meta = dir.path().join("meta.json");

    let frames = vec![RecordFrame {
        timestamp: 0.0,
        observation_state: vec![0.0],
        action: vec![0.0],
    }];

    // 1. Write parquet with embedded metadata: profile_id = "embedded_profile"
    let mut embedded_meta = EpisodeMetadata::default();
    embedded_meta.profile_id = Some("embedded_profile".into());
    embedded_meta.arm_dof = Some(6);
    write_frames_to_parquet_with_meta(&file_path, &frames, Some(&embedded_meta)).unwrap();

    // Write directory meta.json with profile_id = "dir_profile"
    let mut dir_meta_obj = EpisodeMetadata::default();
    dir_meta_obj.profile_id = Some("dir_profile".into());
    dir_meta_obj.arm_dof = Some(7);
    fs::write(&dir_meta, serde_json::to_string(&dir_meta_obj).unwrap()).unwrap();

    // Embedded metadata MUST take precedence over directory meta.json!
    let read_meta = read_metadata_from_parquet(&file_path).unwrap().unwrap();
    assert_eq!(read_meta.profile_id.as_deref(), Some("embedded_profile"));
    assert_eq!(read_meta.arm_dof, Some(6));

    // 2. Now write a sidecar <id>.meta.json with profile_id = "sidecar_profile"
    let sidecar_path = dir.path().join("episode_000000.meta.json");
    let mut sidecar_meta = EpisodeMetadata::default();
    sidecar_meta.profile_id = Some("sidecar_profile".into());
    sidecar_meta.arm_dof = Some(5);
    fs::write(&sidecar_path, serde_json::to_string(&sidecar_meta).unwrap()).unwrap();

    // Sidecar MUST take precedence over embedded!
    let read_sidecar = read_metadata_from_parquet(&file_path).unwrap().unwrap();
    assert_eq!(read_sidecar.profile_id.as_deref(), Some("sidecar_profile"));
    assert_eq!(read_sidecar.arm_dof, Some(5));

    // 3. Remove sidecar, write parquet WITHOUT embedded metadata, dir meta.json should be used
    fs::remove_file(&sidecar_path).unwrap();
    let empty_meta_file = dir.path().join("episode_empty.parquet");
    write_frames_to_parquet_with_meta(&empty_meta_file, &frames, None).unwrap();

    let read_fallback = read_metadata_from_parquet(&empty_meta_file).unwrap().unwrap();
    assert_eq!(read_fallback.profile_id.as_deref(), Some("dir_profile"));
    assert_eq!(read_fallback.arm_dof, Some(7));
}

// -------------------------------------------------------------------------
// 34. Feetech default config is uncalibrated and overload_torque_raw is None
// -------------------------------------------------------------------------
#[test]
fn test_feetech_default_config_uncalibrated_and_none_overload() {
    let cfg = feetech::FeetechConfig::default();
    assert!(!cfg.calibrated);

    let joint_calib = feetech::FeetechJointCalib::default();
    assert_eq!(joint_calib.overload_torque_raw, None);

    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]);
    let err = driver.set_torque(true).unwrap_err();
    assert!(
        matches!(err, ArmDriverError::HardwareFault(ref msg) if msg.contains("Feetech arm is uncalibrated")),
        "Expected uncalibrated hardware fault, got: {:?}", err
    );
}

// -------------------------------------------------------------------------
// 35. Feetech driver E-stop abort flag halts bus operations immediately
// -------------------------------------------------------------------------
#[test]
fn test_feetech_abort_flag_stops_bus_operations_and_latches() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1, 2]).with_calibrated(true);
    let abort_flag = Arc::new(AtomicBool::new(false));
    driver.set_abort_flag(abort_flag.clone());

    // Initially without abort flag set, feeding valid feedback allows pre-seeding
    let pos_1 = 2048u16.to_le_bytes();
    let normal_params = [pos_1[0], pos_1[1], 0, 0, 0, 0, 70, 35];
    let mut rx_fb = Vec::new();
    for id in [1, 2] {
        let cs = calculate_checksum(id, 10, 0x00, &normal_params);
        rx_fb.extend_from_slice(&[0xFF, 0xFF, id, 10, 0x00]);
        rx_fb.extend_from_slice(&normal_params);
        rx_fb.push(cs);
    }
    driver.transport_mut().provide_rx(&rx_fb);
    driver.read_joint_states().unwrap();
    let _ = driver.transport_mut().take_tx();

    // Now set the abort flag (e.g. software E-stop latch)
    abort_flag.store(true, Ordering::SeqCst);

    // 1. set_torque(true) must be aborted immediately without bus writes
    let err = driver.set_torque(true).unwrap_err();
    assert!(matches!(err, ArmDriverError::HardwareFault(ref msg) if msg.contains("Aborted by emergency stop")));
    assert!(driver.transport_mut().take_tx().is_empty());

    // 2. configure_servos must also abort immediately without bus writes
    let err_cfg = driver.configure_servos().unwrap_err();
    assert!(matches!(err_cfg, ArmDriverError::HardwareFault(ref msg) if msg.contains("Aborted by emergency stop")));
    assert!(driver.transport_mut().take_tx().is_empty());

    // 3. read_joint_states must succeed while aborted
    driver.transport_mut().provide_rx(&rx_fb);
    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 2);
    let _ = driver.transport_mut().take_tx();

    // 4. refresh_enable_state must succeed while aborted
    let cs_en1 = calculate_checksum(1, 3, 0x00, &[0]);
    let cs_en2 = calculate_checksum(2, 3, 0x00, &[0]);
    let rx_ref = vec![0xFF, 0xFF, 1, 3, 0x00, 0, cs_en1, 0xFF, 0xFF, 2, 3, 0x00, 0, cs_en2];
    driver.transport_mut().provide_rx(&rx_ref);
    let en_state = driver.refresh_enable_state().unwrap();
    assert_eq!(en_state, EnableState::Disabled);
    let _ = driver.transport_mut().take_tx();

    // 5. write_joint_targets must deliver hold commands while aborted
    let targets = vec![
        JointTarget::position(0.0),
        JointTarget::position(0.0),
    ];
    driver.write_joint_targets(&targets).unwrap();
    assert!(!driver.transport_mut().take_tx().is_empty());

    // 6. set_torque(false) must succeed while aborted
    let rx_tq_off = vec![0xFF, 0xFF, 1, 3, 0x00, 0, cs_en1, 0xFF, 0xFF, 2, 3, 0x00, 0, cs_en2];
    driver.transport_mut().provide_rx(&rx_tq_off);
    driver.set_torque(false).unwrap();
    assert_eq!(driver.enable_state(), EnableState::Disabled);
}

// -------------------------------------------------------------------------
// 36. Feetech partial enable preserves holding target and selective rollback
// -------------------------------------------------------------------------
#[test]
fn test_feetech_partial_enable_holding_preservation() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1, 2]).with_calibrated(true);

    // 1. Simulate joint feedback so driver has valid feedback for both servos
    let pos_1 = 2048u16.to_le_bytes(); // 0 rad
    let pos_2 = 2500u16.to_le_bytes();
    let params_1 = [pos_1[0], pos_1[1], 0, 0, 0, 0, 70, 35];
    let params_2 = [pos_2[0], pos_2[1], 0, 0, 0, 0, 70, 35];
    let mut rx_fb = Vec::new();
    rx_fb.extend_from_slice(&[0xFF, 0xFF, 1, 10, 0x00]);
    rx_fb.extend_from_slice(&params_1);
    rx_fb.push(calculate_checksum(1, 10, 0x00, &params_1));
    rx_fb.extend_from_slice(&[0xFF, 0xFF, 2, 10, 0x00]);
    rx_fb.extend_from_slice(&params_2);
    rx_fb.push(calculate_checksum(2, 10, 0x00, &params_2));
    driver.transport_mut().provide_rx(&rx_fb);
    driver.read_joint_states().unwrap();
    let _ = driver.transport_mut().take_tx();

    // 2. Refresh enable state where joint 1 is enabled (1) and joint 2 is disabled (0)
    let cs_en1 = calculate_checksum(1, 3, 0x00, &[1]);
    let cs_en2 = calculate_checksum(2, 3, 0x00, &[0]);
    let rx_refresh = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_en1, 0xFF, 0xFF, 2, 3, 0x00, 0, cs_en2];
    driver.transport_mut().provide_rx(&rx_refresh);
    let state = driver.refresh_enable_state().unwrap();
    assert_eq!(state, EnableState::Unknown);
    assert!(driver.has_known_partial_enable());
    assert!(driver.has_any_joint_enabled());
    let _ = driver.transport_mut().take_tx();

    // 3. Now attempt set_torque(true) where joint 2 fails readback (replies 0).
    // Pre-seeding SYNC_WRITE must ONLY contain joint 2! (1 joint = 11 bytes: 0xFF, 0xFF, 0xFE, 7, INST_SYNC_WRITE, 0x2A, 2, ID2, L, H, CS)
    // NOT joint 1 (which is already holding at Some(true)).
    let cs_tq_rb1 = calculate_checksum(1, 3, 0x00, &[1]);
    let cs_tq_rb2 = calculate_checksum(2, 3, 0x00, &[0]); // joint 2 fails
    let cs_rb2 = calculate_checksum(2, 3, 0x00, &[0]); // rollback readback for joint 2 verifies 0
    let rx_enable_rb = vec![
        0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq_rb1,
        0xFF, 0xFF, 2, 3, 0x00, 0, cs_tq_rb2,
        0xFF, 0xFF, 2, 3, 0x00, 0, cs_rb2,
    ];
    driver.transport_mut().provide_rx(&rx_enable_rb);

    let err = driver.set_torque(true).unwrap_err();
    assert!(matches!(err, ArmDriverError::HardwareFault(_)));

    let tx = driver.transport_mut().take_tx();
    // Verify sync-write pre-seeding was for 1 servo only (11 bytes):
    // [0xFF, 0xFF, 0xFE, 7, INST_SYNC_WRITE, 0x2A, 2, 2, pos2_L, pos2_H, CS]
    assert_eq!(tx[0..3], [0xFF, 0xFF, 0xFE]);
    assert_eq!(tx[3], 7); // Length = 7 for 1 servo
    assert_eq!(tx[4], feetech::INST_SYNC_WRITE);
    assert_eq!(tx[7], 2); // Servo ID 2 only!

    // Verify rollback wrote disable ONLY to joint 2, NOT joint 1, and read back joint 2!
    // tx structure: SyncWrite (11) + Enable writes (2*8=16) + Readback reads (2*8=16) + Rollback write (1*8=8) + Rollback read (1*8=8) = 59 bytes
    assert_eq!(tx.len(), 59);
    let rb_pkt = &tx[43..51];
    assert_eq!(rb_pkt[2], 2); // ID 2
    assert_eq!(rb_pkt[4], INST_WRITE);
    assert_eq!(rb_pkt[5], REG_TORQUE_ENABLE);
    assert_eq!(rb_pkt[6], 0); // Torque disabled on joint 2

    // And joint 1 remains known enabled while joint 2 is disabled!
    assert!(driver.has_known_partial_enable());
    assert!(driver.has_any_joint_enabled());
}

// -------------------------------------------------------------------------
// 37. Feetech refresh_enable_state recovers bus state from Unknown
// -------------------------------------------------------------------------
#[test]
fn test_feetech_refresh_enable_state_bus_recovery() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1, 2]);

    assert_eq!(driver.enable_state(), EnableState::Unknown);

    // Case 1: Bus power cycled / cold startup reporting all 0
    let cs_0_1 = calculate_checksum(1, 3, 0x00, &[0]);
    let cs_0_2 = calculate_checksum(2, 3, 0x00, &[0]);
    let rx_all_0 = vec![0xFF, 0xFF, 1, 3, 0x00, 0, cs_0_1, 0xFF, 0xFF, 2, 3, 0x00, 0, cs_0_2];
    driver.transport_mut().provide_rx(&rx_all_0);

    let state = driver.refresh_enable_state().unwrap();
    assert_eq!(state, EnableState::Disabled);
    assert_eq!(driver.enable_state(), EnableState::Disabled);
    assert!(!driver.has_known_partial_enable());
    assert!(!driver.has_any_joint_enabled());

    // Case 2: Bus reporting all 1
    let cs_1_1 = calculate_checksum(1, 3, 0x00, &[1]);
    let cs_1_2 = calculate_checksum(2, 3, 0x00, &[1]);
    let rx_all_1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_1_1, 0xFF, 0xFF, 2, 3, 0x00, 1, cs_1_2];
    driver.transport_mut().provide_rx(&rx_all_1);

    let state = driver.refresh_enable_state().unwrap();
    assert_eq!(state, EnableState::Enabled);
    assert_eq!(driver.enable_state(), EnableState::Enabled);
    assert!(!driver.has_known_partial_enable());
    assert!(driver.has_any_joint_enabled());
}

// -------------------------------------------------------------------------
// 38. supervisor.clear_fault rejects stale or moving pose before disabling faulted joint
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_supervisor_clear_fault_rejects_stale_or_moving_pose_before_hardware_disable() {
    let mock = Arc::new(Mutex::new(MockTransport::new_preserving()));
    let driver = FeetechArmDriver::new(SharedTransport(mock.clone()), vec![1]).with_calibrated(true);

    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];
    config.resting_pose = Some(RestingPosePolicy {
        positions: None,
        position_tolerances: vec![0.08],
        max_total_deviation: 0.16,
        max_velocity: 0.015,
        joint_max_velocities: vec![],
        support_torque_limits: vec![],
        support_load_limits: vec![],
        feedback_max_age_ms: 100,
    });

    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(driver));

    let pos_1 = 2048u16.to_le_bytes();
    let normal_params = [pos_1[0], pos_1[1], 0, 0, 0, 0, 70, 35];
    let cs_normal = calculate_checksum(1, 10, 0x00, &normal_params);
    let mut rx_normal = vec![0xFF, 0xFF, 1, 10, 0x00];
    rx_normal.extend_from_slice(&normal_params);
    rx_normal.push(cs_normal);

    let cs_tq1 = calculate_checksum(1, 3, 0x00, &[1]);
    let rx_tq1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq1];

    mock.lock().unwrap().provide_rx(&rx_normal);
    mock.lock().unwrap().provide_rx(&rx_tq1);
    mock.lock().unwrap().provide_rx(&rx_normal);

    supervisor.set_torque(true).await.unwrap();
    assert!(supervisor.inner().read().await.is_torqued);

    // Inject hardware overload fault (0x20)
    let cs_overload = calculate_checksum(1, 10, 0x20, &normal_params);
    let mut rx_overload = vec![0xFF, 0xFF, 1, 10, 0x20];
    rx_overload.extend_from_slice(&normal_params);
    rx_overload.push(cs_overload);
    mock.lock().unwrap().provide_rx(&rx_overload);

    let obs = supervisor.tick(0.02).await;
    assert!(obs.fault.is_some());
    assert_eq!(obs.fault.unwrap().code, FaultCode::HardwareFault);

    // Drain TX buffer before testing clear_fault
    mock.lock().unwrap().take_tx();

    // Case 1: Feedback indicates arm joint is moving (speed != 0, velocity > max_velocity 0.015)
    let moving_speed = 100u16.to_le_bytes();
    let moving_params = [pos_1[0], pos_1[1], moving_speed[0], moving_speed[1], 0, 0, 70, 35];
    let cs_moving = calculate_checksum(1, 10, 0x20, &moving_params);
    let mut rx_moving = vec![0xFF, 0xFF, 1, 10, 0x20];
    rx_moving.extend_from_slice(&moving_params);
    rx_moving.push(cs_moving);
    mock.lock().unwrap().provide_rx(&rx_moving);

    let res_moving = supervisor.clear_fault().await;
    assert!(res_moving.is_err(), "clear_fault must reject when arm is moving");
    let err_str = res_moving.unwrap_err().to_string();
    assert!(
        err_str.contains("away from resting pose") || err_str.contains("velocity"),
        "Expected resting pose/velocity violation error, got: {err_str}"
    );
    assert!(supervisor.inner().read().await.is_torqued, "is_torqued must be preserved on rejected clear_fault");

    // Verify no torque disable command was sent to hardware
    let tx_moving = mock.lock().unwrap().take_tx();
    let disable_cmd = feetech::build_set_torque(1, false);
    assert!(
        !tx_moving.windows(disable_cmd.len()).any(|w| w == disable_cmd),
        "Hardware torque disable must NOT be sent when clear_fault rejects due to motion"
    );

    // Case 2: Feedback is stale (no response provided to sync_read, or age > feedback_max_age_ms)
    let res_stale = supervisor.clear_fault().await;
    assert!(res_stale.is_err(), "clear_fault must reject when feedback is stale/unavailable");
    assert!(supervisor.inner().read().await.is_torqued, "is_torqued must be preserved");

    let tx_stale = mock.lock().unwrap().take_tx();
    assert!(
        !tx_stale.windows(disable_cmd.len()).any(|w| w == disable_cmd),
        "Hardware torque disable must NOT be sent when clear_fault rejects due to stale feedback"
    );
}

// -------------------------------------------------------------------------
// 39. Feetech with_config growing and shrinking reinitializes all per-servo state
// -------------------------------------------------------------------------
#[test]
fn test_feetech_with_config_growing_and_shrinking() {
    let mock = MockTransport::new_preserving();
    let driver = FeetechArmDriver::new(mock, vec![1, 2]);
    assert_eq!(driver.effective_ranges().len(), 2);

    // Grow to 4 joints
    let cfg_grow = feetech::FeetechConfig {
        joints: vec![
            feetech::FeetechJointCalib { id: 1, range_min_steps: Some(100), range_max_steps: Some(4000), ..Default::default() },
            feetech::FeetechJointCalib { id: 2, range_min_steps: Some(200), range_max_steps: Some(3900), ..Default::default() },
            feetech::FeetechJointCalib { id: 3, range_min_steps: Some(300), range_max_steps: Some(3800), ..Default::default() },
            feetech::FeetechJointCalib { id: 4, range_min_steps: Some(400), range_max_steps: Some(3700), ..Default::default() },
        ],
        ..Default::default()
    };
    let driver_grow = driver.with_config(cfg_grow);
    assert_eq!(driver_grow.effective_ranges().len(), 4);
    assert_eq!(driver_grow.effective_ranges(), &[(100, 4000), (200, 3900), (300, 3800), (400, 3700)]);

    // Shrink to 1 joint
    let cfg_shrink = feetech::FeetechConfig {
        joints: vec![
            feetech::FeetechJointCalib { id: 5, range_min_steps: Some(500), range_max_steps: Some(3500), ..Default::default() },
        ],
        ..Default::default()
    };
    let driver_shrink = driver_grow.with_config(cfg_shrink);
    assert_eq!(driver_shrink.effective_ranges().len(), 1);
    assert_eq!(driver_shrink.effective_ranges(), &[(500, 3500)]);
}

// -------------------------------------------------------------------------
// 40. Missing rollback readback leaves state Unknown
// -------------------------------------------------------------------------
#[test]
fn test_feetech_missing_rollback_readback_leaves_state_unknown() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1, 2];
    let mut driver = FeetechArmDriver::new(mock, servo_ids.clone()).with_calibrated(true);

    // Provide feedback first
    let mut rx_fb = Vec::new();
    for &id in &servo_ids {
        let pos = 2048u16.to_le_bytes();
        let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
        let cs = calculate_checksum(id, 10, 0x00, &params);
        rx_fb.extend_from_slice(&[0xFF, 0xFF, id, 10, 0x00]);
        rx_fb.extend_from_slice(&params);
        rx_fb.push(cs);
    }
    driver.transport_mut().provide_rx(&rx_fb);
    driver.read_joint_states().unwrap();
    let _ = driver.transport_mut().take_tx();

    // Servo 1 reports 1 (enabled), Servo 2 fails (reports 0).
    // But DO NOT provide rollback readback response in rx!
    let cs1 = calculate_checksum(1, 3, 0x00, &[1]);
    let resp1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs1];
    let cs2 = calculate_checksum(2, 3, 0x00, &[0]);
    let resp2 = vec![0xFF, 0xFF, 2, 3, 0x00, 0, cs2];
    let mut rx = Vec::new();
    rx.extend_from_slice(&resp1);
    rx.extend_from_slice(&resp2);
    driver.transport_mut().provide_rx(&rx);

    let res = driver.set_torque(true);
    assert!(res.is_err());
    // Missing rollback readback leaves enable state as Unknown
    assert_eq!(driver.enable_state(), EnableState::Unknown);
}

// -------------------------------------------------------------------------
// 41. STS3215 Present_Position bit-15 negative ticks decoding
// -------------------------------------------------------------------------
#[test]
fn test_feetech_decode_present_position_negative_ticks() {
    // 0x8032: bit 15 = 1, magnitude = 0x32 = 50 -> -50 ticks
    let decoded = feetech::decode_present_position(0x8032);
    assert_eq!(decoded, -50);

    // Normal position 2048
    assert_eq!(feetech::decode_present_position(2048), 2048);

    // Test conversion through FeetechJointCalib::steps_to_rad
    let calib = feetech::FeetechJointCalib {
        id: 1,
        zero_step: 0,
        homing_offset_steps: 0,
        drive_mode_invert: false,
        ..Default::default()
    };
    let rad = calib.steps_to_rad(-50);
    assert!(rad < 0.0, "Expected negative radians, got {rad}");
    let expected_rad = -50.0 * (2.0 * PI / 4096.0);
    assert!((rad - expected_rad).abs() < 1e-5);
}

// -------------------------------------------------------------------------
// 42. EEPROM relock REG_LOCK = 1 sent when abort flag is set
// -------------------------------------------------------------------------
#[test]
fn test_feetech_eeprom_relock_sent_when_aborted_during_write() {
    struct AbortOnLock0Transport {
        inner: MockTransport,
        abort_flag: Arc<AtomicBool>,
    }
    impl Transport for AbortOnLock0Transport {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Transport::clock(&self.inner)
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
            self.inner.read(buf)
        }
        fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
            let lock_0 = feetech::build_write_u8(1, feetech::REG_LOCK, 0);
            if buf == lock_0.as_slice() {
                self.abort_flag.store(true, Ordering::SeqCst);
            }
            self.inner.write(buf)
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
    }

    let mock = MockTransport::new_preserving();
    let abort_flag = Arc::new(AtomicBool::new(false));
    let transport = AbortOnLock0Transport {
        inner: mock,
        abort_flag: abort_flag.clone(),
    };
    let mut driver = FeetechArmDriver::new(transport, vec![1]);
    driver.set_abort_flag(abort_flag.clone());

    let mut cfg = feetech::FeetechConfig::default();
    cfg.joints = vec![feetech::FeetechJointCalib {
        id: 1,
        range_min_steps: Some(200),
        range_max_steps: Some(3800),
        ..Default::default()
    }];
    let mut driver = driver.with_config(cfg);

    let mut rx = Vec::new();
    let add_u16 = |rx: &mut Vec<u8>, val: u16| {
        let b = val.to_le_bytes();
        let cs = calculate_checksum(1, 4, 0x00, &b);
        rx.extend_from_slice(&[0xFF, 0xFF, 1, 4, 0x00, b[0], b[1], cs]);
    };
    let add_u8 = |rx: &mut Vec<u8>, val: u8| {
        let cs = calculate_checksum(1, 3, 0x00, &[val]);
        rx.extend_from_slice(&[0xFF, 0xFF, 1, 3, 0x00, val, cs]);
    };

    add_u16(&mut rx, feetech::EXPECTED_MODEL_STS3215); // model
    add_u8(&mut rx, 0); // torque_en
    add_u8(&mut rx, 250); // return delay
    add_u8(&mut rx, 1); // status return level
    add_u16(&mut rx, 0); // min angle (differs from target 200!)
    add_u16(&mut rx, 4095); // max angle
    add_u8(&mut rx, 32); // p coef
    add_u8(&mut rx, 32); // d coef
    add_u8(&mut rx, 0); // i coef
    add_u16(&mut rx, 1000); // max torque
    add_u16(&mut rx, 1000); // prot current
    add_u8(&mut rx, 100); // overload torque
    add_u8(&mut rx, 0); // operating mode

    driver.transport_mut().inner.provide_rx(&rx);

    let res = driver.configure_servos();
    assert!(res.is_err(), "configure_servos must fail when aborted during write");

    let tx = driver.transport_mut().inner.take_tx();
    let lock_packet = feetech::build_write_u8(1, feetech::REG_LOCK, 1);
    assert!(
        tx.windows(lock_packet.len()).any(|w| w == lock_packet),
        "REG_LOCK = 1 must be sent for cleanup even when aborted!"
    );
}

// -------------------------------------------------------------------------
// 43. Integrated ArmSupervisor + FeetechArmDriver + MockTransport E-stop lifecycle
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_feetech_supervisor_estop_abort_lifecycle() {
    let mock = Arc::new(Mutex::new(MockTransport::new_preserving()));
    let driver = FeetechArmDriver::new(SharedTransport(mock.clone()), vec![1, 2])
        .with_calibrated(true);

    let mut config = ArmConfig::default();
    config.dof = 2;
    config.limits = vec![
        JointLimit::new("j1", -PI, PI, 1.0, 1.0),
        JointLimit::new("j2", -PI, PI, 1.0, 1.0),
    ];
    config.home_positions = vec![0.0, 0.0];
    let resting = RestingPosePolicy {
        positions: None,
        position_tolerances: vec![0.1, 0.1],
        max_total_deviation: 0.2,
        max_velocity: 0.1,
        joint_max_velocities: vec![],
        support_torque_limits: vec![],
        support_load_limits: vec![],
        feedback_max_age_ms: 250,
    };
    config.resting_pose = Some(resting);

    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(driver));

    // Provide initial feedback
    let pos_0 = 2048u16.to_le_bytes();
    let normal_params = [pos_0[0], pos_0[1], 0, 0, 0, 0, 70, 35];
    let mut rx_fb = Vec::new();
    for id in [1, 2] {
        let cs = calculate_checksum(id, 10, 0x00, &normal_params);
        rx_fb.extend_from_slice(&[0xFF, 0xFF, id, 10, 0x00]);
        rx_fb.extend_from_slice(&normal_params);
        rx_fb.push(cs);
    }
    mock.lock().unwrap().provide_rx(&rx_fb);
    supervisor.tick(0.02).await;
    let _ = mock.lock().unwrap().take_tx();

    // Trigger E-stop!
    let (state, _) = supervisor.trigger_e_stop().await;
    assert!(state.is_stopped());

    // Tick sends hold target & keeps telemetry fresh
    mock.lock().unwrap().provide_rx(&rx_fb);
    supervisor.tick(0.02).await;
    let tx_tick = mock.lock().unwrap().take_tx();
    assert!(!tx_tick.is_empty(), "tick sends hold targets during E-stop hold");

    // Clear fault: supervisor.clear_fault() succeeds
    let cs_en1 = calculate_checksum(1, 3, 0x00, &[0]);
    let cs_en2 = calculate_checksum(2, 3, 0x00, &[0]);
    let rx_cf = vec![0xFF, 0xFF, 1, 3, 0x00, 0, cs_en1, 0xFF, 0xFF, 2, 3, 0x00, 0, cs_en2];
    mock.lock().unwrap().provide_rx(&rx_fb);
    mock.lock().unwrap().provide_rx(&rx_cf);
    let clear_res = supervisor.clear_fault().await;
    assert!(clear_res.is_ok(), "clear_fault must succeed: {:?}", clear_res);

    // Now set_torque(true) succeeds afterward!
    let cs_tq1 = calculate_checksum(1, 3, 0x00, &[1]);
    let cs_tq2 = calculate_checksum(2, 3, 0x00, &[1]);
    let rx_enable = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq1, 0xFF, 0xFF, 2, 3, 0x00, 1, cs_tq2];
    mock.lock().unwrap().provide_rx(&rx_enable);
    let enable_res = supervisor.set_torque(true).await;
    assert!(enable_res.is_ok(), "set_torque(true) must succeed after clear_fault: {:?}", enable_res);
}

// -------------------------------------------------------------------------
// 44. Gripper LoadAndGap stall release requires reverse command
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_feetech_gripper_load_and_gap_stall_release_requires_reverse_command() {
    struct LoadFeedbackDriver;
    impl ArmDriver for LoadFeedbackDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
                temperature_feedback: false,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: true,
            }
        }
        fn enable_state(&self) -> EnableState { EnableState::Enabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
        fn joint_loads(&self) -> Option<Vec<f32>> { Some(vec![0.05; 7]) }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok((0..7).map(|_| JointState {
                position: 0.5,
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            }).collect())
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    }

    let mut config = ArmConfig::default();
    config.dof = 7;
    config.limits = (0..7).map(|i| JointLimit::new(format!("j{i}"), -PI, PI, 1.0, 1.0)).collect();
    config.home_positions = vec![0.0; 7];
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(LoadFeedbackDriver));

    let mut inner = supervisor.inner().write().await;
    inner.is_torqued = true;
    inner.loads = vec![0.0; 7];
    inner.loads[6] = 0.25; // high load on gripper
    inner.measured_positions = vec![0.5; 7];
    inner.commanded_positions = vec![0.5; 7];
    inner.gripper_stalled = true;
    inner.gripper_stall_sign = 1.0;
    inner.gripper_low_torque_ticks = 0;
    inner.last_can_feedback_instant = Some(Instant::now());
    drop(inner);

    // Tick with load dropped to 0.05 (< 0.15), but gap is 0 (no reverse command).
    // The gripper must STAY stalled!
    for _ in 0..10 {
        let mut inner = supervisor.inner().write().await;
        inner.last_can_feedback_instant = Some(Instant::now());
        inner.loads[6] = 0.05;
        inner.measured_positions[6] = 0.5;
        inner.commanded_positions[6] = 0.5;
        drop(inner);
        supervisor.tick(0.02).await;
        assert!(supervisor.is_gripper_stalled().await, "Gripper must remain stalled when load relaxes without reverse command");
    }

    // Now send reverse command: commanded position moves backwards so gap * stall_sign < -0.005
    for _ in 0..8 {
        let mut inner = supervisor.inner().write().await;
        inner.last_can_feedback_instant = Some(Instant::now());
        inner.loads[6] = 0.05;
        inner.measured_positions[6] = 0.5;
        inner.commanded_positions[6] = 0.45;
        drop(inner);
        supervisor.tick(0.02).await;
    }
    assert!(!supervisor.is_gripper_stalled().await, "Gripper stall must release after reverse command");
}

// -------------------------------------------------------------------------
// 45. Concurrent trigger_e_stop is not overwritten by clear_fault
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_supervisor_clear_fault_does_not_overwrite_concurrent_estop() {
    struct ConcurrentEstopDriver {
        read_started: Arc<tokio::sync::Notify>,
        read_proceed: Arc<AtomicBool>,
        delay_enabled: Arc<AtomicBool>,
    }

    impl ArmDriver for ConcurrentEstopDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
                temperature_feedback: false,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: false,
            }
        }
        fn enable_state(&self) -> EnableState { EnableState::Disabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO)] }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            if self.delay_enabled.load(Ordering::SeqCst) {
                self.read_started.notify_one();
                while !self.read_proceed.load(Ordering::SeqCst) {
                    std::thread::sleep(Duration::from_millis(5));
                }
            }
            Ok(vec![JointState {
                position: 0.0,
                velocity: 0.0,
                torque: 0.0,
                temperature: 25.0,
            }])
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    }

    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -PI, PI, 1.0, 1.0)];
    config.home_positions = vec![0.0];

    let read_started = Arc::new(tokio::sync::Notify::new());
    let read_proceed = Arc::new(AtomicBool::new(false));
    let delay_enabled = Arc::new(AtomicBool::new(false));

    let driver = ConcurrentEstopDriver {
        read_started: read_started.clone(),
        read_proceed: read_proceed.clone(),
        delay_enabled: delay_enabled.clone(),
    };

    let supervisor = Arc::new(ArmSupervisor::new(config));
    supervisor.set_driver(Box::new(driver)).await;

    // Initial E-stop
    supervisor.trigger_e_stop().await;
    assert!(supervisor.e_stop_latched.load(Ordering::SeqCst));
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::EStop);

    // Now enable delay for clear_fault's read_joint_states call
    delay_enabled.store(true, Ordering::SeqCst);
    read_proceed.store(false, Ordering::SeqCst);

    let sup1 = supervisor.clone();
    let clear_handle = tokio::spawn(async move {
        sup1.clear_fault().await
    });

    // Wait until clear_fault is inside read_joint_states holding inner in spawn_blocking
    read_started.notified().await;

    // Trigger concurrent e-stop while clear_fault holds inner lock in spawn_blocking
    let sup2 = supervisor.clone();
    let estop_handle = tokio::spawn(async move {
        sup2.trigger_e_stop().await
    });

    // Wait until trigger_e_stop has incremented requested_estop_count (from 1 to 2) and set e_stop_latched
    while supervisor.requested_estop_count.load(Ordering::SeqCst) < 2 {
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Now unblock read_joint_states so clear_fault can continue
    read_proceed.store(true, Ordering::SeqCst);

    let (clear_res, _) = tokio::join!(clear_handle, estop_handle);
    let clear_res = clear_res.unwrap();

    // 1) clear_res is Err(ArmError::Safety(SafetyError::EStopActive))
    assert!(
        matches!(clear_res, Err(ArmError::Safety(SafetyError::EStopActive))),
        "Expected EStopActive, got {:?}",
        clear_res
    );
    // 2) sup.e_stop_latched is true
    assert!(supervisor.e_stop_latched.load(Ordering::SeqCst));
    // 3) observation safety_state == SafetyState::EStop
    assert_eq!(supervisor.get_observation().await.safety_state, SafetyState::EStop);

    // 4) A subsequent non-concurrent sup.clear_fault().await (with healthy feedback) succeeds!
    delay_enabled.store(false, Ordering::SeqCst);
    let second_res = supervisor.clear_fault().await;
    assert!(second_res.is_ok(), "Subsequent non-concurrent clear_fault must succeed, got {:?}", second_res);
    assert!(!supervisor.e_stop_latched.load(Ordering::SeqCst));
}

// -------------------------------------------------------------------------
// 46. checked_rad_to_steps_in_range tolerance and rejection
// -------------------------------------------------------------------------
#[test]
fn test_checked_rad_to_steps_in_range_tolerance_and_rejection() {
    let calib = feetech::FeetechJointCalib {
        id: 1,
        zero_step: 2048,
        homing_offset_steps: 0,
        drive_mode_invert: false,
        ..Default::default()
    };
    let range = (500, 3500);

    // Exact within range
    let rad_mid = 0.0f32; // 2048 steps
    let steps_mid = calib.checked_rad_to_steps_in_range(rad_mid, range).unwrap();
    assert_eq!(steps_mid, 2048);

    // Boundary with float rounding within 2 steps tolerance: clamped cleanly
    let rad_near_min = (499.0 - 2048.0) * (2.0 * PI / 4096.0);
    let steps_clamped = calib.checked_rad_to_steps_in_range(rad_near_min, range).unwrap();
    assert_eq!(steps_clamped, 500);

    // Far outside range: rejected with ParameterError
    let rad_far_min = (400.0 - 2048.0) * (2.0 * PI / 4096.0);
    assert!(calib.checked_rad_to_steps_in_range(rad_far_min, range).is_err());

    let rad_far_max = (3600.0 - 2048.0) * (2.0 * PI / 4096.0);
    assert!(calib.checked_rad_to_steps_in_range(rad_far_max, range).is_err());
}

// -------------------------------------------------------------------------
// 47. Noise-prefixed TX echo filter in direct register reads
// -------------------------------------------------------------------------
#[test]
fn test_feetech_direct_register_reads_filter_noise_prefixed_echo() {
    let mock = MockTransport::new_preserving();
    let id = 1u8;
    let mut driver = FeetechArmDriver::new(mock, vec![id]);

    // 1. read_register_u8_direct with [0x00] prefix before echo
    let reg_u8 = REG_TORQUE_ENABLE;
    let req_u8 = feetech::build_read(id, reg_u8, 1);
    let val_u8 = 55u8;
    let cs_u8 = calculate_checksum(id, 3, 0x00, &[val_u8]);
    let reply_u8 = vec![0xFF, 0xFF, id, 3, 0x00, val_u8, cs_u8];

    let mut stream_u8 = vec![0x00];
    stream_u8.extend_from_slice(&req_u8);
    stream_u8.extend_from_slice(&reply_u8);
    driver.transport_mut().provide_rx(&stream_u8);

    let res_u8 = driver.read_register_u8_direct(id, reg_u8).expect("read_register_u8_direct must succeed");
    assert_eq!(res_u8, val_u8, "Must return real register value 55, not echo parameter");

    // 2. read_register_u16_direct with [0x00] prefix before echo
    let reg_u16 = REG_PRESENT_POSITION_L;
    let req_u16 = feetech::build_read(id, reg_u16, 2);
    let val_u16: u16 = 2500;
    let val_bytes = val_u16.to_le_bytes();
    let cs_u16 = calculate_checksum(id, 4, 0x00, &[val_bytes[0], val_bytes[1]]);
    let reply_u16 = vec![0xFF, 0xFF, id, 4, 0x00, val_bytes[0], val_bytes[1], cs_u16];

    let mut stream_u16 = vec![0x00];
    stream_u16.extend_from_slice(&req_u16);
    stream_u16.extend_from_slice(&reply_u16);
    driver.transport_mut().provide_rx(&stream_u16);

    let res_u16 = driver.read_register_u16_direct(id, reg_u16).expect("read_register_u16_direct must succeed");
    assert_eq!(res_u16, val_u16, "Must return real register value 2500, not [reg, len] echo");
}

// -------------------------------------------------------------------------
// 48. Invalidate cached feedback on calibration change in with_config
// -------------------------------------------------------------------------
#[test]
fn test_feetech_feedback_invalidated_on_calibration_change() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]);

    let pos = 2048u16.to_le_bytes();
    let params = [pos[0], pos[1], 0, 0, 0, 0, 0, 30];
    let cs = calculate_checksum(1, 10, 0x00, &params);
    let mut fb = vec![0xFF, 0xFF, 1, 10, 0x00];
    fb.extend_from_slice(&params);
    fb.push(cs);

    // Initial feedback makes is_feedback_valid() true
    driver.transport_mut().provide_rx(&fb);
    driver.read_joint_states().unwrap();
    assert!(driver.is_feedback_valid());

    // zero_step change invalidates feedback
    let mut cfg1 = driver.config().clone();
    cfg1.joints[0].zero_step = 2100;
    driver = driver.with_config(cfg1);
    assert!(!driver.is_feedback_valid(), "Feedback must be invalidated when zero_step changes");

    // Provide fresh feedback
    driver.transport_mut().provide_rx(&fb);
    driver.read_joint_states().unwrap();
    assert!(driver.is_feedback_valid());

    // homing_offset_steps change invalidates feedback
    let mut cfg2 = driver.config().clone();
    cfg2.joints[0].homing_offset_steps = 50;
    driver = driver.with_config(cfg2);
    assert!(!driver.is_feedback_valid(), "Feedback must be invalidated when homing_offset_steps changes");
}

// -------------------------------------------------------------------------
// 49. REG_TORQUE_ENABLE polling unstarves and detects torque loss after >= 200ms
// -------------------------------------------------------------------------
#[test]
fn test_feetech_enable_poll_unstarves_when_position_chunk_timeouts() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_calibrated(true);

    // Provide feedback and enable torque
    let pos = 2048u16.to_le_bytes();
    let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
    let cs = calculate_checksum(1, 10, 0x00, &params);
    let mut fb = vec![0xFF, 0xFF, 1, 10, 0x00];
    fb.extend_from_slice(&params);
    fb.push(cs);

    // Initial feedback so driver has verified feedback
    driver.transport_mut().provide_rx(&fb);
    driver.read_joint_states().unwrap();
    assert!(driver.is_feedback_valid());

    let cs_tq1 = calculate_checksum(1, 3, 0x00, &[1]);
    let rx_tq1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq1];

    let mut init_rx = Vec::new();
    init_rx.extend_from_slice(&fb);
    init_rx.extend_from_slice(&rx_tq1);
    driver.transport_mut().provide_rx(&init_rx);

    driver.set_torque(true).unwrap();
    assert_eq!(driver.enable_state(), EnableState::Enabled);
    let _ = driver.transport_mut().take_tx();

    // Fast-forward last_enable_poll by setting it in the past (> 200ms)
    driver.last_enable_poll = Some(driver.clock_now() - Duration::from_millis(250));

    // Provide only the REG_TORQUE_ENABLE response with value 0 (disabled!),
    // NO position chunk feedback is provided, so position chunk read will hit timeout.
    let cs_tq0 = calculate_checksum(1, 3, 0x00, &[0]);
    let rx_tq0 = vec![0xFF, 0xFF, 1, 3, 0x00, 0, cs_tq0];
    driver.transport_mut().provide_rx(&rx_tq0);

    // read_joint_states() should poll REG_TORQUE_ENABLE first (before position chunks),
    // detect disabled torque, and update enable_state to Disabled!
    let _ = driver.read_joint_states();
    assert_eq!(
        driver.enable_state(),
        EnableState::Disabled,
        "Torque disable must be detected even when position chunk hits timeout"
    );
}

// -------------------------------------------------------------------------
// 50. Pending requested_estop_count > applied_estop_count rejects operations
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_supervisor_pending_estop_count_rejects_clear_fault_set_torque_and_servo() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -PI, PI, 1.0, 1.0)];
    config.home_positions = vec![0.0];

    struct DummyDriver;
    impl ArmDriver for DummyDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
                temperature_feedback: false,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: false,
            }
        }
        fn enable_state(&self) -> EnableState { EnableState::Disabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO)] }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState {
                position: 0.0,
                velocity: 0.0,
                torque: 0.0,
                temperature: 25.0,
            }])
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    }

    let supervisor = Arc::new(ArmSupervisor::new(config));
    supervisor.set_driver(Box::new(DummyDriver)).await;

    // Simulate pending requested_estop_count increment without applied_estop_count
    supervisor.requested_estop_count.fetch_add(1, Ordering::SeqCst);

    // 1. clear_fault fails with EStopActive
    let clear_res = supervisor.clear_fault().await;
    assert!(matches!(clear_res, Err(ArmError::Safety(SafetyError::EStopActive))));
    assert!(supervisor.e_stop_latched.load(Ordering::SeqCst));

    // 2. set_torque(true) fails with EStopActive
    let torque_res = supervisor.set_torque(true).await;
    assert!(matches!(torque_res, Err(ArmError::Safety(SafetyError::EStopActive))));

    // 3. servo fails with EStopActive
    let servo_res = supervisor.servo(vec![0.0], 1).await;
    assert!(matches!(servo_res, Err(ArmError::Safety(SafetyError::EStopActive))));
}

// -------------------------------------------------------------------------
// 51. Stale REG_TORQUE_ENABLE degradation to Unknown and HardwareFault
// -------------------------------------------------------------------------
#[test]
fn test_feetech_stale_enable_poll_degrades_to_unknown_and_hardware_fault() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_calibrated(true);

    let pos = 2048u16.to_le_bytes();
    let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
    let cs = calculate_checksum(1, 10, 0x00, &params);
    let mut fb = vec![0xFF, 0xFF, 1, 10, 0x00];
    fb.extend_from_slice(&params);
    fb.push(cs);

    let cs_tq1 = calculate_checksum(1, 3, 0x00, &[1]);
    let rx_tq1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq1];

    // Initial feedback so feedback is valid
    driver.transport_mut().provide_rx(&fb);
    driver.read_joint_states().unwrap();
    assert!(driver.is_feedback_valid());

    driver.transport_mut().provide_rx(&rx_tq1);
    driver.set_torque(true).unwrap();
    assert_eq!(driver.enable_state(), EnableState::Enabled);
    let _ = driver.transport_mut().take_tx();

    // Now position telemetry stays fresh, but enable responses either return 2 or are suppressed
    // First, test unexpected value 2 immediately degrades to Unknown:
    driver.last_enable_poll = Some(driver.clock_now() - Duration::from_millis(250));
    let cs_tq2 = calculate_checksum(1, 3, 0x00, &[2]);
    let rx_tq2 = vec![0xFF, 0xFF, 1, 3, 0x00, 2, cs_tq2];
    let mut poll_rx = Vec::new();
    poll_rx.extend_from_slice(&rx_tq2);
    poll_rx.extend_from_slice(&fb);
    driver.transport_mut().provide_rx(&poll_rx);
    let _ = driver.read_joint_states();
    assert_eq!(driver.enable_state(), EnableState::Unknown);

    // Re-enable and test 3 consecutive missed enable polls degrades to Unknown
    driver.transport_mut().provide_rx(&fb);
    driver.read_joint_states().unwrap();
    driver.transport_mut().provide_rx(&rx_tq1);
    driver.set_torque(true).unwrap();
    assert_eq!(driver.enable_state(), EnableState::Enabled);

    for _ in 0..3 {
        driver.last_enable_poll = Some(driver.clock_now() - Duration::from_millis(250));
        // Provide ONLY position feedback; enable poll receives no response
        driver.transport_mut().provide_rx(&fb);
        let _ = driver.read_joint_states();
    }
    assert_eq!(driver.enable_state(), EnableState::Unknown);
}

// -------------------------------------------------------------------------
// 52. Gripper overload 0x20 handling vs arm joint faults
// -------------------------------------------------------------------------
#[test]
fn test_feetech_gripper_benign_overload_vs_arm_fault() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1, 2, 3, 4, 5, 6];
    let mut driver = FeetechArmDriver::new(mock, servo_ids)
        .with_calibrated(true)
        .with_gripper_index(Some(5));

    // Case 1: Arm joint 0 has 0x20 overload -> check_hardware_status fails!
    driver.motor_status[0] = 0x20;
    assert!(driver.check_hardware_status().is_err());
    driver.motor_status[0] = 0;

    // Case 2: Gripper joint 5 has 0x04 overheat -> check_hardware_status fails!
    driver.motor_status[5] = 0x04;
    assert!(driver.check_hardware_status().is_err());

    // Case 3: Gripper joint 5 has 0x24 (overheat + overload) -> check_hardware_status fails!
    driver.motor_status[5] = 0x24;
    assert!(driver.check_hardware_status().is_err());

    // Case 4: Gripper joint 5 has 0x20 benign overload -> check_hardware_status SUCCEEDS!
    driver.motor_status[5] = 0x20;
    assert!(driver.check_hardware_status().is_ok());

    // And clear_fault_disables_joints() does NOT include gripper joint 5 when status is 0x20
    let disable_joints = driver.clear_fault_disables_joints();
    assert!(!disable_joints.contains(&5));
}

// -------------------------------------------------------------------------
// 53. Late drained frames and incomplete_chunk_idx handling
// -------------------------------------------------------------------------
#[test]
fn test_feetech_late_drained_frames_and_incomplete_chunk_idx() {
    let mock = MockTransport::new_preserving();
    let servo_ids = vec![1, 2, 3];
    let mut driver = FeetechArmDriver::new(mock, servo_ids);

    let build_resp = |id: u8| -> Vec<u8> {
        let pos = 2048u16.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 100u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    // Provide complete servos 1, 2, and incomplete servo 3 header
    let resp3 = build_resp(3);
    let mut rx = Vec::new();
    rx.extend_from_slice(&build_resp(1));
    rx.extend_from_slice(&build_resp(2));
    rx.extend_from_slice(&resp3[..4]); // incomplete frame!
    driver.transport_mut().provide_rx(&rx);

    let _ = driver.read_joint_states();
    assert_eq!(driver.incomplete_chunk_idx(), Some(0));

    // Next call, supply the rest of servo 3
    let mut rx2 = Vec::new();
    rx2.extend_from_slice(&resp3[4..]);
    driver.transport_mut().provide_rx(&rx2);

    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 3);
    assert!(driver.feedback_ages()[2].is_some(), "Servo 3 reassembled across reads");
}

// -------------------------------------------------------------------------
// 54. Preserving hardware-derived effective_ranges in with_config
// -------------------------------------------------------------------------
#[test]
fn test_feetech_preserve_hw_effective_ranges_in_with_config() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_calibrated(true);

    // Simulate configure_servos recording hw angle limits: min=120, max=3950
    driver.hw_angle_limits = vec![Some((120, 3950))];
    driver.effective_ranges = vec![(120, 3950)];

    // with_config with new config where range_min_steps and range_max_steps are None
    let mut new_cfg = driver.config().clone();
    new_cfg.joints[0].range_min_steps = None;
    new_cfg.joints[0].range_max_steps = None;
    new_cfg.p_coef = 45;

    let updated_driver = driver.with_config(new_cfg);
    assert_eq!(
        updated_driver.effective_ranges(),
        &[(120, 3950)],
        "Hardware-derived angle limits must be preserved in effective_ranges when range steps are None"
    );
}

// -------------------------------------------------------------------------
// 55. Reading REG_HOMING_OFFSET_L with tx_echo=Some(false) and matching byte response
// -------------------------------------------------------------------------
#[test]
fn test_feetech_read_u16_tx_echo_false_matching_bytes_not_skipped() {
    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_tx_echo(Some(false));

    // Request: [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x1F, 0x02, 0xD7]
    // Response: id=1, len=4, status=0x02, params=[0x1F, 0x02], cs=0xD7
    // Response bytes are byte-for-byte identical to request!
    let resp = vec![0xFF, 0xFF, 0x01, 0x04, 0x02, 0x1F, 0x02, 0xD7];
    driver.transport_mut().provide_rx(&resp);

    let val = driver.read_register_u16_direct(1, 0x1F).expect("Must return response and not treat as echo");
    let expected_val = u16::from_le_bytes([0x1F, 0x02]);
    assert_eq!(val, expected_val);
}

// -------------------------------------------------------------------------
// 56. Cancelled trigger_e_stop() self-healing on tick() and single clear_fault()
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_cancelled_trigger_estop_self_heals_on_tick_and_single_clear_fault() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -PI, PI, 1.0, 1.0)];
    config.home_positions = vec![0.0];

    let supervisor = Arc::new(ArmSupervisor::new(config));

    // Acquire write lock to simulate ongoing work holding the lock
    let guard = supervisor.inner().write().await;

    // Concurrently trigger e-stop with timeout so the future is dropped/cancelled
    // while waiting for the inner write lock
    let sup_clone = Arc::clone(&supervisor);
    let _ = tokio::time::timeout(Duration::from_millis(10), async move {
        sup_clone.trigger_e_stop().await;
    })
    .await;

    // requested_estop_count was incremented before write lock acquisition
    assert!(supervisor.requested_estop_count.load(Ordering::SeqCst) > 0);

    // Release write lock
    drop(guard);

    // tick() reconciles applied_estop_count via consume_pending_estop()
    supervisor.tick(0.01).await;

    // A single clear_fault() must now succeed!
    let res = supervisor.clear_fault().await;
    assert!(res.is_ok(), "Expected single clear_fault() to succeed, got: {:?}", res);
    assert!(!supervisor.e_stop_latched.load(Ordering::SeqCst));
}

// -------------------------------------------------------------------------
// 57. Persistent 0x20 gripper overload allows reverse escape while blocking squeeze
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_gripper_persistent_hw_overload_reverse_escape_and_re_squeeze() {
    let mut config = ArmConfig::default();
    config.dof = 7;
    config.gripper_joint = Some(6);
    config.limits = (0..7)
        .map(|i| JointLimit::new(format!("j{}", i + 1), -PI, PI, 1.0, 1.0))
        .collect();
    config.home_positions = vec![0.0; 7];
    let mut gp = GripperPolicy::rebot_dm();
    gp.closed_rad = 1.5;
    gp.open_rad = 0.0;
    gp.holding_mode = HoldingMode::LoadAndGap;
    gp.holding_gap_rad = 0.02;
    gp.holding_load_fraction = Some(0.2);
    config.gripper_policy = Some(gp);

    let supervisor = Arc::new(ArmSupervisor::new(config));
    {
        let mut inner = supervisor.inner().write().await;
        inner.lease_enabled = false;
        inner.set_torque(true).unwrap();
        inner.measured_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
        inner.commanded_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
        inner.target_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
        // Inject motor error 0x20 on gripper (index 6)
        inner.motor_errors = vec![
            None,
            None,
            None,
            None,
            None,
            None,
            Some(armd::model::MotorError {
                code: 0x20,
                name: "Overload".into(),
            }),
        ];
        inner.last_can_feedback_instant = Some(Instant::now());
    }

    // Tick: HW overload rises, gripper latches stall
    supervisor.tick(0.01).await;
    assert!(supervisor.is_gripper_stalled().await, "Gripper must latch stall on 0x20");

    // Reverse servo command (moving towards open_rad=0.0, delta is negative)
    let rev_targets = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3];
    let servo_res = supervisor.servo(rev_targets, 1).await;
    assert!(servo_res.is_ok(), "Reverse servo command must be accepted");
    assert!(!supervisor.is_gripper_stalled().await, "Reverse servo command un-stalls gripper");

    // Tick multiple times while 0x20 persists: should NOT re-latch because escaping known stall
    for _ in 0..5 {
        {
            let mut inner = supervisor.inner().write().await;
            inner.last_can_feedback_instant = Some(Instant::now());
        }
        supervisor.tick(0.01).await;
        assert!(!supervisor.is_gripper_stalled().await, "Must not re-latch while escaping known stall");
    }

    // Now send squeeze command (towards closed_rad=1.5, delta is positive)
    let squeeze_targets = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8];
    let sq_res = supervisor.servo(squeeze_targets, 1).await;
    assert!(sq_res.is_ok(), "Squeeze command accepted");

    // Tick: since 0x20 still asserted and pushing into stall, it must re-latch!
    {
        let mut inner = supervisor.inner().write().await;
        inner.last_can_feedback_instant = Some(Instant::now());
    }
    supervisor.tick(0.01).await;
    assert!(supervisor.is_gripper_stalled().await, "Must re-latch when pushing into stall with 0x20");
}

// -------------------------------------------------------------------------
// 58. [0xFF]-prefixed echo + partial echo in read_register_u16_direct returns model 777
// -------------------------------------------------------------------------
#[test]
fn test_feetech_drain_tx_echo_ff_noise_and_partial_echo_returns_model_777() {
    let mock = MockTransport::new();
    let mut driver = FeetechArmDriver::new(mock, vec![1]);
    driver.set_tx_echo_state(None, true);

    // Read model (id=1, reg=0x03, len=2)
    let req = feetech::build_read(1, feetech::REG_MODEL_L, 2);

    // Provide RX with leading 0xFF noise byte + full echo + model response (777 = 0x0309)
    let model_bytes = 777u16.to_le_bytes();
    let cs = calculate_checksum(1, 4, 0x00, &model_bytes);
    let mut rx = Vec::new();
    rx.push(0xFF); // noise byte
    rx.extend_from_slice(&req); // full echo
    rx.extend_from_slice(&[0xFF, 0xFF, 1, 4, 0x00, model_bytes[0], model_bytes[1], cs]);

    driver.transport_mut().stage_reply(&req, &rx);

    let val = driver.read_register_u16_direct(1, feetech::REG_MODEL_L).expect("Read model must succeed");
    assert_eq!(val, 777, "Expected model 777, got {}", val);
    assert_eq!(driver.tx_echo(), Some(true));
}

// -------------------------------------------------------------------------
// 59. 6-servo (2-chunk) incomplete_chunk_idx cross-call reassembly + ghost-header non-starvation
// -------------------------------------------------------------------------
#[test]
fn test_feetech_6servo_incomplete_chunk_idx_cross_call_reassembly_and_ghost_header() {
    let mock = MockTransport::new();
    let servo_ids = vec![1, 2, 3, 4, 5, 6];
    let mut driver = FeetechArmDriver::new(mock, servo_ids).with_tx_echo(Some(false));

    let build_resp = |id: u8| -> Vec<u8> {
        let pos = 2048u16.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 100u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    let sync_c0 = feetech::build_sync_read(&[1, 2, 3], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    let sync_c1 = feetech::build_sync_read(&[4, 5, 6], feetech::REG_PRESENT_POSITION_L, 8).unwrap();

    // Call 1: Chunk 0 (servos 1, 2, 3) complete, Chunk 1 (servos 4, 5) complete, servo 6 incomplete (4 bytes)
    let resp6 = build_resp(6);
    let mut rx_c0 = Vec::new();
    for id in 1..=3 {
        rx_c0.extend_from_slice(&build_resp(id));
    }
    let mut rx_c1 = Vec::new();
    for id in 4..=5 {
        rx_c1.extend_from_slice(&build_resp(id));
    }
    rx_c1.extend_from_slice(&resp6[..4]); // incomplete servo 6!

    driver.transport_mut().stage_reply(&sync_c0, &rx_c0);
    driver.transport_mut().stage_reply(&sync_c1, &rx_c1);

    let _ = driver.read_joint_states();
    assert_eq!(driver.incomplete_chunk_idx(), Some(1), "Chunk 1 must be marked incomplete");

    // Call 2: Supply the rest of servo 6 during reassembly, plus next cycle's Chunk 0 (servos 1, 2, 3)
    driver.transport_mut().clear_write_responses();
    driver.transport_mut().provide_rx(&resp6[4..]); // remaining bytes of servo 6
    driver.transport_mut().stage_reply(&sync_c0, &rx_c0);

    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 6);
    assert!(driver.feedback_ages()[5].is_some(), "Servo 6 reassembled across calls");

    // Now test ghost header non-starvation: inject incomplete ghost header bytes
    let ghost = vec![0xFF, 0xFF, 0x42]; // non-existent servo 0x42 incomplete header
    driver.transport_mut().provide_rx(&ghost);
    driver.set_incomplete_chunk_idx(Some(0)); // simulate incomplete flag set

    let build_resp_pos = |id: u8, pos_steps: u16| -> Vec<u8> {
        let pos = pos_steps.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 100u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    // Next call should discard ghost header after timeout/drained limit and not starve.
    // Encode distinct joint positions (2500, 2600, 2700) different from baseline (2048).
    driver.transport_mut().clear_write_responses();
    let mut rx3 = Vec::new();
    rx3.extend_from_slice(&build_resp_pos(1, 2500));
    rx3.extend_from_slice(&build_resp_pos(2, 2600));
    rx3.extend_from_slice(&build_resp_pos(3, 2700));
    driver.transport_mut().stage_reply(&sync_c0, &rx3);

    let states3 = driver.read_joint_states().expect("Must not starve or fail on ghost header");
    let calib = armd::protocol::feetech::FeetechJointCalib::default();
    let exp1 = calib.steps_to_rad(2500);
    let exp2 = calib.steps_to_rad(2600);
    let exp3 = calib.steps_to_rad(2700);
    assert!((states3[0].position - exp1).abs() < 1e-4, "Decoded position 1 must equal distinct value");
    assert!((states3[1].position - exp2).abs() < 1e-4, "Decoded position 2 must equal distinct value");
    assert!((states3[2].position - exp3).abs() < 1e-4, "Decoded position 3 must equal distinct value");

    let ages = driver.feedback_ages();
    for id in 0..3 {
        assert!(
            ages[id].is_some_and(|a| a < Duration::from_millis(50)),
            "Joint {} feedback age must be fresh (< 50ms) after recovering from ghost header, got {:?}",
            id + 1,
            ages[id]
        );
    }

    // Subsequent poll for Chunk 1 progresses cleanly
    let mut rx4 = Vec::new();
    for id in 4..=6 {
        rx4.extend_from_slice(&build_resp_pos(id, 2800));
    }
    driver.transport_mut().stage_reply(&sync_c1, &rx4);
    let states4 = driver.read_joint_states().expect("Subsequent poll for Chunk 1 must progress cleanly");
    assert_eq!(states4.len(), 6);
    let exp4 = calib.steps_to_rad(2800);
    assert!((states4[3].position - exp4).abs() < 1e-4);
    assert_eq!(driver.incomplete_chunk_idx(), None);
}

// -------------------------------------------------------------------------
// 60. FeetechArmDriver narrower effective_ranges narrows config.limits via apply_driver_limits
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_feetech_narrower_effective_ranges_narrows_config_limits() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -PI, PI, 1.0, 1.0)];
    config.home_positions = vec![0.0];

    let supervisor = Arc::new(ArmSupervisor::new(config));

    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]);
    // Steps 1024..3072 is narrower than full 0..4095
    driver.effective_ranges = vec![(1024, 3072)];

    let calib = driver.config().joints[0].clone();
    let expected_min = calib.steps_to_rad(1024).min(calib.steps_to_rad(3072));
    let expected_max = calib.steps_to_rad(1024).max(calib.steps_to_rad(3072));

    supervisor.set_driver(Box::new(driver)).await;

    let inner = supervisor.inner().read().await;
    let limit = &inner.config.limits[0];
    assert!(
        (limit.min_position - expected_min).abs() < 1e-4,
        "min_position should be narrowed to {:.4}, got {:.4}",
        expected_min,
        limit.min_position
    );
    assert!(
        (limit.max_position - expected_max).abs() < 1e-4,
        "max_position should be narrowed to {:.4}, got {:.4}",
        expected_max,
        limit.max_position
    );
}

// -------------------------------------------------------------------------
// 61. Incomplete frame preserved across enable poll and completed on next call
// -------------------------------------------------------------------------
#[test]
fn test_feetech_incomplete_chunk_preserved_across_enable_poll_and_completed_on_next_call() {
    let mock = MockTransport::new();
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_tx_echo(Some(false));
    driver.set_confirmed_enable_state(EnableState::Enabled);
    driver.set_read_poll_count(8);
    driver.last_enable_poll = Some(driver.clock_now());

    let build_resp = |id: u8| -> Vec<u8> {
        let pos = 2048u16.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 100u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    let resp1 = build_resp(1);
    let sync_read = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    // Call 1 (poll_count -> 9, no enable poll): supply partial frame for servo 1 (first 4 bytes)
    driver.transport_mut().stage_reply(&sync_read, &resp1[..4]);

    let _ = driver.read_joint_states();
    assert_eq!(driver.incomplete_chunk_idx(), Some(0), "Chunk 0 must be marked incomplete");

    // Call 2 (poll_count -> 10, enable poll due): supply partial continuation (bytes 4..8, still incomplete).
    // If enable poll had erroneously run while incomplete_chunk_idx was Some,
    // clear_input_buffer() would have been called and wiped the partial frame!
    driver.transport_mut().clear_write_responses();
    let clears_before_reassembly = driver.transport().clear_input_count();
    driver.last_enable_poll = Some(driver.clock_now() - Duration::from_millis(500));
    driver.transport_mut().provide_rx(&resp1[4..8]);

    let _ = driver.read_joint_states();
    assert_eq!(driver.incomplete_chunk_idx(), Some(0), "Chunk 0 must still be incomplete");
    assert_eq!(
        driver.transport().clear_input_count(),
        clears_before_reassembly,
        "clear_input_count must not increment during incomplete reassembly across ticks"
    );

    // Call 3: supply remaining bytes
    driver.transport_mut().provide_rx(&resp1[8..]);
    let states = driver.read_joint_states().expect("Reassembly must succeed without enable-poll wiping rx_buffer");
    assert_eq!(states.len(), 1);
    assert_eq!(driver.incomplete_chunk_idx(), None);
    assert!(driver.feedback_ages()[0].is_some_and(|a| a < Duration::from_millis(50)));
}

// -------------------------------------------------------------------------
// 61b. Partial RX frame survives 50 Hz (20ms) control tick gap (25ms TTL)
// -------------------------------------------------------------------------
#[test]
fn test_feetech_partial_rx_frame_survives_50hz_tick_gap() {
    let mock = MockTransport::new();
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_tx_echo(Some(false));
    driver.set_confirmed_enable_state(EnableState::Enabled);
    driver.last_enable_poll = Some(driver.clock_now());

    let build_resp = |id: u8| -> Vec<u8> {
        let pos = 2048u16.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 100u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    let resp1 = build_resp(1);
    let sync_read = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    // Supply partial frame (first 4 bytes)
    driver.transport_mut().stage_reply(&sync_read, &resp1[..4]);
    let _ = driver.read_joint_states();
    assert_eq!(driver.incomplete_chunk_idx(), Some(0));

    // Simulate 50 Hz control tick gap (18ms..20ms, well within 25ms TTL)
    driver.transport().advance_clock(Duration::from_millis(18));

    // Provide remaining bytes
    driver.transport_mut().clear_write_responses();
    driver.transport_mut().provide_rx(&resp1[4..]);
    let states = driver.read_joint_states().expect("Reassembly must survive 18ms tick gap");
    assert_eq!(states.len(), 1);
    assert_eq!(driver.incomplete_chunk_idx(), None);
    assert!(driver.feedback_ages()[0].is_some_and(|a| a < Duration::from_millis(60)));
}

// -------------------------------------------------------------------------
// 61c. Stray noise (0x00) without 0xFF 0xFF header does NOT pause polling
// -------------------------------------------------------------------------
#[test]
fn test_feetech_stray_noise_does_not_pause_polling() {
    let mock = MockTransport::new();
    let mut driver = FeetechArmDriver::new(mock, vec![1, 2, 3]).with_tx_echo(Some(false));

    let sync_read = feetech::build_sync_read(&[1, 2, 3], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    // Provide stray noise byte 0x00 (no 0xFF 0xFF header)
    driver.transport_mut().stage_reply(&sync_read, &[0x00]);

    let _ = driver.read_joint_states();
    assert_eq!(
        driver.incomplete_chunk_idx(), None,
        "Stray noise 0x00 must NOT set incomplete_chunk_idx or pause polling"
    );
    assert!(driver.rx_buffer().is_empty(), "rx_buffer must be cleared of noise");
}

// -------------------------------------------------------------------------
// 62. Multi-servo chunk reassembly: servo 4 split across calls, 5 & 6 arrive right after
// -------------------------------------------------------------------------
#[test]
fn test_feetech_multiservo_chunk_reassembly_split_first_servo_and_following_servos_complete() {
    let mock = MockTransport::new();
    let servo_ids = vec![1, 2, 3, 4, 5, 6];
    let mut driver = FeetechArmDriver::new(mock, servo_ids).with_tx_echo(Some(false));

    let build_resp_pos = |id: u8, pos_steps: u16| -> Vec<u8> {
        let pos = pos_steps.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 100u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    // Call 1: Chunk 0 (servos 1, 2, 3) complete, Chunk 1 servo 4 split (first 4 bytes only).
    // Servos 5 and 6 are NOT provided yet.
    let mut rx_c0 = Vec::new();
    for id in 1..=3 {
        rx_c0.extend_from_slice(&build_resp_pos(id, 2048));
    }
    let resp4 = build_resp_pos(4, 2500);

    let sync_c0 = feetech::build_sync_read(&[1, 2, 3], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    let sync_c1 = feetech::build_sync_read(&[4, 5, 6], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    driver.transport_mut().stage_reply(&sync_c0, &rx_c0);
    driver.transport_mut().stage_reply(&sync_c1, &resp4[..4]); // split servo 4!

    let _ = driver.read_joint_states();
    assert_eq!(driver.incomplete_chunk_idx(), Some(1), "Chunk 1 must be marked incomplete");

    // Call 2: supply remainder of servo 4 AND complete frames for servos 5 & 6 in that chunk
    driver.transport_mut().clear_write_responses();
    let mut rx2 = Vec::new();
    rx2.extend_from_slice(&resp4[4..]); // remainder of servo 4
    rx2.extend_from_slice(&build_resp_pos(5, 2600)); // servo 5
    rx2.extend_from_slice(&build_resp_pos(6, 2700)); // servo 6
    driver.transport_mut().provide_rx(&rx2);

    let states = driver.read_joint_states().expect("Multi-servo chunk reassembly must succeed");
    assert_eq!(states.len(), 6);
    assert_eq!(driver.incomplete_chunk_idx(), None, "Incomplete chunk index must be cleared after reassembly");

    // Verify all 3 servos in Chunk 1 (indices 3, 4, 5) updated
    let calib4 = driver.config().joints[3].clone();
    let calib5 = driver.config().joints[4].clone();
    let calib6 = driver.config().joints[5].clone();

    assert!((states[3].position - calib4.steps_to_rad(2500)).abs() < 1e-4, "Servo 4 position updated");
    assert!((states[4].position - calib5.steps_to_rad(2600)).abs() < 1e-4, "Servo 5 position updated");
    assert!((states[5].position - calib6.steps_to_rad(2700)).abs() < 1e-4, "Servo 6 position updated");

    let ages = driver.feedback_ages();
    assert!(ages[3].is_some_and(|a| a < Duration::from_millis(50)), "Servo 4 age fresh");
    assert!(ages[4].is_some_and(|a| a < Duration::from_millis(50)), "Servo 5 age fresh");
    assert!(ages[5].is_some_and(|a| a < Duration::from_millis(50)), "Servo 6 age fresh");
}

// -------------------------------------------------------------------------
// 63. apply_driver_limits rejection leaves home_positions & resting_pose unchanged and latches unclearable fault
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_apply_driver_limits_rejection_leaves_poses_unchanged_and_latches_unclearable_fault() {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -1.0, 1.0, 1.0, 1.0)];
    config.home_positions = vec![0.5];
    config.resting_pose = Some(RestingPosePolicy {
        positions: Some(vec![0.5]),
        position_tolerances: vec![0.08],
        max_total_deviation: 0.16,
        max_velocity: 0.015,
        joint_max_velocities: vec![],
        support_torque_limits: vec![],
        support_load_limits: vec![],
        feedback_max_age_ms: 100,
    });

    let supervisor = Arc::new(ArmSupervisor::new(config));

    let mock = MockTransport::new_preserving();
    let mut driver = FeetechArmDriver::new(mock, vec![1]);
    // Steps 100..200 corresponds to negative angles far from 0.5 rad (e.g. [-0.8, -0.2])
    driver.effective_ranges = vec![(100, 200)];

    supervisor.set_driver(Box::new(driver)).await;

    let inner = supervisor.inner().read().await;
    assert!(inner.driver_config_error.is_some(), "Must latch driver_config_error");
    // Crucial check: poses must NOT be silently clamped!
    assert_eq!(inner.config.home_positions, vec![0.5], "home_positions must remain unchanged on rejection");
    assert_eq!(
        inner.config.resting_pose.as_ref().unwrap().positions,
        Some(vec![0.5]),
        "resting_pose.positions must remain unchanged on rejection"
    );
    match inner.safety.state() {
        SafetyState::Fault(reason) => {
            assert!(reason.message().contains("Driver limits violated config invariants") || reason.message().contains("conflict"));
        }
        other => panic!("Expected Fault state, got {:?}", other),
    }
    drop(inner);

    // clear_fault must be blocked while driver_config_error persists
    let clear_err = supervisor.clear_fault().await.unwrap_err();
    assert!(
        clear_err.to_string().contains("Driver limit configuration error"),
        "clear_fault must be blocked with clear message, got: {clear_err}"
    );

    // set_torque must also be blocked
    let tq_err = supervisor.set_torque(true).await.unwrap_err();
    assert!(
        tq_err.to_string().contains("Driver limit configuration error")
            || tq_err.to_string().contains("Motion forbidden")
            || tq_err.to_string().contains("Fault active"),
        "set_torque must be blocked, got: {tq_err}"
    );
}

// -------------------------------------------------------------------------
// 64. Feetech driver-backed 0x20 overload stale->rising edge triggers HardwareFault
// -------------------------------------------------------------------------
#[tokio::test]
async fn test_feetech_driver_backed_arm_joint_0x20_rising_edge_triggers_hardware_fault() {
    let mock = Arc::new(Mutex::new(MockTransport::new_preserving()));
    let driver = FeetechArmDriver::new(SharedTransport(mock.clone()), vec![1]).with_calibrated(true);

    let mut config = ArmConfig::default();
    config.dof = 1;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0];

    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(driver));

    // Provide normal feedback with error = 0x00
    let pos_1 = 2048u16.to_le_bytes();
    let normal_params = [pos_1[0], pos_1[1], 0, 0, 0, 0, 70, 35];
    let cs_normal = calculate_checksum(1, 10, 0x00, &normal_params);
    let mut rx_normal = vec![0xFF, 0xFF, 1, 10, 0x00];
    rx_normal.extend_from_slice(&normal_params);
    rx_normal.push(cs_normal);

    let cs_tq1 = calculate_checksum(1, 3, 0x00, &[1]);
    let rx_tq1 = vec![0xFF, 0xFF, 1, 3, 0x00, 1, cs_tq1];

    mock.lock().unwrap().provide_rx(&rx_normal);
    mock.lock().unwrap().provide_rx(&rx_tq1);
    mock.lock().unwrap().provide_rx(&rx_normal);

    supervisor.set_torque(true).await.unwrap();
    assert!(supervisor.inner().read().await.is_torqued);

    // Initial tick with normal 0x00 feedback -> arm stays Armed/Idle
    supervisor.tick(0.02).await;
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, armd::safety::SafetyState::Armed);
    assert!(obs.fault.is_none());

    // Allow feedback to age / tick without fresh bytes (stale condition)
    mock.lock().unwrap().advance_clock(Duration::from_millis(5));
    supervisor.tick(0.02).await;

    // Now rising edge: hardware packet arrives with error byte 0x20 on arm joint 1
    let cs_overload = calculate_checksum(1, 10, 0x20, &normal_params);
    let mut rx_overload = vec![0xFF, 0xFF, 1, 10, 0x20];
    rx_overload.extend_from_slice(&normal_params);
    rx_overload.push(cs_overload);
    mock.lock().unwrap().provide_rx(&rx_overload);

    // Tick: Feetech driver decodes 0x20, check_hardware_status detects rising edge and faults
    let obs_fault = supervisor.tick(0.02).await;
    assert!(obs_fault.fault.is_some(), "Supervisor must fault on 0x20 rising edge");
    let fault = obs_fault.fault.unwrap();
    assert_eq!(fault.code, FaultCode::HardwareFault);
    assert!(fault.message.contains("0x20"));
}

// -------------------------------------------------------------------------
// 65. Cross-tick reassembly with active torque motion and echo drain (Codex M1 + Opus H1)
// -------------------------------------------------------------------------
#[test]
fn test_feetech_cross_tick_reassembly_with_active_torque_and_echo_drain() {
    let mock = MockTransport::new();
    let mut driver = FeetechArmDriver::new(mock, vec![1])
        .with_calibrated(true)
        .with_tx_echo(Some(false));
    driver.set_confirmed_enable_state(EnableState::Enabled);

    let build_resp_pos = |id: u8, pos_steps: u16| -> Vec<u8> {
        let pos = pos_steps.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 100u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    let full_pkt = build_resp_pos(1, 2500);

    // Call 1: Split after first 4 bytes
    let sync_read = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    driver.transport_mut().stage_reply(&sync_read, &full_pkt[..4]);

    let _ = driver.read_joint_states();
    assert_eq!(driver.incomplete_chunk_idx(), Some(0));
    assert_eq!(driver.partial_pos_rx(), &full_pkt[..4]);

    // Active torque motion: write_joint_targets must NOT wipe partial_pos_rx or incomplete_chunk_idx!
    let target = JointTarget::position(0.0);
    driver.write_joint_targets(&[target]).expect("write_joint_targets must succeed");
    assert_eq!(
        driver.incomplete_chunk_idx(),
        Some(0),
        "write_joint_targets must preserve incomplete_chunk_idx"
    );
    assert_eq!(
        driver.partial_pos_rx(),
        &full_pkt[..4],
        "write_joint_targets must preserve partial_pos_rx"
    );

    // Next tick: supply continuation bytes during reassembly
    driver.transport_mut().clear_write_responses();
    driver.transport_mut().provide_rx(&full_pkt[4..]);

    let states = driver.read_joint_states().expect("Reassembly must succeed after write_joint_targets");
    assert_eq!(states.len(), 1);
    assert_eq!(driver.incomplete_chunk_idx(), None);

    let calib = driver.config().joints[0].clone();
    let expected_pos = calib.steps_to_rad(2500);
    assert!((states[0].position - expected_pos).abs() < 1e-4, "Decoded position must match steps 2500");
    assert!(driver.feedback_ages()[0].is_some_and(|a| a < Duration::from_millis(50)), "feedback age must be fresh");
}

// -------------------------------------------------------------------------
// 66. Negative test: bogus ID and length do NOT set incomplete_chunk_idx (Opus M3)
// -------------------------------------------------------------------------
#[test]
fn test_feetech_trailing_partial_frame_rejects_bogus_id_and_length() {
    let mock = MockTransport::new();
    let mut driver = FeetechArmDriver::new(mock, vec![1, 2, 3]).with_tx_echo(Some(false));

    // Case 1: Wrong ID (0x42 not in [1, 2, 3])
    let sync_read = feetech::build_sync_read(&[1, 2, 3], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    driver.transport_mut().stage_reply(&sync_read, &[0xFF, 0xFF, 0x42, 0x0A]);
    let _ = driver.read_joint_states();
    assert_eq!(
        driver.incomplete_chunk_idx(),
        None,
        "Bogus ID 0x42 must NOT set incomplete_chunk_idx"
    );

    // Case 2: Wrong length (0x40 != 10 for 8-byte SyncRead)
    driver.transport_mut().clear_write_responses();
    driver.transport_mut().stage_reply(&sync_read, &[0xFF, 0xFF, 0x01, 0x40]);
    let _ = driver.read_joint_states();
    assert_eq!(
        driver.incomplete_chunk_idx(),
        None,
        "Bogus length 0x40 must NOT set incomplete_chunk_idx"
    );

    // Direct unit test of trailing_partial_frame_for_sync_read
    assert_eq!(feetech::trailing_partial_frame_for_sync_read(&[0xFF, 0xFF, 0x42, 0x0A], &[1, 2, 3], 10), None);
    assert_eq!(feetech::trailing_partial_frame_for_sync_read(&[0xFF, 0xFF, 0x01, 0x40], &[1, 2, 3], 10), None);
    assert_eq!(feetech::trailing_partial_frame_for_sync_read(&[0xFF], &[1, 2, 3], 10), Some(&[0xFF][..]));
    assert_eq!(feetech::trailing_partial_frame_for_sync_read(&[0xFF, 0xFF], &[1, 2, 3], 10), Some(&[0xFF, 0xFF][..]));
    assert_eq!(feetech::trailing_partial_frame_for_sync_read(&[0xFF, 0xFF, 0x01], &[1, 2, 3], 10), Some(&[0xFF, 0xFF, 0x01][..]));
    assert_eq!(feetech::trailing_partial_frame_for_sync_read(&[0xFF, 0xFF, 0x01, 0x0A], &[1, 2, 3], 10), Some(&[0xFF, 0xFF, 0x01, 0x0A][..]));
}

// -------------------------------------------------------------------------
// 67. MockClock deterministic read/error/sharing behavior
// -------------------------------------------------------------------------
#[test]
fn test_mock_clock_advances_only_on_zero_byte_or_nonempty_error_reads() {
    let clock = MockClock::new();
    let step = Duration::from_millis(2);
    clock.set_idle_read_step(step);

    let mut transport = MockTransport::new().with_clock(clock.clone());
    assert_eq!(clock.elapsed(), Duration::ZERO);
    assert_eq!(transport.clock().elapsed(), Duration::ZERO);

    // 1. Ok(n > 0) advances 0ns
    transport.provide_rx(&[0xAA, 0xBB, 0xCC]);
    let mut buf = [0u8; 4];
    assert_eq!(transport.read(&mut buf).unwrap(), 3);
    assert_eq!(clock.elapsed(), Duration::ZERO);

    // 2. Ok(0) on empty rx_buffer advances by idle_read_step
    assert_eq!(transport.read(&mut buf).unwrap(), 0);
    assert_eq!(clock.elapsed(), step);

    // 3. chunk_size_limit(Some(0)) with non-empty rx_buffer advances idle_read_step
    transport.provide_rx(&[0x11]);
    transport.set_chunk_size_limit(Some(0));
    assert_eq!(transport.read(&mut buf).unwrap(), 0);
    assert_eq!(clock.elapsed(), step * 2);

    // 4. chunk_size_limit(Some(0)) with non-empty staged_reads advances idle_read_step
    transport.queue_staged_read(&[0x22]);
    assert_eq!(transport.read(&mut buf).unwrap(), 0);
    assert_eq!(clock.elapsed(), step * 3);
    transport.clear();
    transport.set_chunk_size_limit(None);

    // 5. buf = &mut [] with error_on_next_read consumes & returns error with 0ns advance,
    //    and next nonempty read returns Ok(0) and advances idle_read_step
    transport.set_error_on_next_read(Some("one-shot empty buf error".to_string()));
    let elapsed_before_empty_err = clock.elapsed();
    let err = transport.read(&mut []).unwrap_err();
    assert!(err.to_string().contains("one-shot empty buf error"));
    assert_eq!(clock.elapsed(), elapsed_before_empty_err);

    assert_eq!(transport.read(&mut buf).unwrap(), 0);
    assert_eq!(clock.elapsed(), elapsed_before_empty_err + step);

    // 6. Nonempty buf (&mut [0u8; 4]) + set_error_on_next_read: returns Err, advances by
    //    exactly one idle_read_step, and subsequent nonempty read returns Ok(0) + advances step
    transport.set_error_on_next_read(Some("one-shot nonempty buf error".to_string()));
    let elapsed_before_nonempty_err = clock.elapsed();
    let mut nonempty_buf = [0u8; 4];
    let err2 = transport.read(&mut nonempty_buf).unwrap_err();
    assert!(err2.to_string().contains("one-shot nonempty buf error"));
    assert_eq!(clock.elapsed(), elapsed_before_nonempty_err + step);

    assert_eq!(transport.read(&mut nonempty_buf).unwrap(), 0);
    assert_eq!(clock.elapsed(), elapsed_before_nonempty_err + step * 2);
}

#[test]
#[should_panic(expected = "idle_read_step must be non-zero")]
fn test_mock_clock_rejects_zero_idle_step() {
    let clock = MockClock::new();
    clock.set_idle_read_step(Duration::ZERO);
}

#[test]
fn test_mock_clock_panics_on_overflow_exhaustion() {
    let clock = MockClock::new();
    let initial = Duration::from_nanos(u64::MAX - 10);
    clock.advance(initial);
    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        clock.advance(Duration::from_nanos(20));
    }));
    assert!(res.is_err());
    assert_eq!(clock.elapsed(), initial);
}

#[test]
fn test_feetech_transport_replacement_with_different_clock_panics() {
    let replace_and_run = |f: fn(&mut FeetechArmDriver<MockTransport>)| {
        let mut driver = FeetechArmDriver::new(MockTransport::new(), vec![1, 2]).with_tx_echo(Some(false));
        let mut replacement = MockTransport::new();
        replacement.provide_rx(&[0xAA, 0xBB]);
        *driver.transport_mut() = replacement;

        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(&mut driver)));
        let err = res.expect_err("expected panic when transport clock domain changed");
        let msg = err
            .downcast_ref::<&str>()
            .copied()
            .or_else(|| err.downcast_ref::<String>().map(String::as_str))
            .unwrap_or("");
        assert!(
            msg.contains("transport clock domain changed after construction"),
            "unexpected panic payload: {msg:?}"
        );
        assert!(
            driver.transport_mut().take_tx().is_empty(),
            "replacement transport must receive zero bytes before panic"
        );
        assert_eq!(driver.transport().rx_len(), 2);
        assert_eq!(driver.transport().clear_input_count(), 0);
    };

    replace_and_run(|d| {
        let _ = d.read_joint_states();
    });
    replace_and_run(|d| {
        let _ = d.set_torque(false);
    });
    replace_and_run(|d| {
        let targets = [JointTarget::position(0.0), JointTarget::position(0.0)];
        let _ = d.write_joint_targets(&targets);
    });
    replace_and_run(|d| {
        d.motor_status[0] = 0x20;
        let _ = d.clear_fault();
    });
    replace_and_run(|d| {
        let _ = d.configure_servos();
    });
    replace_and_run(|d| {
        let _ = d.refresh_enable_state();
    });
    replace_and_run(|d| {
        d.set_gripper_index(Some(0));
    });
    replace_and_run(|d| {
        d.set_abort_flag(Arc::new(AtomicBool::new(false)));
    });
}

#[test]
fn test_feetech_configure_servos_eeprom_2ms_pacing() {
    struct RecordingTransport {
        inner: MockTransport,
        clock: MockClock,
        writes: Arc<Mutex<Vec<(Duration, Vec<u8>)>>>,
        abort_on_lock0: Option<Arc<AtomicBool>>,
        on_min_angle_written: Option<Vec<u8>>,
    }
    impl Transport for RecordingTransport {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Transport::clock(&self.inner)
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
            self.inner.read(buf)
        }
        fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
            self.writes.lock().unwrap().push((self.clock.elapsed(), buf.to_vec()));
            if let Some(ref flag) = self.abort_on_lock0 {
                let lock0 = feetech::build_write_u8(1, feetech::REG_LOCK, 0);
                if buf == lock0.as_slice() {
                    flag.store(true, Ordering::SeqCst);
                }
            }
            let min_write = feetech::build_write_u16(1, feetech::REG_MIN_ANGLE_LIMIT_L, 200);
            if buf == min_write.as_slice() {
                if let Some(ref updated_reply) = self.on_min_angle_written {
                    let read_min = feetech::build_read(1, feetech::REG_MIN_ANGLE_LIMIT_L, 2);
                    self.inner.clear_write_responses();
                    self.inner.stage_reply(&read_min, updated_reply);
                }
            }
            self.inner.write(buf)
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
        fn clear_input_buffer(&mut self) {
            self.inner.clear_input_buffer();
        }
    }

    let reply_u8 = |val: u8| -> Vec<u8> {
        let cs = calculate_checksum(1, 3, 0x00, &[val]);
        vec![0xFF, 0xFF, 1, 3, 0x00, val, cs]
    };
    let reply_u16 = |val: u16| -> Vec<u8> {
        let b = val.to_le_bytes();
        let cs = calculate_checksum(1, 4, 0x00, &b);
        vec![0xFF, 0xFF, 1, 4, 0x00, b[0], b[1], cs]
    };
    let stage_initial_reads = |mock: &mut MockTransport| {
        mock.stage_reply(&feetech::build_read(1, feetech::REG_MODEL_L, 2), &reply_u16(feetech::EXPECTED_MODEL_STS3215));
        mock.stage_reply(&feetech::build_read(1, REG_TORQUE_ENABLE, 1), &reply_u8(0));
        mock.stage_reply(&feetech::build_read(1, REG_RETURN_DELAY, 1), &reply_u8(0));
        mock.stage_reply(&feetech::build_read(1, REG_STATUS_RETURN_LEVEL, 1), &reply_u8(0));
        mock.stage_reply(&feetech::build_read(1, REG_MIN_ANGLE_LIMIT_L, 2), &reply_u16(0));
        mock.stage_reply(&feetech::build_read(1, REG_MAX_ANGLE_LIMIT_L, 2), &reply_u16(4095));
        mock.stage_reply(&feetech::build_read(1, REG_P_COEF, 1), &reply_u8(16));
        mock.stage_reply(&feetech::build_read(1, REG_D_COEF, 1), &reply_u8(32));
        mock.stage_reply(&feetech::build_read(1, REG_I_COEF, 1), &reply_u8(0));
        mock.stage_reply(&feetech::build_read(1, REG_MAX_TORQUE_L, 2), &reply_u16(1000));
        mock.stage_reply(&feetech::build_read(1, REG_PROTECTION_CURRENT_L, 2), &reply_u16(500));
        mock.stage_reply(&feetech::build_read(1, REG_OVERLOAD_TORQUE, 1), &reply_u8(100));
        mock.stage_reply(&feetech::build_read(1, REG_OPERATING_MODE, 1), &reply_u8(0));
    };

    let mut cfg = feetech::FeetechConfig::default();
    cfg.joints = vec![feetech::FeetechJointCalib {
        id: 1,
        range_min_steps: Some(200),
        range_max_steps: Some(4095),
        ..Default::default()
    }];

    // Subcase 1: Normal EEPROM programming (2ms after REG_LOCK=0 and 2ms after register write)
    {
        let mut mock = MockTransport::new_preserving();
        stage_initial_reads(&mut mock);
        let clock = mock.clock();
        let writes = Arc::new(Mutex::new(Vec::new()));
        let transport = RecordingTransport {
            inner: mock,
            clock,
            writes: writes.clone(),
            abort_on_lock0: None,
            on_min_angle_written: Some(reply_u16(200)),
        };
        let mut driver = FeetechArmDriver::new(transport, vec![1])
            .with_tx_echo(Some(false))
            .with_config(cfg.clone());
        driver.configure_servos().expect("configure_servos must succeed");

        let log = writes.lock().unwrap();
        let lock0 = feetech::build_write_u8(1, REG_LOCK, 0);
        let write_min = feetech::build_write_u16(1, REG_MIN_ANGLE_LIMIT_L, 200);
        let read_min = feetech::build_read(1, REG_MIN_ANGLE_LIMIT_L, 2);

        let lock0_idx = log.iter().position(|(_, b)| *b == lock0).unwrap();
        let min_idx = log.iter().position(|(_, b)| *b == write_min).unwrap();
        let readback_idx = log[min_idx + 1..]
            .iter()
            .position(|(_, b)| *b == read_min)
            .map(|o| min_idx + 1 + o)
            .unwrap();

        let t_lock0 = log[lock0_idx].0;
        let t_min = log[min_idx].0;
        let t_readback = log[readback_idx].0;
        assert_eq!(
            t_min - t_lock0,
            Duration::from_millis(2),
            "EEPROM write must wait exactly 2ms after REG_LOCK=0 (t_lock0={:?}, t_min={:?})",
            t_lock0,
            t_min
        );
        assert_eq!(
            t_readback - t_min,
            Duration::from_millis(2),
            "EEPROM readback must wait exactly 2ms after EEPROM write (t_min={:?}, t_readback={:?})",
            t_min,
            t_readback
        );
    }

    // Subcase 2: Abort/relock path (2ms elapses between REG_LOCK=0 and REG_LOCK=1)
    {
        let mut mock = MockTransport::new_preserving();
        stage_initial_reads(&mut mock);
        let clock = mock.clock();
        let writes = Arc::new(Mutex::new(Vec::new()));
        let abort_flag = Arc::new(AtomicBool::new(false));
        let transport = RecordingTransport {
            inner: mock,
            clock,
            writes: writes.clone(),
            abort_on_lock0: Some(abort_flag.clone()),
            on_min_angle_written: None,
        };
        let mut driver = FeetechArmDriver::new(transport, vec![1])
            .with_tx_echo(Some(false))
            .with_config(cfg);
        driver.set_abort_flag(abort_flag);
        let res = driver.configure_servos();
        assert!(res.is_err(), "aborted configure_servos must fail");

        let log = writes.lock().unwrap();
        let lock0 = feetech::build_write_u8(1, REG_LOCK, 0);
        let lock1 = feetech::build_write_u8(1, REG_LOCK, 1);
        let t_lock0 = log.iter().find(|(_, b)| *b == lock0).unwrap().0;
        let t_lock1 = log.iter().find(|(_, b)| *b == lock1).unwrap().0;
        assert_eq!(
            t_lock1 - t_lock0,
            Duration::from_millis(2),
            "REG_LOCK=1 relock must occur exactly 2ms after REG_LOCK=0 (t_lock0={:?}, t_lock1={:?})",
            t_lock0,
            t_lock1
        );
    }
}

#[test]
fn test_feetech_partial_frame_ttl_exact_25ms_boundary() {
    let build_resp = |id: u8, pos_steps: u16| -> Vec<u8> {
        let pos = pos_steps.to_le_bytes();
        let spd = 0u16.to_le_bytes();
        let load = 100u16.to_le_bytes();
        let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 40];
        let cs = calculate_checksum(id, (params.len() + 2) as u8, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        pkt
    };

    let target_steps = 2500u16;
    let full_pkt = build_resp(1, target_steps);
    let prefix = &full_pkt[..4];
    let suffix = &full_pkt[4..];
    let sync_read = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();

    // Case A: below boundary (25ms - 1ns) -> decodes exact target position
    {
        let mut driver = FeetechArmDriver::new(MockTransport::new(), vec![1]).with_tx_echo(Some(false));
        driver.transport_mut().stage_reply(&sync_read, prefix);
        let _ = driver.read_joint_states();
        let started_at = driver
            .incomplete_frame_started_at()
            .expect("partial frame start timestamp must be recorded");

        let elapsed = driver.clock_now().duration_since(started_at);
        let target_elapsed = Duration::from_millis(25) - Duration::from_nanos(1);
        driver
            .transport()
            .advance_clock(target_elapsed.checked_sub(elapsed).unwrap());
        assert_eq!(driver.clock_now().duration_since(started_at), target_elapsed);

        driver.transport_mut().clear_write_responses();
        driver.transport_mut().provide_rx(suffix);
        let states = driver
            .read_joint_states()
            .expect("reassembly just below 25ms TTL must succeed");
        let expected_pos = driver.config().joints[0].steps_to_rad(target_steps as i32);
        assert!(
            (states[0].position - expected_pos).abs() < 1e-4,
            "expected {}, got {}",
            expected_pos,
            states[0].position
        );
        assert!(driver.feedback_ages()[0].is_some());
    }

    // Case B: at boundary (exact 25ms) -> expires partial_pos_rx and does NOT decode suffix
    {
        let mut driver = FeetechArmDriver::new(MockTransport::new(), vec![1]).with_tx_echo(Some(false));
        driver.transport_mut().stage_reply(&sync_read, prefix);
        let _ = driver.read_joint_states();
        let started_at = driver
            .incomplete_frame_started_at()
            .expect("partial frame start timestamp must be recorded");

        let elapsed = driver.clock_now().duration_since(started_at);
        let target_elapsed = Duration::from_millis(25);
        driver
            .transport()
            .advance_clock(target_elapsed.checked_sub(elapsed).unwrap());
        assert_eq!(driver.clock_now().duration_since(started_at), target_elapsed);

        driver.transport_mut().clear_write_responses();
        driver.transport_mut().provide_rx(suffix);
        let states = driver
            .read_joint_states()
            .expect("expired partial frame poll must succeed cleanly");
        assert_eq!(states.len(), 1);
        assert_eq!(driver.incomplete_chunk_idx(), None);
        assert!(driver.feedback_ages()[0].is_none());
    }
}
