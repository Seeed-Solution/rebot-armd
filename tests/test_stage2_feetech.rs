use std::f32::consts::PI;
use std::time::Duration;

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
use armd::supervisor::ArmSupervisor;
use armd::transport::{MockTransport, SerialTransportConfig};

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
    let mut driver = FeetechArmDriver::new(mock, servo_ids).with_tx_echo(Some(true));

    // Prepare response: TX packet echoed first, followed by RX response
    let pkt_write = feetech::build_set_torque(1, true);
    let params = [1u8];
    let len = (params.len() + 2) as u8;
    let cs = calculate_checksum(1, len, 0x00, &params);
    let pkt_read_resp = vec![0xFF, 0xFF, 1, len, 0x00, 1, cs];

    let mut stream = Vec::new();
    // 1. Echo of write command
    stream.extend_from_slice(&pkt_write);
    // 2. Read command will be sent by set_torque -> echo of read command
    let pkt_read_cmd = feetech::build_read(1, REG_TORQUE_ENABLE, 1);
    stream.extend_from_slice(&pkt_read_cmd);
    // 3. Real response to read command
    stream.extend_from_slice(&pkt_read_resp);

    driver.transport_mut().provide_rx(&stream);
    driver.set_torque(true).unwrap();
    assert_eq!(driver.enable_state(), EnableState::Enabled);
}

// -------------------------------------------------------------------------
// 5. Readback-verified set_torque enable, rollback on failure, and disable
// -------------------------------------------------------------------------
#[test]
fn test_feetech_torque_enable_rollback() {
    let mock = MockTransport::new();
    let servo_ids = vec![1, 2];
    let mut driver = FeetechArmDriver::new(mock, servo_ids);

    // Servo 1 reports 1 (enabled), Servo 2 fails (reports 0)
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
    let err = res.unwrap_err();
    assert!(matches!(err, ArmDriverError::HardwareFault(_)));

    // Verify rollback: driver wrote disable packets (REG_TORQUE_ENABLE = 0)
    let tx = driver.transport_mut().take_tx();
    // Enable writes (2*8=16) + Readback reads (2*8=16) + Rollback writes (2*8=16) = 48 bytes
    assert_eq!(tx.len(), 48);
    // Rollback write to servo 1 at offset 32: [0xFF, 0xFF, 1, 4, 3, 0x28, 0, cs]
    assert_eq!(tx[32 + 2], 1); // ID 1
    assert_eq!(tx[32 + 4], INST_WRITE);
    assert_eq!(tx[32 + 5], REG_TORQUE_ENABLE);
    assert_eq!(tx[32 + 6], 0); // Torque disable
}

// -------------------------------------------------------------------------
// 6. SYNC_READ 3-servo chunking for 6-DOF arm
// -------------------------------------------------------------------------
#[test]
fn test_feetech_sync_read_chunking_6dof() {
    let mock = MockTransport::new();
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
    let mock = MockTransport::new();
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
        (REG_STATUS_RETURN_LEVEL, 1),
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
    let mock = MockTransport::new();
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids);

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
    let mock = MockTransport::new();
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids);

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
        (REG_STATUS_RETURN_LEVEL, 1),
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

    let tx = driver.transport_mut().take_tx();
    for window in tx.windows(8) {
        if window[0] == 0xFF && window[1] == 0xFF && window[4] == INST_WRITE {
            panic!("no write commands allowed when registers already match");
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
    let mock = MockTransport::new();
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
    let mock = MockTransport::new();
    let servo_ids = vec![1];
    let mut driver = FeetechArmDriver::new(mock, servo_ids);

    let pos = 2048u16.to_le_bytes();
    let spd = 0u16.to_le_bytes();
    let load = 150u16.to_le_bytes();
    let params = [pos[0], pos[1], spd[0], spd[1], load[0], load[1], 75, 42];
    let cs = calculate_checksum(1, (params.len() + 2) as u8, 0x00, &params);
    let mut full_pkt = vec![0xFF, 0xFF, 1, (params.len() + 2) as u8, 0x00];
    full_pkt.extend_from_slice(&params);
    full_pkt.push(cs);

    // Provide fragmented bytes: first 3 bytes, then 2 bytes, then the rest
    driver.transport_mut().provide_rx(&full_pkt[..3]);
    let _ = driver.read_joint_states();
    assert!(driver.feedback_ages()[0].is_none(), "Incomplete header cannot yield state");

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

// -------------------------------------------------------------------------
// 14. with_config updates servo_ids used in bus operations
// -------------------------------------------------------------------------
#[test]
fn test_feetech_with_config_updates_servo_ids_in_bus_ops() {
    let mock = MockTransport::new();
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
    let mock = MockTransport::new();
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
    let mock = MockTransport::new();
    let mut driver = FeetechArmDriver::new(mock, vec![1]).with_tx_echo(Some(false));

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
    let mock = MockTransport::new();
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
    let driver2 = FeetechArmDriver::new(MockTransport::new(), vec![1])
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
    let mock = MockTransport::new();
    let mut driver = FeetechArmDriver::new(mock, vec![1, 2]);

    // 1. Initially without feedback: set_torque(true) should NOT send SYNC_WRITE
    let cs1 = calculate_checksum(1, 3, 0x00, &[1]);
    let cs2 = calculate_checksum(2, 3, 0x00, &[1]);
    driver.transport_mut().provide_rx(&[
        0xFF, 0xFF, 1, 3, 0x00, 1, cs1,
        0xFF, 0xFF, 2, 3, 0x00, 1, cs2,
    ]);
    driver.set_torque(true).unwrap();
    let tx_no_fb = driver.transport_mut().take_tx();
    // Two write packets (TORQUE_ENABLE = 1) + two read packets (readback REG_TORQUE_ENABLE)
    assert!(!tx_no_fb.windows(2).any(|w| w == [INST_SYNC_READ, feetech::REG_GOAL_POSITION_L]
        || w == [feetech::INST_SYNC_WRITE, feetech::REG_GOAL_POSITION_L]),
        "Without feedback, set_torque should not send SYNC_WRITE to Goal Position");

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
