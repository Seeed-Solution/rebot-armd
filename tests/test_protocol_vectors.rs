use std::f32::consts::PI;
use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, FashionStarArmDriver, FeetechArmDriver,
    JointTarget, RobStrideArmDriver,
};
use armd::protocol::damiao::{
    self, DamiaoLimits, CMD_DISABLE, CMD_ENABLE,
};
use armd::protocol::fashionstar::{
    self, CMD_PING, CMD_QUERY_SERVO_ANGLE, CMD_SET_SERVO_ANGLE, CMD_SET_TORQUE,
    FASHIONSTAR_HEADER, FASHIONSTAR_REQUEST_HEADER, FASHIONSTAR_RESPONSE_HEADER,
};
use armd::protocol::feetech::{
    self, FEETECH_HEADER, INST_PING, INST_READ, INST_SYNC_WRITE, INST_WRITE, REG_GOAL_POSITION,
    REG_PRESENT_POSITION, REG_TORQUE_ENABLE,
};
use armd::protocol::robstride::{
    self, RobStrideLimits, TYPE_ENABLE, TYPE_MOTION_CONTROL, TYPE_STOP,
};
use armd::transport::{
    CanFrame, CanTransport, MockCanTransport, MockTransport, SlcanCodec, SlcanTransport,
};

// =========================================================================
// 1. Feetech STS3215 Protocol Golden Vectors & Driver Tests
// =========================================================================

#[test]
fn test_feetech_checksum_and_packet_vectors() {
    // Vector 1: Ping packet for Servo ID 1
    // ID=1, Length=2, Instruction=0x01 (PING), Params=[]
    // Sum = 1 + 2 + 1 = 4. Checksum = !(4) & 0xFF = 0xFB
    let ping_packet = feetech::build_ping(1);
    let expected_ping = vec![0xFF, 0xFF, 0x01, 0x02, INST_PING, 0xFB];
    assert_eq!(ping_packet, expected_ping);

    // Vector 2: Read 2 bytes from register 0x38 (Present Position) for Servo ID 2
    // ID=2, Length=4, Instruction=0x02, Params=[0x38, 0x02]
    // Sum = 2 + 4 + 2 + 0x38 + 2 = 66 = 0x42. Checksum = !(0x42) & 0xFF = 0xBD
    let read_packet = feetech::build_read(2, REG_PRESENT_POSITION, 2);
    let expected_read = vec![0xFF, 0xFF, 0x02, 0x04, INST_READ, 0x38, 0x02, 0xBD];
    assert_eq!(read_packet, expected_read);

    // Vector 3: Torque Enable = true for Servo ID 1
    // Address 0x28, Data=[1]
    // ID=1, Length=4, Instruction=0x03, Params=[0x28, 0x01]
    // Sum = 1 + 4 + 3 + 40 + 1 = 49 = 0x31. Checksum = !(0x31) & 0xFF = 0xCE
    let torque_enable = feetech::build_set_torque(1, true);
    assert_eq!(
        torque_enable,
        vec![0xFF, 0xFF, 0x01, 0x04, INST_WRITE, REG_TORQUE_ENABLE, 0x01, 0xCE]
    );

    // Vector 4: Torque Enable = false for Servo ID 1
    // Sum = 1 + 4 + 3 + 40 + 0 = 48 = 0x30. Checksum = !(0x30) & 0xFF = 0xCF
    let torque_disable = feetech::build_set_torque(1, false);
    assert_eq!(
        torque_disable,
        vec![0xFF, 0xFF, 0x01, 0x04, INST_WRITE, REG_TORQUE_ENABLE, 0x00, 0xCF]
    );

    // Vector 5: SYNC_WRITE goal positions for Servo 1 (pos=2048 = 0x0800) and Servo 2 (pos=1024 = 0x0400)
    let sync_positions = vec![(1u8, 2048u16), (2u8, 1024u16)];
    let sync_pkt = feetech::build_sync_write_positions(&sync_positions).unwrap();
    assert_eq!(&sync_pkt[0..2], &FEETECH_HEADER);
    assert_eq!(sync_pkt[2], feetech::BROADCAST_ID); // 0xFE
    assert_eq!(sync_pkt[4], INST_SYNC_WRITE); // 0x83
    assert_eq!(sync_pkt[5], REG_GOAL_POSITION); // 0x2A
    assert_eq!(sync_pkt[6], 2); // Data len per servo = 2 bytes
    // Servo 1 entry: ID=1, 0x00, 0x08
    assert_eq!(&sync_pkt[7..10], &[1, 0x00, 0x08]);
    // Servo 2 entry: ID=2, 0x00, 0x04
    assert_eq!(&sync_pkt[10..13], &[2, 0x00, 0x04]);
    // Verify checksum matches calculate_checksum
    let calculated_cs = feetech::calculate_checksum(
        sync_pkt[2],
        sync_pkt[3],
        sync_pkt[4],
        &sync_pkt[5..sync_pkt.len() - 1],
    );
    assert_eq!(*sync_pkt.last().unwrap(), calculated_cs);
}

#[test]
fn test_feetech_response_parsing_and_error_handling() {
    // Valid response from Servo 1: Error=0, Data=[0x00, 0x08] (position 2048)
    // ID=1, Length=4, Error=0x00, Params=[0x00, 0x08]
    // Sum = 1 + 4 + 0 + 0 + 8 = 13 = 0x0D. Checksum = !(0x0D) & 0xFF = 0xF2
    let valid_response = vec![0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2];
    let (pkt, consumed) = feetech::parse_response(&valid_response).unwrap();
    assert_eq!(consumed, 8);
    assert_eq!(pkt.id, 1);
    assert_eq!(pkt.error, 0);
    assert_eq!(pkt.params, vec![0x00, 0x08]);

    // Checksum mismatch interception
    let mut corrupt_checksum = valid_response.clone();
    corrupt_checksum[7] ^= 0x01; // flip 1 bit
    let err = feetech::parse_response(&corrupt_checksum).unwrap_err();
    match err {
        feetech::FeetechError::ChecksumMismatch { expected, calculated } => {
            assert_eq!(expected, 0xF3);
            assert_eq!(calculated, 0xF2);
        }
        _ => panic!("Expected ChecksumMismatch, got {:?}", err),
    }

    // Truncated frame interception
    let truncated = &valid_response[..5];
    assert!(matches!(
        feetech::parse_response(truncated),
        Err(feetech::FeetechError::IncompleteFrame { .. })
    ));

    // Stream with noise before valid frame
    let mut noisy_stream = vec![0x00, 0x12, 0xAA, 0x55];
    noisy_stream.extend_from_slice(&valid_response);
    let (pkt2, consumed2) = feetech::parse_response(&noisy_stream).unwrap();
    assert_eq!(consumed2, 4 + 8);
    assert_eq!(pkt2.id, 1);
}

#[test]
fn test_feetech_radian_step_conversions() {
    let offset = 2048u16;

    // 2048 steps -> 0.0 rad
    let zero_rad = feetech::steps_to_rad(2048, offset);
    assert!((zero_rad - 0.0).abs() < 1e-6);

    // 0.0 rad -> 2048 steps
    assert_eq!(feetech::rad_to_steps(0.0, offset), 2048);

    // Roundtrip tests across range [-PI, PI]
    for deg in [-180, -90, -45, 0, 45, 90, 180] {
        let original_rad = (deg as f32) * PI / 180.0;
        let steps = feetech::rad_to_steps(original_rad, offset);
        let reconstructed_rad = feetech::steps_to_rad(steps, offset);
        // STS3215 resolution: 2*PI / 4096 ~= 0.001534 rad
        let diff = (original_rad - reconstructed_rad).abs();
        assert!(
            diff <= 0.002,
            "Deg {} failed: orig={}, reconstructed={}, diff={}",
            deg,
            original_rad,
            reconstructed_rad,
            diff
        );
    }

    // Clamping limits
    assert_eq!(feetech::rad_to_steps(10.0, offset), 4095);
    assert_eq!(feetech::rad_to_steps(-10.0, offset), 0);

    // Temperature & load conversions
    assert_eq!(feetech::raw_to_temperature(55), 55.0);
    // Raw load 500 (50% torque positive)
    let torque_half = feetech::raw_load_to_torque(500);
    assert!((torque_half - 1.5).abs() < 1e-4);
}

#[test]
fn test_feetech_driver_mock_loopback() {
    let mock = MockTransport::new();
    let servo_ids = vec![1, 2, 3];
    let mut driver = FeetechArmDriver::new(mock, servo_ids.clone());

    // 1. Test set_torque(true)
    driver.set_torque(true).unwrap();
    let tx_bytes = driver.transport_mut().take_tx();
    // 3 servos * 8 bytes each = 24 bytes
    assert_eq!(tx_bytes.len(), 24);

    // 2. Test write_joint_targets
    let targets = vec![
        JointTarget::position(0.0),
        JointTarget::position(PI / 4.0),
        JointTarget::position(-PI / 4.0),
    ];
    driver.write_joint_targets(&targets).unwrap();
    let sync_tx = driver.transport_mut().take_tx();
    assert!(!sync_tx.is_empty());
    assert_eq!(sync_tx[4], INST_SYNC_WRITE);

    // Target count mismatch error
    let invalid_targets = vec![JointTarget::position(0.0)];
    let err = driver.write_joint_targets(&invalid_targets).unwrap_err();
    assert!(matches!(err, ArmDriverError::JointCountMismatch { .. }));

    // 3. Test read_joint_states with simulated responses
    // For each servo, prepare an 8-byte response frame
    // Address 0x38: pos(2), speed(2), load(2), volt(1), temp(1)
    let mut rx_responses = Vec::new();
    for id in &servo_ids {
        let pos_bytes = 2048u16.to_le_bytes();
        let speed_bytes = 0i16.to_le_bytes();
        let load_bytes = 100u16.to_le_bytes();
        let volt = 74u8; // 7.4V
        let temp = 38u8; // 38 C
        let params = vec![
            pos_bytes[0], pos_bytes[1],
            speed_bytes[0], speed_bytes[1],
            load_bytes[0], load_bytes[1],
            volt, temp,
        ];
        let length = (params.len() + 2) as u8;
        let cs = feetech::calculate_checksum(*id, length, 0x00, &params);
        let mut pkt = vec![0xFF, 0xFF, *id, length, 0x00];
        pkt.extend_from_slice(&params);
        pkt.push(cs);
        rx_responses.extend_from_slice(&pkt);
    }

    assert!(!driver.is_feedback_valid());
    driver.transport_mut().provide_rx(&rx_responses);
    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 3);
    for s in &states {
        assert!((s.position - 0.0).abs() < 1e-4);
        assert!((s.temperature - 38.0).abs() < 1e-4);
    }
    assert!(driver.is_feedback_valid());
}

// =========================================================================
// 2. FashionStar UartServo Protocol Golden Vectors & Driver Tests
// =========================================================================

#[test]
fn test_fashionstar_packet_vectors() {
    // Vector 1: Ping packet for Servo ID 1
    // Wire format: [Header0, Header1, Cmd, Length, Id, ...payload, Checksum]
    // Header=[0x12, 0x4C], CMD=0x01, Length=1, ID=1, Checksum=(0x12 + 0x4C + 1 + 1 + 1) & 0xFF = 0x61
    let ping = fashionstar::build_ping(1);
    assert_eq!(ping, vec![0x12, 0x4C, CMD_PING, 0x01, 0x01, 0x61]);

    // Vector 2: Set Torque = 1 for Servo ID 2
    // CMD=0x1A, Length=2, ID=2, Content=[0x01]
    // Checksum = (0x12 + 0x4C + 0x1A + 2 + 2 + 1) & 0xFF = 0x7D
    let torque_pkt = fashionstar::build_set_torque(2, true);
    assert_eq!(
        torque_pkt,
        vec![0x12, 0x4C, CMD_SET_TORQUE, 0x02, 0x02, 0x01, 0x7D]
    );

    // Vector 3: Query Angle for Servo ID 3
    // CMD=0x0A, Length=1, ID=3
    // Checksum = (0x12 + 0x4C + 0x0A + 1 + 3) & 0xFF = 0x6C
    let query_pkt = fashionstar::build_query_angle(3);
    assert_eq!(
        query_pkt,
        vec![0x12, 0x4C, CMD_QUERY_SERVO_ANGLE, 0x01, 0x03, 0x6C]
    );

    // Vector 4: Set Angle: ID=1, angle=90.0 deg (900 tenth-deg = 0x0384), interval=50ms, power=1000
    let set_angle_pkt = fashionstar::build_set_angle(1, 900, 50, 1000);
    assert_eq!(&set_angle_pkt[0..2], &FASHIONSTAR_HEADER);
    assert_eq!(set_angle_pkt[2], CMD_SET_SERVO_ANGLE);
    assert_eq!(set_angle_pkt[3], 7); // Length = 6 (content) + 1 (id) = 7
    assert_eq!(set_angle_pkt[4], 1); // ID
    let cs = fashionstar::calculate_checksum(
        FASHIONSTAR_REQUEST_HEADER,
        CMD_SET_SERVO_ANGLE,
        7,
        1,
        &set_angle_pkt[5..11],
    );
    assert_eq!(*set_angle_pkt.last().unwrap(), cs);

    // Vector 5: Official golden response frame vector: 05 1C 0A 03 00 86 03 B7
    // Header=[0x05, 0x1C], CMD=0x0A, Len=0x03, ID=0x00, Payload=[0x86, 0x03], CS=0xB7
    let golden_resp = vec![0x05, 0x1C, 0x0A, 0x03, 0x00, 0x86, 0x03, 0xB7];
    let (parsed_golden, consumed_golden) = fashionstar::parse_packet(&golden_resp).unwrap();
    assert_eq!(consumed_golden, 8);
    assert_eq!(parsed_golden.header, FASHIONSTAR_RESPONSE_HEADER);
    assert_eq!(parsed_golden.cmd_id, 0x0A);
    assert_eq!(parsed_golden.id, 0x00);
    assert_eq!(parsed_golden.content, vec![0x86, 0x03]);

    // Vector 6: Response frame parsing with response header [0x05, 0x1C]
    let resp_content = [0x84, 0x03]; // 900 tenth-deg (90.0 deg)
    let resp_cs = fashionstar::calculate_checksum(
        FASHIONSTAR_RESPONSE_HEADER,
        CMD_QUERY_SERVO_ANGLE,
        3, // length = 2 + 1 = 3
        1, // ID = 1
        &resp_content,
    );
    let mut resp_frame = vec![0x05, 0x1C, CMD_QUERY_SERVO_ANGLE, 0x03, 0x01];
    resp_frame.extend_from_slice(&resp_content);
    resp_frame.push(resp_cs);

    let (parsed_resp, consumed_resp) = fashionstar::parse_packet(&resp_frame).unwrap();
    assert_eq!(consumed_resp, 8);
    assert_eq!(parsed_resp.header, FASHIONSTAR_RESPONSE_HEADER);
    assert_eq!(parsed_resp.id, 1);
    assert_eq!(parsed_resp.cmd_id, CMD_QUERY_SERVO_ANGLE);
    assert_eq!(parsed_resp.content, resp_content);
}

#[test]
fn test_fashionstar_error_handling_and_parsing() {
    let valid_packet = vec![0x12, 0x4C, CMD_PING, 0x01, 0x01, 0x61];
    let (pkt, consumed) = fashionstar::parse_packet(&valid_packet).unwrap();
    assert_eq!(consumed, 6);
    assert_eq!(pkt.id, 1);
    assert_eq!(pkt.cmd_id, CMD_PING);

    // Corrupted checksum
    let mut bad_cs = valid_packet.clone();
    bad_cs[5] = 0xFF;
    assert!(matches!(
        fashionstar::parse_packet(&bad_cs),
        Err(fashionstar::FashionStarError::ChecksumMismatch { .. })
    ));

    // Truncated packet
    assert!(matches!(
        fashionstar::parse_packet(&valid_packet[..4]),
        Err(fashionstar::FashionStarError::IncompleteFrame { .. })
    ));
}

#[test]
fn test_fashionstar_angle_unit_conversions() {
    // 90.0 degrees (900 tenth-degrees) -> PI/2 radians
    let rad = fashionstar::tenth_deg_to_rad(900);
    assert!((rad - PI / 2.0).abs() < 1e-4);

    // PI/2 radians -> 900 tenth-degrees
    let tenth_deg = fashionstar::rad_to_tenth_deg(PI / 2.0);
    assert_eq!(tenth_deg, 900);

    // -45.0 degrees
    let rad_neg = fashionstar::tenth_deg_to_rad(-450);
    assert!((rad_neg - (-PI / 4.0)).abs() < 1e-4);
    assert_eq!(fashionstar::rad_to_tenth_deg(-PI / 4.0), -450);

    // Magnetic encoder conversion: center 2048 -> 0 rad
    assert!((fashionstar::magnetic_to_rad(2048, 2048) - 0.0).abs() < 1e-6);
    assert_eq!(fashionstar::rad_to_magnetic(0.0, 2048), 2048);
}

#[test]
fn test_fashionstar_driver_mock_loopback() {
    let mock = MockTransport::new();
    let servo_ids = vec![1, 2];
    let mut driver = FashionStarArmDriver::new(mock, servo_ids.clone());

    driver.set_torque(true).unwrap();
    let tx = driver.transport_mut().take_tx();
    assert_eq!(tx.len(), 2 * 7); // 2 servos * 7 bytes

    // Prepare simulated angle responses, including an RS485 half-duplex TX echo before the response
    let echo_pkt = fashionstar::build_query_angle(1);
    let mut full_resp = echo_pkt; // injected echo of [0x12, 0x4C, ...]

    // Servo 1: 0.0 rad (0 tenth-deg)
    let content_1 = 0i16.to_le_bytes();
    let cs_1 = fashionstar::calculate_checksum(
        FASHIONSTAR_RESPONSE_HEADER,
        CMD_QUERY_SERVO_ANGLE,
        3,
        1,
        &content_1,
    );
    let resp_1 = vec![0x05, 0x1C, CMD_QUERY_SERVO_ANGLE, 0x03, 0x01, content_1[0], content_1[1], cs_1];
    full_resp.extend_from_slice(&resp_1);

    // Servo 2: PI/2 rad (900 tenth-deg)
    let content_2 = 900i16.to_le_bytes();
    let cs_2 = fashionstar::calculate_checksum(
        FASHIONSTAR_RESPONSE_HEADER,
        CMD_QUERY_SERVO_ANGLE,
        3,
        2,
        &content_2,
    );
    let resp_2 = vec![0x05, 0x1C, CMD_QUERY_SERVO_ANGLE, 0x03, 0x02, content_2[0], content_2[1], cs_2];
    full_resp.extend_from_slice(&resp_2);

    assert!(!driver.is_feedback_valid());
    driver.transport_mut().provide_rx(&full_resp);

    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 2);
    assert!((states[0].position - 0.0).abs() < 1e-4);
    assert!((states[1].position - PI / 2.0).abs() < 1e-4);
    assert!(driver.is_feedback_valid());

    // Feeding 0x0C (CMD_QUERY_RAW_MAGNETIC) response frame must be rejected by angle query parser
    let raw_mag_content = 500i16.to_le_bytes();
    let cs_mag = fashionstar::calculate_checksum(
        FASHIONSTAR_RESPONSE_HEADER,
        fashionstar::CMD_QUERY_RAW_MAGNETIC,
        3,
        1,
        &raw_mag_content,
    );
    let raw_mag_resp = vec![
        0x05, 0x1C, fashionstar::CMD_QUERY_RAW_MAGNETIC, 0x03, 0x01,
        raw_mag_content[0], raw_mag_content[1], cs_mag,
    ];
    driver.transport_mut().provide_rx(&raw_mag_resp);
    let states_after = driver.read_joint_states().unwrap();
    assert_eq!(states_after[0].position, 0.0); // Stays at 0.0, not overwritten by 0x0C
}

// =========================================================================
// 3. Damiao Motor Protocol Golden Vectors & Driver Tests
// =========================================================================

#[test]
fn test_damiao_mit_quantization_and_packet_vectors() {
    let limits = DamiaoLimits::default();

    // Verify float_to_uint and uint_to_float roundtrip precision
    // Position: [-12.5, 12.5], 16-bit
    for test_p in [-12.5, -6.28, 0.0, 3.14159, 12.5] {
        let u = damiao::float_to_uint(test_p, limits.p_min, limits.p_max, 16);
        let f = damiao::uint_to_float(u, limits.p_min, limits.p_max, 16);
        assert!((test_p - f).abs() < 0.001);
    }

    // Velocity: [-30, 30], 12-bit
    for test_v in [-30.0, -10.0, 0.0, 15.0, 30.0] {
        let u = damiao::float_to_uint(test_v, limits.v_min, limits.v_max, 12);
        let f = damiao::uint_to_float(u, limits.v_min, limits.v_max, 12);
        assert!((test_v - f).abs() < 0.02);
    }

    // MIT control frame packaging
    let frame = damiao::build_mit_frame(
        0x01,
        0.0,    // pos: midpoint -> 0x8000
        0.0,    // vel: midpoint -> 0x0800
        50.0,   // Kp
        1.0,    // Kd
        0.0,    // torque: midpoint -> 0x0800
        &limits,
    );
    assert_eq!(frame.id, 0x01);
    assert_eq!(frame.dlc, 8);
    // Byte 0-1 should represent midpoint of 16-bit uint (32767/32768)
    let p_unpacked = u16::from_be_bytes([frame.data[0], frame.data[1]]);
    assert!((p_unpacked as i32 - 32767).abs() <= 1);
}

#[test]
fn test_damiao_command_frames_and_feedback() {
    let limits = DamiaoLimits::default();

    // Enable and Disable frames
    let enable_frame = damiao::build_enable_frame(0x02);
    assert_eq!(enable_frame.id, 0x02);
    assert_eq!(enable_frame.data, CMD_ENABLE);

    let disable_frame = damiao::build_disable_frame(0x02);
    assert_eq!(disable_frame.id, 0x02);
    assert_eq!(disable_frame.data, CMD_DISABLE);

    // Feedback frame decoding
    // Mock feedback from Motor 3: ID=3 (low 4 bits), State=1 (high 4 bits) -> data[0] = 0x13
    let p_mid = damiao::float_to_uint(0.0, limits.p_min, limits.p_max, 16);
    let v_mid = damiao::float_to_uint(0.0, limits.v_min, limits.v_max, 12);
    let t_mid = damiao::float_to_uint(0.0, limits.t_min, limits.t_max, 12);

    let mut data = [0u8; 8];
    data[0] = 0x13; // state=1, id=3
    data[1] = (p_mid >> 8) as u8;
    data[2] = (p_mid & 0xFF) as u8;
    data[3] = (v_mid >> 4) as u8;
    data[4] = (((v_mid & 0x0F) << 4) | (t_mid >> 8)) as u8;
    data[5] = (t_mid & 0xFF) as u8;
    data[6] = 42; // MOS temp = 42 C
    data[7] = 39; // Rotor temp = 39 C

    let feedback_can = CanFrame::new_standard(0x00, &data).unwrap();
    let fb = damiao::unpack_feedback_frame(&feedback_can, &limits).unwrap();
    assert_eq!(fb.motor_id, 3);
    assert_eq!(fb.state_or_error, 1);
    assert!((fb.position - 0.0).abs() < 0.001);
    assert!((fb.velocity - 0.0).abs() < 0.02);
    assert!((fb.torque - 0.0).abs() < 0.02);
    assert_eq!(fb.t_mos, 42.0);
    assert_eq!(fb.t_rotor, 39.0);
}

#[test]
fn test_damiao_serial_bridge_vectors() {
    let can = CanFrame::new_standard(0x0105, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
    let encoded = damiao::encode_serial_bridge_frame(&can);
    assert_eq!(encoded.len(), 14);
    assert_eq!(&encoded[0..2], &damiao::SERIAL_BRIDGE_HEADER);

    let (decoded, consumed) = damiao::decode_serial_bridge_frame(&encoded).unwrap();
    assert_eq!(consumed, 14);
    assert_eq!(decoded.id, 0x0105);
    assert_eq!(decoded.data, can.data);

    // Checksum mismatch interception
    let mut corrupt = encoded.clone();
    corrupt[13] ^= 0xFF;
    assert!(matches!(
        damiao::decode_serial_bridge_frame(&corrupt),
        Err(damiao::DamiaoError::ChecksumMismatch { .. })
    ));
}

#[test]
fn test_damiao_driver_mock_loopback() {
    let mock_can = MockCanTransport::new();
    let motor_ids = vec![1, 2];
    let mut driver = DamiaoArmDriver::new(mock_can, motor_ids.clone());

    // 1. Set torque enable: provide motor enable feedback frames (state=1)
    let mut d1_en = [0u8; 8];
    d1_en[0] = 0x11; // state=1 (enabled), motor_id=1
    let mut d2_en = [0u8; 8];
    d2_en[0] = 0x12; // state=1 (enabled), motor_id=2
    driver.can_mut().provide_rx_frame(CanFrame::new_standard(0x11, &d1_en).unwrap());
    driver.can_mut().provide_rx_frame(CanFrame::new_standard(0x12, &d2_en).unwrap());

    driver.set_torque(true).unwrap();
    let tx = driver.can_mut().take_tx_frames();
    assert_eq!(tx.len(), 2);
    assert_eq!(tx[0].id, 1);
    assert_eq!(tx[0].data, CMD_ENABLE);
    assert_eq!(tx[1].id, 2);
    assert_eq!(tx[1].data, CMD_ENABLE);

    // 2. Write joint targets
    let targets = vec![
        JointTarget::position(1.0),
        JointTarget::position(-1.0),
    ];
    driver.write_joint_targets(&targets).unwrap();
    let mit_tx = driver.can_mut().take_tx_frames();
    assert_eq!(mit_tx.len(), 2);

    // 3. Receive feedback frames
    let limits = DamiaoLimits::default();
    let p1 = damiao::float_to_uint(1.0, limits.p_min, limits.p_max, 16);
    let mut d1 = [0u8; 8];
    d1[0] = 0x01; // motor_id = 1
    d1[1] = (p1 >> 8) as u8;
    d1[2] = (p1 & 0xFF) as u8;
    d1[6] = 45; // temp

    driver.can_mut().provide_rx_frame(CanFrame::new_standard(0, &d1).unwrap());
    let states = driver.read_joint_states().unwrap();
    assert!((states[0].position - 1.0).abs() < 0.005);
    assert_eq!(states[0].temperature, 45.0);

    // Also test feedback frame with real hardware master CAN ID: 0x10 + motor_id (0x11, 0x12)
    let p2 = damiao::float_to_uint(0.5, limits.p_min, limits.p_max, 16);
    let mut d1_master = [0u8; 8];
    d1_master[0] = 0x11; // state=1 (enabled), motor_id=1
    d1_master[1] = (p2 >> 8) as u8;
    d1_master[2] = (p2 & 0xFF) as u8;
    d1_master[6] = 48;

    let mut d2_master = [0u8; 8];
    d2_master[0] = 0x12; // state=1 (enabled), motor_id=2
    d2_master[1] = (p2 >> 8) as u8;
    d2_master[2] = (p2 & 0xFF) as u8;
    d2_master[6] = 46;

    driver.can_mut().provide_rx_frame(CanFrame::new_standard(0x11, &d1_master).unwrap());
    driver.can_mut().provide_rx_frame(CanFrame::new_standard(0x12, &d2_master).unwrap());
    let states = driver.read_joint_states().unwrap();
    assert!((states[0].position - 0.5).abs() < 0.005);
    assert_eq!(states[0].temperature, 48.0);
    assert!((states[1].position - 0.5).abs() < 0.005);
    assert_eq!(states[1].temperature, 46.0);
    assert!(driver.is_feedback_valid());

    // 4. Test filtering of invalid CAN ID and out-of-range motor ID
    let mut bad_d = [0u8; 8];
    bad_d[0] = 0x00; // invalid motor_id = 0
    driver.can_mut().provide_rx_frame(CanFrame::new_standard(0, &bad_d).unwrap());
    // Mismatched CAN ID (0x99)
    driver.can_mut().provide_rx_frame(CanFrame::new_standard(0x99, &d1).unwrap());
    let _ = driver.read_joint_states().unwrap();

    // 5. Test best-effort disable
    driver.set_torque(false).unwrap();
    let disable_tx = driver.can_mut().take_tx_frames();
    assert_eq!(disable_tx.len(), 2);
    assert_eq!(disable_tx[0].data, CMD_DISABLE);
    assert_eq!(disable_tx[1].data, CMD_DISABLE);
}

// =========================================================================
// 4. RobStride CAN Protocol Golden Vectors & Driver Tests
// =========================================================================

#[test]
fn test_robstride_extended_can_id_and_mit_vectors() {
    let limits = RobStrideLimits::default();

    // Test 29-bit extended CAN ID packing & unpacking
    // Type=1 (Motion Control), Data=0x1234, MotorID=0x05
    let ext_id = robstride::make_extended_id(TYPE_MOTION_CONTROL, 0x1234, 0x05);
    let (t, d, m) = robstride::parse_extended_id(ext_id);
    assert_eq!(t, TYPE_MOTION_CONTROL);
    assert_eq!(d, 0x1234);
    assert_eq!(m, 0x05);

    // Enable Frame (Type 3)
    let enable_frame = robstride::build_enable_frame(0x02, 0xFD);
    assert!(enable_frame.is_extended);
    let (c_type, data_field, m_id) = robstride::parse_extended_id(enable_frame.id);
    assert_eq!(c_type, TYPE_ENABLE);
    assert_eq!(data_field, 0xFD);
    assert_eq!(m_id, 0x02);

    // Stop Frame (Type 4)
    let stop_frame = robstride::build_stop_frame(0x02, 0xFD);
    let (c_type_stop, data_stop, _) = robstride::parse_extended_id(stop_frame.id);
    assert_eq!(c_type_stop, TYPE_STOP);
    assert_eq!(data_stop, 0xFD);

    // MIT mode frame construction
    let mit_frame = robstride::build_mit_frame(
        0x01,
        0.0,
        0.0,
        50.0,
        1.0,
        0.0,
        &limits,
    );
    assert!(mit_frame.is_extended);
    assert_eq!(mit_frame.dlc, 8);
    let (comm_type, t_raw, target_id) = robstride::parse_extended_id(mit_frame.id);
    assert_eq!(comm_type, TYPE_MOTION_CONTROL);
    assert_eq!(target_id, 0x01);
    assert_eq!(t_raw, 32768); // Zero torque is 32768 in 16-bit quantization

    // 4 x 16-bit big-endian payload
    let p_unpacked = u16::from_be_bytes([mit_frame.data[0], mit_frame.data[1]]);
    let v_unpacked = u16::from_be_bytes([mit_frame.data[2], mit_frame.data[3]]);
    assert_eq!(p_unpacked, 32768);
    assert_eq!(v_unpacked, 32768);
}

#[test]
fn test_robstride_feedback_unpacking() {
    let limits = RobStrideLimits::default();

    // Construct mock feedback: ID=1, pos=3.14159, vel=5.0, torque=2.0, temp=50.0 (500 in 0.1C), fault=3
    let p_raw = robstride::float_to_uint(3.14159, limits.p_min, limits.p_max, 16);
    let v_raw = robstride::float_to_uint(5.0, limits.v_min, limits.v_max, 16);
    let t_raw = robstride::float_to_uint(2.0, limits.t_min, limits.t_max, 16);
    let temp_raw: u16 = 500; // 500 * 0.1 C = 50.0 C

    let p_b = p_raw.to_be_bytes();
    let v_b = v_raw.to_be_bytes();
    let t_b = t_raw.to_be_bytes();
    let temp_b = temp_raw.to_be_bytes();

    let data = [p_b[0], p_b[1], v_b[0], v_b[1], t_b[0], t_b[1], temp_b[0], temp_b[1]];
    // Extended ID: type=0x02, fault=0x03, motor_id=0x01, master_id=0xFD
    let ext_id = robstride::build_feedback_extended_id(0x03, 0x01, 0xFD);
    let frame = CanFrame::new_extended(ext_id, &data).unwrap();

    let fb = robstride::unpack_feedback_frame(&frame, &limits).unwrap();
    assert_eq!(fb.motor_id, 1);
    assert!((fb.position - 3.14159).abs() < 0.005);
    assert!((fb.velocity - 5.0).abs() < 0.01);
    assert!((fb.torque - 2.0).abs() < 0.01);
    assert_eq!(fb.mos_temperature, 50.0);
    assert_eq!(fb.fault_flags, 3);

    // Negative tests:
    // 1. Rejection of standard (non-extended) CAN frame
    let std_frame = CanFrame::new_standard(0x123, &data).unwrap();
    assert!(matches!(
        robstride::unpack_feedback_frame(&std_frame, &limits),
        Err(robstride::RobStrideError::UnexpectedType { is_extended: false, .. })
    ));

    // 2. Rejection of non-0x02 extended frame (e.g. comm_type = 1 motion control)
    let bad_type_id = robstride::make_extended_id(robstride::TYPE_MOTION_CONTROL, 0x00, 0x01);
    let bad_type_frame = CanFrame::new_extended(bad_type_id, &data).unwrap();
    assert!(matches!(
        robstride::unpack_feedback_frame(&bad_type_frame, &limits),
        Err(robstride::RobStrideError::UnexpectedType { comm_type: 1, is_extended: true })
    ));

    // 3. Rejection of RTR frame
    let mut rtr_frame = CanFrame::new_extended(ext_id, &data).unwrap();
    rtr_frame.is_rtr = true;
    assert!(matches!(
        robstride::unpack_feedback_frame(&rtr_frame, &limits),
        Err(robstride::RobStrideError::ProtocolViolation)
    ));
}

#[test]
fn test_robstride_driver_mock_loopback() {
    let mock_can = MockCanTransport::new();
    let motor_ids = vec![1, 2];
    let mut driver = RobStrideArmDriver::new(mock_can, motor_ids.clone());

    driver.set_torque(true).unwrap();
    let tx = driver.can_mut().take_tx_frames();
    assert_eq!(tx.len(), 2);
    for frame in &tx {
        assert!(frame.is_extended);
        let (comm_type, _, _) = robstride::parse_extended_id(frame.id);
        assert_eq!(comm_type, TYPE_ENABLE);
    }

    driver.write_joint_targets(&[
        JointTarget::position(0.5),
        JointTarget::position(-0.5),
    ]).unwrap();
    let mit_tx = driver.can_mut().take_tx_frames();
    assert_eq!(mit_tx.len(), 2);

    // Provide simulated feedback frames for motor 1 and motor 2
    let fb_id1 = robstride::build_feedback_extended_id(0x00, 1, 0xFD);
    let fb_id2 = robstride::build_feedback_extended_id(0x00, 2, 0xFD);
    let data1 = [128u8, 0, 128, 0, 128, 0, 0, 250]; // mid-range pos/vel/torque, temp 250 -> 25.0C
    let data2 = [128u8, 0, 128, 0, 128, 0, 1, 4];   // temp 260 -> 26.0C
    driver.can_mut().provide_rx_frames(&[
        CanFrame::new_extended(fb_id1, &data1).unwrap(),
        CanFrame::new_extended(fb_id2, &data2).unwrap(),
    ]);
    let states = driver.read_joint_states().unwrap();
    assert_eq!(states.len(), 2);
    assert!((states[0].temperature - 25.0).abs() < 1e-4);
    assert!((states[1].temperature - 26.0).abs() < 1e-4);
}

// =========================================================================
// 5. LAWICEL SLCAN Protocol & Transport Loopback Tests
// =========================================================================

#[test]
fn test_slcan_standard_frame_encoding_decoding() {
    // Standard data frame: ID=0x123, DLC=4, Data=[0x11, 0x22, 0x33, 0x44]
    let frame = CanFrame::new_standard(0x123, &[0x11, 0x22, 0x33, 0x44]).unwrap();
    let encoded = SlcanCodec::encode_frame(&frame);
    assert_eq!(encoded, "t123411223344\r");

    let decoded = SlcanCodec::decode_frame(&encoded).unwrap();
    assert_eq!(decoded.id, 0x123);
    assert!(!decoded.is_extended);
    assert!(!decoded.is_rtr);
    assert_eq!(decoded.dlc, 4);
    assert_eq!(&decoded.data[..4], &[0x11, 0x22, 0x33, 0x44]);
}

#[test]
fn test_slcan_extended_frame_encoding_decoding() {
    // Extended data frame: ID=0x00010002, DLC=8
    let data = [1, 2, 3, 4, 5, 6, 7, 8];
    let frame = CanFrame::new_extended(0x00010002, &data).unwrap();
    let encoded = SlcanCodec::encode_frame(&frame);
    assert_eq!(encoded, "T0001000280102030405060708\r");

    let decoded = SlcanCodec::decode_frame(&encoded).unwrap();
    assert_eq!(decoded.id, 0x00010002);
    assert!(decoded.is_extended);
    assert_eq!(decoded.dlc, 8);
    assert_eq!(decoded.data, data);
}

#[test]
fn test_slcan_rtr_frames() {
    // Standard RTR: ID=0x7DF, DLC=8
    let rtr_std = CanFrame::new_rtr_standard(0x7DF, 8);
    let enc_std = SlcanCodec::encode_frame(&rtr_std);
    assert_eq!(enc_std, "r7DF8\r");
    let dec_std = SlcanCodec::decode_frame(&enc_std).unwrap();
    assert!(dec_std.is_rtr);
    assert_eq!(dec_std.id, 0x7DF);

    // Extended RTR: ID=0x18DAF110, DLC=4
    let rtr_ext = CanFrame::new_rtr_extended(0x18DAF110, 4);
    let enc_ext = SlcanCodec::encode_frame(&rtr_ext);
    assert_eq!(enc_ext, "R18DAF1104\r");
    let dec_ext = SlcanCodec::decode_frame(&enc_ext).unwrap();
    assert!(dec_ext.is_rtr);
    assert!(dec_ext.is_extended);
    assert_eq!(dec_ext.id, 0x18DAF110);
}

#[test]
fn test_slcan_error_and_malformed_frames() {
    // Empty
    assert!(SlcanCodec::decode_frame("").is_err());
    // Truncated standard
    assert!(SlcanCodec::decode_frame("t12").is_err());
    // Invalid hex in ID
    assert!(SlcanCodec::decode_frame("t12G411223344").is_err());
    // DLC > 8
    assert!(SlcanCodec::decode_frame("t1239112233445566778899").is_err());
    // Unknown prefix
    assert!(SlcanCodec::decode_frame("x1234").is_err());
}

#[test]
fn test_slcan_transport_streaming_loopback() {
    let mut mock = MockTransport::new();
    mock.set_loopback(true);

    let mut slcan = SlcanTransport::new(mock);

    // Open channel command
    slcan.open_channel().unwrap();
    // Set bitrate S8 (1Mbps)
    slcan.set_bitrate(8).unwrap();

    // Clear loopback buffer from command echoes
    slcan.inner_mut().clear();

    // Transmit standard CAN frame and receive back via loopback
    let f1 = CanFrame::new_standard(0x321, &[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();
    slcan.transmit(&f1).unwrap();
    let rx1 = slcan.receive().unwrap();
    assert_eq!(rx1.id, 0x321);
    assert_eq!(&rx1.data[..4], &[0xDE, 0xAD, 0xBE, 0xEF]);

    // Transmit extended CAN frame and receive back via loopback
    let f2 = CanFrame::new_extended(0x1ABCDEF0, &[0xAA, 0xBB, 0xCC]).unwrap();
    slcan.transmit(&f2).unwrap();
    let rx2 = slcan.receive().unwrap();
    assert_eq!(rx2.id, 0x1ABCDEF0);
    assert!(rx2.is_extended);
    assert_eq!(&rx2.data[..3], &[0xAA, 0xBB, 0xCC]);
}

// =========================================================================
// 6. Cross-Layer End-to-End Test: Damiao Arm Driver over SLCAN over Mock Serial
// =========================================================================

#[test]
fn test_e2e_damiao_over_slcan_mock_serial() {
    let mut mock_serial = MockTransport::new();
    mock_serial.set_loopback(false);

    let slcan = SlcanTransport::new(mock_serial);
    let mut arm = DamiaoArmDriver::new(slcan, vec![1]);

    // Pre-inject motor 1 feedback frame with state=1 (enabled)
    // Motor 1 feedback: CAN ID 0x11, DLC 8, data[0] = 0x11 (enabled, motor_id=1)
    // In SLCAN format: t01181100000000002823\r
    arm.can_mut().inner_mut().provide_rx(b"t01181100000000002823\r");

    // Command arm torque
    arm.set_torque(true).unwrap();

    // Inspect underlying serial stream to verify SLCAN ASCII output
    let serial_raw = arm.can_mut().inner_mut().take_tx();
    let ascii_str = std::str::from_utf8(&serial_raw).unwrap();
    // Damiao enable frame: ID 1, DLC 8, data 8 bytes FF FF FF FF FF FF FF FC
    // SLCAN representation: t0018FFFFFFFFFFFFFFFC\r
    assert!(ascii_str.contains("t0018FFFFFFFFFFFFFFFC\r"));

    // Now inject motor feedback as an SLCAN ASCII string on serial RX
    // Motor 1 feedback frame: CAN ID 0, data [0x01, p_h, p_l, v_h, (v_l|t_h), t_l, 40, 35]
    let limits = DamiaoLimits::default();
    let p_mid = damiao::float_to_uint(0.0, limits.p_min, limits.p_max, 16);
    let v_mid = damiao::float_to_uint(0.0, limits.v_min, limits.v_max, 12);
    let t_mid = damiao::float_to_uint(0.0, limits.t_min, limits.t_max, 12);
    let mut d = [0u8; 8];
    d[0] = 0x01; // motor 1
    d[1] = (p_mid >> 8) as u8;
    d[2] = (p_mid & 0xFF) as u8;
    d[3] = (v_mid >> 4) as u8;
    d[4] = (((v_mid & 0x0F) << 4) | (t_mid >> 8)) as u8;
    d[5] = (t_mid & 0xFF) as u8;
    d[6] = 40; // 40 C
    d[7] = 35; // 35 C

    let feedback_can = CanFrame::new_standard(0x000, &d).unwrap();
    let slcan_ascii = SlcanCodec::encode_frame(&feedback_can);

    // Provide the ASCII line into serial RX
    arm.can_mut().inner_mut().provide_rx(slcan_ascii.as_bytes());

    let states = arm.read_joint_states().unwrap();
    assert_eq!(states.len(), 1);
    assert!((states[0].position - 0.0).abs() < 0.001);
    assert_eq!(states[0].temperature, 40.0);
}
