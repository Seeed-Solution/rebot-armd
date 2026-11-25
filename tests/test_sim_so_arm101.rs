//! Integration tests for 6-DOF SimArmDriver (SO-101 / Feetech load mode).

use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

use armd::driver::{ArmDriver, ArmDriverError, EnableState, JointMode, JointTarget};
use armd::sim_driver::{parse_state, SimArmDriver, SimCommand, SimState, SimTarget};

struct MockSim {
    socket: UdpSocket,
    armd_addr: Option<SocketAddr>,
}

impl MockSim {
    fn new() -> Self {
        let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        socket.set_read_timeout(Some(Duration::from_millis(200))).unwrap();
        Self { socket, armd_addr: None }
    }

    fn addr(&self) -> String {
        format!("udp://{}", self.socket.local_addr().unwrap())
    }

    fn recv(&mut self) -> SimCommand {
        let mut buf = [0u8; 4096];
        let (n, from) = self.socket.recv_from(&mut buf).unwrap();
        self.armd_addr = Some(from);
        serde_json::from_slice(&buf[..n]).unwrap()
    }

    fn recv_kind(&mut self, kind: &str) -> SimCommand {
        loop {
            let c = self.recv();
            if c.kind == kind {
                return c;
            }
        }
    }

    fn send_state(&mut self, state: &SimState) {
        if self.armd_addr.is_none() {
            self.recv();
        }
        let bytes = serde_json::to_vec(state).unwrap();
        self.socket.send_to(&bytes, self.armd_addr.unwrap()).unwrap();
    }
}

fn settle() {
    std::thread::sleep(Duration::from_millis(30));
}

#[test]
fn test_sim_so101_capabilities_6dof_vs_7dof() {
    let sim = MockSim::new();
    let drv6_default = SimArmDriver::connect(&sim.addr(), 6).unwrap();
    let cap6_def = drv6_default.capabilities();
    assert!(cap6_def.torque_feedback, "Default 6-DOF SimArmDriver must preserve torque_feedback true");
    assert!(!cap6_def.load_feedback, "Default 6-DOF SimArmDriver must have load_feedback false");
    assert!(drv6_default.joint_loads().is_none(), "Default 6-DOF SimArmDriver must return None for joint_loads");

    let drv6 = SimArmDriver::connect(&sim.addr(), 6).unwrap().with_load_only_feedback(true);
    let cap6 = drv6.capabilities();
    assert!(!cap6.torque_feedback, "6-DOF SimArmDriver must have torque_feedback false (Feetech mode)");
    assert!(cap6.load_feedback, "6-DOF SimArmDriver must have load_feedback true");
    assert!(cap6.enable_feedback, "6-DOF SimArmDriver must have enable_feedback true");
    assert!(cap6.velocity_feedback, "6-DOF SimArmDriver must have velocity_feedback true");
    assert!(cap6.temperature_feedback, "6-DOF SimArmDriver must have temperature_feedback true");
    assert!(!cap6.torque_feedforward, "6-DOF SimArmDriver torque_feedforward false");

    // Contrast with 7-DOF B601-DM driver:
    let drv7 = SimArmDriver::connect(&sim.addr(), 7).unwrap();
    let cap7 = drv7.capabilities();
    assert!(cap7.torque_feedback, "7-DOF SimArmDriver must preserve torque_feedback true");
    assert!(!cap7.load_feedback, "7-DOF SimArmDriver must have load_feedback false");
    assert!(drv7.joint_loads().is_none(), "7-DOF SimArmDriver must return None for joint_loads");
}

#[test]
fn test_sim_so101_handshake_enable_disable() {
    let mut sim = MockSim::new();
    let mut drv = SimArmDriver::connect(&sim.addr(), 6).unwrap().with_load_only_feedback(true);
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..6 {
        arm_joints[i] = i;
    }
    drv.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.05,
    });

    let handle = std::thread::spawn(move || {
        let cmd = sim.recv_kind("cmd");
        assert_eq!(cmd.enable, Some(true));
        assert_eq!(cmd.targets, None);
        let st = SimState {
            v: 1,
            kind: "state".into(),
            session: 42,
            seq: 1,
            ack: cmd.seq,
            enabled: true,
            q: vec![0.0; 6],
            dq: vec![0.0; 6],
            tau: vec![0.1; 6],
            temp: vec![35.0; 6],
            loads: None,
            t: 0.002,
        };
        sim.send_state(&st);
        sim
    });

    drv.set_torque(true).unwrap();
    assert!(drv.is_enabled());
    assert_eq!(drv.enable_state(), EnableState::Enabled);
    let mut sim = handle.join().unwrap();

    // Now disable
    let handle2 = std::thread::spawn(move || {
        let cmd = sim.recv_kind("cmd");
        assert_eq!(cmd.enable, Some(false));
        let st = SimState {
            v: 1,
            kind: "state".into(),
            session: 42,
            seq: 2,
            ack: cmd.seq,
            enabled: false,
            q: vec![0.0; 6],
            dq: vec![0.0; 6],
            tau: vec![0.0; 6],
            temp: vec![35.0; 6],
            loads: None,
            t: 0.004,
        };
        sim.send_state(&st);
    });

    drv.set_torque(false).unwrap();
    assert!(!drv.is_enabled());
    assert_eq!(drv.enable_state(), EnableState::Disabled);
    handle2.join().unwrap();
}

#[test]
fn test_sim_so101_targets_6dof_and_mismatch() {
    let mut sim = MockSim::new();
    let mut drv = SimArmDriver::connect(&sim.addr(), 6).unwrap().with_load_only_feedback(true);

    // Writing 7 targets to 6-DOF driver fails with JointCountMismatch
    let err7 = drv.write_joint_targets(&vec![JointTarget::position(0.0); 7]).unwrap_err();
    assert!(matches!(err7, ArmDriverError::JointCountMismatch { expected: 6, actual: 7 }));

    // Writing 5 targets to 6-DOF driver fails with JointCountMismatch
    let err5 = drv.write_joint_targets(&vec![JointTarget::position(0.0); 5]).unwrap_err();
    assert!(matches!(err5, ArmDriverError::JointCountMismatch { expected: 6, actual: 5 }));

    // Writing 6 targets succeeds and simulator receives wire commands
    let targets = vec![
        JointTarget::position(0.1),
        JointTarget::position(-0.2),
        JointTarget::position(0.3),
        JointTarget::position(-0.4),
        JointTarget::position(0.5),
        JointTarget::position(0.8), // gripper
    ];
    drv.write_joint_targets(&targets).unwrap();

    let cmd = sim.recv_kind("cmd");
    let wire = cmd.targets.expect("targets present");
    assert_eq!(wire.len(), 6);
    assert_eq!(wire[0], SimTarget::PosVel { q: 0.1, vlim: 2.0 });
    assert_eq!(wire[5], SimTarget::PosVel { q: 0.8, vlim: 2.0 });

    // Mode switch: MIT on joint 1 (shoulder_lift)
    let modes = vec![
        JointMode::Position,
        JointMode::Mit,
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
    ];
    let hold = vec![
        JointTarget::position(0.1),
        JointTarget::mit(-0.2, 0.0, 15.0, 1.5, 0.8),
        JointTarget::position(0.3),
        JointTarget::position(-0.4),
        JointTarget::position(0.5),
        JointTarget::position(0.8),
    ];
    drv.set_joint_modes(&modes, &hold).unwrap();
    let cmd2 = sim.recv_kind("cmd");
    let wire2 = cmd2.targets.expect("targets present");
    assert_eq!(wire2[1], SimTarget::Mit { q: -0.2, dq: 0.0, kp: 15.0, kd: 1.5, tau: 0.8 });
}

#[test]
fn test_sim_so101_loads_computed_from_tau() {
    let mut sim = MockSim::new();
    let mut drv = SimArmDriver::connect(&sim.addr(), 6).unwrap().with_load_only_feedback(true);

    // State datagram with tau in N·m, no explicit loads field.
    // 2.94 N·m is the STS3215 stall torque.
    // Normalized load is |tau| / 2.94 clamped to [0.0, 1.0].
    let st = SimState {
        v: 1,
        kind: "state".into(),
        session: 100,
        seq: 1,
        ack: 0,
        enabled: true,
        q: vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        dq: vec![0.01; 6],
        tau: vec![1.47, -2.94, 0.0, 0.735, -5.88, 2.94], // 1.47/2.94=0.5, -2.94->1.0, 0->0.0, 0.735/2.94=0.25, -5.88 clamped to 1.0
        temp: vec![38.0; 6],
        loads: None,
        t: 0.1,
    };
    sim.send_state(&st);
    settle();

    let states = drv.read_joint_states().unwrap();
    assert_eq!(states.len(), 6);
    assert_eq!(states[0].position, 0.1);
    assert_eq!(states[0].torque, 0.0); // Feetech-like mode: torque reports 0.0

    let loads = drv.joint_loads().expect("joint_loads present for 6-DOF");
    assert_eq!(loads.len(), 6);
    assert!((loads[0] - 0.5).abs() < 1e-4, "expected 0.5, got {}", loads[0]);
    assert!((loads[1] - 1.0).abs() < 1e-4, "expected 1.0, got {}", loads[1]);
    assert!((loads[2] - 0.0).abs() < 1e-4, "expected 0.0, got {}", loads[2]);
    assert!((loads[3] - 0.25).abs() < 1e-4, "expected 0.25, got {}", loads[3]);
    assert!((loads[4] - 1.0).abs() < 1e-4, "expected 1.0 (clamped), got {}", loads[4]);
    assert!((loads[5] - 1.0).abs() < 1e-4, "expected 1.0, got {}", loads[5]);

    // joint_torques() returns raw simulator torques
    let torques = drv.joint_torques();
    assert_eq!(torques.len(), 6);
    assert_eq!(torques[0], 1.47);
    assert_eq!(torques[1], -2.94);
    assert_eq!(torques[4], -5.88);
}

#[test]
fn test_sim_so101_loads_explicit_from_simulator() {
    let mut sim = MockSim::new();
    let mut drv = SimArmDriver::connect(&sim.addr(), 6).unwrap().with_load_only_feedback(true);

    let explicit_loads = vec![0.12, 0.34, 0.56, 0.78, 0.90, 0.15];
    let st = SimState {
        v: 1,
        kind: "state".into(),
        session: 101,
        seq: 1,
        ack: 0,
        enabled: true,
        q: vec![0.0; 6],
        dq: vec![0.0; 6],
        tau: vec![0.0; 6],
        temp: vec![32.0; 6],
        loads: Some(explicit_loads.clone()),
        t: 0.2,
    };
    sim.send_state(&st);
    settle();

    let _ = drv.read_joint_states().unwrap();
    let loads = drv.joint_loads().expect("joint_loads present");
    for i in 0..6 {
        assert!((loads[i] - explicit_loads[i]).abs() < 1e-4);
    }
}

#[test]
fn test_sim_so101_parse_state_validation() {
    // Valid 6-DOF state
    let json_valid = br#"{"v":1,"type":"state","session":1,"seq":1,"ack":0,"enabled":true,"q":[0,0,0,0,0,0],"dq":[0,0,0,0,0,0],"tau":[0,0,0,0,0,0],"temp":[30,30,30,30,30,30],"loads":[0.1,0.2,0.3,0.4,0.5,0.6]}"#;
    assert!(parse_state(json_valid, 6).is_some());
    assert!(parse_state(json_valid, 7).is_none(), "DOF mismatch must be rejected");

    // Invalid load length
    let json_bad_load_len = br#"{"v":1,"type":"state","session":1,"seq":1,"ack":0,"enabled":true,"q":[0,0,0,0,0,0],"dq":[0,0,0,0,0,0],"tau":[0,0,0,0,0,0],"temp":[30,30,30,30,30,30],"loads":[0.1,0.2]}"#;
    assert!(parse_state(json_bad_load_len, 6).is_none());

    // Non-finite load value
    let json_nan_load = br#"{"v":1,"type":"state","session":1,"seq":1,"ack":0,"enabled":true,"q":[0,0,0,0,0,0],"dq":[0,0,0,0,0,0],"tau":[0,0,0,0,0,0],"temp":[30,30,30,30,30,30],"loads":[0.1,null,0.3,0.4,0.5,0.6]}"#;
    assert!(parse_state(json_nan_load, 6).is_none());
}
