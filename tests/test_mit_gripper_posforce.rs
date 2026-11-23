//! J7 gripper in Damiao position-force mode (CTRL_MODE 4) with a motor current ceiling:
//! frame bytes, the enable handshake (RAM mode write with torque off, register reply, readback),
//! refusal and fallback paths, recovery holds, J1-J6 frames unchanged, and a plant whose actuator
//! current saturates against a jaw stop. Fake CAN bus with a virtual clock.

mod mit_support;

use std::time::Duration;

use armd::driver::{gripper_drive_from_profile, ArmDriver, DamiaoArmDriver, DamiaoControlMode, GripperDrive, GripperPosForce, JointTarget};
use armd::model::{ArmConfig, DamiaoGripperDrive, DamiaoGripperMode};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::transport::{CanFrame, MockClock};
use mit_support::FakeBus;

const J7: usize = 6;
const RAW: u16 = 220;

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect()
}

fn pf() -> GripperPosForce {
    GripperPosForce { joint: J7, current_raw: RAW, close_speed_rad_s: 0.30, closed_rad: 0.0, open_rad: -5.7,
        calibrated: false, ceiling_torque_nm: None, torque_reference: None }
}

/// Driver with fresh, stationary, torque-off feedback; J7 jaw at -1.0 rad (open side).
fn rig(pos_force: bool) -> (DamiaoArmDriver<FakeBus>, FakeBus, MockClock) {
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
    bus.with(|s| { for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; } s.motors[J7].q = -1.0; });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(b601_limits()).unwrap();
    if pos_force { d = d.with_gripper_pos_force(pf()).unwrap(); } else { d = d.with_gripper_pos_vel(J7); }
    // Observe an empty receive queue before pushing feedback so samples carry a provable receive-age bound.
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    bus.clear_log();
    (d, bus, clock)
}

fn reg_write(id: u16, rid: u8, v: u32) -> [u8; 8] { let b = v.to_le_bytes(); [id as u8, 0, damiao::REG_WRITE, rid, b[0], b[1], b[2], b[3]] }
fn is_reg_read(f: &CanFrame, id: u16, rid: u8) -> bool { f.id == 0x7FF && f.data[0] == id as u8 && f.data[2] == damiao::REG_READ && f.data[3] == rid }
/// Command frames that could move motor 7 (MIT 7, POS_VEL 0x107, POS_FORCE 0x307), not enable/disable.
fn j7_commands(log: &[CanFrame]) -> Vec<CanFrame> {
    log.iter().filter(|f| (f.id == 7 && ![damiao::CMD_ENABLE, damiao::CMD_DISABLE, damiao::CMD_CLEAR_FAULTS].contains(&f.data)) || f.id == 0x107 || f.id == 0x307).copied().collect()
}

fn targets(j7: f32) -> Vec<JointTarget> {
    (0..7).map(|i| if i == J7 { JointTarget::position(j7) } else { JointTarget::position(0.1 * i as f32 + 0.01) }).collect()
}

#[test]
fn pos_force_frame_bytes() {
    let f = damiao::build_pos_force_frame(damiao::POS_FORCE_BASE_ID, 7, 1.0, damiao::pos_force_vel_raw(0.30), 220);
    assert_eq!(f.id, 0x307);
    assert_eq!(f.data, [0x00, 0x00, 0x80, 0x3F, 0x1E, 0x00, 0xDC, 0x00]);
    assert_eq!(damiao::pos_force_vel_raw(2.0), 200);
    assert_eq!(damiao::pos_force_vel_raw(-0.304), 30);
    assert_eq!(damiao::pos_force_vel_raw(f32::NAN), 0);
    assert_eq!(damiao::pos_force_vel_raw(1e9), u16::MAX);
}

#[test]
fn enable_writes_mode_4_reads_it_back_then_enables() {
    let (mut d, bus, _) = rig(true);
    d.set_torque(true).expect("enable");
    let log = bus.tx_log();
    let write = log.iter().position(|f| f.id == 0x7FF && f.data == reg_write(7, damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE)).expect("mode write");
    let read = log.iter().position(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)).expect("readback");
    let enable = log.iter().position(|f| f.id == 7 && f.data == damiao::CMD_ENABLE).expect("enable frame");
    assert!(write < read && read < enable, "order write {write} < read {read} < enable {enable}: {log:?}");
    assert!(j7_commands(&log).is_empty(), "J7 command frame before the first target: {:?}", j7_commands(&log));
    assert_eq!(log.iter().filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE).count(), 1, "only J7's mode is written");
    assert_eq!(bus.with(|s| s.motors[J7].ctrl_mode()), damiao::CTRL_MODE_POS_FORCE);
    assert_eq!(d.joint_control_modes()[J7], Some(DamiaoControlMode::PosForce));
    let cap = d.motor_current_cap().unwrap();
    assert!(cap.active && !cap.calibrated && cap.mode == "pos_force" && cap.current_raw == Some(RAW), "{cap:?}");
    assert!(cap.note.contains("uncalibrated"));
}

#[test]
fn final_writer_caps_every_j7_frame_and_j1_j6_frames_are_unchanged() {
    let (mut pfd, pf_bus, _) = rig(true);
    let (mut legacy, legacy_bus, _) = rig(false);
    pfd.set_torque(true).unwrap();
    legacy.set_torque(true).unwrap();
    for (j7, vel_raw) in [(-0.5f32, 30u16), (-2.0, 200)] { // closing (toward 0.0): 0.30 rad/s; opening: 2.0 rad/s
        pf_bus.clear_log();
        legacy_bus.clear_log();
        // same measured state on both drivers before the write
        pfd.read_joint_states().unwrap();
        legacy.read_joint_states().unwrap();
        pf_bus.clear_log();
        legacy_bus.clear_log();
        pfd.write_joint_targets(&targets(j7)).unwrap();
        legacy.write_joint_targets(&targets(j7)).unwrap();
        let a = pf_bus.tx_log();
        let b = legacy_bus.tx_log();
        let arm = |l: &[CanFrame]| l.iter().filter(|f| (0x101..=0x106).contains(&f.id)).copied().collect::<Vec<_>>();
        assert_eq!(arm(&a), arm(&b), "J1-J6 frames differ");
        assert_eq!(arm(&a).len(), 6);
        let j7a: Vec<_> = j7_commands(&a);
        assert_eq!(j7a.len(), 1, "{j7a:?}");
        assert_eq!(j7a[0].id, 0x307);
        let mut want = [0u8; 8];
        want[0..4].copy_from_slice(&j7.to_le_bytes());
        want[4..6].copy_from_slice(&vel_raw.to_le_bytes());
        want[6..8].copy_from_slice(&RAW.to_le_bytes());
        assert_eq!(j7a[0].data, want, "J7 frame for target {j7}");
        assert!(j7_commands(&b).iter().all(|f| f.id == 0x107), "legacy J7 stays POS_VEL: {:?}", j7_commands(&b));
        // the plants now differ only in J7 (pos-force follows the target the same way)
    }
    // an explicit fast close still encodes at most close_speed
    pfd.read_joint_states().unwrap();
    pf_bus.clear_log();
    let mut t = targets(-0.1);
    t[J7].velocity = Some(3.0);
    pfd.write_joint_targets(&t).unwrap();
    let f = j7_commands(&pf_bus.tx_log())[0];
    assert_eq!(u16::from_le_bytes([f.data[4], f.data[5]]), 30);
    assert_eq!(u16::from_le_bytes([f.data[6], f.data[7]]), RAW);
}

#[test]
fn failed_mode_exchange_leaves_capped_operation_unavailable_and_sends_no_j7_command() {
    type Setup = fn(&mut mit_support::FakeMotor);
    let cases: [(&str, Setup, &str); 4] = [
        ("no write acknowledgement", |m| m.no_write_ack = true, "could not be restored"),
        ("wrong acknowledged value", |m| m.ack_value_override = Some(damiao::CTRL_MODE_POS_VEL), "could not be restored"),
        ("mode not applied (readback 2)", |m| m.ignore_reg_writes = true, "reads 2"),
        ("silent after the mode write", |m| m.silent_after_mode_ack = true, "no fresh reply"),
    ];
    for (name, setup, why) in cases {
        let (mut d, bus, _) = rig(true);
        bus.with(|s| setup(&mut s.motors[J7]));
        let e = d.set_torque(true).unwrap_err().to_string();
        assert!(e.contains(why), "{name}: {e}");
        let log = bus.tx_log();
        assert!(j7_commands(&log).is_empty(), "{name}: J7 command frames {:?}", j7_commands(&log));
        assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE), "{name}: enable frames after a failed exchange");
        assert!(!d.is_enabled());
        assert!(!d.motor_current_cap().unwrap().active, "{name}");
        bus.clear_log();
        let w = d.write_joint_targets(&targets(-0.5));
        assert!(w.is_err_and(|e| e.to_string().contains("capped operation unavailable")), "{name}");
        assert!(bus.tx_log().is_empty(), "{name}: frames on a refused batch: {:?}", bus.tx_log());
    }
}

#[test]
fn mode_is_written_only_on_fresh_stationary_torque_off_feedback() {
    // moving jaw
    let (mut d, bus, _) = rig(true);
    bus.with(|s| { s.motors[J7].v = 0.5; s.push_all_feedback(); });
    d.read_joint_states().unwrap();
    bus.clear_log();
    let e = d.set_torque(true).unwrap_err().to_string();
    assert!(e.contains("fresh stationary"), "{e}");
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "mode written while moving");
    // stale feedback (200 ms old)
    let (mut d, bus, clock) = rig(true);
    clock.advance(Duration::from_millis(200));
    bus.with(|s| s.motors[J7].mute_feedback = true);
    bus.clear_log();
    assert!(d.set_torque(true).unwrap_err().to_string().contains("fresh stationary"));
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE));
}

#[test]
fn handshake_drain_applies_queued_telemetry_before_mode_write_checks() {
    // 1. Newer queued telemetry reports moving jaw (v = 0.5) without an intervening read_joint_states():
    //    the pre-write drain must update the cache so the stationarity check refuses the write.
    let (mut d, bus, _) = rig(true);
    bus.with(|s| { s.motors[J7].v = 0.5; s.push_feedback(J7); });
    bus.clear_log();
    let e = d.set_torque(true).unwrap_err().to_string();
    assert!(e.contains("still false"), "{e}");
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "mode written despite drained moving frame");
    assert!(!bus.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE), "enabled despite drained moving frame");

    // 2. Newer queued telemetry reports motor already enabled (state = 1): state-0 check must refuse.
    let (mut d, bus, _) = rig(true);
    bus.with(|s| { s.motors[J7].state_override = Some(1); s.push_feedback(J7); });
    bus.clear_log();
    let e = d.set_torque(true).unwrap_err().to_string();
    assert!(e.contains("torque off false"), "{e}");
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "mode written despite drained state-1 frame");

    // 3. A stationary state-0 frame that sat in the RX buffer past 100 ms since the last empty queue
    //    must not spoof fresh feedback during the pre-write drain.
    let (mut d, bus, clock) = rig(true);
    bus.with(|s| s.push_feedback(J7));
    clock.advance(Duration::from_millis(200));
    bus.clear_log();
    let e = d.set_torque(true).unwrap_err().to_string();
    assert!(e.contains("fresh false"), "{e}");
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "mode written on aged queued frame");

    // 4. A stationary state-0 frame already sitting in RX BEFORE the driver's first observed empty
    //    queue has unknown age and must not authorize a mode write.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock);
    bus.with(|s| { s.motors[J7].q = -1.0; s.push_all_feedback(); });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits()).unwrap()
        .with_gripper_pos_force(pf()).unwrap();
    bus.clear_log();
    let e = d.set_torque(true).unwrap_err().to_string();
    assert!(e.contains("fresh false"), "accepted pre-boundary startup sample: {e}");
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "mode written on pre-boundary sample");

    // 5. A feedback frame on J6's arbitration ID (0x16) carrying J7 (id=7) in payload byte 0
    //    must not overwrite a moving J7 sample from J7's real ID (0x17) during the pre-write drain.
    let (mut d, bus, _) = rig(true);
    bus.with(|s| {
        s.motors[J7].v = 0.5;
        s.push_feedback(J7);
        let mut spoof = s.motors[J7].feedback_frame();
        // Build a stationary state-0 J7 payload delivered on J6's MST_ID 0x16.
        s.motors[J7].v = 0.0;
        let still = s.motors[J7].feedback_frame();
        spoof.id = 0x16;
        spoof.data = still.data;
        s.rx.push_back(spoof);
    });
    bus.clear_log();
    let e = d.set_torque(true).unwrap_err().to_string();
    assert!(e.contains("still false"), "0x16 spoof replaced moving J7 telemetry: {e}");
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE));
}

#[test]
fn every_full_enable_reads_the_mode_back_and_a_motor_reset_is_rewritten() {
    let (mut d, bus, _) = rig(true);
    d.set_torque(true).unwrap();
    d.set_torque(false).unwrap();
    bus.clear_log();
    d.set_torque(true).unwrap();
    let log = bus.tx_log();
    assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "mode rewritten on a re-enable");
    assert_eq!(log.iter().filter(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)).count(), 1, "readback on the re-enable");
    // the motor lost its RAM mode (power cycle): the readback fails, the next enable rewrites it
    d.set_torque(false).unwrap();
    bus.with(|s| s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL));
    assert!(d.set_torque(true).unwrap_err().to_string().contains("reads 2"));
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    bus.clear_log();
    d.set_torque(true).expect("rewritten and confirmed");
    assert!(bus.tx_log().iter().any(|f| f.data == reg_write(7, damiao::RID_CTRL_MODE, 4)));
    assert!(d.motor_current_cap().unwrap().active);
}

#[test]
fn recovery_hold_of_a_stale_j7_is_capped() {
    let (mut d, bus, clock) = rig(true);
    d.set_torque(true).unwrap();
    d.write_joint_targets(&targets(-1.0)).unwrap();
    bus.with(|s| s.motors[J7].mute_feedback = true);
    clock.advance(Duration::from_millis(50));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    bus.clear_log();
    let _ = d.read_joint_states();
    let _ = d.read_joint_states();
    let j7 = j7_commands(&bus.tx_log());
    assert!(!j7.is_empty(), "no recovery frame for the stale J7: {:?}", bus.tx_log());
    assert!(j7.iter().all(|f| f.id == 0x307 && u16::from_le_bytes([f.data[6], f.data[7]]) == RAW), "{j7:?}");
}

#[test]
fn legacy_pos_vel_profile_reports_no_current_cap() {
    let (mut d, bus, _) = rig(false);
    d.set_torque(true).unwrap();
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[3] == damiao::RID_CTRL_MODE), "legacy enable touched CTRL_MODE");
    let cap = d.motor_current_cap().unwrap();
    assert_eq!((cap.mode, cap.current_raw, cap.active), ("pos_vel", None, false));
    assert_eq!(cap.note, "no motor current cap (legacy POS_VEL gripper)");
}

/// Plant with actuator-current saturation: closing onto a jaw stop, the pos-force gripper's torque
/// stays at the current ceiling; the same close in POS_VEL reaches stiffness x penetration.
#[test]
fn closing_onto_a_stop_saturates_at_the_current_ceiling() {
    let peak = |pos_force: bool| -> f32 {
        let (mut d, bus, _) = rig(pos_force);
        d.set_torque(true).unwrap();
        bus.with(|s| s.motors[J7].stop_at = Some(-0.5));
        let mut peak = 0.0f32;
        for k in 0..20 {
            let q = -1.0 + 0.05 * k as f32; // closing past the stop at -0.5
            d.write_joint_targets(&targets(q)).unwrap();
            let s = d.read_joint_states().unwrap();
            peak = peak.max(s[J7].torque.abs());
        }
        peak
    };
    let capped = peak(true);
    let legacy = peak(false);
    let ceiling = f32::from(RAW) * 0.0007;
    eprintln!("closing onto a stop: pos-force peak {capped:.3} N·m (ceiling {ceiling:.3}), POS_VEL peak {legacy:.3} N·m");
    assert!(capped <= ceiling + 0.01, "pos-force peak {capped} above the ceiling {ceiling}");
    assert!(capped > 0.5 * ceiling, "the stop was not reached: {capped}");
    assert!(legacy > 10.0 * ceiling, "POS_VEL plant not uncapped: {legacy}");
}

#[test]
fn profile_selects_the_gripper_drive() {
    let cfg = ArmConfig::rebot_dm();
    let g = cfg.damiao_gripper.clone().unwrap();
    assert_eq!((g.mode, g.current_ceiling_raw, g.calibrated), (DamiaoGripperMode::PosForce, 220, false));
    match gripper_drive_from_profile(&cfg).unwrap() {
        Some(GripperDrive::PosForce(p)) => assert_eq!((p.joint, p.current_raw, p.close_speed_rad_s, p.closed_rad), (6, 220, 0.30, cfg.gripper_policy.as_ref().unwrap().closed_rad)),
        other => panic!("{other:?}"),
    }
    let legacy = ArmConfig { damiao_gripper: Some(DamiaoGripperDrive::legacy_pos_vel()), ..ArmConfig::rebot_dm() };
    assert_eq!(gripper_drive_from_profile(&legacy).unwrap(), Some(GripperDrive::PosVel(6)));
    let mut bad = ArmConfig { damiao_gripper: Some(DamiaoGripperDrive { current_ceiling_raw: 0, ..DamiaoGripperDrive::rebot_dm() }), ..ArmConfig::rebot_dm() };
    assert!(bad.validate().is_err());
    let mut uncal = ArmConfig { damiao_gripper: Some(DamiaoGripperDrive { calibrated: true, ..DamiaoGripperDrive::rebot_dm() }), ..ArmConfig::rebot_dm() };
    assert!(uncal.validate().is_err(), "calibrated without a torque equivalent");
    ArmConfig::rebot_dm().validate().expect("default profile valid");
}

#[test]
fn safety_describe_reports_the_gripper_current_cap() {
    use armd::supervisor::SupervisorInner;
    // profile only (no driver): configured, not applied
    let inner = SupervisorInner::new(ArmConfig::rebot_dm());
    let cap = &inner.safety_describe()["hardware"]["gripper_current_cap"];
    assert_eq!(cap["mode"], "pos_force");
    assert_eq!(cap["current_raw"], 220);
    assert_eq!(cap["calibrated"], false);
    assert_eq!(cap["active"], false);
    // with the pos-force driver after a confirmed enable
    let (mut d, _bus, _) = rig(true);
    d.set_torque(true).unwrap();
    let inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(d));
    let cap = &inner.safety_describe()["hardware"]["gripper_current_cap"];
    assert_eq!(cap["active"], true, "{cap}");
    assert_eq!(cap["calibrated"], false);
    // legacy profile
    let cfg = ArmConfig { damiao_gripper: Some(DamiaoGripperDrive::legacy_pos_vel()), ..ArmConfig::rebot_dm() };
    let cap = SupervisorInner::new(cfg).safety_describe()["hardware"]["gripper_current_cap"].clone();
    assert_eq!(cap["note"], "no motor current cap (legacy POS_VEL gripper)");
}
