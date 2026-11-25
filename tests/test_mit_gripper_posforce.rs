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
    let payload = damiao::encode_pos_force_payload(1.0, damiao::pos_force_vel_raw(0.30), 220);
    assert_eq!(payload, [0x00, 0x00, 0x80, 0x3F, 0x1E, 0x00, 0xDC, 0x00]);
    // Verify the single emitter actually transmits CAN ID 0x307 with this exact payload layout.
    let (mut d, bus, _) = rig(true);
    bus.with(|s| s.motors[J7].q = -1.0);
    d.read_joint_states().unwrap();
    d.set_torque(true).expect("enable");
    bus.clear_log();
    let mut t = targets(1.0);
    t[J7].velocity = Some(0.30);
    d.begin_tx_tick();
    d.write_joint_targets_bounded(&t).expect("write");
    let j7 = j7_commands(&bus.tx_log());
    assert_eq!(j7.len(), 1);
    assert_eq!(j7[0].id, 0x307);
    assert_eq!(j7[0].data, [0x00, 0x00, 0x80, 0x3F, 0x1E, 0x00, 0xDC, 0x00]);
    assert_eq!(damiao::pos_force_vel_raw(2.0), 200);
    assert_eq!(damiao::pos_force_vel_raw(-0.304), 30);
    assert_eq!(damiao::pos_force_vel_raw(0.306), 30, "must quantize downward so wire speed <= ceiling");
    assert_eq!(damiao::pos_force_vel_raw(0.3099), 30);
    assert_eq!(damiao::pos_force_vel_raw(0.29), 28, "0.29_f32 is 0.28999999...; strict f64 floor avoids exceeding speed_rad_s");
    for v in [0.29_f32, 0.30, 0.304, 0.306, 0.3099, 2.0] {
        let raw = damiao::pos_force_vel_raw(v);
        assert!(f64::from(raw) / 100.0 <= f64::from(v.abs()), "v={v} raw={raw} exceeded speed_rad_s");
    }
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
    let hold = log.iter().position(|f| f.id == 0x307).expect("pre-enable pos-force hold");
    let enable = log.iter().position(|f| f.id == 7 && f.data == damiao::CMD_ENABLE).expect("enable frame");
    assert!(write < read && read < hold && hold < enable, "order write {write} < read {read} < hold {hold} < enable {enable}: {log:?}");
    let cmds = j7_commands(&log);
    assert_eq!(cmds.len(), 1, "only the pre-enable hold frame before enable: {cmds:?}");
    let hold_pos = f32::from_le_bytes([cmds[0].data[0], cmds[0].data[1], cmds[0].data[2], cmds[0].data[3]]);
    let hold_vel = u16::from_le_bytes([cmds[0].data[4], cmds[0].data[5]]);
    let hold_cur = u16::from_le_bytes([cmds[0].data[6], cmds[0].data[7]]);
    assert!((hold_pos - (-1.0)).abs() < 1e-3, "hold position {hold_pos}");
    assert_eq!(hold_vel, 0, "pre-enable hold must be zero-velocity Hold (vel_raw=0), got {hold_vel}");
    assert_eq!(hold_cur, RAW);
    assert_eq!(log.iter().filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE).count(), 1, "only J7's mode is written");
    assert_eq!(bus.with(|s| s.motors[J7].ctrl_mode()), damiao::CTRL_MODE_POS_FORCE);
    assert_eq!(d.joint_control_modes()[J7], Some(DamiaoControlMode::PosForce));
    let cap = d.motor_current_cap().unwrap();
    assert!(cap.active && !cap.calibrated && cap.mode == "pos_force" && cap.current_raw == Some(RAW), "{cap:?}");
    assert!(cap.note.contains("uncalibrated"));
}

#[test]
fn enable_after_restart_with_j7_already_in_mode_4_emits_readback_hold_enable() {
    // Motor stayed powered across a host restart: J7 is already in CTRL_MODE 4 with a stale
    // firmware target at 0.0 (closed), while the jaw is physically open at -1.75 rad.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
    bus.with(|s| {
        for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; }
        s.motors[J7].q = -1.75;
        s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
        s.motors[J7].last_pos_force = Some((0.0, 200, RAW));
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits()).unwrap()
        .with_gripper_pos_force(GripperPosForce { close_speed_rad_s: 0.306, ..pf() }).unwrap();
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    bus.clear_log();

    // Before enable, readback has not run yet so capped operation is unavailable.
    assert!(!d.motor_current_cap().unwrap().active);
    assert!(d.write_joint_targets(&targets(-1.75)).is_err());

    d.set_torque(true).expect("enable after restart");
    let log = bus.tx_log();
    assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "mode rewritten when already 4: {log:?}");
    let read = log.iter().position(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)).expect("readback");
    let hold = log.iter().position(|f| f.id == 0x307).expect("pre-enable hold");
    let enable = log.iter().position(|f| f.id == 7 && f.data == damiao::CMD_ENABLE).expect("enable");
    assert!(read < hold && hold < enable, "order read {read} < hold {hold} < enable {enable}: {log:?}");
    let cmds = j7_commands(&log);
    assert_eq!(cmds.len(), 1, "{cmds:?}");
    let hold_pos = f32::from_le_bytes([cmds[0].data[0], cmds[0].data[1], cmds[0].data[2], cmds[0].data[3]]);
    let hold_vel = u16::from_le_bytes([cmds[0].data[4], cmds[0].data[5]]);
    let hold_cur = u16::from_le_bytes([cmds[0].data[6], cmds[0].data[7]]);
    assert!((hold_pos - (-1.75)).abs() < 1e-3, "hold position {hold_pos}");
    assert_eq!(hold_vel, 0, "pre-enable hold must be zero-velocity Hold (vel_raw=0), got {hold_vel}");
    assert_eq!(hold_cur, RAW);
    let last_pf = bus.with(|s| s.motors[J7].last_pos_force).expect("motor received pre-enable hold");
    assert!((last_pf.0 - (-1.75)).abs() < 1e-3 && last_pf.1 == 0 && last_pf.2 == RAW, "{last_pf:?}");
    assert!(d.is_enabled() && d.motor_current_cap().unwrap().active);
    // Subsequent closing target still quantizes 0.306 downward to 30 (0.30 rad/s).
    bus.clear_log();
    d.write_joint_targets(&targets(-1.0)).expect("closing target");
    let close_cmds = j7_commands(&bus.tx_log());
    assert_eq!(close_cmds.len(), 1);
    let close_vel = u16::from_le_bytes([close_cmds[0].data[4], close_cmds[0].data[5]]);
    assert_eq!(close_vel, 30, "closing target must quantize 0.306 downward to 30 (0.30 rad/s), got {close_vel}");

    // Without fresh J7 feedback after restart (mode already 4), enable must fail closed before sending a hold or enable.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
    bus.with(|s| {
        s.motors[J7].q = -1.75;
        s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits()).unwrap()
        .with_gripper_pos_force(pf()).unwrap();
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    clock.advance(Duration::from_millis(200));
    bus.with(|s| s.motors[J7].mute_feedback = true);
    bus.clear_log();
    let err = d.set_torque(true).unwrap_err().to_string();
    assert!(err.contains("no fresh measured position"), "{err}");
    assert!(j7_commands(&bus.tx_log()).is_empty(), "sent J7 hold on stale feedback");
    assert!(!bus.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE), "sent enable on stale feedback");
    assert!(!d.is_enabled());

    // A frame queued BEFORE the driver's first empty-queue observation has unknown age and must not
    // supply the pre-enable hold position on restart when the motor does not answer 0xCC refresh.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock);
    bus.with(|s| {
        s.motors[J7].q = -1.75;
        s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
        s.push_all_feedback();
        s.motors[J7].mute_feedback = true;
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits()).unwrap()
        .with_gripper_pos_force(pf()).unwrap();
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    bus.clear_log();
    let err = d.set_torque(true).unwrap_err().to_string();
    assert!(err.contains("no fresh measured position"), "accepted unknown-age startup sample: {err}");
    assert!(bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC), "expected 0xCC refresh attempt");
    assert!(j7_commands(&bus.tx_log()).is_empty(), "sent J7 hold on unknown-age startup sample");
    assert!(!bus.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE));
    assert!(!d.is_enabled());

    // Repeated set_torque(true) when J7 feedback is 150 ms old (> 100 ms pos-force bound, < 250 ms
    // driver bound) while J1-J6 stay fresh at state 1 must NOT take the fast path: it fails closed
    // without sending a hold or enable frame.
    let (mut d, bus, clock) = rig(true);
    d.set_torque(true).unwrap();
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    bus.with(|s| s.motors[J7].mute_feedback = true);
    clock.advance(Duration::from_millis(150));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    d.read_joint_states().unwrap();
    assert!(d.is_feedback_valid(), "J7 at 150 ms is still within 250 ms driver bound");
    bus.clear_log();
    let err = d.set_torque(true).unwrap_err().to_string();
    assert!(err.contains("no fresh measured position"), "took fast path past 100 ms J7 bound: {err}");
    assert!(j7_commands(&bus.tx_log()).is_empty(), "sent J7 hold with 150 ms stale feedback");
    assert!(!bus.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE));
    assert!(!d.is_enabled() && !d.motor_current_cap().unwrap().active);

    // check_mapping_ranges must NOT promote a PosForce joint from a wrong-CAN-ID CTRL_MODE=4 reply,
    // and must NOT promote an uncertain non-PosForce joint back to Some(PosVel).
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock);
    bus.with(|s| {
        s.motors[J7].q = -1.75;
        s.motors[J7].reply_id = 0x16; // wrong MST_ID for J7
        s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits()).unwrap()
        .with_gripper_pos_force(pf()).unwrap();
    d.check_mapping_ranges();
    assert_eq!(d.joint_control_modes()[J7], None, "promoted J7 to PosForce from wrong CAN ID 0x16");

    let (mut d_leg, bus_leg, _) = rig(false);
    // Force J1 mode to None via a failed mode switch, then run check_mapping_ranges: J1 must stay None.
    bus_leg.with(|s| s.motors[0].no_write_ack = true);
    let mut mit_j1 = vec![armd::driver::JointMode::Position; 7];
    mit_j1[0] = armd::driver::JointMode::Mit;
    assert!(d_leg.set_joint_modes(&mit_j1, &targets(-1.0)).is_err());
    assert_eq!(d_leg.joint_control_modes()[0], None);
    d_leg.check_mapping_ranges();
    assert_eq!(d_leg.joint_control_modes()[0], None, "check_mapping_ranges promoted uncertain non-PosForce joint");
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
    //    must not spoof fresh feedback during the pre-write drain when the motor does not answer 0xCC.
    let (mut d, bus, clock) = rig(true);
    bus.with(|s| { s.push_feedback(J7); s.motors[J7].mute_feedback = true; });
    clock.advance(Duration::from_millis(200));
    bus.clear_log();
    let e = d.set_torque(true).unwrap_err().to_string();
    assert!(e.contains("fresh false"), "{e}");
    assert!(bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC), "expected 0xCC refresh attempt");
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "mode written on aged queued frame");

    // 4. A stationary state-0 frame already sitting in RX BEFORE the driver's first observed empty
    //    queue has unknown age and must not authorize a mode write when the motor does not answer 0xCC.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock);
    bus.with(|s| { s.motors[J7].q = -1.0; s.push_all_feedback(); s.motors[J7].mute_feedback = true; });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits()).unwrap()
        .with_gripper_pos_force(pf()).unwrap();
    bus.clear_log();
    let e = d.set_torque(true).unwrap_err().to_string();
    assert!(e.contains("fresh false"), "accepted pre-boundary startup sample: {e}");
    assert!(bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC), "expected 0xCC refresh attempt");
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
    let (mut d, bus, clock) = rig(true);
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..6 {
        arm_joints[i] = i;
    }
    let mut rest_positions = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..6 {
        rest_positions[i] = 0.1 * i as f32;
    }
    rest_positions[J7] = -1.0;
    d.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions,
        rest_pose_tolerance_rad: 0.05,
    });
    let collect_k3 = |d: &mut DamiaoArmDriver<FakeBus>| {
        clock.advance(Duration::from_millis(21));
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(11));
            }
            bus.with(|s| s.push_all_feedback());
            d.read_joint_states().unwrap();
        }
    };
    d.set_torque(true).unwrap();
    collect_k3(&mut d);
    d.set_torque(false).unwrap();
    bus.clear_log();
    d.set_torque(true).unwrap();
    let log = bus.tx_log();
    assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "mode rewritten on a re-enable");
    assert_eq!(log.iter().filter(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)).count(), 1, "readback on the re-enable");
    // the motor lost its RAM mode (power cycle): the readback fails, the next enable rewrites it
    collect_k3(&mut d);
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
fn readback_requires_motor_master_can_id_and_ctrl_mode_rid() {
    let reg_reply = |can_id: u16, motor_id: u16, op: u8, rid: u8, val: u32| -> CanFrame {
        let [lo, hi] = motor_id.to_le_bytes();
        let b = val.to_le_bytes();
        CanFrame::new_standard(can_id, &[lo, hi, op, rid, b[0], b[1], b[2], b[3]]).unwrap()
    };
    // 1. Spoofed CTRL_MODE=4 readback arriving on wrong CAN arbitration IDs (0x16, 0x11, 0x00)
    //    must be rejected (J7 expects MST_ID 0x17).
    for wrong_id in [0x16u16, 0x11, 0x00] {
        let (mut d, bus, clock) = rig(true);
        bus.with(|s| s.motors[J7].silent_after_mode_ack = true);
        let mut p = d.begin_torque(true).unwrap();
        let mut injected = false;
        let err = loop {
            match p {
                armd::driver::TorqueProgress::Done(_) => panic!("accepted spoofed CAN ID 0x{wrong_id:X}"),
                armd::driver::TorqueProgress::Pending { .. } => {
                    if !injected && bus.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) {
                        injected = true;
                        bus.with(|s| s.rx.push_back(reg_reply(wrong_id, 7, damiao::REG_READ, damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE)));
                    }
                    clock.advance(Duration::from_micros(500));
                    match d.poll_torque(clock.now() + Duration::from_millis(5)) {
                        Ok(next) => p = next,
                        Err(e) => break e,
                    }
                }
            }
        };
        assert!(injected, "read request was never sent");
        assert!(err.to_string().contains("no fresh reply"), "wrong_id=0x{wrong_id:X}: {err}");
        assert!(!d.is_enabled());
        assert!(!d.motor_current_cap().unwrap().active);
        assert!(!bus.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE));
    }

    // 2. Reply on J7's expected CAN ID 0x17 carrying value 4, but wrong register ID (RID_TIMEOUT)
    //    or wrong opcode (REG_WRITE) must be rejected.
    for (op, rid) in [(damiao::REG_READ, damiao::RID_TIMEOUT), (damiao::REG_WRITE, damiao::RID_CTRL_MODE)] {
        let (mut d, bus, clock) = rig(true);
        bus.with(|s| s.motors[J7].silent_after_mode_ack = true);
        let mut p = d.begin_torque(true).unwrap();
        let mut injected = false;
        let err = loop {
            match p {
                armd::driver::TorqueProgress::Done(_) => panic!("accepted wrong op/rid ({op:#x}, {rid})"),
                armd::driver::TorqueProgress::Pending { .. } => {
                    if !injected && bus.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) {
                        injected = true;
                        bus.with(|s| s.rx.push_back(reg_reply(0x17, 7, op, rid, damiao::CTRL_MODE_POS_FORCE)));
                    }
                    clock.advance(Duration::from_micros(500));
                    match d.poll_torque(clock.now() + Duration::from_millis(5)) {
                        Ok(next) => p = next,
                        Err(e) => break e,
                    }
                }
            }
        };
        assert!(injected);
        assert!(err.to_string().contains("no fresh reply"), "op={op:#x} rid={rid}: {err}");
        assert!(!d.is_enabled());
    }

    // 3. Configured master ID (`with_master_id(0x27)`): keep telemetry and write ack on 0x27,
    //    vary only the readback reply ID — default 0x17 must time out after the read request,
    //    while configured 0x27 succeeds.
    let make_custom = |reply_id: u16| -> (DamiaoArmDriver<FakeBus>, FakeBus, MockClock) {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; }
            s.motors[J7].q = -1.0;
            s.motors[J7].reply_id = reply_id;
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_master_id(0x27)
            .with_joint_limits(b601_limits()).unwrap()
            .with_gripper_pos_force(pf()).unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        bus.clear_log();
        (d, bus, clock)
    };
    let (mut d_bad, bus_bad, clock_bad) = make_custom(0x27);
    bus_bad.with(|s| s.motors[J7].silent_after_mode_ack = true);
    let mut p = d_bad.begin_torque(true).unwrap();
    let mut injected = false;
    let err = loop {
        match p {
            armd::driver::TorqueProgress::Done(_) => panic!("accepted 0x17 readback when master_id is 0x27"),
            armd::driver::TorqueProgress::Pending { .. } => {
                if !injected && bus_bad.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) {
                    injected = true;
                    bus_bad.with(|s| s.rx.push_back(reg_reply(0x17, 7, damiao::REG_READ, damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE)));
                }
                clock_bad.advance(Duration::from_micros(500));
                match d_bad.poll_torque(clock_bad.now() + Duration::from_millis(5)) {
                    Ok(next) => p = next,
                    Err(e) => break e,
                }
            }
        }
    };
    assert!(injected, "readback request was not sent under configured master_id");
    assert!(err.to_string().contains("no fresh reply"), "{err}");
    assert!(j7_commands(&bus_bad.tx_log()).is_empty());
    assert!(!bus_bad.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE));
    assert!(!d_bad.is_enabled());

    let (mut d_ok, _, _) = make_custom(0x27);
    d_ok.set_torque(true).expect("accepted configured master_id 0x27");
    assert!(d_ok.is_enabled() && d_ok.motor_current_cap().unwrap().active);
}

#[test]
fn readback_reply_at_or_after_50ms_deadline_times_out_fail_closed() {
    let valid_reply = || {
        let b = damiao::CTRL_MODE_POS_FORCE.to_le_bytes();
        CanFrame::new_standard(0x17, &[7, 0, damiao::REG_READ, damiao::RID_CTRL_MODE, b[0], b[1], b[2], b[3]]).unwrap()
    };
    let step_to_readback_sent = || -> (DamiaoArmDriver<FakeBus>, FakeBus, MockClock, std::time::Instant) {
        let (mut d, bus, clock) = rig(true);
        bus.with(|s| s.motors[J7].silent_after_mode_ack = true);
        let mut p = d.begin_torque(true).unwrap();
        loop {
            match p {
                armd::driver::TorqueProgress::Pending { wake } if bus.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) => {
                    return (d, bus, clock, wake);
                }
                armd::driver::TorqueProgress::Pending { .. } => {
                    clock.advance(Duration::from_micros(500));
                    p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
                }
                armd::driver::TorqueProgress::Done(_) => panic!("unexpectedly finished before readback"),
            }
        }
    };

    // 1. Reply sitting in RX at exact 50 ms deadline or 10 ms past it must time out (fail closed).
    for past in [Duration::ZERO, Duration::from_millis(10)] {
        let (mut d, bus, clock, deadline) = step_to_readback_sent();
        bus.with(|s| { s.motors[J7].silent = false; s.rx.push_back(valid_reply()); });
        let dt = deadline.saturating_duration_since(clock.now()) + past;
        clock.advance(dt);
        let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
        assert!(err.contains("no fresh reply within 50 ms"), "past={past:?}: {err}");
        assert!(!d.is_enabled());
        assert!(!d.motor_current_cap().unwrap().active);
        assert!(!bus.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE));
        assert!(d.write_joint_targets(&targets(-0.5)).is_err());
    }

    // 2. Reply arriving 1 ms before the 50 ms deadline is accepted and enables.
    let (mut d, bus, clock, deadline) = step_to_readback_sent();
    bus.with(|s| { s.motors[J7].silent = false; s.rx.push_back(valid_reply()); });
    let dt = deadline.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1));
    clock.advance(dt);
    let mut p = d.poll_torque(clock.now() + Duration::from_millis(5)).expect("reply before 50 ms deadline accepted");
    while let armd::driver::TorqueProgress::Pending { .. } = p {
        clock.advance(Duration::from_micros(500));
        p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
    }
    assert!(d.is_enabled() && d.motor_current_cap().unwrap().active);
}

#[test]
fn timely_readback_and_restore_replies_accepted_at_expiry_wake() {
    let reg_reply = |op: u8, val: u32| {
        let b = val.to_le_bytes();
        CanFrame::new_standard(0x17, &[7, 0, op, damiao::RID_CTRL_MODE, b[0], b[1], b[2], b[3]]).unwrap()
    };

    // 1. Readback: reply arrives and is recorded by an intervening read 1 ms before expiry,
    //    then processed at the exact advertised expiry wake -> accepted and enables cleanly.
    let (mut d, bus, clock) = rig(true);
    bus.with(|s| s.motors[J7].silent_after_mode_ack = true);
    let mut p = d.begin_torque(true).unwrap();
    let wake = loop {
        match p {
            armd::driver::TorqueProgress::Pending { wake } if bus.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) => {
                break wake;
            }
            armd::driver::TorqueProgress::Pending { .. } => {
                clock.advance(Duration::from_micros(500));
                p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
            }
            armd::driver::TorqueProgress::Done(_) => panic!("finished before readback"),
        }
    };
    let to_pre = wake.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1));
    clock.advance(to_pre);
    bus.with(|s| {
        s.motors[J7].silent = false;
        s.rx.push_back(reg_reply(damiao::REG_READ, damiao::CTRL_MODE_POS_FORCE));
    });
    // Intervening receive records the reply at wake - 1 ms (strictly before deadline).
    d.read_joint_states().unwrap();
    // Advance to the exact advertised expiry wake and poll: timely reply must be accepted.
    clock.advance(Duration::from_millis(1));
    assert_eq!(clock.now(), wake);
    let mut p = d.poll_torque(clock.now() + Duration::from_millis(5))
        .expect("timely readback recorded 1 ms before expiry must be accepted at expiry wake");
    while let armd::driver::TorqueProgress::Pending { .. } = p {
        clock.advance(Duration::from_micros(500));
        p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
    }
    assert!(d.is_enabled() && d.motor_current_cap().unwrap().active);

    // 2. Readback negative control: a reply read AT the exact expiry wake (rx_at == deadline)
    //    is not timely and must still time out fail-closed.
    let (mut d, bus, clock) = rig(true);
    bus.with(|s| s.motors[J7].silent_after_mode_ack = true);
    let mut p = d.begin_torque(true).unwrap();
    let wake = loop {
        match p {
            armd::driver::TorqueProgress::Pending { wake } if bus.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) => {
                break wake;
            }
            armd::driver::TorqueProgress::Pending { .. } => {
                clock.advance(Duration::from_micros(500));
                p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
            }
            armd::driver::TorqueProgress::Done(_) => panic!("finished before readback"),
        }
    };
    clock.advance(wake.saturating_duration_since(clock.now()));
    bus.with(|s| {
        s.motors[J7].silent = false;
        s.rx.push_back(reg_reply(damiao::REG_READ, damiao::CTRL_MODE_POS_FORCE));
    });
    d.read_joint_states().unwrap(); // rx_at == wake (at deadline, not before)
    let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
    assert!(err.contains("no fresh reply within 50 ms"), "{err}");
    assert!(!d.is_enabled());

    // 3. Restore: mode-write ACK arrives and is recorded 1 ms before expiry, then processed at
    //    the exact advertised restore expiry wake -> accepted, proceeds to readback and enables.
    let (mut d, bus, clock) = rig(true);
    bus.with(|s| s.motors[J7].no_write_ack = true);
    let p = d.begin_torque(true).unwrap();
    assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
    // First poll observes no ACK yet and advertises restore expiry wake.
    let wake = match d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap() {
        armd::driver::TorqueProgress::Pending { wake } => wake,
        other => panic!("expected Pending restore wake, got {other:?}"),
    };
    let to_pre = wake.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1));
    clock.advance(to_pre);
    bus.with(|s| {
        s.motors[J7].no_write_ack = false;
        s.rx.push_back(reg_reply(damiao::REG_WRITE, damiao::CTRL_MODE_POS_FORCE));
    });
    d.read_joint_states().unwrap(); // records ACK at wake - 1 ms
    clock.advance(Duration::from_millis(1));
    assert_eq!(clock.now(), wake);
    let mut p = d.poll_torque(clock.now() + Duration::from_millis(5))
        .expect("timely restore ACK recorded 1 ms before expiry must be accepted at expiry wake");
    // Refresh J7 feedback so the subsequent readback -> pre-enable hold stays within 100 ms.
    bus.with(|s| s.push_all_feedback());
    while let armd::driver::TorqueProgress::Pending { .. } = p {
        clock.advance(Duration::from_micros(500));
        p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
    }
    assert!(d.is_enabled() && d.motor_current_cap().unwrap().active);

    // 4. Restore negative control: no timely ACK before expiry wake (ACK sits unread at wake) -> fails closed.
    let (mut d, bus, clock) = rig(true);
    bus.with(|s| s.motors[J7].no_write_ack = true);
    let _ = d.begin_torque(true).unwrap();
    let wake = match d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap() {
        armd::driver::TorqueProgress::Pending { wake } => wake,
        other => panic!("expected Pending restore wake, got {other:?}"),
    };
    clock.advance(wake.saturating_duration_since(clock.now()));
    bus.with(|s| s.rx.push_back(reg_reply(damiao::REG_WRITE, damiao::CTRL_MODE_POS_FORCE)));
    let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
    assert!(err.contains("could not be restored before enable"), "{err}");
    assert!(!d.is_enabled());
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
    let close_raw = damiao::pos_force_vel_raw(0.30);
    assert!(j7.iter().all(|f| {
        f.id == 0x307
            && u16::from_le_bytes([f.data[4], f.data[5]]) <= close_raw
            && u16::from_le_bytes([f.data[6], f.data[7]]) == RAW
    }), "{j7:?}");
}

#[test]
fn stale_j7_recovery_caps_speed_and_fails_closed_without_fresh_feedback() {
    let (mut d, bus, clock) = rig(true);
    d.set_torque(true).unwrap();
    d.write_joint_targets(&targets(-1.0)).unwrap();
    // Drain both the empty-queue boundary and the pending replies of write_joint_targets at t=0.
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    let close_raw = damiao::pos_force_vel_raw(0.30);

    // 1. Brief 50 ms gap (still within 100 ms freshness bound): recovery hold stays at <= close_speed (30, not 200).
    bus.with(|s| s.motors[J7].mute_feedback = true);
    clock.advance(Duration::from_millis(50));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    bus.clear_log();
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    let j7 = j7_commands(&bus.tx_log());
    assert!(!j7.is_empty(), "expected capped hold at 50 ms");
    for f in &j7 {
        let pos = f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]);
        let vel = u16::from_le_bytes([f.data[4], f.data[5]]);
        let cur = u16::from_le_bytes([f.data[6], f.data[7]]);
        assert_eq!(f.id, 0x307);
        assert!((pos - (-1.0)).abs() < 1e-3, "pos={pos}");
        assert!(vel <= close_raw, "recovery bypassed close_speed: vel_raw={vel} > {close_raw}");
        assert_eq!(cur, RAW);
    }

    // 2. Past 100 ms freshness bound (120 ms, still < 250 ms so arm stays enabled):
    //    no fresh J7 feedback -> recovery must emit no J7 motion command (fail closed; 0xCC refresh only).
    clock.advance(Duration::from_millis(70));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    bus.clear_log();
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    let log = bus.tx_log();
    assert!(j7_commands(&log).is_empty(), "emitted J7 motion frame without fresh feedback: {:?}", j7_commands(&log));
    assert!(log.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC), "expected 0xCC refresh: {log:?}");

    // 3. While J7 feedback is stale (> 100 ms), unfenced write_joint_targets NEVER skips J7 and
    //    NEVER sends the new target (-2.0): it sends the 0x307 FREEZE frame (pos = last fresh
    //    measured J7 position -1.0, vel_raw = 0, same current cap), lets J1-J6 continue, emits
    //    a J7 0xCC refresh, and reports Stale via take_last_pos_force_tx_status.
    bus.clear_log();
    d.begin_tx_tick();
    d.write_joint_targets(&targets(-2.0)).unwrap();
    let stale_log = bus.tx_log();
    let stale_j7 = j7_commands(&stale_log);
    assert_eq!(stale_j7.len(), 1, "expected 1 0x307 freeze frame while stale: {stale_j7:?}");
    assert_eq!(stale_j7[0].id, 0x307);
    let freeze_pos = f32::from_le_bytes(stale_j7[0].data[0..4].try_into().unwrap());
    let freeze_vel = u16::from_le_bytes([stale_j7[0].data[4], stale_j7[0].data[5]]);
    let freeze_cur = u16::from_le_bytes([stale_j7[0].data[6], stale_j7[0].data[7]]);
    assert!((freeze_pos - (-1.0)).abs() < 1e-3, "freeze must hold last fresh pos -1.0, got {freeze_pos}");
    assert_eq!(freeze_vel, 0, "freeze frame must carry vel_raw == 0");
    assert_eq!(freeze_cur, RAW, "freeze frame must preserve current cap");
    assert!(
        (1..=6u32).all(|id| stale_log.iter().any(|f| f.id == 0x100 + id)),
        "expected J1-J6 commands to continue while J7 is stale: {stale_log:?}"
    );
    assert!(
        stale_log.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
        "expected J7 0xCC refresh during stale outage: {stale_log:?}"
    );
    assert!(
        matches!(d.take_last_pos_force_tx_status(), armd::driver::PosForceTxStatus::Stale { joint: J7, .. }),
        "expected Stale status after emitting J7 freeze frame"
    );

    // 4. Fresh feedback returns with jaw physically opened to -2.5: ramping back toward -1.0 closes at <= close_speed;
    //    a non-finite J7 target refuses the entire batch before sending J1-J6; strictly opening allows opening speed.
    bus.with(|s| {
        s.motors[J7].mute_feedback = false;
        s.motors[J7].q = -2.5;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();
    bus.clear_log();
    let nan_err = d.write_joint_targets(&targets(f32::NAN)).unwrap_err().to_string();
    assert!(nan_err.contains("not finite"), "{nan_err}");
    assert!(bus.tx_log().is_empty(), "partial J1-J6 batch sent on non-finite J7 target: {:?}", bus.tx_log());

    d.write_joint_targets(&targets(-1.0)).unwrap();
    let close_cmd = j7_commands(&bus.tx_log());
    assert_eq!(close_cmd.len(), 1);
    assert_eq!(u16::from_le_bytes([close_cmd[0].data[4], close_cmd[0].data[5]]), close_raw);
    assert_eq!(u16::from_le_bytes([close_cmd[0].data[6], close_cmd[0].data[7]]), RAW);

    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    bus.clear_log();
    d.write_joint_targets(&targets(-2.0)).unwrap();
    let open_cmd = j7_commands(&bus.tx_log());
    assert_eq!(open_cmd.len(), 1);
    assert_eq!(u16::from_le_bytes([open_cmd[0].data[4], open_cmd[0].data[5]]), damiao::pos_force_vel_raw(2.0));

    // 5. Non-0.01-multiple close_speed_rad_s (0.306) must quantize downward to raw 30 (0.30 <= 0.306),
    //    never rounding up to raw 31 (0.31 > 0.306) on closing frames or stale recovery holds.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
    bus.with(|s| { s.motors[J7].q = -1.0; });
    let mut cfg = pf();
    cfg.close_speed_rad_s = 0.306;
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits()).unwrap()
        .with_gripper_pos_force(cfg).unwrap();
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    d.set_torque(true).unwrap();
    bus.clear_log();
    d.write_joint_targets(&targets(-0.5)).unwrap();
    let close_306 = j7_commands(&bus.tx_log());
    assert_eq!(u16::from_le_bytes([close_306[0].data[4], close_306[0].data[5]]), 30);
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    bus.with(|s| s.motors[J7].mute_feedback = true);
    clock.advance(Duration::from_millis(50));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    bus.clear_log();
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    let rec_306 = j7_commands(&bus.tx_log());
    assert!(!rec_306.is_empty());
    assert!(rec_306.iter().all(|f| u16::from_le_bytes([f.data[4], f.data[5]]) == 30), "{rec_306:?}");
}

#[test]
fn pos_force_mid_batch_expiry_and_invalid_handover_fail_closed() {
    use armd::driver::JointMode;

    // 1. Mid-batch expiry: feedback is 99 ms old at batch entry, J1-J6 take 1 ms each ->
    //    J7 is 105 ms old when its frame is built -> unfenced write_joint_targets sends J1-J6,
    //    emits a 0x307 FREEZE frame (pos = last fresh measured J7, vel_raw = 0, NOT -2.0!),
    //    requests a J7 0xCC refresh, and reports Stale.
    let (mut d, bus, clock) = rig(true);
    d.set_torque(true).unwrap();
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    let last_fresh_j7 = bus.with(|s| s.motors[J7].q);
    clock.advance(Duration::from_millis(99));
    bus.with(|s| s.tx_step = Duration::from_millis(1));
    bus.clear_log();
    d.begin_tx_tick();
    d.write_joint_targets(&targets(-2.0)).unwrap();
    let mid_log = bus.tx_log();
    let mid_j7 = j7_commands(&mid_log);
    assert_eq!(mid_j7.len(), 1, "expected 1 0x307 freeze frame after mid-batch expiry: {mid_j7:?}");
    assert_eq!(mid_j7[0].id, 0x307);
    let mid_pos = f32::from_le_bytes(mid_j7[0].data[0..4].try_into().unwrap());
    let mid_vel = u16::from_le_bytes([mid_j7[0].data[4], mid_j7[0].data[5]]);
    let mid_cur = u16::from_le_bytes([mid_j7[0].data[6], mid_j7[0].data[7]]);
    assert!((mid_pos - last_fresh_j7).abs() < 1e-3, "mid-batch freeze pos={mid_pos} != {last_fresh_j7}");
    assert_eq!(mid_vel, 0, "mid-batch freeze must carry vel_raw == 0");
    assert_eq!(mid_cur, RAW);
    assert!(
        (1..=6u32).all(|id| mid_log.iter().any(|f| f.id == 0x100 + id)),
        "expected J1-J6 to be sent before J7 expired: {mid_log:?}"
    );
    assert!(
        mid_log.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
        "expected J7 0xCC refresh after mid-batch expiry: {mid_log:?}"
    );
    assert!(
        matches!(d.take_last_pos_force_tx_status(), armd::driver::PosForceTxStatus::Stale { joint: J7, .. }),
        "expected Stale status after mid-batch expiry"
    );

    // 2. WouldBlock stall crossing 100 ms: single-joint pos-force driver at 99 ms tries once,
    //    stall advances clock to 101 ms -> backpressured pass stops without retrying the moving
    //    target, reports Stale, and on the next tick sends priority 0xCC refresh AND a 0x301
    //    FREEZE frame (pos = -1.0, vel_raw = 0, NEVER the new target -2.0!).
    let clock = MockClock::new();
    let one_limit = vec![b601_limits()[J7]];
    let bus = FakeBus::with_clock(&one_limit, clock.clone());
    bus.with(|s| { s.motors[0].q = -1.0; });
    let mut one_pf = pf();
    one_pf.joint = 0;
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1])
        .with_joint_limits(one_limit).unwrap()
        .with_gripper_pos_force(one_pf).unwrap();
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    d.set_torque(true).unwrap();
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    clock.advance(Duration::from_millis(99));
    bus.with(|s| { s.tx_stall = true; s.stall_step = Duration::from_millis(2); });
    bus.clear_log();
    d.begin_tx_tick();
    let _ = d.write_joint_targets_bounded(&[JointTarget::position(-2.0)]).unwrap();
    let stall_log = bus.tx_log();
    let pf_attempts = stall_log.iter().filter(|f| f.id == 0x301).count();
    assert_eq!(pf_attempts, 1, "retried pos-force transmit past freshness bound: {stall_log:?}");
    assert!(
        matches!(d.take_last_pos_force_tx_status(), armd::driver::PosForceTxStatus::Stale { joint: 0, .. }),
        "expected Stale status after stall crossed 100 ms"
    );
    bus.with(|s| { s.tx_stall = false; });
    bus.clear_log();
    d.begin_tx_tick();
    d.write_joint_targets(&[JointTarget::position(-2.0)]).unwrap();
    let next_log = bus.tx_log();
    let next_pf: Vec<_> = next_log.iter().copied().filter(|f| f.id == 0x301).collect();
    assert_eq!(next_pf.len(), 1, "expected 1 0x301 freeze frame on next tick: {next_log:?}");
    let fpos = f32::from_le_bytes(next_pf[0].data[0..4].try_into().unwrap());
    let fvel = u16::from_le_bytes([next_pf[0].data[4], next_pf[0].data[5]]);
    let fcur = u16::from_le_bytes([next_pf[0].data[6], next_pf[0].data[7]]);
    assert!((fpos - (-1.0)).abs() < 1e-3, "expected freeze at last fresh -1.0 (not -2.0), got {fpos}");
    assert_eq!(fvel, 0, "expected freeze vel_raw == 0");
    assert_eq!(fcur, RAW);
    assert!(
        next_log.iter().any(|f| f.id == 0x7FF && f.data[0] == 1 && f.data[2] == 0xCC),
        "expected priority 0xCC refresh on next tick after backpressured stall: {next_log:?}"
    );

    // 3. Non-finite velocity (NaN / Inf) on an opening target refuses the entire batch before J1.
    let (mut d, bus, _) = rig(true);
    d.set_torque(true).unwrap();
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    for bad_vel in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        bus.clear_log();
        let mut t = targets(-2.0);
        t[J7].velocity = Some(bad_vel);
        let err = d.write_joint_targets(&t).unwrap_err().to_string();
        assert!(err.contains("not finite"), "bad_vel={bad_vel}: {err}");
        assert!(bus.tx_log().is_empty(), "sent partial batch for bad_vel={bad_vel}: {:?}", bus.tx_log());
    }

    // 4. Mode handover back to Position (PosForce) while enabled with invalid hold or stale feedback
    //    must refuse BEFORE writing CTRL_MODE (both set_joint_modes and set_joint_modes_confirmed).
    let (mut d, bus, clock) = rig(true);
    d.set_torque(true).unwrap();
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    let mut mit_modes = vec![JointMode::Position; 7];
    mit_modes[J7] = JointMode::Mit;
    let pos_modes = vec![JointMode::Position; 7];
    let valid_hold = targets(-1.0);
    d.set_joint_modes(&mit_modes, &valid_hold).unwrap();
    assert_eq!(d.joint_control_modes()[J7], Some(DamiaoControlMode::Mit));

    // 4a. NaN hold position
    let mut nan_hold = valid_hold.clone();
    nan_hold[J7].position = f32::NAN;
    bus.clear_log();
    let err = d.set_joint_modes(&pos_modes, &nan_hold).unwrap_err().to_string();
    assert!(err.contains("mode switch refused"), "{err}");
    assert!(bus.tx_log().is_empty(), "wrote CTRL_MODE on NaN hold: {:?}", bus.tx_log());

    // 4b. Infinite hold velocity
    let mut inf_hold = valid_hold.clone();
    inf_hold[J7].velocity = Some(f32::INFINITY);
    bus.clear_log();
    let err = d.set_joint_modes(&pos_modes, &inf_hold).unwrap_err().to_string();
    assert!(err.contains("mode switch refused"), "{err}");
    assert!(bus.tx_log().is_empty(), "wrote CTRL_MODE on Inf velocity hold: {:?}", bus.tx_log());

    // 4c. Stale feedback (> 100 ms) refuses both set_joint_modes and set_joint_modes_confirmed before CTRL_MODE write
    bus.with(|s| s.motors[J7].mute_feedback = true);
    clock.advance(Duration::from_millis(120));
    bus.clear_log();
    let err = d.set_joint_modes(&pos_modes, &valid_hold).unwrap_err().to_string();
    assert!(err.contains("mode switch refused"), "{err}");
    assert!(bus.tx_log().is_empty(), "set_joint_modes wrote CTRL_MODE while stale: {:?}", bus.tx_log());

    bus.clear_log();
    let err = d.set_joint_modes_confirmed(&pos_modes, &valid_hold).unwrap_err().to_string();
    assert!(err.contains("mode switch refused"), "{err}");
    assert!(bus.tx_log().is_empty(), "set_joint_modes_confirmed wrote CTRL_MODE while stale: {:?}", bus.tx_log());
}

#[test]
fn mode_handover_hold_backpressure_past_100ms_sends_no_stale_hold() {
    use std::sync::{Arc, Mutex};
    use std::time::Instant;
    use armd::driver::JointMode;
    use armd::transport::{CanTransport, DriverClock, TransportError};

    /// Injects SocketCAN-like WouldBlock backpressure on selected CAN IDs without forwarding
    /// blocked frames to `inner`, advancing the shared clock on each blocked attempt.
    #[derive(Clone)]
    struct BackpressureBus {
        inner: FakeBus,
        clock: MockClock,
        target_id: u32,
        target_op: Option<u8>,
        stall_remaining: Arc<Mutex<usize>>,
        stall_step: Duration,
        target_attempts: Arc<Mutex<Vec<Instant>>>,
    }

    impl CanTransport for BackpressureBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            self.inner.clock()
        }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            if frame.id == self.target_id && self.target_op.is_none_or(|op| frame.data[2] == op) {
                self.target_attempts.lock().unwrap().push(self.clock.now());
                let mut rem = self.stall_remaining.lock().unwrap();
                if *rem > 0 {
                    *rem -= 1;
                    self.clock.advance(self.stall_step);
                    return Err(TransportError::WouldBlock);
                }
            }
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
            self.inner.receive_timestamped()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
    }

    let make_mit_ready = |target_id: u32, target_op: Option<u8>, stall_remaining: usize, stall_step: Duration| {
        let clock = MockClock::new();
        let inner = FakeBus::with_clock(&b601_limits(), clock.clone());
        inner.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; }
            s.motors[J7].q = -1.0;
        });
        let wrapper = BackpressureBus {
            inner: inner.clone(),
            clock: clock.clone(),
            target_id,
            target_op,
            stall_remaining: Arc::new(Mutex::new(0)),
            stall_step,
            target_attempts: Arc::new(Mutex::new(Vec::new())),
        };
        let mut d = DamiaoArmDriver::new(wrapper.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits()).unwrap()
            .with_gripper_pos_force(pf()).unwrap();
        d.read_joint_states().unwrap();
        inner.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        d.set_torque(true).unwrap();
        d.read_joint_states().unwrap();
        d.read_joint_states().unwrap();
        let mut mit_modes = vec![JointMode::Position; 7];
        mit_modes[J7] = JointMode::Mit;
        d.set_joint_modes(&mit_modes, &targets(-1.0)).unwrap();
        assert_eq!(d.joint_control_modes()[J7], Some(DamiaoControlMode::Mit));
        // Refresh J7 feedback at a clean t0 so age is exact, then arm backpressure.
        inner.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.read_joint_states().unwrap();
        inner.with(|s| s.motors[J7].mute_feedback = true);
        inner.clear_log();
        *wrapper.stall_remaining.lock().unwrap() = stall_remaining;
        wrapper.target_attempts.lock().unwrap().clear();
        (d, inner, wrapper, clock)
    };

    let pos_modes = vec![JointMode::Position; 7];
    let hold = targets(-1.0);

    // 1. Both set_joint_modes and set_joint_modes_confirmed: 0x307 hold hits WouldBlock at 99 ms,
    //    advancing the clock to 101 ms (> 100 ms bound). Even though the bus would accept retry #1,
    //    the driver must recheck freshness, refuse to retry, emit ZERO 0x307 frames onto the bus,
    //    and leave J7 unconfirmed (fail closed).
    for confirmed in [false, true] {
        let (mut d, inner, wrapper, clock) = make_mit_ready(0x307, None, 1, Duration::from_millis(2));
        let t0 = clock.now();
        clock.advance(Duration::from_millis(99));
        let res = if confirmed {
            d.set_joint_modes_confirmed(&pos_modes, &hold)
        } else {
            d.set_joint_modes(&pos_modes, &hold)
        };
        assert!(res.is_err(), "confirmed={confirmed}: expected failure when hold retry crosses 100 ms");
        let attempts = wrapper.target_attempts.lock().unwrap().clone();
        assert_eq!(attempts.len(), 1, "confirmed={confirmed}: retried 0x307 past 100 ms: {attempts:?}");
        assert!(attempts[0].saturating_duration_since(t0) <= Duration::from_millis(100));
        let log = inner.tx_log();
        assert!(!log.iter().any(|f| f.id == 0x307 || f.id == 0x107),
            "confirmed={confirmed}: stale 0x307/0x107 hold reached the bus: {log:?}");
        if let Some(ctrl_idx) = log.iter().position(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE) {
            assert!(j7_commands(&log[ctrl_idx + 1..]).is_empty(),
                "confirmed={confirmed}: J7 command sent after CTRL_MODE=4 write: {:?}", j7_commands(&log[ctrl_idx + 1..]));
        }
        assert_eq!(d.joint_control_modes()[J7], None, "confirmed={confirmed}: J7 must be unconfirmed");
    }

    // 2. Positive control: when WouldBlock at 90 ms advances only 500 µs (retry at 90.6 ms <= 100 ms),
    //    both paths retry and deliver exactly one fresh capped 0x307 hold frame.
    for confirmed in [false, true] {
        let (mut d, inner, wrapper, clock) = make_mit_ready(0x307, None, 1, Duration::from_micros(500));
        inner.with(|s| s.motors[J7].mute_feedback = false);
        clock.advance(Duration::from_millis(90));
        let res = if confirmed {
            d.set_joint_modes_confirmed(&pos_modes, &hold)
        } else {
            d.set_joint_modes(&pos_modes, &hold)
        };
        res.unwrap_or_else(|e| panic!("confirmed={confirmed}: fresh retry should succeed: {e}"));
        assert_eq!(wrapper.target_attempts.lock().unwrap().len(), 2, "confirmed={confirmed}: expected 1 blocked + 1 succeeded attempt");
        let pf_holds: Vec<_> = inner.tx_log().into_iter().filter(|f| f.id == 0x307).collect();
        assert_eq!(pf_holds.len(), 1, "confirmed={confirmed}: {pf_holds:?}");
        assert_eq!(d.joint_control_modes()[J7], Some(DamiaoControlMode::PosForce));
    }

    // 3. WouldBlock on CTRL_MODE=4 write (0x7FF) crossing 100 ms must refuse to retry CTRL_MODE
    //    and send zero 0x307 hold frames.
    for confirmed in [false, true] {
        let (mut d, inner, wrapper, clock) = make_mit_ready(0x7FF, Some(damiao::REG_WRITE), 1, Duration::from_millis(2));
        clock.advance(Duration::from_millis(99));
        let res = if confirmed {
            d.set_joint_modes_confirmed(&pos_modes, &hold)
        } else {
            d.set_joint_modes(&pos_modes, &hold)
        };
        assert!(res.is_err(), "confirmed={confirmed}: expected failure when CTRL_MODE retry crosses 100 ms");
        assert_eq!(wrapper.target_attempts.lock().unwrap().len(), 1, "confirmed={confirmed}: retried CTRL_MODE past 100 ms");
        assert!(!inner.tx_log().iter().any(|f| f.id == 0x307 || f.id == 0x107),
            "confirmed={confirmed}: emitted 0x307/0x107 after expired CTRL_MODE write: {:?}", inner.tx_log());
    }
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
    assert_eq!((g.mode, g.current_ceiling_raw, g.calibrated), (DamiaoGripperMode::PosForce, 460, false));
    match gripper_drive_from_profile(&cfg).unwrap() {
        Some(GripperDrive::PosForce(p)) => assert_eq!((p.joint, p.current_raw, p.close_speed_rad_s, p.closed_rad), (6, 460, 0.30, cfg.gripper_policy.as_ref().unwrap().closed_rad)),
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
    assert_eq!(cap["current_raw"], 460);
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

#[test]
fn estop_holds_measured_position_only_when_feedback_is_fresh_else_commanded() {
    use armd::supervisor::SupervisorInner;
    let run_estop = |age: Duration, measured_j7: f32, commanded_j7: f32| -> (f32, f32, f32) {
        let (mut d, bus, clock) = rig(true);
        d.set_torque(true).unwrap();
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(d));
        inner.is_torqued = true;
        // Move the fake jaw and sample it through tick() so measured_positions and the supervisor's
        // coherent feedback snapshot come from real hardware feedback (not manual field writes).
        inner.tick(0.02);
        bus.with(|s| {
            s.motors[J7].q = measured_j7;
            s.push_all_feedback();
        });
        inner.tick(0.02);
        let decoded_j7 = inner.measured_positions[J7];
        assert!((decoded_j7 - measured_j7).abs() < 1e-3, "decoded={decoded_j7} want={measured_j7}");
        inner.commanded_positions[J7] = commanded_j7;
        bus.with(|s| for m in &mut s.motors { m.mute_feedback = true; });
        clock.advance(age);
        inner.apply_stop_policy(armd::StopPolicy::EStopLatch);
        (inner.commanded_positions[J7], inner.target_positions[J7], decoded_j7)
    };

    // Fresh feedback (50 ms and 249 ms <= 250 ms): locks onto measured position.
    for age_ms in [50u64, 249] {
        let (cmd, tgt, decoded) = run_estop(Duration::from_millis(age_ms), -1.2, -0.6);
        assert!((cmd - decoded).abs() < 1e-5 && (tgt - decoded).abs() < 1e-5, "age={age_ms}ms: cmd={cmd} tgt={tgt} decoded={decoded}");
    }

    // Stale feedback (251 ms and 300 ms > 250 ms): falls back to commanded position.
    for age_ms in [251u64, 300] {
        let (cmd, tgt, _) = run_estop(Duration::from_millis(age_ms), -2.0, -0.6);
        assert!((cmd - (-0.6)).abs() < 1e-5 && (tgt - (-0.6)).abs() < 1e-5, "age={age_ms}ms: cmd={cmd} tgt={tgt}");
    }

    // Aged unread RX queue (> 250 ms old) decoded immediately before E-stop: decode age is 0 ms,
    // but feedback_queue_bounds is 300 ms old -> must still fall back to commanded position.
    let (mut d, bus, clock) = rig(true);
    d.set_torque(true).unwrap();
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(d));
    inner.is_torqued = true;
    inner.tick(0.02);
    bus.with(|s| {
        s.motors[J7].q = -2.2;
        s.push_all_feedback();
        for m in &mut s.motors { m.mute_feedback = true; }
    });
    clock.advance(Duration::from_millis(300));
    inner.tick(0.02); // decodes the 300 ms-old queued frames right before E-stop
    assert!((inner.measured_positions[J7] - (-2.2)).abs() < 1e-3, "aged sample decoded");
    inner.commanded_positions[J7] = -0.6;
    inner.apply_stop_policy(armd::StopPolicy::EStopLatch);
    assert!((inner.commanded_positions[J7] - (-0.6)).abs() < 1e-5, "snapped to aged unread queue sample: {}", inner.commanded_positions[J7]);
    assert!((inner.target_positions[J7] - (-0.6)).abs() < 1e-5);

    // Pre-boundary startup frames (decoded before any empty queue was observed -> feedback_queue_bounds == None)
    // have unknown receive age -> E-stop must hold commanded position.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock);
    bus.with(|s| {
        s.motors[J7].q = -2.2;
        s.push_all_feedback();
        for m in &mut s.motors { m.mute_feedback = true; }
    });
    let d = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits()).unwrap()
        .with_gripper_pos_force(pf()).unwrap();
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(d));
    inner.tick(0.02);
    assert!((inner.measured_positions[J7] - (-2.2)).abs() < 1e-3);
    inner.commanded_positions[J7] = -0.6;
    inner.apply_stop_policy(armd::StopPolicy::EStopLatch);
    assert!((inner.commanded_positions[J7] - (-0.6)).abs() < 1e-5, "snapped to unknown-age startup sample: {}", inner.commanded_positions[J7]);
    assert!((inner.target_positions[J7] - (-0.6)).abs() < 1e-5);

    // Driver-side drain divergence: supervisor's measured_positions is 300 ms old (-1.0), while a
    // direct driver read refreshes the driver's own cache to 0 ms (-2.2) without updating the
    // supervisor snapshot -> E-stop must hold commanded (-0.6), never the stale -1.0 snapshot.
    let (mut d, bus, clock) = rig(true);
    d.set_torque(true).unwrap();
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(d));
    inner.is_torqued = true;
    inner.tick(0.02);
    assert!((inner.measured_positions[J7] - (-1.0)).abs() < 1e-3);
    clock.advance(Duration::from_millis(300));
    // Establish a fresh empty-queue boundary on the driver and decode fresh -2.2 feedback
    // directly into the driver cache without running inner.tick().
    {
        let mut drv = inner.driver.as_mut().unwrap().write().unwrap();
        drv.read_joint_states().unwrap();
        bus.with(|s| {
            s.motors[J7].q = -2.2;
            s.push_all_feedback();
        });
        drv.read_joint_states().unwrap();
        assert!(drv.feedback_all_within(7, Duration::from_millis(250)));
    }
    assert!((inner.measured_positions[J7] - (-1.0)).abs() < 1e-3, "supervisor still holds stale -1.0");
    inner.commanded_positions[J7] = -0.6;
    inner.apply_stop_policy(armd::StopPolicy::EStopLatch);
    assert!((inner.commanded_positions[J7] - (-0.6)).abs() < 1e-5, "driver drain authorized stale supervisor snapshot: {}", inner.commanded_positions[J7]);
    assert!((inner.target_positions[J7] - (-0.6)).abs() < 1e-5);
}

#[test]
fn estop_never_blocks_when_driver_mutex_held_by_another_thread() {
    use std::sync::{mpsc, Arc};
    use std::time::Instant;
    use armd::gravity_comp::{GravityCompPhase, GravityCompRun};
    use armd::model::GravityCompPolicy;
    use armd::supervisor::SupervisorInner;

    let setup_with_fresh_feedback = |floating_gravity_comp: bool| {
        let (mut d, bus, _) = rig(true);
        d.set_torque(true).unwrap();
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(d));
        inner.is_torqued = true;
        inner.tick(0.02);
        bus.with(|s| {
            for i in 0..6 { s.motors[i].q = 0.25 + 0.05 * i as f32; }
            s.motors[J7].q = -1.2;
            s.push_all_feedback();
        });
        inner.tick(0.02);
        assert!((inner.measured_positions[J7] - (-1.2)).abs() < 1e-3);
        for i in 0..6 { inner.commanded_positions[i] = -0.1 * (i as f32 + 1.0); }
        inner.commanded_positions[J7] = -0.6;
        if floating_gravity_comp {
            inner.gravity_comp = Some(GravityCompRun {
                phase: GravityCompPhase::Active,
                policy: GravityCompPolicy::rebot_dm(),
                elapsed_s: 1.0,
                scale: 0.8,
                damping_kd: [0.5; 6],
                entry_pose: [0.0; 6],
                entry_torque: [0.0; 6],
                tau_ff: [0.0; 6],
                kp: [0.0; 6],
                kd: [0.5; 6],
                position: [0.0; 6],
                overspeed_count: 0,
                prev_measured: None,
                lease_generation: None,
            });
        }
        inner
    };

    // 1. Safe multi-thread contention (zero unsafe): move the driver Mutex into an Arc held by
    //    another OS thread and move the owned SupervisorInner onto the E-stop thread.
    //    Test both ordinary operation and floating gravity compensation: E-stop must complete
    //    within one 20 ms tick without waiting on the lock and hold pre-stop commanded_positions.
    for floating_gc in [false, true] {
        let mut inner = setup_with_fresh_feedback(floating_gc);
        let pre_cmd = inner.commanded_positions.clone();
        let shared_driver = Arc::new(inner.driver.take().unwrap().into_legacy_mutex().unwrap());
        let holder_driver = Arc::clone(&shared_driver);
        let estop_driver = Arc::clone(&shared_driver);
        let (locked_tx, locked_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();
        let (done_tx, done_rx) = mpsc::channel();

        let holder = std::thread::spawn(move || {
            let _guard = holder_driver.lock().unwrap();
            locked_tx.send(()).unwrap();
            let _ = release_rx.recv();
        });

        locked_rx.recv().unwrap();
        let estop_thread = std::thread::spawn(move || {
            let t0 = Instant::now();
            inner.apply_estop_latch_with_driver(Some(&estop_driver));
            let _ = done_tx.send((t0.elapsed(), inner));
        });

        // Put the timeout on E-stop completion, then always release and join both threads so
        // scheduler jitter cannot release the mutex early and any blocking regression fails fast.
        let estop_res = done_rx.recv_timeout(Duration::from_millis(250));
        let _ = release_tx.send(());
        holder.join().unwrap();
        estop_thread.join().unwrap();
        let (elapsed, inner) = estop_res.expect("E-stop blocked past 250 ms while driver mutex was held");

        assert!(elapsed < Duration::from_millis(20), "floating_gc={floating_gc}: E-stop blocked on driver mutex: {elapsed:?}");
        assert_eq!(*inner.safety.state(), armd::SafetyState::EStop);
        for i in 0..7 {
            assert!((inner.commanded_positions[i] - pre_cmd[i]).abs() < 1e-5,
                "floating_gc={floating_gc} joint {i}: expected commanded {}, got {}", pre_cmd[i], inner.commanded_positions[i]);
            assert!((inner.target_positions[i] - pre_cmd[i]).abs() < 1e-5,
                "floating_gc={floating_gc} joint {i}: target expected {}, got {}", pre_cmd[i], inner.target_positions[i]);
        }
        if floating_gc {
            let run = inner.gravity_comp.as_ref().expect("gravity comp run kept for exiting");
            assert_eq!(run.phase.name(), "exiting");
            for i in 0..6 {
                assert!((run.position[i] - pre_cmd[i]).abs() < 1e-5,
                    "floating_gc joint {i}: exit hold overwrote commanded {} with {}", pre_cmd[i], run.position[i]);
            }
            assert_eq!(inner.gravity_comp_last_exit.as_ref().map(|e| e.code.as_str()), Some("ESTOP"));
        }
    }

    // 2. End-to-end `apply_stop_policy(StopPolicy::EStopLatch)` with a poisoned inline driver mutex
    //    and floating gravity compensation also completes immediately and falls back to commanded_positions.
    let mut inner = setup_with_fresh_feedback(true);
    let pre_cmd = inner.commanded_positions.clone();
    let _ = std::thread::scope(|s| {
        let drv = inner.driver.as_ref().unwrap().as_legacy_lock().unwrap();
        s.spawn(move || {
            let _guard = drv.lock().unwrap();
            panic!("intentional poison");
        }).join()
    });
    let t0 = Instant::now();
    inner.apply_stop_policy(armd::StopPolicy::EStopLatch);
    let elapsed = t0.elapsed();
    assert!(elapsed < Duration::from_millis(20), "poisoned E-stop blocked: {elapsed:?}");
    assert_eq!(*inner.safety.state(), armd::SafetyState::EStop);
    for i in 0..7 {
        assert!((inner.commanded_positions[i] - pre_cmd[i]).abs() < 1e-5);
        assert!((inner.target_positions[i] - pre_cmd[i]).abs() < 1e-5);
    }
}

fn is_j7_refresh(f: &CanFrame) -> bool {
    f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC
}

#[test]
fn enable_refreshes_stale_j7_feedback_across_ages_and_fails_closed_when_silent() {
    let make_driver = |already_mode_4: bool| {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; }
            s.motors[J7].q = -1.4;
            if already_mode_4 {
                s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
            }
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits()).unwrap()
            .with_gripper_pos_force(pf()).unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        // Refresh feedback right after check_mapping_ranges so age starts cleanly at 0 ms.
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.read_joint_states().unwrap();
        bus.clear_log();
        (d, bus, clock)
    };

    // 1. Live motor answering 0xCC: ages 0, 60, 99, 101, 150 ms all succeed 20x each
    //    for both cold-start (CTRL_MODE write + readback) and restart (already mode 4).
    for already_mode_4 in [false, true] {
        for age_ms in [0u64, 60, 99, 101, 150] {
            for iter in 0..20 {
                let (mut d, bus, clock) = make_driver(already_mode_4);
                clock.advance(Duration::from_millis(age_ms));
                d.set_torque(true).unwrap_or_else(|e| {
                    panic!("already_mode_4={already_mode_4} age={age_ms}ms iter={iter}: enable failed: {e}")
                });
                assert!(d.is_enabled() && d.motor_current_cap().unwrap().active,
                    "already_mode_4={already_mode_4} age={age_ms}ms iter={iter}");
                let log = bus.tx_log();
                let hold = log.iter().position(|f| f.id == 0x307).expect("pre-enable 0x307 hold");
                let enable = log.iter().position(|f| f.id == 7 && f.data == damiao::CMD_ENABLE).expect("CMD_ENABLE");
                assert!(hold < enable, "hold must precede enable: {log:?}");
                if age_ms > 100 {
                    let refresh = log.iter().position(is_j7_refresh).expect("expected 0xCC refresh for stale J7");
                    if already_mode_4 {
                        let read = log.iter().position(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)).expect("readback");
                        assert!(read < refresh && refresh < hold, "order read {read} < refresh {refresh} < hold {hold}");
                    } else {
                        let write = log.iter().position(|f| f.id == 0x7FF && f.data == reg_write(7, damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE)).expect("mode write");
                        assert!(refresh < write && write < hold, "order refresh {refresh} < write {write} < hold {hold}");
                    }
                }
            }
        }
    }

    // 2. Silent J7 (does not answer 0xCC): stale ages fail closed 20x each with the exact same errors.
    for iter in 0..20 {
        for age_ms in [101u64, 150] {
            // 2a. Cold-start mode-write precondition fails closed.
            let (mut d, bus, clock) = make_driver(false);
            bus.with(|s| s.motors[J7].mute_feedback = true);
            clock.advance(Duration::from_millis(age_ms));
            let err = d.set_torque(true).unwrap_err().to_string();
            assert!(err.contains("position-force mode not written: needs fresh stationary feedback with torque off"),
                "cold age={age_ms}ms iter={iter}: {err}");
            assert!(bus.tx_log().iter().any(is_j7_refresh), "expected 0xCC refresh attempt");
            assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE), "wrote CTRL_MODE without fresh reply");
            assert!(j7_commands(&bus.tx_log()).is_empty());
            assert!(!bus.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE));
            assert!(!d.is_enabled());

            // 2b. Mode-4 restart pre-enable hold fails closed.
            let (mut d, bus, clock) = make_driver(true);
            bus.with(|s| s.motors[J7].mute_feedback = true);
            clock.advance(Duration::from_millis(age_ms));
            let err = d.set_torque(true).unwrap_err().to_string();
            assert!(err.contains("no fresh measured position for pre-enable hold"),
                "mode4 age={age_ms}ms iter={iter}: {err}");
            assert!(bus.tx_log().iter().any(is_j7_refresh), "expected 0xCC refresh attempt");
            assert!(j7_commands(&bus.tx_log()).is_empty());
            assert!(!bus.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE));
            assert!(!d.is_enabled());
        }

        // 2c. 99 ms at begin_enable ages past 100 ms during readback (tx_step = 1 ms -> 100.5 ms) ->
        //     pre-enable refresh must fail closed when J7 does not answer 0xCC.
        let (mut d, bus, clock) = make_driver(true);
        bus.with(|s| {
            s.motors[J7].mute_feedback = true;
            s.tx_step = Duration::from_millis(1);
        });
        clock.advance(Duration::from_millis(99));
        let err = d.set_torque(true).unwrap_err().to_string();
        assert!(err.contains("no fresh measured position for pre-enable hold"), "99ms silent iter={iter}: {err}");
        assert!(bus.tx_log().iter().any(is_j7_refresh));
        assert!(j7_commands(&bus.tx_log()).is_empty());
        assert!(!bus.tx_log().iter().any(|f| f.data == damiao::CMD_ENABLE));
        assert!(!d.is_enabled());
    }
}

#[test]
fn enable_refresh_deadline_boundaries_and_failure_paths() {
    let make_stale = |already_mode_4: bool| {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; }
            s.motors[J7].q = -1.6;
            if already_mode_4 {
                s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
            }
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits()).unwrap()
            .with_gripper_pos_force(pf()).unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.read_joint_states().unwrap();
        clock.advance(Duration::from_millis(120));
        bus.clear_log();
        (d, bus, clock)
    };

    // 1. Both Phase 1A (pre-restore) and Phase 1B (pre-enable): reply recorded 1 ms before expiry
    //    wake is accepted when processed at the exact advertised expiry wake; reply AT expiry wake
    //    (rx_at == deadline) fails closed.
    for already_mode_4 in [false, true] {
        // 1a. Timely reply at wake - 1 ms -> accepted at wake.
        let (mut d, bus, clock) = make_stale(already_mode_4);
        bus.with(|s| s.motors[J7].mute_feedback = true);
        let mut p = d.begin_torque(true).unwrap();
        let wake = loop {
            match p {
                armd::driver::TorqueProgress::Pending { wake }
                    if bus.tx_log().iter().any(is_j7_refresh) && wake > clock.now() => break wake,
                armd::driver::TorqueProgress::Pending { .. } => {
                    clock.advance(Duration::from_micros(500));
                    p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
                }
                armd::driver::TorqueProgress::Done(_) => panic!("finished before refresh"),
            }
        };
        let to_pre = wake.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1));
        clock.advance(to_pre);
        bus.with(|s| {
            s.motors[J7].mute_feedback = false;
            s.push_feedback(J7);
        });
        d.read_joint_states().unwrap(); // recorded at wake - 1 ms (< deadline)
        clock.advance(Duration::from_millis(1));
        assert_eq!(clock.now(), wake);
        let mut p = d.poll_torque(clock.now() + Duration::from_millis(5))
            .unwrap_or_else(|e| panic!("already_mode_4={already_mode_4}: timely refresh reply rejected at wake: {e}"));
        while let armd::driver::TorqueProgress::Pending { .. } = p {
            clock.advance(Duration::from_micros(500));
            p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
        }
        assert!(d.is_enabled() && d.motor_current_cap().unwrap().active);

        // 1b. Late reply recorded AT exact expiry wake (rx_at == deadline) -> rejected fail closed.
        let (mut d, bus, clock) = make_stale(already_mode_4);
        bus.with(|s| s.motors[J7].mute_feedback = true);
        let mut p = d.begin_torque(true).unwrap();
        let wake = loop {
            match p {
                armd::driver::TorqueProgress::Pending { wake }
                    if bus.tx_log().iter().any(is_j7_refresh) && wake > clock.now() => break wake,
                armd::driver::TorqueProgress::Pending { .. } => {
                    clock.advance(Duration::from_micros(500));
                    p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
                }
                armd::driver::TorqueProgress::Done(_) => panic!("finished before refresh"),
            }
        };
        clock.advance(wake.saturating_duration_since(clock.now()));
        bus.with(|s| {
            s.motors[J7].mute_feedback = false;
            s.push_feedback(J7);
        });
        d.read_joint_states().unwrap(); // rx_at == wake (not strictly before deadline)
        let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
        if already_mode_4 {
            assert!(err.contains("no fresh measured position for pre-enable hold"), "{err}");
        } else {
            assert!(err.contains("position-force mode not written"), "{err}");
        }
        assert!(!d.is_enabled());
    }

    // 2. Slow transmit consuming > 50 ms inside the refresh phase must NOT extend the fixed 50 ms deadline.
    let (mut d, bus, _clock) = make_stale(false);
    bus.with(|s| {
        s.tx_step = Duration::from_millis(55);
        s.immediate_replies = true;
    });
    let err = d.set_torque(true).unwrap_err().to_string();
    assert!(err.contains("position-force mode not written"), "slow transmit extended 50 ms deadline: {err}");
    assert!(!d.is_enabled() && !d.torque_transition_pending());

    // 3. Wrong feedback CAN IDs (0x00, 0x07, 0x16 carrying J7 payload) must NOT satisfy refresh
    //    in default mode (expected 0x17); and with configured master_id 0x27, default 0x17 / 0x00 /
    //    0x07 are rejected while 0x27 succeeds in both refresh phases.
    for already_mode_4 in [false, true] {
        for wrong_id in [0x00u32, 0x07, 0x16] {
            let (mut d, bus, clock) = make_stale(already_mode_4);
            bus.with(|s| s.motors[J7].mute_feedback = true);
            let mut p = d.begin_torque(true).unwrap();
            let wake = loop {
                match p {
                    armd::driver::TorqueProgress::Pending { wake }
                        if bus.tx_log().iter().any(is_j7_refresh) && wake > clock.now() => break wake,
                    armd::driver::TorqueProgress::Pending { .. } => {
                        clock.advance(Duration::from_micros(500));
                        p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
                    }
                    armd::driver::TorqueProgress::Done(_) => panic!("finished before refresh"),
                }
            };
            bus.with(|s| {
                let mut spoof = s.motors[J7].feedback_frame();
                spoof.id = wrong_id;
                s.rx.push_back(spoof);
            });
            clock.advance(Duration::from_millis(1));
            let p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }),
                "already_mode_4={already_mode_4} wrong CAN ID {wrong_id:#x} satisfied refresh");
            clock.advance(wake.saturating_duration_since(clock.now()));
            assert!(d.poll_torque(clock.now() + Duration::from_millis(5)).is_err());
            assert!(!d.is_enabled());
        }

        // Configured master_id 0x27: rejects 0x17/0x00/0x07, accepts 0x27 in both refresh phases.
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; }
            s.motors[J7].q = -1.6;
            s.motors[J7].reply_id = 0x27;
            if already_mode_4 {
                s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
            }
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_master_id(0x27)
            .with_joint_limits(b601_limits()).unwrap()
            .with_gripper_pos_force(pf()).unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.read_joint_states().unwrap();
        clock.advance(Duration::from_millis(120));
        bus.clear_log();
        bus.with(|s| s.motors[J7].mute_feedback = true);
        let mut p = d.begin_torque(true).unwrap();
        while !(matches!(p, armd::driver::TorqueProgress::Pending { wake } if wake > clock.now())
            && bus.tx_log().iter().any(is_j7_refresh))
        {
            clock.advance(Duration::from_micros(500));
            p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
        }
        // Spoofed 0x17, 0x00, 0x07 must be ignored when master_id is 0x27.
        for wrong_id in [0x17u32, 0x00, 0x07] {
            bus.with(|s| {
                let mut spoof = s.motors[J7].feedback_frame();
                spoof.id = wrong_id;
                s.rx.push_back(spoof);
            });
            clock.advance(Duration::from_millis(1));
            let p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }),
                "master_id=0x27 accepted wrong CAN ID {wrong_id:#x}");
        }
        // Legitimate reply on configured 0x27 completes refresh and enables.
        bus.with(|s| {
            s.motors[J7].mute_feedback = false;
            s.push_feedback(J7);
        });
        clock.advance(Duration::from_millis(1));
        let mut p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
        while let armd::driver::TorqueProgress::Pending { .. } = p {
            clock.advance(Duration::from_micros(500));
            p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
        }
        assert!(d.is_enabled() && d.motor_current_cap().unwrap().active,
            "master_id=0x27 failed to enable after timely 0x27 refresh reply");
    }

    // 4. Phase 1A (pre-restore) refresh receiving moving (v=0.5) or state-1 feedback refuses mode write.
    for (moving, state_one, expected) in [(true, false, "still false"), (false, true, "torque off false")] {
        let (mut d, bus, _) = make_stale(false);
        bus.with(|s| {
            if moving { s.motors[J7].v = 0.5; }
            if state_one { s.motors[J7].state_override = Some(1); }
        });
        let err = d.set_torque(true).unwrap_err().to_string();
        assert!(err.contains(expected), "moving={moving} state_one={state_one}: {err}");
        assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE));
        assert!(!d.is_enabled());
    }

    // 5. Transmit / flush failure on 0xCC immediately cleans up enable state.
    for already_mode_4 in [false, true] {
        let (mut d, bus, clock) = make_stale(already_mode_4);
        if already_mode_4 {
            // Let readback request flush succeed (1st flush), then fail the 0xCC refresh flush.
            let _ = d.begin_torque(true).unwrap();
            while !bus.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) {
                clock.advance(Duration::from_micros(500));
                let p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
                assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
            }
            bus.with(|s| s.fail_flushes = 1);
            let mut got_err = false;
            for _ in 0..20 {
                clock.advance(Duration::from_micros(500));
                match d.poll_torque(clock.now() + Duration::from_millis(5)) {
                    Err(_) => { got_err = true; break; }
                    Ok(armd::driver::TorqueProgress::Pending { .. }) => {}
                    Ok(armd::driver::TorqueProgress::Done(_)) => panic!("unexpected Done"),
                }
            }
            assert!(got_err, "expected flush error on PreEnableRefresh");
        } else {
            bus.with(|s| s.fail_flushes = 1);
            assert!(d.begin_torque(true).is_err());
        }
        assert!(!d.is_enabled() && !d.torque_transition_pending());
    }

    // 6. Mid-refresh begin_torque(false) abandons cleanly and completes disable.
    for already_mode_4 in [false, true] {
        let (mut d, bus, clock) = make_stale(already_mode_4);
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        for i in 0..6 {
            arm_joints[i] = i;
        }
        let mut rest_positions = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
        for i in 0..6 {
            rest_positions[i] = 0.1 * i as f32;
        }
        rest_positions[J7] = -1.6;
        d.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
            safety_stopped: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 6,
            rest_positions,
            rest_pose_tolerance_rad: 0.05,
        });
        bus.with(|s| s.motors[J7].mute_feedback = true);
        let _ = d.begin_torque(true).unwrap();
        while !bus.tx_log().iter().any(is_j7_refresh) {
            clock.advance(Duration::from_micros(500));
            let p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
        }
        assert!(d.torque_transition_pending());
        bus.with(|s| s.motors[J7].mute_feedback = false);
        d.set_torque(false).expect("disable mid-refresh must succeed cleanly");
        assert!(!d.is_enabled() && !d.torque_transition_pending());
    }

    // 7. RX backlog preventing drain until deadline: PreEnableRefresh must NOT send 0xCC after expiry.
    let (mut d, bus, clock) = make_stale(true);
    bus.with(|s| s.motors[J7].mute_feedback = true);
    let mut p = d.begin_torque(true).unwrap();
    let read_wake = loop {
        match p {
            armd::driver::TorqueProgress::Pending { wake }
                if bus.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) && wake > clock.now() => break wake,
            armd::driver::TorqueProgress::Pending { .. } => {
                clock.advance(Duration::from_micros(500));
                p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
            }
            armd::driver::TorqueProgress::Done(_) => panic!("finished before readback"),
        }
    };
    // Deliver timely CTRL_MODE=4 readback at read_wake - 1 ms, then advance to read_wake so
    // poll_readback skips RX drain and enters PreEnableRefresh with boundary = None when queue
    // is flooded with >64 frames.
    let b = damiao::CTRL_MODE_POS_FORCE.to_le_bytes();
    let rb = CanFrame::new_standard(0x17, &[7, 0, damiao::REG_READ, damiao::RID_CTRL_MODE, b[0], b[1], b[2], b[3]]).unwrap();
    let to_pre = read_wake.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1));
    clock.advance(to_pre);
    bus.with(|s| { s.rx.push_back(rb); });
    d.read_joint_states().unwrap();
    clock.advance(Duration::from_millis(1));
    // Force last_read_drained = false by reading while 70 frames are queued.
    bus.with(|s| {
        let filler = s.motors[0].feedback_frame();
        for _ in 0..70 { s.rx.push_back(filler); }
    });
    d.read_joint_states().unwrap(); // consumes 64 frames -> last_read_drained == false
    bus.clear_log();
    // Transition from Readback into PreEnableRefresh (boundary == None), then poll once while
    // RX stays flooded so PreEnableRefresh advertises its 50 ms deadline without sending 0xCC.
    let enter = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
    assert!(matches!(enter, armd::driver::TorqueProgress::Pending { .. }));
    assert!(!bus.tx_log().iter().any(is_j7_refresh), "sent 0xCC without observed empty RX queue");
    bus.with(|s| {
        let filler = s.motors[0].feedback_frame();
        for _ in 0..200 { s.rx.push_back(filler); }
    });
    let ref_wake = match d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap() {
        armd::driver::TorqueProgress::Pending { wake } => wake,
        other => panic!("expected PreEnableRefresh pending deadline, got {other:?}"),
    };
    assert!(ref_wake > clock.now(), "expected future 50 ms deadline");
    assert!(!bus.tx_log().iter().any(is_j7_refresh), "sent 0xCC while RX remained flooded");
    clock.advance(ref_wake.saturating_duration_since(clock.now()));
    let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
    assert!(err.contains("no fresh measured position for pre-enable hold"), "{err}");
    assert!(!bus.tx_log().iter().any(is_j7_refresh), "sent 0xCC at/after deadline without drain");
    assert!(!d.is_enabled());

    // 8. Pre-request queued J7 feedback when poll_readback wakes at its expiry (so it skips RX drain
    //    while last_read_drained is still true from an earlier read): PreEnableRefresh MUST drain
    //    first to establish a fresh boundary before sending 0xCC, and MUST NOT accept that pre-request
    //    frame when J7 never answers 0xCC.
    let (mut d, bus, clock) = make_stale(true);
    bus.with(|s| s.motors[J7].mute_feedback = true);
    let mut p = d.begin_torque(true).unwrap();
    let read_wake = loop {
        match p {
            armd::driver::TorqueProgress::Pending { wake }
                if bus.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) && wake > clock.now() => break wake,
            armd::driver::TorqueProgress::Pending { .. } => {
                clock.advance(Duration::from_micros(500));
                p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
            }
            armd::driver::TorqueProgress::Done(_) => panic!("finished before readback"),
        }
    };
    let b = damiao::CTRL_MODE_POS_FORCE.to_le_bytes();
    let rb = CanFrame::new_standard(0x17, &[7, 0, damiao::REG_READ, damiao::RID_CTRL_MODE, b[0], b[1], b[2], b[3]]).unwrap();
    let to_pre = read_wake.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1));
    clock.advance(to_pre);
    bus.with(|s| s.rx.push_back(rb));
    // Intervening read records timely CTRL_MODE=4 AND sets last_read_drained = true + fresh rx_empty_instant.
    d.read_joint_states().unwrap();
    // Now a pre-request J7 feedback frame sits unread in RX while J7 stays muted (never answers 0xCC).
    clock.advance(Duration::from_millis(1));
    assert_eq!(clock.now(), read_wake);
    bus.with(|s| {
        let pre_req = s.motors[J7].feedback_frame();
        s.rx.push_back(pre_req);
    });
    bus.clear_log();
    // poll_readback wakes at read_wake (expired_before_read == true -> skips RX drain) and enters
    // PreEnableRefresh with boundary = None (must NOT reuse earlier empty-queue boundary).
    let enter = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
    assert!(matches!(enter, armd::driver::TorqueProgress::Pending { .. }));
    assert!(!bus.tx_log().iter().any(is_j7_refresh), "reused stale pre-readback boundary without draining");
    // First refresh poll drains the pre-request J7 frame, establishes a brand-new boundary, then sends 0xCC.
    let ref_wake = match d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap() {
        armd::driver::TorqueProgress::Pending { wake } => wake,
        other => panic!("expected PreEnableRefresh pending after draining pre-request frame, got {other:?}"),
    };
    assert!(bus.tx_log().iter().any(is_j7_refresh), "expected 0xCC after establishing fresh boundary");
    // J7 is muted and never answers 0xCC -> must still fail closed at deadline.
    clock.advance(ref_wake.saturating_duration_since(clock.now()));
    let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
    assert!(err.contains("no fresh measured position for pre-enable hold"), "{err}");
    assert!(j7_commands(&bus.tx_log()).is_empty());
    assert!(!d.is_enabled());

    // 9. Multi-joint stray restore where J7 feedback is 99 ms old at begin_enable and crosses
    //    100 ms while J1..J6 transmit their mode writes: live J7 refreshes via 0xCC mid-restore
    //    and enables; muted J7 attempts 0xCC and fails closed without writing J7 CTRL_MODE.
    for mute_j7 in [false, true] {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = 0.1 * i as f32;
                // Put J1..J6 in MIT (mode 1) and J7 in POS_VEL (mode 2) so all 7 joints are stray.
                if i < J7 {
                    m.set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_MIT);
                }
            }
            s.motors[J7].q = -1.6;
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits()).unwrap()
            .with_gripper_pos_force(pf()).unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.read_joint_states().unwrap();
        clock.advance(Duration::from_millis(99));
        bus.with(|s| {
            s.tx_step = Duration::from_millis(1); // J1..J6 writes push J7 age past 100 ms before J7's check
            s.motors[J7].mute_feedback = mute_j7;
        });
        bus.clear_log();
        if mute_j7 {
            let err = d.set_torque(true).unwrap_err().to_string();
            assert!(err.contains("position-force mode not written"), "muted mid-restore: {err}");
            assert!(bus.tx_log().iter().any(is_j7_refresh), "expected mid-restore 0xCC refresh attempt");
            assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data == reg_write(7, damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE)),
                "wrote J7 CTRL_MODE=4 without fresh refresh reply");
            assert!(j7_commands(&bus.tx_log()).is_empty());
            assert!(!d.is_enabled());
        } else {
            d.set_torque(true).expect("live J7 should refresh via 0xCC after J1..J6 restore aged J7 past 100 ms");
            assert!(d.is_enabled() && d.motor_current_cap().unwrap().active);
            let log = bus.tx_log();
            let j1_write = log.iter().position(|f| f.id == 0x7FF && f.data[0] == 1 && f.data[2] == damiao::REG_WRITE).expect("J1 write");
            let refresh = log.iter().position(is_j7_refresh).expect("mid-restore J7 0xCC refresh");
            let j7_write = log.iter().position(|f| f.id == 0x7FF && f.data == reg_write(7, damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE)).expect("J7 write");
            let hold = log.iter().position(|f| f.id == 0x307).expect("J7 hold");
            assert!(j1_write < refresh && refresh < j7_write && j7_write < hold,
                "order j1_write {j1_write} < refresh {refresh} < j7_write {j7_write} < hold {hold}");
        }
    }
}

#[test]
fn enable_refresh_transmit_boundary_and_live_safety_veto() {
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};
    use armd::transport::{CanTransport, DriverClock, TransportError};

    #[derive(Clone)]
    struct RefreshProbeBus {
        inner: FakeBus,
        clock: MockClock,
        inject_on_wouldblock: Arc<Mutex<Option<CanFrame>>>,
        /// `(skip_empty_reads, frame, pre_tx_gap)`: on the `(skip_empty_reads + 1)`-th empty RX
        /// read, queue `(frame, Some(clock.now()))` into `timed_rx` and advance the clock by
        /// `pre_tx_gap` so the frame arrives after the final empty read and strictly before `tx_at`.
        inject_between_empty_and_tx: Arc<Mutex<Option<(usize, CanFrame, Duration)>>>,
        timed_rx: Arc<Mutex<VecDeque<(CanFrame, Option<std::time::Instant>)>>>,
        advance_schedule: Arc<Mutex<VecDeque<Duration>>>,
    }

    impl RefreshProbeBus {
        fn new(clock: MockClock) -> Self {
            Self {
                inner: FakeBus::with_clock(&b601_limits(), clock.clone()),
                clock,
                inject_on_wouldblock: Arc::new(Mutex::new(None)),
                inject_between_empty_and_tx: Arc::new(Mutex::new(None)),
                timed_rx: Arc::new(Mutex::new(VecDeque::new())),
                advance_schedule: Arc::new(Mutex::new(VecDeque::new())),
            }
        }
    }

    impl CanTransport for RefreshProbeBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> { self.inner.transmit(frame) }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.receive_timestamped().map(|(f, _)| f)
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
            if let Some(item) = self.timed_rx.lock().unwrap().pop_front() {
                if let Some(d) = self.advance_schedule.lock().unwrap().pop_front() {
                    if !d.is_zero() {
                        self.clock.advance(d);
                    }
                }
                return Ok(item);
            }
            match self.inner.receive_timestamped() {
                Ok((frame, ts)) => {
                    if let Some(d) = self.advance_schedule.lock().unwrap().pop_front() {
                        if !d.is_zero() {
                            self.clock.advance(d);
                        }
                    }
                    Ok((frame, ts))
                }
                Err(e @ (TransportError::Timeout | TransportError::WouldBlock)) => {
                    if let Some(injected) = self.inject_on_wouldblock.lock().unwrap().take() {
                        self.inner.with(|s| s.rx.push_back(injected));
                    }
                    let mut armed = self.inject_between_empty_and_tx.lock().unwrap();
                    if let Some((skip, frame, gap)) = armed.take() {
                        if skip == 0 {
                            let arrived_at = self.clock.now();
                            self.timed_rx.lock().unwrap().push_back((frame, Some(arrived_at)));
                            if !gap.is_zero() {
                                self.clock.advance(gap);
                            }
                        } else {
                            *armed = Some((skip - 1, frame, gap));
                        }
                    }
                    Err(e)
                }
                Err(e) => Err(e),
            }
        }
        fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
    }

    let make_probe_stale = |already_mode_4: bool| {
        let clock = MockClock::new();
        let bus = RefreshProbeBus::new(clock.clone());
        bus.inner.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; }
            s.motors[J7].q = -1.6;
            if already_mode_4 {
                s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
            }
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits()).unwrap()
            .with_gripper_pos_force(pf()).unwrap();
        d.read_joint_states().unwrap();
        bus.inner.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        bus.inner.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.read_joint_states().unwrap();
        clock.advance(Duration::from_millis(120));
        bus.inner.clear_log();
        (d, bus, clock)
    };

    // (a) Pre-transmit reply queued after caller's empty-queue drain and BEFORE 0xCC transmit,
    //     while J7 is muted so it never answers 0xCC: both RefreshBeforeRestore and
    //     PreEnableRefresh must reject it and emit neither mode-write nor hold/enable.
    {
        // a1. RefreshBeforeRestore
        let (mut d, bus, clock) = make_probe_stale(false);
        let pre_tx_j7 = bus.inner.with(|s| {
            s.motors[J7].mute_feedback = true;
            s.motors[J7].feedback_frame()
        });
        *bus.inject_on_wouldblock.lock().unwrap() = Some(pre_tx_j7);
        let p = d.begin_torque(true).unwrap();
        assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
        assert!(bus.inner.tx_log().iter().any(is_j7_refresh), "expected 0xCC transmit");
        let wake = match d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap() {
            armd::driver::TorqueProgress::Pending { wake } => wake,
            other => panic!("expected RefreshBeforeRestore pending wake, got {other:?}"),
        };
        assert!(wake > clock.now());
        clock.advance(wake.saturating_duration_since(clock.now()));
        let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
        assert!(err.contains("no timely post-refresh feedback"), "{err}");
        let log = bus.inner.tx_log();
        assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE));
        assert!(j7_commands(&log).is_empty());
        assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE));
        assert!(!d.is_enabled());

        // a2. PreEnableRefresh
        let (mut d, bus, clock) = make_probe_stale(true);
        bus.inner.with(|s| s.motors[J7].mute_feedback = true);
        let _ = d.begin_torque(true).unwrap();
        while !bus.inner.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) {
            clock.advance(Duration::from_micros(500));
            let p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
        }
        let b = damiao::CTRL_MODE_POS_FORCE.to_le_bytes();
        let rb = CanFrame::new_standard(0x17, &[7, 0, damiao::REG_READ, damiao::RID_CTRL_MODE, b[0], b[1], b[2], b[3]]).unwrap();
        let pre_tx_j7 = bus.inner.with(|s| {
            s.rx.push_back(rb);
            s.motors[J7].feedback_frame()
        });
        // Arm injection on the WouldBlock right after poll_readback drains the REG_READ reply,
        // so the frame sits in RX right before start_feedback_refresh transmits 0xCC.
        *bus.inject_on_wouldblock.lock().unwrap() = Some(pre_tx_j7);
        clock.advance(Duration::from_millis(1));
        assert!(matches!(d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap(), armd::driver::TorqueProgress::Pending { .. }));
        assert!(bus.inner.tx_log().iter().any(is_j7_refresh), "expected 0xCC transmit");
        assert!(j7_commands(&bus.inner.tx_log()).is_empty());
        let ref_wake = match d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap() {
            armd::driver::TorqueProgress::Pending { wake } => wake,
            other => panic!("expected PreEnableRefresh pending wake, got {other:?}"),
        };
        assert!(ref_wake > clock.now());
        clock.advance(ref_wake.saturating_duration_since(clock.now()));
        let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
        assert!(err.contains("no fresh measured position for pre-enable hold"), "{err}");
        let log = bus.inner.tx_log();
        assert!(j7_commands(&log).is_empty());
        assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE));
        assert!(!d.is_enabled());
    }

    // (a3) Reply arriving AFTER the final pre-transmit empty read (`drain_rx_for_boundary`'s
    //      `WouldBlock`) and BEFORE `transmit(0xCC)`: it is dequeued post-transmit with a
    //      post-transmit sequence number (`seq > boundary`), yet MUST be rejected because its
    //      receive time precedes `tx_at`, in both RefreshBeforeRestore and PreEnableRefresh.
    for already_mode_4 in [false, true] {
        let (mut d, bus, clock) = make_probe_stale(already_mode_4);
        let pre_tx_j7 = bus.inner.with(|s| {
            s.motors[J7].mute_feedback = true;
            s.motors[J7].q = -1.35;
            s.motors[J7].feedback_frame()
        });
        let seq_before_poll = if already_mode_4 {
            let _ = d.begin_torque(true).unwrap();
            while !bus.inner.tx_log().iter().any(|f| is_reg_read(f, 7, damiao::RID_CTRL_MODE)) {
                clock.advance(Duration::from_micros(500));
                let p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
                assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
            }
            let b = damiao::CTRL_MODE_POS_FORCE.to_le_bytes();
            let rb = CanFrame::new_standard(0x17, &[7, 0, damiao::REG_READ, damiao::RID_CTRL_MODE, b[0], b[1], b[2], b[3]]).unwrap();
            bus.inner.with(|s| s.rx.push_back(rb));
            // Empty read #0 = poll_readback's receive_feedback; empty read #1 = drain_rx_for_boundary
            // right before 0xCC transmit.
            *bus.inject_between_empty_and_tx.lock().unwrap() = Some((1, pre_tx_j7, Duration::from_micros(100)));
            clock.advance(Duration::from_millis(1));
            assert!(matches!(d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap(), armd::driver::TorqueProgress::Pending { .. }));
            assert!(bus.inner.tx_log().iter().any(is_j7_refresh), "expected 0xCC transmit");
            d.feedback_seqs()[J7]
        } else {
            // Empty read #0 = drain_rx_before_command; empty read #1 = drain_rx_for_boundary
            // right before 0xCC transmit.
            *bus.inject_between_empty_and_tx.lock().unwrap() = Some((1, pre_tx_j7, Duration::from_micros(100)));
            let p = d.begin_torque(true).unwrap();
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
            assert!(bus.inner.tx_log().iter().any(is_j7_refresh), "expected 0xCC transmit");
            d.feedback_seqs()[J7]
        };
        assert_eq!(bus.timed_rx.lock().unwrap().len(), 1,
            "already_mode_4={already_mode_4}: pre-tx reply must still sit unread after 0xCC transmit");
        let wake = match d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap() {
            armd::driver::TorqueProgress::Pending { wake } => wake,
            other => panic!("already_mode_4={already_mode_4}: pre-tx reply was accepted! got {other:?}"),
        };
        assert_eq!(d.feedback_seqs()[J7], seq_before_poll.wrapping_add(1),
            "already_mode_4={already_mode_4}: pre-tx reply must have been dequeued post-transmit");
        assert!(wake > clock.now());
        clock.advance(wake.saturating_duration_since(clock.now()));
        let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
        if already_mode_4 {
            assert!(err.contains("no fresh measured position for pre-enable hold"), "{err}");
        } else {
            assert!(err.contains("no timely post-refresh feedback"), "{err}");
        }
        let log = bus.inner.tx_log();
        if !already_mode_4 {
            assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE));
        }
        assert!(j7_commands(&log).is_empty());
        assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE));
        assert!(!d.is_enabled());
    }

    // Helper to drive `d` into active refresh (`RefreshBeforeRestore` or `PreEnableRefresh`) with `0xCC` sent.
    let drive_to_refresh = |already_mode_4: bool| {
        let (mut d, bus, clock) = make_probe_stale(already_mode_4);
        bus.inner.with(|s| s.motors[J7].mute_feedback = true);
        let mut p = d.begin_torque(true).unwrap();
        let wake = loop {
            match p {
                armd::driver::TorqueProgress::Pending { wake }
                    if bus.inner.tx_log().iter().any(is_j7_refresh) && wake > clock.now() => break wake,
                armd::driver::TorqueProgress::Pending { .. } => {
                    clock.advance(Duration::from_micros(500));
                    p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
                }
                armd::driver::TorqueProgress::Done(_) => panic!("finished before refresh"),
            }
        };
        (d, bus, clock, wake)
    };

    // (b) Timely reply (deadline - 1 ms) followed in the SAME receive_feedback pass by a newer
    //     stationary torque-off reply (deadline + 1 ms, state 0): both RefreshBeforeRestore and
    //     PreEnableRefresh accept and enable, and the 0x307 pre-enable hold uses the newest
    //     fresh validated stationary position (-1.2), not the older -1.6 sample.
    for already_mode_4 in [false, true] {
        let (mut d, bus, clock, wake) = drive_to_refresh(already_mode_4);
        clock.advance(wake.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1)));
        bus.inner.with(|s| {
            s.motors[J7].mute_feedback = false;
            s.motors[J7].q = -1.6;
            let f1 = s.motors[J7].feedback_frame();
            s.motors[J7].q = -1.2;
            let f2 = s.motors[J7].feedback_frame();
            s.rx.push_back(f1);
            s.rx.push_back(f2);
        });
        *bus.advance_schedule.lock().unwrap() = VecDeque::from([Duration::ZERO, Duration::from_millis(2)]);
        let mut p = d.poll_torque(clock.now() + Duration::from_millis(10))
            .unwrap_or_else(|e| panic!("already_mode_4={already_mode_4}: same-pass benign late frame failed: {e}"));
        while let armd::driver::TorqueProgress::Pending { .. } = p {
            clock.advance(Duration::from_micros(500));
            p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
        }
        assert!(d.is_enabled() && d.motor_current_cap().unwrap().active, "already_mode_4={already_mode_4}");
        let log = bus.inner.tx_log();
        let hold = log.iter().position(|f| f.id == 0x307).expect("0x307 hold");
        let enable = log.iter().position(|f| f.id == 7 && f.data == damiao::CMD_ENABLE).expect("CMD_ENABLE");
        assert!(hold < enable);
        if already_mode_4 {
            let hold_f = log[hold];
            let hold_pos = f32::from_le_bytes([hold_f.data[0], hold_f.data[1], hold_f.data[2], hold_f.data[3]]);
            let hold_vel = u16::from_le_bytes([hold_f.data[4], hold_f.data[5]]);
            assert!((hold_pos - (-1.2)).abs() < 1e-3, "expected newest validated hold position -1.2, got {hold_pos}");
            assert_eq!(hold_vel, 0, "pre-enable hold must be zero-velocity Hold (vel_raw=0), got {hold_vel}");
        }
    }

    // (b2) Unsafe timely reply (moving v=0.5, enabled state=1, or fault state=0x08) followed in
    //      the SAME receive_feedback pass by a benign late reply (v=0.0, state=0): both
    //      RefreshBeforeRestore and PreEnableRefresh must fail closed with NO mode-write, hold, or enable.
    for already_mode_4 in [false, true] {
        for (label, v, st_ovr) in [
            ("moving", 0.5_f32, None),
            ("state_1", 0.0_f32, Some(1u8)),
            ("fault_0x08", 0.0_f32, Some(0x08u8)),
        ] {
            let (mut d, bus, clock, wake) = drive_to_refresh(already_mode_4);
            clock.advance(wake.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1)));
            bus.inner.with(|s| {
                s.motors[J7].mute_feedback = false;
                s.motors[J7].v = v;
                s.motors[J7].state_override = st_ovr;
                let unsafe_timely = s.motors[J7].feedback_frame();
                s.motors[J7].v = 0.0;
                s.motors[J7].state_override = None;
                let safe_late = s.motors[J7].feedback_frame();
                s.rx.push_back(unsafe_timely);
                s.rx.push_back(safe_late);
            });
            bus.inner.clear_log();
            *bus.advance_schedule.lock().unwrap() = VecDeque::from([Duration::ZERO, Duration::from_millis(2)]);
            let err = d.poll_torque(clock.now() + Duration::from_millis(10))
                .expect_err(&format!("already_mode_4={already_mode_4} unsafe-timely/safe-late {label} must veto"))
                .to_string();
            assert!(!err.is_empty());
            let log = bus.inner.tx_log();
            if !already_mode_4 {
                assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
                    "{label} wrote CTRL_MODE");
            }
            assert!(j7_commands(&log).is_empty(), "already_mode_4={already_mode_4} {label} sent hold: {log:?}");
            assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE),
                "already_mode_4={already_mode_4} {label} sent CMD_ENABLE");
            assert!(!d.is_enabled());
        }
    }

    // (c1) Timely reply followed in the SAME receive_feedback pass by an UNSAFE late reply
    //      (moving v=0.5, enabled state=1, or fault state=0x08): both RefreshBeforeRestore and
    //      PreEnableRefresh veto and emit neither mode-write nor 0x307 hold nor CMD_ENABLE.
    for already_mode_4 in [false, true] {
        for (label, v, st_ovr) in [
            ("moving", 0.5_f32, None),
            ("state_1", 0.0_f32, Some(1u8)),
            ("fault_0x08", 0.0_f32, Some(0x08u8)),
        ] {
            let (mut d, bus, clock, wake) = drive_to_refresh(already_mode_4);
            clock.advance(wake.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1)));
            bus.inner.with(|s| {
                s.motors[J7].mute_feedback = false;
                let safe = s.motors[J7].feedback_frame();
                s.motors[J7].v = v;
                s.motors[J7].state_override = st_ovr;
                let unsafe_f = s.motors[J7].feedback_frame();
                s.rx.push_back(safe);
                s.rx.push_back(unsafe_f);
            });
            bus.inner.clear_log();
            *bus.advance_schedule.lock().unwrap() = VecDeque::from([Duration::ZERO, Duration::from_millis(2)]);
            let err = d.poll_torque(clock.now() + Duration::from_millis(10))
                .expect_err(&format!("already_mode_4={already_mode_4} {label} should veto enable"))
                .to_string();
            assert!(!err.is_empty());
            let log = bus.inner.tx_log();
            if !already_mode_4 {
                assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
                    "{label} wrote CTRL_MODE");
            }
            assert!(j7_commands(&log).is_empty(), "already_mode_4={already_mode_4} {label} sent hold: {log:?}");
            assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE),
                "already_mode_4={already_mode_4} {label} sent CMD_ENABLE");
            assert!(!d.is_enabled());
        }
    }

    // (c2) Direct readback path (already_mode_4 = true, fresh J7 feedback, no refresh needed)
    //      with unsafe live telemetry (moving v=0.5, state 1, or fault 0x08) must veto before
    //      0x307 hold or CMD_ENABLE.
    for (label, v, st_ovr, expect_substr) in [
        ("moving", 0.5_f32, None, "still false"),
        ("state_1", 0.0_f32, Some(1u8), "torque off false"),
        ("fault_0x08", 0.0_f32, Some(0x08u8), "status 8"),
    ] {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; }
            s.motors[J7].q = -1.6;
            s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits()).unwrap()
            .with_gripper_pos_force(pf()).unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        bus.with(|s| {
            s.motors[J7].v = v;
            s.motors[J7].state_override = st_ovr;
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();
        bus.clear_log();
        let err = d.set_torque(true).unwrap_err().to_string();
        assert!(err.contains("pre-enable hold refused: needs stationary torque-off feedback"), "{label}: {err}");
        assert!(err.contains(expect_substr), "{label}: {err}");
        let log = bus.tx_log();
        assert!(j7_commands(&log).is_empty(), "{label} emitted 0x307 hold: {log:?}");
        assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE), "{label} emitted CMD_ENABLE");
        assert!(!d.is_enabled());
    }

    // (d) Healthy reply first dequeued at the exact expiry wake (no prior poll / read_joint_states):
    //     - When its receive time is strictly before the deadline (`wake - 1 ms`), one bounded RX
    //       drain at the expiry wake accepts it and enables cleanly in both phases.
    //     - Negative control: a late reply first dequeued at the expiry wake with receive time
    //       at the deadline (`wake`) is rejected fail-closed.
    for already_mode_4 in [false, true] {
        // d1. Timely reply (rx_at = wake - 1 ms) first dequeued at expiry wake -> accepted.
        let (mut d, bus, clock, wake) = drive_to_refresh(already_mode_4);
        let to_pre = wake.saturating_duration_since(clock.now()).saturating_sub(Duration::from_millis(1));
        clock.advance(to_pre);
        let timely_rx_at = clock.now();
        let timely_frame = bus.inner.with(|s| {
            s.motors[J7].mute_feedback = false;
            s.motors[J7].q = -1.45;
            s.motors[J7].feedback_frame()
        });
        bus.timed_rx.lock().unwrap().push_back((timely_frame, Some(timely_rx_at)));
        // Advance directly to expiry wake WITHOUT calling read_joint_states() beforehand.
        clock.advance(Duration::from_millis(1));
        assert_eq!(clock.now(), wake);
        let mut p = d.poll_torque(clock.now() + Duration::from_millis(5))
            .unwrap_or_else(|e| panic!("already_mode_4={already_mode_4}: timely reply first dequeued at expiry wake timed out: {e}"));
        while let armd::driver::TorqueProgress::Pending { .. } = p {
            clock.advance(Duration::from_micros(500));
            p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
        }
        assert!(d.is_enabled() && d.motor_current_cap().unwrap().active, "already_mode_4={already_mode_4}");

        // d2. Late reply (rx_at == wake == deadline) first dequeued at expiry wake -> rejected.
        let (mut d, bus, clock, wake) = drive_to_refresh(already_mode_4);
        clock.advance(wake.saturating_duration_since(clock.now()));
        assert_eq!(clock.now(), wake);
        let late_frame = bus.inner.with(|s| {
            s.motors[J7].mute_feedback = false;
            s.motors[J7].feedback_frame()
        });
        bus.timed_rx.lock().unwrap().push_back((late_frame, Some(wake)));
        let err = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap_err().to_string();
        if already_mode_4 {
            assert!(err.contains("no fresh measured position for pre-enable hold"), "{err}");
        } else {
            assert!(err.contains("no timely post-refresh feedback"), "{err}");
        }
        assert!(!d.is_enabled());
    }
}

#[test]
fn pos_force_full_stroke_close_plans_at_close_speed_without_timeout() {
    use std::sync::Arc;
    use armd::motion::{GripperCommand, GripperRequest};
    use armd::supervisor::SupervisorInner;
    use armd::transport::{CanTransport, DriverClock, TransportError};

    const DT: f64 = 0.02;

    /// Wraps `FakeBus` so J7 moves at the wire-encoded speed limit (`vel_raw * 0.01` rad/s in
    /// pos-force 0x307; `vlim` in POS_VEL 0x107) instead of teleporting instantaneously.
    #[derive(Clone)]
    struct RateLimitedBus {
        inner: FakeBus,
        dt: f32,
    }

    impl CanTransport for RateLimitedBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let prev_q = self.inner.with(|s| s.motors[J7].q);
            self.inner.transmit(frame)?;
            if frame.id == 0x307 || frame.id == 0x107 {
                self.inner.with(|s| {
                    let m = &mut s.motors[J7];
                    if !m.enabled { return; }
                    let target_p = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
                    let wire_v = if frame.id == 0x307 {
                        f32::from(u16::from_le_bytes([frame.data[4], frame.data[5]])) * 0.01
                    } else {
                        f32::from_le_bytes([frame.data[4], frame.data[5], frame.data[6], frame.data[7]]).abs()
                    };
                    let max_step = wire_v * self.dt;
                    let delta = (target_p - prev_q).clamp(-max_step, max_step);
                    let mut next_q = prev_q + delta;
                    if let Some(stop) = m.stop_at {
                        if next_q > stop {
                            next_q = stop;
                            m.tau = f32::from(u16::from_le_bytes([frame.data[6], frame.data[7]])) * 0.002;
                        }
                    }
                    m.q = next_q;
                    m.v = (next_q - prev_q) / self.dt;
                    // Replace the feedback frame pushed by FakeBus with the rate-limited state.
                    let updated = m.feedback_frame();
                    if let Some(last) = s.pending.back_mut().filter(|f| f.id == updated.id) {
                        *last = updated;
                    } else if let Some(last) = s.rx.back_mut().filter(|f| f.id == updated.id) {
                        *last = updated;
                    }
                });
            }
            Ok(())
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> { self.inner.receive_timestamped() }
        fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
    }

    let make_sup = |pos_force_speed: Option<f32>, start_j7: f32| {
        let clock = MockClock::new();
        let inner_bus = FakeBus::with_clock(&b601_limits(), clock.clone());
        inner_bus.with(|s| {
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.0; if i == J7 { m.q = start_j7; } }
        });
        let bus = RateLimitedBus { inner: inner_bus.clone(), dt: DT as f32 };
        let mut d = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits()).unwrap();
        let mut cfg = ArmConfig::rebot_dm();
        if let Some(close_speed) = pos_force_speed {
            d = d.with_gripper_pos_force(GripperPosForce { close_speed_rad_s: close_speed, ..pf() }).unwrap();
            cfg.damiao_gripper = Some(DamiaoGripperDrive { close_speed_rad_s: close_speed, ..DamiaoGripperDrive::rebot_dm() });
        } else {
            d = d.with_gripper_pos_vel(J7);
            cfg.damiao_gripper = Some(DamiaoGripperDrive::legacy_pos_vel());
        }
        d.read_joint_states().unwrap();
        inner_bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        let mut sup = Box::new(SupervisorInner::new(cfg).with_driver(Box::new(d)));
        sup.lease_enabled = false;
        sup.tick(DT);
        { let __s = sup.current_admission_stamp(); sup.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
        sup.tick(DT);
        (sup, inner_bus, clock)
    };

    let run_to_end = |sup: &mut SupervisorInner, clock: &MockClock, id: u64, max_s: f64| {
        for _ in 0..(max_s / DT) as usize {
            clock.advance(Duration::from_millis(20));
            sup.tick(DT);
            let a = sup.last_action.clone().unwrap();
            assert_eq!(a.id, id);
            if !a.is_running() { return a; }
        }
        panic!("action {id} still running after {max_s} s: {:?}", sup.last_action);
    };

    // 1. Pos-force full-stroke close (-5.7 -> 0.0) via Move(0.0) and Close(grasp=false):
    //    expected duration >= 5.7 / 0.30 s, completes without GRIPPER_TIMEOUT, all close frames <= 0.30 rad/s.
    for req in [
        GripperRequest { command: GripperCommand::Move, fraction: Some(0.0), grasp: false, speed: 1.0, timeout_s: None, tag: None },
        GripperRequest { command: GripperCommand::Close, fraction: None, grasp: false, speed: 1.0, timeout_s: None, tag: None },
    ] {
        let (mut sup, inner_bus, clock) = make_sup(Some(0.30), -5.7);
        inner_bus.clear_log();
        let epoch = sup.safety.epoch_id();
        let (id, expected) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a, req.clone(), epoch)) } }.unwrap();
        assert!(expected >= 5.7 / 0.30, "req={req:?}: expected_duration {expected:.2} s < {:.2} s", 5.7 / 0.30);
        let a = run_to_end(&mut sup, &clock, id, 25.0);
        assert_eq!(a.state, "done", "req={req:?}: {a:?}");
        assert!(sup.measured_positions[J7].abs() < 0.05, "req={req:?}: J7 ended at {}", sup.measured_positions[J7]);
        let close_frames = j7_commands(&inner_bus.tx_log());
        assert!(!close_frames.is_empty());
        let close_raw = damiao::pos_force_vel_raw(0.30);
        assert!(close_frames.iter().all(|f| {
            let r = u16::from_le_bytes([f.data[4], f.data[5]]);
            f.id == 0x307 && (1..=close_raw).contains(&r)
        }), "closing frame out of 1..={close_raw} vel_raw range");
    }

    // 2. Pos-force opening (0.0 -> -5.7) keeps the requested 1.0 rad/s speed limit (not slowed to 0.30).
    let (mut sup, inner_open, clock) = make_sup(Some(0.30), 0.0);
    inner_open.clear_log();
    let epoch = sup.safety.epoch_id();
    let open_req = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: None, tag: None };
    let (id, expected_open) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a, open_req, epoch)) } }.unwrap();
    assert!(expected_open > 5.7 && expected_open < 8.0, "opening slowed unexpectedly: {expected_open:.2} s");
    let a = run_to_end(&mut sup, &clock, id, 10.0);
    assert_eq!(a.state, "done", "{a:?}");
    assert!((sup.measured_positions[J7] - (-5.7)).abs() < 0.05, "J7 open {}", sup.measured_positions[J7]);
    assert!(j7_commands(&inner_open.tx_log()).iter().all(|f| f.id == 0x307 && u16::from_le_bytes([f.data[4], f.data[5]]) >= 1),
        "opening emitted zero vel_raw frame");

    // 3. Legacy pos_vel mode full-stroke close (-5.7 -> 0.0) is unchanged (1.0 rad/s, expected < 8.0 s).
    let (mut sup, _inner_bus, clock) = make_sup(None, -5.7);
    let epoch = sup.safety.epoch_id();
    let close_req = GripperRequest { command: GripperCommand::Move, fraction: Some(0.0), grasp: false, speed: 1.0, timeout_s: None, tag: None };
    let (id, expected_pv) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a, close_req, epoch)) } }.unwrap();
    assert!(expected_pv > 5.7 && expected_pv < 8.0, "pos_vel expected_duration changed: {expected_pv:.2} s");
    let a = run_to_end(&mut sup, &clock, id, 10.0);
    assert_eq!(a.state, "done", "{a:?}");
    assert!(sup.measured_positions[J7].abs() < 0.05, "pos_vel J7 {}", sup.measured_positions[J7]);

    // 4. Quantization boundary (0.03_f32 floors to raw 2 = 0.02 rad/s on the wire):
    //    planner uses the quantized 0.02 rad/s rate (not unquantized 0.03), does not double-floor
    //    wire velocity to raw 1 (0.01 rad/s), and never emits vel_raw == 0 (even during braking).
    assert_eq!(damiao::pos_force_vel_raw(0.03), 2);
    let (mut sup_full, _, _) = make_sup(Some(0.03), -5.7);
    let ep_full = sup_full.safety.epoch_id();
    let (_, expected_full_q) = { let s = sup_full.current_admission_stamp(); { let __s = s; sup_full.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
        GripperRequest { command: GripperCommand::Move, fraction: Some(0.0), grasp: false, speed: 1.0, timeout_s: None, tag: None },
        ep_full,
    )) } }.unwrap();
    assert!(expected_full_q >= 5.7 / 0.02, "expected quantized duration >= 285 s, got {expected_full_q:.2} s");

    // 0.20 rad at 0.02 rad/s takes ~10.25 s: fails under unquantized 0.03 timeout (~9.4 s) AND
    // fails if double-floored to raw 1 (20 s vs 12.75 s timeout).
    let (mut sup_q, inner_q, clock_q) = make_sup(Some(0.03), -0.20);
    inner_q.clear_log();
    let ep_q = sup_q.safety.epoch_id();
    let (id_q, expected_q) = { let s = sup_q.current_admission_stamp(); { let __s = s; sup_q.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
        GripperRequest { command: GripperCommand::Move, fraction: Some(0.0), grasp: false, speed: 1.0, timeout_s: None, tag: None },
        ep_q,
    )) } }.unwrap();
    assert!(expected_q >= 0.20 / 0.02, "quantized expected_duration {expected_q:.2} s < 10.0 s");
    let aq = run_to_end(&mut sup_q, &clock_q, id_q, 15.0);
    assert_eq!(aq.state, "done", "quantized 0.03->0.02 close failed: {aq:?}");
    assert!(sup_q.measured_positions[J7].abs() < 0.05, "quantized J7 {}", sup_q.measured_positions[J7]);
    let q_vel_raws: Vec<u16> = j7_commands(&inner_q.tx_log()).iter()
        .map(|f| { assert_eq!(f.id, 0x307); u16::from_le_bytes([f.data[4], f.data[5]]) })
        .collect();
    assert!(!q_vel_raws.is_empty());
    assert!(q_vel_raws.iter().all(|&r| (1..=2).contains(&r)), "wire speed out of 1..=2: {q_vel_raws:?}");
    assert!(q_vel_raws.contains(&2), "wire speed double-floored below raw 2: {q_vel_raws:?}");

    // 5. Zero-encoding speeds (< 0.01 rad/s wire quantization, including 0.01_f32 which rounds
    //    below 0.01 in IEEE-754) are rejected across config validation, driver builder, per-action
    //    admission (without mutating supervisor state), and direct final-writer batches (0 frames sent).
    for bad_speed in [0.005_f32, 0.01_f32] {
        assert_eq!(damiao::pos_force_vel_raw(bad_speed), 0);
        let mut bad_cfg = ArmConfig {
            damiao_gripper: Some(DamiaoGripperDrive { close_speed_rad_s: bad_speed, ..DamiaoGripperDrive::rebot_dm() }),
            ..ArmConfig::rebot_dm()
        };
        assert!(bad_cfg.validate().is_err(), "PosForce config accepted zero-encoding close_speed {bad_speed}");
        let bus = FakeBus::new(&b601_limits());
        assert!(
            DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
                .with_gripper_pos_force(GripperPosForce { close_speed_rad_s: bad_speed, ..pf() })
                .is_err(),
            "driver accepted zero-encoding close_speed {bad_speed}"
        );

        // Per-action admission rejects both closing and opening zero-encoding speeds without
        // committing action state or mutating releasing / target_positions.
        for cmd in [GripperCommand::Close, GripperCommand::Open] {
            let (mut sup_bad, _, _) = make_sup(Some(0.30), -1.0);
            let ep = sup_bad.safety.epoch_id();
            let prev_targets = sup_bad.target_positions.clone();
            let prev_releasing = sup_bad.releasing;
            let err = { let s = sup_bad.current_admission_stamp(); { let __s = s; sup_bad.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
                GripperRequest { command: cmd, fraction: None, grasp: false, speed: f64::from(bad_speed), timeout_s: None, tag: None },
                ep,
            )) } };
            assert!(err.is_err(), "start_gripper accepted zero-encoding speed {bad_speed} for {cmd:?}");
            assert!(sup_bad.active_trajectory.is_none() && sup_bad.last_action.is_none());
            assert_eq!(sup_bad.releasing, prev_releasing);
            assert_eq!(sup_bad.target_positions, prev_targets);
        }

        // Direct final-writer batch with nonzero sub-quantum velocity fails closed before frame 0.
        let (mut d, bus, _) = rig(true);
        d.set_torque(true).unwrap();
        d.read_joint_states().unwrap();
        bus.clear_log();
        let mut t = targets(-0.5);
        t[J7].velocity = Some(bad_speed);
        assert!(d.write_joint_targets(&t).is_err(), "final writer accepted zero-encoding velocity {bad_speed}");
        assert!(bus.tx_log().is_empty(), "sent partial batch on zero-encoding velocity {bad_speed}: {:?}", bus.tx_log());
        // Hold (velocity = Some(0.0) or None) stays valid with vel_raw >= 1.
        t[J7].velocity = Some(0.0);
        d.write_joint_targets(&t).expect("hold velocity=0.0 must succeed");
        let hold_f = j7_commands(&bus.tx_log())[0];
        assert_eq!(u16::from_le_bytes([hold_f.data[4], hold_f.data[5]]), damiao::pos_force_vel_raw(0.30));
    }
    assert_eq!(damiao::pos_force_vel_raw(0.0101), 1);
    ArmConfig {
        damiao_gripper: Some(DamiaoGripperDrive { close_speed_rad_s: 0.0101, ..DamiaoGripperDrive::rebot_dm() }),
        ..ArmConfig::rebot_dm()
    }.validate().expect("0.0101 rad/s quantizes to raw 1 and must validate");
    ArmConfig {
        damiao_gripper: Some(DamiaoGripperDrive { close_speed_rad_s: 0.005, ..DamiaoGripperDrive::legacy_pos_vel() }),
        ..ArmConfig::rebot_dm()
    }.validate().expect("legacy PosVel ignores pos-force wire quantization");

    // 6. Joint max_velocity ceiling contract across config validation, servo streaming,
    //    whole-arm quintic references, and stream braking:
    //    - ArmConfig::validate rejects J7 max_velocity < 0.01 rad/s in PosForce mode.
    //    - Even if an unvalidated config sets J7 max_velocity = 0.005, streaming does NOT silently
    //      raise velocity to raw 1 (0.01 rad/s): it fails closed with 0 CAN frames sent.
    //    - With valid limits (>= 0.01 rad/s), whole-arm quintic and servo stream + braking complete
    //      without fault and emit only nonzero vel_raw (>= 1) frames.
    for bad_lim in [0.005_f32, 0.01_f32] {
        let mut cfg_bad_lim = ArmConfig::rebot_dm();
        cfg_bad_lim.limits[J7].max_velocity = bad_lim;
        assert!(cfg_bad_lim.validate().is_err(), "PosForce config accepted sub-quantum J7 max_velocity {bad_lim}");
    }
    let mut cfg_ok_lim = ArmConfig::rebot_dm();
    cfg_ok_lim.limits[J7].max_velocity = 0.0101;
    cfg_ok_lim.validate().expect("J7 max_velocity 0.0101 must validate");
    let mut cfg_pv_lim = ArmConfig { damiao_gripper: Some(DamiaoGripperDrive::legacy_pos_vel()), ..ArmConfig::rebot_dm() };
    cfg_pv_lim.limits[J7].max_velocity = 0.005;
    cfg_pv_lim.validate().expect("legacy PosVel ignores pos-force J7 max_velocity quantization");

    // Unvalidated sub-quantum J7 limit on servo stream: must NOT silently bump 0.005 -> raw 1 (0.01).
    {
        let (mut sup_lim, inner_lim, clock_lim) = make_sup(Some(0.30), -1.0);
        sup_lim.config.limits[J7].max_velocity = 0.005;
        sup_lim.config.joint_tracking = None;
        let mut tgt = sup_lim.commanded_positions.clone();
        tgt[J7] = -0.5;
        sup_lim.servo_target = Some(tgt);
        sup_lim.last_motion_frame = std::time::Instant::now();
        inner_lim.clear_log();
        clock_lim.advance(Duration::from_millis(20));
        sup_lim.tick(DT);
        let lim_tx = inner_lim.tx_log();
        let lim_arm: Vec<_> = lim_tx.iter().filter(|f| (0x101..=0x106).contains(&f.id)).collect();
        assert!(lim_arm.is_empty(), "sub-quantum J7 limit 0.005 must not send arm motion frames: {lim_tx:?}");
        let lim_j7 = j7_commands(&lim_tx);
        assert_eq!(lim_j7.len(), 1, "expected 1 stationary vel_raw=0 hold frame from R1 tick epilogue: {lim_j7:?}");
        let lim_pos = f32::from_le_bytes(lim_j7[0].data[0..4].try_into().unwrap());
        let lim_vel = u16::from_le_bytes([lim_j7[0].data[4], lim_j7[0].data[5]]);
        assert!((lim_pos - (-1.0)).abs() < 1e-3, "invalid target -0.5 must not reach wire: {lim_pos}");
        assert_eq!(lim_vel, 0, "sub-quantum J7 limit 0.005 must not be bumped to vel_raw >= 1: {lim_j7:?}");
        assert!(sup_lim.safety.state().is_stopped(), "expected fault on unrepresentable stream ceiling");
    }

    // Valid J7 limit (2.0 rad/s): whole-arm quintic (jog) moving J7 and servo stream + stream
    // braking both emit exclusively vel_raw >= 1 (zero faults, zero vel_raw == 0 frames).
    {
        let (mut sup_qnt, inner_qnt, clock_qnt) = make_sup(Some(0.30), -1.0);
        sup_qnt.config.joint_tracking = None;
        let ep = sup_qnt.safety.epoch_id();
        let mut goal = sup_qnt.commanded_positions.clone();
        goal[J7] = -0.8;
        inner_qnt.clear_log();
        let (_, id, _) = { let s = sup_qnt.current_admission_stamp(); { let __s = s; sup_qnt.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, goal, Some(1.0), Some(ep), "jog")) } }.unwrap();
        let a = run_to_end(&mut sup_qnt, &clock_qnt, id, 5.0);
        assert_eq!(a.state, "done", "whole-arm quintic moving J7 failed: {a:?}");
        let qnt_raws: Vec<u16> = j7_commands(&inner_qnt.tx_log()).iter()
            .map(|f| { assert_eq!(f.id, 0x307); u16::from_le_bytes([f.data[4], f.data[5]]) })
            .collect();
        assert!(!qnt_raws.is_empty());
        assert!(qnt_raws.iter().all(|&r| (1..=damiao::pos_force_vel_raw(0.30)).contains(&r)),
            "quintic emitted invalid vel_raw: {qnt_raws:?}");

        // Servo stream moving J7 followed by stream drop -> stream braking to 0.0.
        let mut tgt = sup_qnt.commanded_positions.clone();
        tgt[J7] = -0.5;
        sup_qnt.servo_target = Some(tgt);
        sup_qnt.last_motion_frame = std::time::Instant::now();
        inner_qnt.clear_log();
        for _ in 0..10 {
            clock_qnt.advance(Duration::from_millis(20));
            sup_qnt.tick(DT);
            assert!(!sup_qnt.safety.state().is_stopped(), "servo stream faulted: {:?}", sup_qnt.safety.state());
        }
        sup_qnt.servo_target = None; // trigger stream braking
        for _ in 0..20 {
            clock_qnt.advance(Duration::from_millis(20));
            sup_qnt.tick(DT);
            assert!(!sup_qnt.safety.state().is_stopped(), "stream braking faulted: {:?}", sup_qnt.safety.state());
        }
        assert_eq!(sup_qnt.commanded_velocities[J7], 0.0, "stream braking did not come to rest");
        let stream_raws: Vec<u16> = j7_commands(&inner_qnt.tx_log()).iter()
            .map(|f| { assert_eq!(f.id, 0x307); u16::from_le_bytes([f.data[4], f.data[5]]) })
            .collect();
        assert!(!stream_raws.is_empty());
        assert!(stream_raws.iter().all(|&r| (1..=damiao::pos_force_vel_raw(0.30)).contains(&r)),
            "servo stream/braking emitted invalid vel_raw: {stream_raws:?}");
    }

    // 7. Zero-velocity position targets (grasp squeeze, settle, and idle holds) must respect the
    //    active authorized speed ceiling and never jump to close_speed_rad_s (0.30 -> raw 30).
    {
        // 7a. Low-speed grasp (request.speed = 0.025 -> raw 2 = 0.02 rad/s) hitting an object stop:
        //     travel, contact, squeeze (v == 0.0 with displaced target), settle, and the final
        //     completion tick all stay in 1..=2 (never jumping to close_speed_rad_s raw 30).
        let (mut sup_g, inner_g, clock_g) = make_sup(Some(0.30), -0.60);
        inner_g.with(|s| s.motors[J7].stop_at = Some(-0.50));
        inner_g.clear_log();
        let ep = sup_g.safety.epoch_id();
        let (gid, _) = { let s = sup_g.current_admission_stamp(); { let __s = s; sup_g.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 0.025, timeout_s: None, tag: None },
            ep,
        )) } }.unwrap();
        let ga = run_to_end(&mut sup_g, &clock_g, gid, 15.0);
        assert_eq!((ga.state.as_str(), ga.holding), ("done", Some(true)), "low-speed grasp failed: {ga:?}");
        let grasp_raws: Vec<u16> = j7_commands(&inner_g.tx_log()).iter()
            .map(|f| { assert_eq!(f.id, 0x307); u16::from_le_bytes([f.data[4], f.data[5]]) })
            .collect();
        assert!(!grasp_raws.is_empty());
        assert!(grasp_raws.iter().all(|&r| (1..=2).contains(&r)),
            "low-speed grasp squeeze/settle exceeded requested raw 2: {grasp_raws:?}");
        assert!(grasp_raws.contains(&2));

        // 7a-post. Persistent post-completion hold + object yield + autonomous driver recovery:
        //          retained squeeze target must stay at raw 1..=2 across idle ticks, when the
        //          held object yields, and during a 60 ms feedback-gap recovery hold.
        inner_g.clear_log();
        for _ in 0..15 {
            clock_g.advance(Duration::from_millis(20));
            sup_g.tick(DT);
        }
        // Object yields: jaw now chases the retained squeeze target at <= 0.02 rad/s (raw 2).
        inner_g.with(|s| s.motors[J7].stop_at = None);
        for _ in 0..15 {
            clock_g.advance(Duration::from_millis(20));
            sup_g.tick(DT);
        }
        let post_raws: Vec<u16> = j7_commands(&inner_g.tx_log()).iter()
            .map(|f| { assert_eq!(f.id, 0x307); u16::from_le_bytes([f.data[4], f.data[5]]) })
            .collect();
        assert!(!post_raws.is_empty());
        assert!(post_raws.iter().all(|&r| (1..=2).contains(&r)),
            "post-completion hold / object yield exceeded requested raw 2: {post_raws:?}");

        // Autonomous driver recovery hold (60 ms J7 feedback gap, still <= 100 ms freshness bound)
        // during the low-speed hold stays at raw 2 (never jumps to close_speed_rad_s raw 30).
        inner_g.with(|s| s.motors[J7].mute_feedback = true);
        clock_g.advance(Duration::from_millis(60));
        inner_g.with(|s| for i in 0..6 { s.push_feedback(i); });
        inner_g.clear_log();
        sup_g.tick(DT);
        let rec_low_raws: Vec<u16> = j7_commands(&inner_g.tx_log()).iter()
            .map(|f| { assert_eq!(f.id, 0x307); u16::from_le_bytes([f.data[4], f.data[5]]) })
            .collect();
        assert!(!rec_low_raws.is_empty(), "expected recovery hold frame at 60 ms");
        assert!(rec_low_raws.iter().all(|&r| (1..=2).contains(&r)),
            "autonomous recovery bypassed low-speed ceiling raw 2: {rec_low_raws:?}");

        // Subsequent explicit Open at 1.0 rad/s replaces the low-speed hold ceiling.
        inner_g.with(|s| { s.motors[J7].mute_feedback = false; s.push_all_feedback(); });
        sup_g.tick(DT);
        inner_g.clear_log();
        let ep_open = sup_g.safety.epoch_id();
        let (oid, _) = { let s = sup_g.current_admission_stamp(); { let __s = s; sup_g.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: None, tag: None },
            ep_open,
        )) } }.unwrap();
        let oa = run_to_end(&mut sup_g, &clock_g, oid, 10.0);
        assert_eq!(oa.state, "done", "{oa:?}");
        let open_raws: Vec<u16> = j7_commands(&inner_g.tx_log()).iter()
            .map(|f| { assert_eq!(f.id, 0x307); u16::from_le_bytes([f.data[4], f.data[5]]) })
            .collect();
        assert!(open_raws.contains(&100), "subsequent Open stayed stuck at old hold ceiling: {open_raws:?}");

        // 7b. Joint ceiling below close_speed_rad_s (0.15 < 0.30): gripper_drive_from_profile
        //     caps close_speed_rad_s to 0.15, idle holds use raw 15, AND autonomous driver
        //     recovery at 60 ms stays at raw 15 (never jumps to raw 30).
        let mut cfg_15 = ArmConfig::rebot_dm();
        cfg_15.limits[J7].max_velocity = 0.15;
        match gripper_drive_from_profile(&cfg_15).unwrap() {
            Some(GripperDrive::PosForce(p)) => assert_eq!(p.close_speed_rad_s, 0.15),
            other => panic!("expected PosForce capped at 0.15, got {other:?}"),
        }
        let (mut sup_low, inner_low, clock_low) = make_sup(Some(0.30), -1.0);
        sup_low.config.limits[J7].max_velocity = 0.15;
        inner_low.clear_log();
        clock_low.advance(Duration::from_millis(20));
        sup_low.tick(DT);
        let low_raws: Vec<u16> = j7_commands(&inner_low.tx_log()).iter()
            .map(|f| { assert_eq!(f.id, 0x307); u16::from_le_bytes([f.data[4], f.data[5]]) })
            .collect();
        assert!(!low_raws.is_empty());
        assert!(low_raws.iter().all(|&r| r == 15), "zero-velocity hold bypassed J7 ceiling 0.15: {low_raws:?}");

        // Autonomous driver recovery (60 ms gap) under J7 ceiling 0.15 stays at raw 15 (not 30).
        inner_low.with(|s| s.motors[J7].mute_feedback = true);
        clock_low.advance(Duration::from_millis(60));
        inner_low.with(|s| for i in 0..6 { s.push_feedback(i); });
        inner_low.clear_log();
        sup_low.tick(DT);
        let rec_15_raws: Vec<u16> = j7_commands(&inner_low.tx_log()).iter()
            .map(|f| { assert_eq!(f.id, 0x307); u16::from_le_bytes([f.data[4], f.data[5]]) })
            .collect();
        assert!(!rec_15_raws.is_empty(), "expected recovery hold at 60 ms under 0.15 ceiling");
        assert!(rec_15_raws.iter().all(|&r| r == 15),
            "autonomous recovery bypassed J7 ceiling 0.15: {rec_15_raws:?}");

        // 7c. Unvalidated sub-quantum J7 ceiling (0.005) on a zero-velocity tick fails closed:
        //     0 arm motion frames, latches fault, and the single J7 tick-epilogue emitter sends
        //     only a stationary hold (vel_raw == 0 at measured -1.0; never bumping 0.005 to vel_raw >= 1).
        inner_low.with(|s| { s.motors[J7].mute_feedback = false; s.push_all_feedback(); });
        sup_low.tick(DT);
        sup_low.config.limits[J7].max_velocity = 0.005;
        inner_low.clear_log();
        clock_low.advance(Duration::from_millis(20));
        sup_low.tick(DT);
        let j7_7c = j7_commands(&inner_low.tx_log());
        assert_eq!(j7_7c.len(), 1, "single J7 epilogue emitter must send 1 stationary hold: {:?}", inner_low.tx_log());
        assert_eq!(
            u16::from_le_bytes([j7_7c[0].data[4], j7_7c[0].data[5]]),
            0,
            "zero-velocity tick bypassed unrepresentable ceiling 0.005 (must hold at vel_raw=0): {:?}",
            j7_7c[0]
        );
        let hold_p_7c = f32::from_le_bytes(j7_7c[0].data[0..4].try_into().unwrap());
        assert!((hold_p_7c - (-1.0)).abs() < 1e-2, "hold pos {hold_p_7c} != measured -1.0");
        assert!(sup_low.safety.state().is_stopped());
    }

    // 8. Macro gripper waits reconcile with planned slow-close duration and overall macro deadline:
    //    - A close whose expected_duration exceeds remaining macro budget is rejected before install.
    //    - A close whose expected_duration exceeds phase_timeout_s but fits within the overall macro
    //      deadline is NOT killed at phase_timeout_s and completes cleanly.
    {
        let rt = tokio::runtime::Builder::new_current_thread().enable_time().build().unwrap();
        rt.block_on(async {
            // 8a. Close needing ~285 s inside a 60 s macro budget -> rejected before installing.
            let (sup_inner, _, _) = make_sup(Some(0.03), -5.7);
            let sup = Arc::new(armd::supervisor::ArmSupervisor::new(ArmConfig::rebot_dm()));
            let (id, stamp) = {
                let mut inner = sup.inner().write().await;
                *inner = *sup_inner;
                let ep = inner.safety.epoch_id();
                let stamp = inner.current_admission_stamp();
                ({ let __s = stamp; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_sequence_action(__a, None, ep)) }.unwrap(), stamp)
            };
            let ctx = armd::macro_action::MacroCtx {
                sup: sup.clone(),
                id,
                stamp,
                deadline: std::time::Instant::now() + Duration::from_secs(60),
                phase_timeout_s: 25.0,
                timeout_code: "GRASP_TIMEOUT",
                timeout_message: "macro timeout".into(),
            };
            let res = ctx.gripper("close", GripperCommand::Close).await;
            match res {
                Err(armd::macro_action::Stop::Fail(code, msg)) => {
                    assert_eq!(code, "GRASP_TIMEOUT");
                    assert!(msg.contains("remaining macro budget"), "{msg}");
                }
                other => panic!("expected insufficient-budget rejection, got {other:?}"),
            }
            assert!(sup.inner().read().await.active_trajectory.is_none(), "installed over-budget sub-motion");

            // 8b. Close exceeding phase_timeout_s (0.05 s) but fitting overall deadline (60 s) completes.
            let (sup_inner2, _, clock2) = make_sup(Some(0.10), -0.10);
            let sup2 = Arc::new(armd::supervisor::ArmSupervisor::new(ArmConfig::rebot_dm()));
            let (id2, stamp2) = {
                let mut inner = sup2.inner().write().await;
                *inner = *sup_inner2;
                let ep = inner.safety.epoch_id();
                let stamp = inner.current_admission_stamp();
                ({ let __s = stamp; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_sequence_action(__a, None, ep)) }.unwrap(), stamp)
            };
            let ctx2 = armd::macro_action::MacroCtx {
                sup: sup2.clone(),
                id: id2,
                stamp: stamp2,
                deadline: std::time::Instant::now() + Duration::from_secs(60),
                phase_timeout_s: 0.05, // deliberately shorter than the ~120 ms real wait
                timeout_code: "GRASP_TIMEOUT",
                timeout_message: "macro timeout".into(),
            };
            let sup_tick = sup2.clone();
            let ticker = tokio::spawn(async move {
                // Pace ticks past the 50 ms phase_timeout_s so any clamp to phase_timeout_s fails.
                for _ in 0..100 {
                    tokio::time::sleep(Duration::from_millis(2)).await;
                    clock2.advance(Duration::from_millis(20));
                    let mut inner = sup_tick.inner().write().await;
                    inner.tick(DT);
                    if inner.active_trajectory.is_none() && inner.measured_positions[J7].abs() < 0.05 {
                        break;
                    }
                }
            });
            let done = ctx2.gripper_move("close", 0.0).await.expect("slow macro close killed at phase_timeout_s");
            ticker.await.unwrap();
            assert!(done.gripper_gap_rad.is_some());
            assert!(sup2.inner().read().await.measured_positions[J7].abs() < 0.05);

            // 8c. pos_force open keeps timeout_s == 8.0; pos_vel close uses the unified
            //     gripper_policy.close_speed_rad_s (0.3 rad/s) with the derived 1.2x deadline;
            //     both ARE clamped to phase_timeout_s.
            for (label, speed_cfg, start_q, cmd, expect_explicit_8s) in [
                ("pos_vel_close", None, -5.7_f32, GripperCommand::Close, false),
                ("pos_force_open", Some(0.30_f32), 0.0_f32, GripperCommand::Open, true),
            ] {
                let (sup_inner3, _, _) = make_sup(speed_cfg, start_q);
                let sup3 = Arc::new(armd::supervisor::ArmSupervisor::new(ArmConfig::rebot_dm()));
                let (id3, stamp3) = {
                    let mut inner = sup3.inner().write().await;
                    *inner = *sup_inner3;
                    let ep = inner.safety.epoch_id();
                    let stamp = inner.current_admission_stamp();
                    ({ let __s = stamp; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_sequence_action(__a, None, ep)) }.unwrap(), stamp)
                };
                let ctx3 = armd::macro_action::MacroCtx {
                    sup: sup3.clone(),
                    id: id3,
                    stamp: stamp3,
                    deadline: std::time::Instant::now() + Duration::from_secs(60),
                    phase_timeout_s: 0.05,
                    timeout_code: "GRASP_TIMEOUT",
                    timeout_message: "macro timeout".into(),
                };
                let sup_watch = sup3.clone();
                let seen_timeout = Arc::new(std::sync::Mutex::new(None));
                let seen_out = seen_timeout.clone();
                let watcher = tokio::spawn(async move {
                    for _ in 0..20 {
                        if let Some(active) = &sup_watch.inner().read().await.active_trajectory {
                            if let armd::motion::Reference::Gripper(gm) = &active.reference {
                                *seen_out.lock().unwrap() = Some(gm.timeout_s);
                                break;
                            }
                        }
                        tokio::time::sleep(Duration::from_millis(2)).await;
                    }
                });
                let t0 = std::time::Instant::now();
                let res = ctx3.gripper(label, cmd).await;
                watcher.await.unwrap();
                assert!(t0.elapsed() < Duration::from_secs(1), "{label} was not clamped to phase_timeout_s");
                let got_timeout = *seen_timeout.lock().unwrap();
                if expect_explicit_8s {
                    assert_eq!(got_timeout, Some(8.0), "{label} did not keep timeout_s == 8.0");
                } else {
                    let t = got_timeout.expect("pos_vel_close timeout observed");
                    let min_t = armd::motion::GRIPPER_TRAVEL_TIME_FACTOR * (5.7 / 0.3);
                    assert!(
                        t >= min_t && t <= armd::motion::GRIPPER_MAX_DEFAULT_TIMEOUT_S,
                        "{label}: expected unified derived deadline in [{min_t:.2}, {}], got {t:.2}",
                        armd::motion::GRIPPER_MAX_DEFAULT_TIMEOUT_S
                    );
                }
                match res {
                    Err(armd::macro_action::Stop::Fail(code, _)) => assert_eq!(code, "GRASP_TIMEOUT", "{label}"),
                    other => panic!("{label}: expected GRASP_TIMEOUT at phase_timeout_s, got {other:?}"),
                }
            }

            // 8c-min. Unified closing speed: macro closes use min(gripper_policy.close_speed_rad_s,
            //         damiao_gripper.close_speed_rad_s) quantized to 0.01 rad/s in both orderings.
            for (policy_speed, damiao_speed, expect_planned) in [
                (0.20_f32, 0.30_f32, 0.20_f64),
                (0.30_f32, 0.259_f32, 0.25_f64),
            ] {
                let (mut sup_m, _, _) = make_sup(Some(damiao_speed), -1.0);
                sup_m.config.gripper_policy.as_mut().unwrap().close_speed_rad_s = Some(policy_speed);
                assert!((sup_m.effective_planned_close_speed() - expect_planned).abs() < 1e-9);
                let ep = sup_m.safety.epoch_id();
                let stamp = sup_m.current_admission_stamp();
                let adm_seq = sup_m.check_admission(stamp).unwrap();
                let id_m = SupervisorInner::begin_sequence_action(adm_seq, None, ep).unwrap();
                let req = GripperRequest {
                    command: GripperCommand::Close,
                    fraction: None,
                    grasp: true,
                    speed: f64::from(policy_speed),
                    timeout_s: None,
                    tag: None,
                };
                let adm_g = sup_m.check_admission(stamp.for_macro_child()).unwrap();
                SupervisorInner::start_macro_gripper_within(adm_g, id_m, req, Some(60.0), true).unwrap();
                match &sup_m.active_trajectory.as_ref().unwrap().reference {
                    armd::motion::Reference::Gripper(gm) => {
                        assert!((gm.planned_speed - expect_planned).abs() < 1e-9,
                            "policy={policy_speed} damiao={damiao_speed}: planned_speed {} != {expect_planned}",
                            gm.planned_speed);
                    }
                    other => panic!("expected Gripper reference, got {other:?}"),
                }
            }

            // 8d. sequences::expected_duration with a pos-force close step returns slow-close duration (~19 s).
            let (sup_seq, _, _) = make_sup(Some(0.30), -5.7);
            let seq_dur = armd::sequences::expected_duration(
                &sup_seq,
                &[(0, armd::sequences::PinnedStep::Gripper { action: armd::sequences::GripperAction::Close, fraction: None })],
                1,
                false,
            );
            assert!(seq_dur >= 1.0 + 5.7 / 0.30, "expected slow pos-force sequence duration >= 20 s, got {seq_dur:.2} s");

            // 8d-explicit. Public start_macro_gripper preserves an explicit caller-supplied timeout_s
            //              even on a pos-force close.
            {
                let (mut sup_exp, _, _) = make_sup(Some(0.30), -5.7);
                let ep_exp = sup_exp.safety.epoch_id();
                let id_exp = { let s = sup_exp.current_admission_stamp(); { let __s = s; sup_exp.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_sequence_action(__a, None, ep_exp)) } }.unwrap();
                let exp_req = GripperRequest { command: GripperCommand::Close, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(2.0), tag: None };
                { let s = sup_exp.current_admission_stamp(); { let __s = s; sup_exp.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_macro_gripper(__a, id_exp, exp_req)) } }.unwrap();
                match &sup_exp.active_trajectory.as_ref().unwrap().reference {
                    armd::motion::Reference::Gripper(gm) => assert_eq!(gm.timeout_s, 2.0),
                    other => panic!("expected Gripper reference, got {other:?}"),
                }
            }

            // 8e. Remaining macro budget computed under the supervisor write lock refuses installation
            //     if the lock was held past `ctx.deadline` (pos-force close, pos-force open, pos_vel close),
            //     and direct `start_macro_gripper_within` with budget <= 0 refuses before installing.
            for (label, speed_cfg, start_q, cmd) in [
                ("pos_force_close", Some(0.30_f32), -5.7_f32, GripperCommand::Close),
                ("pos_force_open", Some(0.30_f32), 0.0_f32, GripperCommand::Open),
                ("pos_vel_close", None, -5.7_f32, GripperCommand::Close),
            ] {
                let (sup_inner_b, _, _) = make_sup(speed_cfg, start_q);
                let sup_b = Arc::new(armd::supervisor::ArmSupervisor::new(ArmConfig::rebot_dm()));
                let (id_b, stamp_b) = {
                    let mut inner = sup_inner_b;
                    let ep = inner.safety.epoch_id();
                    let stamp = inner.current_admission_stamp();
                    let id = { let __s = stamp; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_sequence_action(__a, None, ep)) }.unwrap();
                    // Direct zero / negative budget checks before installing any trajectory.
                    let req = GripperRequest { command: cmd, fraction: None, grasp: false, speed: 1.0, timeout_s: Some(8.0), tag: None };
                    for bad_budget in [0.0_f64, -1.0_f64] {
                        let err = { let __s = stamp.for_macro_child(); inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_macro_gripper_within(__a, id, req.clone(), Some(bad_budget), true)) }.unwrap_err();
                        assert!(err.to_string().contains("remaining macro budget"), "{label} budget={bad_budget}: {err}");
                        assert!(inner.active_trajectory.is_none());
                        assert_ne!(inner.macro_sub(id), Some(armd::macro_action::SubState::Running));
                    }
                    *sup_b.inner().write().await = *inner;
                    (id, stamp)
                };
                let guard = sup_b.inner().write().await;
                let ctx_b = armd::macro_action::MacroCtx {
                    sup: sup_b.clone(),
                    id: id_b,
                    stamp: stamp_b,
                    deadline: std::time::Instant::now() + Duration::from_millis(40),
                    phase_timeout_s: 25.0,
                    timeout_code: "GRASP_TIMEOUT",
                    timeout_message: "macro timeout".into(),
                };
                let handle = tokio::spawn(async move { ctx_b.gripper(label, cmd).await });
                tokio::time::sleep(Duration::from_millis(80)).await;
                drop(guard);
                match handle.await.unwrap() {
                    Err(armd::macro_action::Stop::Fail(code, _)) => assert_eq!(code, "GRASP_TIMEOUT", "{label}"),
                    other => panic!("{label}: expected GRASP_TIMEOUT after held lock past deadline, got {other:?}"),
                }
                let inner_after = sup_b.inner().read().await;
                assert!(inner_after.active_trajectory.is_none(), "{label} installed trajectory past deadline");
                assert_ne!(inner_after.macro_sub(id_b), Some(armd::macro_action::SubState::Running), "{label} left sub running");
            }
        });
    }

    // 9. f64 speed quantization never rounds upward across a wire quantum via f64->f32 cast.
    for (req_speed, want_raw) in [(0.299_999_999_f64, 29u16), (0.049_999_999_f64, 4u16)] {
        assert_eq!(DamiaoGripperDrive::quantize_vel_raw_f64(req_speed), want_raw);
        let policy = ArmConfig::rebot_dm().gripper_policy.unwrap();
        let lim = ArmConfig::rebot_dm().limits[J7].clone();
        let gm = armd::motion::GripperMotion::try_with_close_speed_limit(
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: false, speed: req_speed, timeout_s: None, tag: None },
            policy, lim, -1.0, J7, Some(0.30),
        ).unwrap();
        assert_eq!(gm.planned_speed, f64::from(want_raw) / 100.0);
        assert!(gm.planned_speed <= req_speed, "planned_speed {} exceeded request {}", gm.planned_speed, req_speed);
        assert!(f64::from(gm.limit.max_velocity) <= f64::from(want_raw) / 100.0);
        assert_eq!(damiao::pos_force_vel_raw(gm.wire_velocity_ceiling()), want_raw);
    }

    // 10. step_speed_f32 vs wire_speed_f32 across representative raw values, and per-frame
    //     full close (-5.7 -> 0.0) at close_speed_rad_s = 0.299 (wire raw 29 = 0.29 rad/s).
    for raw in [1u16, 2, 3, 28, 29, 30, 100, 200] {
        let step = DamiaoGripperDrive::step_speed_f32(raw);
        let wire = DamiaoGripperDrive::wire_speed_f32(raw);
        assert!(f64::from(step) <= f64::from(raw) / 100.0, "raw={raw} step={step}");
        assert_eq!(DamiaoGripperDrive::quantize_vel_raw(wire), raw, "raw={raw} wire={wire}");
        assert!(f64::from(wire) >= f64::from(raw) / 100.0, "raw={raw} wire={wire}");
    }
    {
        let (mut sup_c, inner_c, clock_c) = make_sup(Some(0.299), -5.7);
        assert_eq!(damiao::pos_force_vel_raw(0.299), 29);
        inner_c.clear_log();
        let ep_c = sup_c.safety.epoch_id();
        let (id_c, _) = { let s = sup_c.current_admission_stamp(); { let __s = s; sup_c.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: false, speed: 1.0, timeout_s: None, tag: None },
            ep_c,
        )) } }.unwrap();
        let mut saw_cruise_29 = false;
        let mut frames_checked = 0usize;
        let mut done_action = None;
        for _ in 0..(25.0 / DT) as usize {
            let q_prev = sup_c.commanded_positions[J7];
            inner_c.clear_log();
            clock_c.advance(Duration::from_millis(20));
            sup_c.tick(DT);
            let q_curr = sup_c.commanded_positions[J7];
            for f in j7_commands(&inner_c.tx_log()) {
                assert_eq!(f.id, 0x307);
                let vel_raw = u16::from_le_bytes([f.data[4], f.data[5]]);
                assert!((1..=29).contains(&vel_raw), "vel_raw={vel_raw} out of 1..=29");
                assert_eq!(DamiaoGripperDrive::quantize_vel_raw(DamiaoGripperDrive::wire_speed_f32(vel_raw)), vel_raw);
                let wire_limit = f64::from(vel_raw) / 100.0;
                assert!(
                    f64::from(sup_c.commanded_velocities[J7].abs()) <= wire_limit,
                    "planner velocity {} exceeded wire_limit {} (vel_raw={vel_raw})",
                    sup_c.commanded_velocities[J7].abs(),
                    wire_limit
                );
                let step_rate = (f64::from(q_curr) - f64::from(q_prev)).abs() / DT;
                assert!(
                    step_rate <= wire_limit,
                    "stepped displacement rate {step_rate} exceeded wire_limit {wire_limit} (vel_raw={vel_raw})"
                );
                if vel_raw == 29 {
                    saw_cruise_29 = true;
                }
                frames_checked += 1;
            }
            let a = sup_c.last_action.clone().unwrap();
            assert_eq!(a.id, id_c);
            if !a.is_running() {
                done_action = Some(a);
                break;
            }
        }
        let a = done_action.expect("full close at 0.299 did not finish within 25s");
        assert_eq!(a.state, "done", "{a:?}");
        assert!(sup_c.measured_positions[J7].abs() < 0.05, "J7 ended at {}", sup_c.measured_positions[J7]);
        assert!(frames_checked > 0 && saw_cruise_29, "frames_checked={frames_checked} saw_cruise_29={saw_cruise_29}");
    }
    {
        // Grasp close (grasp: true) through contact and squeeze against a stop at -2.0 rad:
        // every 0x307 frame (accel, cruise, contact, squeeze entry, and squeeze hold) respects
        // the wire-encoded speed limit, AND the holding decision waits until the wire-stepped
        // squeeze command actually reaches the full -1.85 rad (-2.0 + 0.15) target (>= 0.52 s)
        // plus the 0.30 s settle window instead of deciding after 0.30 s short of target.
        let (mut sup_g, inner_g, clock_g) = make_sup(Some(0.299), -5.7);
        inner_g.with(|s| s.motors[J7].stop_at = Some(-2.0));
        inner_g.clear_log();
        let ep_g = sup_g.safety.epoch_id();
        let (id_g, expected_g) = { let s = sup_g.current_admission_stamp(); { let __s = s; sup_g.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 1.0, timeout_s: None, tag: None },
            ep_g,
        )) } }.unwrap();
        // expected_duration includes travel (5.7 / 0.29) + squeeze advance (0.15 / 0.29) + 0.5 + settle (0.3).
        assert!(
            expected_g >= 5.7 / 0.29 + 0.15 / 0.29 + 0.79,
            "expected_duration {expected_g:.3} s missing squeeze advance time"
        );
        let mut frames_checked = 0usize;
        let mut squeeze_start_tick: Option<usize> = None;
        let mut done_tick: Option<usize> = None;
        let mut max_cmd_q = f32::NEG_INFINITY;
        let mut done_action = None;
        for tick in 0..(25.0 / DT) as usize {
            let q_prev = sup_g.commanded_positions[J7];
            inner_g.clear_log();
            clock_g.advance(Duration::from_millis(20));
            sup_g.tick(DT);
            let q_curr = sup_g.commanded_positions[J7];
            max_cmd_q = max_cmd_q.max(q_curr);
            if squeeze_start_tick.is_none() && q_curr > -2.0 && sup_g.measured_positions[J7] <= -1.99 {
                squeeze_start_tick = Some(tick);
            }
            for f in j7_commands(&inner_g.tx_log()) {
                assert_eq!(f.id, 0x307);
                let vel_raw = u16::from_le_bytes([f.data[4], f.data[5]]);
                assert!((1..=29).contains(&vel_raw), "grasp vel_raw={vel_raw} out of 1..=29");
                let wire_limit = f64::from(vel_raw) / 100.0;
                assert!(
                    f64::from(sup_g.commanded_velocities[J7].abs()) <= wire_limit,
                    "grasp planner velocity {} exceeded wire_limit {} (vel_raw={vel_raw})",
                    sup_g.commanded_velocities[J7].abs(),
                    wire_limit
                );
                let step_rate = (f64::from(q_curr) - f64::from(q_prev)).abs() / DT;
                assert!(
                    step_rate <= wire_limit,
                    "grasp stepped displacement rate {step_rate} exceeded wire_limit {wire_limit} (vel_raw={vel_raw})"
                );
                frames_checked += 1;
            }
            let a = sup_g.last_action.clone().unwrap();
            assert_eq!(a.id, id_g);
            if !a.is_running() {
                done_tick = Some(tick);
                done_action = Some(a);
                break;
            }
        }
        let a = done_action.expect("grasp close at 0.299 did not finish within 25s");
        assert_eq!((a.state.as_str(), a.holding), ("done", Some(true)), "{a:?}");
        let sq_start = squeeze_start_tick.expect("never entered squeeze");
        let sq_dur_s = (done_tick.unwrap() - sq_start) as f64 * DT;
        assert!(
            max_cmd_q >= -1.855,
            "squeeze ended short of full -1.85 target: max_cmd_q={max_cmd_q:.4}, sq_dur_s={sq_dur_s:.2}"
        );
        assert!(
            sq_dur_s >= 0.78,
            "squeeze holding decided too early ({sq_dur_s:.2} s < ~0.52 s advance + 0.30 s settle)"
        );
        assert!(frames_checked > 0, "frames_checked={frames_checked}");

        // Pos-force opening keeps previous opening behaviour (no new wire stepping or rounding).
        let policy = ArmConfig::rebot_dm().gripper_policy.unwrap();
        let lim = ArmConfig::rebot_dm().limits[J7].clone();
        let open_gm = armd::motion::GripperMotion::try_with_close_speed_limit(
            GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 0.299, timeout_s: None, tag: None },
            policy, lim, 0.0, J7, Some(0.30),
        ).unwrap();
        assert_eq!(open_gm.planned_vel_raw, None, "opening should not enable close wire stepping");
        assert_eq!(open_gm.planned_speed, 0.299, "opening should keep requested speed without close rounding");
    }
}

#[test]
fn stalled_empty_jaw_close_c300_pattern_fails_instead_of_done() {
    use std::sync::{Arc, Mutex};
    use armd::driver::{ArmDriverError, DriverCapabilities, EnableState};
    use armd::model::{GripperPolicy, JointState};
    use armd::motion::{GripperCommand, GripperMotion, GripperRequest, StepCtx, StepStatus, GRIPPER_STOP_SHORT_RAD};
    use armd::supervisor::{HeldObject, ParkRelease, SupervisorInner};

    const DT: f64 = 0.02;

    // 1. Replay c300.json move 2: gripper_move(0.0) from -5.6401 stalls at -5.547 with ceiling
    //    torque 0.554-0.569 N·m and near-zero velocity -> must fail with GRIPPER_TIMEOUT, not done.
    struct C300StallState {
        q: [f32; 7],
        v: [f32; 7],
        tau: [f32; 7],
        enabled: EnableState,
        stall_at: Option<f32>,
        ceiling_nm: f32,
        block_open: bool,
    }
    struct C300StallDriver(Arc<Mutex<C300StallState>>);
    impl ArmDriver for C300StallDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
                torque_feedforward: false, enable_feedback: true, load_feedback: false }
        }
        fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            let s = self.0.lock().unwrap();
            Ok((0..7).map(|i| JointState { position: s.q[i], velocity: s.v[i], torque: s.tau[i], temperature: 30.0 }).collect())
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            let mut s = self.0.lock().unwrap();
            for (i, t) in targets.iter().enumerate().take(7) {
                if i != J7 { s.q[i] = t.position; s.v[i] = 0.0; continue; }
                let delta = t.position - s.q[J7];
                if s.block_open && delta < -1e-4 {
                    s.v[J7] = -0.007;
                    s.tau[J7] = -s.ceiling_nm;
                    continue;
                }
                if let Some(stop) = s.stall_at {
                    if delta > 0.0 && s.q[J7] + delta.min(0.30 * DT as f32) >= stop {
                        s.q[J7] = stop;
                        s.v[J7] = -0.007;
                        s.tau[J7] = s.ceiling_nm;
                        continue;
                    }
                }
                let step = delta.clamp(-0.30 * DT as f32, 0.30 * DT as f32);
                s.q[J7] += step;
                s.v[J7] = step / DT as f32;
                s.tau[J7] = if step.abs() > 1e-4 { 0.35 * step.signum() } else { 0.0 };
            }
            Ok(())
        }
        fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
            self.0.lock().unwrap().enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
            Ok(())
        }
    }

    let state = Arc::new(Mutex::new(C300StallState {
        q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5.6401],
        v: [0.0; 7],
        tau: [0.0; 7],
        enabled: EnableState::Disabled,
        stall_at: Some(-5.547),
        ceiling_nm: 0.564, // c300.json plateau 0.554-0.569 N·m
        block_open: false,
    }));
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(C300StallDriver(state.clone())));
    inner.lease_enabled = false;
    inner.tick(DT);
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
    let epoch = inner.safety.epoch_id();
    let req = GripperRequest { command: GripperCommand::Move, fraction: Some(0.0), grasp: false, speed: 1.0, timeout_s: None, tag: None };
    let (id, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a, req, epoch)) } }.unwrap();
    let mut finished = None;
    for _ in 0..200 {
        inner.tick(DT);
        let a = inner.last_action.clone().unwrap();
        if !a.is_running() { finished = Some(a); break; }
    }
    let a = finished.expect("stalled c300 close must finish on stall");
    assert_eq!(a.id, id);
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("failed", Some("GRIPPER_TIMEOUT")), "{a:?}");
    assert!(a.message.as_deref().unwrap_or("").contains("stalled short of target"), "{a:?}");
    assert!(inner.gripper_stalled && inner.gripper_stall_sign > 0.0);
    assert!((inner.commanded_positions[J7] - (-5.547)).abs() < 1e-3, "held at {}", inner.commanded_positions[J7]);

    // 2. Grasp close (grasp=true) hitting the same stalled ceiling torque keeps its contact=holding semantics.
    let grasp_state = Arc::new(Mutex::new(C300StallState {
        q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5.0],
        v: [0.0; 7],
        tau: [0.0; 7],
        enabled: EnableState::Disabled,
        stall_at: Some(-3.0),
        ceiling_nm: 0.564,
        block_open: false,
    }));
    let mut grasp_sup = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(C300StallDriver(grasp_state)));
    grasp_sup.lease_enabled = false;
    grasp_sup.tick(DT);
    { let __s = grasp_sup.current_admission_stamp(); grasp_sup.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
    let ep = grasp_sup.safety.epoch_id();
    let (gid, _) = { let s = grasp_sup.current_admission_stamp(); { let __s = s; grasp_sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
        GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 1.0, timeout_s: None, tag: None }, ep
    )) } }.unwrap();
    let mut ga = None;
    for _ in 0..800 {
        grasp_sup.tick(DT);
        let a = grasp_sup.last_action.clone().unwrap();
        if !a.is_running() { ga = Some(a); break; }
    }
    let ga = ga.expect("grasp close finished");
    assert_eq!(ga.id, gid);
    assert_eq!((ga.state.as_str(), ga.holding), ("done", Some(true)), "{ga:?}");

    // 3. Direct GripperMotion boundary checks: < 0.25 Done, == 0.25 Failed, > 0.25 Failed, NaN Failed,
    //    covering Move / Open / Close(grasp=false), while Seek stays Done.
    let policy = GripperPolicy::rebot_dm();
    let lim = ArmConfig::rebot_dm().limits[J7].clone();
    let step_stalled = |cmd: GripperCommand, frac: Option<f64>, start: f32, meas: f32, cmd_q: f32, sign: f32| {
        let req = GripperRequest { command: cmd, fraction: frac, grasp: false, speed: 1.0, timeout_s: Some(10.0), tag: None };
        let mut gm = GripperMotion::new(req, policy.clone(), lim.clone(), start, J7);
        let mut m = [0.0f32; 7];
        let mut q = [0.0f32; 7];
        m[J7] = meas;
        q[J7] = cmd_q;
        let ctx = StepCtx {
            dt: DT, time_scale: 1.0, measured: &m, measured_velocities: &[0.0; 7],
            commanded: &q, commanded_velocities: &[0.0; 7], torques: &[0.0; 7], loads: &[0.0; 7],
            feedback_fresh: true, torque_feedback: true, load_feedback: false,
            gripper_stalled: true, gripper_stall_sign: sign, gripper_force_capped: false,
        };
        gm.step(&ctx)
    };

    // Move(0.0) target = 0.0: < 0.25 -> Done; == 0.25 -> Failed; > 0.25 -> Failed; NaN -> Failed.
    let ok_short = step_stalled(GripperCommand::Move, Some(0.0), -1.0, -(GRIPPER_STOP_SHORT_RAD - 0.01), -0.24, 1.0);
    assert!(matches!(ok_short.status, StepStatus::Done { .. }), "{ok_short:?}");
    let eq_bound = step_stalled(GripperCommand::Move, Some(0.0), -1.0, -GRIPPER_STOP_SHORT_RAD, -0.25, 1.0);
    assert!(matches!(eq_bound.status, StepStatus::Failed { code: "GRIPPER_TIMEOUT", .. }), "{eq_bound:?}");
    assert!((eq_bound.positions[J7] - (-GRIPPER_STOP_SHORT_RAD)).abs() < 1e-5 && eq_bound.velocities[J7] == 0.0);
    let far_open = step_stalled(GripperCommand::Open, None, 0.0, -3.0, -3.2, -1.0);
    assert!(matches!(far_open.status, StepStatus::Failed { code: "GRIPPER_TIMEOUT", .. }), "{far_open:?}");
    assert!((far_open.positions[J7] - (-3.0)).abs() < 1e-5 && far_open.velocities[J7] == 0.0);
    let far_close = step_stalled(GripperCommand::Close, None, -5.0, -2.0, -1.8, 1.0);
    assert!(matches!(far_close.status, StepStatus::Failed { code: "GRIPPER_TIMEOUT", .. }), "{far_close:?}");
    let nan_meas = step_stalled(GripperCommand::Move, Some(0.0), -5.0, f32::NAN, -2.5, 1.0);
    assert!(matches!(nan_meas.status, StepStatus::Failed { code: "GRIPPER_TIMEOUT", .. }), "{nan_meas:?}");
    assert!((nan_meas.positions[J7] - (-2.5)).abs() < 1e-5 && nan_meas.velocities[J7] == 0.0);
    // Seek (zero_check) stalled far from +0.6 target still succeeds.
    let seek_ok = step_stalled(GripperCommand::Seek, None, -1.0, -0.3, -0.3, 1.0);
    assert!(matches!(seek_ok.status, StepStatus::Done { .. }), "{seek_ok:?}");

    // 4. Park-release stalled Opening fails without falsely clearing `held` or proceeding to Closing.
    let park_state = Arc::new(Mutex::new(C300StallState {
        q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.0],
        v: [0.0; 7],
        tau: [0.0; 7],
        enabled: EnableState::Disabled,
        stall_at: None,
        ceiling_nm: 0.60,
        block_open: true,
    }));
    let mut park_sup = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(C300StallDriver(park_state)));
    park_sup.lease_enabled = false;
    park_sup.tick(DT);
    { let __s = park_sup.current_admission_stamp(); park_sup.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
    park_sup.pending_normal_unload = true;
    park_sup.park_gripper_contact = true;
    park_sup.held = Some(HeldObject { label: "box".into(), spill_safe: false });
    // First tick installs ParkRelease::Opening; subsequent ticks stall while opening and abort park_release.
    for _ in 0..40 {
        park_sup.tick(DT);
        if park_sup.park_release.is_none() && park_sup.active_trajectory.is_none() && !park_sup.park_gripper_contact {
            break;
        }
    }
    assert!(park_sup.held.is_some(), "stalled park-release opening falsely cleared held");
    assert_ne!(park_sup.park_release, Some(ParkRelease::Closing), "proceeded to Closing after stalled Opening");
    assert!(park_sup.park_release.is_none() && !park_sup.park_gripper_contact);
}

#[test]
fn pos_force_slow_travel_high_torque_progress_window_and_true_stall() {
    use std::sync::{Arc, Mutex};
    use armd::driver::{ArmDriverError, DriverCapabilities, EnableState, MotorCurrentCap};
    use armd::model::JointState;
    use armd::motion::{GripperCommand, GripperRequest};
    use armd::supervisor::SupervisorInner;

    struct SlowTravelState {
        q_phys: f32,
        q_reported: f32,
        v_reported: f32,
        tau: f32,
        enabled: EnableState,
        stop_at: Option<f32>,
        travel_tau: f32,
        stall_tau: f32,
        speed: f32,
        dt: f32,
        tick_count: u64,
        fb_every_n_ticks: u64,
        jitter_amp: f32,
        close_speed_cap: Option<f32>,
    }
    struct SlowTravelDriver(Arc<Mutex<SlowTravelState>>);
    impl ArmDriver for SlowTravelDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
                torque_feedforward: false, enable_feedback: true, load_feedback: false }
        }
        fn motor_current_cap(&self) -> Option<MotorCurrentCap> {
            let cap = self.0.lock().unwrap().close_speed_cap?;
            Some(MotorCurrentCap {
                joint: J7,
                mode: "pos_force",
                current_raw: Some(460),
                active: true,
                calibrated: false,
                ceiling_torque_nm: None,
                torque_reference: None,
                close_speed_rad_s: Some(cap),
                note: "",
            })
        }
        fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            let s = self.0.lock().unwrap();
            Ok((0..7).map(|i| {
                if i == J7 {
                    JointState { position: s.q_reported, velocity: s.v_reported, torque: s.tau, temperature: 30.0 }
                } else {
                    JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }
                }
            }).collect())
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            let mut s = self.0.lock().unwrap();
            if let Some(t) = targets.get(J7) {
                s.tick_count += 1;
                let max_step = s.speed * s.dt;
                let delta = (t.position - s.q_phys).clamp(-max_step, max_step);
                let mut next_q = s.q_phys + delta;
                let hit_stop = if let Some(stop) = s.stop_at {
                    if next_q >= stop {
                        next_q = stop;
                        true
                    } else {
                        false
                    }
                } else {
                    false
                };
                s.q_phys = next_q;
                s.tau = if hit_stop { s.stall_tau } else if delta.abs() > 1e-6 { s.travel_tau } else { 0.0 };
                if s.tick_count.is_multiple_of(s.fb_every_n_ticks) {
                    let jitter = if s.jitter_amp > 0.0 {
                        if (s.tick_count / s.fb_every_n_ticks).is_multiple_of(2) { s.jitter_amp } else { -s.jitter_amp }
                    } else {
                        0.0
                    };
                    s.q_reported = s.q_phys + jitter;
                }
                // Deliberately report low instantaneous velocity (<= speed, e.g. 0.0) so only
                // windowed position progress distinguishes slow travel from a true stall.
                s.v_reported = 0.0;
            }
            Ok(())
        }
        fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
            self.0.lock().unwrap().enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
            Ok(())
        }
    }

    let make_sup = |start_q: f32, speed: f32, stop_at: Option<f32>, travel_tau: f32, stall_tau: f32, fb_every: u64, jitter: f32| {
        let st = Arc::new(Mutex::new(SlowTravelState {
            q_phys: start_q,
            q_reported: start_q,
            v_reported: 0.0,
            tau: 0.0,
            enabled: EnableState::Disabled,
            stop_at,
            travel_tau,
            stall_tau,
            speed,
            dt: 0.02,
            tick_count: 0,
            fb_every_n_ticks: fb_every,
            jitter_amp: jitter,
            close_speed_cap: None,
        }));
        let mut sup = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(SlowTravelDriver(st.clone())));
        sup.lease_enabled = false;
        sup.tick(0.02);
        { let __s = sup.current_admission_stamp(); sup.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.unwrap();
        sup.tick(0.02);
        (sup, st)
    };

    // 1a. Slow non-grasp travel at 0.06 rad/s near ceiling torque (0.55 N·m >= 0.444 N·m)
    //     completes cleanly without false stall latch.
    {
        let (mut sup, _) = make_sup(-0.5, 0.06, None, 0.55, 0.55, 1, 0.0);
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: false, speed: 0.06, timeout_s: None, tag: None },
            ep,
        )) } }.unwrap();
        let mut done = None;
        for _ in 0..800 {
            sup.tick(0.02);
            assert!(!sup.gripper_stalled, "0.06 rad/s free travel falsely latched stall at q={}", sup.measured_positions[J7]);
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() { done = Some(a); break; }
        }
        let a = done.expect("0.06 rad/s non-grasp close did not finish");
        assert_eq!((a.id, a.state.as_str()), (id, "done"), "{a:?}");
        assert!(sup.measured_positions[J7].abs() < 0.05);
    }

    // 1b. Slow grasp at 0.06 rad/s with 0.55 N·m travel torque advances from -2.5 to object at -2.0
    //     without premature Squeeze, then holds cleanly at -2.0.
    {
        let (mut sup, _) = make_sup(-2.5, 0.06, Some(-2.0), 0.55, 0.55, 1, 0.0);
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 0.06, timeout_s: None, tag: None },
            ep,
        )) } }.unwrap();
        let mut done = None;
        for _ in 0..1000 {
            sup.tick(0.02);
            if sup.measured_positions[J7] < -2.02 {
                assert!(!sup.gripper_stalled, "0.06 rad/s grasp falsely stalled before -2.0 at {}", sup.measured_positions[J7]);
                if let Some(active) = &sup.active_trajectory {
                    if let armd::motion::Reference::Gripper(gm) = &active.reference {
                        assert!(gm.contact_rad.is_none(), "premature contact at {}", sup.measured_positions[J7]);
                    }
                }
            }
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() { done = Some(a); break; }
        }
        let a = done.expect("0.06 rad/s grasp did not finish");
        assert_eq!((a.id, a.state.as_str(), a.holding), (id, "done", Some(true)), "{a:?}");
        assert!((sup.measured_positions[J7] - (-2.0)).abs() < 0.05, "ended at {}", sup.measured_positions[J7]);
    }

    // 2. Slow travel at raw speeds 1, 2, 3 (0.01, 0.02, 0.03 rad/s) at 0.55 N·m with repeated
    //    feedback samples (updating every 2 ticks / 40 ms cadence) AND variable dt (0.01, 0.02, 0.04 s)
    //    completes without false stall latch or premature squeeze.
    let dts = [0.01_f64, 0.02, 0.04];
    for speed in [0.01_f32, 0.02, 0.03] {
        let (mut sup, st) = make_sup(-0.12, speed, None, 0.55, 0.55, 2, 0.0);
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: false, speed: f64::from(speed), timeout_s: None, tag: None },
            ep,
        )) } }.unwrap();
        let mut done = None;
        for step in 0..1500 {
            let dt = dts[step % dts.len()];
            st.lock().unwrap().dt = dt as f32;
            sup.tick(dt);
            assert!(!sup.gripper_stalled, "speed={speed} falsely latched stall at step={step} q={}", sup.measured_positions[J7]);
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() { done = Some(a); break; }
        }
        let a = done.unwrap_or_else(|| panic!("speed={speed} variable-dt repeated-fb close did not finish"));
        assert_eq!((a.id, a.state.as_str()), (id, "done"), "speed={speed}: {a:?}");
        assert!(sup.measured_positions[J7].abs() < 0.05, "speed={speed} ended at {}", sup.measured_positions[J7]);
    }
    // Also verify grasp=true at raw 1 (0.01 rad/s) with 40 ms repeated feedback + variable dt
    // advances from -0.50 to stop -0.40 without premature squeeze before holding.
    {
        let (mut sup, st) = make_sup(-0.50, 0.01, Some(-0.40), 0.55, 0.55, 2, 0.0);
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 0.01, timeout_s: None, tag: None },
            ep,
        )) } }.unwrap();
        let mut done = None;
        for step in 0..1500 {
            let dt = dts[step % dts.len()];
            st.lock().unwrap().dt = dt as f32;
            sup.tick(dt);
            if sup.measured_positions[J7] < -0.42 {
                assert!(!sup.gripper_stalled, "0.01 rad/s grasp falsely stalled at {}", sup.measured_positions[J7]);
                if let Some(active) = &sup.active_trajectory {
                    if let armd::motion::Reference::Gripper(gm) = &active.reference {
                        assert!(gm.contact_rad.is_none(), "0.01 rad/s grasp premature contact at {}", sup.measured_positions[J7]);
                    }
                }
            }
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() { done = Some(a); break; }
        }
        let a = done.expect("0.01 rad/s variable-dt grasp did not finish");
        assert_eq!((a.id, a.state.as_str(), a.holding), (id, "done", Some(true)), "{a:?}");
        assert!((sup.measured_positions[J7] - (-0.40)).abs() < 0.05, "ended at {}", sup.measured_positions[J7]);
    }

    // 3. True stall (c300 pattern at -5.547 rad, 0.56 N·m, with +-0.0004 rad = 0.0008 rad
    //    peak-to-peak encoder jitter) still latches stall and fails with GRIPPER_TIMEOUT within <= 0.5 s.
    {
        let (mut sup, st) = make_sup(-5.6401, 0.30, Some(-5.547), 0.35, 0.56, 1, 0.0004);
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Move, fraction: Some(0.0), grasp: false, speed: 1.0, timeout_s: Some(30.0), tag: None },
            ep,
        )) } }.unwrap();
        let mut hit_stop_tick: Option<usize> = None;
        let mut done_tick: Option<usize> = None;
        let mut done = None;
        for tick in 0..200 {
            sup.tick(0.02);
            if hit_stop_tick.is_none() && (st.lock().unwrap().q_phys - (-5.547)).abs() < 1e-4 {
                hit_stop_tick = Some(tick);
            }
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() {
                done_tick = Some(tick);
                done = Some(a);
                break;
            }
        }
        let a = done.expect("true stall with 0.0008 rad peak-to-peak jitter did not finish");
        assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "failed", Some("GRIPPER_TIMEOUT")), "{a:?}");
        assert!(sup.gripper_stalled && sup.gripper_stall_sign > 0.0);
        let detect_s = (done_tick.unwrap() - hit_stop_tick.expect("never reached -5.547")) as f64 * 0.02;
        assert!(detect_s <= 0.50, "c300 stall took {detect_s:.2} s (> 0.50 s bound)");
    }

    // 3b. Oscillating jam (+-0.005 rad = 0.010 rad peak-to-peak, 5x the 0.002 rad progress eps)
    //     around -3.0 rad with a 30 s action timeout must NOT keep resetting the stall timer and
    //     MUST fail with GRIPPER_TIMEOUT within <= 0.50 s after progress stops.
    for (label, planned_speed, cap_opt) in [
        ("fast_0.30", 1.0_f64, Some(0.30_f32)),
        ("slow_0.06", 0.06_f64, None),
    ] {
        let (mut sup, st) = make_sup(-3.10, 0.30, Some(-3.0), 0.35, 0.65, 1, 0.005);
        st.lock().unwrap().close_speed_cap = cap_opt;
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: false, speed: planned_speed, timeout_s: Some(30.0), tag: None },
            ep,
        )) } }.unwrap();
        let mut hit_stop_tick: Option<usize> = None;
        let mut done_tick: Option<usize> = None;
        let mut done = None;
        for tick in 0..400 {
            sup.tick(0.02);
            if hit_stop_tick.is_none() && (st.lock().unwrap().q_phys - (-3.0)).abs() < 1e-4 {
                hit_stop_tick = Some(tick);
            }
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() {
                done_tick = Some(tick);
                done = Some(a);
                break;
            }
        }
        let a = done.unwrap_or_else(|| panic!("{label}: oscillating jam did not fail within 8 s"));
        assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "failed", Some("GRIPPER_TIMEOUT")), "{label}: {a:?}");
        assert!(sup.gripper_stalled && sup.gripper_stall_sign > 0.0, "{label}: stall latch not set");
        let detect_s = (done_tick.unwrap() - hit_stop_tick.expect("never reached jam stop")) as f64 * 0.02;
        assert!(detect_s <= 0.50, "{label}: oscillating jam took {detect_s:.2} s (> 0.50 s bound)");
    }

    // 3c. Empty-jaw close (both grasp=false and grasp=true) with a sustained 0.75 N·m torque bump
    //     while moving (above old 0.60 N·m contact threshold, below 0.80 N·m measured free-motion
    //     peak) completes cleanly all the way to 0.0 rad without premature contact or stall.
    for grasp in [false, true] {
        let (mut sup, st) = make_sup(-2.0, 0.29, None, 0.75, 0.75, 1, 0.0);
        st.lock().unwrap().close_speed_cap = Some(0.30);
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp, speed: 1.0, timeout_s: None, tag: None },
            ep,
        )) } }.unwrap();
        let mut done = None;
        for _ in 0..800 {
            sup.tick(0.02);
            assert!(!sup.gripper_stalled, "grasp={grasp}: 0.75 N·m moving bump falsely latched stall at {}", sup.measured_positions[J7]);
            if sup.measured_positions[J7] < -0.20 {
                if let Some(active) = &sup.active_trajectory {
                    if let armd::motion::Reference::Gripper(gm) = &active.reference {
                        assert!(gm.contact_rad.is_none(), "grasp={grasp}: 0.75 N·m moving bump falsely triggered contact at {}", sup.measured_positions[J7]);
                    }
                }
            }
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() { done = Some(a); break; }
        }
        let a = done.unwrap_or_else(|| panic!("grasp={grasp}: empty-jaw close with 0.75 N·m bump did not finish"));
        assert_eq!(a.id, id);
        assert_eq!(a.state, "done", "grasp={grasp}: {a:?}");
        if grasp {
            assert_eq!(a.holding, Some(false), "empty-jaw grasp falsely reported holding=true: {a:?}");
        }
        assert!(sup.measured_positions[J7].abs() < 0.05, "grasp={grasp}: ended short at {}", sup.measured_positions[J7]);
    }

    // 3d. Grasp hitting a stopped object with 0.65 N·m (>= 0.60 N·m contact_torque_nm, below 0.82 N·m
    //     moving floor) still detects contact once net progress stops and finishes holding=true.
    {
        let (mut sup, st) = make_sup(-2.5, 0.29, Some(-2.0), 0.35, 0.65, 1, 0.0);
        st.lock().unwrap().close_speed_cap = Some(0.30);
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 1.0, timeout_s: None, tag: None },
            ep,
        )) } }.unwrap();
        let mut done = None;
        for _ in 0..600 {
            sup.tick(0.02);
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() { done = Some(a); break; }
        }
        let a = done.expect("0.65 N·m stopped grasp did not finish");
        assert_eq!((a.id, a.state.as_str(), a.holding), (id, "done", Some(true)), "{a:?}");
        assert!((sup.measured_positions[J7] - (-2.0)).abs() < 0.05, "ended at {}", sup.measured_positions[J7]);
    }

    // 4. Realistic lag (>= 0.30 rad) when planned_speed = 0.29 rad/s (close_speed_rad_s = 0.299)
    //    while the physical jaw moves at 0.06 rad/s:
    //    4a. Non-grasp move(0.0) with near-ceiling torque 0.78 N·m (above 0.444 and 0.60, below 1.0 cap)
    //        completes cleanly as `done` without false stall latch.
    {
        let (mut sup, st) = make_sup(-2.0, 0.06, None, 0.78, 0.78, 1, 0.0);
        st.lock().unwrap().close_speed_cap = Some(0.299);
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Move, fraction: Some(0.0), grasp: false, speed: 1.0, timeout_s: Some(45.0), tag: None },
            ep,
        )) } }.unwrap();
        let mut saw_lag_ge_030 = false;
        let mut done = None;
        for _ in 0..2200 {
            sup.tick(0.02);
            if sup.commanded_positions[J7] - sup.measured_positions[J7] >= 0.30 {
                saw_lag_ge_030 = true;
            }
            assert!(!sup.gripper_stalled, "lagging non-grasp move at 0.78 N·m falsely stalled at q={}", sup.measured_positions[J7]);
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() { done = Some(a); break; }
        }
        let a = done.expect("lagging non-grasp move at 0.78 N·m did not finish");
        assert!(saw_lag_ge_030, "expected >= 0.30 rad following lag during travel");
        assert_eq!((a.id, a.state.as_str()), (id, "done"), "{a:?}");
        assert!(sup.measured_positions[J7].abs() < 0.05);
    }

    //    4b. Grasp close (grasp: true) at planned_speed = 0.29 rad/s while physical jaw moves at
    //        0.06 rad/s with lag >= 0.30 rad and torque 0.55 N·m (above 0.444, below 0.60):
    //        does NOT falsely trip lag_hit, stall, or stillness while advancing, reaches object at -2.0,
    //        and finishes `done` with `holding == Some(true)`.
    {
        let (mut sup, st) = make_sup(-2.6, 0.06, Some(-2.0), 0.55, 0.55, 1, 0.0);
        st.lock().unwrap().close_speed_cap = Some(0.299);
        let ep = sup.safety.epoch_id();
        let (id, _) = { let s = sup.current_admission_stamp(); { let __s = s; sup.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::start_gripper(__a,
            GripperRequest { command: GripperCommand::Close, fraction: None, grasp: true, speed: 1.0, timeout_s: None, tag: None },
            ep,
        )) } }.unwrap();
        match &sup.active_trajectory.as_ref().unwrap().reference {
            armd::motion::Reference::Gripper(gm) => assert_eq!(gm.planned_speed, 0.29),
            other => panic!("expected Gripper reference, got {other:?}"),
        }
        let mut saw_lag_ge_030 = false;
        let mut done = None;
        for _ in 0..1000 {
            sup.tick(0.02);
            if sup.measured_positions[J7] < -2.02 {
                if sup.commanded_positions[J7] - sup.measured_positions[J7] >= 0.30 {
                    saw_lag_ge_030 = true;
                }
                assert!(!sup.gripper_stalled, "lagging grasp falsely stalled before -2.0 at {}", sup.measured_positions[J7]);
                if let Some(active) = &sup.active_trajectory {
                    if let armd::motion::Reference::Gripper(gm) = &active.reference {
                        assert!(gm.contact_rad.is_none(), "lagging grasp falsely tripped contact at {}", sup.measured_positions[J7]);
                    }
                }
            }
            let a = sup.last_action.clone().unwrap();
            if !a.is_running() { done = Some(a); break; }
        }
        let a = done.expect("lagging grasp did not finish");
        assert!(saw_lag_ge_030, "expected >= 0.30 rad following lag during grasp travel");
        assert_eq!((a.id, a.state.as_str(), a.holding), (id, "done", Some(true)), "{a:?}");
        assert!((sup.measured_positions[J7] - (-2.0)).abs() < 0.05, "ended at {}", sup.measured_positions[J7]);
    }
}

/// Post-merge Item 2: J7 refresh must never declare Ready on a stationary reply while newer
/// J7 motion or fault replies are still queued beyond the single-pass RX budget.
///
/// Covers both `RefreshBeforeRestore` (`already_mode_4 = false`) and `PreEnableRefresh`
/// (`already_mode_4 = true`):
/// 1. Stationary J7 reply followed by 63 filler frames (exhausting the 64-frame RX pass) and a
///    queued J7 fault (`0x08`) or moving (`v = 0.5`) reply -> first poll stays `Pending` (NOT
///    Ready, zero mode-write / 0x307 hold / CMD_ENABLE); second poll drains the queued reply
///    and fails closed reporting the fault / motion.
/// 2. Budgeted poll (`max_rx_frames = 4`): stationary J7 reply + 2 filler + J7 fault `0x08`
///    followed by a trailing stationary J7 reply -> vetoes immediately with fault `status 8`
///    (trailing benign reply cannot mask the intervening fault).
/// 3. Stationary J7 reply followed by an un-drained backlog past the 50 ms deadline -> stays
///    `Pending` before the deadline and fails closed at the deadline.
#[test]
fn enable_refresh_waits_for_empty_queue_and_reports_queued_fault_or_motion_beyond_rx_budget() {
    let is_j7_refresh = |f: &CanFrame| f.id == 0x7FF && f.data[0] == 7 && f.data[1] == 0 && f.data[2] == 0xCC;

    let drive_to_active_refresh = |already_mode_4: bool| {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = 0.1 * i as f32;
            }
            s.motors[J7].q = -1.0;
            if already_mode_4 {
                s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
            }
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits())
            .unwrap()
            .with_gripper_pos_force(pf())
            .unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        // Age J7 feedback past 100 ms without polling so enable enters `RefreshBeforeRestore`
        // (when !already_mode_4) or `PreEnableRefresh` (after readback, when already_mode_4).
        clock.advance(Duration::from_millis(120));
        bus.with(|s| s.motors[J7].mute_feedback = true);
        bus.clear_log();
        let mut p = d.begin_torque(true).unwrap();
        let wake = loop {
            match p {
                armd::driver::TorqueProgress::Pending { wake }
                    if bus.tx_log().iter().any(is_j7_refresh) && wake > clock.now() => break wake,
                armd::driver::TorqueProgress::Pending { .. } => {
                    clock.advance(Duration::from_micros(500));
                    p = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
                }
                armd::driver::TorqueProgress::Done(_) => panic!("finished before refresh"),
            }
        };
        bus.clear_log();
        (d, bus, clock, wake)
    };

    // 1. Stationary reply followed by 63 filler frames + queued fault (0x08) or moving (v=0.5)
    //    reply beyond the 64-frame RX budget -> poll 1 stays Pending, poll 2 reports fault/motion.
    for already_mode_4 in [false, true] {
        for (label, v, st_ovr, expect_substr) in [
            ("fault_0x08", 0.0_f32, Some(0x08u8), "status 8"),
            ("moving", 0.5_f32, None, "still false"),
        ] {
            let (mut d, bus, clock, _wake) = drive_to_active_refresh(already_mode_4);
            clock.advance(Duration::from_millis(2));
            bus.with(|s| {
                s.motors[J7].mute_feedback = false;
                s.motors[J7].v = 0.0;
                s.motors[J7].state_override = None;
                let safe_j7 = s.motors[J7].feedback_frame();
                let filler = s.motors[0].feedback_frame();
                s.motors[J7].v = v;
                s.motors[J7].state_override = st_ovr;
                let bad_j7 = s.motors[J7].feedback_frame();
                s.rx.push_back(safe_j7);
                for _ in 0..63 {
                    s.rx.push_back(filler);
                }
                // Frame 65 (beyond the 64-frame RX pass budget): newer J7 fault / motion!
                s.rx.push_back(bad_j7);
            });

            // Poll 1: reads frames 1..=64 (including safe_j7), does NOT observe empty queue ->
            // MUST remain Pending and emit NO mode-write, NO 0x307 hold, NO CMD_ENABLE!
            let p1 = d
                .poll_torque(clock.now() + Duration::from_millis(5))
                .unwrap_or_else(|e| panic!("already_mode_4={already_mode_4} {label}: unexpected error on poll 1: {e}"));
            assert!(
                matches!(p1, armd::driver::TorqueProgress::Pending { .. }),
                "already_mode_4={already_mode_4} {label}: must stay Pending while queue is not drained, got {p1:?}"
            );
            let log1 = bus.tx_log();
            assert!(
                !log1.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
                "already_mode_4={already_mode_4} {label}: wrote CTRL_MODE before queue drained: {log1:?}"
            );
            assert!(
                j7_commands(&log1).is_empty(),
                "already_mode_4={already_mode_4} {label}: sent 0x307 hold before queue drained: {log1:?}"
            );
            assert!(
                !log1.iter().any(|f| f.data == damiao::CMD_ENABLE),
                "already_mode_4={already_mode_4} {label}: sent CMD_ENABLE before queue drained: {log1:?}"
            );

            // Poll 2: dequeues frame 65 (fault / motion) and hits empty queue -> MUST fail closed
            // and report the fault / motion!
            clock.advance(Duration::from_millis(1));
            let err = d
                .poll_torque(clock.now() + Duration::from_millis(5))
                .expect_err(&format!("already_mode_4={already_mode_4} {label}: queued unsafe J7 reply must veto"))
                .to_string();
            assert!(
                err.contains(expect_substr),
                "already_mode_4={already_mode_4} {label}: expected '{expect_substr}' in error, got: {err}"
            );
            let log2 = bus.tx_log();
            assert!(!log2.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE));
            assert!(j7_commands(&log2).is_empty());
            assert!(!log2.iter().any(|f| f.data == damiao::CMD_ENABLE));
            assert!(!d.is_enabled());
        }
    }

    // 2. Intervening fault across a small budgeted poll (max_rx_frames = 4) cannot be masked by
    //    a trailing benign stationary J7 frame queued behind it.
    for already_mode_4 in [false, true] {
        let (mut d, bus, clock, _wake) = drive_to_active_refresh(already_mode_4);
        clock.advance(Duration::from_millis(2));
        bus.with(|s| {
            s.motors[J7].mute_feedback = false;
            s.motors[J7].v = 0.0;
            s.motors[J7].state_override = None;
            let safe_j7 = s.motors[J7].feedback_frame();
            let filler = s.motors[0].feedback_frame();
            s.motors[J7].state_override = Some(0x08);
            let fault_j7 = s.motors[J7].feedback_frame();
            s.motors[J7].state_override = None;
            let trailing_safe_j7 = s.motors[J7].feedback_frame();
            // Poll 1 (budget 4) reads: safe_j7, filler, filler, fault_j7; trailing_safe_j7 stays queued.
            s.rx.push_back(safe_j7);
            s.rx.push_back(filler);
            s.rx.push_back(filler);
            s.rx.push_back(fault_j7);
            s.rx.push_back(trailing_safe_j7);
        });
        let err = d
            .poll_torque_budgeted(clock.now() + Duration::from_millis(5), 4)
            .expect_err(&format!("already_mode_4={already_mode_4}: intervening fault must veto immediately"))
            .to_string();
        assert!(err.contains("status 8"), "already_mode_4={already_mode_4}: expected fault status 8, got: {err}");
        let log = bus.tx_log();
        assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE));
        assert!(j7_commands(&log).is_empty());
        assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE));
        assert!(!d.is_enabled());
    }

    // 3. Stationary reply followed by an un-drained backlog that never reaches empty before the
    //    50 ms deadline -> stays Pending before deadline, then fails closed at the deadline.
    for already_mode_4 in [false, true] {
        let (mut d, bus, clock, wake) = drive_to_active_refresh(already_mode_4);
        clock.advance(Duration::from_millis(2));
        let filler = bus.with(|s| {
            s.motors[J7].mute_feedback = false;
            let safe_j7 = s.motors[J7].feedback_frame();
            let filler = s.motors[0].feedback_frame();
            s.rx.push_back(safe_j7);
            for _ in 0..150 {
                s.rx.push_back(filler);
            }
            filler
        });
        let p1 = d.poll_torque(clock.now() + Duration::from_millis(5)).unwrap();
        assert!(matches!(p1, armd::driver::TorqueProgress::Pending { .. }));
        // Advance to deadline while keeping the RX queue flooded beyond 64 frames.
        bus.with(|s| {
            for _ in 0..100 {
                s.rx.push_back(filler);
            }
        });
        clock.advance(wake.saturating_duration_since(clock.now()));
        let err = d
            .poll_torque(clock.now() + Duration::from_millis(5))
            .expect_err(&format!("already_mode_4={already_mode_4}: undrained refresh at deadline must fail closed"))
            .to_string();
        assert!(!err.is_empty());
        let log = bus.tx_log();
        assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE));
        assert!(j7_commands(&log).is_empty());
        assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE));
        assert!(!d.is_enabled());
    }
}

/// Major 1 regression: driving the driver clock across 100 ms specifically between the entry
/// freshness gate (`check_pos_force_freshness_for_tx`) and staging (`pending_target_cmds_checked`)
/// MUST NOT abort the arm batch with `PosForceTargetUnavailable` in EITHER motion
/// (`write_joint_targets_bounded`) or stopped-hold (`write_estop_hold_bounded`); it must skip
/// only J7, send healthy J1–J6 frames, request a J7 `0xCC` refresh, and report `Stale`.
#[test]
fn pos_force_crossing_100ms_between_entry_and_staging_skips_j7_in_motion_and_hold() {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::time::Instant;
    use armd::driver::{PosForceTxStatus, TxBatchOutcome};
    use armd::transport::{CanTransport, DriverClock, TransportError};

    struct LeapOnNthNowClock {
        inner: MockClock,
        armed: AtomicBool,
        target_call: AtomicUsize,
        calls: AtomicUsize,
        jump: Duration,
    }
    impl DriverClock for LeapOnNthNowClock {
        fn clock_id(&self) -> u64 {
            self.inner.clock_id()
        }
        fn now(&self) -> Instant {
            if self.armed.load(Ordering::SeqCst) {
                let n = self.calls.fetch_add(1, Ordering::SeqCst) + 1;
                if n == self.target_call.load(Ordering::SeqCst) {
                    self.inner.advance(self.jump);
                }
            }
            self.inner.now()
        }
        fn sleep(&self, dur: Duration) {
            self.inner.sleep(dur);
        }
        fn yield_now(&self) {
            self.inner.yield_now();
        }
    }

    #[derive(Clone)]
    struct LeapBus {
        inner: FakeBus,
        clock: Arc<LeapOnNthNowClock>,
    }
    impl CanTransport for LeapBus {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            self.inner.receive_timestamped()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
        fn flush_is_noop(&self) -> bool {
            self.inner.flush_is_noop()
        }
        fn supports_nonblocking_rt(&self) -> bool {
            self.inner.supports_nonblocking_rt()
        }
        fn supports_bounded_flush(&self) -> bool {
            self.inner.supports_bounded_flush()
        }
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(self.clock.clone())
        }
    }

    for is_hold in [false, true] {
        // Sweep the clock jump across every `now()` call from entry through staging & drain so
        // crossing 100 ms between entry and staging (and mid-staging / mid-drain) is covered.
        for jump_on_call in 1..=6usize {
            let base_clock = MockClock::new();
            let raw_bus = FakeBus::with_clock(&b601_limits(), base_clock.clone());
            raw_bus.with(|s| {
                for (i, m) in s.motors.iter_mut().enumerate() {
                    m.q = 0.1 * i as f32;
                }
                s.motors[J7].q = -1.0;
                s.immediate_replies = true;
            });
            let leap_clock = Arc::new(LeapOnNthNowClock {
                inner: base_clock.clone(),
                armed: AtomicBool::new(false),
                target_call: AtomicUsize::new(jump_on_call),
                calls: AtomicUsize::new(0),
                jump: Duration::from_millis(5),
            });
            let bus = LeapBus { inner: raw_bus.clone(), clock: leap_clock.clone() };
            let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(b601_limits())
                .unwrap()
                .with_gripper_pos_force(pf())
                .unwrap();
            d.read_joint_states().unwrap();
            raw_bus.with(|s| s.push_all_feedback());
            d.read_joint_states().unwrap();
            d.check_mapping_ranges();
            d.set_torque(true).expect("enable");

            // Keep J1–J6 fresh (queue_bound <= 10 ms) while parking J7 at 98 ms (still <= 100 ms!).
            raw_bus.with(|s| {
                s.immediate_replies = false;
                s.motors[J7].mute_feedback = true;
            });
            base_clock.advance(Duration::from_millis(88));
            raw_bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            d.read_joint_states().unwrap();
            base_clock.advance(Duration::from_millis(10));
            raw_bus.with(|s| for i in 0..6 { s.push_feedback(i); });
            d.read_joint_states().unwrap();
            // Now J7 age = 98 ms (passes entry gate!), J1–J6 age = 0 ms (queue_bound = 10 ms).
            assert!(d.pos_force_feedback_fresh(), "J7 must start fresh at 98 ms");

            raw_bus.clear_log();
            d.begin_tx_tick();
            leap_clock.calls.store(0, Ordering::SeqCst);
            leap_clock.armed.store(true, Ordering::SeqCst);
            let ts = targets(-1.2);
            let res = if is_hold {
                d.write_estop_hold_bounded(&ts)
            } else {
                d.write_joint_targets_bounded(&ts)
            };
            leap_clock.armed.store(false, Ordering::SeqCst);
            assert!(
                leap_clock.calls.load(Ordering::SeqCst) >= jump_on_call,
                "is_hold={is_hold} jump_on_call={jump_on_call}: jump must have fired"
            );
            assert_eq!(
                res.unwrap_or_else(|e| panic!("is_hold={is_hold} jump={jump_on_call}: must skip J7 cleanly, not fail: {e:?}")),
                TxBatchOutcome::Complete,
                "is_hold={is_hold} jump={jump_on_call}: healthy arm batch must complete"
            );
            assert!(
                matches!(d.take_last_pos_force_tx_status(), PosForceTxStatus::Stale { joint: J7, .. }),
                "is_hold={is_hold} jump={jump_on_call}: expected PosForceTxStatus::Stale for J7"
            );
            let log = raw_bus.tx_log();
            let j7_cmds = j7_commands(&log);
            assert_eq!(
                j7_cmds.len(),
                1,
                "is_hold={is_hold} jump={jump_on_call}: expected 1 0x307 freeze frame: {log:?}"
            );
            assert_eq!(j7_cmds[0].id, 0x307);
            let fpos = f32::from_le_bytes(j7_cmds[0].data[0..4].try_into().unwrap());
            let fvel = u16::from_le_bytes([j7_cmds[0].data[4], j7_cmds[0].data[5]]);
            let fcur = u16::from_le_bytes([j7_cmds[0].data[6], j7_cmds[0].data[7]]);
            assert!(
                (fpos - (-1.0)).abs() < 1e-3,
                "is_hold={is_hold} jump={jump_on_call}: freeze pos={fpos} must hold last fresh -1.0 (not -1.2)"
            );
            assert_eq!(fvel, 0, "is_hold={is_hold} jump={jump_on_call}: freeze vel_raw must be 0");
            assert_eq!(fcur, RAW);
            assert!(
                log.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
                "is_hold={is_hold} jump={jump_on_call}: expected J7 0xCC refresh: {log:?}"
            );
            for id in 1u32..=6 {
                assert!(
                    log.iter().any(|f| f.id == 0x100 + id),
                    "is_hold={is_hold} jump={jump_on_call}: J{id} frame missing: {log:?}"
                );
            }
        }
    }
}

/// Blocker 2 driver unit regression: `pos_force_feedback_fresh()` resolves the configured J7
/// independently of `pos_force_verified` or active TX mode (disabled/unverified, unknown mode,
/// MIT mode) and enforces the inclusive `<= 100 ms` receive + queue-bound + finite-position check.
#[test]
fn pos_force_feedback_fresh_fail_closed_across_unverified_unknown_mode_and_100ms_boundary() {
    let (mut d, bus, clock) = rig(true);
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..6 {
        arm_joints[i] = i;
    }
    let mut rest_positions = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..6 {
        rest_positions[i] = 0.1 * i as f32;
    }
    rest_positions[J7] = -1.0;
    d.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions,
        rest_pose_tolerance_rad: 0.05,
    });
    bus.with(|s| s.immediate_replies = true);
    d.set_torque(true).expect("enable pos-force");

    // 1. Inclusive 100 ms boundary: exact 100 ms -> true; 101 ms -> false.
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    clock.advance(Duration::from_millis(100));
    assert!(d.pos_force_feedback_fresh(), "exact 100 ms age must be fresh (inclusive)");
    clock.advance(Duration::from_millis(1));
    assert!(!d.pos_force_feedback_fresh(), "101 ms age must be stale");

    // 2. Disabled / unverified mode (`set_torque(false)`): J7 aged 150 ms (still <= 250 ms
    //    `is_feedback_valid`) MUST still report `pos_force_feedback_fresh() == false`.
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(11));
        }
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
    }
    d.set_torque(false).expect("disable torque");
    bus.with(|s| s.motors[J7].mute_feedback = true);
    clock.advance(Duration::from_millis(140));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    d.read_joint_states().unwrap();
    clock.advance(Duration::from_millis(10));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    d.read_joint_states().unwrap();
    assert!(d.is_feedback_valid(), "J7 at 150 ms is still within 250 ms general feedback validity");
    assert!(
        !d.pos_force_feedback_fresh(),
        "disabled/unverified J7 at 150 ms must NOT bypass 100 ms freshness check"
    );

    // 3. Unknown J7 mode (`joint_modes[6] = None` after dropped CTRL_MODE write reply):
    //    J7 aged 150 ms MUST still report `pos_force_feedback_fresh() == false`.
    bus.with(|s| {
        s.motors[J7].mute_feedback = false;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    d.set_torque(true).expect("re-enable");
    bus.with(|s| s.motors[J7].no_write_ack = true);
    let mut modes = vec![armd::driver::JointMode::Position; 7];
    modes[J7] = armd::driver::JointMode::Mit;
    let holds = targets(-1.0);
    let _ = d.set_joint_modes(&modes, &holds);
    bus.with(|s| {
        s.motors[J7].no_write_ack = false;
        s.motors[J7].mute_feedback = true;
    });
    assert!(d.joint_modes_uncertain(), "J7 mode must now be unknown");
    clock.advance(Duration::from_millis(140));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    d.read_joint_states().unwrap();
    clock.advance(Duration::from_millis(10));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    d.read_joint_states().unwrap();
    assert!(d.is_feedback_valid());
    assert!(
        !d.pos_force_feedback_fresh(),
        "unknown-mode J7 at 150 ms must NOT bypass 100 ms freshness check"
    );
    // Fresh J7 reply (<= 100 ms) restores freshness even while mode is unknown.
    bus.with(|s| {
        s.motors[J7].mute_feedback = false;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();
    assert!(d.pos_force_feedback_fresh(), "fresh <= 100 ms J7 must satisfy pos_force_feedback_fresh");
}

/// R2 Blocker 1 regression:
/// - A J7-only carried-over frame parked at exact 100 ms whose FIFO age also crosses `frame_max_age`
///   (45 ms) MUST NOT be counted as FIFO `Expired` (`CAN_TX_STALLED`) even when the clock crosses
///   100 ms on any `now()` call during preflight, mid-drain, or post-flush cleanup (shared single
///   clock sample at each cleanup boundary).
/// - Negative control: a carried-over J1–J6 frame aged past 45 ms STILL returns `TxBatchOutcome::Expired`.
#[test]
fn pos_force_carryover_j7_at_100ms_boundary_never_counted_as_fifo_expired() {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::time::Instant;
    use armd::driver::{PosForceTxStatus, TxBatchOutcome, TxBudget};
    use armd::transport::{CanTransport, DriverClock, TransportError};

    struct LeapOnNthNowClock {
        inner: MockClock,
        armed: AtomicBool,
        target_call: AtomicUsize,
        calls: AtomicUsize,
        jump: Duration,
    }
    impl DriverClock for LeapOnNthNowClock {
        fn clock_id(&self) -> u64 {
            self.inner.clock_id()
        }
        fn now(&self) -> Instant {
            if self.armed.load(Ordering::SeqCst) {
                let n = self.calls.fetch_add(1, Ordering::SeqCst) + 1;
                if n == self.target_call.load(Ordering::SeqCst) {
                    self.inner.advance(self.jump);
                }
            }
            self.inner.now()
        }
        fn sleep(&self, dur: Duration) {
            self.inner.sleep(dur);
        }
        fn yield_now(&self) {
            self.inner.yield_now();
        }
    }

    #[derive(Clone)]
    struct LeapBus {
        inner: FakeBus,
        clock: Arc<LeapOnNthNowClock>,
    }
    impl CanTransport for LeapBus {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            self.inner.receive_timestamped()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
        fn flush_is_noop(&self) -> bool {
            self.inner.flush_is_noop()
        }
        fn supports_nonblocking_rt(&self) -> bool {
            self.inner.supports_nonblocking_rt()
        }
        fn supports_bounded_flush(&self) -> bool {
            self.inner.supports_bounded_flush()
        }
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(self.clock.clone())
        }
    }

    // 1. J7-only carry-over parked at exact 100 ms (and queued_age = 44 ms), jumping +2 ms across
    //    both 100 ms freshness and 45 ms FIFO expiry on EVERY `now()` call (1..=8): MUST NEVER
    //    return `TxBatchOutcome::Expired`!
    for jump_on_call in 1..=8usize {
        let base_clock = MockClock::new();
        let raw_bus = FakeBus::with_clock(&b601_limits(), base_clock.clone());
        raw_bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = 0.1 * i as f32;
            }
            s.motors[J7].q = -1.0;
            s.immediate_replies = true;
        });
        let leap_clock = Arc::new(LeapOnNthNowClock {
            inner: base_clock.clone(),
            armed: AtomicBool::new(false),
            target_call: AtomicUsize::new(jump_on_call),
            calls: AtomicUsize::new(0),
            jump: Duration::from_millis(2),
        });
        let bus = LeapBus { inner: raw_bus.clone(), clock: leap_clock.clone() };
        let b6 = TxBudget {
            max_frames: 6,
            max_duration: Duration::from_millis(10),
            frame_max_age: Duration::from_millis(45),
        };
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits())
            .unwrap()
            .with_gripper_pos_force(pf())
            .unwrap()
            .with_tx_budget(b6);
        d.read_joint_states().unwrap();
        raw_bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        d.set_torque(true).expect("enable");
        // Establish fresh empty-RX boundary + J1-J7 samples at t = 0 right after enable handshake.
        d.read_joint_states().unwrap();
        raw_bus.with(|s| {
            s.immediate_replies = false;
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();
        raw_bus.with(|s| s.motors[J7].mute_feedback = true);

        // Advance 56 ms (keeping J1-J6 fresh), then emit batch with budget=6 -> J1-J6 send, ONLY J7 deferred!
        base_clock.advance(Duration::from_millis(56));
        raw_bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        d.read_joint_states().unwrap();
        d.begin_tx_tick();
        assert_eq!(
            d.write_joint_targets_bounded(&targets(-1.1)).unwrap(),
            TxBatchOutcome::Deferred { pending: 1 }
        );
        // Advance 44 ms more -> J7 feedback age = exact 100 ms, carried-over J7 queue age = 44 ms (< 45 ms).
        base_clock.advance(Duration::from_millis(44));
        raw_bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        d.read_joint_states().unwrap();
        assert!(d.pos_force_feedback_fresh(), "J7 starts at exact 100 ms");

        raw_bus.clear_log();
        d.begin_tx_tick();
        leap_clock.calls.store(0, Ordering::SeqCst);
        leap_clock.armed.store(true, Ordering::SeqCst);
        let outcome = d
            .write_joint_targets_bounded(&targets(-1.2))
            .unwrap_or_else(|e| panic!("jump={jump_on_call}: unexpected error {e:?}"));
        leap_clock.armed.store(false, Ordering::SeqCst);
        assert!(
            !matches!(outcome, TxBatchOutcome::Expired { .. }),
            "jump={jump_on_call}: J7-only carry-over crossing 100 ms must NEVER return Expired, got {outcome:?}"
        );
        assert!(
            matches!(d.take_last_pos_force_tx_status(), PosForceTxStatus::Stale { joint: J7, .. }),
            "jump={jump_on_call}: expected Stale status for J7"
        );
        // Under `max_frames = 6`, budget reservation emits J7's freeze frame immediately (deferring
        // 1 arm joint); if J7 itself was deferred on the first pass, draining completes it cleanly.
        if matches!(outcome, TxBatchOutcome::Deferred { pending: 1 })
            && j7_commands(&raw_bus.tx_log()).is_empty()
        {
            d.begin_tx_tick();
            let drain = d.drain_pending_tx().unwrap();
            assert_eq!(drain, TxBatchOutcome::Complete, "jump={jump_on_call}: freeze frame must drain cleanly");
        }
        let j7_cmds = j7_commands(&raw_bus.tx_log());
        assert_eq!(
            j7_cmds.len(),
            1,
            "jump={jump_on_call}: expected 1 0x307 freeze frame instead of stale target: {j7_cmds:?}"
        );
        let fpos = f32::from_le_bytes(j7_cmds[0].data[0..4].try_into().unwrap());
        let fvel = u16::from_le_bytes([j7_cmds[0].data[4], j7_cmds[0].data[5]]);
        let fcur = u16::from_le_bytes([j7_cmds[0].data[6], j7_cmds[0].data[7]]);
        assert!((fpos - (-1.0)).abs() < 1e-3, "jump={jump_on_call}: freeze pos={fpos} != -1.0");
        assert_eq!(fvel, 0, "jump={jump_on_call}: freeze vel_raw must be 0");
        assert_eq!(fcur, RAW);
    }

    // 1b. True J7 carry-over at index 0 (no intermediate recovery write wiping `pending_tx`),
    //     parked at exact 100 ms feedback age and 44 ms queue age, jumping +2 ms on EVERY `now()`
    //     call (1..=8): converts in-place to `PosForceFreeze` (NEVER `Expired`) and emits the
    //     exact 0x307 freeze frame on that very tick!
    for jump_on_call in 1..=8usize {
        let base_clock = MockClock::new();
        let raw_bus = FakeBus::with_clock(&b601_limits(), base_clock.clone());
        raw_bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = 0.1 * i as f32;
            }
            s.motors[J7].q = -1.0;
            s.immediate_replies = true;
        });
        let leap_clock = Arc::new(LeapOnNthNowClock {
            inner: base_clock.clone(),
            armed: AtomicBool::new(false),
            target_call: AtomicUsize::new(jump_on_call),
            calls: AtomicUsize::new(0),
            jump: Duration::from_millis(2),
        });
        let bus = LeapBus { inner: raw_bus.clone(), clock: leap_clock.clone() };
        let b6 = TxBudget {
            max_frames: 6,
            max_duration: Duration::from_millis(10),
            frame_max_age: Duration::from_millis(45),
        };
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits())
            .unwrap()
            .with_gripper_pos_force(pf())
            .unwrap()
            .with_tx_budget(b6);
        d.read_joint_states().unwrap();
        raw_bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        d.set_torque(true).expect("enable");
        d.read_joint_states().unwrap();
        raw_bus.with(|s| {
            s.immediate_replies = false;
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();
        raw_bus.with(|s| s.motors[J7].mute_feedback = true);

        base_clock.advance(Duration::from_millis(56));
        raw_bus.with(|s| for i in 0..6 { s.push_feedback(i); });
        d.read_joint_states().unwrap();
        d.begin_tx_tick();
        assert_eq!(
            d.write_joint_targets_bounded(&targets(-1.1)).unwrap(),
            TxBatchOutcome::Deferred { pending: 1 }
        );
        // Advance 44 ms WITHOUT `read_joint_states()` so carried-over J7 stays at index 0
        // (`queued_age = 44 ms`, `j7_age = 100 ms`).
        base_clock.advance(Duration::from_millis(44));
        assert!(d.pos_force_feedback_fresh(), "carryover jump={jump_on_call}: J7 starts at exact 100 ms");

        raw_bus.clear_log();
        d.begin_tx_tick();
        leap_clock.calls.store(0, Ordering::SeqCst);
        leap_clock.armed.store(true, Ordering::SeqCst);
        let outcome = d
            .write_joint_targets_bounded(&targets(-1.2))
            .unwrap_or_else(|e| panic!("carryover jump={jump_on_call}: unexpected error {e:?}"));
        leap_clock.armed.store(false, Ordering::SeqCst);
        assert!(
            !matches!(outcome, TxBatchOutcome::Expired { .. }),
            "carryover jump={jump_on_call}: J7 carry-over crossing 100 ms + 45 ms must NEVER return Expired, got {outcome:?}"
        );
        let j7_cmds = j7_commands(&raw_bus.tx_log());
        assert_eq!(
            j7_cmds.len(),
            1,
            "carryover jump={jump_on_call}: expected immediate 0x307 freeze frame at index 0: {j7_cmds:?}"
        );
        let fpos = f32::from_le_bytes(j7_cmds[0].data[0..4].try_into().unwrap());
        let fvel = u16::from_le_bytes([j7_cmds[0].data[4], j7_cmds[0].data[5]]);
        let fcur = u16::from_le_bytes([j7_cmds[0].data[6], j7_cmds[0].data[7]]);
        assert!((fpos - (-1.0)).abs() < 1e-3, "carryover jump={jump_on_call}: freeze pos={fpos} != -1.0");
        assert_eq!(fvel, 0, "carryover jump={jump_on_call}: freeze vel_raw must be 0");
        assert_eq!(fcur, RAW);
        assert!(
            matches!(d.take_last_pos_force_tx_status(), PosForceTxStatus::Stale { joint: J7, .. }),
            "carryover jump={jump_on_call}: expected Stale status for J7"
        );
    }

    // 2. Negative control: an expired J1–J6 carry-over frame (> 45 ms) alongside stale J7 MUST
    //    still return `TxBatchOutcome::Expired { dropped: 1, pending: 0 }`.
    let (mut d, bus, clock) = rig(true);
    bus.with(|s| s.immediate_replies = true);
    d.set_torque(true).expect("enable");
    d = d.with_tx_budget(TxBudget {
        max_frames: 5,
        max_duration: Duration::from_millis(10),
        frame_max_age: Duration::from_millis(45),
    });
    bus.with(|s| {
        s.immediate_replies = false;
        s.motors[J7].mute_feedback = true;
    });
    d.begin_tx_tick();
    // Budget 5 -> sends J1-J5, defers J6 + J7.
    assert_eq!(
        d.write_joint_targets_bounded(&targets(-1.1)).unwrap(),
        TxBatchOutcome::Deferred { pending: 2 }
    );
    clock.advance(Duration::from_millis(105));
    bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    d.read_joint_states().unwrap();
    d.begin_tx_tick();
    assert_eq!(
        d.write_joint_targets_bounded(&targets(-1.2)).unwrap(),
        TxBatchOutcome::Expired { dropped: 1, pending: 0 },
        "stale J7 is dropped by freshness gate, while expired J6 MUST still return Expired"
    );
}

/// I16 Item 2: Calibration close at raw 29 must be quantized ONCE and sent on CAN 0x307 as
/// raw 29 (never double-floored raw 29 -> 0.29_f64 -> raw 28).
#[tokio::test]
async fn calibration_close_at_raw_29_is_sent_as_raw_29() {
    use std::sync::Arc;
    use armd::transport::{CanTransport, DriverClock, TransportError};

    const DT: f64 = 0.02;

    #[derive(Clone)]
    struct RateLimitedBus {
        inner: FakeBus,
        dt: f32,
    }

    impl CanTransport for RateLimitedBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let prev_q = self.inner.with(|s| s.motors[J7].q);
            self.inner.transmit(frame)?;
            if frame.id == 0x307 || frame.id == 0x107 {
                self.inner.with(|s| {
                    let m = &mut s.motors[J7];
                    if !m.enabled { return; }
                    let target_p = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
                    let wire_v = if frame.id == 0x307 {
                        f32::from(u16::from_le_bytes([frame.data[4], frame.data[5]])) * 0.01
                    } else {
                        f32::from_le_bytes([frame.data[4], frame.data[5], frame.data[6], frame.data[7]]).abs()
                    };
                    let max_step = wire_v * self.dt;
                    let delta = (target_p - prev_q).clamp(-max_step, max_step);
                    let next_q = prev_q + delta;
                    m.q = next_q;
                    m.v = (next_q - prev_q) / self.dt;
                    let updated = m.feedback_frame();
                    if let Some(last) = s.pending.back_mut().filter(|f| f.id == updated.id) {
                        *last = updated;
                    } else if let Some(last) = s.rx.back_mut().filter(|f| f.id == updated.id) {
                        *last = updated;
                    }
                });
            }
            Ok(())
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> { self.inner.receive_timestamped() }
        fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
    }

    // Check both speed-limiting sources that quantize to raw 29:
    // (a) damiao_gripper.close_speed_rad_s = 0.299 (policy 0.30)
    // (b) gripper_policy.close_speed_rad_s = 0.295 (damiao 0.30)
    // (c) wire_speed_f32(29) exact boundary
    for (label, damiao_speed, policy_speed) in [
        ("damiao_0.299", 0.299_f32, 0.30_f32),
        ("policy_0.295", 0.30_f32, 0.295_f32),
        ("wire_29", DamiaoGripperDrive::wire_speed_f32(29), 0.30_f32),
    ] {
        assert_eq!(DamiaoGripperDrive::quantize_vel_raw(damiao_speed.min(policy_speed)), 29, "{label}");
        // Demonstrate the raw 29 -> 0.29_f64 -> 28 IEEE-754 floor trap that used to occur.
        assert_eq!(DamiaoGripperDrive::quantize_vel_raw_f64(29.0 / 100.0), 28);

        let clock = MockClock::new();
        let inner_bus = FakeBus::with_clock(&b601_limits(), clock.clone());
        inner_bus.with(|s| {
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = if i == J7 { -5.4 } else { 0.0 };
            }
        });
        let bus = RateLimitedBus { inner: inner_bus.clone(), dt: DT as f32 };
        let d = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits()).unwrap()
            .with_gripper_pos_force(GripperPosForce { close_speed_rad_s: damiao_speed, ..pf() }).unwrap();

        let mut cfg = ArmConfig::rebot_dm();
        cfg.gripper_policy.as_mut().unwrap().close_speed_rad_s = Some(policy_speed);
        cfg.damiao_gripper = Some(DamiaoGripperDrive {
            close_speed_rad_s: damiao_speed,
            ..DamiaoGripperDrive::rebot_dm()
        });
        let sup = Arc::new(armd::ArmSupervisor::new(cfg));
        sup.set_driver(Box::new(d)).await;
        {
            let mut inner = sup.inner().write().await;
            inner.lease_enabled = false;
            inner.follower = Some("damiao".into());
            inner.tick(DT);
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            armd::SupervisorInner::enable_torque(admitted).unwrap();
            clock.advance(Duration::from_millis(20));
            inner.tick(DT);
            assert!((inner.effective_planned_close_speed() - 0.29).abs() < 1e-12, "{label}");
        }

        let (cal_id, _) = armd::gripper_calib::start_calibration(&sup, sup.capture_admission_stamp()).await.unwrap();
        // Wait for the calibration orchestrator to install stroke 1 (closing to 5 % open).
        let mut installed_raw = None;
        for _ in 0..100 {
            {
                let inner = sup.inner().read().await;
                if let Some(armd::motion::Reference::Gripper(gm)) = inner.active_trajectory.as_ref().map(|a| &a.reference) {
                    installed_raw = gm.planned_vel_raw;
                    assert_eq!(gm.planned_speed, 0.29, "{label}: planned_speed");
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert_eq!(installed_raw, Some(29), "{label}: calibration close stroke double-quantized");

        inner_bus.clear_log();
        let mut max_vel_raw = 0u16;
        let mut saw_29 = false;
        let mut frames_checked = 0usize;
        // Step through acceleration + cruise of stroke 1 (~1.5 s = 75 ticks).
        for _ in 0..80 {
            inner_bus.clear_log();
            clock.advance(Duration::from_millis(20));
            {
                let mut inner = sup.inner().write().await;
                inner.tick(DT);
            }
            for f in j7_commands(&inner_bus.tx_log()) {
                assert_eq!(f.id, 0x307, "{label}");
                let vel_raw = u16::from_le_bytes([f.data[4], f.data[5]]);
                assert!((1..=29).contains(&vel_raw), "{label}: vel_raw={vel_raw} out of 1..=29");
                max_vel_raw = max_vel_raw.max(vel_raw);
                if vel_raw == 29 { saw_29 = true; }
                frames_checked += 1;
            }
        }
        assert!(frames_checked > 0, "{label}: no 0x307 frames emitted");
        assert!(saw_29 && max_vel_raw == 29, "{label}: expected cruise vel_raw 29, got max {max_vel_raw}");

        // Cleanly cancel the calibration orchestrator before next case.
        {
            let mut inner = sup.inner().write().await;
            assert_eq!(inner.macro_run.as_ref().map(|m| m.id), Some(cal_id));
            inner.request_macro_stop();
        }
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
}

/// Round 5 M1 + Round 8 D1 regression:
/// 1. A J7 feedback frame that sat 150 ms in the RX queue (both WITH a transport `rx_timestamp`
///    near `queue_bound` and WITHOUT `rx_timestamp`) MUST fail the exact same live freshness
///    predicate at decode time and NEVER overwrite `pf_last_fresh_position` / freeze target.
/// 2. With no provably fresh J7 sample ever decoded since boot (`pf_last_fresh_position == None`),
///    `POS_FORCE_BOOT_FALLBACK` is `NoOutputUntilFresh`: no `0x307` frame is sent before the first
///    fresh sample, and enable fails closed.
#[test]
fn pos_force_freeze_cache_rejects_150ms_queued_sample_and_boot_fallback_no_output_until_fresh() {
    use armd::driver::{PosForceBootFallback, POS_FORCE_BOOT_FALLBACK};
    use armd::transport::{CanTransport, TransportError};
    use std::sync::{Arc, Mutex};
    assert_eq!(POS_FORCE_BOOT_FALLBACK, PosForceBootFallback::NoOutputUntilFresh);

    #[derive(Clone)]
    struct TimestampOverrideBus {
        inner: FakeBus,
        next_j7_ts: Arc<Mutex<Option<std::time::Instant>>>,
    }
    impl CanTransport for TimestampOverrideBus {
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            self.inner.transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
            let (f, inner_ts) = self.inner.receive_timestamped()?;
            let ts = if f.id == 0x17 {
                self.next_j7_ts.lock().unwrap().take().or(inner_ts)
            } else {
                inner_ts
            };
            Ok((f, ts))
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.inner.flush()
        }
        fn flush_is_noop(&self) -> bool {
            self.inner.flush_is_noop()
        }
        fn supports_nonblocking_rt(&self) -> bool {
            self.inner.supports_nonblocking_rt()
        }
        fn supports_bounded_flush(&self) -> bool {
            self.inner.supports_bounded_flush()
        }
        fn clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
            self.inner.clock()
        }
    }

    // --- (1) Frame sitting 150 ms in RX queue NEVER becomes the freeze target ---
    let clock = MockClock::new();
    let raw_bus = FakeBus::with_clock(&b601_limits(), clock.clone());
    raw_bus.with(|s| {
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = 0.1 * i as f32;
        }
        s.motors[J7].q = -2.0;
        s.immediate_replies = true;
    });
    let ts_slot = Arc::new(Mutex::new(None));
    let bus = TimestampOverrideBus { inner: raw_bus.clone(), next_j7_ts: ts_slot.clone() };
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits())
        .unwrap()
        .with_gripper_pos_force(pf())
        .unwrap();
    d.read_joint_states().unwrap();
    raw_bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    d.set_torque(true).expect("enable");
    // Fresh boundary at t = 0 with J7 at -2.0 rad.
    d.read_joint_states().unwrap();
    raw_bus.with(|s| {
        s.immediate_replies = false;
        s.rx.clear();
        s.pending.clear();
        s.motors[J7].q = -2.0;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();
    let fresh_pos = d.pos_force_last_fresh_position().expect("fresh -2.0 cached");
    assert!((fresh_pos - (-2.0)).abs() < 1e-2, "expected -2.0, got {fresh_pos}");

    // Case 1a: Frame with hardware `rx_timestamp` = t0 + 10 ms (only 10 ms after `queue_bound`)
    // that sat in the RX queue until `decode_now` = t0 + 160 ms (150 ms old when decoded!).
    let t_bound = clock.now();
    *ts_slot.lock().unwrap() = Some(t_bound + Duration::from_millis(10));
    raw_bus.with(|s| {
        s.rx.clear();
        s.pending.clear();
        s.motors[J7].q = -0.5;
        s.push_feedback(J7);
        s.motors[J7].mute_feedback = true;
    });
    clock.advance(Duration::from_millis(160));
    raw_bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    d.read_joint_states().unwrap();
    assert!(
        !d.pos_force_feedback_fresh(),
        "150 ms queued J7 sample must not be considered fresh"
    );
    let cached_after_ts = d.pos_force_last_fresh_position().expect("old fresh pos preserved");
    assert!(
        (cached_after_ts - fresh_pos).abs() < 1e-5,
        "150 ms queued frame (with rx_timestamp) poisoned freeze cache: {cached_after_ts} != {fresh_pos}"
    );
    raw_bus.clear_log();
    d.begin_tx_tick();
    let _ = d.write_joint_targets_bounded(&targets(0.0)).unwrap();
    let j7_tx = j7_commands(&raw_bus.tx_log());
    assert_eq!(j7_tx.len(), 1, "expected 1 freeze frame: {j7_tx:?}");
    let p_wire = f32::from_le_bytes(j7_tx[0].data[0..4].try_into().unwrap());
    let v_wire = u16::from_le_bytes([j7_tx[0].data[4], j7_tx[0].data[5]]);
    let i_wire = u16::from_le_bytes([j7_tx[0].data[6], j7_tx[0].data[7]]);
    assert!((p_wire - fresh_pos).abs() < 1e-5, "freeze frame used stale -0.5: {p_wire}");
    assert_eq!(v_wire, 0);
    assert_eq!(i_wire, RAW);

    // Case 1b: Frame without `rx_timestamp` (`None`) sitting 150 ms past `queue_bound`.
    raw_bus.with(|s| {
        s.motors[J7].mute_feedback = false;
        s.motors[J7].q = -0.25;
        s.push_feedback(J7);
        s.motors[J7].mute_feedback = true;
    });
    clock.advance(Duration::from_millis(150));
    raw_bus.with(|s| for i in 0..6 { s.push_feedback(i); });
    d.read_joint_states().unwrap();
    let cached_after_qb = d.pos_force_last_fresh_position().expect("old fresh pos still preserved");
    assert!(
        (cached_after_qb - fresh_pos).abs() < 1e-5,
        "150 ms queued frame (via queue_bound) poisoned freeze cache: {cached_after_qb} != {fresh_pos}"
    );

    // --- (2) Unbounded pre-drain sample since boot NEVER populates freeze cache, and enable fails closed ---
    let clock2 = MockClock::new();
    let bus2 = FakeBus::with_clock(&b601_limits(), clock2.clone());
    bus2.with(|s| {
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = 0.1 * i as f32;
        }
        s.motors[J7].q = -1.5;
        s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
        s.immediate_replies = true;
    });
    let mut d2 = DamiaoArmDriver::new(bus2.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits())
        .unwrap()
        .with_gripper_pos_force(pf())
        .unwrap();
    // Push J7 feedback BEFORE the first empty-queue drain (unbounded) -> never qualifies as fresh!
    bus2.with(|s| {
        s.push_all_feedback();
        s.motors[J7].mute_feedback = true;
    });
    d2.read_joint_states().unwrap();
    assert_eq!(
        d2.pos_force_last_fresh_position(),
        None,
        "unbounded pre-drain feedback must NOT populate pf_last_fresh_position"
    );
    assert!(
        d2.set_torque(true).is_err(),
        "enable must still fail closed when no provably fresh J7 sample has been observed since boot"
    );
    assert_eq!(d2.pos_force_last_fresh_position(), None);
}
