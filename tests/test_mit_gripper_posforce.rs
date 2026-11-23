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
    assert!(hold_vel <= damiao::pos_force_vel_raw(0.30), "hold velocity {hold_vel}");
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
    assert_eq!(hold_vel, 30, "pre-enable hold must quantize 0.306 downward to 30 (0.30 rad/s), got {hold_vel}");
    assert_eq!(hold_cur, RAW);
    let last_pf = bus.with(|s| s.motors[J7].last_pos_force).expect("motor received pre-enable hold");
    assert!((last_pf.0 - (-1.75)).abs() < 1e-3 && last_pf.1 == 30 && last_pf.2 == RAW, "{last_pf:?}");
    assert!(d.is_enabled() && d.motor_current_cap().unwrap().active);

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
    // supply the pre-enable hold position on restart either.
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock);
    bus.with(|s| {
        s.motors[J7].q = -1.75;
        s.motors[J7].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_FORCE);
        s.push_all_feedback();
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits()).unwrap()
        .with_gripper_pos_force(pf()).unwrap();
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    bus.clear_log();
    let err = d.set_torque(true).unwrap_err().to_string();
    assert!(err.contains("no fresh measured position"), "accepted unknown-age startup sample: {err}");
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

    // 3. While J7 feedback is stale (> 100 ms), write_joint_targets must refuse the batch
    //    before sending any frame (no changed target reaches J7 during the outage).
    bus.clear_log();
    let stale_err = d.write_joint_targets(&targets(-2.0)).unwrap_err().to_string();
    assert!(stale_err.contains("feedback stale"), "{stale_err}");
    assert!(bus.tx_log().is_empty(), "frames sent during J7 stale outage: {:?}", bus.tx_log());

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
    //    J7 is 105 ms old when its frame is built -> write_joint_targets returns Err and emits NO 0x307 frame.
    let (mut d, bus, clock) = rig(true);
    d.set_torque(true).unwrap();
    d.read_joint_states().unwrap();
    d.read_joint_states().unwrap();
    clock.advance(Duration::from_millis(99));
    bus.with(|s| s.tx_step = Duration::from_millis(1));
    bus.clear_log();
    let err = d.write_joint_targets(&targets(-2.0)).unwrap_err().to_string();
    assert!(err.contains("feedback stale"), "{err}");
    assert!(j7_commands(&bus.tx_log()).is_empty(), "emitted 0x307 after mid-batch expiry: {:?}", j7_commands(&bus.tx_log()));

    // 2. WouldBlock stall crossing 100 ms: single-joint pos-force driver at 99 ms tries once,
    //    stall advances clock to 101 ms -> retry re-checks freshness and fails closed without a 2nd attempt.
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
    let err = d.write_joint_targets(&[JointTarget::position(-2.0)]).unwrap_err().to_string();
    assert!(err.contains("feedback stale before transmit"), "{err}");
    assert_eq!(bus.tx_log().len(), 1, "retried transmit past freshness bound: {:?}", bus.tx_log());

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
        let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
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
        let shared_driver = Arc::new(inner.driver.take().unwrap());
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
        let drv = inner.driver.as_ref().unwrap();
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
