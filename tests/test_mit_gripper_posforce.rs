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
