//! Joint tracking Phase 2 (MIT impedance tracking on Damiao motors), disabled by default.
//!
//! Commissioning and qualification gate: the TIMEOUT commissioning writes RAM only (0x55) and
//! confirms the value with a separate read; the qualification record is bound to the profile,
//! joint zero and motor identity; every arm rechecks it against a fresh readback and refuses
//! MIT (the arm does not arm) when anything does not match.

mod mit_support;

use armd::driver::{ArmDriver, DamiaoArmDriver};
use armd::mit::{self, MitJointQualification, MitQualificationFile};
use armd::model::ArmConfig;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::tracking::{JointTrackingConfig, TrackingStrategy};
use armd::{SupervisorInner, TrajectoryPlanner};
use mit_support::{assert_no_flash_write, FakeBus, SW_VER};
use serde_json::json;

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect()
}

fn driver_on(bus: &FakeBus) -> DamiaoArmDriver<FakeBus> {
    DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(b601_limits()).unwrap()
}

/// Disabled motors with fresh feedback (state 0), as after power-up and a range check.
fn disabled_driver(bus: &FakeBus) -> DamiaoArmDriver<FakeBus> {
    let mut d = driver_on(bus);
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d
}

/// MIT on J4 (arm slot 3) with gains under the B601 hand-guide ceilings (J4 15 / 0.5).
pub fn mit_config_j4() -> ArmConfig {
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    for j in &mut jt.joints { j.strategy = TrackingStrategy::Position; }
    let j4 = &mut jt.joints[3];
    j4.strategy = TrackingStrategy::Mit;
    j4.kp = 10.0;
    j4.kd = 0.3;
    j4.hold_kp = Some(12.0);
    j4.hold_kd = Some(0.4);
    j4.effort_limit_nm = 3.0;
    j4.gravity_scale = 1.0;
    ArmConfig { lease_timeout_ms: 600_000, workspace: None, joint_tracking: Some(jt), ..ArmConfig::rebot_dm() }
}

fn record_j4(timeout_raw: u32) -> MitQualificationFile {
    let l = b601_limits()[3];
    MitQualificationFile {
        version: mit::QUALIFICATION_VERSION,
        joints: vec![MitJointQualification {
            profile_id: "rebot_b601dm".into(), joint: 4, joint_name: ArmConfig::rebot_dm().joint_names[3].clone(),
            zero_fingerprint: mit::zero_fingerprint(0.0), motor_id: 4, reply_id: 0x14, esc_id: 4, mst_id: 0x14, sw_ver: SW_VER,
            pmax: l.p_max, vmax: l.v_max, tmax: l.t_max, timeout_raw, timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
            timeout_unit_verified: true, tested_response: "disabled".into(), measured_expiry_ms: 101.0, worst_latency_ms: 8.0,
            load_envelope: "unloaded, supported".into(), max_kp: 15.0, max_kd: 0.5, effort_ceiling_nm: 4.0,
            date: "2026-11-22T00:00:00Z".into(), trace_id: "test".into(),
        }],
        invalidated: None,
    }
}

fn with_record(inner: &mut SupervisorInner, rec: MitQualificationFile) {
    inner.set_calibration(Some(armd::calibration::CalibrationFile { mit_qualification: serde_json::to_value(&rec).ok(), ..Default::default() }));
}

#[test]
fn mit_timeout_commissioning_writes_ram_only_and_confirms_with_a_separate_read() {
    let bus = FakeBus::new(&b601_limits());
    let mut d = disabled_driver(&bus);
    bus.with(|s| s.tx_log.clear());
    let res = d.commission_motor_timeout(&[3], 2000).expect("commissioning");
    assert_eq!(res.len(), 1);
    let r = &res[0];
    assert_eq!((r.before.timeout_raw, r.written_raw, r.ack_raw, r.readback.timeout_raw), (0, 2000, 2000, 2000));
    assert_eq!((r.readback.motor_id, r.readback.reply_id, r.readback.mst_id, r.readback.sw_ver), (4, 0x14, 0x14, SW_VER));
    assert_eq!((r.unit_us, r.unit_verified), (50, false));
    assert_eq!(bus.with(|s| s.motors[3].reg_u32(damiao::RID_TIMEOUT)), 2000);
    let log = bus.tx_log();
    assert_no_flash_write(&log);
    // exactly one register write: RID 9, little-endian 2000 = 0x07D0, RAM opcode 0x55
    let writes: Vec<_> = log.iter().filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE).collect();
    assert_eq!(writes.len(), 1, "{writes:?}");
    assert_eq!(writes[0].data, [4, 0, 0x55, 9, 0xD0, 0x07, 0, 0]);
    // a separate TIMEOUT read follows the write
    let w = log.iter().position(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE).unwrap();
    assert!(log[w + 1..].iter().any(|f| f.id == 0x7FF && f.data[..4] == [4, 0, 0x33, 9]), "no readback after the write");
    // nothing but register traffic: no enable, no command frame
    assert!(log.iter().all(|f| f.id == 0x7FF), "{log:?}");
}

#[test]
fn mit_timeout_commissioning_refusals() {
    let limits = b601_limits();
    // torque on
    let bus = FakeBus::new(&limits);
    let mut d = disabled_driver(&bus);
    d.set_torque(true).unwrap();
    assert!(d.commission_motor_timeout(&[3], 2000).is_err());
    assert_eq!(bus.with(|s| s.motors[3].reg_u32(damiao::RID_TIMEOUT)), 0);
    // acknowledgement carries another value
    let bus = FakeBus::new(&limits);
    let mut d = disabled_driver(&bus);
    bus.with(|s| s.motors[3].ack_value_override = Some(1000));
    assert!(d.commission_motor_timeout(&[3], 2000).unwrap_err().to_string().contains("acknowledged"));
    // no acknowledgement
    let bus = FakeBus::new(&limits);
    let mut d = disabled_driver(&bus);
    bus.with(|s| s.motors[3].no_write_ack = true);
    assert!(d.commission_motor_timeout(&[3], 2000).unwrap_err().to_string().contains("acknowledge"));
    // acknowledged but not stored: the separate read disagrees
    let bus = FakeBus::new(&limits);
    let mut d = disabled_driver(&bus);
    bus.with(|s| s.motors[3].ignore_reg_writes = true);
    assert!(d.commission_motor_timeout(&[3], 2000).unwrap_err().to_string().contains("readback"));
    // replies on a CAN ID other than the reported MST_ID
    let bus = FakeBus::new(&limits);
    let mut d = disabled_driver(&bus);
    bus.with(|s| s.motors[3].reply_id = 0x24);
    assert!(d.commission_motor_timeout(&[3], 2000).is_err());
    assert_eq!(bus.with(|s| s.motors[3].reg_u32(damiao::RID_TIMEOUT)), 0, "no write before the identity matches");
    // out-of-range counts and silent motor
    let bus = FakeBus::new(&limits);
    let mut d = disabled_driver(&bus);
    assert!(d.commission_motor_timeout(&[3], 0).is_err());
    assert!(d.commission_motor_timeout(&[3], mit::MAX_TIMEOUT_COUNTS + 1).is_err());
    bus.with(|s| s.motors[3].silent = true);
    assert!(d.read_motor_identity(&[3]).is_err());
    assert_no_flash_write(&bus.tx_log());
}

#[test]
fn mit_timeout_command_through_the_supervisor_is_torque_off_only() {
    let bus = FakeBus::new(&b601_limits());
    let d = disabled_driver(&bus);
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(0.02);
    let dry = inner.mit_timeout_command(&json!({"joints": [4], "dry_run": true})).unwrap();
    assert_eq!(dry["readback"][0]["timeout_raw"], json!(0), "{dry}");
    assert_eq!(dry["timeout_unit"]["unit_verified"], json!(false));
    assert!(inner.mit_timeout_command(&json!({"joints": [7]})).is_err(), "the gripper is refused");
    assert!(inner.mit_timeout_command(&json!({"joints": [4], "extra": 1})).is_err());
    let done = inner.mit_timeout_command(&json!({"joints": [4], "timeout_counts": 2400})).unwrap();
    assert_eq!(done["commissioning"][0]["readback"]["timeout_raw"], json!(2400), "{done}");
    assert_eq!(bus.with(|s| s.motors[3].reg_u32(damiao::RID_TIMEOUT)), 2400);
    with_record(&mut inner, record_j4(2400));
    inner.set_torque(true).unwrap();
    let e = inner.mit_timeout_command(&json!({"joints": [4]})).unwrap_err();
    assert_eq!(e.code, "MIT_TIMEOUT_ARMED");
    assert_no_flash_write(&bus.tx_log());
}

/// Supervisor with the Damiao driver on the fake bus and the MIT J4 profile.
fn mit_rig(timeout_on_motor: u32, rec: Option<MitQualificationFile>) -> (SupervisorInner, FakeBus) {
    let bus = FakeBus::new(&b601_limits());
    bus.with(|s| s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, timeout_on_motor));
    let d = disabled_driver(&bus);
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(0.02);
    if let Some(r) = rec { with_record(&mut inner, r); }
    (inner, bus)
}

fn enable_frames(bus: &FakeBus) -> usize {
    bus.tx_log().iter().filter(|f| f.data == damiao::CMD_ENABLE).count()
}

#[test]
fn mit_gate_refuses_to_arm_unqualified_hardware() {
    let cases: Vec<(&str, u32, Option<MitQualificationFile>, Box<dyn Fn(&mut SupervisorInner, &FakeBus)>)> = vec![
        ("no record", 2000, None, Box::new(|_, _| {})),
        ("TIMEOUT readback 0", 0, Some(record_j4(2000)), Box::new(|_, _| {})),
        ("TIMEOUT differs from the record", 1000, Some(record_j4(2000)), Box::new(|_, _| {})),
        ("firmware changed", 2000, Some(record_j4(2000)), Box::new(|_, b| b.with(|s| s.motors[3].set_reg_u32(damiao::RID_SW_VER, SW_VER + 1)))),
        ("other motor ID", 2000, Some(record_j4(2000)), Box::new(|_, b| b.with(|s| s.motors[3].set_reg_u32(damiao::RID_ESC_ID, 9)))),
        ("TMAX changed", 2000, Some(record_j4(2000)), Box::new(|_, b| b.with(|s| s.motors[3].set_reg_u32(damiao::RID_TMAX, 12.0f32.to_bits())))),
        ("register reads unanswered", 2000, Some(record_j4(2000)), Box::new(|_, b| b.with(|s| s.motors[3].silent = true))),
        ("untested", 2000, Some({ let mut r = record_j4(2000); r.joints[0].measured_expiry_ms = 0.0; r }), Box::new(|_, _| {})),
        ("invalidated by a zero", 2000, Some({ let mut r = record_j4(2000);
            r.invalidated = Some(armd::calibration::CalibrationStale { reason: "zero".into(), at: "now".into() }); r }), Box::new(|_, _| {})),
        ("joint zero changed", 2000, Some(record_j4(2000)), Box::new(|inner, _| inner.set_joint_zero_offsets(&[0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0]).unwrap())),
        ("kp above the qualified ceiling", 2000, Some({ let mut r = record_j4(2000); r.joints[0].max_kp = 8.0; r }), Box::new(|_, _| {})),
        ("effort limit above the qualified ceiling", 2000, Some({ let mut r = record_j4(2000); r.joints[0].effort_ceiling_nm = 2.0; r }), Box::new(|_, _| {})),
        ("timeout unit not verified on the device", 2000, Some({ let mut r = record_j4(2000); r.joints[0].timeout_unit_verified = false; r }), Box::new(|_, _| {})),
        ("measured latency does not fit the 45 ms deadline at 50 Hz (20 + 21 + 5 ms margin)", 2000,
            Some({ let mut r = record_j4(2000); r.joints[0].worst_latency_ms = 21.0; r }), Box::new(|_, _| {})),
        ("control period too long for the deadline (25 Hz: 40 + 8 + 5 ms)", 2000, Some(record_j4(2000)),
            Box::new(|inner, _| inner.config.control_frequency_hz = 25)),
        ("other profile", 2000, Some({ let mut r = record_j4(2000); r.joints[0].profile_id = "so_arm101".into(); r }), Box::new(|_, _| {})),
    ];
    for (label, timeout, rec, mutate) in cases {
        let (mut inner, bus) = mit_rig(timeout, rec);
        mutate(&mut inner, &bus);
        bus.with(|s| s.tx_log.clear());
        let err = inner.set_torque(true).expect_err(label);
        assert!(err.to_string().contains("MIT_UNQUALIFIED"), "{label}: {err}");
        assert!(!inner.is_torqued, "{label}: armed");
        assert_eq!(enable_frames(&bus), 0, "{label}: an enable frame was sent");
        assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)), "{label}");
        assert!(inner.mit_gate_error.is_some(), "{label}");
        assert_no_flash_write(&bus.tx_log());
    }
}

#[test]
fn mit_gate_passes_qualified_hardware_and_rechecks_on_every_arm() {
    let (mut inner, bus) = mit_rig(2000, Some(record_j4(2000)));
    inner.set_torque(true).expect("qualified J4 arms");
    assert!(inner.is_torqued && inner.mit_gate_error.is_none());
    // the motor lost its RAM TIMEOUT (power dip): the rearm refuses
    bus.with(|s| s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, 0));
    let err = inner.set_torque(true).unwrap_err();
    assert!(err.to_string().contains("TIMEOUT readback is 0"), "{err}");
    assert_no_flash_write(&bus.tx_log());
}

#[test]
fn mit_not_requested_sends_no_register_reads_at_arm() {
    // the Phase 1 profile (tracking off) and an enabled position-lead profile: no readback
    for jt in [None, Some(JointTrackingConfig::rebot_dm()), Some(JointTrackingConfig { enabled: true, ..JointTrackingConfig::rebot_dm() })] {
        let bus = FakeBus::new(&b601_limits());
        let d = disabled_driver(&bus);
        let cfg = ArmConfig { lease_timeout_ms: 600_000, workspace: None, joint_tracking: jt, ..ArmConfig::rebot_dm() };
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(d));
        inner.lease_enabled = false;
        inner.tick(0.02);
        bus.with(|s| s.tx_log.clear());
        inner.set_torque(true).unwrap();
        let reads = bus.tx_log().iter().filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ).count();
        assert_eq!(reads, 0);
    }
}

#[test]
fn mit_config_acceptance_keeps_phase1_results() {
    let mut base = mit_config_j4();
    assert!(base.validate().is_ok(), "{:?}", base.validate());
    let mut cases: Vec<(&str, Box<dyn Fn(&mut JointTrackingConfig)>)> = vec![
        ("mit without hold gains (a Phase 1 profile)", Box::new(|jt| { jt.joints[3].hold_kp = None; })),
        ("hold gains on a position_lead joint", Box::new(|jt| jt.joints[0].hold_kp = Some(1.0))),
        ("zero kp", Box::new(|jt| jt.joints[3].kp = 0.0)),
        ("zero effort limit", Box::new(|jt| jt.joints[3].effort_limit_nm = 0.0)),
        ("kp above the hand-guide ceiling 15", Box::new(|jt| jt.joints[3].kp = 15.5)),
        ("hold_kd above the ceiling 0.5", Box::new(|jt| jt.joints[3].hold_kd = Some(0.6))),
        ("coulomb above the effort limit", Box::new(|jt| jt.joints[3].coulomb_nm = 3.5)),
        ("viscous term at max velocity above the limit", Box::new(|jt| jt.joints[3].viscous_nm_s_rad = 2.0)),
        ("inertia term at max acceleration above the limit", Box::new(|jt| jt.joints[3].inertia_kg_m2 = 1.0)),
    ];
    cases.push(("mit on the gripper slot", Box::new(|jt| jt.joints.push(jt.joints[3].clone()))));
    for (label, mutate) in &cases {
        let mut cfg = base.clone();
        mutate(cfg.joint_tracking.as_mut().unwrap());
        assert!(cfg.validate().is_err(), "{label}: accepted");
        // a disabled block never rejects the profile (as in Phase 1)
        cfg.joint_tracking.as_mut().unwrap().enabled = false;
        assert!(cfg.validate().is_ok(), "{label}: a disabled block rejected the profile");
    }
    // the shipped profile: MIT disabled, no hold keys, serialized form unchanged
    let shipped = ArmConfig::from_profile_file(concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/profile/rebot-b601dm.json")).unwrap();
    let jt = shipped.joint_tracking.as_ref().unwrap();
    assert!(!jt.enabled && !jt.mit_requested() && jt.joints.iter().all(|j| j.strategy != TrackingStrategy::Mit && j.hold_kp.is_none()));
    let text = serde_json::to_string(jt).unwrap();
    assert!(!text.contains("hold_kp") && !text.contains("hold_kd"), "{text}");
}

#[test]
fn mit_qualification_record_is_checked_against_a_fresh_readback_before_storing() {
    let (mut inner, bus) = mit_rig(2000, None);
    let body = |r: &MitQualificationFile| json!({"record": serde_json::to_value(r).unwrap()});
    // matches the motor: the file to store holds the record
    let file = inner.mit_qualification_check(&body(&record_j4(2000))).unwrap();
    assert_eq!(file.mit_record().unwrap(), Some(record_j4(2000)));
    assert!(file.validate().is_ok());
    // mismatching TIMEOUT / firmware / zero, unknown fields, torque on: refused
    assert_eq!(inner.mit_qualification_check(&body(&record_j4(1000))).unwrap_err().code, "MIT_QUALIFICATION_MISMATCH");
    bus.with(|s| s.motors[3].set_reg_u32(damiao::RID_SW_VER, 1));
    assert_eq!(inner.mit_qualification_check(&body(&record_j4(2000))).unwrap_err().code, "MIT_QUALIFICATION_MISMATCH");
    bus.with(|s| s.motors[3].set_reg_u32(damiao::RID_SW_VER, SW_VER));
    let mut v = body(&record_j4(2000));
    v["record"]["joints"][0]["extra"] = json!(1);
    assert!(inner.mit_qualification_check(&v).is_err());
    inner.set_joint_zero_offsets(&[0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0]).unwrap();
    assert!(inner.mit_qualification_check(&body(&record_j4(2000))).is_err());
}

#[test]
fn mit_qualification_only_calibration_file_validates() {
    let mut file = armd::calibration::CalibrationFile::default();
    file.set_mit_record(&record_j4(2000));
    assert!(file.validate().is_ok(), "{:?}", file.validate());
    assert_eq!(file.mit_record().unwrap(), Some(record_j4(2000)));
    let text = serde_json::to_string(&file).unwrap();
    let back: armd::calibration::CalibrationFile = serde_json::from_str(&text).unwrap();
    assert_eq!(back, file);
    // an invalid record is refused on its own; the file stays valid (review #1)
    for mutate in [|r: &mut MitQualificationFile| r.joints[0].timeout_raw = 0, |r: &mut MitQualificationFile| r.joints[0].effort_ceiling_nm = 99.0] {
        let mut r = record_j4(2000);
        mutate(&mut r);
        let mut bad = file.clone();
        bad.set_mit_record(&r);
        assert!(bad.mit_record().is_err());
        assert!(bad.validate().is_ok());
    }
}

// ---------------------------------------------------------------------------------------------
// Supervisor MIT output (commit 3): handover, mixed output, stationary hold, safety matrix.
// The real DamiaoArmDriver on the fake bus (motors follow their command at once).
// ---------------------------------------------------------------------------------------------

const DT: f64 = 0.02;
const Q0: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];

fn record_j4_zero(timeout_raw: u32, zero: f32) -> MitQualificationFile {
    let mut r = record_j4(timeout_raw);
    r.joints[0].zero_fingerprint = mit::zero_fingerprint(zero);
    r
}

/// Armed supervisor with MIT active on J4 (motor zero offset `zero_j4`, rad).
fn active_rig(cfg: ArmConfig, zero_j4: f32) -> (SupervisorInner, FakeBus) {
    let bus = FakeBus::new(&b601_limits());
    bus.with(|s| for (i, m) in s.motors.iter_mut().enumerate() { m.q = Q0[i] + if i == 3 { zero_j4 } else { 0.0 }; });
    let mut d = disabled_driver(&bus);
    d.check_mapping_ranges();
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(d));
    inner.lease_enabled = false;
    if zero_j4 != 0.0 { inner.set_joint_zero_offsets(&[0.0, 0.0, 0.0, zero_j4, 0.0, 0.0, 0.0]).unwrap(); }
    inner.tick(DT);
    inner.mit_timeout_command(&json!({"joints": [4]})).unwrap();
    with_record(&mut inner, record_j4_zero(2000, zero_j4));
    inner.set_torque(true).unwrap();
    assert!(inner.mit_motion_blocker().is_some(), "motion refused before the handover");
    inner.tick(DT);
    inner.tick(DT);
    assert!(inner.mit_active(), "handover: {:?}", inner.mit.handover_error);
    (inner, bus)
}

fn mit_frames(log: &[armd::transport::CanFrame], id: u16) -> Vec<(f32, f32, f32, f32, f32)> {
    let l = b601_limits()[id as usize - 1];
    log.iter().filter(|f| f.id == u32::from(id) && ![damiao::CMD_ENABLE, damiao::CMD_DISABLE, damiao::CMD_CLEAR_FAULTS].contains(&f.data))
        .map(|f| match mit_support::decode_mit(&f.data, &l) {
            mit_support::MotorCmd::Mit { p, v, kp, kd, t } => (p, v, kp, kd, t),
            _ => unreachable!(),
        }).collect()
}

fn posvel_frames(log: &[armd::transport::CanFrame], id: u16) -> Vec<f32> {
    log.iter().filter(|f| f.id == 0x100 + u32::from(id)).map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]])).collect()
}

fn lsb(span: f32, bits: u32) -> f32 { span / ((1u32 << bits) - 1) as f32 }

#[test]
fn mit_frame_fields_and_effort() {
    let zero = 0.05;
    let (mut inner, bus) = active_rig(mit_config_j4(), zero);
    let l4 = b601_limits()[3];
    let (lsb_p, lsb_v, lsb_k, lsb_d, lsb_t) = (lsb(2.0 * l4.p_max, 16), lsb(2.0 * l4.v_max, 12), lsb(l4.kp_max, 12), lsb(l4.kd_max, 12), lsb(2.0 * l4.t_max, 12));
    // a J4 excursion out and back (reversal)
    bus.clear_log();
    let mut nominal = Vec::new();
    for leg in [0.10_f32, -0.10] {
        let inp = inner.plan_input().unwrap();
        let mut goal = [0.0_f64; 6];
        for i in 0..6 { goal[i] = f64::from(inner.commanded_positions[i]); }
        goal[3] += f64::from(leg);
        inner.install_joint_goal(&inp, &goal, 0.3, "joint_move", None, None).unwrap();
        for _ in 0..200 {
            inner.tick(DT);
            nominal.push((inner.commanded_positions[3], inner.commanded_velocities[3], inner.commanded_accelerations[3]));
            if inner.active_trajectory.is_none() && inner.commanded_velocities[3] == 0.0 { break; }
        }
    }
    assert!(!inner.safety.state().is_stopped(), "{:?}", inner.safety.state());
    let log = bus.tx_log();
    let frames = mit_frames(&log, 4);
    assert_eq!(frames.len(), nominal.len(), "one MIT frame per tick on J4");
    assert!(posvel_frames(&log, 4).is_empty(), "J4 gets no POS_VEL frame while MIT owns it");
    for k in [1u16, 2, 3, 5, 6, 7] { assert_eq!(posvel_frames(&log, k).len(), nominal.len(), "J{k} stays POS_VEL"); }
    let jt = mit_config_j4().joint_tracking.unwrap().joints[3].clone();
    let mut prev_t: Option<f32> = None;
    let mut saw_forward = false;
    let mut saw_reverse = false;
    for (k, ((p, v, kp, kd, t), (qd, vd, ad))) in frames.iter().zip(&nominal).enumerate() {
        // position after the joint zero conversion, velocity, gains
        assert!((p - (qd + zero)).abs() <= lsb_p, "tick {k}: wire position {p} != nominal {qd} + zero {zero}");
        assert!((v - vd).abs() <= lsb_v, "tick {k}: wire velocity {v} != nominal {vd}");
        let moving = *vd != 0.0 || *ad != 0.0;
        let (ekp, ekd) = if moving { (jt.kp, jt.kd) } else { (jt.hold_kp.unwrap(), jt.hold_kd.unwrap()) };
        assert!((kp - ekp).abs() <= lsb_k && (kd - ekd).abs() <= lsb_d, "tick {k}: gains {kp}/{kd}, expected {ekp}/{ekd}");
        assert!(t.abs() <= jt.effort_limit_nm + 1e-4, "tick {k}: torque {t} beyond the effort limit");
        if let Some(pt) = prev_t {
            assert!((t - pt).abs() <= 15.0 * DT as f32 + lsb_t + 1e-4, "tick {k}: torque slew {pt} -> {t}");
        }
        prev_t = Some(*t);
        saw_forward |= *vd > 0.01;
        saw_reverse |= *vd < -0.01;
    }
    assert!(saw_forward && saw_reverse);
    // the feed-forward equals the qualified gravity support (gravity_scale 1) once settled
    let g = inner.config.arm_model.gravity_torques(&inner.measured_positions[..6]).unwrap()[3];
    let last = frames.last().unwrap().4;
    assert!((last - g.clamp(-3.0, 3.0)).abs() <= lsb_t + 1e-3, "settled tau_ff {last} vs gravity {g}");
    assert_no_flash_write(&log);
}

#[test]
fn mit_effort_saturation_and_empty_interval() {
    // effort limit below the J4 gravity load: tau_ff saturates inside [-L, L] on the wire grid
    let mut cfg = mit_config_j4();
    {
        let j4 = &mut cfg.joint_tracking.as_mut().unwrap().joints[3];
        j4.effort_limit_nm = 0.2;
    }
    let (mut inner, bus) = active_rig(cfg, 0.0);
    let g = inner.config.arm_model.gravity_torques(&inner.measured_positions[..6]).unwrap()[3];
    assert!(g.abs() > 0.2, "pose must load J4 beyond the limit, gravity {g}");
    bus.clear_log();
    for _ in 0..10 { inner.tick(DT); }
    let frames = mit_frames(&bus.tx_log(), 4);
    let l4 = b601_limits()[3];
    let lsb_t = lsb(2.0 * l4.t_max, 12);
    // inside [-L, L] (and [-L - pd, L - pd]) on the wire grid, at the saturated bound
    for f in &frames { assert!(f.4.abs() <= 0.2 + 1e-5 && f.4.abs() >= 0.2 - 4.0 * lsb_t, "saturated torque {}", f.4); }
    assert!(frames.iter().all(|f| f.4.signum() == g.signum()), "saturates on the gravity side");

    // PD alone exceeds the limit: [-L, L] and [-L - pd, L - pd] do not meet -> nothing is written
    let mut cfg = mit_config_j4();
    {
        let j4 = &mut cfg.joint_tracking.as_mut().unwrap().joints[3];
        j4.effort_limit_nm = 1.0;
        j4.gravity_scale = 0.0;
    }
    let (mut inner, bus) = active_rig(cfg, 0.0);
    bus.with(|s| { s.follow_commands = false; s.motors[3].q += 0.2; }); // hold kp 12 x 0.2 rad = 2.4 N·m > 2 L
    bus.with(|s| s.push_all_feedback());
    bus.clear_log();
    inner.tick(DT);
    assert!(bus.tx_log().iter().all(|f| f.id == 0x7FF), "a batch was written despite the empty interval: {:?}", bus.tx_log());
    let fault = inner.safety.fault().expect("refused output latches a fault");
    assert_eq!(fault.code, armd::FaultCode::DriverWriteFailed, "{fault:?}");
    assert!(fault.message.contains("empty effort interval"), "{}", fault.message);
}

#[test]
fn mit_owner_handover() {
    // activation refused while a joint moves
    let bus = FakeBus::new(&b601_limits());
    bus.with(|s| for (i, m) in s.motors.iter_mut().enumerate() { m.q = Q0[i]; });
    let mut d = disabled_driver(&bus);
    d.check_mapping_ranges();
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(DT);
    inner.mit_timeout_command(&json!({"joints": [4]})).unwrap();
    with_record(&mut inner, record_j4(2000));
    inner.set_torque(true).unwrap();
    bus.with(|s| { s.follow_commands = false; s.motors[1].v = 0.3; });
    for _ in 0..5 { bus.with(|s| s.push_all_feedback()); inner.tick(DT); }
    assert!(!inner.mit_active(), "handover while J2 moves");
    let e = inner.plan_input().unwrap_err();
    assert!(e.to_string().contains("MIT tracking is not active"), "{e}");
    bus.with(|s| { s.motors[1].v = 0.0; s.follow_commands = true; });
    bus.clear_log();
    for _ in 0..3 { bus.with(|s| s.push_all_feedback()); inner.tick(DT); }
    assert!(inner.mit_active());
    // the handover: J4 mode write, then the MIT hold at the measured pose with zero velocity,
    // the jaw keeps its command
    let log = bus.tx_log();
    let w = log.iter().position(|f| f.id == 0x7FF && f.data[..4] == [4, 0, 0x55, 10]).expect("J4 CTRL_MODE write");
    let hold = mit_frames(&log[w..], 4)[0];
    assert!((hold.0 - Q0[3]).abs() < 1e-3 && hold.1.abs() < 0.02, "hold {hold:?}");
    assert!(posvel_frames(&log, 7).iter().all(|p| (p - inner.commanded_positions[6]).abs() < 1e-6));
    assert!(!log.iter().any(|f| f.id == 0x7FF && f.data[2] == 0x55 && f.data[3] == 10 && f.data[0] != 4), "only J4 switched");

    // gravity compensation entry hands back first (POS_VEL, confirmed), exit, then a fresh handover
    bus.clear_log();
    inner.enter_gravity_comp(Default::default()).expect("gravity comp entry");
    assert!(!inner.mit_active() && inner.mit.owner == mit::MitOwner::Off);
    let log = bus.tx_log();
    let back = log.iter().position(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_POS_VEL).data).expect("J4 back to POS_VEL");
    let gc = log.iter().position(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(1, damiao::CTRL_MODE_MIT).data).expect("gravity comp switch");
    assert!(back < gc, "hand back before the gravity compensation switch");
    for _ in 0..20 { inner.tick(DT); }
    assert!(!inner.mit_active(), "no tracking overlay during hand guiding");
    bus.clear_log();
    inner.exit_gravity_comp("TEST", "test exit");
    for _ in 0..200 { inner.tick(DT); if inner.gravity_comp.is_none() { break; } }
    assert!(inner.gravity_comp.is_none());
    for _ in 0..5 { inner.tick(DT); }
    assert!(inner.mit_active(), "re-entry through a fresh handover: {:?}", inner.mit.handover_error);
    assert!(bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_MIT).data));

    // E-stop: the arm holds; J4 gets the stationary MIT hold (hold gains, zero velocity)
    inner.apply_stop_policy(armd::StopPolicy::EStopLatch);
    bus.clear_log();
    for _ in 0..5 { inner.tick(DT); }
    let jt = mit_config_j4().joint_tracking.unwrap().joints[3].clone();
    let frames = mit_frames(&bus.tx_log(), 4);
    assert!(!frames.is_empty());
    for f in &frames { assert!(f.1.abs() < 0.02 && (f.2 - jt.hold_kp.unwrap()).abs() < 0.2, "{f:?}"); }
}

#[test]
fn mit_park_hands_back_before_moving() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    bus.clear_log();
    inner.apply_stop_policy(armd::StopPolicy::NormalUnload);
    for _ in 0..3 { inner.tick(DT); }
    assert!(!inner.mit_active(), "park hands back at rest");
    let log = bus.tx_log();
    let back = log.iter().position(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_POS_VEL).data).expect("J4 back to POS_VEL");
    assert!(mit_frames(&log[back..], 4).iter().all(|f| f.1 == 0.0 || f.1.abs() < 0.02), "no MIT motion frame after the hand back");
    for _ in 0..100 { inner.tick(DT); }
    let log = bus.tx_log();
    assert!(mit_frames(&log[back + 1..], 4).len() <= 1, "J4 parks in POS_VEL");
    assert!(posvel_frames(&log[back..], 4).len() > 50);
}

#[test]
fn mit_collision_and_guarded_retreat_hold_without_dynamic_terms() {
    let mut cfg = mit_config_j4();
    {
        let j4 = &mut cfg.joint_tracking.as_mut().unwrap().joints[3];
        j4.coulomb_nm = 0.5;
        j4.viscous_nm_s_rad = 0.5;
    }
    let (mut inner, _bus) = active_rig(cfg, 0.0);
    // moving reference: dynamic terms present
    let mut t = vec![armd::driver::JointTarget::position(0.0); 7];
    for (i, tt) in t.iter_mut().enumerate() { tt.position = inner.commanded_positions[i]; tt.velocity = Some(0.0); }
    t[3].velocity = Some(0.05);
    inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(
        TrajectoryPlanner::plan(&inner.commanded_positions.clone(), &[0.0; 7], &inner.commanded_positions.clone(), &[0.0; 7], Some(1.0), &inner.config.limits).unwrap(), None));
    let g = inner.config.arm_model.gravity_torques(&inner.measured_positions[..6]).unwrap()[3];
    inner.mit.last_ff[3] = g + 0.5 + 0.025;
    let mut moving = t.clone();
    let ff = inner.mit_prepare(&mut moving, DT).unwrap();
    assert!((ff[3] - (g + 0.5 + 0.025)).abs() < 0.02, "coulomb + viscous on top of gravity: {} vs {}", ff[3], g + 0.525);
    // collision hold: dynamic terms removed (slewed), hold gains
    inner.collision.hold = Some(armd::collision::CollisionHold { joint: 1, residual_nm: 9.0, threshold_nm: 5.0, since_s: 0.0, quiet_s: 0.0 });
    let mut held = t.clone();
    let ff = inner.mit_prepare(&mut held, DT).unwrap();
    assert!(ff[3] < g + 0.525 - 0.29 && ff[3] >= g + 0.525 - 0.30 - 0.01, "slewed toward gravity only: {}", ff[3]);
    assert!((held[3].kp.unwrap() - 12.0).abs() < 0.13, "hold gain on the wire grid");
    inner.collision.hold = None;
    // guarded retreat: the total effort never pushes into the held direction
    inner.torque_guard.hold = Some(armd::torque_guard::TorqueHold { joint: 3, hold_q: inner.commanded_positions[3], sign: 1.0,
        torque_nm: 3.0, threshold_nm: 2.0, since_s: 0.0, anchor_q: inner.commanded_positions[3], push_in: false, retreat: Default::default() });
    inner.mit.last_ff[3] = 0.0;
    let mut r = t.clone();
    r[3].velocity = Some(0.0);
    let ff = inner.mit_prepare(&mut r, DT).unwrap();
    let kp = r[3].kp.unwrap();
    let kd = r[3].kd.unwrap();
    let pd = kp * (r[3].position - inner.measured_positions[3]) + kd * (0.0 - inner.measured_velocities[3]);
    assert!(pd + ff[3] <= 1e-3, "effort {} into the held direction", pd + ff[3]);
    // infeasible retreat (pd already pushes into the hold beyond what the slew allows): refused
    inner.measured_positions[3] -= 0.1;
    let mut r2 = t.clone();
    r2[3].velocity = Some(0.0);
    inner.mit.last_ff[3] = 0.0;
    assert!(inner.mit_prepare(&mut r2, DT).is_err(), "infeasible retreat must be refused");
}

#[test]
fn mit_bus_loss_cancels_the_group() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    bus.with(|s| s.motors[3].silent = true);
    bus.clear_log();
    let start = std::time::Instant::now();
    while start.elapsed() < std::time::Duration::from_millis(60) {
        std::thread::sleep(std::time::Duration::from_millis(5));
        inner.tick(DT);
        if inner.safety.state().is_stopped() { break; }
    }
    let fault = inner.safety.fault().expect("bus loss latched");
    assert_eq!(fault.code, armd::FaultCode::BusLoss, "{fault:?}");
    assert!(!fault.recoverable);
    assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
    bus.clear_log();
    for _ in 0..10 { std::thread::sleep(std::time::Duration::from_millis(5)); inner.tick(DT); }
    assert!(bus.tx_log().iter().all(|f| f.id == 0x7FF), "output after the loss: {:?}", bus.tx_log());
    assert!(command_frames(&bus.tx_log()).is_empty());
}

fn command_frames(log: &[armd::transport::CanFrame]) -> Vec<armd::transport::CanFrame> {
    log.iter().filter(|f| (1..=7).contains(&f.id) || (0x101..=0x107).contains(&f.id)).copied().collect()
}

#[test]
fn mit_partial_send_reconnect_through_the_supervisor() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    bus.with(|s| s.fail_tx_after = Some(3));
    inner.tick(DT);
    let fault = inner.safety.fault().expect("partial write latches");
    assert_eq!(fault.code, armd::FaultCode::BusLoss, "a loss latches BUS_LOSS wherever it is detected");
    assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
    assert!(inner.tracking.last_issued_positions.is_none(), "history discarded");
    bus.with(|s| s.fail_tx_after = None);
    bus.clear_log();
    for _ in 0..5 { inner.tick(DT); }
    assert!(command_frames(&bus.tx_log()).is_empty(), "{:?}", bus.tx_log());
    // time alone resumes nothing
    std::thread::sleep(std::time::Duration::from_millis(400));
    bus.clear_log();
    for _ in 0..5 { inner.tick(DT); }
    assert!(bus.tx_log().is_empty(), "polls resumed without clearance: {:?}", bus.tx_log());
    // reconnect: explicit clearance (polls resume, the first try waits for fresh feedback), rearm
    // (fresh readback, base modes restored), then a fresh handover
    let mut cleared = false;
    for _ in 0..20 {
        if inner.clear_fault().is_ok() { cleared = true; break; }
        std::thread::sleep(std::time::Duration::from_millis(40));
        inner.tick(DT);
    }
    assert!(cleared, "clearance");
    bus.clear_log();
    inner.set_torque(true).expect("rearm");
    assert!(bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[..4] == [4, 0, 0x33, 9]), "TIMEOUT readback on the rearm");
    assert!(bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_POS_VEL).data), "J4 base mode restored");
    for _ in 0..3 { inner.tick(DT); }
    assert!(inner.mit_active(), "{:?}", inner.mit.handover_error);
}

#[test]
fn mit_failed_rollback_blocks_fault_clearance() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    // park: the hand back is acknowledged, then the motor goes silent -> rollback unconfirmed
    bus.with(|s| s.motors[3].silent_after_mode_ack = true);
    inner.apply_stop_policy(armd::StopPolicy::NormalUnload);
    inner.tick(DT);
    assert_eq!(inner.mit.owner, mit::MitOwner::Lost, "{:?}", inner.mit.handover_error);
    assert!(inner.safety.state().is_stopped());
    let e = inner.clear_fault().unwrap_err();
    assert!(e.to_string().contains("uncertain"), "{e}");
}

/// Review (Phase 2) #1: `"hold_kp": null` / `"hold_kd": null` on a non-MIT joint is refused like
/// at f258cf9 (an unknown key there), and so is null on an MIT joint.
#[test]
fn mit_hold_keys_null_is_refused_like_the_base() {
    for (slot, key) in [(0usize, "hold_kp"), (0, "hold_kd"), (3, "hold_kp")] {
        let mut v = serde_json::to_value(mit_config_j4().joint_tracking.unwrap()).unwrap();
        v["joints"][slot][key] = serde_json::Value::Null;
        let jt: JointTrackingConfig = serde_json::from_value(v).unwrap();
        let mut cfg = ArmConfig { joint_tracking: Some(jt), ..mit_config_j4() };
        assert!(cfg.validate().is_err(), "{key}: null accepted on slot {slot}");
    }
}

/// Review #3: a rearm of an armed arm with MIT active hands back first (at rest), then reads the
/// motors; a failed readback (TIMEOUT lost, motor silent) revokes the authorization before
/// returning: MIT stays inactive afterwards and motions are refused.
#[test]
fn mit_failed_readback_on_rearm_revokes_authorization() {
    for label in ["timeout zero", "silent motor"] {
        let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
        bus.clear_log();
        let res = if label == "timeout zero" {
            bus.with(|s| s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, 0));
            inner.set_torque(true)
        } else {
            // J4 answers the hand back, then nothing (readback unanswered)
            inner.mit_hand_back("test").unwrap();
            bus.with(|s| s.motors[3].silent = true);
            inner.set_torque(true)
        };
        assert!(res.is_err(), "{label}: rearm passed");
        assert!(!inner.mit_active(), "{label}: MIT still owns J4");
        assert!(!inner.mit.armed_ok, "{label}: authorization kept");
        let log = bus.tx_log();
        if label == "timeout zero" {
            let back = log.iter().position(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_POS_VEL).data)
                .expect("hand back before the readback");
            let read = log.iter().position(|f| f.id == 0x7FF && f.data[..4] == [4, 0, 0x33, 9]).expect("TIMEOUT readback");
            assert!(back < read, "{label}: readback before the hand back");
        }
        bus.with(|s| s.motors[3].silent = false);
        for _ in 0..5 { inner.tick(DT); }
        assert!(!inner.mit_active(), "{label}: MIT re-activated without a passed gate");
        assert!(inner.mit_motion_blocker().is_some(), "{label}: motions allowed");
    }
}

/// Review #5: a disable that fails (no frame reaches the motors, they stay enabled in MIT) never
/// reopens the cached MIT frame for driver recovery: a joint going stale afterwards gets no
/// replay and no invented hold.
#[test]
fn mit_failed_disable_never_replays_the_cached_mit_frame() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    for _ in 0..3 { inner.tick(DT); }
    bus.with(|s| s.fail_tx_ids = (1..=7).collect());
    inner.apply_stop_policy(armd::StopPolicy::ThermalTrip);
    assert!(bus.with(|s| s.motors[3].enabled), "the disable must not have reached J4");
    bus.with(|s| { s.fail_tx_ids.clear(); s.motors[3].silent = true; });
    bus.clear_log();
    for _ in 0..12 {
        std::thread::sleep(std::time::Duration::from_millis(10));
        inner.tick(DT);
    }
    let replays: Vec<_> = bus.tx_log().into_iter().filter(|f| f.id == 4 && f.data != damiao::CMD_DISABLE).collect();
    assert!(replays.is_empty(), "cached MIT frame replayed after the failed disable: {replays:?}");
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x104), "invented POS_VEL hold on the MIT joint");
}

/// Review #8: the effort bound holds for the frame the motor decodes: with every field on the
/// wire grid (position after the zero offset, velocity, kp, kd, torque), |pd + tau_ff| <= L where
/// pd uses the decoded fields and the tick's measured state. Saturated case (gravity above L) over
/// several limits and zero offsets, moving out and back and holding.
#[test]
fn mit_effort_bound_holds_on_decoded_wire_fields() {
    let mut worst_excess = f32::MIN;
    let mut frames_checked = 0;
    for (n, l) in [0.20_f32, 0.213, 0.227, 0.241].into_iter().enumerate() {
        for zero in [0.0_f32, 0.0131, 0.0287] {
            let mut cfg = mit_config_j4();
            { let j4 = &mut cfg.joint_tracking.as_mut().unwrap().joints[3]; j4.effort_limit_nm = l; j4.kd = 0.47; j4.hold_kd = Some(0.47); j4.kp = 13.3; }
            let (mut inner, bus) = active_rig(cfg, zero);
            for leg in [0.04_f32 + 0.003 * n as f32, -0.04] {
                let inp = inner.plan_input().unwrap();
                let mut goal = [0.0_f64; 6];
                for i in 0..6 { goal[i] = f64::from(inner.commanded_positions[i]); }
                goal[3] += f64::from(leg);
                inner.install_joint_goal(&inp, &goal, 0.3, "joint_move", None, None).unwrap();
                for _ in 0..150 {
                    // the joint lags its command by a varying amount: pd is nonzero
                    bus.with(|s| s.follow_commands = false);
                    bus.clear_log();
                    inner.tick(DT);
                    bus.with(|s| { let m = &mut s.motors[3]; if let Some(mit_support::MotorCmd::Mit { p, .. }) = m.last_cmd { m.q += 0.7 * (p - m.q); m.v = 0.02; } });
                    let (q_m, v_m) = (inner.measured_positions[3], inner.measured_velocities[3]);
                    if let Some(&(p, v, kp, kd, t)) = mit_frames(&bus.tx_log(), 4).first() {
                        let pd = kp * (p - zero - q_m) + kd * (v - v_m);
                        worst_excess = worst_excess.max((pd + t).abs() - l);
                        frames_checked += 1;
                    }
                    assert!(!inner.safety.state().is_stopped(), "{:?}", inner.safety.state());
                    if inner.active_trajectory.is_none() && inner.commanded_velocities[3] == 0.0 { break; }
                }
            }
        }
    }
    eprintln!("decoded |pd + tau| - L: worst {worst_excess:.6} over {frames_checked} frames");
    assert!(frames_checked > 500);
    assert!(worst_excess <= 1e-5, "decoded effort exceeds the limit by {worst_excess}");
    assert!(worst_excess > -0.01, "the bound was not exercised: {worst_excess}");
}

/// Review #9: a loss latches BUS_LOSS also when the arm is already stopped by another fault.
#[test]
fn mit_loss_latches_bus_loss_even_when_already_stopped() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    inner.safety.trigger_fault_coded(armd::FaultCode::FollowingError, armd::FaultReason::Other("test".into()));
    bus.with(|s| s.motors[3].silent = true);
    let start = std::time::Instant::now();
    while start.elapsed() < std::time::Duration::from_millis(80) && inner.mit.owner != mit::MitOwner::Lost {
        std::thread::sleep(std::time::Duration::from_millis(5));
        inner.tick(DT);
    }
    assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
    assert_eq!(inner.safety.fault().unwrap().code, armd::FaultCode::BusLoss);
}

/// Review #10: gravity compensation entry validates its request (scale, damping) before it
/// touches ownership: a refused request leaves MIT active and sends no mode write.
#[test]
fn mit_gravity_entry_validates_before_the_hand_back() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    bus.clear_log();
    let bad = [armd::gravity_comp::GravityCompRequest { scale: Some(9.0), damping_kd: None },
        armd::gravity_comp::GravityCompRequest { scale: None, damping_kd: Some(vec![99.0; 6]) }];
    for req in bad {
        assert!(inner.enter_gravity_comp(req).is_err());
        assert!(inner.mit_active(), "a refused request changed the owner");
    }
    assert!(!bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == 0x55), "mode write for a refused request");
}

/// Review #10 (real path): a collision trip during an MIT move turns J4 into the stationary hold:
/// hold gains, zero velocity, tau_ff slewing (15 N·m/s) from the moving value to gravity only.
#[test]
fn mit_collision_trip_holds_through_the_tick() {
    let mut cfg = mit_config_j4();
    { let j4 = &mut cfg.joint_tracking.as_mut().unwrap().joints[3]; j4.coulomb_nm = 0.8; }
    let (mut inner, bus) = active_rig(cfg, 0.0);
    // the fake reports the model gravity torque (no residual) plus `extra` on J2
    let feed = |inner: &SupervisorInner, extra: f32| {
        let g = inner.config.arm_model.gravity_torques(&inner.measured_positions[..6]).unwrap();
        bus.with(|s| for i in 0..6 { s.motors[i].tau = g[i] + if i == 1 { extra } else { 0.0 }; });
    };
    feed(&inner, 0.0);
    for _ in 0..3 { inner.tick(DT); feed(&inner, 0.0); }
    inner.collision.enabled = true;
    let inp = inner.plan_input().unwrap();
    let mut goal = [0.0_f64; 6];
    for i in 0..6 { goal[i] = f64::from(inner.commanded_positions[i]); }
    goal[3] -= 0.3;
    inner.install_joint_goal(&inp, &goal, 0.3, "joint_move", None, None).unwrap();
    for _ in 0..25 { inner.tick(DT); feed(&inner, 0.0); }
    let moving_tau = mit_frames(&bus.tx_log(), 4).last().unwrap().4;
    assert!(inner.active_trajectory.is_some(), "{:?}", inner.current_observation().action.map(|a| a.message));
    // contact on J2: residual far above the trip level
    let mut tripped_at = None;
    let mut prev = moving_tau;
    for k in 0..10 {
        feed(&inner, 25.0);
        bus.clear_log();
        inner.tick(DT);
        if let Some(&(_, _, _, _, t)) = mit_frames(&bus.tx_log(), 4).first() {
            assert!((t - prev).abs() <= 15.0 * DT as f32 + 0.005 + 1e-4, "tau_ff jumped {prev} -> {t} before the trip");
            prev = t;
        }
        if inner.collision.hold.is_some() { tripped_at = Some(k); break; }
    }
    assert!(tripped_at.is_some(), "no collision trip");
    let g4 = inner.config.arm_model.gravity_torques(&inner.measured_positions[..6]).unwrap()[3];
    let jt = mit_config_j4().joint_tracking.unwrap().joints[3].clone();
    for _ in 0..20 {
        bus.clear_log();
        inner.tick(DT);
        let Some(&(_, v, kp, _, t)) = mit_frames(&bus.tx_log(), 4).first() else { continue };
        assert!(v.abs() < 0.02 && (kp - jt.hold_kp.unwrap()).abs() < 0.13, "not a stationary hold: v {v} kp {kp}");
        assert!((t - prev).abs() <= 15.0 * DT as f32 + 0.005 + 1e-4, "tau_ff jumped {prev} -> {t}");
        prev = t;
    }
    assert!((prev - g4.clamp(-3.0, 3.0)).abs() < 0.05, "dynamic terms not removed: {prev} vs gravity {g4}");
    assert!((moving_tau - prev).abs() > 0.3, "the test did not exercise the removal ({moving_tau} -> {prev})");
}

/// Review #10 (real path): a torque-guard hold on J4 (measured torque above 60 % of TMAX) keeps
/// the decoded total effort of every J4 frame out of the held direction.
#[test]
fn mit_torque_guard_hold_never_pushes_into_the_held_direction() {
    let mut cfg = mit_config_j4();
    { let j4 = &mut cfg.joint_tracking.as_mut().unwrap().joints[3]; j4.gravity_scale = 0.0; }
    let (mut inner, bus) = active_rig(cfg, 0.0);
    bus.with(|s| s.motors[3].tau = 7.0);
    let mut held = false;
    for _ in 0..30 {
        bus.clear_log();
        inner.tick(DT);
        if let Some(h) = inner.torque_guard.hold.filter(|h| h.joint == 3) {
            held = true;
            let (q_m, v_m) = (inner.measured_positions[3], inner.measured_velocities[3]);
            for (p, v, kp, kd, t) in mit_frames(&bus.tx_log(), 4) {
                let total = kp * (p - q_m) + kd * (v - v_m) + t;
                assert!(h.sign * total <= 1e-5, "effort {total} into the held direction {}", h.sign);
            }
        }
    }
    assert!(held, "no torque-guard hold on J4");
}

/// Review #11: jog, home, move_to_pose-style quintics and a live policy start are refused while
/// MIT is requested but does not own its joints (handover pending or failed).
#[test]
fn mit_jog_home_and_live_policy_check_the_mit_precondition() {
    let bus = FakeBus::new(&b601_limits());
    bus.with(|s| for (i, m) in s.motors.iter_mut().enumerate() { m.q = Q0[i]; });
    let mut d = disabled_driver(&bus);
    d.check_mapping_ranges();
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(DT);
    inner.mit_timeout_command(&json!({"joints": [4]})).unwrap();
    with_record(&mut inner, record_j4(2000));
    inner.set_torque(true).unwrap();
    assert!(!inner.mit_active(), "no tick yet: handover pending");
    let epoch = inner.safety.epoch_id();
    let mut target = inner.commanded_positions.clone();
    target[3] += 0.05;
    let e = inner.jog_action(target, None, Some(epoch), "joint_move").unwrap_err();
    assert!(e.to_string().contains("MIT tracking is not active"), "jog: {e}");
    let home = inner.config.home_positions.clone();
    let e = inner.jog_action(home, None, Some(epoch), "home").unwrap_err();
    assert!(e.to_string().contains("MIT tracking is not active"), "home: {e}");
    #[cfg(feature = "policy-runtime")]
    {
        let e = inner.check_policy_start_preconditions_with(armd::policy_runtime::PolicyMode::Live, false, true, epoch).unwrap_err();
        assert!(e.to_string().contains("MIT tracking is not active"), "live policy: {e}");
        // shadow mode moves nothing
        let shadow = inner.check_policy_start_preconditions_with(armd::policy_runtime::PolicyMode::Shadow, false, true, epoch);
        assert!(shadow.as_ref().err().is_none_or(|e| !e.to_string().contains("MIT")), "{shadow:?}");
    }
    inner.tick(DT);
    inner.tick(DT);
    assert!(inner.mit_active());
    let mut target = inner.commanded_positions.clone();
    target[3] += 0.05;
    let epoch = inner.safety.epoch_id();
    inner.jog_action(target, None, Some(epoch), "joint_move").expect("jog once MIT owns J4");
}

/// Review #12: a park requested while MIT moves the arm brakes to rest first, hands back through
/// the confirmed stationary switch, and only then installs the park trajectory (also for the
/// lease-expiry park, which takes the same NormalUnload path).
#[test]
fn mit_moving_park_brakes_hands_back_then_parks() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    let inp = inner.plan_input().unwrap();
    let mut goal = [0.0_f64; 6];
    for i in 0..6 { goal[i] = f64::from(inner.commanded_positions[i]); }
    goal[3] -= 0.3;
    inner.install_joint_goal(&inp, &goal, 0.5, "joint_move", None, None).unwrap();
    for _ in 0..15 { inner.tick(DT); }
    assert!(inner.commanded_velocities[3].abs() > 0.01, "moving");
    bus.clear_log();
    inner.apply_stop_policy(armd::StopPolicy::NormalUnload);
    assert!(inner.active_trajectory.is_none() && !inner.pending_normal_unload, "park installed while moving");
    let mut handed_back_tick = None;
    let mut park_tick = None;
    let mut prev_v = inner.commanded_velocities[3].abs();
    for k in 0..200 {
        let before = bus.tx_log().len();
        inner.tick(DT);
        let log = bus.tx_log();
        let new = &log[before..];
        if handed_back_tick.is_none() {
            let back = new.iter().position(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_POS_VEL).data);
            let before_back = &new[..back.unwrap_or(new.len())];
            assert!(posvel_frames(before_back, 4).is_empty(), "POS_VEL on J4 before the hand back (tick {k})");
            if back.is_some() {
                handed_back_tick = Some(k);
                assert!(prev_v < 0.05, "hand back while still moving ({prev_v} rad/s commanded the tick before)");
            } else {
                assert!(inner.commanded_velocities[3].abs() <= prev_v + 1e-6, "not braking at tick {k}");
                assert!(!inner.pending_normal_unload, "park started before the hand back (tick {k})");
                prev_v = inner.commanded_velocities[3].abs();
            }
        }
        if park_tick.is_none() && inner.pending_normal_unload { park_tick = Some(k); }
        if park_tick.is_some() && handed_back_tick.is_some() { break; }
    }
    let (h, p) = (handed_back_tick.expect("hand back"), park_tick.expect("park installed"));
    assert!(h <= p, "park ({p}) before the hand back ({h})");
    assert!(!inner.mit_active());
}

/// Review #13: a batch TIMEOUT write that fails part way reports per joint what was applied,
/// which joint is uncertain (write sent, not confirmed) and which were not attempted.
#[test]
fn mit_timeout_batch_failure_reports_applied_and_uncertain_joints() {
    let bus = FakeBus::new(&b601_limits());
    let d = disabled_driver(&bus);
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(0.02);
    bus.with(|s| s.motors[4].ignore_reg_writes = true); // J5 acknowledges, readback disagrees
    let e = inner.mit_timeout_command(&json!({"joints": [4, 5, 6]})).unwrap_err();
    assert_eq!(e.code, "MIT_TIMEOUT_FAILED");
    let f = e.reply_fields();
    assert_eq!(f["applied"], json!([4]), "{f:?}");
    assert_eq!(f["uncertain"], json!([5]), "{f:?}");
    assert_eq!(f["not_attempted"], json!([6]), "{f:?}");
    assert_eq!(bus.with(|s| s.motors[3].reg_u32(damiao::RID_TIMEOUT)), 2000);
    assert_eq!(bus.with(|s| s.motors[5].reg_u32(damiao::RID_TIMEOUT)), 0);
    // a failure before any write: nothing applied, nothing uncertain
    let bus = FakeBus::new(&b601_limits());
    let d = disabled_driver(&bus);
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(0.02);
    bus.with(|s| s.motors[3].silent = true);
    let e = inner.mit_timeout_command(&json!({"joints": [4, 5]})).unwrap_err();
    let f = e.reply_fields();
    assert_eq!((f["applied"].clone(), f["uncertain"].clone(), f["not_attempted"].clone()), (json!([]), json!([]), json!([5])), "{f:?}");
}

/// Review #14: a feedback age or output gap close to (but under) the 45 ms deadline is counted
/// and logged as a near miss; the group stays active.
#[test]
fn mit_near_misses_are_counted_without_cancelling() {
    for attempt in 0..3 {
        let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
        for _ in 0..3 { inner.tick(DT); }
        bus.with(|s| s.motors[3].silent = true);
        std::thread::sleep(std::time::Duration::from_millis(38));
        bus.with(|s| s.motors[3].silent = false);
        inner.tick(DT);
        if inner.mit.owner == mit::MitOwner::Lost { eprintln!("attempt {attempt}: scheduler crossed 45 ms, retry"); continue; }
        assert!(inner.mit_active());
        assert!(inner.mit.near_misses >= 1, "near miss not counted");
        return;
    }
    panic!("could not stay under 45 ms in three attempts");
}

/// Review round 3, #6: the fence is armed before the first MIT-capable switch frame. A handover
/// whose mode acknowledgement is lost (the motor did switch) waits past 45 ms; the fence trips and
/// the rollback sends no further MIT frame (only the original hold went out).
#[test]
fn mit_handover_is_fenced_from_the_first_switch_frame() {
    let bus = FakeBus::new(&b601_limits());
    bus.with(|s| for (i, m) in s.motors.iter_mut().enumerate() { m.q = Q0[i]; });
    let mut d = disabled_driver(&bus);
    d.check_mapping_ranges();
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(DT);
    inner.mit_timeout_command(&json!({"joints": [4]})).unwrap();
    with_record(&mut inner, record_j4(2000));
    inner.set_torque(true).unwrap();
    bus.with(|s| s.motors[3].no_write_ack = true); // the switch happens, its ACK is lost
    bus.clear_log();
    inner.tick(DT);
    inner.tick(DT);
    assert!(!inner.mit_active());
    let log = bus.tx_log();
    let w = log.iter().position(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_MIT).data).expect("switch to MIT");
    let mit_after = mit_frames(&log[w..], 4).len();
    assert_eq!(mit_after, 1, "MIT frames after the switch: only the hold; the rollback after the 45 ms trip must send none ({mit_after})");
    assert_eq!(inner.mit.owner, mit::MitOwner::Lost, "mode uncertain after an unconfirmed rollback");
}

/// Review round 3, #8: the handover's first MIT frame (the hold) obeys the effort bound on the
/// decoded wire fields too. The joint creeps at a stationary-allowed velocity and gravity drives
/// tau_ff to the bound; the limit is chosen so the bound lands exactly on the torque grid, where
/// an undecoded kd would push the decoded total past it.
#[test]
fn mit_handover_hold_frame_obeys_the_decoded_effort_bound() {
    let l4 = b601_limits()[3];
    // a pose where gravity loads J4 positively (the 12-bit velocity field's zero is +0.0073 rad/s
    // on the wire, which adds to pd on that side)
    let mut qh = Q0;
    qh[3] = -1.435;
    let g = ArmConfig::rebot_dm().arm_model.gravity_torques(&qh[..6]).unwrap()[3];
    let sign = g.signum();
    let grid = |x: f32, lo: f32, hi: f32, bits: u32| damiao::uint_to_float(damiao::float_to_uint(x, lo, hi, bits), lo, hi, bits);
    let v_m = grid(-sign * 0.04, l4.v_min, l4.v_max, 12); // pd = -kd v_m pushes toward g's side
    let mut worst = f32::MIN;
    for k in 0..6 {
        let t_grid = grid(sign * (0.15 + 0.013 * k as f32), l4.t_min, l4.t_max, 12);
        let hold_kd = 0.4_f32;
        let l = t_grid.abs() + hold_kd * v_m.abs(); // the clamp bound L - pd equals the grid point
        if l >= g.abs() { continue; }
        let mut cfg = mit_config_j4();
        { let j4 = &mut cfg.joint_tracking.as_mut().unwrap().joints[3]; j4.effort_limit_nm = l; j4.hold_kd = Some(hold_kd); }
        let bus = FakeBus::new(&b601_limits());
        bus.with(|s| { s.follow_commands = false; for (i, m) in s.motors.iter_mut().enumerate() { m.q = qh[i]; } s.motors[3].v = v_m; });
        let mut d = disabled_driver(&bus);
        d.check_mapping_ranges();
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(d));
        inner.lease_enabled = false;
        inner.tick(DT);
        inner.mit_timeout_command(&json!({"joints": [4]})).unwrap();
        with_record(&mut inner, record_j4(2000));
        inner.set_torque(true).unwrap();
        bus.clear_log();
        for _ in 0..3 { bus.with(|s| s.push_all_feedback()); inner.tick(DT); }
        assert!(inner.mit_active(), "{:?}", inner.mit.handover_error);
        let log = bus.tx_log();
        let w = log.iter().position(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_MIT).data).unwrap();
        let (p, v, kp, kd, t) = mit_frames(&log[w..], 4)[0];
        let (q_m, vm) = (inner.measured_positions[3], inner.measured_velocities[3]);
        let total = kp * (p - q_m) + kd * (v - vm) + t;
        worst = worst.max(total.abs() - l);
    }
    assert!(worst > f32::MIN, "no case exercised");
    assert!(worst <= 1e-6, "handover hold frame exceeds the effort limit by {worst} N·m on the decoded fields");
}

/// Review round 3, #12: a park requested while MIT moves the arm gets an action id at once and
/// keeps it through the deferral; a STOP during the brake cancels the deferred park (no park is
/// launched after settling) and ends that action as cancelled.
#[test]
fn mit_deferred_park_has_an_action_and_stop_cancels_it() {
    let moving = |inner: &mut SupervisorInner| {
        let inp = inner.plan_input().unwrap();
        let mut goal = [0.0_f64; 6];
        for i in 0..6 { goal[i] = f64::from(inner.commanded_positions[i]); }
        goal[3] -= 0.3;
        inner.install_joint_goal(&inp, &goal, 0.5, "joint_move", None, None).unwrap();
        for _ in 0..15 { inner.tick(DT); }
    };
    // park deferred, then STOP
    let (mut inner, _bus) = active_rig(mit_config_j4(), 0.0);
    moving(&mut inner);
    let id = inner.unload_action().unwrap().expect("the deferred park has an action id");
    inner.stop_motion();
    for _ in 0..60 { inner.tick(DT); assert!(!inner.pending_normal_unload, "park launched after a STOP"); }
    let a = inner.current_observation().action.unwrap();
    assert_eq!((a.id, a.state.as_str(), a.code.as_deref()), (id, "cancelled", Some("STOPPED")), "{a:?}");
    // park deferred, no STOP: the same action carries the park
    let (mut inner, _bus) = active_rig(mit_config_j4(), 0.0);
    moving(&mut inner);
    let id = inner.unload_action().unwrap().expect("action id");
    let mut parked = false;
    for _ in 0..30 { inner.tick(DT); if inner.pending_normal_unload { parked = true; break; } }
    assert!(parked);
    let a = inner.current_observation().action.unwrap();
    assert_eq!((a.id, a.state.as_str()), (id, "running"), "{a:?}");
}

/// Review round 3, #3: a rearm requested while MIT moves the arm is refused; the authorization is
/// revoked, the motion brakes to rest (no new motion is accepted meanwhile) and MIT hands back at
/// rest, so MIT output does not continue after the failed rearm.
#[test]
fn mit_rearm_while_moving_brakes_refuses_motion_and_hands_back() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    let inp = inner.plan_input().unwrap();
    let mut goal = [0.0_f64; 6];
    for i in 0..6 { goal[i] = f64::from(inner.commanded_positions[i]); }
    goal[3] -= 0.3;
    inner.install_joint_goal(&inp, &goal, 0.5, "joint_move", None, None).unwrap();
    for _ in 0..15 { inner.tick(DT); }
    assert!(inner.commanded_velocities[3].abs() > 0.01);
    assert!(inner.set_torque(true).is_err(), "rearm accepted while MIT moves the arm");
    assert!(inner.active_trajectory.is_none(), "the motion goes on after the refused rearm");
    assert!(inner.plan_input().is_err(), "motion accepted after the refused rearm");
    bus.clear_log();
    for _ in 0..10 { inner.tick(DT); if !inner.mit_active() { break; } }
    assert!(!inner.mit_active(), "MIT still owns J4");
    assert!(bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data == damiao::build_set_control_mode_frame(4, damiao::CTRL_MODE_POS_VEL).data), "no hand back");
    for _ in 0..5 { inner.tick(DT); }
    assert!(!inner.mit_active(), "re-activated without a passed gate");
}

/// Review round 3, #9: a deadline trip during a hand back (rollback then unconfirmed) is a loss:
/// BUS_LOSS is latched, not a hardware fault; a trip seen after the group was already cancelled
/// for another reason still latches BUS_LOSS.
#[test]
fn mit_trip_during_hand_back_latches_bus_loss() {
    let (mut inner, bus) = active_rig(mit_config_j4(), 0.0);
    bus.with(|s| s.motors[3].silent_after_mode_ack = true); // ACK, then nothing: J4 goes stale
    inner.apply_stop_policy(armd::StopPolicy::NormalUnload);
    inner.tick(DT);
    assert_eq!(inner.mit.owner, mit::MitOwner::Lost);
    assert_eq!(inner.safety.fault().map(|f| f.code), Some(armd::FaultCode::BusLoss), "{:?}", inner.safety.fault());
}

/// Review round 3, #13: a TIMEOUT write that never left the transport (WouldBlock until the
/// retries ran out) is not reported as sent-but-unconfirmed.
#[test]
fn mit_timeout_write_never_sent_is_not_uncertain() {
    let bus = FakeBus::new(&b601_limits());
    let d = disabled_driver(&bus);
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(0.02);
    bus.with(|s| s.stall_reg_writes = true);
    let e = inner.mit_timeout_command(&json!({"joints": [4, 5]})).unwrap_err();
    let f = e.reply_fields();
    assert_eq!((f["applied"].clone(), f["uncertain"].clone(), f["not_attempted"].clone()), (json!([]), json!([]), json!([5])), "{f:?}");
    assert_eq!(bus.with(|s| s.motors[3].reg_u32(damiao::RID_TIMEOUT)), 0);
}
