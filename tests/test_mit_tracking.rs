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
use armd::SupervisorInner;
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
            timeout_unit_verified: false, tested_response: "disabled".into(), measured_expiry_ms: 101.0,
            load_envelope: "unloaded, supported".into(), max_kp: 15.0, max_kd: 0.5, effort_ceiling_nm: 4.0,
            date: "2026-11-22T00:00:00Z".into(), trace_id: "test".into(),
        }],
        invalidated: None,
    }
}

fn with_record(inner: &mut SupervisorInner, rec: MitQualificationFile) {
    inner.set_calibration(Some(armd::calibration::CalibrationFile { mit_qualification: Some(rec), ..Default::default() }));
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
    assert_eq!(file.mit_qualification, Some(record_j4(2000)));
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
    let file = armd::calibration::CalibrationFile { mit_qualification: Some(record_j4(2000)), ..Default::default() };
    assert!(file.validate().is_ok(), "{:?}", file.validate());
    let text = serde_json::to_string(&file).unwrap();
    let back: armd::calibration::CalibrationFile = serde_json::from_str(&text).unwrap();
    assert_eq!(back, file);
    let mut bad = file.clone();
    bad.mit_qualification.as_mut().unwrap().joints[0].timeout_raw = 0;
    assert!(bad.validate().is_err());
    let mut bad = file;
    bad.mit_qualification.as_mut().unwrap().joints[0].effort_ceiling_nm = 99.0;
    assert!(bad.validate().is_err(), "effort ceiling above TMAX");
}
