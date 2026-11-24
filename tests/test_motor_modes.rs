//! Tests for motor control-mode abstraction (Step 1):
//! 1. Static vendor capability table (`VENDOR_CAPABILITIES`) and `needs-vendor-doc` accounting.
//! 2. Loading EVERY shipped profile (`kits/*/profile/*.json`, `profiles/*.json`) and asserting
//!    resolved modes, typed limits, and runtime transition permissions match existing behaviour.
//! 3. `damiao_gripper` compatibility reader and omitted-field translation (including legacy POS_VEL).
//! 4. Load-time validation rejections: conflicts, unsupported vendor bindings, missing
//!    mode-specific limits, and invalid wire ranges.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use armd::model::{
    ArmConfig, DamiaoGripperDrive, DamiaoRange, ImpedanceLimits, JointControlModeConfig,
    ModeLimits, PositionForceLimits, PositionLimits, PositionVelocityLimits, VendorBinding,
    VENDOR_CAPABILITIES,
};
use armd::motor_mode::{
    ControlMode, DAMIAO_DEFAULT_POS_VEL_RAD_S, FASHIONSTAR_DEFAULT_INTERVAL_MS,
    FASHIONSTAR_DEFAULT_POWER, FeedbackSource, ModeSelectionMechanism,
};
use armd::tracking::{JointTrackingConfig, TrackingStrategy};

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn collect_shipped_profiles() -> Vec<PathBuf> {
    let root = repo_root();
    let mut paths = Vec::new();
    for entry in std::fs::read_dir(root.join("profiles")).unwrap() {
        let p = entry.unwrap().path();
        if p.extension().is_some_and(|e| e == "json") {
            paths.push(p);
        }
    }
    for kit in std::fs::read_dir(root.join("kits")).unwrap() {
        let profile_dir = kit.unwrap().path().join("profile");
        if !profile_dir.is_dir() {
            continue;
        }
        for entry in std::fs::read_dir(profile_dir).unwrap() {
            let p = entry.unwrap().path();
            if p.extension().is_some_and(|e| e == "json") {
                paths.push(p);
            }
        }
    }
    paths.sort();
    paths
}

fn rel_path(p: &Path) -> String {
    p.strip_prefix(repo_root())
        .unwrap()
        .to_string_lossy()
        .replace('\\', "/")
}

// =========================================================================
// 1. Vendor capability table
// =========================================================================

#[test]
fn vendor_capability_table_matches_protocol_implementations() {
    assert_eq!(VENDOR_CAPABILITIES.len(), 4);

    let dm = VendorBinding::Damiao.capability();
    assert_eq!(dm.vendor, VendorBinding::Damiao);
    assert_eq!(
        dm.supported_modes,
        &[
            ControlMode::PositionVelocity,
            ControlMode::Impedance,
            ControlMode::PositionForce,
        ]
    );
    assert_eq!(dm.default_base_mode, ControlMode::PositionVelocity);
    assert!(dm.supports_runtime_mode_switch);
    assert!(VendorBinding::Damiao.supports_runtime_switch(
        ControlMode::PositionVelocity,
        ControlMode::Impedance
    ));
    assert!(VendorBinding::Damiao.supports_runtime_switch(
        ControlMode::Impedance,
        ControlMode::PositionVelocity
    ));
    assert!(!VendorBinding::Damiao.supports_runtime_switch(
        ControlMode::PositionForce,
        ControlMode::Impedance
    ));
    assert!(!VendorBinding::Damiao.supports_mode(ControlMode::Position));
    assert_eq!(dm.mode_selection, ModeSelectionMechanism::RegisterWriteAndReadback);
    assert_eq!(dm.feedback_source, FeedbackSource::CommandReplyAndRefresh);
    assert!(dm.feedback_position && dm.feedback_velocity && dm.feedback_torque && dm.feedback_temperature && dm.feedback_enable_state);
    assert!(!dm.feedback_load);
    assert_eq!(dm.default_feedback_max_age_ms, Some(100));
    assert!(!dm.needs_vendor_doc.is_empty());

    let ft = VendorBinding::Feetech.capability();
    assert_eq!(ft.supported_modes, &[ControlMode::Position]);
    assert_eq!(ft.default_base_mode, ControlMode::Position);
    assert!(!ft.supports_runtime_mode_switch);
    assert!(ft.supported_runtime_switches.is_empty());
    assert_eq!(ft.mode_selection, ModeSelectionMechanism::ProvisioningRegisterReadback);
    assert_eq!(ft.feedback_source, FeedbackSource::SyncReadPoll);
    assert!(ft.feedback_position && ft.feedback_velocity && !ft.feedback_torque && ft.feedback_load && ft.feedback_temperature && ft.feedback_enable_state);
    assert!(!ft.needs_vendor_doc.is_empty());

    let rs = VendorBinding::RobStride.capability();
    assert_eq!(rs.supported_modes, &[ControlMode::Impedance]);
    assert_eq!(rs.default_base_mode, ControlMode::Impedance);
    assert!(!rs.supports_runtime_mode_switch);
    assert_eq!(rs.mode_selection, ModeSelectionMechanism::FixedMotionControlType);
    assert_eq!(rs.feedback_source, FeedbackSource::CanFeedbackDrain);
    assert!(rs.feedback_position && rs.feedback_velocity && rs.feedback_torque && !rs.feedback_load && rs.feedback_temperature && !rs.feedback_enable_state);
    assert!(!rs.needs_vendor_doc.is_empty());

    let fs = VendorBinding::FashionStar.capability();
    assert_eq!(fs.supported_modes, &[ControlMode::Position]);
    assert_eq!(fs.default_base_mode, ControlMode::Position);
    assert!(!fs.supports_runtime_mode_switch);
    assert_eq!(fs.mode_selection, ModeSelectionMechanism::ImplicitAngleCommand);
    assert_eq!(fs.feedback_source, FeedbackSource::ExplicitAngleQuery);
    assert!(fs.feedback_position && !fs.feedback_velocity && !fs.feedback_torque && !fs.feedback_load && !fs.feedback_temperature && !fs.feedback_enable_state);
    assert_eq!(fs.default_feedback_max_age_ms, None);
    assert!(!fs.needs_vendor_doc.is_empty());

    assert_eq!(VendorBinding::from_follower_name("rebot-dm"), Some(VendorBinding::Damiao));
    assert_eq!(VendorBinding::from_follower_name("rebot-rs"), Some(VendorBinding::RobStride));
    assert_eq!(VendorBinding::from_follower_name("so-arm101"), Some(VendorBinding::Feetech));
    assert_eq!(VendorBinding::from_follower_name("so-arm100"), Some(VendorBinding::Feetech));
    assert_eq!(VendorBinding::from_follower_name("fashionstar"), Some(VendorBinding::FashionStar));
    assert_eq!(VendorBinding::from_follower_name("star-arm102"), Some(VendorBinding::FashionStar));
    assert_eq!(VendorBinding::from_follower_name("mock"), None);
    assert_eq!(VendorBinding::from_follower_name("sim"), None);
}

// =========================================================================
// 2. Every shipped profile loads and resolves to today's exact behaviour
// =========================================================================

#[test]
fn every_shipped_profile_loads_and_resolves_expected_modes() {
    let paths = collect_shipped_profiles();
    let found: BTreeSet<String> = paths.iter().map(|p| rel_path(p)).collect();
    let expected: BTreeSet<String> = [
        "kits/_template/profile/example.json",
        "kits/_template/profile/example_urdf.json",
        "kits/recamera-b601dm/profile/rebot-b601dm.json",
        "profiles/rebot_b601_rs.json",
        "profiles/so_arm100.json",
        "profiles/so_arm101.json",
    ]
    .into_iter()
    .map(String::from)
    .collect();
    assert_eq!(
        found, expected,
        "shipped profile inventory changed; update mode assertions for any added/removed profile"
    );

    for path in paths {
        let rel = rel_path(&path);
        match rel.as_str() {
            "kits/recamera-b601dm/profile/rebot-b601dm.json" => {
                let mut cfg = ArmConfig::from_profile_file(&path).unwrap();
                cfg.validate().unwrap();
                assert_eq!(cfg.infer_vendor_binding(), Some(VendorBinding::Damiao));
                assert_eq!(cfg.effective_damiao_gripper(), Some(DamiaoGripperDrive::rebot_dm()));

                let resolved = cfg.resolve_control_modes_for_follower("rebot-dm").unwrap();
                assert_eq!(resolved.len(), 7);
                for (i, jm) in resolved[..6].iter().enumerate() {
                    assert_eq!(jm.joint, i);
                    assert_eq!(jm.vendor, Some(VendorBinding::Damiao));
                    assert_eq!(jm.mode, ControlMode::PositionVelocity);
                    assert_eq!(
                        jm.limits,
                        ModeLimits::PositionVelocity(PositionVelocityLimits {
                            max_velocity_rad_s: DAMIAO_DEFAULT_POS_VEL_RAD_S,
                        })
                    );
                    assert_eq!(jm.runtime_modes, vec![ControlMode::Impedance]);
                }
                let j7 = &resolved[6];
                assert_eq!(j7.joint, 6);
                assert_eq!(j7.vendor, Some(VendorBinding::Damiao));
                assert_eq!(j7.mode, ControlMode::PositionForce);
                assert_eq!(
                    j7.limits,
                    ModeLimits::PositionForce(PositionForceLimits {
                        current_ceiling_raw: 460,
                        close_speed_rad_s: 0.30,
                        max_speed_rad_s: None,
                        calibrated: false,
                        ceiling_torque_nm: None,
                        torque_reference: None,
                        feedback_stale_fault_ms: None,
                    })
                );
                assert!(j7.runtime_modes.is_empty());

                // Virtual followers (`sim`, `mock`) infer Damiao and resolve identically.
                assert_eq!(cfg.resolve_control_modes_for_follower("sim").unwrap(), resolved);
                assert_eq!(cfg.resolve_control_modes_for_follower("mock").unwrap(), resolved);
            }
            "profiles/rebot_b601_rs.json" => {
                let mut cfg = ArmConfig::from_profile_file(&path).unwrap();
                cfg.validate().unwrap();
                assert_eq!(cfg.infer_vendor_binding(), Some(VendorBinding::RobStride));

                let resolved = cfg.resolve_control_modes_for_follower("rebot-rs").unwrap();
                assert_eq!(resolved.len(), 6);
                for (i, jm) in resolved.iter().enumerate() {
                    assert_eq!(jm.joint, i);
                    assert_eq!(jm.vendor, Some(VendorBinding::RobStride));
                    assert_eq!(jm.mode, ControlMode::Impedance);
                    assert_eq!(
                        jm.limits,
                        ModeLimits::Impedance(ImpedanceLimits::robstride_default())
                    );
                    assert!(jm.runtime_modes.is_empty());
                }
                assert_eq!(cfg.resolve_control_modes_for_follower("mock").unwrap(), resolved);
            }
            "profiles/so_arm100.json" | "profiles/so_arm101.json" => {
                let mut cfg = ArmConfig::from_profile_file(&path).unwrap();
                cfg.validate().unwrap();
                assert_eq!(cfg.infer_vendor_binding(), Some(VendorBinding::Feetech));

                let follower = if rel.ends_with("so_arm100.json") {
                    "so-arm100"
                } else {
                    "so-arm101"
                };
                let resolved = cfg.resolve_control_modes_for_follower(follower).unwrap();
                assert_eq!(resolved.len(), 6);
                for (i, jm) in resolved.iter().enumerate() {
                    assert_eq!(jm.joint, i);
                    assert_eq!(jm.vendor, Some(VendorBinding::Feetech));
                    assert_eq!(jm.mode, ControlMode::Position);
                    assert_eq!(
                        jm.limits,
                        ModeLimits::Position(PositionLimits {
                            max_velocity_rad_s: None,
                            step_range: Some([0, 4095]),
                            interval_ms: None,
                            power_limit: None,
                        })
                    );
                    assert!(jm.runtime_modes.is_empty());
                }
                assert_eq!(cfg.resolve_control_modes_for_follower("mock").unwrap(), resolved);
            }
            "kits/_template/profile/example.json" => {
                let mut cfg = ArmConfig::from_profile_file(&path).unwrap();
                cfg.validate().unwrap();
                assert_eq!(cfg.infer_vendor_binding(), None);

                let dm = cfg.resolve_control_modes_for_follower("rebot-dm").unwrap();
                assert_eq!(dm.len(), 6);
                assert!(dm.iter().all(|j| j.mode == ControlMode::PositionVelocity && j.runtime_modes.is_empty()));

                let rs = cfg.resolve_control_modes_for_follower("rebot-rs").unwrap();
                assert!(rs.iter().all(|j| j.mode == ControlMode::Impedance && j.runtime_modes.is_empty()));

                let ft = cfg.resolve_control_modes_for_follower("so-arm101").unwrap();
                assert!(ft.iter().all(|j| j.mode == ControlMode::Position && j.runtime_modes.is_empty()));

                let fs = cfg.resolve_control_modes_for_follower("fashionstar").unwrap();
                assert!(fs.iter().all(|j| {
                    j.mode == ControlMode::Position
                        && j.limits
                            == ModeLimits::Position(PositionLimits {
                                max_velocity_rad_s: None,
                                step_range: None,
                                interval_ms: Some(FASHIONSTAR_DEFAULT_INTERVAL_MS),
                                power_limit: Some(FASHIONSTAR_DEFAULT_POWER),
                            })
                        && j.runtime_modes.is_empty()
                }));
                assert_eq!(cfg.resolve_control_modes_for_follower("star-arm102").unwrap(), fs);

                let mock = cfg.resolve_control_modes_for_follower("mock").unwrap();
                assert!(mock.iter().all(|j| j.vendor.is_none() && j.mode == ControlMode::Position));
            }
            "kits/_template/profile/example_urdf.json" => {
                // Template references placeholder "my-arm.urdf"; swap in a real URDF just like test_arm_profile.
                let mut cfg: ArmConfig =
                    serde_json::from_reader(std::fs::File::open(&path).unwrap()).unwrap();
                cfg.urdf = Some(repo_root().join("assets/ReBot_Arm_RS.urdf").to_string_lossy().into());
                cfg.chain.as_mut().unwrap().tip_link = "gripper_end".into();
                cfg.load_arm_model(None).unwrap();
                cfg.validate().unwrap();

                let rs = cfg.resolve_control_modes(VendorBinding::RobStride).unwrap();
                assert_eq!(rs.len(), 6);
                assert!(rs.iter().all(|j| j.mode == ControlMode::Impedance));
                let fs = cfg.resolve_control_modes(VendorBinding::FashionStar).unwrap();
                assert!(fs.iter().all(|j| j.mode == ControlMode::Position));
            }
            other => panic!("unhandled shipped profile: {other}"),
        }
    }
}

// =========================================================================
// 3. Legacy Damiao profiles, runtime MIT transitions, damiao_gripper compat
// =========================================================================

#[test]
fn legacy_damiao_gripper_and_runtime_mit_translation() {
    // (a) Legacy B601-DM with `damiao_gripper = None`: J1..J6 POS_VEL + runtime MIT, J7 POS_VEL.
    let mut legacy_none = ArmConfig::rebot_dm();
    legacy_none.damiao_gripper = None;
    legacy_none.validate().unwrap();
    assert_eq!(legacy_none.effective_damiao_gripper(), None);
    let modes_none = legacy_none.resolve_control_modes(VendorBinding::Damiao).unwrap();
    for j in &modes_none[..6] {
        assert_eq!(j.mode, ControlMode::PositionVelocity);
        assert_eq!(j.runtime_modes, vec![ControlMode::Impedance]);
    }
    assert_eq!(modes_none[6].mode, ControlMode::PositionVelocity);
    assert!(modes_none[6].runtime_modes.is_empty());

    // (b) Legacy B601-DM with explicit `damiao_gripper.mode = pos_vel`: same resolved modes.
    let mut legacy_pv = ArmConfig::rebot_dm();
    legacy_pv.damiao_gripper = Some(DamiaoGripperDrive::legacy_pos_vel());
    legacy_pv.validate().unwrap();
    assert_eq!(
        legacy_pv.effective_damiao_gripper(),
        Some(DamiaoGripperDrive::legacy_pos_vel())
    );
    let modes_pv = legacy_pv.resolve_control_modes(VendorBinding::Damiao).unwrap();
    assert_eq!(modes_pv, modes_none);

    // (c) Migrated B601-DM: `damiao_gripper` omitted, J7 declared via `control_mode`.
    let mut migrated = ArmConfig::rebot_dm();
    migrated.damiao_gripper = None;
    let pf = PositionForceLimits::from_damiao_gripper(&DamiaoGripperDrive::rebot_dm());
    let mut cm = vec![None; 7];
    cm[6] = Some(JointControlModeConfig::position_force(pf.clone()));
    migrated.control_mode = Some(cm);
    migrated.validate().unwrap();
    assert_eq!(
        migrated.effective_damiao_gripper(),
        Some(DamiaoGripperDrive::rebot_dm())
    );
    let modes_mig = migrated.resolve_control_modes(VendorBinding::Damiao).unwrap();
    assert_eq!(modes_mig[6].mode, ControlMode::PositionForce);
    assert_eq!(modes_mig[6].limits, ModeLimits::PositionForce(pf));
    assert!(modes_mig[6].runtime_modes.is_empty());
    assert_eq!(modes_mig[0].runtime_modes, vec![ControlMode::Impedance]);

    // (d) Matching `damiao_gripper` + `control_mode[6]` together is accepted without conflict.
    let mut both = ArmConfig::rebot_dm();
    let mut cm_both = vec![None; 7];
    cm_both[6] = Some(JointControlModeConfig::position_force(
        PositionForceLimits::from_damiao_gripper(&DamiaoGripperDrive::rebot_dm()),
    ));
    both.control_mode = Some(cm_both);
    both.validate().unwrap();
    assert_eq!(
        both.resolve_control_modes(VendorBinding::Damiao).unwrap(),
        modes_mig
    );

    // (e) Damiao with enabled MIT joint_tracking on J2 (index 1) keeps runtime Impedance.
    let mut mit_cfg = ArmConfig::rebot_dm();
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    jt.joints[1].strategy = TrackingStrategy::Mit;
    jt.joints[1].kp = 20.0;
    jt.joints[1].kd = 0.8;
    jt.joints[1].hold_kp = Some(25.0);
    jt.joints[1].hold_kd = Some(0.9);
    jt.joints[1].effort_limit_nm = 5.0;
    mit_cfg.joint_tracking = Some(jt);
    mit_cfg.validate().unwrap();
    let modes_mit = mit_cfg.resolve_control_modes(VendorBinding::Damiao).unwrap();
    assert_eq!(modes_mit[1].mode, ControlMode::PositionVelocity);
    assert_eq!(modes_mit[1].runtime_modes, vec![ControlMode::Impedance]);

    // (f) Damiao without gravity_comp and without enabled MIT tracking has empty runtime_modes.
    let mut no_gc = ArmConfig::rebot_dm();
    no_gc.gravity_comp = None;
    no_gc.validate().unwrap();
    let modes_no_gc = no_gc.resolve_control_modes(VendorBinding::Damiao).unwrap();
    assert!(modes_no_gc.iter().all(|j| j.runtime_modes.is_empty()));
}

// =========================================================================
// 4. Load-time validation rejections (conflicts, unsupported bindings,
//    missing mode-specific limits, invalid wire ranges)
// =========================================================================

#[test]
fn json_round_trip_and_missing_or_mismatched_mode_limits_rejected() {
    // Round-trip every valid mode variant.
    let samples = [
        JointControlModeConfig::position(PositionLimits {
            max_velocity_rad_s: Some(2.0),
            step_range: Some([100, 3900]),
            interval_ms: None,
            power_limit: None,
        })
        .with_vendor(VendorBinding::Feetech),
        JointControlModeConfig::position_velocity(PositionVelocityLimits {
            max_velocity_rad_s: 2.0,
        })
        .with_vendor(VendorBinding::Damiao)
        .with_runtime_modes(vec![ControlMode::Impedance]),
        JointControlModeConfig::impedance(ImpedanceLimits {
            p_max: 12.5,
            v_max: 10.0,
            t_max: 28.0,
            kp_max: 300.0,
            kd_max: 3.0,
            effort_limit_nm: Some(12.0),
        })
        .with_vendor(VendorBinding::RobStride),
        JointControlModeConfig::position_force(PositionForceLimits {
            current_ceiling_raw: 460,
            close_speed_rad_s: 0.30,
            max_speed_rad_s: Some(2.0),
            calibrated: true,
            ceiling_torque_nm: Some(0.87),
            torque_reference: Some("feedback_equivalent".into()),
            feedback_stale_fault_ms: Some(250),
        })
        .with_vendor(VendorBinding::Damiao),
    ];
    for sample in &samples {
        let json = serde_json::to_string(sample).unwrap();
        let parsed: JointControlModeConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(&parsed, sample, "round-trip failed for {json}");
    }

    // Omitting `limits` on `position` parses and validates cleanly.
    let pos_only: JointControlModeConfig =
        serde_json::from_str(r#"{"mode":"position"}"#).unwrap();
    assert_eq!(pos_only.mode, ControlMode::Position);
    assert!(pos_only.limits.is_none());

    // Omitting `limits` on modes that require limits fails validation.
    for mode_str in ["position_velocity", "impedance", "position_force"] {
        let entry: JointControlModeConfig =
            serde_json::from_str(&format!(r#"{{"mode":"{mode_str}"}}"#)).unwrap();
        let mut cfg = ArmConfig::default();
        let mut cm = vec![None; cfg.dof];
        cm[0] = Some(entry);
        cfg.control_mode = Some(cm);
        let err = cfg.validate().unwrap_err();
        assert!(
            err.contains("missing mode-specific limits"),
            "expected missing limits error for {mode_str}, got: {err}"
        );
    }

    // Mismatched JSON limit keys for mode rejected at deserialize.
    assert!(
        serde_json::from_str::<JointControlModeConfig>(
            r#"{"mode":"position_force","limits":{"max_velocity_rad_s":2.0}}"#
        )
        .is_err()
    );
    assert!(
        serde_json::from_str::<JointControlModeConfig>(
            r#"{"mode":"impedance","limits":{"current_ceiling_raw":460,"close_speed_rad_s":0.3}}"#
        )
        .is_err()
    );

    // Rust-constructed mismatched `ModeLimits` variant rejected at `validate()`.
    let mut cfg = ArmConfig::default();
    let mut cm = vec![None; cfg.dof];
    cm[0] = Some(JointControlModeConfig {
        mode: ControlMode::Position,
        limits: Some(ModeLimits::PositionVelocity(
            PositionVelocityLimits::damiao_default(),
        )),
        vendor: None,
        runtime_modes: None,
    });
    cfg.control_mode = Some(cm);
    let err = cfg.validate().unwrap_err();
    assert!(err.contains("do not match mode"), "{err}");

    // Wrong length rejected.
    let mut bad_len = ArmConfig::default();
    bad_len.control_mode = Some(vec![None; 3]);
    let err = bad_len.validate().unwrap_err();
    assert!(err.contains("does not match DOF"), "{err}");
}

#[test]
fn unsupported_vendor_bindings_and_cross_vendor_conflicts_rejected() {
    // Damiao does not support bare Position.
    let mut cfg = ArmConfig::rebot_dm();
    let mut cm = vec![None; 7];
    cm[0] = Some(JointControlModeConfig::position(PositionLimits::default()));
    cfg.control_mode = Some(cm);
    let err = cfg.resolve_control_modes(VendorBinding::Damiao).unwrap_err();
    assert!(err.contains("does not support control mode 'position'"), "{err}");

    // Feetech rejects PositionVelocity, Impedance, PositionForce.
    for entry in [
        JointControlModeConfig::position_velocity(PositionVelocityLimits::damiao_default()),
        JointControlModeConfig::impedance(ImpedanceLimits::damiao_default()),
        JointControlModeConfig::position_force(PositionForceLimits::from_damiao_gripper(
            &DamiaoGripperDrive::rebot_dm(),
        )),
    ] {
        let mut c = ArmConfig::default();
        let mut cm = vec![None; c.dof];
        cm[0] = Some(entry);
        c.control_mode = Some(cm);
        let err = c.resolve_control_modes(VendorBinding::Feetech).unwrap_err();
        assert!(err.contains("does not support control mode"), "{err}");
    }

    // RobStride rejects Position, PositionVelocity, PositionForce.
    for entry in [
        JointControlModeConfig::position(PositionLimits::default()),
        JointControlModeConfig::position_velocity(PositionVelocityLimits::damiao_default()),
        JointControlModeConfig::position_force(PositionForceLimits::from_damiao_gripper(
            &DamiaoGripperDrive::rebot_dm(),
        )),
    ] {
        let mut c = ArmConfig::default();
        let mut cm = vec![None; c.dof];
        cm[0] = Some(entry);
        c.control_mode = Some(cm);
        let err = c.resolve_control_modes(VendorBinding::RobStride).unwrap_err();
        assert!(err.contains("does not support control mode"), "{err}");
    }

    // FashionStar rejects Impedance.
    let mut c = ArmConfig::default();
    let mut cm = vec![None; c.dof];
    cm[0] = Some(
        JointControlModeConfig::impedance(ImpedanceLimits::robstride_default())
            .with_vendor(VendorBinding::FashionStar),
    );
    c.control_mode = Some(cm);
    let err = c.validate().unwrap_err();
    assert!(err.contains("does not support control mode 'impedance'"), "{err}");

    // Damiao rejects runtime switch from PositionForce -> Impedance.
    let mut c = ArmConfig::rebot_dm();
    let mut cm = vec![None; 7];
    cm[6] = Some(
        JointControlModeConfig::position_force(PositionForceLimits::from_damiao_gripper(
            &DamiaoGripperDrive::rebot_dm(),
        ))
        .with_vendor(VendorBinding::Damiao)
        .with_runtime_modes(vec![ControlMode::Impedance]),
    );
    c.control_mode = Some(cm);
    let err = c.validate().unwrap_err();
    assert!(err.contains("does not support runtime switch"), "{err}");

    // Mixed vendors across joints in one profile rejected.
    let mut mixed = ArmConfig::default();
    let mut cm = vec![None; 6];
    cm[0] = Some(
        JointControlModeConfig::position_velocity(PositionVelocityLimits::damiao_default())
            .with_vendor(VendorBinding::Damiao),
    );
    cm[1] = Some(
        JointControlModeConfig::position(PositionLimits::default())
            .with_vendor(VendorBinding::Feetech),
    );
    mixed.control_mode = Some(cm);
    let err = mixed.validate().unwrap_err();
    assert!(err.contains("conflicts with earlier vendor"), "{err}");

    // Profile with damiao_ranges rejected when bound to RobStride / Feetech / FashionStar.
    let dm_cfg = ArmConfig::rebot_dm();
    for v in [
        VendorBinding::Feetech,
        VendorBinding::RobStride,
        VendorBinding::FashionStar,
    ] {
        let err = dm_cfg.resolve_control_modes(v).unwrap_err();
        assert!(err.contains("conflicts with damiao_ranges / damiao_gripper"), "{err}");
    }

    // Profile with feetech section rejected when bound to Damiao.
    let so101 = ArmConfig::from_profile_file(repo_root().join("profiles/so_arm101.json")).unwrap();
    let err = so101.resolve_control_modes(VendorBinding::Damiao).unwrap_err();
    assert!(err.contains("conflicts with feetech"), "{err}");
}

#[test]
fn profile_conflicts_and_invalid_wire_ranges_rejected() {
    // 1. damiao_gripper mode conflict with control_mode[6].
    let mut c = ArmConfig::rebot_dm();
    let mut cm = vec![None; 7];
    cm[6] = Some(JointControlModeConfig::position_velocity(
        PositionVelocityLimits::damiao_default(),
    ));
    c.control_mode = Some(cm);
    let err = c.validate().unwrap_err();
    assert!(err.contains("conflicts with damiao_gripper.mode"), "{err}");

    // 2. damiao_gripper parameter conflict (current_ceiling_raw differs).
    let mut c = ArmConfig::rebot_dm();
    let mut pf = PositionForceLimits::from_damiao_gripper(&DamiaoGripperDrive::rebot_dm());
    pf.current_ceiling_raw = 300;
    let mut cm = vec![None; 7];
    cm[6] = Some(JointControlModeConfig::position_force(pf));
    c.control_mode = Some(cm);
    let err = c.validate().unwrap_err();
    assert!(err.contains("conflict with damiao_gripper"), "{err}");

    // 3. damiao_ranges conflict with ImpedanceLimits wire range.
    let mut c = ArmConfig::rebot_dm();
    let mut cm = vec![None; 7];
    cm[0] = Some(JointControlModeConfig::impedance(ImpedanceLimits {
        p_max: 12.5,
        v_max: 30.0, // conflicts with REBOT_DM_J1_J3 v_max = 10.0
        t_max: 28.0,
        kp_max: 500.0,
        kd_max: 5.0,
        effort_limit_nm: None,
    }));
    c.control_mode = Some(cm);
    let err = c.validate().unwrap_err();
    assert!(err.contains("conflicts with damiao_ranges[0]"), "{err}");

    // 4. Explicit runtime_modes: [] conflicts with gravity_comp on arm joint.
    let mut c = ArmConfig::rebot_dm();
    let mut cm = vec![None; 7];
    cm[0] = Some(
        JointControlModeConfig::position_velocity(PositionVelocityLimits::damiao_default())
            .with_runtime_modes(vec![]),
    );
    c.control_mode = Some(cm);
    let err = c.validate().unwrap_err();
    assert!(err.contains("forbids runtime impedance"), "{err}");

    // 5. Invalid wire ranges: PositionVelocity exceeding damiao_ranges[0].v_max (10.0).
    let mut c = ArmConfig::rebot_dm();
    let mut cm = vec![None; 7];
    cm[0] = Some(JointControlModeConfig::position_velocity(
        PositionVelocityLimits {
            max_velocity_rad_s: 12.0,
        },
    ));
    c.control_mode = Some(cm);
    let err = c.validate().unwrap_err();
    assert!(err.contains("exceeds damiao_ranges[0].v_max"), "{err}");

    // 6. Invalid wire ranges: Impedance kp_max > 500, kd_max > 5, effort_limit > t_max, joint pos > p_max.
    for bad in [
        ImpedanceLimits { p_max: 12.5, v_max: 10.0, t_max: 28.0, kp_max: 500.1, kd_max: 5.0, effort_limit_nm: None },
        ImpedanceLimits { p_max: 12.5, v_max: 10.0, t_max: 28.0, kp_max: 500.0, kd_max: 5.1, effort_limit_nm: None },
        ImpedanceLimits { p_max: 12.5, v_max: 10.0, t_max: 28.0, kp_max: 500.0, kd_max: 5.0, effort_limit_nm: Some(29.0) },
        ImpedanceLimits { p_max: 2.0, v_max: 10.0, t_max: 28.0, kp_max: 500.0, kd_max: 5.0, effort_limit_nm: None }, // joint_1 limit +/-3.14 > 2.0
    ] {
        let mut c = ArmConfig::default();
        let mut cm = vec![None; 6];
        cm[0] = Some(JointControlModeConfig::impedance(bad));
        c.control_mode = Some(cm);
        assert!(c.validate().is_err(), "expected invalid impedance range to fail: {bad:?}");
    }

    // 7. Invalid wire ranges: PositionForce current_raw=0/10001, zero-quantizing close_speed, >655.35 rad/s, max_speed < close_speed.
    for bad in [
        PositionForceLimits { current_ceiling_raw: 0, close_speed_rad_s: 0.3, max_speed_rad_s: None, calibrated: false, ceiling_torque_nm: None, torque_reference: None, feedback_stale_fault_ms: None },
        PositionForceLimits { current_ceiling_raw: 10_001, close_speed_rad_s: 0.3, max_speed_rad_s: None, calibrated: false, ceiling_torque_nm: None, torque_reference: None, feedback_stale_fault_ms: None },
        PositionForceLimits { current_ceiling_raw: 460, close_speed_rad_s: 0.005, max_speed_rad_s: None, calibrated: false, ceiling_torque_nm: None, torque_reference: None, feedback_stale_fault_ms: None },
        PositionForceLimits { current_ceiling_raw: 460, close_speed_rad_s: 700.0, max_speed_rad_s: None, calibrated: false, ceiling_torque_nm: None, torque_reference: None, feedback_stale_fault_ms: None },
        PositionForceLimits { current_ceiling_raw: 460, close_speed_rad_s: 0.5, max_speed_rad_s: Some(0.2), calibrated: false, ceiling_torque_nm: None, torque_reference: None, feedback_stale_fault_ms: None },
        PositionForceLimits { current_ceiling_raw: 460, close_speed_rad_s: 0.3, max_speed_rad_s: None, calibrated: false, ceiling_torque_nm: None, torque_reference: None, feedback_stale_fault_ms: Some(150) },
    ] {
        let mut c = ArmConfig::rebot_dm();
        c.damiao_gripper = None;
        let mut cm = vec![None; 7];
        cm[6] = Some(JointControlModeConfig::position_force(bad.clone()));
        c.control_mode = Some(cm);
        assert!(c.validate().is_err(), "expected invalid pos-force limits to fail: {bad:?}");
    }

    // 8. Feetech invalid step_range / FashionStar-only fields rejected; FashionStar step_range rejected.
    let mut ft = ArmConfig::from_profile_file(repo_root().join("profiles/so_arm101.json")).unwrap();
    let mut cm = vec![None; 6];
    cm[0] = Some(JointControlModeConfig::position(PositionLimits {
        interval_ms: Some(20),
        ..Default::default()
    }));
    ft.control_mode = Some(cm);
    assert!(ft.validate().unwrap_err().contains("interval_ms"));

    let mut fs = ArmConfig::default();
    let mut cm = vec![None; 6];
    cm[0] = Some(JointControlModeConfig::position(PositionLimits {
        step_range: Some([0, 4095]),
        ..Default::default()
    }));
    fs.control_mode = Some(cm);
    assert!(
        fs.resolve_control_modes(VendorBinding::FashionStar)
            .unwrap_err()
            .contains("step_range")
    );

    // 9. Damiao PositionForce on gripper requires gripper_policy.
    let mut no_gp = ArmConfig::rebot_dm();
    no_gp.damiao_gripper = None;
    no_gp.gripper_policy = None;
    let mut cm = vec![None; 7];
    cm[6] = Some(JointControlModeConfig::position_force(
        PositionForceLimits::from_damiao_gripper(&DamiaoGripperDrive::rebot_dm()),
    ));
    no_gp.control_mode = Some(cm);
    assert!(no_gp.validate().unwrap_err().contains("gripper_policy"));

    // 10. Damiao joint position limits exceeding wire p_max rejected.
    let mut big_pos = ArmConfig::rebot_dm();
    big_pos.damiao_ranges.as_mut().unwrap()[0] = DamiaoRange { p_max: 2.0, v_max: 10.0, t_max: 28.0 };
    assert!(
        big_pos
            .resolve_control_modes(VendorBinding::Damiao)
            .unwrap_err()
            .contains("exceeds Damiao wire p_max")
    );
}
