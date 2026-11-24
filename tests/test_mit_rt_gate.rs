//! MIT tracking is fail-closed on the RT executor's scheduling: hardware that passes every
//! qualification check still refuses MIT when the executor did not get SCHED_FIFO or locked
//! memory (`armd::rt_core::sched`). The readiness is process-global, so this test has its
//! own binary.

mod mit_support;

use armd::diagnostics::tick::ControlThreadSched;
use armd::driver::{ArmDriver, DamiaoArmDriver};
use armd::mit::{self, MitJointQualification, MitQualificationFile};
use armd::model::ArmConfig;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::sched::{self, RtReadiness};
use armd::tracking::{JointTrackingConfig, TrackingStrategy};
use armd::SupervisorInner;
use mit_support::{FakeBus, SW_VER};

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect()
}

fn mit_config_j4() -> ArmConfig {
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

fn record_j4() -> MitQualificationFile {
    let l = b601_limits()[3];
    MitQualificationFile {
        version: mit::QUALIFICATION_VERSION,
        joints: vec![MitJointQualification {
            profile_id: "rebot_b601dm".into(), joint: 4, joint_name: ArmConfig::rebot_dm().joint_names[3].clone(),
            zero_fingerprint: mit::zero_fingerprint(0.0), motor_id: 4, reply_id: 0x14, esc_id: 4, mst_id: 0x14, sw_ver: SW_VER,
            pmax: l.p_max, vmax: l.v_max, tmax: l.t_max, timeout_raw: 2000, timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
            timeout_unit_verified: true, tested_response: "disabled".into(), measured_expiry_ms: 101.0, worst_latency_ms: 8.0,
            load_envelope: "unloaded, supported".into(), max_kp: 15.0, max_kd: 0.5, effort_ceiling_nm: 4.0,
            date: "2026-11-22T00:00:00Z".into(), trace_id: "test".into(),
        }],
        invalidated: None,
    }
}

#[test]
fn mit_is_refused_when_the_executor_is_not_real_time() {
    let bus = FakeBus::with_clock(&b601_limits(), armd::transport::MockClock::new());
    bus.with(|s| s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, 2000));
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(b601_limits()).unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(0.02);
    inner.set_calibration(Some(armd::calibration::CalibrationFile { mit_qualification: serde_json::to_value(record_j4()).ok(), ..Default::default() }));

    // the executor ran at SCHED_FIFO but the memory lock failed
    sched::set_readiness(RtReadiness::from_sched(&ControlThreadSched {
        policy: "fifo".into(), priority: 45, realtime: true, requested_priority: Some(45), cpu: None,
        memory_locked: Some(false), memory_lock: Some("failed: Cannot allocate memory".into()), errors: vec![],
    }));
    bus.with(|s| s.tx_log.clear());
    let err = { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.expect_err("MIT refused");
    assert!(err.to_string().contains("MIT_UNQUALIFIED") && err.to_string().contains("RT executor"), "{err}");
    assert!(!inner.is_torqued);
    assert_eq!(bus.tx_log().iter().filter(|f| f.data == damiao::CMD_ENABLE).count(), 0, "no enable frame");
    assert!(inner.mit_gate_error.as_deref().is_some_and(|e| e.contains("locked memory")));
}
