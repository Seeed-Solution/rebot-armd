//! Structured `fault` field in the observation (code, message, recoverable).

use armd::{ArmConfig, ArmSupervisor, FaultCode, FaultReason, SafetyState};

fn sim() -> ArmSupervisor {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.joint_names.truncate(2);
    config.limits.truncate(2);
    config.home_positions = vec![0.0; 2];
    ArmSupervisor::new(config)
}

#[tokio::test]
async fn fault_is_null_while_healthy_and_serialized_as_null() {
    let supervisor = sim();
    let obs = supervisor.tick(0.02).await;
    assert!(obs.fault.is_none());
    let json = serde_json::to_value(&obs).unwrap();
    assert!(json.as_object().unwrap().contains_key("fault"));
    assert!(json["fault"].is_null());
}

#[tokio::test]
async fn control_deadline_is_recoverable_and_clear_resets_it() {
    let supervisor = sim();
    supervisor.set_lease_enabled(false).await;
    supervisor.set_torque(true).await.unwrap();
    supervisor.tick(0.02).await;
    let obs = supervisor.tick(0.3).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::Other(_))), "{:?}", obs.safety_state);
    let json = serde_json::to_value(&obs).unwrap();
    assert_eq!(json["fault"]["code"], "CONTROL_DEADLINE");
    assert_eq!(json["fault"]["recoverable"], true);
    assert_eq!(json["fault"]["message"], "Control loop deadline exceeded");
    // safety_state keeps its shape.
    assert_eq!(json["safety_state"]["Fault"]["Other"], "Control loop deadline exceeded");
    supervisor.clear_fault().await.unwrap();
    assert!(supervisor.tick(0.02).await.fault.is_none());
}

#[tokio::test]
async fn estop_and_typed_reasons_map_to_codes() {
    let supervisor = sim();
    supervisor.trigger_e_stop().await;
    let fault = supervisor.tick(0.02).await.fault.unwrap();
    assert_eq!(fault.code, FaultCode::Estop);
    assert!(!fault.recoverable);

    let supervisor = sim();
    supervisor.inner().write().await.safety.trigger_fault(FaultReason::ThermalTrip { joint_idx: 1, temperature: 80.0, limit: 70.0 });
    let fault = supervisor.get_observation().await.fault.unwrap();
    assert_eq!(fault.code, FaultCode::ThermalTrip);
    assert!(!fault.recoverable);
    assert!(fault.message.contains("Joint 2"), "{}", fault.message);

    supervisor.inner().write().await.safety.trigger_fault(FaultReason::BusLoss);
    assert_eq!(supervisor.get_observation().await.fault.unwrap().code, FaultCode::BusLoss);
}

#[test]
fn fault_code_all_lists_every_variant_once() {
    let names: Vec<String> = FaultCode::ALL.iter().map(|c| serde_json::to_value(c).unwrap().as_str().unwrap().to_string()).collect();
    assert_eq!(names, [
        "ESTOP", "BUS_LOSS", "THERMAL_TRIP", "JOINT_LIMIT", "WATCHDOG_TIMEOUT", "FOLLOWING_ERROR", "CONTROL_DEADLINE",
        "INVALID_TELEMETRY", "DRIVER_WRITE_FAILED", "HARDWARE_FAULT", "ENABLE_FAILED", "SERVO_NO_BRAKE",
        "HANDOVER_UNSETTLED", "GRAVITY_COMP_FAILED", "COLLISION", "OTHER",
    ]);
}
