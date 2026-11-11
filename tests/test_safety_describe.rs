//! `GET /api/v1/safety/describe` (contract section 12): shape, one entry per fault code, and
//! every value taken from the config, the runtime settings and the code's constants.
mod common;

use std::time::Duration;
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, JointState, KeepoutBox};
use armd::safety_describe::{fault_recovery, HEARTBEAT_RECOMMENDED_MS, RECOVERY_STEPS};
use armd::supervisor::{BUS_LOSS_TIMEOUT, CONTROL_DEADLINE, PREEMPT_COOLDOWN, THERMAL_CLEAR_MARGIN_C, WATCHDOG_PARK_TIMEOUT};
use armd::{ArmSupervisor, FaultCode};
use serde_json::{json, Value};

/// Driver whose feedback capabilities the test chooses.
struct CapsDriver(DriverCapabilities);

impl ArmDriver for CapsDriver {
    fn capabilities(&self) -> DriverCapabilities { self.0 }
    fn enable_state(&self) -> EnableState { EnableState::Disabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 30.0 }; 7])
    }
    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> { Ok(()) }
}

fn full_caps() -> DriverCapabilities {
    DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true }
}

async fn describe(supervisor: &ArmSupervisor) -> Value {
    supervisor.inner().read().await.safety_describe()
}

fn stop<'a>(d: &'a Value, id: &str) -> Option<&'a Value> {
    d["stops"].as_array().unwrap().iter().find(|s| s["id"] == id)
}

fn ms(d: Duration) -> u64 { d.as_millis() as u64 }

#[tokio::test]
async fn describe_route_has_the_v1_shape_without_a_lease() {
    let (addr, supervisor) = common::spawn_server(ArmConfig::rebot_dm()).await;
    supervisor.set_follower("mock").await;
    let response = reqwest::get(format!("http://{addr}/api/v1/safety/describe")).await.unwrap();
    assert_eq!(response.status(), 200);
    let d: Value = response.json().await.unwrap();

    let mut keys: Vec<&str> = d.as_object().unwrap().keys().map(String::as_str).collect();
    keys.sort_unstable();
    assert_eq!(keys, ["describe_version", "device", "faults", "hardware", "limits", "not_a_safety_function", "stops", "timing"]);
    assert_eq!(d["describe_version"], 1);
    assert_eq!(d["not_a_safety_function"], true);
    assert_eq!(d["device"]["follower"], "mock");
    assert_eq!(d["device"]["armd_version"], env!("CARGO_PKG_VERSION"));
    assert_eq!(d["device"]["dof"], 7);
    assert_eq!(d["device"]["joint_names"].as_array().unwrap().len(), 7);
    for key in ["control_hz", "lease_timeout_ms", "heartbeat_recommended_ms", "bus_loss_timeout_ms", "tick_deadline_ms", "preemption_cooldown_ms"] {
        assert!(d["timing"][key].is_u64(), "timing.{key}: {d}");
    }
    let joint = &d["limits"]["joints"][0];
    for key in ["name", "min", "max", "max_velocity", "max_acceleration"] { assert!(!joint[key].is_null(), "{key}"); }
    assert!(d["limits"]["workspace"].is_object() && d["limits"]["keepout_boxes"].is_array());
    assert!(d["limits"]["action_speed_max"]["linear_mps"].is_f64());

    let ids: Vec<&str> = d["stops"].as_array().unwrap().iter().map(|s| s["id"].as_str().unwrap()).collect();
    assert_eq!(ids, ["e_stop", "watchdog", "bus_loss", "control_deadline", "following_error", "thermal"]);
    for s in d["stops"].as_array().unwrap() {
        for key in ["trigger", "effect", "lease_required", "clear"] { assert!(!s[key].is_null(), "{}: {key}", s["id"]); }
    }
    let e_stop = stop(&d, "e_stop").unwrap();
    assert_eq!(e_stop["trigger"], json!({"ws": {"type": "estop"}, "rest": "POST /api/v1/safety/e_stop"}));
    assert_eq!(e_stop["lease_required"], false);
    assert_eq!(e_stop["effect"], "hold_position_torque_on");

    assert_eq!(d["hardware"]["physical_estop"], false);
    assert_eq!(d["hardware"]["torque_off_on_fault"], false);
    assert!(d["hardware"]["notes"].as_array().unwrap().iter().any(|n| n == "software stops only; no wired emergency stop"));
    assert!(supervisor.get_schema().await.capabilities.safety_describe);
}

#[tokio::test]
async fn faults_list_every_code_with_its_recovery() {
    let d = describe(&ArmSupervisor::new(ArmConfig::rebot_dm())).await;
    let faults = d["faults"].as_array().unwrap();
    assert_eq!(faults.len(), FaultCode::ALL.len());
    for (entry, &code) in faults.iter().zip(FaultCode::ALL) {
        assert_eq!(entry["code"], serde_json::to_value(code).unwrap());
        assert_eq!(entry["recoverable"], code.recoverable());
        assert_eq!(entry["auto_clear"], false);
        assert_eq!(entry["recovery"], json!(fault_recovery(code)));
        let steps = entry["recovery"].as_array().unwrap();
        assert!(steps.iter().any(|s| s == "clear_fault"), "{entry}");
        assert!(steps.iter().all(|s| RECOVERY_STEPS.contains(&s.as_str().unwrap())), "{entry}");
    }
    let by_code = |c: &str| faults.iter().find(|f| f["code"] == c).unwrap().clone();
    assert_eq!(by_code("WATCHDOG_TIMEOUT")["recoverable"], true);
    assert_eq!(by_code("FOLLOWING_ERROR")["recoverable"], false);
    assert_eq!(by_code("THERMAL_TRIP")["recovery"], json!(["wait_cooldown", "clear_fault", "torque_on"]));
    // Each faulting stop's clear steps are those of its fault code.
    for s in d["stops"].as_array().unwrap().iter().filter(|s| !s["fault_code"].is_null()) {
        assert_eq!(s["clear"], by_code(s["fault_code"].as_str().unwrap())["recovery"], "{}", s["id"]);
    }
}

#[tokio::test]
async fn timing_and_limits_follow_the_constants_and_the_profile() {
    let mut config = ArmConfig::rebot_dm();
    config.lease_timeout_ms = 400;
    config.max_temperature = 61.5;
    config.following_error.max_error_rad = 0.45;
    config.following_error.persistence_ms = 320;
    config.planned_max_velocity = Some(0.6);
    config.speed_cap = Some(armd::model::SpeedCap { linear_mps: Some(0.04), angular_rps: None, joint_velocity_scale: None });
    let d = describe(&ArmSupervisor::new(config.clone())).await;
    let t = &d["timing"];
    assert_eq!(t["control_hz"], config.control_frequency_hz);
    assert_eq!(t["lease_timeout_ms"], 400);
    assert_eq!(t["heartbeat_recommended_ms"], 100, "capped at a quarter of the lease timeout");
    assert_eq!(t["bus_loss_timeout_ms"], ms(BUS_LOSS_TIMEOUT));
    assert_eq!(t["tick_deadline_ms"], ms(CONTROL_DEADLINE));
    assert_eq!(t["preemption_cooldown_ms"], ms(PREEMPT_COOLDOWN));
    assert_eq!(t["following_error"], json!({"max_error_rad": 0.45f32, "persistence_ms": 320}));

    assert_eq!(stop(&d, "watchdog").unwrap()["timeout_ms"], 400);
    assert_eq!(stop(&d, "watchdog").unwrap()["park_budget"]["min_ms"], ms(WATCHDOG_PARK_TIMEOUT));
    assert_eq!(stop(&d, "bus_loss").unwrap()["timeout_ms"], ms(BUS_LOSS_TIMEOUT));
    assert_eq!(stop(&d, "control_deadline").unwrap()["deadline_ms"], ms(CONTROL_DEADLINE));
    assert_eq!(stop(&d, "following_error").unwrap()["persistence_ms"], 320);
    let thermal = stop(&d, "thermal").unwrap();
    assert_eq!(thermal["max_temperature_c"], 61.5);
    assert_eq!(thermal["clear_below_c"], f64::from(61.5 - THERMAL_CLEAR_MARGIN_C));

    let l = &d["limits"];
    assert_eq!(l["joints"].as_array().unwrap().len(), config.limits.len());
    assert_eq!(l["joints"][1]["max"], f64::from(config.limits[1].max_position));
    assert_eq!(l["planned_max_velocity"], f64::from(0.6f32));
    assert_eq!(l["action_speed_max"]["linear_mps"], f64::from(0.04f32));
    assert_eq!(l["action_speed_max"]["angular_rps"], armd::motion::MAX_ANGULAR_RPS);

    let default = describe(&ArmSupervisor::new(ArmConfig::rebot_dm())).await;
    assert_eq!(default["timing"]["heartbeat_recommended_ms"], HEARTBEAT_RECOMMENDED_MS);
}

#[tokio::test]
async fn watchdog_effect_matches_this_config() {
    // Resting pose and a driver with velocity/torque/enable feedback: park.
    let parks = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(CapsDriver(full_caps())));
    assert_eq!(stop(&describe(&parks).await, "watchdog").unwrap()["effect"], "park_then_torque_off");
    // No enable feedback: hold.
    let caps = DriverCapabilities { enable_feedback: false, ..full_caps() };
    let holds = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(CapsDriver(caps)));
    assert_eq!(stop(&describe(&holds).await, "watchdog").unwrap()["effect"], "hold_and_fault");
    // No resting pose: hold, with or without a driver.
    let no_pose = ArmConfig { resting_pose: None, ..ArmConfig::rebot_dm() };
    assert_eq!(stop(&describe(&ArmSupervisor::new(no_pose.clone())).await, "watchdog").unwrap()["effect"], "hold_and_fault");
    let with_driver = ArmSupervisor::new(no_pose).with_driver(Box::new(CapsDriver(full_caps())));
    assert_eq!(stop(&describe(&with_driver).await, "watchdog").unwrap()["effect"], "hold_and_fault");
}

#[tokio::test]
async fn runtime_keepout_is_reflected() {
    let supervisor = ArmSupervisor::new(ArmConfig::rebot_dm());
    let before = describe(&supervisor).await;
    assert_eq!(before["limits"]["keepout_source"], "profile");
    let b = KeepoutBox { name: "tray".into(), min_m: [0.2, -0.1, 0.0], max_m: [0.3, 0.1, 0.1] };
    supervisor.inner().write().await.set_keepout_boxes(vec![b]).unwrap();
    let after = describe(&supervisor).await;
    assert_eq!(after["limits"]["keepout_source"], "runtime");
    assert_eq!(after["limits"]["keepout_boxes"], json!([{"name": "tray", "min_m": [0.2f32, -0.1f32, 0.0f32], "max_m": [0.3f32, 0.1f32, 0.1f32], "source": "runtime"}]));
}

#[tokio::test]
async fn collision_stop_only_with_the_capability_and_follows_the_runtime_setting() {
    // No driver (mock): no torque feedback, no collision_guard capability.
    let mock = ArmSupervisor::new(ArmConfig::rebot_dm());
    assert!(!mock.get_schema().await.capabilities.collision_guard);
    assert!(stop(&describe(&mock).await, "collision_protective_stop").is_none());
    // Torque feedback but no gravity model: still absent.
    let no_model = ArmSupervisor::new(ArmConfig { gravity_model: None, gravity_comp: None, ..ArmConfig::rebot_dm() }).with_driver(Box::new(CapsDriver(full_caps())));
    assert!(stop(&describe(&no_model).await, "collision_protective_stop").is_none());

    let guarded = ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(CapsDriver(full_caps())));
    assert!(guarded.get_schema().await.capabilities.collision_guard);
    let d = describe(&guarded).await;
    let c = stop(&d, "collision_protective_stop").unwrap();
    let p = ArmConfig::rebot_dm().collision.unwrap_or_default();
    assert_eq!(c["enabled"], p.enabled);
    assert_eq!(c["sensitivity"], "medium");
    assert_eq!(c["thresholds_nm"], json!(p.thresholds_nm));
    assert_eq!(c["auto_resume_s"], p.resume_after_s);
    assert_eq!(c["escalation"], json!({"max_auto_resumes": p.max_auto_resumes, "window_s": p.escalate_window_s, "fault_code": "COLLISION"}));
    assert_eq!(c["lease_required"], false);
    assert_eq!(c["clear"], json!(["auto_resume"]));

    guarded.tick(0.02).await; // the guard learns the driver's torque feedback in the tick
    {
        let mut inner = guarded.inner().write().await;
        armd::collision::update(&mut inner, &json!({"enabled": true, "sensitivity": "high"})).unwrap();
    }
    let d = describe(&guarded).await;
    let c = stop(&d, "collision_protective_stop").unwrap();
    assert_eq!(c["enabled"], true);
    assert_eq!(c["sensitivity"], "high");
    assert_eq!(c["source"], "runtime");
    assert_eq!(c["thresholds_nm"], json!(guarded.inner().read().await.collision_thresholds()));
}

#[tokio::test]
async fn hardware_section_follows_the_profile() {
    let mut config = ArmConfig::rebot_dm();
    let json: serde_json::Value = serde_json::from_str(r#"{"physical_estop": true}"#).unwrap();
    config.hardware = Some(serde_json::from_value(json).unwrap());
    let d = describe(&ArmSupervisor::new(config.clone())).await;
    assert_eq!(d["hardware"]["physical_estop"], true);
    let notes = d["hardware"]["notes"].as_array().unwrap();
    assert!(notes.iter().all(|n| n != "software stops only; no wired emergency stop"), "{notes:?}");
    assert_eq!(notes.len(), 2);
    config.hardware = Some(armd::model::HardwareInfo { physical_estop: true, notes: Some(vec!["mushroom button on the base cuts motor power".into()]) });
    let d = describe(&ArmSupervisor::new(config)).await;
    assert_eq!(d["hardware"]["notes"], json!(["mushroom button on the base cuts motor power"]));
}
