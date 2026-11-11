//! Profile `speed_cap`: min() with request speeds and planned_max_velocity for planned motion.

mod common;

use armd::kinematics;
use armd::model::{ArmConfig, ObservationSchema, SpeedCap};
use armd::motion::{plan_cartesian, resolve_goal, GoalSpec, Speed};
use armd::SupervisorInner;
use common::{sim_at, INSPECT};

fn capped(cap: SpeedCap) -> ArmConfig {
    ArmConfig { speed_cap: Some(cap), ..ArmConfig::rebot_dm() }
}

fn jog_duration(config: ArmConfig, kind: &str) -> f64 {
    let mut inner = sim_at(config, &[0.0; 7]);
    let target = if kind == "home" { inner.config.home_positions.clone() } else { INSPECT.to_vec() };
    let (_, _, duration) = inner.jog_action(target, None, None, kind).unwrap();
    duration
}

#[test]
fn joint_velocity_scale_lowers_jog_and_home_speed() {
    let half = capped(SpeedCap { joint_velocity_scale: Some(0.5), ..Default::default() });
    for kind in ["joint_move", "home"] {
        let free = jog_duration(ArmConfig::rebot_dm(), kind);
        let slow = jog_duration(half.clone(), kind);
        assert!((slow / free - 2.0).abs() < 0.02, "{kind}: {free} -> {slow}");
    }
    let inner = SupervisorInner::new(half);
    let limits = inner.planning_limits();
    for l in &limits[..6] { assert!((l.max_velocity - 0.225).abs() < 1e-6); }
    assert_eq!(limits[6].max_velocity, 2.0, "gripper exempt like planned_max_velocity");
}

#[test]
fn linear_cap_lowers_cartesian_duration_and_never_raises_it() {
    let plan = |config: ArmConfig, speed: Speed| {
        let inner = sim_at(config, &INSPECT);
        let input = inner.plan_input().unwrap();
        let p0 = kinematics::fk(&input.q0).p;
        let goal = resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.08, p0[1], p0[2]], r: None });
        plan_cartesian(&input, goal, speed).unwrap().duration
    };
    let fast = Speed { linear: 0.15, angular: 1.0 };
    let free = plan(ArmConfig::rebot_dm(), fast);
    let slow = plan(capped(SpeedCap { linear_mps: Some(0.02), ..Default::default() }), fast);
    assert!(slow >= 1.875 * 0.08 / 0.02 * 1.1 - 1e-6, "linear cap 0.02 m/s: {slow}");
    assert!(slow > free * 2.0, "{free} -> {slow}");
    // A cap above the request leaves the request.
    let req = Speed { linear: 0.02, angular: 0.5 };
    let a = plan(ArmConfig::rebot_dm(), req);
    let b = plan(capped(SpeedCap { linear_mps: Some(0.1), angular_rps: Some(1.0), joint_velocity_scale: None }), req);
    assert!((a - b).abs() < 1e-9);
}

#[test]
fn speed_cap_is_validated_and_published_in_the_schema() {
    for bad in [
        SpeedCap { linear_mps: Some(0.0), ..Default::default() },
        SpeedCap { angular_rps: Some(f32::NAN), ..Default::default() },
        SpeedCap { joint_velocity_scale: Some(1.5), ..Default::default() },
        SpeedCap { joint_velocity_scale: Some(0.0), ..Default::default() },
    ] {
        assert!(capped(bad).validate().is_err(), "{bad:?}");
    }
    let cap = SpeedCap { linear_mps: Some(0.05), angular_rps: Some(0.5), joint_velocity_scale: Some(0.8) };
    let schema = serde_json::to_value(ObservationSchema::from_config(&capped(cap))).unwrap();
    assert_eq!(schema["speed_cap"]["joint_velocity_scale"].as_f64().unwrap() as f32, 0.8);
    let none = serde_json::to_value(ObservationSchema::from_config(&ArmConfig::rebot_dm())).unwrap();
    assert!(none["speed_cap"].is_null());
    // Profiles parse with the section.
    let mut json = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
    json["speed_cap"] = serde_json::json!({"linear_mps": 0.05, "angular_rps": 0.5, "joint_velocity_scale": 0.8});
    let mut parsed: ArmConfig = serde_json::from_value(json).unwrap();
    parsed.validate().unwrap();
    assert_eq!(parsed.speed_cap, Some(cap));
}
