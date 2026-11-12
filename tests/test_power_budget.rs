//! Supply power budget (src/power.rs, docs/control_safety_contract.md "Power budget"):
//! the supply-current model, its profile section and the per-tick estimate.
//!
//! Every current printed here is the model's estimate from datasheet motor parameters, not a
//! measurement (the arm has no current sensor).
mod common;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::JointState;
use armd::power::{JointPower, PeakWindow, PowerBudget};
use armd::{ArmConfig, ArmObservation, SupervisorInner};
use common::INSPECT;

const J4: usize = 3;
const DT: f64 = 0.02;

#[derive(Default)]
struct ArmState {
    q: [f32; 7],
    v: [f32; 7],
    torqued: bool,
    /// Reported torque override per joint.
    forced: [Option<f32>; 7],
}

/// Ideal position servo: follows the commanded position and velocity, reports the built-in
/// model gravity torque (or a forced value). Simulation, not hardware.
#[derive(Clone)]
struct SimArm(Arc<Mutex<ArmState>>);

impl SimArm {
    fn new(q: [f32; 7]) -> Self { Self(Arc::new(Mutex::new(ArmState { q, ..Default::default() }))) }
    fn with<R>(&self, f: impl FnOnce(&mut ArmState) -> R) -> R { f(&mut self.0.lock().unwrap()) }
}

impl ArmDriver for SimArm {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState { if self.0.lock().unwrap().torqued { EnableState::Enabled } else { EnableState::Disabled } }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.0.lock().unwrap().torqued }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let s = self.0.lock().unwrap();
        let g = armd::compute_gravity_torques(&std::array::from_fn(|i| s.q[i])).unwrap();
        Ok((0..7).map(|i| JointState {
            position: s.q[i], velocity: s.v[i], temperature: 30.0,
            torque: s.forced[i].unwrap_or(if i < 6 { g[i] } else { 0.0 }),
        }).collect())
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.0.lock().unwrap();
        if !s.torqued { return Ok(()); }
        for (i, t) in targets.iter().enumerate().take(7) {
            s.q[i] = t.position;
            s.v[i] = t.velocity.unwrap_or(0.0);
        }
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().torqued = enabled;
        Ok(())
    }
}

fn rig_at(config: ArmConfig, q: [f32; 7]) -> (SupervisorInner, SimArm) {
    let arm = SimArm::new(q);
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..config });
    inner.lease_enabled = false;
    inner.driver = Some(Mutex::new(Box::new(arm.clone())));
    inner.commanded_positions = q.to_vec();
    inner.measured_positions = q.to_vec();
    inner.target_positions = q.to_vec();
    inner.set_torque(true).unwrap();
    ticks(&mut inner, 5);
    assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());
    (inner, arm)
}

fn ticks(inner: &mut SupervisorInner, n: usize) -> ArmObservation {
    let mut obs = None;
    for _ in 0..n { obs = Some(inner.tick(DT)); }
    obs.unwrap()
}

fn dm() -> PowerBudget { PowerBudget::rebot_dm() }

/// Copper loss + mechanical draw of one joint, written out from the documented equation.
fn joint_w(p: &JointPower, b: &PowerBudget, tau: f32, omega: f32) -> f32 {
    (tau * omega).max(0.0) / b.efficiency + b.k_cu * (tau / p.kt_nm_per_a).powi(2) * p.r_phase_ohm + p.idle_w
}

#[test]
fn b601_dm_defaults_are_the_lrs_150_24_and_the_datasheet_motors() {
    let b = dm();
    assert_eq!((b.supply_rated_a, b.supply_trip_a, b.v_bus, b.base_w, b.k_cu, b.efficiency), (6.5, 8.5, 24.0, 4.0, 1.5, 0.8));
    assert_eq!(b.joints.len(), 7);
    for j in 0..3 { assert_eq!(b.joints[j], JointPower::DM_J4340P); }
    for j in 3..7 { assert_eq!(b.joints[j], JointPower::DM_J4310); }
    // kt at the output: 40 N·m / 19.85 A and 12.5 N·m / 20 A
    assert!((JointPower::DM_J4340P.kt_nm_per_a - 40.0 / 19.85).abs() < 1e-3);
    assert!((JointPower::DM_J4310.kt_nm_per_a - 12.5 / 20.0).abs() < 1e-6);
    let cfg = ArmConfig::rebot_dm();
    assert_eq!(cfg.power_budget.as_ref(), Some(&b));
    cfg.clone().validate().unwrap();
}

#[test]
fn idle_arm_draws_idle_and_base_load() {
    let b = dm();
    let e = b.estimate(&[0.0; 7], &[0.0; 7]);
    let want = (4.0 + 7.0 * 0.5) / 24.0;
    assert!((e.bus_a - want).abs() < 1e-6, "{} vs {want}", e.bus_a);
    assert!((b.idle_a() - want).abs() < 1e-6);
    assert_eq!(e.regen_w, 0.0);
}

#[test]
fn known_inputs_give_the_documented_current() {
    let b = dm();
    // J1 10 N·m at +1 rad/s: 10 / 0.8 + 1.5 · (10 / 2.015)² · 0.72 + 0.5 W
    let mut tau = [0.0f32; 7];
    let mut omega = [0.0f32; 7];
    tau[0] = 10.0;
    omega[0] = 1.0;
    let e = b.estimate(&tau, &omega);
    let j1 = 12.5 + 1.5 * (10.0f32 / 2.015).powi(2) * 0.72 + 0.5;
    let want = (j1 + 6.0 * 0.5 + 4.0) / 24.0;
    assert!((e.bus_a - want).abs() < 1e-5, "{} vs {want}", e.bus_a);
    assert!((e.joint_a[0] - j1 / 24.0).abs() < 1e-5);
    eprintln!("J1 10 N·m at 1 rad/s: {:.3} A ({j1:.1} W on J1)", e.bus_a);
    // same torque driven backwards: the mechanical power is not credited, it is regen
    omega[0] = -1.0;
    let back = b.estimate(&tau, &omega);
    assert!((back.regen_w - 10.0 * 0.8).abs() < 1e-5);
    assert!((back.bus_a - (want - 12.5 / 24.0)).abs() < 1e-5, "{}", back.bus_a);
    // every joint, against the written-out equation
    let tau = [3.0, -8.0, 5.0, 2.5, -1.0, 0.5, 1.5];
    let omega = [0.3, 0.4, -0.2, 1.0, 2.0, -3.0, 0.0];
    let e = b.estimate(&tau, &omega);
    let sum: f32 = (0..7).map(|j| joint_w(&b.joints[j], &b, tau[j], omega[j])).sum();
    assert!((e.bus_a - (sum + b.base_w) / b.v_bus).abs() < 1e-5);
    // non-finite feedback counts as 0
    let e = b.estimate(&[f32::NAN; 7], &[f32::INFINITY; 7]);
    assert!((e.bus_a - b.idle_a()).abs() < 1e-6);
}

/// The 2026-11-12 incident (black box at 5 Hz): J4 stalled at 9.5 N·m, J2 ~2.4 N·m and J3
/// ~3.2 N·m holding, others small; the supply tripped (teardown: hiccup at ~8.5 A).
#[test]
fn incident_estimate_is_at_or_above_the_trip() {
    let b = dm();
    let tau = [0.2, 2.4, 3.2, 9.5, 0.3, 0.1, 0.2];
    let e = b.estimate(&tau, &[0.0; 7]);
    eprintln!("incident estimate: {:.2} A (J4 {:.2} A, J2 {:.3} A, J3 {:.3} A); trip ~{} A, rated {} A",
        e.bus_a, e.joint_a[J4], e.joint_a[1], e.joint_a[2], b.supply_trip_a, b.supply_rated_a);
    assert!(e.bus_a >= 8.5 && e.bus_a < 9.5, "{}", e.bus_a);
    // J4 alone: torque at which the estimate reaches the trip / the rating
    let at = |amps: f32| ((amps * b.v_bus - b.base_w - 7.0 * 0.5) / (b.k_cu * 0.58)).sqrt() * 0.625;
    eprintln!("J4 stalled alone reaches {} A at {:.2} N·m, {} A at {:.2} N·m", b.supply_rated_a, at(6.5), b.supply_trip_a, at(8.5));
}

#[test]
fn profile_validation() {
    let ok = dm();
    ok.validate(7).unwrap();
    assert!(ok.validate(6).unwrap_err().contains("needs 6 entries"));
    let bad = |f: &dyn Fn(&mut PowerBudget)| { let mut b = dm(); f(&mut b); b.validate(7).is_err() };
    assert!(bad(&|b| b.supply_rated_a = 0.0));
    assert!(bad(&|b| b.supply_trip_a = 6.0));
    assert!(bad(&|b| b.v_bus = 2.0));
    assert!(bad(&|b| b.base_w = -1.0));
    assert!(bad(&|b| b.k_cu = 0.0));
    assert!(bad(&|b| b.efficiency = 1.2));
    assert!(bad(&|b| b.filter_s = f32::NAN));
    assert!(bad(&|b| b.joints[2].kt_nm_per_a = 0.0));
    assert!(bad(&|b| b.joints[2].r_phase_ohm = -0.1));
    assert!(bad(&|b| { b.joints.pop(); }));
    // through the profile
    let mut cfg = ArmConfig::rebot_dm();
    cfg.power_budget.as_mut().unwrap().joints.pop();
    assert!(cfg.validate().unwrap_err().contains("power_budget.joints"));
}

#[test]
fn serde_defaults_and_absent_section() {
    // absent: off, and other profiles have none
    let mut v = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
    v.as_object_mut().unwrap().remove("power_budget");
    let cfg: ArmConfig = serde_json::from_value(v.clone()).unwrap();
    assert!(cfg.power_budget.is_none());
    assert!(ArmConfig::default().power_budget.is_none());
    let rs: ArmConfig = serde_json::from_str(&std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/profiles/rebot_b601_rs.json")).unwrap()).unwrap();
    assert!(rs.power_budget.is_none());
    assert!(!armd::model::Capabilities::from_config(&cfg).power_budget);
    // only the joints given: every other field takes its default
    v["power_budget"] = serde_json::json!({"joints": [
        {"kt_nm_per_a": 2.015, "r_phase_ohm": 0.72}, {"kt_nm_per_a": 2.015, "r_phase_ohm": 0.72}, {"kt_nm_per_a": 2.015, "r_phase_ohm": 0.72},
        {"kt_nm_per_a": 0.625, "r_phase_ohm": 0.58}, {"kt_nm_per_a": 0.625, "r_phase_ohm": 0.58}, {"kt_nm_per_a": 0.625, "r_phase_ohm": 0.58},
        {"kt_nm_per_a": 0.625, "r_phase_ohm": 0.58}]});
    let mut cfg: ArmConfig = serde_json::from_value(v.clone()).unwrap();
    cfg.validate().unwrap();
    assert_eq!(cfg.power_budget.as_ref().unwrap(), &dm());
    assert!(armd::model::Capabilities::from_config(&cfg).power_budget);
    // a joint entry needs kt and r
    v["power_budget"]["joints"][0] = serde_json::json!({"kt_nm_per_a": 2.0});
    assert!(serde_json::from_value::<ArmConfig>(v.clone()).is_err());
    // the section without joints does not validate
    v["power_budget"] = serde_json::json!({"supply_rated_a": 14.6, "supply_trip_a": 18.0});
    let mut cfg: ArmConfig = serde_json::from_value(v).unwrap();
    assert!(cfg.validate().is_err());
}

#[test]
fn peak_window_keeps_the_recent_maximum() {
    let mut w: PeakWindow<10> = PeakWindow::new(0.5);
    w.push(0.0, 2.0);
    w.push(0.2, 7.0);
    w.push(1.0, 3.0);
    assert_eq!(w.max(), 7.0);
    w.push(4.6, 1.0);
    assert_eq!(w.max(), 7.0, "still inside 5 s");
    w.push(5.6, 1.0);
    assert_eq!(w.max(), 3.0, "the 7.0 bucket aged out");
    w.push(100.0, 0.5);
    assert_eq!(w.max(), 0.5, "a long gap clears every bucket");
}

#[test]
fn observation_carries_the_filtered_estimate() {
    let (mut inner, arm) = rig_at(ArmConfig::rebot_dm(), INSPECT);
    let obs = ticks(&mut inner, 30);
    let p = obs.power.clone().expect("power_budget on");
    assert!(p.valid && p.rated_a == 6.5 && p.scale == 1.0 && !p.limiting && !p.hold);
    assert_eq!(p.per_joint_a.len(), 7);
    let b = dm();
    let tau: Vec<f32> = obs.torques.clone();
    let want = b.estimate(&tau, &obs.velocities).bus_a;
    assert!((p.bus_a - want).abs() < 2e-3, "{} vs {want}", p.bus_a);
    assert!((p.ratio - p.bus_a / 6.5).abs() < 2e-3);
    eprintln!("holding INSPECT: {:.3} A (idle + base {:.3} A)", p.bus_a, b.idle_a());
    // a step in J4 torque: the display filter (0.25 s) lags, the 5 s peak follows the fast filter
    arm.with(|s| s.forced[J4] = Some(8.0));
    let obs = ticks(&mut inner, 5);
    let p = obs.power.clone().unwrap();
    let stepped = b.estimate(&obs.torques, &obs.velocities).bus_a;
    assert!(p.bus_a < 0.5 * stepped, "display filter after 0.1 s: {} of {stepped}", p.bus_a);
    assert!(p.peak_a > 0.8 * stepped, "peak {} of {stepped} (fast filter 40 ms, 5 ticks)", p.peak_a);
    let obs = ticks(&mut inner, 75);
    let p = obs.power.clone().unwrap();
    assert!((p.bus_a - stepped).abs() < 0.01 * stepped, "{} vs {stepped} after 1.6 s", p.bus_a);
    eprintln!("J4 forced 8 N·m: {:.2} A", p.bus_a);
    // diagnostics: peaks and the budget in force
    let d = inner.power_diagnostics();
    assert_eq!(d["enabled"], true);
    assert!(d["peak_60s_a"].as_f64().unwrap() >= f64::from(stepped) * 0.99);
    assert!(d["max_a"].as_f64().unwrap() >= f64::from(stepped) * 0.99);
    assert_eq!(d["budget"]["supply_rated_a"], 6.5);
    // no driver / no torque feedback: published as not valid
    let mut sim = common::sim_at(ArmConfig::rebot_dm(), &INSPECT);
    let p = sim.tick(DT).power.unwrap();
    assert!(!p.valid && p.bus_a == 0.0);
    // no budget: null
    let mut cfg = ArmConfig::rebot_dm();
    cfg.power_budget = None;
    let (mut inner, _) = rig_at(cfg, INSPECT);
    assert!(ticks(&mut inner, 2).power.is_none());
    assert_eq!(inner.power_diagnostics()["enabled"], false);
}

#[tokio::test]
async fn diagnostics_power_route() {
    let (addr, _sup) = common::spawn_server(ArmConfig::rebot_dm()).await;
    let v: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/power")).await.unwrap().json().await.unwrap();
    assert_eq!(v["enabled"], true);
    assert_eq!(v["estimate_only"], true);
    assert_eq!(v["observation"]["valid"], false, "no driver");
    assert_eq!(v["budget"]["joints"].as_array().unwrap().len(), 7);
}

/// Per-tick cost of the estimate (printed; the bound only catches gross regressions).
#[test]
fn estimate_cost_per_tick() {
    let b = dm();
    let mut m = armd::power::PowerMonitor::default();
    let n = 200_000;
    let t0 = std::time::Instant::now();
    let mut acc = 0.0f32;
    for k in 0..n {
        let x = (k % 100) as f32 * 0.01;
        let tau = [x, 2.0 * x, -x, 0.5, 0.1, x, 0.0];
        let e = b.estimate(&tau, &[x; 7]);
        m.update(&b, &e, 0.02, k as f64 * 0.02);
        acc += m.fast_a;
    }
    let per = t0.elapsed().as_secs_f64() / n as f64;
    eprintln!("estimate + filters: {:.0} ns per tick ({acc:.0})", per * 1e9);
    assert!(per < 20e-6);
}
