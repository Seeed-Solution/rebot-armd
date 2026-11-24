//! Zero calibration: WS `/api/v1/calibration/motor_zero {joints, dry_run, force, method}` makes
//! the current position of the given joints (1-based) the profile's `zero_reference` (the folded
//! rest pose; 0 on the B601-DM). `method`: `motor` (the driver's set-zero command, Damiao flash;
//! afterwards the joint zero offsets of those joints are -reference) or `software`
//! (calibration.json `joint_zero`, every driver, reversible with WS
//! `/api/v1/calibration/joint_zero/restore {backup}`; src/joint_zero.rs). Absent: `motor` when the
//! driver supports it, else `software`.
//!
//! Checked under the lock: the driver supports it (`calibration_tools.motor_zero`), every joint
//! is in `calibration_tools.motor_zero.joints`, the caller holds the lease (WS command), torque
//! is off, no action or motion runs (MOTOR_ZERO_ARMED), the arm is not in Fault / EStop
//! (MOTOR_ZERO_FAULT), the feedback is fresh (MOTOR_ZERO_NO_FEEDBACK) and, without
//! `force`, no joint reads more than [`MOTOR_ZERO_MAX_OFFSET_DEG`] from its reference (MOTOR_ZERO_FAR).
//! `dry_run` returns the readings and the predicted change and changes nothing.
//!
//! A real run (its own task, `calibration_busy` and `motor_zero_busy` set: motion, torque,
//! clear_fault and configuration writes are refused meanwhile) follows the official sequence
//! (reBotArm_control_py `RebotArm.set_zero`): disable every motor, wait for fresh feedback
//! confirming each requested motor disabled, send the set-zero command per joint 0.1 s apart,
//! then wait up to 1 s until every zeroed joint reads within [`MOTOR_ZERO_VERIFY_MRAD`] of its reference in
//! feedback newer than its command (MOTOR_ZERO_UNVERIFIED otherwise, with the readings). It
//! never moves the arm; torque stays off. Each run is appended to `motor_zero.jsonl` in the
//! calibration backup directory, and the calibrations measured with the old zero are
//! invalidated ([`crate::calibration::invalidate_after_zero`]): the kinematic correction is
//! removed (live and in `calibration.json`), the camera calibration is marked stale and, for
//! joints whose reading moved by more than 0.5 deg, the saved poses and (gripper joint) the gripper
//! calibration are marked stale.
//!
//! The software zero averages the readings over [`crate::joint_zero::SOFTWARE_CAPTURE`] (the arm
//! must stay still), backs up `calibration.json`, installs offset = motor reading - reference and
//! verifies the readings like the motor zero.
//!
//! A motor zero cannot be restored: the motors hold the new zero; a different zero is set again
//! by putting the arm at that pose and running the tool.

use std::sync::Arc;
use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::calibration::{CalibError, CalibrationDeps, LeaseCheck};
use crate::model::{MOTOR_ZERO_MAX_OFFSET_DEG, MOTOR_ZERO_VERIFY_MRAD};
use crate::safety::SafetyState;
use crate::supervisor::{ArmSupervisor, SupervisorInner};

pub const PATH: &str = "/api/v1/calibration/motor_zero";
/// Time allowed for fresh feedback confirming the motors disabled.
pub const DISABLE_CONFIRM_TIMEOUT: Duration = Duration::from_secs(1);
/// Spacing of the set-zero commands (official sequence: 0.1 s).
pub const COMMAND_SPACING: Duration = Duration::from_millis(100);
/// Time allowed for the readings to reach 0 after the last command.
pub const VERIFY_TIMEOUT: Duration = Duration::from_secs(1);
const POLL: Duration = Duration::from_millis(20);
/// Log of the runs, in the calibration backup directory.
pub const LOG_NAME: &str = "motor_zero.jsonl";

/// Body of the command.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorZeroRequest {
    /// 0-based joint indices, in request order.
    pub joints: Vec<usize>,
    pub dry_run: bool,
    pub force: bool,
    /// `motor` / `software`; None = the default ([`check`] resolves it).
    pub method: Option<String>,
}

/// Largest motion (rad) of a joint during the software capture.
pub const CAPTURE_MAX_SPREAD_RAD: f32 = 0.01;
/// WS path of the joint zero restore.
pub const RESTORE_PATH: &str = "/api/v1/calibration/joint_zero/restore";

fn invalid(m: impl Into<String>) -> CalibError { CalibError::new(400, "INVALID_COMMAND", m) }

pub fn parse(body: &Value) -> Result<MotorZeroRequest, CalibError> {
    let o = body.as_object().ok_or_else(|| invalid("Command body must be an object"))?;
    for k in o.keys() {
        if !["joints", "dry_run", "force", "method"].contains(&k.as_str()) { return Err(invalid(format!("unknown field: {k}"))); }
    }
    let arr = o.get("joints").and_then(Value::as_array).filter(|a| !a.is_empty())
        .ok_or_else(|| invalid("joints must be a non-empty array of joint numbers (1-based)"))?;
    let mut joints = Vec::with_capacity(arr.len());
    for v in arr {
        let j = v.as_u64().filter(|j| *j >= 1 && *j <= 64).ok_or_else(|| invalid("joints must be joint numbers (1-based)"))? as usize;
        if joints.contains(&(j - 1)) { return Err(invalid(format!("joint {j} is listed twice"))); }
        joints.push(j - 1);
    }
    let flag = |k: &str| match o.get(k) {
        None | Some(Value::Null) => Ok(false),
        Some(Value::Bool(b)) => Ok(*b),
        Some(_) => Err(invalid(format!("{k} must be boolean"))),
    };
    let method = match o.get("method") {
        None | Some(Value::Null) => None,
        Some(Value::String(m)) if m == "motor" || m == "software" => Some(m.clone()),
        Some(_) => return Err(invalid("method must be \"motor\" or \"software\"")),
    };
    Ok(MotorZeroRequest { joints, dry_run: flag("dry_run")?, force: flag("force")?, method })
}

fn max_offset_rad() -> f32 { (MOTOR_ZERO_MAX_OFFSET_DEG as f32).to_radians() }
fn verify_rad() -> f32 { MOTOR_ZERO_VERIFY_MRAD as f32 / 1000.0 }

/// Torque on, a pending disable, an action, a trajectory or servo streaming (MOTOR_ZERO_ARMED).
fn armed_reason(inner: &SupervisorInner) -> Option<String> {
    if inner.is_torqued || inner.disable_pending { return Some("torque is on".into()); }
    if inner.last_action.as_ref().is_some_and(|a| a.is_running()) || inner.macro_run.is_some() { return Some("an action is running".into()); }
    if inner.gravity_comp.is_some() { return Some("gravity compensation is on".into()); }
    if inner.active_trajectory.is_some() || inner.servo_target.is_some() { return Some("a motion is running".into()); }
    if !matches!(inner.safety.state(), SafetyState::Idle) { return Some(format!("the arm is {:?}", inner.safety.state())); }
    None
}

/// The checks of a run, also before each command of a running one.
fn state_checks(inner: &SupervisorInner) -> Result<(), CalibError> {
    if inner.safety.state().is_stopped() {
        return Err(CalibError::new(409, "MOTOR_ZERO_FAULT", format!("the arm is in {:?}; clear the fault first", inner.safety.state())));
    }
    if let Some(why) = armed_reason(inner) {
        return Err(CalibError::new(409, "MOTOR_ZERO_ARMED",
            format!("motor zero needs torque off and no action running ({why}); park or disable torque first")));
    }
    Ok(())
}

/// Current readings of all joints; fresh feedback required with a driver.
fn readings(inner: &SupervisorInner) -> Result<Vec<f32>, CalibError> {
    if let Some(d) = inner.driver.as_ref() {
        let fresh = d.lock().is_ok_and(|drv| drv.is_feedback_valid());
        if !fresh { return Err(CalibError::new(409, "MOTOR_ZERO_NO_FEEDBACK", "the motor feedback is not fresh; check the bus and the motor power")); }
    }
    let q = inner.measured_positions.clone();
    if q.len() != inner.config.dof || !q.iter().all(|v| v.is_finite()) {
        return Err(CalibError::new(409, "MOTOR_ZERO_NO_FEEDBACK", "no valid joint readings"));
    }
    Ok(q)
}

/// The reference (joint angles) of the zero: the profile's `zero_reference`, else 0.
fn reference(inner: &SupervisorInner) -> Vec<f32> {
    inner.zero_reference_joints().unwrap_or_else(|| vec![0.0; inner.config.dof])
}

fn far(q: &[f32], r: &[f32], j: usize) -> bool { (q[j] - r[j]).abs() > max_offset_rad() }

fn joint_rows(req: &MotorZeroRequest, q: &[f32], r: &[f32]) -> Vec<Value> {
    req.joints.iter().map(|&j| json!({"joint": j + 1, "name": format!("J{}", j + 1), "reading_rad": q[j],
        "reading_deg": f64::from(q[j]).to_degrees(), "reference_rad": r[j], "after_rad": r[j], "change_rad": r[j] - q[j], "far": far(q, r, j)})).collect()
}

/// What a run would invalidate now (poses / gripper calibration only for joints moving > 0.5 deg).
fn invalidates(inner: &SupervisorInner, req: &MotorZeroRequest, q: &[f32], r: &[f32]) -> Value {
    let moved: Vec<usize> = req.joints.iter().copied().filter(|&j| f64::from((q[j] - r[j]).abs()) > crate::joint_zero::STALE_THRESHOLD_RAD).collect();
    let grip = inner.config.gripper_index();
    json!({"kinematics": inner.config.arm_model.correction().is_some(), "camera": inner.camera_calibrated(),
        "poses": if moved.is_empty() { Vec::new() } else { inner.poses.list.poses.iter().map(|p| p.name.clone()).collect() },
        "gripper_calibration": grip.is_some_and(|g| moved.contains(&g)) && (inner.gripper_calib_offset.is_some() || inner.gripper_calib_jaw_opening.is_some())})
}

/// The method of a request: given, else `motor` when the driver supports it, else `software`.
fn method_of(req: &MotorZeroRequest, tool: &crate::model::MotorZeroTool) -> &'static str {
    match req.method.as_deref() {
        Some("software") => "software",
        Some(_) => "motor",
        None if tool.motor => "motor",
        None => "software",
    }
}

/// Under the lock: parse and check. Ok(None) = dry run reply; Ok(Some) = a real run to start
/// (the caller sets the busy flags).
pub fn check(inner: &SupervisorInner, body: &Value) -> Result<(MotorZeroRequest, Option<Value>), CalibError> {
    let mut req = parse(body)?;
    let tool = inner.calibration_tools().motor_zero;
    if !tool.support.supported {
        return Err(CalibError::new(400, "UNSUPPORTED", format!("motor zero is not supported: {}", tool.support.reason.unwrap_or_default())));
    }
    let method = method_of(&req, &tool);
    req.method = Some(method.into());
    if method == "motor" && !tool.motor {
        return Err(CalibError::new(400, "UNSUPPORTED", format!("motor zero is not supported by this driver; use method \"software\" ({})",
            tool.support.notes.join("; "))));
    }
    if method == "software" && !tool.software {
        return Err(CalibError::new(400, "UNSUPPORTED", "the software zero needs a profile zero_reference"));
    }
    if let Some(j) = req.joints.iter().find(|j| !tool.joints.contains(&(*j + 1))) {
        return Err(CalibError::new(400, "JOINT_UNSUPPORTED", format!("joint {} is not in calibration_tools.motor_zero.joints {:?}", j + 1, tool.joints)));
    }
    if inner.calibration_busy { return Err(CalibError::new(409, "CONFIG_BUSY", "a calibration apply, restore or motor zero is running")); }
    state_checks(inner)?;
    let q = readings(inner)?;
    let r = reference(inner);
    let rows = joint_rows(&req, &q, &r);
    let persist = if method == "software" { Some("calibration_file".to_string()) } else { tool.support.persist.clone() };
    if req.dry_run {
        let far_j: Vec<usize> = req.joints.iter().filter(|&&j| far(&q, &r, j)).map(|j| j + 1).collect();
        return Ok((req.clone(), Some(json!({"dry_run": true, "joints": rows, "needs_force": !far_j.is_empty(), "far_joints": far_j,
            "invalidates": invalidates(inner, &req, &q, &r), "zero_method": method, "method": tool.support.method, "persist": persist,
            "persist_verified": method == "software" || tool.support.persist_verified, "max_offset_deg": MOTOR_ZERO_MAX_OFFSET_DEG}))));
    }
    let far_s: Vec<String> = req.joints.iter().filter(|&&j| far(&q, &r, j))
        .map(|&j| format!("J{} reads {:.1} deg (reference {:.1} deg)", j + 1, f64::from(q[j]).to_degrees(), f64::from(r[j]).to_degrees())).collect();
    if !far_s.is_empty() && !req.force {
        let far = far_s;
        let mut e = CalibError::new(409, "MOTOR_ZERO_FAR", format!("{} (more than {MOTOR_ZERO_MAX_OFFSET_DEG} deg from the reference); \
            check the arm is in the rest pose, or send force: true for a lost zero", far.join(", ")));
        e.extra.insert("joints".into(), json!(rows));
        return Err(e);
    }
    Ok((req, None))
}

/// Per-joint state of the driver for the disable confirmation and the verification.
struct Feedback { ages: Vec<Option<Duration>>, status: Option<Vec<u8>> }

fn feedback(inner: &SupervisorInner) -> Option<Feedback> {
    let d = inner.driver.as_ref()?;
    let drv = d.lock().ok()?;
    Some(Feedback { ages: drv.feedback_ages(), status: drv.motor_status() })
}

/// Sample newer than `since` for joint `j` (always true for the mock follower).
fn fresh_since(fb: &Option<Feedback>, j: usize, since: Instant) -> bool {
    match fb {
        None => true,
        Some(f) => f.ages.get(j).copied().flatten().is_some_and(|age| Instant::now().checked_sub(age).is_some_and(|t| t > since)),
    }
}

async fn locked_step<T>(sup: &ArmSupervisor, check: &LeaseCheck, step: &'static str,
    f: impl FnOnce(&mut SupervisorInner) -> Result<T, CalibError>) -> Result<T, CalibError> {
    let mut inner = sup.inner().write().await;
    let at = |mut e: CalibError| { e.extra.insert("step".into(), json!(step)); e };
    check(&mut inner).map_err(at)?;
    state_checks(&inner).map_err(at)?;
    f(&mut inner).map_err(at)
}

/// The joint zero after zeroing `joints` (0-based) with offsets `new` (None = unchanged in the
/// file: offsets all 0 and no `joint_zero` stored).
fn zero_entry(inner: &SupervisorInner, new: &[f32], method: &str) -> Option<Option<crate::joint_zero::JointZero>> {
    let name = inner.config.zero_reference.as_ref().map_or("rest", |r| r.name.as_str()).to_string();
    let stored = inner.calibration.as_ref().and_then(|c| c.joint_zero.as_ref()).is_some();
    if new.iter().all(|o| *o == 0.0) {
        return stored.then_some(None);
    }
    Some(Some(crate::joint_zero::JointZero::new(new, method, &name)))
}

/// Off-lock check for `OWNER_EXECUTOR` mode: validates request against `PublishedConfig` +
/// `StateHub`, failing closed with `EXECUTOR_CAPABILITY_UNSUPPORTED` when hardware motor zero
/// requires synchronous CAN register I/O.
pub fn check_executor(
    pub_cfg: &crate::rt_core::requests::config::PublishedConfig,
    obs: &crate::model::ArmObservation,
    body: &Value,
) -> Result<(MotorZeroRequest, Option<Value>), CalibError> {
    let mut req = parse(body)?;
    let tool = pub_cfg.calibration_tools().motor_zero;
    if !tool.support.supported {
        return Err(CalibError::new(400, "UNSUPPORTED", format!("motor zero is not supported: {}", tool.support.reason.unwrap_or_default())));
    }
    let method = method_of(&req, &tool);
    req.method = Some(method.into());
    if method == "motor" && !tool.motor {
        return Err(CalibError::new(400, "UNSUPPORTED", format!("motor zero is not supported by this driver; use method \"software\" ({})",
            tool.support.notes.join("; "))));
    }
    if pub_cfg.has_driver {
        return Err(CalibError::new(
            503,
            "EXECUTOR_CAPABILITY_UNSUPPORTED",
            "hardware motor/software zero requires synchronous driver access unsupported under the RT executor",
        ));
    }
    if method == "software" && !tool.software {
        return Err(CalibError::new(400, "UNSUPPORTED", "the software zero needs a profile zero_reference"));
    }
    if let Some(j) = req.joints.iter().find(|j| !tool.joints.contains(&(*j + 1))) {
        return Err(CalibError::new(400, "JOINT_UNSUPPORTED", format!("joint {} is not in calibration_tools.motor_zero.joints {:?}", j + 1, tool.joints)));
    }
    if pub_cfg.calibration_busy {
        return Err(CalibError::new(409, "CONFIG_BUSY", "a calibration apply, restore or motor zero is running"));
    }
    state_checks_obs(obs)?;
    let q = readings_obs(pub_cfg, obs)?;
    let r = reference_pub(pub_cfg);
    let rows = joint_rows(&req, &q, &r);
    let persist = if method == "software" { Some("calibration_file".to_string()) } else { tool.support.persist.clone() };
    if req.dry_run {
        let far_j: Vec<usize> = req.joints.iter().filter(|&&j| far(&q, &r, j)).map(|j| j + 1).collect();
        return Ok((req.clone(), Some(json!({"dry_run": true, "joints": rows, "needs_force": !far_j.is_empty(), "far_joints": far_j,
            "invalidates": invalidates_pub(pub_cfg, &req, &q, &r), "zero_method": method, "method": tool.support.method, "persist": persist,
            "persist_verified": method == "software" || tool.support.persist_verified, "max_offset_deg": MOTOR_ZERO_MAX_OFFSET_DEG}))));
    }
    let far_s: Vec<String> = req.joints.iter().filter(|&&j| far(&q, &r, j))
        .map(|&j| format!("J{} reads {:.1} deg (reference {:.1} deg)", j + 1, f64::from(q[j]).to_degrees(), f64::from(r[j]).to_degrees())).collect();
    if !far_s.is_empty() && !req.force {
        let mut e = CalibError::new(409, "MOTOR_ZERO_FAR", format!("{} (more than {MOTOR_ZERO_MAX_OFFSET_DEG} deg from the reference); \
            check the arm is in the rest pose, or send force: true for a lost zero", far_s.join(", ")));
        e.extra.insert("joints".into(), json!(rows));
        return Err(e);
    }
    Ok((req, None))
}

/// Off-lock check for `/api/v1/calibration/joint_zero/restore` in `OWNER_EXECUTOR` mode.
pub fn check_restore_executor(
    pub_cfg: &crate::rt_core::requests::config::PublishedConfig,
    obs: &crate::model::ArmObservation,
    body: &Value,
) -> Result<String, CalibError> {
    let name = body.get("backup").and_then(Value::as_str)
        .filter(|n| !n.is_empty() && n.len() <= 64 && n.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_'))
        .ok_or_else(|| invalid("backup must be the name of a software zero backup (GET /api/v1/calibration backups)"))?.to_string();
    if body.as_object().is_some_and(|o| o.keys().any(|k| k != "backup")) { return Err(invalid("only the field backup is allowed")); }
    if pub_cfg.has_driver {
        return Err(CalibError::new(
            503,
            "EXECUTOR_CAPABILITY_UNSUPPORTED",
            "hardware joint zero restore requires synchronous driver access unsupported under the RT executor",
        ));
    }
    if pub_cfg.config.zero_reference.is_none() { return Err(CalibError::new(400, "UNSUPPORTED", "the joint zero needs a profile zero_reference")); }
    if pub_cfg.calibration_busy { return Err(CalibError::new(409, "CONFIG_BUSY", "a calibration apply, restore or zero is running")); }
    state_checks_obs(obs)?;
    Ok(name)
}

fn state_checks_obs(obs: &crate::model::ArmObservation) -> Result<(), CalibError> {
    if obs.safety_state.is_stopped() {
        return Err(CalibError::new(409, "MOTOR_ZERO_FAULT", format!("the arm is in {:?}; clear the fault first", obs.safety_state)));
    }
    let why = if obs.is_torqued || obs.disable_pending {
        Some("torque is on".to_string())
    } else if obs.action.as_ref().is_some_and(|a| a.is_running()) {
        Some("an action is running".to_string())
    } else if obs.control_mode == crate::model::ControlMode::GravityComp {
        Some("gravity compensation is on".to_string())
    } else if !matches!(obs.safety_state, SafetyState::Idle) {
        Some(format!("the arm is {:?}", obs.safety_state))
    } else {
        None
    };
    if let Some(why) = why {
        return Err(CalibError::new(409, "MOTOR_ZERO_ARMED",
            format!("motor zero needs torque off and no action running ({why}); park or disable torque first")));
    }
    Ok(())
}

fn readings_obs(
    pub_cfg: &crate::rt_core::requests::config::PublishedConfig,
    obs: &crate::model::ArmObservation,
) -> Result<Vec<f32>, CalibError> {
    if pub_cfg.has_driver && !obs.feedback_valid {
        return Err(CalibError::new(409, "MOTOR_ZERO_NO_FEEDBACK", "the motor feedback is not fresh; check the bus and the motor power"));
    }
    if obs.positions.len() != pub_cfg.config.dof || !obs.positions.iter().all(|v| v.is_finite()) {
        return Err(CalibError::new(409, "MOTOR_ZERO_NO_FEEDBACK", "no valid joint readings"));
    }
    Ok(obs.positions.clone())
}

fn reference_pub(pub_cfg: &crate::rt_core::requests::config::PublishedConfig) -> Vec<f32> {
    if pub_cfg.config.zero_reference.is_some() {
        pub_cfg.config.home_positions.clone()
    } else {
        vec![0.0; pub_cfg.config.dof]
    }
}

fn invalidates_pub(
    pub_cfg: &crate::rt_core::requests::config::PublishedConfig,
    req: &MotorZeroRequest,
    q: &[f32],
    r: &[f32],
) -> Value {
    let moved: Vec<usize> = req.joints.iter().copied().filter(|&j| f64::from((q[j] - r[j]).abs()) > crate::joint_zero::STALE_THRESHOLD_RAD).collect();
    let grip = pub_cfg.config.gripper_index();
    json!({"kinematics": pub_cfg.config.arm_model.correction().is_some(), "camera": pub_cfg.camera_calibrated(),
        "poses": if moved.is_empty() { Vec::new() } else { pub_cfg.poses.list.poses.iter().map(|p| p.name.clone()).collect() },
        "gripper_calibration": grip.is_some_and(|g| moved.contains(&g)) && (pub_cfg.gripper_calib_offset.is_some() || pub_cfg.gripper_calib_jaw_opening.is_some())})
}

fn zero_entry_pub(
    pub_cfg: &crate::rt_core::requests::config::PublishedConfig,
    new: &[f32],
    method: &str,
) -> Option<Option<crate::joint_zero::JointZero>> {
    let name = pub_cfg.config.zero_reference.as_ref().map_or("rest", |r| r.name.as_str()).to_string();
    let stored = pub_cfg.calibration.as_ref().and_then(|c| c.joint_zero.as_ref()).is_some();
    if new.iter().all(|o| *o == 0.0) {
        return stored.then_some(None);
    }
    Some(Some(crate::joint_zero::JointZero::new(new, method, &name)))
}

/// The real run (after [`check`]; the caller holds the busy flags and clears them afterwards).
pub async fn run(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, req: MotorZeroRequest, check: &LeaseCheck) -> Result<Value, CalibError> {
    if sup.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return run_executor(sup, deps, req, None).await;
    }
    run_legacy(sup, deps, req, check, None).await
}

/// Authority-bound run for WS/Executor calibration jobs carrying ingress [`CalibCommitAuth`].
pub async fn run_with_auth(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    req: MotorZeroRequest,
    check: &LeaseCheck,
    auth: &crate::calibration::CalibCommitAuth,
) -> Result<Value, CalibError> {
    if sup.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return run_executor(sup, deps, req, Some(auth)).await;
    }
    run_legacy(sup, deps, req, check, Some(auth)).await
}

async fn run_legacy(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    req: MotorZeroRequest,
    check: &LeaseCheck,
    auth: Option<&crate::calibration::CalibCommitAuth>,
) -> Result<Value, CalibError> {
    if req.method.as_deref() == Some("software") {
        return run_software(sup, deps, req, check, auth).await;
    }
    let started = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
    // 1. readings, then disable every motor (official sequence), torque stays off
    let (before, r, t_disable) = locked_step(sup, check, "disable", |inner| {
        let q = readings(inner)?;
        if let Some(d) = inner.driver.as_ref() {
            let mut drv = d.lock().map_err(|_| CalibError::new(500, "INTERNAL", "driver unavailable"))?;
            drv.set_torque(false).map_err(|e| CalibError::new(500, "MOTOR_ZERO_NOT_DISABLED", format!("disable failed: {e}")))?;
        }
        Ok((q, reference(inner), Instant::now()))
    }).await?;
    // 2. fresh feedback confirming each requested motor disabled (state 0)
    let deadline = t_disable + DISABLE_CONFIRM_TIMEOUT;
    loop {
        let pending: Vec<usize> = {
            let inner = sup.inner().read().await;
            let fb = feedback(&inner);
            req.joints.iter().copied().filter(|&j| {
                let disabled = fb.as_ref().is_none_or(|f| f.status.as_ref().is_some_and(|s| s.get(j) == Some(&0)));
                !(disabled && fresh_since(&fb, j, t_disable))
            }).collect()
        };
        if pending.is_empty() { break; }
        if Instant::now() >= deadline {
            return Err(CalibError::new(409, "MOTOR_ZERO_NOT_DISABLED", format!("joints {:?} did not confirm disabled within {} ms; nothing was changed",
                pending.iter().map(|j| j + 1).collect::<Vec<_>>(), DISABLE_CONFIRM_TIMEOUT.as_millis())));
        }
        tokio::time::sleep(POLL).await;
    }
    // every check passed: invalidate a qualification of the zeroed joints on disk before the
    // first zero command (a failed write refuses the zero, nothing changed)
    let one_based: Vec<usize> = req.joints.iter().map(|j| j + 1).collect();
    crate::calibration::invalidate_mit_before_zero_with_auth(sup, deps, &one_based, "motor", auth).await?;
    // 3. one set-zero command per joint, spaced; the joint zero offset of each zeroed joint
    // becomes -reference (the motor reads 0 at the reference)
    let mut sent: Vec<(usize, Instant)> = Vec::new();
    let mut failure = None;
    for (k, &j) in req.joints.iter().enumerate() {
        if k > 0 { tokio::time::sleep(COMMAND_SPACING).await; }
        let rj = r[j];
        let res = locked_step(sup, check, "set_zero", |inner| {
            inner.set_motor_zero(j).map_err(|e| CalibError::new(500, "MOTOR_ZERO_FAILED", format!("J{}: {e}", j + 1)))?;
            let mut off = inner.joint_zero_offsets.clone();
            off[j] = -rj;
            inner.set_joint_zero_offsets(&off).map_err(|e| CalibError::new(500, "MOTOR_ZERO_FAILED", format!("J{} offset: {e}", j + 1)))?;
            Ok(off)
        }).await;
        match res {
            Ok(off) => {
                sup.published_config_hub().apply_joint_zero_offsets(&off);
                sent.push((j, Instant::now()));
            }
            Err(mut e) if sent.is_empty() => { e.message.push_str("; no zero was changed"); return Err(e); }
            Err(e) => { failure = Some(e); break; }
        }
    }
    // 4. verification from feedback newer than each command
    let rows = verify(sup, &req, &before, &r, &sent).await;
    let verified = rows.iter().all(|r| r["verified"] == json!(true));
    // 5. invalidate what was measured with the old zero, store the joint zero, log
    let zeroed: Vec<usize> = sent.iter().map(|(j, _)| j + 1).collect();
    let changes: Vec<f32> = sent.iter().map(|(j, _)| r[*j] - before[*j]).collect();
    let entry = { let inner = sup.inner().read().await; zero_entry(&inner, &inner.joint_zero_offsets, "motor") };
    let inv = crate::calibration::invalidate_after_zero_with_auth(sup, deps, &zeroed, &changes, "motor", entry, auth).await;
    let result = if failure.is_some() { "partial" } else if verified { "verified" } else { "unverified" };
    let log = write_log(deps, &started, &req, &before, &rows, result, &inv).await;
    if let Some(mut e) = failure {
        e.message.push_str(&format!("; joints {zeroed:?} were zeroed before the failure"));
        e.extra.insert("joints".into(), json!(rows));
        e.extra.insert("invalidated".into(), inv);
        e.extra.insert("log".into(), json!(log));
        return Err(e);
    }
    let tool = sup.inner().read().await.calibration_tools().motor_zero;
    tracing::warn!("Motor zero of joints {zeroed:?} (readings before {:?}): {}", req.joints.iter().map(|&j| before[j]).collect::<Vec<_>>(),
        if verified { "verified" } else { "NOT verified" });
    if !verified {
        let mut e = CalibError::new(500, "MOTOR_ZERO_UNVERIFIED", format!("the set-zero commands were sent but not every joint read within {MOTOR_ZERO_VERIFY_MRAD} mrad of its reference \
            within {} ms; check the readings and run the tool again", VERIFY_TIMEOUT.as_millis()));
        e.extra.insert("joints".into(), json!(rows));
        e.extra.insert("invalidated".into(), inv);
        e.extra.insert("log".into(), json!(log));
        return Err(e);
    }
    Ok(json!({"joints": rows, "invalidated": inv, "log": log, "zero_method": "motor", "method": tool.support.method, "persist": tool.support.persist,
        "persist_verified": tool.support.persist_verified, "torque": "off"}))
}

/// Software zero (Legacy): capture readings, back up `calibration.json`, then atomically persist
/// (`calibration.json` + stale `poses.json` + stale `gripper_calib.json`) and install the new
/// offsets ONLY at activation through the unified common commit path (`run_config_commit_detailed`).
async fn run_software(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    req: MotorZeroRequest,
    check: &LeaseCheck,
    auth: Option<&crate::calibration::CalibCommitAuth>,
) -> Result<Value, CalibError> {
    let started = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
    // 1. capture: mean of fresh readings over SOFTWARE_CAPTURE; the arm must stay still
    let (r, n) = locked_step(sup, check, "capture", |inner| { readings(inner)?; Ok((reference(inner), inner.config.dof)) }).await?;
    let t0 = Instant::now();
    let (mut sum, mut lo, mut hi, mut count) = (vec![0.0f64; n], vec![f32::INFINITY; n], vec![f32::NEG_INFINITY; n], 0u32);
    while t0.elapsed() < crate::joint_zero::SOFTWARE_CAPTURE || count == 0 {
        let q = locked_step(sup, check, "capture", |inner| readings(inner)).await?;
        for j in 0..n { sum[j] += f64::from(q[j]); lo[j] = lo[j].min(q[j]); hi[j] = hi[j].max(q[j]); }
        count += 1;
        tokio::time::sleep(POLL).await;
    }
    let before: Vec<f32> = sum.iter().map(|v| (*v / f64::from(count)) as f32).collect();
    let moving: Vec<String> = req.joints.iter().filter(|&&j| hi[j] - lo[j] > CAPTURE_MAX_SPREAD_RAD)
        .map(|&j| format!("J{} moved {:.2} deg", j + 1, f64::from(hi[j] - lo[j]).to_degrees())).collect();
    if !moving.is_empty() {
        return Err(CalibError::new(409, "ZERO_MOVING", format!("{} during the {} ms capture; hold the arm still in the rest pose and run again; nothing was changed",
            moving.join(", "), crate::joint_zero::SOFTWARE_CAPTURE.as_millis())));
    }
    // 2. new offsets: offset += mean reading - reference
    let (old, new, entry) = {
        let inner = sup.inner().read().await;
        let old = inner.joint_zero_offsets.clone();
        let mut new = old.clone();
        for &j in &req.joints { new[j] = old[j] + (before[j] - r[j]); }
        let entry = zero_entry(&inner, &new, "software");
        (old, new, entry)
    };
    if let Some(j) = req.joints.iter().find(|&&j| f64::from(new[j].abs()) > crate::joint_zero::MAX_OFFSET_RAD) {
        return Err(CalibError::new(409, "MOTOR_ZERO_FAR", format!("J{} would need a joint zero offset of {:.1} deg; nothing was changed", j + 1, f64::from(new[*j]).to_degrees())));
    }
    // 3. backup before touching disk or live state
    let backup = write_zero_backup(deps, "software_zero", &old).await.map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("backup: {e}; nothing was changed")))?;
    if auth.is_none() {
        locked_step(sup, check, "apply", |_| Ok(())).await?;
    }
    // 4. Persist calibration.json + stale poses/gripper files AND install live offsets ONLY at
    //    activation (after rename + parent-dir fsync) inside ONE reservation.
    let zeroed: Vec<usize> = req.joints.iter().map(|j| j + 1).collect();
    let changes: Vec<f32> = req.joints.iter().map(|&j| r[j] - before[j]).collect();
    let inv = crate::calibration::commit_joint_zero_and_invalidation(
        sup,
        deps,
        &zeroed,
        &changes,
        "software",
        entry,
        Some(new.clone()),
        auth,
        true,
    )
    .await
    .map_err(|e| e.at("apply"))?;
    let t_apply = Instant::now();
    // 5. verify against post-activation readings, then write log
    let sent: Vec<(usize, Instant)> = req.joints.iter().map(|&j| (j, t_apply)).collect();
    let rows = verify(sup, &req, &before, &r, &sent).await;
    let verified = rows.iter().all(|r| r["verified"] == json!(true));
    let log = write_log(deps, &started, &req, &before, &rows, if verified { "verified" } else { "unverified" }, &inv).await;
    tracing::warn!("Software zero of joints {zeroed:?}: offsets {old:?} -> {new:?} rad (backup {})", backup.display());
    let mut out = json!({"joints": rows, "invalidated": inv, "log": log, "zero_method": "software", "persist": "calibration_file", "persist_verified": true,
        "torque": "off", "backup": backup.display().to_string(), "offsets_before": old, "offsets_after": new});
    if !verified {
        let mut e = CalibError::new(500, "MOTOR_ZERO_UNVERIFIED", format!("the joint zero was installed but not every joint read within {MOTOR_ZERO_VERIFY_MRAD} mrad of its reference; \
            restore backup {} or run the tool again", backup.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()));
        if let Value::Object(m) = out.take() { e.extra = m; }
        return Err(e);
    }
    out["restore"] = json!({"path": RESTORE_PATH, "backup": backup.file_name().map(|n| n.to_string_lossy().into_owned())});
    Ok(out)
}

/// A backup of `calibration.json` before a joint zero change: `<backup_dir>/<time>/` with
/// `calibration.json` (when it exists) and `manifest.json` (`reason`, `joint_zero_offsets`).
async fn write_zero_backup(deps: &CalibrationDeps, reason: &str, offsets: &[f32]) -> Result<std::path::PathBuf, String> {
    let (root, calib) = (deps.backup_dir.clone(), deps.path.clone());
    let manifest = json!({"created_at": chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true), "reason": reason, "joint_zero_offsets": offsets});
    crate::calibration::blocking(move || {
        std::fs::create_dir_all(&root)?;
        let dir = root.join(crate::calibration::backup_name(&root));
        std::fs::create_dir(&dir)?;
        let mut files = vec!["manifest.json"];
        if let Some(t) = crate::calibration::read_if_file(&calib)? { crate::persist::write_atomic(&dir.join("calibration.json"), &t)?; files.push("calibration.json"); }
        let mut manifest = manifest;
        manifest["files"] = json!(files);
        crate::persist::write_atomic(&dir.join("manifest.json"), &serde_json::to_string_pretty(&manifest).unwrap_or_default())?;
        Ok(dir)
    }).await
}

/// Under the lock: check a joint zero restore `{backup}` (torque off, no action, lease).
pub fn check_restore(inner: &SupervisorInner, body: &Value) -> Result<String, CalibError> {
    let name = body.get("backup").and_then(Value::as_str)
        .filter(|n| !n.is_empty() && n.len() <= 64 && n.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_'))
        .ok_or_else(|| invalid("backup must be the name of a software zero backup (GET /api/v1/calibration backups)"))?.to_string();
    if body.as_object().is_some_and(|o| o.keys().any(|k| k != "backup")) { return Err(invalid("only the field backup is allowed")); }
    if inner.config.zero_reference.is_none() { return Err(CalibError::new(400, "UNSUPPORTED", "the joint zero needs a profile zero_reference")); }
    if inner.calibration_busy { return Err(CalibError::new(409, "CONFIG_BUSY", "a calibration apply, restore or zero is running")); }
    state_checks(inner)?;
    Ok(name)
}

/// WS `/api/v1/calibration/joint_zero/restore {backup}`: put back the joint zero a software zero
/// backup holds (its `calibration.json` `joint_zero`, none = offsets 0), with a new backup first;
/// invalidates like a zero.
pub async fn restore(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, name: String, check: &LeaseCheck) -> Result<Value, CalibError> {
    if sup.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return restore_executor(sup, deps, name, None).await;
    }
    restore_legacy(sup, deps, name, check, None).await
}

/// Authority-bound restore for WS/Executor calibration jobs carrying ingress [`CalibCommitAuth`].
pub async fn restore_with_auth(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    name: String,
    check: &LeaseCheck,
    auth: &crate::calibration::CalibCommitAuth,
) -> Result<Value, CalibError> {
    if sup.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return restore_executor(sup, deps, name, Some(auth)).await;
    }
    restore_legacy(sup, deps, name, check, Some(auth)).await
}

async fn restore_legacy(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    name: String,
    check: &LeaseCheck,
    auth: Option<&crate::calibration::CalibCommitAuth>,
) -> Result<Value, CalibError> {
    let dir = deps.backup_dir.join(&name);
    let manifest: Value = std::fs::read_to_string(dir.join("manifest.json")).ok().and_then(|t| serde_json::from_str(&t).ok())
        .ok_or_else(|| CalibError::new(404, "BACKUP_NOT_FOUND", format!("backup {name}: no manifest.json")))?;
    if manifest.get("reason").and_then(Value::as_str) != Some("software_zero") {
        return Err(invalid(format!("backup {name} is not a software zero backup")));
    }
    let target: Option<crate::joint_zero::JointZero> = match crate::calibration::read_if_file(&dir.join("calibration.json")) {
        Ok(None) => None,
        Ok(Some(t)) => serde_json::from_str::<crate::calibration::CalibrationFile>(&t).map_err(|e| invalid(format!("backup {name}: calibration.json: {e}")))?.joint_zero,
        Err(e) => return Err(CalibError::new(500, "PERSIST_FAILED", format!("backup {name}: {e}"))),
    };
    let (old, new, r) = locked_step(sup, check, "restore", |inner| {
        let n = inner.config.dof;
        let new = target.as_ref().map_or_else(|| vec![0.0; n], |z| z.offsets());
        if let Some(z) = &target { z.validate(Some(n)).map_err(|e| invalid(format!("backup {name}: {e}")))?; }
        Ok((inner.joint_zero_offsets.clone(), new, reference(inner)))
    }).await?;
    let backup = write_zero_backup(deps, "joint_zero_restore", &old).await.map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("backup: {e}; nothing was changed")))?;
    if auth.is_none() {
        locked_step(sup, check, "apply", |_| Ok(())).await?;
    }
    let joints: Vec<usize> = (0..new.len()).filter(|&j| old[j] != new[j]).collect();
    let changes: Vec<f32> = joints.iter().map(|&j| old[j] - new[j]).collect();
    let zeroed: Vec<usize> = joints.iter().map(|j| j + 1).collect();
    let entry = Some(target.filter(|z| !z.is_zero()));
    let inv = crate::calibration::commit_joint_zero_and_invalidation(
        sup,
        deps,
        &zeroed,
        &changes,
        "restored",
        entry,
        Some(new.clone()),
        auth,
        true,
    )
    .await
    .map_err(|e| e.at("apply"))?;
    let _ = r;
    tracing::warn!("Joint zero restored from backup {name}: offsets {old:?} -> {new:?} rad");
    Ok(json!({"restored": name, "backup": backup.display().to_string(), "offsets_before": old, "offsets_after": new, "invalidated": inv}))
}

fn check_executor_auth(
    sup: &ArmSupervisor,
    auth: Option<&crate::calibration::CalibCommitAuth>,
    step: &'static str,
) -> Result<(), CalibError> {
    let Some(a) = auth else { return Ok(()); };
    if let Err((_, e)) = (a.off_thread_check)() {
        return Err(e.at(step));
    }
    if Instant::now() > a.deadline {
        return Err(CalibError::new(409, "DEADLINE_EXPIRED", "motor zero command deadline expired").at(step));
    }
    if sup.estop_epoch() != a.admission.estop_epoch {
        return Err(CalibError::new(
            409,
            "EPOCH_MISMATCH",
            "The epoch changed during motor zero (stop or e-stop)",
        )
        .at(step));
    }
    if sup.current_stop_gen() != a.admission.stop_gen {
        return Err(CalibError::new(
            409,
            "STOPPED",
            "An authorized stop intervened during motor zero",
        )
        .at(step));
    }
    Ok(())
}

async fn dispatch_set_joint_zero_offsets_executor(
    sup: &Arc<ArmSupervisor>,
    offsets: Vec<f32>,
    auth: Option<&crate::calibration::CalibCommitAuth>,
) -> Result<(), CalibError> {
    use crate::rt_core::requests::config::{ConfigAuthContext, ConfigReply, ConfigRequest};
    let applied = offsets.clone();
    let precomputed_floor = sup
        .published_config_hub()
        .precompute_effective_floor_for_offsets(&offsets);
    let auth_ctx = auth.map(|a| ConfigAuthContext {
        admission: a.admission,
        lease_holder: a.lease_holder,
    });
    let expected_stop_gen = auth.map(|a| a.admission.stop_gen);
    let deadline = auth.map(|a| a.deadline);
    let job_token = auth.and_then(|a| a.job_token);
    match sup
        .dispatch_config_request_owned_reliable(ConfigRequest::SetJointZeroOffsets {
            offsets,
            precomputed_floor,
            auth: auth_ctx,
            expected_stop_gen,
            deadline,
            job_token,
        })
        .await
    {
        Ok(ConfigReply::JointZeroOffsetsSet(Ok(()))) => {
            sup.published_config_hub().apply_joint_zero_offsets(&applied);
            Ok(())
        }
        Ok(ConfigReply::JointZeroOffsetsSet(Err(e))) => Err(e.to_calib_error()),
        Err(e) => Err(e.to_calib_error()),
        _ => Err(CalibError::new(500, "INTERNAL", "unexpected SetJointZeroOffsets reply")),
    }
}

async fn dispatch_set_motor_zero_executor(
    sup: &Arc<ArmSupervisor>,
    joint: usize,
    auth: Option<&crate::calibration::CalibCommitAuth>,
) -> Result<(), CalibError> {
    use crate::rt_core::requests::config::{ConfigAuthContext, ConfigReply, ConfigRequest};
    let auth_ctx = auth.map(|a| ConfigAuthContext {
        admission: a.admission,
        lease_holder: a.lease_holder,
    });
    let expected_stop_gen = auth.map(|a| a.admission.stop_gen);
    let deadline = auth.map(|a| a.deadline);
    let job_token = auth.and_then(|a| a.job_token);
    match sup
        .dispatch_config_request_owned_reliable(ConfigRequest::SetMotorZero {
            joint,
            auth: auth_ctx,
            expected_stop_gen,
            deadline,
            job_token,
        })
        .await
    {
        Ok(ConfigReply::MotorZeroSet(Ok(()))) => Ok(()),
        Ok(ConfigReply::MotorZeroSet(Err(e))) => Err(e.to_calib_error()),
        Err(e) => Err(e.to_calib_error()),
        _ => Err(CalibError::new(500, "INTERNAL", "unexpected SetMotorZero reply")),
    }
}

/// Executor-mode motor/software zero (never acquires `SupervisorLock`; fails closed on hardware
/// driver access, carries job-bound authority to every owner mutation, and installs software
/// zero offsets ONLY at activation after rename + parent-dir fsync through the common commit path).
async fn run_executor(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    req: MotorZeroRequest,
    auth: Option<&crate::calibration::CalibCommitAuth>,
) -> Result<Value, CalibError> {
    let pub_cfg = sup.published_config().await.map_err(|e| CalibError::new(503, "CONFIG_UNAVAILABLE", e.to_string()))?;
    if pub_cfg.has_driver {
        return Err(CalibError::new(
            503,
            "EXECUTOR_CAPABILITY_UNSUPPORTED",
            "hardware motor/software zero requires synchronous driver access unsupported under the RT executor",
        ));
    }
    let latest_obs = || {
        sup.hub()
            .latest()
            .map(|s| s.obs.clone())
            .ok_or_else(|| CalibError::new(503, "OBSERVATION_UNAVAILABLE", "No published observation available"))
    };
    if req.method.as_deref() == Some("software") {
        let started = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
        let r = reference_pub(&pub_cfg);
        let n = pub_cfg.config.dof;
        let t0 = Instant::now();
        let (mut sum, mut lo, mut hi, mut count) = (vec![0.0f64; n], vec![f32::INFINITY; n], vec![f32::NEG_INFINITY; n], 0u32);
        while t0.elapsed() < crate::joint_zero::SOFTWARE_CAPTURE || count == 0 {
            check_executor_auth(sup, auth, "capture")?;
            let obs = latest_obs()?;
            state_checks_obs(&obs)?;
            let q = readings_obs(&pub_cfg, &obs)?;
            for j in 0..n { sum[j] += f64::from(q[j]); lo[j] = lo[j].min(q[j]); hi[j] = hi[j].max(q[j]); }
            count += 1;
            tokio::time::sleep(POLL).await;
        }
        let before: Vec<f32> = sum.iter().map(|v| (*v / f64::from(count)) as f32).collect();
        let moving: Vec<String> = req.joints.iter().filter(|&&j| hi[j] - lo[j] > CAPTURE_MAX_SPREAD_RAD)
            .map(|&j| format!("J{} moved {:.2} deg", j + 1, f64::from(hi[j] - lo[j]).to_degrees())).collect();
        if !moving.is_empty() {
            return Err(CalibError::new(409, "ZERO_MOVING", format!("{} during the {} ms capture; hold the arm still in the rest pose and run again; nothing was changed",
                moving.join(", "), crate::joint_zero::SOFTWARE_CAPTURE.as_millis())));
        }
        let old = pub_cfg.joint_zero_offsets.clone();
        let mut new = old.clone();
        for &j in &req.joints { new[j] = old[j] + (before[j] - r[j]); }
        if let Some(j) = req.joints.iter().find(|&&j| f64::from(new[j].abs()) > crate::joint_zero::MAX_OFFSET_RAD) {
            return Err(CalibError::new(409, "MOTOR_ZERO_FAR", format!("J{} would need a joint zero offset of {:.1} deg; nothing was changed", j + 1, f64::from(new[*j]).to_degrees())));
        }
        check_executor_auth(sup, auth, "backup")?;
        let backup = write_zero_backup(deps, "software_zero", &old).await.map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("backup: {e}; nothing was changed")))?;
        check_executor_auth(sup, auth, "apply")?;
        let zeroed: Vec<usize> = req.joints.iter().map(|j| j + 1).collect();
        let changes: Vec<f32> = req.joints.iter().map(|&j| r[j] - before[j]).collect();
        let entry = zero_entry_pub(&pub_cfg, &new, "software");
        // Atomic single-reservation commit: calibration.json + stale poses/gripper files are
        // staged, renamed, and dir-synced BEFORE live offsets and invalidations activate.
        let inv = crate::calibration::commit_joint_zero_and_invalidation(
            sup,
            deps,
            &zeroed,
            &changes,
            "software",
            entry,
            Some(new.clone()),
            auth,
            true,
        )
        .await
        .map_err(|e| e.at("apply"))?;
        let rows = verify_executor(sup, &req, &before, &r).await;
        let verified = rows.iter().all(|r| r["verified"] == json!(true));
        let log = write_log(deps, &started, &req, &before, &rows, if verified { "verified" } else { "unverified" }, &inv).await;
        let mut out = json!({"joints": rows, "invalidated": inv, "log": log, "zero_method": "software", "persist": "calibration_file", "persist_verified": true,
            "torque": "off", "backup": backup.display().to_string(), "offsets_before": old, "offsets_after": new});
        if !verified {
            let mut e = CalibError::new(500, "MOTOR_ZERO_UNVERIFIED", format!("the joint zero was installed but not every joint read within {MOTOR_ZERO_VERIFY_MRAD} mrad of its reference"));
            if let Value::Object(m) = out.take() { e.extra = m; }
            return Err(e);
        }
        out["restore"] = json!({"path": RESTORE_PATH, "backup": backup.file_name().map(|n| n.to_string_lossy().into_owned())});
        return Ok(out);
    }

    let started = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
    check_executor_auth(sup, auth, "disable")?;
    let obs0 = latest_obs()?;
    state_checks_obs(&obs0)?;
    let before = readings_obs(&pub_cfg, &obs0)?;
    let r = reference_pub(&pub_cfg);
    let one_based: Vec<usize> = req.joints.iter().map(|j| j + 1).collect();
    crate::calibration::invalidate_mit_before_zero_with_auth(sup, deps, &one_based, "motor", auth).await?;
    let mut off = pub_cfg.joint_zero_offsets.clone();
    let mut sent = Vec::new();
    let mut failure = None;
    for (k, &j) in req.joints.iter().enumerate() {
        if k > 0 { tokio::time::sleep(COMMAND_SPACING).await; }
        let res = async {
            check_executor_auth(sup, auth, "set_zero")?;
            dispatch_set_motor_zero_executor(sup, j, auth).await?;
            off[j] = -r[j];
            dispatch_set_joint_zero_offsets_executor(sup, off.clone(), auth).await
        }.await;
        match res {
            Ok(()) => sent.push(j),
            Err(mut e) if sent.is_empty() => { e.message.push_str("; no zero was changed"); return Err(e); }
            Err(e) => { failure = Some(e); break; }
        }
    }
    let rows = verify_executor(sup, &req, &before, &r).await;
    let verified = rows.iter().all(|r| r["verified"] == json!(true));
    let zeroed: Vec<usize> = sent.iter().map(|j| j + 1).collect();
    let changes: Vec<f32> = sent.iter().map(|j| r[*j] - before[*j]).collect();
    let entry = zero_entry_pub(&pub_cfg, &off, "motor");
    let inv = crate::calibration::invalidate_after_zero_strict_with_auth(sup, deps, &zeroed, &changes, "motor", entry, auth).await?;
    let result = if failure.is_some() { "partial" } else if verified { "verified" } else { "unverified" };
    let log = write_log(deps, &started, &req, &before, &rows, result, &inv).await;
    if let Some(mut e) = failure {
        e.message.push_str(&format!("; joints {zeroed:?} were zeroed before the failure"));
        e.extra.insert("joints".into(), json!(rows));
        e.extra.insert("invalidated".into(), inv);
        e.extra.insert("log".into(), json!(log));
        return Err(e);
    }
    let tool = pub_cfg.calibration_tools().motor_zero;
    if !verified {
        let mut e = CalibError::new(500, "MOTOR_ZERO_UNVERIFIED", "not every joint read within verification tolerance".to_string());
        e.extra.insert("joints".into(), json!(rows));
        e.extra.insert("invalidated".into(), inv);
        e.extra.insert("log".into(), json!(log));
        return Err(e);
    }
    Ok(json!({"joints": rows, "invalidated": inv, "log": log, "zero_method": "motor", "method": tool.support.method, "persist": tool.support.persist,
        "persist_verified": tool.support.persist_verified, "torque": "off"}))
}

async fn verify_executor(sup: &ArmSupervisor, req: &MotorZeroRequest, before: &[f32], r: &[f32]) -> Vec<Value> {
    let deadline = Instant::now() + VERIFY_TIMEOUT;
    loop {
        let mut all = true;
        let obs = sup.hub().latest().map(|s| s.obs.clone());
        let rows: Vec<Value> = req.joints.iter().map(|&j| {
            let after = obs.as_ref().and_then(|o| o.positions.get(j).copied()).unwrap_or(f32::NAN);
            let ok = after.is_finite() && (after - r[j]).abs() < verify_rad();
            all &= ok;
            json!({"joint": j + 1, "before_rad": before[j], "after_rad": after, "reference_rad": r[j], "sent": true, "fresh": true, "verified": ok})
        }).collect();
        if all || Instant::now() >= deadline {
            return rows;
        }
        tokio::time::sleep(POLL).await;
    }
}

async fn restore_executor(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    name: String,
    auth: Option<&crate::calibration::CalibCommitAuth>,
) -> Result<Value, CalibError> {
    let pub_cfg = sup.published_config().await.map_err(|e| CalibError::new(503, "CONFIG_UNAVAILABLE", e.to_string()))?;
    if pub_cfg.has_driver {
        return Err(CalibError::new(
            503,
            "EXECUTOR_CAPABILITY_UNSUPPORTED",
            "hardware joint zero restore requires synchronous driver access unsupported under the RT executor",
        ));
    }
    check_executor_auth(sup, auth, "restore")?;
    let dir = deps.backup_dir.join(&name);
    let manifest: Value = std::fs::read_to_string(dir.join("manifest.json")).ok().and_then(|t| serde_json::from_str(&t).ok())
        .ok_or_else(|| CalibError::new(404, "BACKUP_NOT_FOUND", format!("backup {name}: no manifest.json")))?;
    if manifest.get("reason").and_then(Value::as_str) != Some("software_zero") {
        return Err(invalid(format!("backup {name} is not a software zero backup")));
    }
    let target: Option<crate::joint_zero::JointZero> = match crate::calibration::read_if_file(&dir.join("calibration.json")) {
        Ok(None) => None,
        Ok(Some(t)) => serde_json::from_str::<crate::calibration::CalibrationFile>(&t).map_err(|e| invalid(format!("backup {name}: calibration.json: {e}")))?.joint_zero,
        Err(e) => return Err(CalibError::new(500, "PERSIST_FAILED", format!("backup {name}: {e}"))),
    };
    let n = pub_cfg.config.dof;
    let new = target.as_ref().map_or_else(|| vec![0.0; n], |z| z.offsets());
    if let Some(z) = &target { z.validate(Some(n)).map_err(|e| invalid(format!("backup {name}: {e}")))?; }
    let old = pub_cfg.joint_zero_offsets.clone();
    check_executor_auth(sup, auth, "backup")?;
    let backup = write_zero_backup(deps, "joint_zero_restore", &old).await.map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("backup: {e}; nothing was changed")))?;
    check_executor_auth(sup, auth, "apply")?;
    let joints: Vec<usize> = (0..new.len()).filter(|&j| old[j] != new[j]).collect();
    let changes: Vec<f32> = joints.iter().map(|&j| old[j] - new[j]).collect();
    let zeroed: Vec<usize> = joints.iter().map(|j| j + 1).collect();
    let entry = Some(target.filter(|z| !z.is_zero()));
    let inv = crate::calibration::commit_joint_zero_and_invalidation(
        sup,
        deps,
        &zeroed,
        &changes,
        "restored",
        entry,
        Some(new.clone()),
        auth,
        true,
    )
    .await
    .map_err(|e| e.at("apply"))?;
    Ok(json!({"restored": name, "backup": backup.display().to_string(), "offsets_before": old, "offsets_after": new, "invalidated": inv}))
}

/// Wait up to [`VERIFY_TIMEOUT`] for every zeroed joint to read within the tolerance of its
/// reference `r` in feedback newer than its command; one row per requested joint.
async fn verify(sup: &ArmSupervisor, req: &MotorZeroRequest, before: &[f32], r: &[f32], sent: &[(usize, Instant)]) -> Vec<Value> {
    let deadline = Instant::now() + VERIFY_TIMEOUT;
    loop {
        let (rows, all) = {
            let inner = sup.inner().read().await;
            let fb = feedback(&inner);
            let mut all = true;
            let rows: Vec<Value> = req.joints.iter().map(|&j| {
                let cmd = sent.iter().find(|(s, _)| *s == j).map(|(_, t)| *t);
                let after = inner.measured_positions.get(j).copied().unwrap_or(f32::NAN);
                let fresh = cmd.is_some_and(|t| fresh_since(&fb, j, t));
                let ok = cmd.is_some() && fresh && (after - r[j]).abs() < verify_rad();
                all &= ok || cmd.is_none();
                json!({"joint": j + 1, "before_rad": before[j], "after_rad": after, "reference_rad": r[j], "sent": cmd.is_some(), "fresh": fresh, "verified": ok})
            }).collect();
            (rows, all)
        };
        if all || Instant::now() >= deadline { return rows; }
        tokio::time::sleep(POLL).await;
    }
}

/// Append the run to the log; returns its path (or the error).
async fn write_log(deps: &CalibrationDeps, started: &str, req: &MotorZeroRequest, before: &[f32], rows: &[Value], result: &str, inv: &Value) -> Value {
    let path = deps.backup_dir.join(LOG_NAME);
    let line = json!({"time": started, "joints": req.joints.iter().map(|j| j + 1).collect::<Vec<_>>(), "readings_before": before,
        "force": req.force, "result": result, "rows": rows, "invalidated": inv}).to_string();
    let p = path.clone();
    let r = tokio::task::spawn_blocking(move || -> std::io::Result<()> {
        use std::io::Write;
        std::fs::create_dir_all(p.parent().unwrap_or(std::path::Path::new(".")))?;
        let mut f = std::fs::OpenOptions::new().create(true).append(true).open(&p)?;
        writeln!(f, "{line}")?;
        f.sync_all()
    }).await;
    match r {
        Ok(Ok(())) => json!(path.display().to_string()),
        Ok(Err(e)) => { tracing::warn!("motor zero log {}: {e}", path.display()); json!({"error": e.to_string()}) }
        Err(e) => json!({"error": e.to_string()}),
    }
}
