//! Motor zero calibration: WS `/api/v1/calibration/motor_zero {joints, dry_run, force}` makes
//! the current position of the given joints (1-based) their zero.
//!
//! Checked under the lock: the driver supports it (`calibration_tools.motor_zero`), every joint
//! is in `calibration_tools.motor_zero.joints`, the caller holds the lease (WS command), torque
//! is off, no action or motion runs (MOTOR_ZERO_ARMED), the arm is not in Fault / EStop
//! (MOTOR_ZERO_FAULT), the feedback is fresh (MOTOR_ZERO_NO_FEEDBACK) and, without
//! `force`, no joint reads more than [`MOTOR_ZERO_MAX_OFFSET_DEG`] from 0 (MOTOR_ZERO_FAR).
//! `dry_run` returns the readings and the predicted change and changes nothing.
//!
//! A real run (its own task, `calibration_busy` and `motor_zero_busy` set: motion, torque,
//! clear_fault and configuration writes are refused meanwhile) follows the official sequence
//! (reBotArm_control_py `RebotArm.set_zero`): disable every motor, wait for fresh feedback
//! confirming each requested motor disabled, send the set-zero command per joint 0.1 s apart,
//! then wait up to 1 s until every zeroed joint reads within [`MOTOR_ZERO_VERIFY_MRAD`] of 0 in
//! feedback newer than its command (MOTOR_ZERO_UNVERIFIED otherwise, with the readings). It
//! never moves the arm; torque stays off. Each run is appended to `motor_zero.jsonl` in the
//! calibration backup directory, and the calibrations measured with the old zero are
//! invalidated ([`crate::calibration::invalidate_after_motor_zero`]): the kinematic correction is
//! removed (live and in `calibration.json`) and the camera calibration is marked stale.
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
}

fn invalid(m: impl Into<String>) -> CalibError { CalibError::new(400, "INVALID_COMMAND", m) }

pub fn parse(body: &Value) -> Result<MotorZeroRequest, CalibError> {
    let o = body.as_object().ok_or_else(|| invalid("Command body must be an object"))?;
    for k in o.keys() {
        if !["joints", "dry_run", "force"].contains(&k.as_str()) { return Err(invalid(format!("unknown field: {k}"))); }
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
    Ok(MotorZeroRequest { joints, dry_run: flag("dry_run")?, force: flag("force")? })
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

fn joint_rows(req: &MotorZeroRequest, q: &[f32]) -> Vec<Value> {
    req.joints.iter().map(|&j| json!({"joint": j + 1, "name": format!("J{}", j + 1), "reading_rad": q[j],
        "reading_deg": f64::from(q[j]).to_degrees(), "after_rad": 0.0, "change_rad": -q[j], "far": q[j].abs() > max_offset_rad()})).collect()
}

/// What a run would invalidate now.
fn invalidates(inner: &SupervisorInner) -> Value {
    json!({"kinematics": inner.config.arm_model.correction().is_some(), "camera": inner.camera_calibrated()})
}

/// Under the lock: parse and check. Ok(None) = dry run reply; Ok(Some) = a real run to start
/// (the caller sets the busy flags).
pub fn check(inner: &SupervisorInner, body: &Value) -> Result<(MotorZeroRequest, Option<Value>), CalibError> {
    let req = parse(body)?;
    let tool = inner.calibration_tools().motor_zero;
    if !tool.support.supported {
        return Err(CalibError::new(400, "UNSUPPORTED", format!("motor zero is not supported: {}", tool.support.reason.unwrap_or_default())));
    }
    if let Some(j) = req.joints.iter().find(|j| !tool.joints.contains(&(*j + 1))) {
        return Err(CalibError::new(400, "JOINT_UNSUPPORTED", format!("joint {} is not in calibration_tools.motor_zero.joints {:?}", j + 1, tool.joints)));
    }
    if inner.calibration_busy { return Err(CalibError::new(409, "CONFIG_BUSY", "a calibration apply, restore or motor zero is running")); }
    state_checks(inner)?;
    let q = readings(inner)?;
    let rows = joint_rows(&req, &q);
    if req.dry_run {
        let far: Vec<usize> = req.joints.iter().filter(|&&j| q[j].abs() > max_offset_rad()).map(|j| j + 1).collect();
        return Ok((req, Some(json!({"dry_run": true, "joints": rows, "needs_force": !far.is_empty(), "far_joints": far,
            "invalidates": invalidates(inner), "method": tool.support.method, "persist": tool.support.persist,
            "persist_verified": tool.support.persist_verified, "max_offset_deg": MOTOR_ZERO_MAX_OFFSET_DEG}))));
    }
    let far: Vec<String> = req.joints.iter().filter(|&&j| q[j].abs() > max_offset_rad())
        .map(|&j| format!("J{} reads {:.1} deg", j + 1, f64::from(q[j]).to_degrees())).collect();
    if !far.is_empty() && !req.force {
        let mut e = CalibError::new(409, "MOTOR_ZERO_FAR", format!("{} (more than {MOTOR_ZERO_MAX_OFFSET_DEG} deg from 0); \
            check the arm is at the zero pose, or send force: true for a lost zero", far.join(", ")));
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

/// The real run (after [`check`]; the caller holds the busy flags and clears them afterwards).
pub async fn run(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, req: MotorZeroRequest, check: &LeaseCheck) -> Result<Value, CalibError> {
    let started = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
    // 1. readings, then disable every motor (official sequence), torque stays off
    let (before, t_disable) = locked_step(sup, check, "disable", |inner| {
        let q = readings(inner)?;
        if let Some(d) = inner.driver.as_ref() {
            let mut drv = d.lock().map_err(|_| CalibError::new(500, "INTERNAL", "driver unavailable"))?;
            drv.set_torque(false).map_err(|e| CalibError::new(500, "MOTOR_ZERO_NOT_DISABLED", format!("disable failed: {e}")))?;
        }
        Ok((q, Instant::now()))
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
    // 3. one set-zero command per joint, spaced
    let mut sent: Vec<(usize, Instant)> = Vec::new();
    for (k, &j) in req.joints.iter().enumerate() {
        if k > 0 { tokio::time::sleep(COMMAND_SPACING).await; }
        let r = locked_step(sup, check, "set_zero", |inner| {
            inner.set_motor_zero(j).map_err(|e| CalibError::new(500, "MOTOR_ZERO_FAILED", format!("J{}: {e}", j + 1)))
        }).await;
        match r {
            Ok(()) => sent.push((j, Instant::now())),
            Err(mut e) if sent.is_empty() => { e.message.push_str("; no zero was changed"); return Err(e); }
            Err(e) => {
                // some zeros changed: log and invalidate before reporting the failure
                let rows = verify(sup, &req, &before, &sent).await;
                let inv = crate::calibration::invalidate_after_motor_zero(sup, deps, &sent.iter().map(|(j, _)| j + 1).collect::<Vec<_>>()).await;
                let log = write_log(deps, &started, &req, &before, &rows, "partial", &inv).await;
                let mut e = e;
                e.message.push_str(&format!("; joints {:?} were zeroed before the failure", sent.iter().map(|(j, _)| j + 1).collect::<Vec<_>>()));
                e.extra.insert("joints".into(), json!(rows));
                e.extra.insert("invalidated".into(), inv);
                e.extra.insert("log".into(), json!(log));
                return Err(e);
            }
        }
    }
    // 4. verification from feedback newer than each command
    let rows = verify(sup, &req, &before, &sent).await;
    let verified = rows.iter().all(|r| r["verified"] == json!(true));
    // 5. invalidate what was measured with the old zero, log
    let zeroed: Vec<usize> = sent.iter().map(|(j, _)| j + 1).collect();
    let inv = crate::calibration::invalidate_after_motor_zero(sup, deps, &zeroed).await;
    let log = write_log(deps, &started, &req, &before, &rows, if verified { "verified" } else { "unverified" }, &inv).await;
    let tool = sup.inner().read().await.calibration_tools().motor_zero;
    tracing::warn!("Motor zero of joints {zeroed:?} (readings before {:?}): {}", req.joints.iter().map(|&j| before[j]).collect::<Vec<_>>(),
        if verified { "verified" } else { "NOT verified" });
    if !verified {
        let mut e = CalibError::new(500, "MOTOR_ZERO_UNVERIFIED", format!("the set-zero commands were sent but not every joint read within {MOTOR_ZERO_VERIFY_MRAD} mrad of 0 \
            within {} ms; check the readings and run the tool again", VERIFY_TIMEOUT.as_millis()));
        e.extra.insert("joints".into(), json!(rows));
        e.extra.insert("invalidated".into(), inv);
        e.extra.insert("log".into(), json!(log));
        return Err(e);
    }
    Ok(json!({"joints": rows, "invalidated": inv, "log": log, "method": tool.support.method, "persist": tool.support.persist,
        "persist_verified": tool.support.persist_verified, "torque": "off"}))
}

/// Wait up to [`VERIFY_TIMEOUT`] for every zeroed joint to read within the tolerance in
/// feedback newer than its command; one row per requested joint.
async fn verify(sup: &ArmSupervisor, req: &MotorZeroRequest, before: &[f32], sent: &[(usize, Instant)]) -> Vec<Value> {
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
                let ok = cmd.is_some() && fresh && after.abs() < verify_rad();
                all &= ok || cmd.is_none();
                json!({"joint": j + 1, "before_rad": before[j], "after_rad": after, "sent": cmd.is_some(), "fresh": fresh, "verified": ok})
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
