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
        let fresh = d.with_ref(|drv| drv.is_feedback_valid()).unwrap_or(false);
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
    if inner.calibration_busy || inner.core.config().held().is_some() {
        return Err(CalibError::new(
            409,
            "CONFIG_BUSY",
            "a calibration apply, restore, motor zero, or configuration change is running",
        ));
    }
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
    d.with_ref(|drv| Feedback { ages: drv.feedback_ages(), status: drv.motor_status() })
}

/// Sample newer than `since` for joint `j` (always true for the mock follower).
fn fresh_since(fb: &Option<Feedback>, j: usize, since: Instant) -> bool {
    match fb {
        None => true,
        Some(f) => f.ages.get(j).copied().flatten().is_some_and(|age| Instant::now().checked_sub(age).is_some_and(|t| t > since)),
    }
}

async fn locked_step<T>(sup: &ArmSupervisor, check: &LeaseCheck, grant: crate::rt_core::config_iface::GrantId, step: &'static str,
    f: impl FnOnce(&mut SupervisorInner) -> Result<T, CalibError>) -> Result<T, CalibError> {
    let mut inner = sup.inner().write().await;
    let at = |mut e: CalibError| { e.extra.insert("step".into(), json!(step)); e };
    if !inner.core.config().held().is_some_and(|h| h.grant == Some(grant)) {
        return Err(at(CalibError::new(409, "CONFIG_BUSY", "configuration reservation was lost during zero")));
    }
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

/// The real run (after [`check`]; the caller holds the busy flags and clears them afterwards).
pub async fn run(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, req: MotorZeroRequest, check: &LeaseCheck) -> Result<Value, CalibError> {
    if sup.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return Err(CalibError::new(
            409,
            "EXECUTOR_UNSUPPORTED",
            format!("{PATH} is not available while the RT executor owns the arm; use the legacy owner"),
        ));
    }
    let zero_guard = crate::rt_core::requests::config::LegacyZeroReservationGuard::try_acquire(sup).await?;
    let grant = zero_guard.grant();
    let res = if req.method.as_deref() == Some("software") {
        run_software(sup, deps, req, check, grant).await
    } else {
        run_motor(sup, deps, req, check, grant).await
    };
    zero_guard.release().await;
    res
}

async fn run_motor(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, req: MotorZeroRequest, check: &LeaseCheck, grant: crate::rt_core::config_iface::GrantId) -> Result<Value, CalibError> {
    let started = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
    // 1. readings, then disable every motor (official sequence), torque stays off
    let (before, r, t_disable) = locked_step(sup, check, grant, "disable", |inner| {
        let q = readings(inner)?;
        if let Some(d) = inner.driver.as_mut() {
            d.with_mut(|drv| {
                let started_at = drv.clock_now();
                drv.disable_torque(crate::driver::DisableKind::DisableRequested { started_at })
            })
                .ok_or_else(|| CalibError::new(500, "INTERNAL", "driver unavailable"))?
                .map_err(|e| CalibError::new(500, "MOTOR_ZERO_NOT_DISABLED", format!("disable failed: {e}")))?;
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
    crate::calibration::invalidate_mit_before_zero(sup, deps, &one_based, "motor").await?;
    // 3. one set-zero command per joint, spaced; the joint zero offset of each zeroed joint
    // becomes -reference (the motor reads 0 at the reference)
    let mut sent: Vec<(usize, Instant)> = Vec::new();
    let mut failure = None;
    for (k, &j) in req.joints.iter().enumerate() {
        if k > 0 { tokio::time::sleep(COMMAND_SPACING).await; }
        let rj = r[j];
        let res = locked_step(sup, check, grant, "set_zero", |inner| {
            inner.set_motor_zero(j).map_err(|e| CalibError::new(500, "MOTOR_ZERO_FAILED", format!("J{}: {e}", j + 1)))?;
            let mut off = inner.joint_zero_offsets.clone();
            off[j] = -rj;
            inner.set_joint_zero_offsets(&off).map_err(|e| CalibError::new(500, "MOTOR_ZERO_FAILED", format!("J{} offset: {e}", j + 1)))
        }).await;
        match res {
            Ok(()) => sent.push((j, Instant::now())),
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
    let inv = crate::calibration::invalidate_after_zero(sup, deps, &zeroed, &changes, "motor", entry).await;
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

/// Software zero: average the readings, back up calibration.json, install the offsets, verify.
async fn run_software(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, req: MotorZeroRequest, check: &LeaseCheck, grant: crate::rt_core::config_iface::GrantId) -> Result<Value, CalibError> {
    let started = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
    // 1. capture: mean of fresh readings over SOFTWARE_CAPTURE; the arm must stay still
    let (r, n) = locked_step(sup, check, grant, "capture", |inner| { readings(inner)?; Ok((reference(inner), inner.config.dof)) }).await?;
    let t0 = Instant::now();
    let (mut sum, mut lo, mut hi, mut count) = (vec![0.0f64; n], vec![f32::INFINITY; n], vec![f32::NEG_INFINITY; n], 0u32);
    while t0.elapsed() < crate::joint_zero::SOFTWARE_CAPTURE || count == 0 {
        let q = locked_step(sup, check, grant, "capture", |inner| readings(inner)).await?;
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
    let (old, new) = {
        let inner = sup.inner().read().await;
        let old = inner.joint_zero_offsets.clone();
        let mut new = old.clone();
        for &j in &req.joints { new[j] = old[j] + (before[j] - r[j]); }
        (old, new)
    };
    if let Some(j) = req.joints.iter().find(|&&j| f64::from(new[j].abs()) > crate::joint_zero::MAX_OFFSET_RAD) {
        return Err(CalibError::new(409, "MOTOR_ZERO_FAR", format!("J{} would need a joint zero offset of {:.1} deg; nothing was changed", j + 1, f64::from(new[*j]).to_degrees())));
    }
    // 3. backup, 4. install, 5. verify, 6. store + invalidate, log
    let backup = write_zero_backup(deps, "software_zero", &old).await.map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("backup: {e}; nothing was changed")))?;
    // validated and backed up: invalidate a qualification of the zeroed joints on disk before the install
    let one_based: Vec<usize> = req.joints.iter().map(|j| j + 1).collect();
    crate::calibration::invalidate_mit_before_zero(sup, deps, &one_based, "software").await?;
    let t_apply = locked_step(sup, check, grant, "apply", |inner| {
        inner.set_joint_zero_offsets(&new).map_err(|e| CalibError::new(500, "MOTOR_ZERO_FAILED", e))?;
        Ok(Instant::now())
    }).await?;
    let sent: Vec<(usize, Instant)> = req.joints.iter().map(|&j| (j, t_apply)).collect();
    let rows = verify(sup, &req, &before, &r, &sent).await;
    let verified = rows.iter().all(|r| r["verified"] == json!(true));
    let zeroed: Vec<usize> = req.joints.iter().map(|j| j + 1).collect();
    let changes: Vec<f32> = req.joints.iter().map(|&j| r[j] - before[j]).collect();
    let entry = { let inner = sup.inner().read().await; zero_entry(&inner, &new, "software") };
    let inv = crate::calibration::invalidate_after_zero(sup, deps, &zeroed, &changes, "software", entry).await;
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

/// A backup of `calibration.json` (and companion `poses.json` / `gripper_calib.json` when
/// present) before a joint zero change: `<backup_dir>/<time>/` with `manifest.json`
/// (`reason`, `joint_zero_offsets`) and durably fsynced backup directory entries.
async fn write_zero_backup_with_companions(
    deps: &CalibrationDeps,
    reason: &str,
    offsets: &[f32],
    poses_path: Option<std::path::PathBuf>,
    gripper_calib_path: Option<std::path::PathBuf>,
) -> Result<std::path::PathBuf, String> {
    let (root, calib) = (deps.backup_dir.clone(), deps.path.clone());
    let manifest = json!({"created_at": chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true), "reason": reason, "joint_zero_offsets": offsets});
    crate::calibration::blocking(move || {
        let durable_root = calib.parent().filter(|p| p.is_dir());
        crate::persist::create_dir_all_durable_under(&root, durable_root)?;
        let dir = root.join(crate::calibration::backup_name(&root));
        std::fs::create_dir(&dir)?;
        let mut files = vec!["manifest.json"];
        if let Some(t) = crate::calibration::read_if_file(&calib)? {
            crate::persist::write_atomic(&dir.join("calibration.json"), &t)?;
            files.push("calibration.json");
        }
        if let Some(p) = poses_path.as_deref() {
            if let Some(t) = crate::calibration::read_if_file(p)? {
                crate::persist::write_atomic(&dir.join("poses.json"), &t)?;
                files.push("poses.json");
            }
        }
        if let Some(g) = gripper_calib_path.as_deref() {
            if let Some(t) = crate::calibration::read_if_file(g)? {
                crate::persist::write_atomic(&dir.join("gripper_calib.json"), &t)?;
                files.push("gripper_calib.json");
            }
        }
        let mut manifest = manifest;
        manifest["files"] = json!(files);
        let manifest_path = dir.join("manifest.json");
        crate::persist::write_atomic(&manifest_path, &serde_json::to_string_pretty(&manifest).unwrap_or_default())?;
        if let crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) =
            crate::persist::PendingDirSync::for_path(&manifest_path).sync_parent_dir()
        {
            return Err(std::io::Error::other(format!("zero backup dir fsync failed: {w}")));
        }
        if let crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) =
            crate::persist::PendingDirSync::for_path(&dir).sync_parent_dir()
        {
            return Err(std::io::Error::other(format!("zero backup root fsync failed: {w}")));
        }
        Ok(dir)
    }).await
}

async fn write_zero_backup(deps: &CalibrationDeps, reason: &str, offsets: &[f32]) -> Result<std::path::PathBuf, String> {
    write_zero_backup_with_companions(deps, reason, offsets, None, None).await
}

/// Under the lock: check a joint zero restore `{backup}` (torque off, no action, lease).
pub fn check_restore(inner: &SupervisorInner, body: &Value) -> Result<String, CalibError> {
    let name = body.get("backup").and_then(Value::as_str)
        .filter(|n| !n.is_empty() && n.len() <= 64 && n.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_'))
        .ok_or_else(|| invalid("backup must be the name of a software zero backup (GET /api/v1/calibration backups)"))?.to_string();
    if body.as_object().is_some_and(|o| o.keys().any(|k| k != "backup")) { return Err(invalid("only the field backup is allowed")); }
    if inner.config.zero_reference.is_none() { return Err(CalibError::new(400, "UNSUPPORTED", "the joint zero needs a profile zero_reference")); }
    if inner.calibration_busy || inner.core.config().held().is_some() {
        return Err(CalibError::new(
            409,
            "CONFIG_BUSY",
            "a calibration apply, restore, zero, or configuration change is running",
        ));
    }
    state_checks(inner)?;
    Ok(name)
}

/// WS `/api/v1/calibration/joint_zero/restore {backup}`: put back the joint zero a software zero
/// backup holds (its `calibration.json` `joint_zero`, none = offsets 0), with a new backup first;
/// invalidates like a zero.
pub async fn restore(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, name: String, check: &LeaseCheck) -> Result<Value, CalibError> {
    if sup.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return Err(CalibError::new(
            409,
            "EXECUTOR_UNSUPPORTED",
            format!("{RESTORE_PATH} is not available while the RT executor owns the arm; use the legacy owner"),
        ));
    }
    let zero_guard = crate::rt_core::requests::config::LegacyZeroReservationGuard::try_acquire(sup).await?;
    let grant = zero_guard.grant();
    let res = restore_with_grant(sup, deps, name, check, grant).await;
    zero_guard.release().await;
    res
}

async fn restore_with_grant(sup: &Arc<ArmSupervisor>, deps: &CalibrationDeps, name: String, check: &LeaseCheck, grant: crate::rt_core::config_iface::GrantId) -> Result<Value, CalibError> {
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
    let (old, new, r) = locked_step(sup, check, grant, "restore", |inner| {
        let n = inner.config.dof;
        let new = target.as_ref().map_or_else(|| vec![0.0; n], |z| z.offsets());
        if let Some(z) = &target { z.validate(Some(n)).map_err(|e| invalid(format!("backup {name}: {e}")))?; }
        Ok((inner.joint_zero_offsets.clone(), new, reference(inner)))
    }).await?;
    let changed: Vec<usize> = (0..old.len()).filter(|&j| old.get(j) != new.get(j)).map(|j| j + 1).collect();
    let backup = write_zero_backup(deps, "joint_zero_restore", &old).await.map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("backup: {e}; nothing was changed")))?;
    crate::calibration::invalidate_mit_before_zero(sup, deps, &changed, "restored").await?;
    locked_step(sup, check, grant, "apply", |inner| inner.set_joint_zero_offsets(&new).map_err(|e| CalibError::new(500, "MOTOR_ZERO_FAILED", e))).await?;
    let joints: Vec<usize> = (0..new.len()).filter(|&j| old[j] != new[j]).collect();
    let changes: Vec<f32> = joints.iter().map(|&j| old[j] - new[j]).collect();
    let entry = Some(target.filter(|z| !z.is_zero()));
    let inv = crate::calibration::invalidate_after_zero(sup, deps, &joints.iter().map(|j| j + 1).collect::<Vec<_>>(), &changes, "restored", entry).await;
    let _ = r;
    tracing::warn!("Joint zero restored from backup {name}: offsets {old:?} -> {new:?} rad");
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

// ============================================================================
// Executor-mode software joint zero & joint-zero restore (PR C-F1 part 1)
// ============================================================================

/// Outcome of synchronous executor-mode software-zero preflight (`check_software_executor`).
pub(crate) enum ExecutorZeroStart {
    DryRunReply(Value),
    Run(MotorZeroRequest, crate::calibration::ExecutorTxnGuard),
}

fn reference_from_published(pub_cfg: &crate::rt_core::requests::config::PublishedConfig) -> Vec<f32> {
    pub_cfg
        .config
        .zero_reference
        .as_ref()
        .map(|_| pub_cfg.config.home_positions.clone())
        .unwrap_or_else(|| vec![0.0; pub_cfg.config.dof])
}

fn invalidates_from_published(
    pub_cfg: &crate::rt_core::requests::config::PublishedConfig,
    req: &MotorZeroRequest,
    q: &[f32],
    r: &[f32],
) -> Value {
    let moved: Vec<usize> = req
        .joints
        .iter()
        .copied()
        .filter(|&j| f64::from((q[j] - r[j]).abs()) > crate::joint_zero::STALE_THRESHOLD_RAD)
        .collect();
    let grip = pub_cfg.config.gripper_index();
    json!({
        "kinematics": pub_cfg.config.arm_model.correction().is_some(),
        "camera": pub_cfg.camera_calibrated(),
        "poses": if moved.is_empty() {
            Vec::new()
        } else {
            pub_cfg.poses.list.poses.iter().map(|p| p.name.clone()).collect()
        },
        "gripper_calibration": grip.is_some_and(|g| moved.contains(&g))
            && (pub_cfg.gripper_calib_offset.is_some() || pub_cfg.gripper_calib_jaw_opening.is_some()),
    })
}

fn zero_entry_from_published(
    pub_cfg: &crate::rt_core::requests::config::PublishedConfig,
    new: &[f32],
    method: &str,
) -> Option<Option<crate::joint_zero::JointZero>> {
    let name = pub_cfg
        .config
        .zero_reference
        .as_ref()
        .map_or("rest", |r| r.name.as_str())
        .to_string();
    let stored = pub_cfg
        .calibration
        .as_ref()
        .and_then(|c| c.joint_zero.as_ref())
        .is_some();
    if new.iter().all(|o| *o == 0.0) {
        return stored.then_some(None);
    }
    Some(Some(crate::joint_zero::JointZero::new(new, method, &name)))
}

/// Synchronous preflight for executor-mode joint zero (`/api/v1/calibration/motor_zero`,
/// both `method == "motor"` and `method == "software"`).
pub(crate) async fn check_software_executor(
    sup: &Arc<ArmSupervisor>,
    body: &Value,
    auth: &crate::calibration::ExecutorCalibAuth,
) -> Result<ExecutorZeroStart, CalibError> {
    let pub_cfg = sup.published_config().await.map_err(CalibError::from)?;
    let mut req = parse(body)?;
    let tool = pub_cfg.calibration_tools().motor_zero;
    if !tool.support.supported {
        return Err(CalibError::new(
            400,
            "UNSUPPORTED",
            format!("motor zero is not supported: {}", tool.support.reason.unwrap_or_default()),
        ));
    }
    let method = method_of(&req, &tool);
    req.method = Some(method.into());
    if method == "motor" && !tool.motor {
        return Err(CalibError::new(
            400,
            "UNSUPPORTED",
            format!(
                "motor zero is not supported by this driver; use method \"software\" ({})",
                tool.support.notes.join("; ")
            ),
        ));
    }
    if method == "software" && !tool.software {
        return Err(CalibError::new(
            400,
            "UNSUPPORTED",
            "the software zero needs a profile zero_reference",
        ));
    }
    if let Some(j) = req.joints.iter().find(|j| !tool.joints.contains(&(*j + 1))) {
        return Err(CalibError::new(
            400,
            "JOINT_UNSUPPORTED",
            format!("joint {} is not in calibration_tools.motor_zero.joints {:?}", j + 1, tool.joints),
        ));
    }
    if let Err((_, arm_err)) = (auth.auth_check)() {
        return Err(CalibError::from(arm_err));
    }
    let txn_id = crate::rt_core::requests::config::next_calibration_txn_id();
    let cfg_auth = crate::rt_core::requests::config::ConfigAuthContext {
        admission: auth.admission,
        lease_holder: auth.lease_holder,
    };
    let mut pre_guard = (!req.dry_run)
        .then(|| crate::calibration::ExecutorTxnGuard::new(Arc::clone(sup), txn_id));
    let txn_kind = if method == "motor" {
        crate::rt_core::requests::config::CalibrationTxnKind::MotorZero {
            set_busy: !req.dry_run,
        }
    } else {
        crate::rt_core::requests::config::CalibrationTxnKind::SoftwareZero {
            set_busy: !req.dry_run,
        }
    };
    let sample = match sup
        .dispatch_config_request(
            crate::rt_core::requests::config::ConfigRequest::BeginCalibrationTxn {
                txn_id,
                auth: cfg_auth,
                operator_grant: auth.operator_grant,
                expected_epoch: auth.admission.estop_epoch,
                expected_stop_gen: auth.admission.stop_gen,
                kind: txn_kind,
            },
        )
        .await
    {
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Ok(s))) => s,
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Err(e))) => {
            if let Some(g) = pre_guard.take() {
                let _ = g.disarm();
            }
            return Err(crate::calibration::calib_error_from_config(e));
        }
        Err(e) => {
            if let Some(g) = pre_guard.take() {
                g.release().await;
            }
            return Err(crate::calibration::calib_error_from_config(e));
        }
        _ => {
            if let Some(g) = pre_guard.take() {
                g.release().await;
            }
            return Err(CalibError::new(500, "INTERNAL", "unexpected begin calibration reply"));
        }
    };
    let q = sample.positions[..sample.dof].to_vec();
    let r = reference_from_published(&pub_cfg);
    let rows = joint_rows(&req, &q, &r);
    let persist = if method == "software" {
        Some("calibration_file".to_string())
    } else {
        tool.support.persist.clone()
    };
    if req.dry_run {
        let far_j: Vec<usize> = req
            .joints
            .iter()
            .filter(|&&j| far(&q, &r, j))
            .map(|j| j + 1)
            .collect();
        return Ok(ExecutorZeroStart::DryRunReply(json!({
            "dry_run": true,
            "joints": rows,
            "needs_force": !far_j.is_empty(),
            "far_joints": far_j,
            "invalidates": invalidates_from_published(&pub_cfg, &req, &q, &r),
            "zero_method": method,
            "method": tool.support.method,
            "persist": persist,
            "persist_verified": method == "software" || tool.support.persist_verified,
            "max_offset_deg": MOTOR_ZERO_MAX_OFFSET_DEG,
        })));
    }
    let guard = pre_guard.expect("non-dry-run software zero holds txn guard");
    let far_s: Vec<String> = req
        .joints
        .iter()
        .filter(|&&j| far(&q, &r, j))
        .map(|&j| {
            format!(
                "J{} reads {:.1} deg (reference {:.1} deg)",
                j + 1,
                f64::from(q[j]).to_degrees(),
                f64::from(r[j]).to_degrees()
            )
        })
        .collect();
    if !far_s.is_empty() && !req.force {
        guard.release().await;
        let mut e = CalibError::new(
            409,
            "MOTOR_ZERO_FAR",
            format!(
                "{} (more than {MOTOR_ZERO_MAX_OFFSET_DEG} deg from the reference); \
                check the arm is in the rest pose, or send force: true for a lost zero",
                far_s.join(", ")
            ),
        );
        e.extra.insert("joints".into(), json!(rows));
        return Err(e);
    }
    sup.published_config_hub().update(|cfg| {
        cfg.calibration_busy = true;
        cfg.calibration_txn_id = Some(txn_id);
    });
    Ok(ExecutorZeroStart::Run(req, guard))
}

/// Synchronous preflight for executor-mode `/api/v1/calibration/joint_zero/restore`.
pub(crate) async fn check_restore_executor(
    sup: &Arc<ArmSupervisor>,
    body: &Value,
    auth: &crate::calibration::ExecutorCalibAuth,
) -> Result<(String, crate::calibration::ExecutorTxnGuard), CalibError> {
    let name = body
        .get("backup")
        .and_then(Value::as_str)
        .filter(|n| {
            !n.is_empty()
                && n.len() <= 64
                && n.chars()
                    .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
        })
        .ok_or_else(|| {
            invalid("backup must be the name of a software zero backup (GET /api/v1/calibration backups)")
        })?
        .to_string();
    if body.as_object().is_some_and(|o| o.keys().any(|k| k != "backup")) {
        return Err(invalid("only the field backup is allowed"));
    }
    let pub_cfg = sup.published_config().await.map_err(CalibError::from)?;
    if pub_cfg.config.zero_reference.is_none() {
        return Err(CalibError::new(
            400,
            "UNSUPPORTED",
            "the joint zero needs a profile zero_reference",
        ));
    }
    if let Err((_, arm_err)) = (auth.auth_check)() {
        return Err(CalibError::from(arm_err));
    }
    let txn_id = crate::rt_core::requests::config::next_calibration_txn_id();
    let cfg_auth = crate::rt_core::requests::config::ConfigAuthContext {
        admission: auth.admission,
        lease_holder: auth.lease_holder,
    };
    let pre_guard = crate::calibration::ExecutorTxnGuard::new(Arc::clone(sup), txn_id);
    match sup
        .dispatch_config_request(
            crate::rt_core::requests::config::ConfigRequest::BeginCalibrationTxn {
                txn_id,
                auth: cfg_auth,
                operator_grant: auth.operator_grant,
                expected_epoch: auth.admission.estop_epoch,
                expected_stop_gen: auth.admission.stop_gen,
                kind: crate::rt_core::requests::config::CalibrationTxnKind::JointZeroRestore,
            },
        )
        .await
    {
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Ok(_))) => {
            sup.published_config_hub().update(|cfg| {
                cfg.calibration_busy = true;
                cfg.calibration_txn_id = Some(txn_id);
            });
            Ok((name, pre_guard))
        }
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Err(e))) => {
            let _ = pre_guard.disarm();
            Err(crate::calibration::calib_error_from_config(e))
        }
        Err(e) => {
            pre_guard.release().await;
            Err(crate::calibration::calib_error_from_config(e))
        }
        _ => {
            pre_guard.release().await;
            Err(CalibError::new(500, "INTERNAL", "unexpected begin restore reply"))
        }
    }
}

async fn sample_zero_state_executor(
    sup: &ArmSupervisor,
    auth: &crate::calibration::ExecutorCalibAuth,
    txn_id: Option<u64>,
    step: &'static str,
    require_zero_state_checks: bool,
    check_authority: bool,
) -> Result<crate::rt_core::requests::config::CalibrationStateSample, CalibError> {
    if check_authority {
        if let Err((_, arm_err)) = (auth.auth_check)() {
            return Err(CalibError::from(arm_err).at(step));
        }
    }
    let cfg_auth = crate::rt_core::requests::config::ConfigAuthContext {
        admission: auth.admission,
        lease_holder: auth.lease_holder,
    };
    match sup
        .dispatch_config_request_reliable(|| {
            crate::rt_core::requests::config::ConfigRequest::SampleCalibrationState {
                auth: cfg_auth,
                operator_grant: auth.operator_grant,
                expected_epoch: auth.admission.estop_epoch,
                expected_stop_gen: auth.admission.stop_gen,
                expected_txn_id: txn_id,
                require_zero_state_checks,
                check_authority,
            }
        })
        .await
    {
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationStateSampled(Ok(s))) => Ok(s),
        Ok(crate::rt_core::requests::config::ConfigReply::CalibrationStateSampled(Err(e))) => {
            Err(crate::calibration::calib_error_from_deferred_config(e).at(step))
        }
        Err(e) => Err(crate::calibration::calib_error_from_deferred_config(e).at(step)),
        _ => Err(CalibError::new(500, "INTERNAL", "unexpected calibration sample reply").at(step)),
    }
}

#[allow(clippy::too_many_arguments)]
async fn commit_software_zero_via_worker(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    auth: &crate::calibration::ExecutorCalibAuth,
    txn_id: u64,
    new_offsets: &[f32],
    joints_one_based: &[usize],
    changes: &[f32],
    method_label: &str,
    new_zero: Option<Option<crate::joint_zero::JointZero>>,
    backup_path: &std::path::Path,
    reason_label: &str,
    held_reservation: Option<crate::rt_core::config_iface::Reservation>,
    hw_mutated: bool,
    is_hardware_motor_zero: bool,
    zeroed_mask: u64,
    raw_after: [f32; crate::rt_core::mailbox::MAX_DOF],
    existing_marker: Option<std::path::PathBuf>,
) -> Result<(crate::rt_core::requests::config::ConfigCommitOutcome, Value), CalibError> {
    use crate::calibration::{
        effective_floor_for_with_legacy, file_fingerprints, legacy_file_fingerprint,
        CalibrationFile, CalibrationStale,
    };
    use crate::rt_core::config_iface::ConfigDomain;
    use crate::rt_core::requests::config::{
        build_config_proposal_with_deadline, pending_commit_marker_path,
        prepare_eval_bufs_off_thread, run_config_commit_detailed,
        run_config_commit_on_held_reservation, ConfigAuthContext, ConfigCandidate, ConfigCommitPlan,
        ConfigWriteOp, FileWriteOp, ReservePrecheck,
    };

    let pub_cfg = sup.published_config().await.map_err(CalibError::from)?;
    let dof = pub_cfg.config.dof;
    if new_offsets.len() != dof || dof > crate::rt_core::mailbox::MAX_DOF {
        return Err(CalibError::new(
            400,
            "INVALID_COMMAND",
            format!("joint zero needs {dof} finite offsets"),
        ));
    }
    let at = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true);
    let moved: Vec<usize> = joints_one_based
        .iter()
        .zip(changes)
        .filter(|(_, d)| f64::from(d.abs()) > crate::joint_zero::STALE_THRESHOLD_RAD)
        .map(|(j, _)| *j)
        .collect();

    let mut file = pub_cfg.calibration.clone();
    let removed = file.as_mut().and_then(|c| c.kinematics.take());
    let had_live_kin = pub_cfg.config.arm_model.correction().is_some();
    let camera_stale = file.as_ref().is_some_and(CalibrationFile::has_camera);
    if let Some(c) = file.as_mut().filter(|c| c.has_camera()) {
        c.stale = Some(CalibrationStale {
            reason: format!(
                "{method_label} zero of joints {joints_one_based:?}; the camera calibration was measured with the old zero"
            ),
            at: at.clone(),
        });
        if let Some(a) = c.acceptance.as_mut() {
            a.invalidated = Some(CalibrationStale {
                reason: format!(
                    "{method_label} zero of joints {joints_one_based:?}; the acceptance was measured with the old zero"
                ),
                at: at.clone(),
            });
        }
    }
    let mut file_changes = removed.is_some() || camera_stale;
    if let Some(c) = file.as_mut() {
        if crate::mit::invalidate_record(
            c,
            joints_one_based,
            &format!(
                "{method_label} zero of joints {joints_one_based:?}; the MIT qualification was measured with the old zero"
            ),
            &at,
        ) {
            file_changes = true;
        }
    }
    if let Some(z) = new_zero {
        if file.as_ref().and_then(|c| c.joint_zero.as_ref()) != z.as_ref() {
            file_changes = true;
        }
        match file.as_mut() {
            Some(c) => c.joint_zero = z,
            None if z.is_some() => {
                file = Some(CalibrationFile {
                    joint_zero: z,
                    ..Default::default()
                });
            }
            None => {}
        }
    }
    if file.as_ref().is_some_and(|c| {
        !c.has_camera()
            && c.joint_zero.is_none()
            && c.kinematics.is_none()
            && c.mit_qualification.is_none()
    }) {
        file = None;
    }

    let next_arm_model = if had_live_kin {
        Some(
            pub_cfg
                .config
                .arm_model
                .with_correction(None)
                .map_err(|e| CalibError::new(500, "INTERNAL", e))?,
        )
    } else {
        None
    };
    let effective_arm_model = next_arm_model.as_ref().unwrap_or(&pub_cfg.config.arm_model);

    let stale_reason = format!("{method_label} zero of joints {moved:?}; taught with the old zero");
    let mut poses_stale = Vec::new();
    let mut next_pose_list: Option<(crate::poses::PoseList, bool)> = None;
    if (!moved.is_empty() && !pub_cfg.poses.list.poses.is_empty()) || had_live_kin {
        let mut list = pub_cfg.poses.list.clone();
        if !moved.is_empty() {
            for p in &mut list.poses {
                p.stale = Some(CalibrationStale {
                    reason: stale_reason.clone(),
                    at: at.clone(),
                });
                poses_stale.push(p.name.clone());
            }
        }
        if had_live_kin {
            for p in &mut list.poses {
                p.tcp = pub_cfg.recompute_pose_tcp_for(effective_arm_model, p);
            }
        }
        next_pose_list = Some((list, !poses_stale.is_empty()));
    }
    let expected_poses_revision = next_pose_list.as_ref().map(|_| pub_cfg.poses.revision);

    let grip_idx = pub_cfg.config.gripper_index();
    let grip_one = grip_idx.map(|g| g + 1);
    // Legacy resets an existing gripper torque baseline for ANY nonzero gripper offset change,
    // independent of the 0.5 deg staleness threshold used for persisted gripper_calib.json.
    let gripper_any_change = grip_idx.is_some_and(|gi| {
        gi < new_offsets.len()
            && gi < pub_cfg.joint_zero_offsets.len()
            && (pub_cfg.joint_zero_offsets[gi] - new_offsets[gi]) != 0.0
    });
    let gripper_moved = grip_one.is_some_and(|g| moved.contains(&g));
    let has_grip_calib =
        pub_cfg.gripper_calib_offset.is_some() || pub_cfg.gripper_calib_jaw_opening.is_some();
    let gripper_stale = gripper_moved && has_grip_calib;
    let next_gripper_stale = gripper_stale.then(|| CalibrationStale {
        reason: stale_reason.clone(),
        at: at.clone(),
    });
    let expected_gripper_calib_revision =
        gripper_stale.then_some(pub_cfg.gripper_calib_revision);

    let effective_floor = pub_cfg.profile_floor_z_m.map(|profile| {
        let (_, fp) = file_fingerprints(
            file.as_ref(),
            &pub_cfg.config.camera_mount,
            new_offsets,
            Some(&pub_cfg.config.active_tool.config_digest),
        );
        let legacy_fp =
            legacy_file_fingerprint(file.as_ref(), &pub_cfg.config.camera_mount, new_offsets);
        let (eff, src, _) = effective_floor_for_with_legacy(
            profile,
            file.as_ref(),
            &fp,
            Some(&legacy_fp),
            &pub_cfg.config.active_tool,
        );
        (eff, src)
    });

    let ws_eval_bufs = pub_cfg.workspace_policy().map(|(ws, cam)| {
        let mut next_ws = ws.clone();
        if let Some((eff, _)) = effective_floor {
            next_ws.floor_z_m = eff;
        }
        prepare_eval_bufs_off_thread(
            effective_arm_model,
            &next_ws,
            cam,
            pub_cfg.config.active_tool.tcp_offset_m,
        )
    });

    // Pre-build replacement gripper baseline off-RT when gripper joint zero changes at all.
    let (replacement_baseline, replacement_baseline_pending) = if gripper_any_change {
        let rb = pub_cfg.config.gripper_policy.as_ref().map(|policy| {
            crate::gripper_baseline::FreeTorqueBaseline::new(
                policy.closed_rad,
                policy.open_rad,
                Some(pub_cfg.config.active_tool.id.clone()),
                Some(pub_cfg.config.active_tool.config_digest.clone()),
                pub_cfg.follower.clone(),
            )
        });
        (rb, Some(Vec::with_capacity(crate::supervisor::GRIPPER_BASELINE_PENDING_MAX)))
    } else {
        (None, None)
    };

    let mut file_ops = Vec::with_capacity(3);
    if file_changes {
        match file.as_ref() {
            Some(c) => {
                let text = serde_json::to_string_pretty(c).unwrap_or_default();
                file_ops.push(FileWriteOp::Replace(crate::persist::PendingWrite::new(
                    deps.path.clone(),
                    text,
                )));
            }
            None => {
                file_ops.push(FileWriteOp::Remove(deps.path.clone()));
            }
        }
    }
    if let (Some((ref list, true)), Some(ref poses_path)) = (&next_pose_list, &pub_cfg.poses.path) {
        file_ops.push(FileWriteOp::Replace(crate::persist::PendingWrite::new(
            poses_path.clone(),
            serde_json::to_string_pretty(list).unwrap_or_default(),
        )));
    }
    if let Some(ref stale) = next_gripper_stale {
        let f = crate::gripper_calib::GripperCalibFile {
            offset_rad: pub_cfg.gripper_calib_offset,
            jaw_opening: pub_cfg.gripper_calib_jaw_opening.clone(),
            ts: Some(
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |d| d.as_secs()),
            ),
            stale: Some(stale.clone()),
            tool_id: Some(pub_cfg.config.active_tool.id.clone()),
            tool_config_digest: Some(pub_cfg.config.active_tool.config_digest.clone()),
        };
        let grip_path = pub_cfg
            .gripper_calib_path
            .clone()
            .unwrap_or_else(crate::gripper_calib::path_from_env);
        file_ops.push(FileWriteOp::Replace(crate::persist::PendingWrite::new(
            grip_path,
            serde_json::to_string_pretty(&f).unwrap_or_default(),
        )));
    }
    // Journal EVERY effectful C-F1 commit (single-file or multi-file) so an authority loss or
    // crash after rename is quarantined on restart rather than silently activating rejected offsets.
    let pending_marker = if existing_marker.is_none() && !file_ops.is_empty() {
        let marker_path = pending_commit_marker_path(&deps.path);
        let marker_text = json!({
            "reason": reason_label,
            "backup": backup_path.display().to_string(),
            "files": file_ops.iter().map(|op| op.target_path().display().to_string()).collect::<Vec<_>>(),
        })
        .to_string();
        Some((marker_path, marker_text))
    } else {
        None
    };

    let mut offsets_arr = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
    offsets_arr[..dof].copy_from_slice(new_offsets);

    let lease_gen = auth.lease_holder.map_or(0, |(_, g)| g);
    let expected_calib_rev = held_reservation
        .as_ref()
        .map_or(pub_cfg.calibration_revision, |r| r.from());
    let proposal = build_config_proposal_with_deadline(
        sup,
        ConfigDomain::Calibration,
        expected_calib_rev,
        auth.admission,
        lease_gen,
        pub_cfg.workspace_revision,
        Instant::now() + crate::supervisor::config::CONFIG_PROPOSAL_BUDGET,
    );
    let cfg_auth = ConfigAuthContext {
        admission: auth.admission,
        lease_holder: auth.lease_holder,
    };

    let pub_offsets = new_offsets.to_vec();
    let pub_cal = file.clone();
    let pub_arm = next_arm_model.clone();
    let pub_poses = next_pose_list.clone();
    let pub_grip_stale = next_gripper_stale.clone();
    let pub_floor = effective_floor;

    let plan = ConfigCommitPlan {
        proposal,
        auth: cfg_auth,
        precheck: ReservePrecheck::SoftwareJointZero {
            allow_during_calibration_busy: true,
            expected_txn_id: Some(txn_id),
            operator_grant: auth.operator_grant,
            expected_gripper_calib_revision,
            is_hardware_motor_zero,
        },
        candidate: ConfigCandidate::SoftwareJointZero {
            new_offsets: offsets_arr,
            dof,
            is_hardware_motor_zero,
            zeroed_mask,
            raw_after,
            calibration: file,
            #[cfg(feature = "policy-runtime")]
            calibration_digest: SupervisorInner::calibration_digest_of(pub_cal.as_ref()),
            next_arm_model,
            next_pose_list,
            next_gripper_stale,
            expected_gripper_calib_revision,
            expected_txn_id: Some(txn_id),
            operator_grant: auth.operator_grant,
            effective_floor,
            ws_eval_bufs,
            reset_gripper_baseline: gripper_any_change,
            replacement_baseline,
            replacement_baseline_pending,
        },
        write: if file_ops.is_empty() && existing_marker.is_none() {
            ConfigWriteOp::Volatile
        } else {
            ConfigWriteOp::Batch {
                ops: file_ops,
                pending_marker,
                existing_marker,
            }
        },
        expected_workspace_revision: Some(pub_cfg.workspace_revision),
        expected_poses_revision,
    };

    let auth_cb = Arc::clone(&auth.auth_check);
    let update_pub = move |cfg: &mut crate::rt_core::requests::config::PublishedConfig,
                           act: crate::rt_core::requests::config::ConfigActivated| {
        if cfg.joint_zero_offsets.len() > 5
            && pub_offsets.len() > 5
            && cfg.joint_zero_offsets[5] != pub_offsets[5]
        {
            cfg.zero_convention = Default::default();
        }
        cfg.joint_zero_offsets = pub_offsets;
        cfg.calibration = pub_cal;
        if let Some(m) = pub_arm {
            cfg.config.arm_model = m;
        }
        if let Some((pl, bump)) = pub_poses {
            cfg.poses.list = pl;
            if bump {
                cfg.poses.revision = act.poses_revision;
            }
        }
        if let Some(st) = pub_grip_stale {
            cfg.gripper_calib_stale = Some(st);
            cfg.gripper_calib_revision = act.gripper_calib_revision;
        }
        if let Some((eff, src)) = pub_floor {
            cfg.floor_source = src;
            if let Some(ws) = cfg.config.workspace.as_mut() {
                ws.floor_z_m = eff;
            }
        }
        cfg.calibration_revision = act.calibration_revision;
        cfg.workspace_revision = act.workspace_revision;
    };
    let commit_res = match held_reservation {
        Some(res) => {
            run_config_commit_on_held_reservation(
                sup,
                plan,
                res,
                hw_mutated,
                move || (auth_cb)(),
                update_pub,
            )
            .await
        }
        None => run_config_commit_detailed(sup, plan, move || (auth_cb)(), update_pub).await,
    };
    let outcome = commit_res
        .map_err(|e| crate::calibration::calib_error_from_commit(e, "apply", Some(backup_path)))?;

    let inv = json!({
        "kinematics_removed": removed.is_some() || had_live_kin,
        "camera_stale": camera_stale,
        "poses_stale": poses_stale,
        "gripper_calibration_stale": gripper_stale,
        "removed_kinematics": removed,
        "at": at,
        "error": Value::Null,
    });
    Ok((outcome, inv))
}

async fn verify_executor(
    sup: &ArmSupervisor,
    auth: &crate::calibration::ExecutorCalibAuth,
    req: &MotorZeroRequest,
    before: &[f32],
    r: &[f32],
    t_apply: Instant,
) -> Vec<Value> {
    let deadline = Instant::now() + VERIFY_TIMEOUT;
    loop {
        let sample = sample_zero_state_executor(sup, auth, None, "verify", false, false).await;
        let (rows, all) = match sample {
            Ok(s) => {
                let mut all = true;
                let rows: Vec<Value> = req
                    .joints
                    .iter()
                    .map(|&j| {
                        let after = if j < s.dof { s.positions[j] } else { f32::NAN };
                        let fresh = !s.has_driver
                            || s.feedback_instants
                                .get(j)
                                .copied()
                                .flatten()
                                .is_some_and(|t| t > t_apply);
                        let ok = fresh && (after - r[j]).abs() < verify_rad();
                        all &= ok;
                        json!({
                            "joint": j + 1,
                            "before_rad": before[j],
                            "after_rad": after,
                            "reference_rad": r[j],
                            "sent": true,
                            "fresh": fresh,
                            "verified": ok,
                        })
                    })
                    .collect();
                (rows, all)
            }
            Err(_) => {
                let rows: Vec<Value> = req
                    .joints
                    .iter()
                    .map(|&j| {
                        json!({
                            "joint": j + 1,
                            "before_rad": before[j],
                            "after_rad": Value::Null,
                            "reference_rad": r[j],
                            "sent": true,
                            "fresh": false,
                            "verified": false,
                        })
                    })
                    .collect();
                (rows, false)
            }
        };
        if all || Instant::now() >= deadline {
            return rows;
        }
        tokio::time::sleep(POLL).await;
    }
}

/// Executor-mode software joint zero through PR C's two-phase config worker.
pub async fn run_software_executor(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    req: MotorZeroRequest,
    auth: crate::calibration::ExecutorCalibAuth,
) -> Result<Value, CalibError> {
    run_software_executor_with_guard(sup, deps, req, auth, None).await
}

pub(crate) async fn run_software_executor_with_guard(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    req: MotorZeroRequest,
    auth: crate::calibration::ExecutorCalibAuth,
    existing_guard: Option<crate::calibration::ExecutorTxnGuard>,
) -> Result<Value, CalibError> {
    if req.method.as_deref() == Some("motor") {
        return run_motor_executor_with_guard(sup, deps, req, auth, existing_guard).await;
    }
    if req.method.as_deref() != Some("software") {
        if let Some(g) = existing_guard {
            g.release().await;
        }
        return Err(invalid("method must be \"motor\" or \"software\""));
    }
    let sup_w = Arc::clone(sup);
    let deps_w = deps.clone();
    let (done_tx, done_rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        let res = async move {
            let guard = match existing_guard {
                Some(g) => g,
                None => match auth.txn_id {
                    Some(id) => crate::calibration::ExecutorTxnGuard::new(Arc::clone(&sup_w), id),
                    None => {
                        if let Err((_, arm_err)) = (auth.auth_check)() {
                            return Err(CalibError::from(arm_err));
                        }
                        let tid = crate::rt_core::requests::config::next_calibration_txn_id();
                        let cfg_auth = crate::rt_core::requests::config::ConfigAuthContext {
                            admission: auth.admission,
                            lease_holder: auth.lease_holder,
                        };
                        let pre_g = crate::calibration::ExecutorTxnGuard::new(Arc::clone(&sup_w), tid);
                        match sup_w
                            .dispatch_config_request(
                                crate::rt_core::requests::config::ConfigRequest::BeginCalibrationTxn {
                                    txn_id: tid,
                                    auth: cfg_auth,
                                    operator_grant: auth.operator_grant,
                                    expected_epoch: auth.admission.estop_epoch,
                                    expected_stop_gen: auth.admission.stop_gen,
                                    kind: crate::rt_core::requests::config::CalibrationTxnKind::SoftwareZero {
                                        set_busy: true,
                                    },
                                },
                            )
                            .await
                        {
                            Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Ok(_))) => {
                                sup_w.published_config_hub().update(|cfg| {
                                    cfg.calibration_busy = true;
                                    cfg.calibration_txn_id = Some(tid);
                                });
                                pre_g
                            }
                            Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Err(e))) => {
                                let _ = pre_g.disarm();
                                return Err(crate::calibration::calib_error_from_config(e));
                            }
                            Err(e) => {
                                pre_g.release().await;
                                return Err(crate::calibration::calib_error_from_config(e));
                            }
                            _ => {
                                pre_g.release().await;
                                return Err(CalibError::new(500, "INTERNAL", "unexpected begin zero reply"));
                            }
                        }
                    }
                },
            };
            let txn_id = guard.txn_id();
            let run_res = async {
                let started = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
                let pub_cfg = sup_w.published_config().await.map_err(CalibError::from)?;
                let r = reference_from_published(&pub_cfg);
                let n = pub_cfg.config.dof;
                // 1. Capture mean of fresh readings over SOFTWARE_CAPTURE while checking authority & state.
                let _ = sample_zero_state_executor(&sup_w, &auth, Some(txn_id), "capture", true, true).await?;
                let t0 = Instant::now();
                let (mut sum, mut lo, mut hi, mut count) = (
                    vec![0.0f64; n],
                    vec![f32::INFINITY; n],
                    vec![f32::NEG_INFINITY; n],
                    0u32,
                );
                while t0.elapsed() < crate::joint_zero::SOFTWARE_CAPTURE || count == 0 {
                    let s = sample_zero_state_executor(&sup_w, &auth, Some(txn_id), "capture", true, true).await?;
                    for j in 0..n {
                        let qj = s.positions[j];
                        sum[j] += f64::from(qj);
                        lo[j] = lo[j].min(qj);
                        hi[j] = hi[j].max(qj);
                    }
                    count += 1;
                    tokio::time::sleep(POLL).await;
                }
                let before: Vec<f32> = sum.iter().map(|v| (*v / f64::from(count)) as f32).collect();
                let moving: Vec<String> = req
                    .joints
                    .iter()
                    .filter(|&&j| hi[j] - lo[j] > CAPTURE_MAX_SPREAD_RAD)
                    .map(|&j| format!("J{} moved {:.2} deg", j + 1, f64::from(hi[j] - lo[j]).to_degrees()))
                    .collect();
                if !moving.is_empty() {
                    return Err(CalibError::new(
                        409,
                        "ZERO_MOVING",
                        format!(
                            "{} during the {} ms capture; hold the arm still in the rest pose and run again; nothing was changed",
                            moving.join(", "),
                            crate::joint_zero::SOFTWARE_CAPTURE.as_millis()
                        ),
                    ));
                }
                // 2. Compute new offsets.
                let pub_now = sup_w.published_config().await.map_err(CalibError::from)?;
                let old = pub_now.joint_zero_offsets.clone();
                let mut new = old.clone();
                for &j in &req.joints {
                    new[j] = old[j] + (before[j] - r[j]);
                }
                if let Some(j) = req
                    .joints
                    .iter()
                    .find(|&&j| f64::from(new[j].abs()) > crate::joint_zero::MAX_OFFSET_RAD)
                {
                    return Err(CalibError::new(
                        409,
                        "MOTOR_ZERO_FAR",
                        format!(
                            "J{} would need a joint zero offset of {:.1} deg; nothing was changed",
                            j + 1,
                            f64::from(new[*j]).to_degrees()
                        ),
                    ));
                }
                // 3. Backup (including companion poses & gripper_calib), then commit through PR C's two-phase worker.
                let backup = write_zero_backup_with_companions(
                    &deps_w,
                    "software_zero",
                    &old,
                    pub_now.poses.path.clone(),
                    pub_now.gripper_calib_path.clone(),
                )
                .await
                .map_err(|e| {
                    CalibError::new(
                        500,
                        "PERSIST_FAILED",
                        format!("backup: {e}; nothing was changed"),
                    )
                })?;
                let zeroed: Vec<usize> = req.joints.iter().map(|j| j + 1).collect();
                let changes: Vec<f32> = req.joints.iter().map(|&j| r[j] - before[j]).collect();
                let entry = zero_entry_from_published(&pub_now, &new, "software");
                let (outcome, inv) = commit_software_zero_via_worker(
                    &sup_w,
                    &deps_w,
                    &auth,
                    txn_id,
                    &new,
                    &zeroed,
                    &changes,
                    "software",
                    entry,
                    &backup,
                    "software_zero",
                    None,
                    false,
                    false,
                    0,
                    [0.0; crate::rt_core::mailbox::MAX_DOF],
                    None,
                )
                .await?;
                // 4. Verify readings newer than activation + write audit log.
                let rows = verify_executor(
                    &sup_w,
                    &auth,
                    &req,
                    &before,
                    &r,
                    outcome.activated.activated_at,
                )
                .await;
                let verified = rows.iter().all(|r| r["verified"] == json!(true));
                let log = write_log(
                    &deps_w,
                    &started,
                    &req,
                    &before,
                    &rows,
                    if verified { "verified" } else { "unverified" },
                    &inv,
                )
                .await;
                tracing::warn!(
                    "Software zero of joints {zeroed:?} (executor): offsets {old:?} -> {new:?} rad (backup {})",
                    backup.display()
                );
                let mut out = json!({
                    "joints": rows,
                    "invalidated": inv,
                    "log": log,
                    "zero_method": "software",
                    "persist": "calibration_file",
                    "persist_verified": true,
                    "torque": "off",
                    "backup": backup.display().to_string(),
                    "offsets_before": old,
                    "offsets_after": new,
                });
                if !verified {
                    let mut e = CalibError::new(
                        500,
                        "MOTOR_ZERO_UNVERIFIED",
                        format!(
                            "the joint zero was installed but not every joint read within {MOTOR_ZERO_VERIFY_MRAD} mrad of its reference; \
                            restore backup {} or run the tool again",
                            backup
                                .file_name()
                                .map(|n| n.to_string_lossy().into_owned())
                                .unwrap_or_default()
                        ),
                    );
                    if let Value::Object(m) = out.take() {
                        e.extra = m;
                    }
                    return Err(e);
                }
                out["restore"] = json!({
                    "path": RESTORE_PATH,
                    "backup": backup.file_name().map(|n| n.to_string_lossy().into_owned()),
                });
                Ok(out)
            }
            .await;
            guard.release().await;
            run_res
        }
        .await;
        let _ = done_tx.send(res);
    });
    done_rx.await.unwrap_or_else(|e| {
        tracing::error!("software zero worker failed: {e}");
        Err(CalibError::new(
            500,
            "INTERNAL",
            "calibration task failed: channel closed; check GET /api/v1/calibration and the backups",
        ))
    })
}

/// Executor-mode `/api/v1/calibration/joint_zero/restore` through PR C's two-phase config worker.
pub async fn restore_executor(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    name: String,
    auth: crate::calibration::ExecutorCalibAuth,
) -> Result<Value, CalibError> {
    restore_executor_with_guard(sup, deps, name, auth, None).await
}

pub(crate) async fn restore_executor_with_guard(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    name: String,
    auth: crate::calibration::ExecutorCalibAuth,
    existing_guard: Option<crate::calibration::ExecutorTxnGuard>,
) -> Result<Value, CalibError> {
    let sup_w = Arc::clone(sup);
    let deps_w = deps.clone();
    let (done_tx, done_rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        let res = async move {
            let guard = match existing_guard {
                Some(g) => g,
                None => match auth.txn_id {
                    Some(id) => crate::calibration::ExecutorTxnGuard::new(Arc::clone(&sup_w), id),
                    None => {
                        if let Err((_, arm_err)) = (auth.auth_check)() {
                            return Err(CalibError::from(arm_err));
                        }
                        let tid = crate::rt_core::requests::config::next_calibration_txn_id();
                        let cfg_auth = crate::rt_core::requests::config::ConfigAuthContext {
                            admission: auth.admission,
                            lease_holder: auth.lease_holder,
                        };
                        let pre_g = crate::calibration::ExecutorTxnGuard::new(Arc::clone(&sup_w), tid);
                        match sup_w
                            .dispatch_config_request(
                                crate::rt_core::requests::config::ConfigRequest::BeginCalibrationTxn {
                                    txn_id: tid,
                                    auth: cfg_auth,
                                    operator_grant: auth.operator_grant,
                                    expected_epoch: auth.admission.estop_epoch,
                                    expected_stop_gen: auth.admission.stop_gen,
                                    kind: crate::rt_core::requests::config::CalibrationTxnKind::JointZeroRestore,
                                },
                            )
                            .await
                        {
                            Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Ok(_))) => {
                                sup_w.published_config_hub().update(|cfg| {
                                    cfg.calibration_busy = true;
                                    cfg.calibration_txn_id = Some(tid);
                                });
                                pre_g
                            }
                            Ok(crate::rt_core::requests::config::ConfigReply::CalibrationTxnBegan(Err(e))) => {
                                let _ = pre_g.disarm();
                                return Err(crate::calibration::calib_error_from_config(e));
                            }
                            Err(e) => {
                                pre_g.release().await;
                                return Err(crate::calibration::calib_error_from_config(e));
                            }
                            _ => {
                                pre_g.release().await;
                                return Err(CalibError::new(500, "INTERNAL", "unexpected begin restore reply"));
                            }
                        }
                    }
                },
            };
            let txn_id = guard.txn_id();
            let run_res = async {
                let dir = deps_w.backup_dir.join(&name);
                let name_c = name.clone();
                let target: Option<crate::joint_zero::JointZero> = tokio::task::spawn_blocking(move || {
                    let manifest: Value = std::fs::read_to_string(dir.join("manifest.json"))
                        .ok()
                        .and_then(|t| serde_json::from_str(&t).ok())
                        .ok_or_else(|| {
                            CalibError::new(
                                404,
                                "BACKUP_NOT_FOUND",
                                format!("backup {name_c}: no manifest.json"),
                            )
                        })?;
                    if manifest.get("reason").and_then(Value::as_str) != Some("software_zero") {
                        return Err(invalid(format!("backup {name_c} is not a software zero backup")));
                    }
                    match crate::calibration::read_if_file(&dir.join("calibration.json")) {
                        Ok(None) => Ok(None),
                        Ok(Some(t)) => Ok(serde_json::from_str::<crate::calibration::CalibrationFile>(&t)
                            .map_err(|e| invalid(format!("backup {name_c}: calibration.json: {e}")))?
                            .joint_zero),
                        Err(e) => Err(CalibError::new(
                            500,
                            "PERSIST_FAILED",
                            format!("backup {name_c}: {e}"),
                        )),
                    }
                })
                .await
                .map_err(|e| CalibError::new(500, "INTERNAL", format!("restore read task: {e}")))??;

                let _ = sample_zero_state_executor(&sup_w, &auth, Some(txn_id), "restore", true, true).await?;
                let pub_cfg = sup_w.published_config().await.map_err(CalibError::from)?;
                let n = pub_cfg.config.dof;
                let new = target.as_ref().map_or_else(|| vec![0.0; n], |z| z.offsets());
                if let Some(z) = &target {
                    z.validate(Some(n))
                        .map_err(|e| invalid(format!("backup {name}: {e}")))?;
                }
                let old = pub_cfg.joint_zero_offsets.clone();
                let backup = write_zero_backup_with_companions(
                    &deps_w,
                    "joint_zero_restore",
                    &old,
                    pub_cfg.poses.path.clone(),
                    pub_cfg.gripper_calib_path.clone(),
                )
                .await
                .map_err(|e| {
                    CalibError::new(
                        500,
                        "PERSIST_FAILED",
                        format!("backup: {e}; nothing was changed"),
                    )
                })?;
                let joints: Vec<usize> = (0..new.len()).filter(|&j| old[j] != new[j]).collect();
                let joints_one: Vec<usize> = joints.iter().map(|j| j + 1).collect();
                let changes: Vec<f32> = joints.iter().map(|&j| old[j] - new[j]).collect();
                let entry = Some(target.filter(|z| !z.is_zero()));
                let (_outcome, inv) = commit_software_zero_via_worker(
                    &sup_w,
                    &deps_w,
                    &auth,
                    txn_id,
                    &new,
                    &joints_one,
                    &changes,
                    "restored",
                    entry,
                    &backup,
                    "joint_zero_restore",
                    None,
                    false,
                    false,
                    0,
                    [0.0; crate::rt_core::mailbox::MAX_DOF],
                    None,
                )
                .await?;
                tracing::warn!(
                    "Joint zero restored from backup {name} (executor): offsets {old:?} -> {new:?} rad"
                );
                Ok(json!({
                    "restored": name,
                    "backup": backup.display().to_string(),
                    "offsets_before": old,
                    "offsets_after": new,
                    "invalidated": inv,
                }))
            }
            .await;
            guard.release().await;
            run_res
        }
        .await;
        let _ = done_tx.send(res);
    });
    done_rx.await.unwrap_or_else(|e| {
        tracing::error!("joint zero restore worker failed: {e}");
        Err(CalibError::new(
            500,
            "INTERNAL",
            "calibration task failed: channel closed; check GET /api/v1/calibration and the backups",
        ))
    })
}

/// Executor-mode hardware motor zero (`method == "motor"`) routed through the RT tick-driven
/// commissioning state machine and PR C's single held-reservation commit path (C-F1 part 2).
pub(crate) async fn run_motor_executor_with_guard(
    sup: &Arc<ArmSupervisor>,
    deps: &CalibrationDeps,
    req: MotorZeroRequest,
    auth: crate::calibration::ExecutorCalibAuth,
    existing_guard: Option<crate::calibration::ExecutorTxnGuard>,
) -> Result<Value, CalibError> {
    use crate::mit::{
        run_rt_commissioning_op, CommissioningAuthority, CommissioningOpKind, CommissioningOpState,
        CommissioningStartRequest, JointCommissioningDisposition,
    };
    use crate::rt_core::config_iface::ConfigDomain;
    use crate::rt_core::requests::config::{
        build_config_proposal_with_deadline, pending_commit_marker_path,
        remove_pending_commit_marker, write_pending_commit_marker, ConfigAuthContext, ConfigReply,
        ConfigRequest, ReleaseConfig, ReservationGuard, ReserveConfig, ReservePrecheck,
        VerifyDomainChecks,
    };

    let sup_w = Arc::clone(sup);
    let deps_w = deps.clone();
    let (done_tx, done_rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        let res = async move {
            let mut txn_guard = match existing_guard {
                Some(g) => g,
                None => match auth.txn_id {
                    Some(id) => crate::calibration::ExecutorTxnGuard::new(Arc::clone(&sup_w), id),
                    None => {
                        if let Err((_, arm_err)) = (auth.auth_check)() {
                            return Err(CalibError::from(arm_err));
                        }
                        let tid = crate::rt_core::requests::config::next_calibration_txn_id();
                        let cfg_auth = ConfigAuthContext {
                            admission: auth.admission,
                            lease_holder: auth.lease_holder,
                        };
                        let pre_g = crate::calibration::ExecutorTxnGuard::new(Arc::clone(&sup_w), tid);
                        match sup_w
                            .dispatch_config_request(ConfigRequest::BeginCalibrationTxn {
                                txn_id: tid,
                                auth: cfg_auth,
                                operator_grant: auth.operator_grant,
                                expected_epoch: auth.admission.estop_epoch,
                                expected_stop_gen: auth.admission.stop_gen,
                                kind: crate::rt_core::requests::config::CalibrationTxnKind::MotorZero {
                                    set_busy: true,
                                },
                            })
                            .await
                        {
                            Ok(ConfigReply::CalibrationTxnBegan(Ok(_))) => {
                                sup_w.published_config_hub().update(|cfg| {
                                    cfg.calibration_busy = true;
                                    cfg.calibration_txn_id = Some(tid);
                                });
                                pre_g
                            }
                            Ok(ConfigReply::CalibrationTxnBegan(Err(e))) => {
                                let _ = pre_g.disarm();
                                return Err(crate::calibration::calib_error_from_config(e));
                            }
                            Err(e) => {
                                pre_g.release().await;
                                return Err(crate::calibration::calib_error_from_config(e));
                            }
                            _ => {
                                pre_g.release().await;
                                return Err(CalibError::new(500, "INTERNAL", "unexpected begin motor zero reply"));
                            }
                        }
                    }
                },
            };
            let txn_id = txn_guard.txn_id();

            let run_res = async {
                let started = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
                let pub_cfg = sup_w.published_config().await.map_err(CalibError::from)?;
                let dof = pub_cfg.config.dof;
                if dof > crate::rt_core::mailbox::MAX_DOF || req.joints.len() > crate::rt_core::mailbox::MAX_DOF {
                    return Err(invalid("too many joints"));
                }
                let r = reference_from_published(&pub_cfg);
                let old = pub_cfg.joint_zero_offsets.clone();
                let sample = sample_zero_state_executor(&sup_w, &auth, Some(txn_id), "disable", true, true).await?;
                let before = sample.positions[..sample.dof].to_vec();

                let mut joints_arr = [0usize; crate::rt_core::mailbox::MAX_DOF];
                for (i, &j) in req.joints.iter().enumerate() {
                    joints_arr[i] = j;
                }
                let mut ref_arr = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
                for (i, &rv) in r.iter().enumerate().take(crate::rt_core::mailbox::MAX_DOF) {
                    ref_arr[i] = rv;
                }

                // 1. Acquire ONE single ConfigDomain::Calibration reservation held across disable,
                //    pre-HW MIT invalidation, hardware set_zero + raw verify, and final commit (M4).
                let lease_gen = auth.lease_holder.map_or(0, |(_, g)| g);
                let cfg_auth = ConfigAuthContext {
                    admission: auth.admission,
                    lease_holder: auth.lease_holder,
                };
                let txn_deadline = Instant::now() + Duration::from_secs(20);
                let proposal = build_config_proposal_with_deadline(
                    &sup_w,
                    ConfigDomain::Calibration,
                    pub_cfg.calibration_revision,
                    auth.admission,
                    lease_gen,
                    pub_cfg.workspace_revision,
                    txn_deadline,
                );
                let reservation = match sup_w
                    .dispatch_config_request_owned_reliable(ConfigRequest::Reserve(ReserveConfig {
                        proposal,
                        auth: cfg_auth,
                        precheck: ReservePrecheck::SoftwareJointZero {
                            allow_during_calibration_busy: true,
                            expected_txn_id: Some(txn_id),
                            operator_grant: auth.operator_grant,
                            expected_gripper_calib_revision: None,
                            is_hardware_motor_zero: true,
                        },
                        expected_workspace_revision: Some(pub_cfg.workspace_revision),
                        expected_poses_revision: None,
                    }))
                    .await
                {
                    Ok(ConfigReply::Reserved(Ok(res))) => res,
                    Ok(ConfigReply::Reserved(Err(e))) => {
                        return Err(crate::calibration::calib_error_from_config(e).at("disable"));
                    }
                    Err(e) => return Err(crate::calibration::calib_error_from_config(e).at("disable")),
                    _ => {
                        return Err(CalibError::new(500, "INTERNAL", "unexpected calibration reserve reply").at("disable"));
                    }
                };
                let grant_id = reservation.grant();
                let mut res_guard = ReservationGuard::new(Arc::clone(&sup_w), reservation);

                // 2. Non-blocking RT disable-all + per-motor disabled confirmation (M1).
                let disable_req = CommissioningStartRequest {
                    kind: CommissioningOpKind::MotorZeroDisable,
                    joints: joints_arr,
                    joint_len: req.joints.len(),
                    timeout_counts: 0,
                    references: ref_arr,
                    authority: CommissioningAuthority {
                        conn_id: auth.admission.connection_id,
                        lease_holder: auth.lease_holder,
                        operator_grant: auth.operator_grant,
                        expected_epoch: auth.admission.estop_epoch,
                        expected_stop_gen: auth.admission.stop_gen,
                        grant_id: Some(grant_id),
                        txn_id,
                    },
                    txn_deadline,
                    prior_disk_mutated: false,
                };
                let disable_snap = match run_rt_commissioning_op(&sup_w, disable_req, auth.auth_check.as_ref()).await {
                    Ok(s) => s,
                    Err(e) => {
                        if let Some(r_tok) = res_guard.take() {
                            let _ = sup_w
                                .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                                    reservation: r_tok,
                                }))
                                .await;
                        }
                        return Err(e.into_calib_error().at("disable"));
                    }
                };
                if disable_snap.state != CommissioningOpState::Done {
                    if let Some(r_tok) = res_guard.take() {
                        let _ = sup_w
                            .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                                reservation: r_tok,
                            }))
                            .await;
                    }
                    let status = if disable_snap.error_status != 0 { disable_snap.error_status } else { 409 };
                    let code = if disable_snap.error_code.is_empty() { "MOTOR_ZERO_NOT_DISABLED" } else { disable_snap.error_code };
                    let msg = if disable_snap.error_msg.is_empty() {
                        format!(
                            "joints {:?} did not confirm disabled within {} ms; nothing was changed",
                            req.joints.iter().map(|j| j + 1).collect::<Vec<_>>(),
                            DISABLE_CONFIRM_TIMEOUT.as_millis()
                        )
                    } else {
                        disable_snap.error_msg.as_str().to_string()
                    };
                    return Err(CalibError::new(status, code, msg).at("disable"));
                }

                // 3. Off-thread backup + durable pending marker BEFORE any irreversible flash TX.
                let backup = write_zero_backup_with_companions(
                    &deps_w,
                    "motor_zero",
                    &old,
                    pub_cfg.poses.path.clone(),
                    pub_cfg.gripper_calib_path.clone(),
                )
                .await
                .map_err(|e| {
                    CalibError::new(500, "PERSIST_FAILED", format!("backup: {e}; nothing was changed"))
                })?;

                let one_based_req: Vec<usize> = req.joints.iter().map(|j| j + 1).collect();
                let marker_path = pending_commit_marker_path(&deps_w.path);
                let marker_text = json!({
                    "reason": "hardware_motor_zero",
                    "backup": backup.display().to_string(),
                    "joints": one_based_req,
                })
                .to_string();
                let mp_w = marker_path.clone();
                if let Err(e) = tokio::task::spawn_blocking(move || write_pending_commit_marker(&mp_w, &marker_text))
                    .await
                    .map_err(|e| e.to_string())
                    .and_then(|r| r)
                {
                    if let Some(r_tok) = res_guard.take() {
                        let _ = sup_w
                            .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                                reservation: r_tok,
                            }))
                            .await;
                    }
                    return Err(CalibError::new(
                        500,
                        "PERSIST_FAILED",
                        format!("pending commit marker could not be written: {e}; nothing was changed"),
                    ));
                }

                // B2: Bind durable pending-marker proof to the active Reserved hold on the RT owner
                // BEFORE any pre-HW MIT invalidation or `StartCommissioning(MotorZeroWriteVerify)`.
                let bind_res = sup_w
                    .dispatch_config_request_reliable(|| ConfigRequest::BindCommissioningMarker { grant_id })
                    .await;
                if !matches!(bind_res, Ok(ConfigReply::CommissioningMarkerBound(Ok(())))) {
                    let mp_rm = marker_path.clone();
                    let rm_ok = tokio::task::spawn_blocking(move || remove_pending_commit_marker(&mp_rm))
                        .await
                        .map_err(|e| e.to_string())
                        .and_then(|r| r)
                        .is_ok();
                    if rm_ok {
                        if let Some(r_tok) = res_guard.take() {
                            let _ = sup_w
                                .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                                    reservation: r_tok,
                                }))
                                .await;
                        }
                    } else {
                        res_guard.mark_hw_mutated();
                        txn_guard.mark_remote_mutated();
                    }
                    return Err(CalibError::new(
                        409,
                        "CONFIG_BUSY",
                        "commissioning reservation lost before binding durable pending marker; nothing was changed",
                    ));
                }

                // 4. Pre-HW MIT qualification invalidation: commit BOTH disk AND live core.calibration
                //    atomically on the HELD reservation BEFORE sending any CMD_SET_ZERO frame (B3),
                //    so disk and live already agree if zero is refused before the first TX!
                if let Some(mut c) = pub_cfg.calibration.clone() {
                    let at = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true);
                    let reason = format!(
                        "motor zero of joints {one_based_req:?}; the MIT qualification was measured with the old zero"
                    );
                    if crate::mit::invalidate_record(&mut c, &one_based_req, &reason, &at) {
                        if let Err((_, arm_err)) = (auth.auth_check)() {
                            let mp_rm = marker_path.clone();
                            let _ = tokio::task::spawn_blocking(move || remove_pending_commit_marker(&mp_rm)).await;
                            if let Some(r_tok) = res_guard.take() {
                                let _ = sup_w
                                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                                        reservation: r_tok,
                                    }))
                                    .await;
                            }
                            return Err(CalibError::from(arm_err).at("mit_invalidate"));
                        }
                        let verify_res = sup_w
                            .dispatch_config_request_owned_reliable(ConfigRequest::VerifyAuthority {
                                domain: ConfigDomain::Calibration,
                                grant: grant_id,
                                proposal,
                                auth: cfg_auth,
                                expected_workspace_revision: Some(pub_cfg.workspace_revision),
                                expected_poses_revision: None,
                                domain_checks: VerifyDomainChecks {
                                    expected_grasp_setup_revision: None,
                                    expected_gripper_calib_revision: None,
                                    require_workspace_unblocked: false,
                                    require_kinematics_disarmed: false,
                                    require_software_zero_idle: true,
                                    require_mit_torque_off: false,
                                    expected_txn_id: Some(txn_id),
                                    operator_grant: Some(auth.operator_grant),
                                },
                            })
                            .await;
                        if !matches!(verify_res, Ok(ConfigReply::AuthorityVerified(Ok(())))) {
                            let mp_rm = marker_path.clone();
                            let _ = tokio::task::spawn_blocking(move || remove_pending_commit_marker(&mp_rm)).await;
                            if let Some(r_tok) = res_guard.take() {
                                let _ = sup_w
                                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                                        reservation: r_tok,
                                    }))
                                    .await;
                            }
                            return Err(match verify_res {
                                Ok(ConfigReply::AuthorityVerified(Err(e))) => {
                                    crate::calibration::calib_error_from_deferred_config(e).at("mit_invalidate")
                                }
                                Err(e) => crate::calibration::calib_error_from_deferred_config(e).at("mit_invalidate"),
                                _ => CalibError::new(500, "INTERNAL", "unexpected verify reply").at("mit_invalidate"),
                            });
                        }
                        let text = serde_json::to_string_pretty(&c)
                            .map_err(|e| CalibError::new(500, "INTERNAL", e.to_string()))?;
                        let path_c = deps_w.path.clone();
                        let stage_res = tokio::task::spawn_blocking(move || -> Result<(), (bool, String)> {
                            let staged = crate::persist::StagedWrite::stage(path_c.clone(), text)
                                .map_err(|e| (false, e))?;
                            let ds = staged.rename_only().map_err(|e| (false, e))?;
                            match ds.sync_parent_dir() {
                                crate::persist::PublishOutcome::Published => Ok(()),
                                crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) => {
                                    Err((true, format!("dir fsync failed: {w}")))
                                }
                            }
                        })
                        .await
                        .map_err(|e| (true, e.to_string()))
                        .and_then(|r| r);
                        match stage_res {
                            Ok(()) => {
                                let pub_c = Some(c.clone());
                                #[cfg(feature = "policy-runtime")]
                                let cal_digest = SupervisorInner::calibration_digest_of(pub_c.as_ref());
                                let r_tok = res_guard.take().expect("reservation held");
                                match sup_w
                                    .dispatch_config_request_owned_reliable(
                                        ConfigRequest::CommitIntermediateCalibration {
                                            reservation: r_tok,
                                            auth: cfg_auth,
                                            calibration: Some(c),
                                            #[cfg(feature = "policy-runtime")]
                                            calibration_digest: cal_digest,
                                            expected_txn_id: txn_id,
                                            operator_grant: auth.operator_grant,
                                        },
                                    )
                                    .await
                                {
                                    Ok(ConfigReply::IntermediateCalibrationCommitted(Ok((new_r, new_rev)))) => {
                                        sup_w.published_config_hub().update(|cfg| {
                                            cfg.calibration = pub_c;
                                            cfg.calibration_revision = new_rev;
                                        });
                                        res_guard = ReservationGuard::new(Arc::clone(&sup_w), new_r);
                                    }
                                    Ok(ConfigReply::IntermediateCalibrationCommitted(Err(e))) => {
                                        txn_guard.mark_remote_mutated();
                                        return Err(crate::calibration::calib_error_from_deferred_config(e).at("mit_invalidate"));
                                    }
                                    Err(e) => {
                                        txn_guard.mark_remote_mutated();
                                        let _ = sup_w
                                            .dispatch_config_request_reliable(|| {
                                                ConfigRequest::ForceReconcile(ConfigDomain::Calibration)
                                            })
                                            .await;
                                        return Err(crate::calibration::calib_error_from_deferred_config(e).at("mit_invalidate"));
                                    }
                                    _ => {
                                        txn_guard.mark_remote_mutated();
                                        let _ = sup_w
                                            .dispatch_config_request_reliable(|| {
                                                ConfigRequest::ForceReconcile(ConfigDomain::Calibration)
                                            })
                                            .await;
                                        return Err(CalibError::new(
                                            500,
                                            "INTERNAL",
                                            "unexpected intermediate calibration commit reply",
                                        ));
                                    }
                                }
                            }
                            Err((false, e)) => {
                                let mp_rm = marker_path.clone();
                                let _ = tokio::task::spawn_blocking(move || remove_pending_commit_marker(&mp_rm)).await;
                                if let Some(r_tok) = res_guard.take() {
                                    let _ = sup_w
                                        .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                                            reservation: r_tok,
                                        }))
                                        .await;
                                }
                                return Err(CalibError::new(
                                    500,
                                    "PERSIST_FAILED",
                                    format!(
                                        "the MIT qualification could not be invalidated in {} ({e}); the zero was refused, nothing was changed",
                                        deps_w.path.display()
                                    ),
                                ));
                            }
                            Err((true, e)) => {
                                res_guard.mark_hw_mutated();
                                txn_guard.mark_remote_mutated();
                                let _ = sup_w
                                    .dispatch_config_request_reliable(|| {
                                        ConfigRequest::ForceReconcile(ConfigDomain::Calibration)
                                    })
                                    .await;
                                return Err(CalibError::new(
                                    500,
                                    "PERSIST_FAILED",
                                    format!(
                                        "MIT qualification invalidation renamed {} but durability failed ({e}); reconciling required",
                                        deps_w.path.display()
                                    ),
                                ));
                            }
                        }
                    }
                }

                // 5. Run tick-driven RT hardware CMD_SET_ZERO + raw post-command verification (M2).
                let zero_req = CommissioningStartRequest {
                    kind: CommissioningOpKind::MotorZeroWriteVerify,
                    joints: joints_arr,
                    joint_len: req.joints.len(),
                    timeout_counts: 0,
                    references: ref_arr,
                    authority: CommissioningAuthority {
                        conn_id: auth.admission.connection_id,
                        lease_holder: auth.lease_holder,
                        operator_grant: auth.operator_grant,
                        expected_epoch: auth.admission.estop_epoch,
                        expected_stop_gen: auth.admission.stop_gen,
                        grant_id: Some(grant_id),
                        txn_id,
                    },
                    txn_deadline,
                    prior_disk_mutated: false,
                };
                let snap = match run_rt_commissioning_op(&sup_w, zero_req, auth.auth_check.as_ref()).await {
                    Ok(s) => s,
                    Err(crate::mit::CommissioningRunError::DefinitivePreTxRefusal(e)) => {
                        let mp_rm = marker_path.clone();
                        let rm_ok = tokio::task::spawn_blocking(move || remove_pending_commit_marker(&mp_rm))
                            .await
                            .map_err(|re| re.to_string())
                            .and_then(|r| r)
                            .is_ok();
                        if rm_ok {
                            if let Some(r_tok) = res_guard.take() {
                                let _ = sup_w
                                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                                        reservation: r_tok,
                                    }))
                                    .await;
                            }
                        } else {
                            res_guard.mark_hw_mutated();
                            txn_guard.mark_remote_mutated();
                        }
                        return Err(e.at("set_zero"));
                    }
                    Err(crate::mit::CommissioningRunError::UnknownPostStartOutcome(e)) => {
                        // B3: Outcome after start submission is unknown — NEVER unlink `.calibration_commit.pending`;
                        // force Reconciling fail-closed so the restart fence survives process exit.
                        res_guard.mark_hw_mutated();
                        txn_guard.mark_remote_mutated();
                        let _ = sup_w
                            .dispatch_config_request_reliable(|| {
                                ConfigRequest::ForceReconcile(ConfigDomain::Calibration)
                            })
                            .await;
                        return Err(e.at("set_zero"));
                    }
                };

                let mut applied_joints = Vec::new();
                let mut uncertain_joints = Vec::new();
                let mut not_attempted_joints = Vec::new();
                let mut rows = Vec::with_capacity(snap.joint_len);
                let mut zeroed_mask = 0u64;
                let mut raw_after = [f32::NAN; crate::rt_core::mailbox::MAX_DOF];
                for i in 0..snap.joint_len {
                    let o = &snap.outcomes[i];
                    let j = o.joint;
                    let sent = o.disposition != JointCommissioningDisposition::Unattempted;
                    match o.disposition {
                        JointCommissioningDisposition::Applied => {
                            applied_joints.push(j + 1);
                            zeroed_mask |= 1u64 << j;
                            raw_after[j] = o.raw_after;
                        }
                        JointCommissioningDisposition::Uncertain => {
                            uncertain_joints.push(j + 1);
                        }
                        JointCommissioningDisposition::Unattempted => {
                            not_attempted_joints.push(j + 1);
                        }
                    }
                    rows.push(json!({
                        "joint": j + 1,
                        "before_rad": before[j],
                        "after_rad": o.joint_after,
                        "reference_rad": r[j],
                        "sent": sent,
                        "fresh": o.fresh,
                        "verified": o.verified,
                    }));
                }

                // Case A: Zero irreversible TX left host -> clean rollback & release.
                if !snap.hw_mutated {
                    let mp_rm = marker_path.clone();
                    let rm_ok = tokio::task::spawn_blocking(move || remove_pending_commit_marker(&mp_rm))
                        .await
                        .map_err(|re| re.to_string())
                        .and_then(|r| r)
                        .is_ok();
                    if rm_ok {
                        if let Some(r_tok) = res_guard.take() {
                            let _ = sup_w
                                .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                                    reservation: r_tok,
                                }))
                                .await;
                        }
                    } else {
                        res_guard.mark_hw_mutated();
                        txn_guard.mark_remote_mutated();
                    }
                    let status = if snap.error_status != 0 { snap.error_status } else { 500 };
                    let code = if snap.error_code.is_empty() { "MOTOR_ZERO_FAILED" } else { snap.error_code };
                    let mut msg = if snap.error_msg.is_empty() {
                        "motor zero failed".to_string()
                    } else {
                        snap.error_msg.as_str().to_string()
                    };
                    if !msg.contains("no zero was changed") && code == "MOTOR_ZERO_FAILED" {
                        msg.push_str("; no zero was changed");
                    }
                    return Err(CalibError::new(status, code, msg).at("set_zero"));
                }

                // Hardware mutated: mark guards immediately so any drop/error keeps Reconciling + marker.
                res_guard.mark_hw_mutated();
                txn_guard.mark_remote_mutated();

                let all_applied = snap.state == CommissioningOpState::Done
                    && uncertain_joints.is_empty()
                    && not_attempted_joints.is_empty()
                    && applied_joints.len() == snap.joint_len;

                // Case B: Partial batch, cancelled after TX, or unverified raw zero ->
                // NEVER claim clean completion; keep HoldState::Reconciling + pending marker!
                if !all_applied {
                    let _ = sup_w
                        .dispatch_config_request_reliable(|| {
                            ConfigRequest::ForceReconcile(ConfigDomain::Calibration)
                        })
                        .await;
                    let fault_msg = format!(
                        "Hardware motor zero incomplete: applied {applied_joints:?}, uncertain {uncertain_joints:?}, not attempted {not_attempted_joints:?}; reconcile hardware and disk before resuming"
                    );
                    let bounded = crate::safety::BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::new(&fault_msg);
                    let _ = sup_w
                        .dispatch_config_request_reliable(|| ConfigRequest::LatchConfigReconcileFault {
                            domain: ConfigDomain::Calibration,
                            message: bounded,
                        })
                        .await;
                    let result_tag = if !not_attempted_joints.is_empty() {
                        "partial"
                    } else {
                        "unverified"
                    };
                    let inv_none = json!({
                        "kinematics_removed": false,
                        "camera_stale": false,
                        "poses_stale": [],
                        "gripper_calibration_stale": false,
                        "reconciling": true,
                    });
                    let log = write_log(&deps_w, &started, &req, &before, &rows, result_tag, &inv_none).await;
                    let status = if snap.error_status != 0 { snap.error_status } else { 500 };
                    let code = if snap.error_code.is_empty() { "MOTOR_ZERO_UNVERIFIED" } else { snap.error_code };
                    let base = if snap.error_msg.is_empty() {
                        format!(
                            "the set-zero commands were sent but not every joint verified within {MOTOR_ZERO_VERIFY_MRAD} mrad"
                        )
                    } else {
                        snap.error_msg.as_str().to_string()
                    };
                    let msg = format!(
                        "{base}; applied {applied_joints:?}, uncertain {uncertain_joints:?}, not attempted {not_attempted_joints:?}"
                    );
                    let mut err = CalibError::new(status, code, msg);
                    err.extra.insert("joints".into(), json!(rows));
                    err.extra.insert("applied".into(), json!(applied_joints));
                    err.extra.insert("uncertain".into(), json!(uncertain_joints));
                    err.extra.insert("not_attempted".into(), json!(not_attempted_joints));
                    err.extra.insert("invalidated".into(), inv_none);
                    err.extra.insert("log".into(), log);
                    return Err(err);
                }

                // Case C: ALL joints sent CMD_SET_ZERO and verified raw ~= 0.0 from fresh feedback!
                // Now persist invalidations + joint_zero offsets and atomically activate on the HELD reservation.
                let pub_now = sup_w.published_config().await.map_err(CalibError::from)?;
                let mut new_offsets = pub_now.joint_zero_offsets.clone();
                for &j in &req.joints {
                    new_offsets[j] = -r[j];
                }
                let changes: Vec<f32> = req.joints.iter().map(|&j| r[j] - before[j]).collect();
                let entry = zero_entry_from_published(&pub_now, &new_offsets, "motor");
                let held_r = res_guard.take().expect("reservation held");
                let (_outcome, inv) = commit_software_zero_via_worker(
                    &sup_w,
                    &deps_w,
                    &auth,
                    txn_id,
                    &new_offsets,
                    &applied_joints,
                    &changes,
                    "motor",
                    entry,
                    &backup,
                    "hardware_motor_zero",
                    Some(held_r),
                    true,
                    true,
                    zeroed_mask,
                    raw_after,
                    Some(marker_path),
                )
                .await?;

                // Commit succeeded cleanly (disk + parent dir fsync + RT swap + PublishedConfigHub + marker removed)!
                txn_guard.clear_remote_mutated();
                let log = write_log(&deps_w, &started, &req, &before, &rows, "verified", &inv).await;
                let tool = sup_w
                    .published_config()
                    .await
                    .map_err(CalibError::from)?
                    .calibration_tools()
                    .motor_zero;
                tracing::warn!(
                    "Motor zero of joints {applied_joints:?} (executor, readings before {:?}): verified",
                    req.joints.iter().map(|&j| before[j]).collect::<Vec<_>>()
                );
                Ok(json!({
                    "joints": rows,
                    "invalidated": inv,
                    "log": log,
                    "zero_method": "motor",
                    "method": tool.support.method,
                    "persist": tool.support.persist,
                    "persist_verified": tool.support.persist_verified,
                    "torque": "off",
                }))
            }
            .await;

            txn_guard.release().await;
            run_res
        }
        .await;
        let _ = done_tx.send(res);
    });
    done_rx.await.unwrap_or_else(|e| {
        tracing::error!("hardware motor zero worker failed: {e}");
        Err(CalibError::new(
            500,
            "INTERNAL",
            "calibration task failed: channel closed; check GET /api/v1/calibration and the backups",
        ))
    })
}
