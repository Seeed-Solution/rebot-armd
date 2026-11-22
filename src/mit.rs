//! MIT impedance tracking on Damiao motors (joint tracking Phase 2), disabled by default.
//!
//! A joint selects MIT with `joint_tracking.joints[k].strategy = "mit"` (plus the MIT-only keys
//! `hold_kp` / `hold_kd`). Activation additionally needs, on every arm and rearm, the
//! qualification gate [`check_gate`]: a calibration record ([`MitQualificationFile`],
//! `calibration.json` `mit_qualification`) bound to the profile, joint, joint zero, motor
//! identity and firmware, and a fresh register readback of every selected motor that matches
//! it, with a nonzero motor command timeout (TIMEOUT, RID 9). Unqualified hardware refuses MIT
//! before any motion: the arm does not arm.
//!
//! Motor-side command timeout. Commissioning ([`TimeoutCommand`], WS
//! `/api/v1/calibration/mit_timeout`, operator only, torque off, load supported) reads the
//! motor identity, firmware, mode, ranges and original TIMEOUT, writes TIMEOUT as a
//! little-endian u32 with the RAM opcode 0x55 (never the flash store 0xAA), checks the write
//! acknowledgement and confirms it with a separate 0x33 read that must match the motor ID, the
//! reply CAN ID (the motor's reported MST_ID), the opcode, the RID and the value. RAM is lost at
//! power-off, so a power cycle leaves TIMEOUT 0 and the gate refuses until it is commissioned
//! again. The unit is documented as 50 µs per count ([`damiao::TIMEOUT_UNIT_US_DOCUMENTED`]:
//! 2000 counts = 100 ms); it is NOT verified on the B601-DM firmware, so the count and the
//! assumed unit are both recorded and the count stays configurable.
//!
//! Rollback: a verified nonzero TIMEOUT stays in RAM after park or rollback (restoring 0 would
//! remove protection while ownership is uncertain); POS_VEL and hand guiding then run with it.

use std::time::Duration;

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::calibration::{CalibError, CalibrationStale};
use crate::model::ArmConfig;
use crate::protocol::damiao;
use crate::tracking::{JointTrackingConfig, TrackingStrategy};

/// WS path of the TIMEOUT commissioning.
pub const TIMEOUT_PATH: &str = "/api/v1/calibration/mit_timeout";
/// WS path that stores the qualification record.
pub const QUALIFICATION_PATH: &str = "/api/v1/calibration/mit_qualification";
/// Dynamic-output deadline while tracking owns an MIT joint: a feedback age or a gap between
/// dynamic outputs of this length cancels the group and latches BUS_LOSS.
pub const MIT_DEADLINE: Duration = Duration::from_millis(45);
/// Default TIMEOUT count: 100 ms at the documented (unverified) 50 µs per count.
pub const DEFAULT_TIMEOUT_COUNTS: u32 = 2000;
/// Largest TIMEOUT count accepted by the commissioning (1 s at the documented unit).
pub const MAX_TIMEOUT_COUNTS: u32 = 20_000;
/// Version of the qualification record schema.
pub const QUALIFICATION_VERSION: u32 = 1;

/// Fresh register readback of one Damiao motor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotorIdentity {
    /// 1-based joint.
    pub joint: usize,
    /// Command CAN ID the driver addresses.
    pub motor_id: u16,
    /// CAN ID the register replies arrived on (equal to `mst_id`).
    pub reply_id: u32,
    pub esc_id: u32,
    pub mst_id: u32,
    pub sw_ver: u32,
    pub ctrl_mode: u32,
    pub pmax: f32,
    pub vmax: f32,
    pub tmax: f32,
    /// TIMEOUT register (counts).
    pub timeout_raw: u32,
}

/// Result of one TIMEOUT commissioning of a motor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimeoutCommissioning {
    pub before: MotorIdentity,
    pub written_raw: u32,
    /// Value carried by the 0x55 write acknowledgement.
    pub ack_raw: u32,
    /// Separate 0x33 readback after the write.
    pub readback: MotorIdentity,
    /// Assumed unit (µs per count) and whether it is verified on this firmware (never true
    /// from software).
    pub unit_us: u32,
    pub unit_verified: bool,
}

/// `calibration.json` `mit_qualification`: the device qualification of MIT joints.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MitQualificationFile {
    pub version: u32,
    pub joints: Vec<MitJointQualification>,
    /// Set by a joint zero change: the qualification was measured with the old zero.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub invalidated: Option<CalibrationStale>,
}

/// Qualification of one MIT joint (section 2 of the Phase 2 design).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MitJointQualification {
    pub profile_id: String,
    /// 1-based joint and its profile name.
    pub joint: usize,
    pub joint_name: String,
    /// [`zero_fingerprint`] of the joint zero offset in effect.
    pub zero_fingerprint: String,
    pub motor_id: u16,
    pub reply_id: u32,
    pub esc_id: u32,
    pub mst_id: u32,
    pub sw_ver: u32,
    /// Verified mapping ranges (PMAX / VMAX / TMAX).
    pub pmax: f32,
    pub vmax: f32,
    pub tmax: f32,
    /// TIMEOUT count and the unit assumed for it.
    pub timeout_raw: u32,
    pub timeout_unit_us: u32,
    /// Whether the unit and expiry latency were verified on the device (section 6 step 2).
    pub timeout_unit_verified: bool,
    /// Observed motor response on command loss (for example `disabled`, `energized_hold`).
    pub tested_response: String,
    /// Measured time from the last command to the response (ms).
    pub measured_expiry_ms: f32,
    /// Load envelope the joint was qualified with (free text: payload, posture range).
    pub load_envelope: String,
    /// Qualified gain ceilings and effort ceiling (N·m).
    pub max_kp: f32,
    pub max_kd: f32,
    pub effort_ceiling_nm: f32,
    /// UTC date (RFC 3339) and the trace id of the qualification run.
    pub date: String,
    pub trace_id: String,
}

/// Fingerprint of a joint zero offset (rad), bit-exact.
pub fn zero_fingerprint(offset_rad: f32) -> String { format!("zero:{:08x}", offset_rad.to_bits()) }

impl MitQualificationFile {
    pub fn validate(&self) -> Result<(), String> {
        if self.version != QUALIFICATION_VERSION {
            return Err(format!("mit_qualification.version must be {QUALIFICATION_VERSION}"));
        }
        if self.joints.is_empty() || self.joints.len() > 16 {
            return Err("mit_qualification.joints: 1..16 entries".into());
        }
        for (k, j) in self.joints.iter().enumerate() {
            let e = |m: &str| format!("mit_qualification.joints[{k}]: {m}");
            if j.joint == 0 || self.joints[..k].iter().any(|o| o.joint == j.joint) {
                return Err(e("joint must be a unique 1-based index"));
            }
            for (name, s) in [("profile_id", &j.profile_id), ("joint_name", &j.joint_name), ("zero_fingerprint", &j.zero_fingerprint),
                ("tested_response", &j.tested_response), ("load_envelope", &j.load_envelope), ("date", &j.date), ("trace_id", &j.trace_id)] {
                if s.is_empty() || s.len() > 256 { return Err(e(&format!("{name}: 1..256 bytes"))); }
            }
            for (name, v) in [("pmax", j.pmax), ("vmax", j.vmax), ("tmax", j.tmax), ("max_kp", j.max_kp), ("max_kd", j.max_kd),
                ("effort_ceiling_nm", j.effort_ceiling_nm), ("measured_expiry_ms", j.measured_expiry_ms)] {
                if !v.is_finite() || v <= 0.0 { return Err(e(&format!("{name} must be finite and > 0"))); }
            }
            if j.timeout_raw == 0 || j.timeout_raw > MAX_TIMEOUT_COUNTS {
                return Err(e(&format!("timeout_raw must be in 1..={MAX_TIMEOUT_COUNTS}")));
            }
            if j.timeout_unit_us == 0 { return Err(e("timeout_unit_us must be > 0")); }
            if j.effort_ceiling_nm > j.tmax { return Err(e("effort_ceiling_nm must not exceed tmax")); }
        }
        Ok(())
    }

    pub fn joint(&self, joint_1based: usize) -> Option<&MitJointQualification> {
        self.joints.iter().find(|j| j.joint == joint_1based)
    }
}

/// 0-based arm joint indices selected for MIT (empty when MIT is not requested).
pub fn mit_joints(cfg: &ArmConfig) -> Vec<usize> {
    let Some(jt) = cfg.joint_tracking.as_ref().filter(|jt| jt.mit_requested()) else { return Vec::new() };
    let arm = cfg.arm_joint_indices();
    jt.mit_joint_slots().into_iter().filter_map(|k| arm.get(k).copied()).collect()
}

/// Profile-level MIT rules (only for an enabled block with `mit` joints): gain ceilings from the
/// profile's hand-guide hold gains (`gravity_comp.hold_kp` / `hold_kd`), each optional term
/// within the effort limit at the profile maxima, and a gravity model when `gravity_scale` > 0.
pub fn validate_profile(cfg: &ArmConfig, jt: &JointTrackingConfig) -> Result<(), String> {
    let slots = jt.mit_joint_slots();
    if slots.is_empty() { return Ok(()); }
    let arm = cfg.arm_joint_indices();
    let Some(gc) = cfg.gravity_comp.as_ref() else {
        return Err("joint_tracking: strategy 'mit' needs the profile's gravity_comp hold gains as gain ceilings".into());
    };
    for k in slots {
        let j = &jt.joints[k];
        let i = arm[k];
        let e = |m: String| format!("joint_tracking.joints[{k}] (mit): {m}");
        let (Some(&kp_max), Some(&kd_max)) = (gc.hold_kp.get(i), gc.hold_kd.get(i)) else {
            return Err(e("no gravity_comp hold gain ceiling for this joint".into()));
        };
        let hold_kp = j.hold_kp.unwrap_or(f32::NAN);
        let hold_kd = j.hold_kd.unwrap_or(f32::NAN);
        for (name, v, max) in [("kp", j.kp, kp_max), ("hold_kp", hold_kp, kp_max), ("kd", j.kd, kd_max), ("hold_kd", hold_kd, kd_max)] {
            if exceeds(v, max) { return Err(e(format!("{name} {v} exceeds the ceiling {max} (gravity_comp hold gains)"))); }
        }
        let lim = &cfg.limits[i];
        let l = j.effort_limit_nm;
        for (name, v) in [("coulomb_nm", j.coulomb_nm), ("viscous_nm_s_rad * max_velocity", j.viscous_nm_s_rad * lim.max_velocity),
            ("inertia_kg_m2 * max_acceleration", j.inertia_kg_m2 * lim.max_acceleration)] {
            if exceeds(v, l) { return Err(e(format!("{name} = {v} does not fit the effort limit {l} N·m"))); }
        }
        if j.gravity_scale > 0.0 && cfg.gravity_model.is_none() {
            return Err(e("gravity_scale > 0 needs a gravity_model".into()));
        }
    }
    Ok(())
}

/// The qualification gate of every selected MIT joint against the stored record and a fresh
/// readback of the motors (`readback`, one entry per joint in `joints`). `supports_switch`:
/// the driver switches single joints to MIT from a POS_VEL base. Every failure is listed.
pub fn check_gate(cfg: &ArmConfig, record: Option<&MitQualificationFile>, zero_offsets: &[f32], joints: &[usize],
    readback: &[MotorIdentity], supports_switch: bool) -> Result<(), String> {
    let jt = cfg.joint_tracking.as_ref();
    let mut errs = Vec::new();
    if !supports_switch { errs.push("the driver cannot switch single joints to MIT from a POS_VEL base".to_string()); }
    let Some(rec) = record else {
        return Err(format!("no mit_qualification record in calibration.json for joints {:?}",
            joints.iter().map(|j| j + 1).collect::<Vec<_>>()));
    };
    if let Err(e) = rec.validate() { errs.push(e); }
    if let Some(inv) = &rec.invalidated { errs.push(format!("mit_qualification invalidated: {}", inv.reason)); }
    let arm = cfg.arm_joint_indices();
    for (n, &i) in joints.iter().enumerate() {
        let jn = i + 1;
        let mut fail = |m: String| errs.push(format!("J{jn}: {m}"));
        let Some(q) = rec.joint(jn) else { fail("no qualification record".into()); continue };
        if Some(q.profile_id.as_str()) != cfg.profile_id.as_deref() { fail(format!("record profile {} != {:?}", q.profile_id, cfg.profile_id)); }
        if cfg.joint_names.get(i) != Some(&q.joint_name) { fail(format!("record joint name {} != {:?}", q.joint_name, cfg.joint_names.get(i))); }
        let fp = zero_fingerprint(zero_offsets.get(i).copied().unwrap_or(0.0));
        if q.zero_fingerprint != fp { fail(format!("joint zero changed since the qualification ({} != {fp})", q.zero_fingerprint)); }
        match readback.get(n) {
            None => fail("no register readback".into()),
            Some(r) => {
                if r.joint != jn { fail(format!("readback is for J{}", r.joint)); }
                if r.timeout_raw == 0 { fail("TIMEOUT readback is 0 (no motor-side command timeout; commission it)".into()); }
                let ident = [("motor_id", u32::from(r.motor_id), u32::from(q.motor_id)), ("reply_id", r.reply_id, q.reply_id),
                    ("esc_id", r.esc_id, q.esc_id), ("mst_id", r.mst_id, q.mst_id), ("sw_ver", r.sw_ver, q.sw_ver),
                    ("timeout_raw", r.timeout_raw, q.timeout_raw)];
                for (name, got, want) in ident {
                    if got != want { fail(format!("{name} readback {got} != qualified {want}")); }
                }
                if r.reply_id != r.mst_id { fail(format!("register replies on 0x{:X}, MST_ID {}", r.reply_id, r.mst_id)); }
                for (name, got, want) in [("pmax", r.pmax, q.pmax), ("vmax", r.vmax, q.vmax), ("tmax", r.tmax, q.tmax)] {
                    if !(got.is_finite() && (got - want).abs() <= 0.01 + want.abs() * 1e-3) { fail(format!("{name} readback {got} != qualified {want}")); }
                }
                if ![damiao::CTRL_MODE_POS_VEL, damiao::CTRL_MODE_MIT].contains(&r.ctrl_mode) { fail(format!("CTRL_MODE {}", r.ctrl_mode)); }
                if let Some(j) = arm.iter().position(|&a| a == i).and_then(|k| jt.and_then(|jt| jt.joints.get(k))) {
                    if j.strategy == TrackingStrategy::Mit {
                        for (name, v, max) in [("kp", j.kp, q.max_kp), ("hold_kp", j.hold_kp.unwrap_or(f32::NAN), q.max_kp),
                            ("kd", j.kd, q.max_kd), ("hold_kd", j.hold_kd.unwrap_or(f32::NAN), q.max_kd),
                            ("effort_limit_nm", j.effort_limit_nm, q.effort_ceiling_nm.min(r.tmax))] {
                            if exceeds(v, max) { fail(format!("{name} {v} exceeds the qualified {max}")); }
                        }
                    }
                }
            }
        }
    }
    if errs.is_empty() { Ok(()) } else { Err(errs.join("; ")) }
}

/// Body of [`TIMEOUT_PATH`].
#[derive(Debug, Clone, PartialEq)]
pub struct TimeoutCommand {
    /// 0-based joints.
    pub joints: Vec<usize>,
    pub timeout_counts: u32,
    pub dry_run: bool,
}

fn invalid(m: impl Into<String>) -> CalibError { CalibError::new(400, "INVALID_COMMAND", m) }

pub fn parse_timeout_command(cfg: &ArmConfig, body: &Value) -> Result<TimeoutCommand, CalibError> {
    let o = body.as_object().ok_or_else(|| invalid("Command body must be an object"))?;
    for k in o.keys() {
        if !["joints", "timeout_counts", "dry_run"].contains(&k.as_str()) { return Err(invalid(format!("unknown field: {k}"))); }
    }
    let arm = cfg.arm_joint_indices();
    let joints: Vec<usize> = o.get("joints").and_then(Value::as_array).filter(|a| !a.is_empty())
        .ok_or_else(|| invalid("joints must be a non-empty array of 1-based arm joints"))?
        .iter().map(|v| v.as_u64().filter(|&j| j >= 1).map(|j| j as usize - 1)
            .filter(|j| arm.contains(j)).ok_or_else(|| invalid("joints must be 1-based arm joints (not the gripper)")))
        .collect::<Result<_, _>>()?;
    if joints.iter().enumerate().any(|(k, j)| joints[..k].contains(j)) { return Err(invalid("joints must be unique")); }
    let timeout_counts = match o.get("timeout_counts") {
        None => DEFAULT_TIMEOUT_COUNTS,
        Some(v) => v.as_u64().filter(|&c| (1..=u64::from(MAX_TIMEOUT_COUNTS)).contains(&c))
            .ok_or_else(|| invalid(format!("timeout_counts must be in 1..={MAX_TIMEOUT_COUNTS}")))? as u32,
    };
    let dry_run = o.get("dry_run").map_or(Ok(false), |v| v.as_bool().ok_or_else(|| invalid("dry_run must be a boolean")))?;
    Ok(TimeoutCommand { joints, timeout_counts, dry_run })
}

#[cfg(not(target_arch = "wasm32"))]
impl crate::supervisor::SupervisorInner {
    fn mit_torque_off_checks(&self) -> Result<(), CalibError> {
        if self.driver.is_none() { return Err(CalibError::new(400, "UNSUPPORTED", "no motor driver")); }
        if self.is_torqued || self.disable_pending {
            return Err(CalibError::new(409, "MIT_TIMEOUT_ARMED", "torque must be off (load mechanically supported) for the TIMEOUT commissioning"));
        }
        if self.calibration_busy { return Err(CalibError::new(409, "CONFIG_BUSY", "a calibration transaction is running")); }
        if self.active_trajectory.is_some() || self.servo_target.is_some() || self.gravity_comp.is_some() {
            return Err(CalibError::new(409, "MIT_TIMEOUT_ARMED", "a motion is running"));
        }
        Ok(())
    }

    /// WS [`TIMEOUT_PATH`]: read (dry run) or commission the motor command timeout of the given
    /// joints. Runs under the supervisor lock with torque off; never moves the arm and never
    /// enables torque; RAM write only (0x55). Returns the readbacks.
    pub fn mit_timeout_command(&mut self, body: &Value) -> Result<Value, CalibError> {
        let cmd = parse_timeout_command(&self.config, body)?;
        self.mit_torque_off_checks()?;
        let d = self.driver.as_ref().expect("checked above");
        let mut driver = d.lock().map_err(|_| CalibError::new(500, "DRIVER", "driver lock poisoned"))?;
        let unit = json!({"unit_us": damiao::TIMEOUT_UNIT_US_DOCUMENTED, "unit_verified": false,
            "note": "50 us per count is documented by Damiao and not verified on this firmware; RAM only (0x55), lost at power-off"});
        if cmd.dry_run {
            let rb = driver.read_motor_identity(&cmd.joints).map_err(|e| CalibError::new(502, "MIT_TIMEOUT_READ", e.to_string()))?;
            return Ok(json!({"dry_run": true, "readback": rb, "timeout_unit": unit}));
        }
        let res = driver.commission_motor_timeout(&cmd.joints, cmd.timeout_counts)
            .map_err(|e| CalibError::new(502, "MIT_TIMEOUT_FAILED", e.to_string()))?;
        tracing::warn!("MIT TIMEOUT commissioned (RAM) for joints {:?}: {} counts", cmd.joints.iter().map(|j| j + 1).collect::<Vec<_>>(), cmd.timeout_counts);
        Ok(json!({"dry_run": false, "timeout_counts": cmd.timeout_counts, "commissioning": res, "timeout_unit": unit}))
    }

    /// Check of WS [`QUALIFICATION_PATH`] `{record}` under the lock: the record is valid and, for
    /// every joint it holds, bound to this profile, joint and joint zero and to a fresh readback of
    /// the motor (identity, firmware, ranges, nonzero TIMEOUT equal to the record). Torque off.
    /// Returns the calibration file to store.
    pub fn mit_qualification_check(&mut self, body: &Value) -> Result<crate::calibration::CalibrationFile, CalibError> {
        let o = body.as_object().ok_or_else(|| invalid("Command body must be an object"))?;
        if o.keys().any(|k| k != "record") { return Err(invalid("the body holds only `record`")); }
        let rec: MitQualificationFile = serde_json::from_value(o.get("record").cloned().unwrap_or(Value::Null))
            .map_err(|e| invalid(format!("record: {e}")))?;
        rec.validate().map_err(invalid)?;
        if rec.invalidated.is_some() { return Err(invalid("record: invalidated records cannot be stored")); }
        self.mit_torque_off_checks()?;
        let joints: Vec<usize> = rec.joints.iter().map(|j| j.joint - 1).collect();
        let arm = self.config.arm_joint_indices();
        if let Some(j) = joints.iter().find(|j| !arm.contains(j)) { return Err(invalid(format!("record: J{} is not an arm joint", j + 1))); }
        let rb = {
            let d = self.driver.as_ref().expect("checked above");
            let mut driver = d.lock().map_err(|_| CalibError::new(500, "DRIVER", "driver lock poisoned"))?;
            driver.read_motor_identity(&joints).map_err(|e| CalibError::new(502, "MIT_TIMEOUT_READ", e.to_string()))?
        };
        // the record's own gain ceilings are checked against the profile's at arm time
        let mut cfg = self.config.clone();
        if let Some(jt) = cfg.joint_tracking.as_mut() {
            for j in jt.joints.iter_mut() { j.strategy = TrackingStrategy::Position; }
        }
        check_gate(&cfg, Some(&rec), &self.joint_zero_offsets, &joints, &rb, true)
            .map_err(|e| CalibError::new(409, "MIT_QUALIFICATION_MISMATCH", e))?;
        let mut file = self.calibration.clone().unwrap_or_default();
        file.mit_qualification = Some(rec);
        file.validate().map_err(invalid)?;
        Ok(file)
    }

    /// The MIT qualification gate for this arm: Ok when MIT is not requested.
    pub fn mit_gate(&self) -> Result<(), String> {
        let joints = mit_joints(&self.config);
        if joints.is_empty() { return Ok(()); }
        let Some(d) = self.driver.as_ref() else { return Err("MIT needs a motor driver".into()) };
        let mut driver = d.lock().map_err(|_| "driver lock poisoned".to_string())?;
        let supports = driver.supports_mit_mode_switch();
        let rb = driver.read_motor_identity(&joints).map_err(|e| format!("register readback failed: {e}"))?;
        check_gate(&self.config, self.calibration.as_ref().and_then(|c| c.mit_qualification.as_ref()),
            &self.joint_zero_offsets, &joints, &rb, supports)
    }
}

/// WS [`QUALIFICATION_PATH`] task (the calibration busy flag is held, so no other calibration
/// write changes `calibration.json` meanwhile): recheck the lease, write the complete document
/// atomically (temporary file + rename), then publish it live. A failed write changes nothing.
#[cfg(not(target_arch = "wasm32"))]
pub async fn store_qualification(sup: &crate::supervisor::ArmSupervisor, deps: &crate::calibration::CalibrationDeps,
    file: crate::calibration::CalibrationFile, check: &crate::calibration::LeaseCheck) -> Result<Value, CalibError> {
    {
        let mut inner = sup.inner().write().await;
        check(&mut inner)?;
    }
    let text = serde_json::to_string_pretty(&file).map_err(|e| CalibError::new(500, "INTERNAL", e.to_string()))?;
    crate::calibration::put_file(deps.path.clone(), Some(text)).await
        .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("calibration file {}: {e}; nothing changed", deps.path.display())))?;
    let joints: Vec<usize> = file.mit_qualification.as_ref().map_or_else(Vec::new, |q| q.joints.iter().map(|j| j.joint).collect());
    let mut inner = sup.inner().write().await;
    inner.set_calibration(Some(file));
    tracing::warn!("MIT qualification stored for joints {joints:?}");
    Ok(json!({"stored": true, "joints": joints}))
}

/// `v` is above `max` or not a number.
fn exceeds(v: f32, max: f32) -> bool { v.is_nan() || v > max }
