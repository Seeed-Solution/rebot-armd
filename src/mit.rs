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
#[cfg(not(target_arch = "wasm32"))]
use crate::driver::JointTarget;

/// Bounded, zero-allocation error text returned by Step-5 output writes.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) type OutputErrorText = crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>;

/// WS path of the TIMEOUT commissioning.
pub const TIMEOUT_PATH: &str = "/api/v1/calibration/mit_timeout";
/// WS path that stores the qualification record.
pub const QUALIFICATION_PATH: &str = "/api/v1/calibration/mit_qualification";
/// Dynamic-output deadline while tracking owns an MIT joint: a feedback age or a gap between
/// dynamic outputs of this length cancels the group and latches BUS_LOSS.
pub const MIT_DEADLINE: Duration = Duration::from_millis(45);
/// Code of a clear_fault that needs the operator's confirmation of the MIT expiry recovery.
pub const RECOVERY_CONFIRM_CODE: &str = "MIT_RECOVERY_CONFIRM_REQUIRED";
/// What the confirmation accepts: torque goes off on every joint.
pub const RECOVERY_WARNING: &str = "A motor's command timeout expired after the MIT loss. Clearing recovers by turning torque off on every joint: \
    the arm drops like a park torque-off. Support the arm or bring it to rest, then clear again with {\"mit_recovery\": \"torque_off_all\"}";
/// Code of a clear_fault refused inside the post-trip quiet period (no frame is sent).
pub const QUIET_PERIOD_CODE: &str = "MIT_QUIET_PERIOD";
/// Body value of clear_fault that confirms the MIT expiry recovery.
pub const RECOVERY_CONFIRM_VALUE: &str = "torque_off_all";

/// Ages and gaps from this value up to the deadline are logged and counted as near misses.
pub const MIT_NEAR_MISS: Duration = Duration::from_millis(36);
/// Margin of the activation criterion: control period + measured worst-case latency + this
/// must stay under [`MIT_DEADLINE`].
pub const MIT_LATENCY_MARGIN_MS: f32 = 5.0;

/// The single deadline comparison used everywhere: `elapsed` reached the MIT deadline.
pub fn past_deadline(elapsed: Duration) -> bool { elapsed >= MIT_DEADLINE }

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

/// A TIMEOUT commissioning batch that stopped part way (0-based joints): what was applied and
/// confirmed, the joint whose write went out unconfirmed (its RAM value is unknown), and the
/// joints not attempted.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug)]
pub struct TimeoutBatchError {
    pub applied: Vec<TimeoutCommissioning>,
    pub uncertain: Vec<usize>,
    pub not_attempted: Vec<usize>,
    pub error: crate::driver::ArmDriverError,
}

#[cfg(not(target_arch = "wasm32"))]
impl std::fmt::Display for TimeoutBatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { write!(f, "{}", self.error) }
}

#[cfg(not(target_arch = "wasm32"))]
impl From<crate::driver::ArmDriverError> for TimeoutBatchError {
    fn from(error: crate::driver::ArmDriverError) -> Self { Self { applied: Vec::new(), uncertain: Vec::new(), not_attempted: Vec::new(), error } }
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
    /// Measured worst-case scheduling + RX queue + TX latency of the control loop (ms): with the
    /// control period and a margin it must fit under the 45 ms deadline ([`check_gate`]).
    pub worst_latency_ms: f32,
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
                ("effort_ceiling_nm", j.effort_ceiling_nm), ("measured_expiry_ms", j.measured_expiry_ms), ("worst_latency_ms", j.worst_latency_ms)] {
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

/// Zero-allocation bitmask of 0-based arm joint indices selected for MIT
/// (0 when MIT is not requested; supports up to 64 joints without heap allocation).
pub fn mit_joint_mask(cfg: &ArmConfig) -> u64 {
    let Some(jt) = cfg.joint_tracking.as_ref().filter(|jt| jt.mit_requested()) else {
        return 0;
    };
    let arm = cfg.arm_joints();
    let mut mask = 0u64;
    for (k, j) in jt.joints.iter().enumerate() {
        if j.strategy == crate::tracking::TrackingStrategy::Mit {
            if let Some(&idx) = arm.get(k) {
                if idx < 64 {
                    mask |= 1u64 << idx;
                }
            }
        }
    }
    mask
}

/// 0-based arm joint indices selected for MIT (empty when MIT is not requested).
pub fn mit_joints(cfg: &ArmConfig) -> Vec<usize> {
    let mask = mit_joint_mask(cfg);
    if mask == 0 {
        return Vec::new();
    }
    (0..cfg.dof.min(64)).filter(|&i| (mask & (1u64 << i)) != 0).collect()
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
        // activation criterion: the deadline is a hard ceiling, so the configured period plus the
        // measured worst-case latency (and a margin) must fit under it
        let period_ms = 1000.0 / cfg.control_frequency_hz.max(1) as f32;
        let budget = period_ms + q.worst_latency_ms + MIT_LATENCY_MARGIN_MS;
        if budget.is_nan() || budget >= MIT_DEADLINE.as_secs_f32() * 1000.0 {
            fail(format!("control period {period_ms:.1} ms + measured worst-case latency {} ms + margin {MIT_LATENCY_MARGIN_MS} ms does not fit under the {} ms deadline",
                q.worst_latency_ms, MIT_DEADLINE.as_millis()));
        }
        if !q.timeout_unit_verified { fail("the TIMEOUT unit and expiry are not verified on the device (timeout_unit_verified false)".into()); }
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
        let res = driver.commission_motor_timeout(&cmd.joints, cmd.timeout_counts).map_err(|e| {
            let one = |v: &[usize]| v.iter().map(|j| j + 1).collect::<Vec<_>>();
            let mut err = CalibError::new(502, "MIT_TIMEOUT_FAILED", format!("{}; applied {:?}, uncertain {:?} (write sent, not confirmed), not attempted {:?}",
                e.error, e.applied.iter().map(|c| c.before.joint).collect::<Vec<_>>(), one(&e.uncertain), one(&e.not_attempted)));
            err.extra.insert("applied".into(), json!(e.applied.iter().map(|c| c.before.joint).collect::<Vec<_>>()));
            err.extra.insert("commissioning".into(), json!(e.applied));
            err.extra.insert("uncertain".into(), json!(one(&e.uncertain)));
            err.extra.insert("not_attempted".into(), json!(one(&e.not_attempted)));
            err
        })?;
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
        file.set_mit_record(&rec);
        file.validate().map_err(invalid)?;
        Ok(file)
    }

    /// The MIT qualification gate for this arm: Ok when MIT is not requested.
    pub fn mit_gate(&self) -> Result<(), String> {
        let joints = mit_joints(&self.config);
        if joints.is_empty() { return Ok(()); }
        // fail-closed: no MIT unless the RT executor runs at SCHED_FIFO with locked memory
        if let Some(why) = crate::rt_core::sched::mit_refusal() { return Err(why); }
        let Some(d) = self.driver.as_ref() else { return Err("MIT needs a motor driver".into()) };
        let mut driver = d.lock().map_err(|_| "driver lock poisoned".to_string())?;
        let supports = driver.supports_mit_mode_switch();
        let rb = driver.read_motor_identity(&joints).map_err(|e| format!("register readback failed: {e}"))?;
        let rec = match self.calibration.as_ref().map(|c| c.mit_record()) {
            Some(Err(e)) => return Err(format!("the stored record is invalid: {e}")),
            Some(Ok(r)) => r,
            None => None,
        };
        check_gate(&self.config, rec.as_ref(), &self.joint_zero_offsets, &joints, &rb, supports)
    }
}

/// WS [`QUALIFICATION_PATH`] task (the calibration busy flag is held, so no other calibration
/// write changes `calibration.json` meanwhile): recheck the lease, write the complete document
/// atomically (temporary file + rename), then publish it live. A failed write changes nothing.
#[cfg(not(target_arch = "wasm32"))]
pub async fn store_qualification(sup: &crate::supervisor::ArmSupervisor, deps: &crate::calibration::CalibrationDeps,
    file: crate::calibration::CalibrationFile, check: &crate::calibration::LeaseCheck) -> Result<Value, CalibError> {
    if sup.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return Err(CalibError::new(
            409,
            "EXECUTOR_UNSUPPORTED",
            format!("{QUALIFICATION_PATH} is not available while the RT executor owns the arm; use the legacy owner"),
        ));
    }
    {
        let mut inner = sup.inner().write().await;
        check(&mut inner)?;
    }
    let text = serde_json::to_string_pretty(&file).map_err(|e| CalibError::new(500, "INTERNAL", e.to_string()))?;
    crate::calibration::put_file(deps.path.clone(), Some(text)).await
        .map_err(|e| CalibError::new(500, "PERSIST_FAILED", format!("calibration file {}: {e}; nothing changed", deps.path.display())))?;
    let joints: Vec<usize> = file.mit_record().ok().flatten().map_or_else(Vec::new, |q| q.joints.iter().map(|j| j.joint).collect());
    let mut inner = sup.inner().write().await;
    inner.set_calibration(Some(file));
    tracing::warn!("MIT qualification stored for joints {joints:?}");
    Ok(json!({"stored": true, "joints": joints}))
}

/// `v` is above `max` or not a number.
fn exceeds(v: f32, max: f32) -> bool { v.is_nan() || v > max }

/// Output owner of the MIT joints ([`MitRuntime::owner`]).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MitOwner {
    /// Position control owns every joint (also: MIT not requested).
    #[default]
    Off,
    /// MIT tracking owns [`MitRuntime::joints`] after a confirmed stationary handover.
    Active,
    /// The group was cancelled (deadline, partial or failed write, uncertain mode): no output
    /// until an explicit fault clearance and a rearm.
    Lost,
}

/// Slew limit of the feed-forward torque (N·m/s), as the ordinary gravity output.
pub const MIT_FF_SLEW_NM_S: f32 = 15.0;
/// Velocity scale of the Coulomb friction term (rad/s).
pub const MIT_COULOMB_VEL_RAD_S: f32 = 0.02;
/// Hold-off of autonomous driver frames after a deadline trip, at least.
pub const MIT_TRIP_HOLDOFF_MIN: Duration = Duration::from_millis(250);

/// MIT tracking state of the supervisor (`SupervisorInner::mit`).
#[derive(Debug, Clone, Default)]
pub struct MitRuntime {
    pub owner: MitOwner,
    /// 0-based joints MIT owns while `Active`.
    pub joints: Vec<usize>,
    /// Feed-forward torque per joint of the last flushed batch (seeded by the handover hold).
    pub last_ff: Vec<f32>,
    /// Last flushed dynamic batch (supervisor clock).
    pub last_output: Option<std::time::Instant>,
    /// The qualification gate passed at the current arm: a handover may be tried.
    pub armed_ok: bool,
    /// Why the last handover (or hand back) failed.
    pub handover_error: Option<String>,
    /// Why the group was cancelled.
    pub lost_reason: Option<String>,
    /// A park was requested while MIT moved the arm: it starts after rest and the hand back.
    pub park_after_hand_back: bool,
    /// Action id of that deferred park (its outcome is reported on it).
    pub deferred_park_action: Option<u64>,
    /// A rearm was refused while MIT moved the arm: brake, then hand back at rest.
    pub revoke_pending: bool,
    /// Feedback ages or output gaps at or above [`MIT_NEAR_MISS`] but under the deadline.
    pub near_misses: u64,
    /// The current clear_fault carries the operator's recovery confirmation.
    pub recovery_confirmed: bool,
}

#[cfg(not(target_arch = "wasm32"))]
fn wire_limits(cfg: &ArmConfig, j: usize) -> damiao::DamiaoLimits {
    cfg.damiao_ranges.as_ref().and_then(|r| r.get(j)).map_or_else(damiao::DamiaoLimits::default, |r| r.limits())
}

/// Round `tau` to the 12-bit MIT torque grid of `l` and keep it inside `[lo, hi]` (one step
/// inward at a bound); None when no grid value lies inside.
fn quantize_into(tau: f32, lo: f32, hi: f32, l: &damiao::DamiaoLimits) -> Option<f32> {
    let grid = |t: f32| damiao::uint_to_float(damiao::float_to_uint(t, l.t_min, l.t_max, 12), l.t_min, l.t_max, 12);
    let lsb = (l.t_max - l.t_min) / 4095.0;
    let eps = 1e-6;
    let mut q = grid(tau);
    if q > hi + eps { q = grid(q - lsb); }
    if q < lo - eps { q = grid(q + lsb); }
    (q >= lo - eps && q <= hi + eps).then_some(q)
}

#[cfg(not(target_arch = "wasm32"))]
impl crate::supervisor::SupervisorInner {
    fn mit_slot(&self, j: usize) -> Option<&crate::tracking::JointTrackingJoint> {
        let k = self.config.arm_joints().iter().position(|&a| a == j)?;
        self.config.joint_tracking.as_ref()?.joints.get(k)
    }

    pub fn mit_active(&self) -> bool { self.mit.owner == MitOwner::Active }

    /// Zero-allocation MIT motion blocker check (fixed `u64` bitmask, no `Vec`, no `String`):
    /// true when MIT is requested on an armed arm but does not currently own its joints.
    pub fn mit_motion_blocked(&self) -> bool {
        if !self.is_torqued || mit_joint_mask(&self.config) == 0 {
            return false;
        }
        self.mit.park_after_hand_back
            || self.mit.revoke_pending
            || self.mit.owner != MitOwner::Active
    }

    /// Why a motion may not start: MIT is requested on an armed arm but does not own its joints.
    pub fn mit_motion_blocker(&self) -> Option<String> {
        if !self.mit_motion_blocked() { return None; }
        if self.mit.park_after_hand_back { return Some("a park waits for the MIT hand back at rest".into()); }
        if self.mit.revoke_pending { return Some("MIT authorization revoked: braking to rest and handing back".into()); }
        match self.mit.owner {
            MitOwner::Active => None,
            MitOwner::Off => Some(format!("MIT tracking is not active: {}",
                self.mit.handover_error.as_deref().unwrap_or("the handover waits for the arm at rest"))),
            MitOwner::Lost => Some(format!("MIT tracking was cancelled: {}", self.mit.lost_reason.as_deref().unwrap_or("?"))),
        }
    }

    /// Gravity torque per joint from the fresh measured posture, when a selected joint uses it.
    fn mit_gravity(&self) -> Result<Vec<f32>, String> {
        let dof = self.config.dof;
        let arm_dof = self.config.arm_dof();
        let needed = self.mit.joints.iter().chain(mit_joints(&self.config).iter())
            .any(|&j| self.mit_slot(j).is_some_and(|c| c.gravity_scale > 0.0));
        if !needed { return Ok(vec![0.0; dof]); }
        if self.config.gravity_model.is_none() || self.measured_positions.len() < arm_dof {
            return Err("gravity model unavailable".into());
        }
        let tau = self.config.arm_model.gravity_torques(&self.measured_positions[..arm_dof]).map_err(|e| format!("gravity: {e:?}"))?;
        if !tau[..arm_dof].iter().all(|t| t.is_finite()) { return Err("gravity torque not finite".into()); }
        let mut g = vec![0.0; dof];
        g[..arm_dof].copy_from_slice(&tau[..arm_dof]);
        Ok(g)
    }

    /// Fresh feedback of every joint: no age past the deadline ([`past_deadline`]).
    fn mit_feedback_fresh(&self) -> bool {
        if !self.feedback_numeric_valid { return false; }
        let ages = match self.driver.as_ref().and_then(|d| d.lock().ok()) { Some(drv) => drv.feedback_ages(), None => return true };
        ages.len() == self.config.dof && ages.iter().all(|a| a.is_some_and(|a| !past_deadline(a)))
    }

    /// A stationary MIT hold: the reference is at rest (fixed nominal, zero velocity and
    /// acceleration), or a safety hold or fault owns the arm.
    fn mit_in_hold(&self, v_d: f32, a_d: f32) -> bool {
        self.safety.state().is_stopped() || self.collision.hold.is_some() || self.servo_blocked_hold
            || self.torque_guard.hold.is_some() || (v_d == 0.0 && a_d == 0.0)
    }

    fn mit_holdoff(&self) -> Duration {
        let rec = self.calibration.as_ref().and_then(|c| c.mit_record().ok().flatten());
        let rec = rec.as_ref();
        let worst = mit_joints(&self.config).iter().filter_map(|&j| rec.and_then(|r| r.joint(j + 1)))
            .map(|q| Duration::from_secs_f32((q.measured_expiry_ms * 2.0 / 1000.0).clamp(0.0, 10.0))).max().unwrap_or_default();
        worst.max(MIT_TRIP_HOLDOFF_MIN)
    }

    /// MIT frame fields for every owned joint of `targets` (nominal `q_d`, `v_d` from the
    /// target, `a_d` from the reference) and the feed-forward torque of the batch:
    /// `pd = kp(q_d - q_m) + kd(v_d - v_m)`, `raw = s·G(q_m) + c·clamp(v_d / 0.02) + b·v_d + J·a_d`
    /// (dynamic terms 0 in a hold), `tau_ff = clamp(raw, I)` with
    /// `I = [-L, L] ∩ [-L - pd, L - pd] ∩ [last ± 15 dt]` and, in a torque-guard hold, the
    /// relieving half-space `sign·(pd + tau_ff) <= 0`, rounded to the wire grid inside `I`.
    /// Refuses (nothing is written) for stale feedback, missing gravity, values outside the
    /// wire ranges or an empty interval.
    pub fn mit_prepare(&self, targets: &mut [JointTarget], dt: f64) -> Result<Vec<f32>, String> {
        if !self.mit_feedback_fresh() { return Err("feedback older than the MIT deadline".into()); }
        let g = self.mit_gravity()?;
        let dt = (dt as f32).clamp(0.001, 0.1);
        let mut ff = self.mit.last_ff.clone();
        ff.resize(self.config.dof, 0.0);
        for &j in &self.mit.joints {
            let t = targets.get_mut(j).ok_or("target vector too short")?;
            let (q_d, v_d) = (t.position, t.velocity.unwrap_or(0.0));
            let a_d = self.commanded_accelerations.get(j).copied().unwrap_or(0.0);
            let hold = self.mit_in_hold(v_d, a_d);
            let last = self.mit.last_ff.get(j).copied().unwrap_or(0.0);
            let (frame, tau) = self.mit_frame(j, q_d, v_d, a_d, hold, g[j], Some((last, dt)))?;
            *t = frame;
            ff[j] = tau;
        }
        Ok(ff)
    }

    /// One MIT frame for joint `j` on the decoded wire grid and its tau_ff (see
    /// [`Self::mit_prepare`]); `slew` = (last tau_ff, dt) limits the change, None (the handover's
    /// first frame) does not. Every bound is checked on the fields the motor decodes.
    #[allow(clippy::too_many_arguments)]
    fn mit_frame(&self, j: usize, q_d: f32, v_d: f32, a_d: f32, hold: bool, g_j: f32, slew: Option<(f32, f32)>)
        -> Result<(JointTarget, f32), String> {
        let c = self.mit_slot(j).ok_or_else(|| format!("J{}: no tracking entry", j + 1))?;
        let (q_m, v_m) = (self.measured_positions[j], self.measured_velocities[j]);
        if ![q_d, v_d, a_d, q_m, v_m].iter().all(|x| x.is_finite()) { return Err(format!("J{}: non-finite reference or feedback", j + 1)); }
        let (kp, kd) = if hold { (c.hold_kp.unwrap_or(0.0), c.hold_kd.unwrap_or(0.0)) } else { (c.kp, c.kd) };
        let wl = wire_limits(&self.config, j);
        let off = self.joint_zero_offsets.get(j).copied().unwrap_or(0.0);
        if (q_d + off).abs() > wl.p_max || v_d.abs() > wl.v_max || kp > wl.kp_max || kd > wl.kd_max || kp <= 0.0 || kd <= 0.0 {
            return Err(format!("J{}: MIT frame outside the wire ranges (q {:.3}, v {v_d:.3}, kp {kp}, kd {kd})", j + 1, q_d + off));
        }
        // every field as the motor decodes it (position in the motor frame, after the zero offset)
        let grid = |x: f32, lo: f32, hi: f32, bits: u32| damiao::uint_to_float(damiao::float_to_uint(x, lo, hi, bits), lo, hi, bits);
        let q_w = grid(q_d + off, wl.p_min, wl.p_max, 16) - off;
        let v_w = grid(v_d, wl.v_min, wl.v_max, 12);
        let kp_w = grid(kp, wl.kp_min, wl.kp_max, 12);
        let kd_w = grid(kd, wl.kd_min, wl.kd_max, 12);
        let pd = kp_w * (q_w - q_m) + kd_w * (v_w - v_m);
        let dynamic = if hold { 0.0 } else {
            c.coulomb_nm * (v_d / MIT_COULOMB_VEL_RAD_S).clamp(-1.0, 1.0) + c.viscous_nm_s_rad * v_d + c.inertia_kg_m2 * a_d
        };
        let raw = c.gravity_scale * g_j + dynamic;
        let l = c.effort_limit_nm;
        let mut lo = (-l).max(-l - pd);
        let mut hi = l.min(l - pd);
        if let Some((last, dt)) = slew {
            lo = lo.max(last - MIT_FF_SLEW_NM_S * dt);
            hi = hi.min(last + MIT_FF_SLEW_NM_S * dt);
        }
        let guard = self.torque_guard.hold.filter(|h| h.joint == j);
        if let Some(h) = guard {
            if h.sign > 0.0 { hi = hi.min(-pd); } else { lo = lo.max(-pd); }
        }
        if lo.is_nan() || hi.is_nan() || lo > hi {
            return Err(format!("J{}: empty effort interval [{lo:.3}, {hi:.3}] N·m (pd {pd:.3}, L {l})", j + 1));
        }
        let tau = quantize_into(raw.clamp(lo, hi), lo, hi, &wl)
            .ok_or_else(|| format!("J{}: no wire torque value inside [{lo:.4}, {hi:.4}]", j + 1))?;
        let total = pd + tau;
        if total.abs() > l + 1e-6 || guard.is_some_and(|h| h.sign * total > 1e-6) {
            return Err(format!("J{}: decoded effort {total:.4} N·m outside the limit {l} or the relieving side", j + 1));
        }
        Ok((JointTarget::mit(q_w, v_w, kp_w, kd_w, tau), tau))
    }

    /// Write one output batch (every ordinary output path). Without MIT: the bounded driver write
    /// (distinguishing `Complete`, transient `Deferred`, `Expired`, and hard `Error`). With MIT
    /// active: the owned joints get MIT frames ([`Self::mit_prepare`]); the feed-forward history
    /// advances only after `Complete`; any non-`Complete` cancels the group. MIT lost: `None`.
    pub(crate) fn write_output_scratch(&mut self, dt: f64) -> crate::supervisor::TickOutput {
        let mut targets = std::mem::take(&mut self.scratch.targets);
        let r = self.write_output_slice(&mut targets, dt, None);
        self.scratch.targets = targets;
        r
    }

    /// [`Self::write_output_scratch`] with explicit active joint limits (used by rejection-fallback
    /// braking before `tracking.fallback_ramp` is installed, reusing `scratch.targets` with 0 alloc).
    pub(crate) fn write_output_scratch_with_limits(
        &mut self,
        dt: f64,
        limits: &[crate::model::JointLimit],
    ) -> crate::supervisor::TickOutput {
        let mut targets = std::mem::take(&mut self.scratch.targets);
        let r = self.write_output_slice(&mut targets, dt, Some(limits));
        self.scratch.targets = targets;
        r
    }

    /// Velocity ceiling (rad/s) currently authorized for joint `g` by the active motion,
    /// persistent gripper hold, fallback ramp, servo stream, or profile limit.
    fn active_joint_velocity_ceiling(&self, g: usize) -> Option<f32> {
        let base = if let Some(active) = &self.active_trajectory {
            if let crate::motion::Reference::Gripper(gm) = &active.reference {
                if gm.gripper_index == g {
                    return Some(gm.wire_velocity_ceiling());
                }
            }
            active.motion_limits.as_ref().and_then(|ls| ls.get(g).map(|l| l.max_velocity))
        } else if let Some(ramp) = &self.tracking.fallback_ramp {
            ramp.limits.get(g).map(|l| l.max_velocity)
        } else if let Some(lim) = self.tracking.retained_motion_limits.as_ref().and_then(|ls| ls.get(g)) {
            Some(lim.max_velocity)
        } else if self.servo_target.is_some() {
            let mut v = self.config.limits.get(g)?.max_velocity;
            #[cfg(feature = "policy-runtime")]
            if let Some(ref run) = self.policy_run {
                if run.mode == crate::policy_runtime::PolicyMode::Live {
                    if let Some(ref gate) = run.verified_release_gate {
                        v *= (gate.max_speed_scale() as f32).clamp(0.1, 1.0);
                    }
                }
            }
            Some(v)
        } else {
            None
        };
        let ceiling = base.or_else(|| self.config.limits.get(g).map(|l| l.max_velocity))?;
        Some(self.gripper_hold_vel_ceiling.map_or(ceiling, |hold_v| ceiling.min(hold_v)))
    }

    fn write_output_slice(
        &mut self,
        targets: &mut [JointTarget],
        dt: f64,
        override_limits: Option<&[crate::model::JointLimit]>,
    ) -> crate::supervisor::TickOutput {
        use crate::driver::TxBatchOutcome;
        use crate::supervisor::TickOutput;
        // an E-stop or authorized stop requested since the tick applied the last one: no dynamic
        // output reaches the bus (the tick applies the stop policy right after, see `tick_control`)
        if self.urgent_stop_unapplied() { return TickOutput::None; }
        let is_hold = self.safety.state().is_stopped();
        let ff = match self.mit.owner {
            MitOwner::Off => None,
            MitOwner::Lost => return TickOutput::None,
            MitOwner::Active => match self.mit_prepare(targets, dt) {
                Ok(ff) => Some(ff),
                Err(e) => {
                    let mut buf = OutputErrorText::empty();
                    let _ = core::fmt::write(&mut buf, format_args!("MIT output refused: {e}"));
                    return TickOutput::Error {
                        message: buf,
                        now: self.mit_now(),
                        max_age: crate::driver::DAMIAO_TX_FRAME_MAX_AGE,
                        is_hold,
                        sent_any: false,
                    };
                }
            },
        };
        let active_pf_ceiling = self.config.gripper_index().and_then(|g| {
            override_limits
                .and_then(|ls| ls.get(g).map(|l| l.max_velocity))
                .map(|c| self.gripper_hold_vel_ceiling.map_or(c, |h| c.min(h)))
                .or_else(|| self.active_joint_velocity_ceiling(g))
        });
        let cutoff = self.active_tx_cutoff;
        let res = self.driver.as_ref().and_then(|d| d.lock().ok())
            .map(|mut drv| {
                // Enforce the active authorized velocity ceiling across both moving ticks and
                // zero-velocity position targets (e.g. grasp squeeze, settle, hold) on an active
                // pos-force gripper joint:
                // - When the authorized ceiling quantizes to raw >= 1, bound zero-velocity targets
                //   and any over-ceiling velocity to `wire_speed_f32(ceiling_raw)`, and floor
                //   transient sub-quantum accel/braking velocities (0 < |v| < 0.01) to raw 1
                //   (always <= authorized ceiling and <= close_speed_rad_s).
                // - When the authorized ceiling itself encodes to 0 (unrepresentable), never fall
                //   back to vlim()/close_speed_rad_s or bump up to raw 1: forward the
                //   unrepresentable ceiling/velocity so DamiaoArmDriver fails closed with 0 frames.
                if let Some(cap) = drv.gripper_pos_force_info().filter(|c| c.active) {
                    if let Some(t) = targets.get_mut(cap.joint) {
                        if !t.velocity.is_some_and(|v| !v.is_finite()) {
                            let ceiling_raw = active_pf_ceiling
                                .filter(|c| c.is_finite() && *c > 0.0)
                                .map(crate::model::DamiaoGripperDrive::quantize_vel_raw)
                                .unwrap_or(0);
                            let close_ok = cap.close_speed_rad_s.is_finite()
                                && crate::model::DamiaoGripperDrive::quantize_vel_raw(cap.close_speed_rad_s) >= 1;
                            if ceiling_raw >= 1 && close_ok {
                                let ceiling_wire = crate::model::DamiaoGripperDrive::wire_speed_f32(ceiling_raw);
                                if t.velocity.is_none_or(|v| v == 0.0) {
                                    t.velocity = Some(ceiling_wire);
                                } else if let Some(v) = t.velocity {
                                    let wire_raw = (f64::from(v.abs()) * 100.0)
                                        .ceil()
                                        .clamp(1.0, f64::from(ceiling_raw)) as u16;
                                    t.velocity = Some(crate::model::DamiaoGripperDrive::wire_speed_f32(wire_raw).copysign(v));
                                }
                            } else if t.velocity.is_none_or(|v| v == 0.0) {
                                if let Some(c) = active_pf_ceiling.filter(|c| c.is_finite() && *c > 0.0) {
                                    t.velocity = Some(c);
                                }
                            }
                        }
                    }
                }
                // Single TX cutoff choke point under the driver lock:
                // - Safety/stopped holds restore the unclipped release cap so an action deadline
                //   never blocks a safety hold.
                // - Motion targets check the action cutoff right before calling the driver (guarding
                //   non-CAN/sim/mock drivers and any wait on `d.lock()`), while `DamiaoArmDriver`
                //   additionally enforces the exact same `tx_release_deadline` per-frame inside `send()`.
                let action_cap = cutoff.and_then(|c| c.drv_action_cutoff);
                if is_hold {
                    if let Some(c) = cutoff {
                        if c.drv_action_cutoff.is_some() {
                            drv.set_tx_release_deadline(Some(c.drv_release_cap));
                        }
                    }
                } else if let Some(cap) = action_cap {
                    let pre_now = drv.clock_now();
                    if pre_now >= cap {
                        drv.discard_pending_tx();
                        return TickOutput::ActionDeadlineRefused { drv_now: pre_now };
                    }
                }
                let write_res = if is_hold {
                    drv.write_estop_hold_bounded(targets)
                } else {
                    drv.write_joint_targets_bounded(targets)
                };
                let now = drv.clock_now();
                if !is_hold
                    && matches!(
                        write_res,
                        Ok(TxBatchOutcome::Deferred { .. })
                            | Err(crate::driver::ArmDriverError::Transport(
                                crate::transport::TransportError::Timeout
                            ))
                    )
                    && action_cap.is_some_and(|cap| now >= cap)
                {
                    drv.discard_pending_tx();
                    return TickOutput::ActionDeadlineRefused { drv_now: now };
                }
                let max_age = drv.tx_frame_max_age();
                let hold_sent_any = is_hold && drv.last_hold_tx_sent_any();
                match write_res {
                    Ok(TxBatchOutcome::Complete) => TickOutput::Complete { is_hold },
                    Ok(TxBatchOutcome::Deferred { pending }) => TickOutput::Deferred {
                        pending,
                        now,
                        max_age,
                        oldest_age: drv.oldest_pending_tx_age(),
                        is_hold,
                        sent_any: hold_sent_any,
                        sent_mask: if is_hold { 0 } else { drv.last_target_tx_sent_mask() },
                    },
                    Ok(TxBatchOutcome::Expired { dropped, pending }) => TickOutput::Expired {
                        dropped,
                        pending,
                        now,
                        max_age,
                        oldest_age: drv.last_tx_expired_age(),
                        is_hold,
                        sent_any: hold_sent_any,
                    },
                    Err(crate::driver::ArmDriverError::EstopInhibited) => {
                        TickOutput::EstopInhibited { is_hold }
                    }
                    Err(e) => {
                        let mut buf = OutputErrorText::empty();
                        e.write_bounded(&mut buf);
                        TickOutput::Error {
                            message: buf,
                            now,
                            max_age,
                            is_hold,
                            sent_any: hold_sent_any,
                        }
                    }
                }
            })
            .unwrap_or(TickOutput::None);
        if let Some(ff) = ff {
            match &res {
                TickOutput::Complete { .. } => {
                    self.mit.last_ff = ff;
                    self.mit.last_output = Some(self.mit_now());
                }
                TickOutput::EstopInhibited { .. } => {
                    self.mit_lose(
                        format!("MIT batch write failed: {}", crate::driver::ESTOP_INHIBITED),
                        crate::safety::FaultCode::BusLoss,
                    );
                }
                TickOutput::Error { message, .. } => {
                    self.mit_lose(format!("MIT batch write failed: {message}"), crate::safety::FaultCode::BusLoss);
                }
                TickOutput::Deferred { pending, .. } => {
                    self.mit_lose(format!("MIT batch write deferred ({pending} frames queued)"), crate::safety::FaultCode::BusLoss);
                }
                TickOutput::Expired { dropped, .. } => {
                    self.mit_lose(format!("MIT batch write expired ({dropped} frames dropped)"), crate::safety::FaultCode::BusLoss);
                }
                TickOutput::ActionDeadlineRefused { .. } | TickOutput::None => {}
            }
        }
        res
    }

    /// Cancel the group: no further output, latched fault, history and caches dropped.
    pub(crate) fn mit_lose(&mut self, reason: String, code: crate::safety::FaultCode) {
        tracing::error!("MIT tracking cancelled: {reason}");
        self.mit.owner = MitOwner::Lost;
        self.mit.lost_reason = Some(reason.clone());
        self.mit.last_output = None;
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            crate::rt_core::scratch::copy_into(&mut self.commanded_positions, &self.measured_positions);
            crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.measured_positions);
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.safety.latch(
            crate::safety::FaultKind::from_code(code),
            crate::safety::FaultDetail::formatted(code, |buf| {
                let _ = core::fmt::write(
                    buf,
                    format_args!(
                        "MIT_LOST: {reason}; if a motor's command timeout expired, clear_fault recovers by turning torque off on every joint (the arm drops like a park torque-off: support it)"
                    ),
                );
            }),
        );
    }

    /// Deadline supervision while MIT owns joints: the driver fence, every joint's feedback age
    /// and the gap since the last flushed dynamic batch, each against [`MIT_DEADLINE`].
    /// Time of the MIT deadline bookkeeping: the driver's clock (the transport's deterministic
    /// clock in tests, the system clock otherwise), the same one that dates the feedback ages.
    fn mit_now(&self) -> std::time::Instant {
        self.driver.as_ref().and_then(|d| d.lock().ok()).map_or_else(std::time::Instant::now, |drv| drv.clock_now())
    }

    /// Fault code of a cancelled group: BUS_LOSS when the driver's deadline fence tripped (a
    /// transport loss, wherever it was detected), otherwise a hardware fault (mode uncertain).
    fn mit_loss_code(&self) -> crate::safety::FaultCode {
        let tripped = self.driver.as_ref().and_then(|d| d.lock().ok()).is_some_and(|drv| drv.mit_fence_trip().is_some());
        if tripped { crate::safety::FaultCode::BusLoss } else { crate::safety::FaultCode::HardwareFault }
    }

    pub(crate) fn mit_supervise(&mut self) {
        if self.mit.owner == MitOwner::Lost {
            // a trip seen after the group was cancelled for another reason is still a loss
            let tripped = self.driver.as_ref().and_then(|d| d.lock().ok()).and_then(|drv| drv.mit_fence_trip());
            if let Some(t) = tripped {
                self.safety.latch(
                    crate::safety::FaultKind::BusLoss,
                    crate::safety::FaultDetail::formatted(crate::safety::FaultCode::BusLoss, |buf| {
                        let _ = core::fmt::write(
                            buf,
                            format_args!(
                                "MIT_LOST: driver deadline fence: {t}; if a motor's command timeout expired, clear_fault recovers by turning torque off on every joint (the arm drops like a park torque-off: support it)"
                            ),
                        );
                    }),
                );
            }
            return;
        }
        if self.mit.owner != MitOwner::Active { return; }
        let (trip, ages) = match self.driver.as_ref().and_then(|d| d.lock().ok()) {
            Some(drv) => (drv.mit_fence_trip(), drv.feedback_ages()),
            None => (None, Vec::new()),
        };
        let now = self.mit_now();
        let gap = self.mit.last_output.map(|t| now.saturating_duration_since(t));
        let reason = if let Some(t) = trip { Some(format!("driver deadline fence: {t}")) }
            else if let Some((i, a)) = ages.iter().enumerate().find(|(_, a)| a.is_none_or(past_deadline)) {
                Some(format!("joint {} feedback age {a:?} reached {MIT_DEADLINE:?}", i + 1))
            } else if ages.len() != self.config.dof { Some("feedback ages unavailable".into()) }
            else { gap.filter(|g| past_deadline(*g)).map(|g| format!("gap between dynamic outputs {g:?} reached {MIT_DEADLINE:?}")) };
        if let Some(r) = reason { self.mit_lose(r, crate::safety::FaultCode::BusLoss); return; }
        let worst = ages.iter().flatten().copied().chain(gap).max().unwrap_or_default();
        if worst >= MIT_NEAR_MISS {
            self.mit.near_misses += 1;
            if self.mit.near_misses.is_power_of_two() {
                tracing::warn!("MIT deadline near miss #{}: {worst:?} of {MIT_DEADLINE:?} (feedback age or output gap)", self.mit.near_misses);
            }
        }
    }

    /// Positions of a handover hold: owned joints at the measured position, the others (and the
    /// jaw) at their nominal command; MIT fields use the hold gains and `ff`, so either control
    /// mode interprets the frame as a stationary hold.
    fn mit_hold_targets(&self, joints: &[usize], ff: &[f32]) -> Vec<JointTarget> {
        (0..self.config.dof).map(|i| {
            if joints.contains(&i) {
                let c = self.mit_slot(i);
                JointTarget::mit(self.measured_positions[i], 0.0, c.and_then(|c| c.hold_kp).unwrap_or(0.0),
                    c.and_then(|c| c.hold_kd).unwrap_or(0.0), ff.get(i).copied().unwrap_or(0.0))
            } else {
                JointTarget { position: self.commanded_positions[i], velocity: Some(0.0), torque: Some(0.0), kp: None, kd: None }
            }
        }).collect()
    }

    /// Try the handover to MIT (each tick): only from fresh, stationary, confirmed position
    /// control of an armed arm whose qualification passed at this arm, with nothing else owning
    /// the output (no motion, park, gravity compensation, hold or fault).
    pub(crate) fn mit_maybe_activate(&mut self) {
        if mit_joint_mask(&self.config) == 0 || self.mit.owner != MitOwner::Off || !self.mit.armed_ok || !self.is_torqued
            || self.disable_pending || self.pending_rearm || self.pending_normal_unload || self.gravity_comp.is_some()
            || self.safety.state().is_stopped() || !self.safety.state().is_operational() || self.handover_settling_since.is_some()
            || self.active_trajectory.is_some() || self.servo_target.is_some() || self.servo_stream_active
            || self.collision.hold.is_some() || self.torque_guard.hold.is_some() || self.servo_blocked_hold
            || self.commanded_velocities.iter().any(|v| *v != 0.0)
            || !self.mit_feedback_fresh() || !self.tracking_stationary_evidence() {
            return;
        }
        let joints = mit_joints(&self.config);
        let g = match self.mit_gravity() {
            Ok(g) => g,
            Err(e) => { self.mit.handover_error = Some(e); self.mit.armed_ok = false; return; }
        };
        // the hold at the measured pose, zero velocity, hold gains; checked on the decoded fields
        let mut ff = vec![0.0; self.config.dof];
        let mut frames: Vec<(usize, JointTarget)> = Vec::new();
        for &j in &joints {
            match self.mit_frame(j, self.measured_positions[j], 0.0, 0.0, true, g[j], None) {
                Ok((frame, tau)) => { ff[j] = tau; frames.push((j, frame)); }
                Err(e) => {
                    self.mit.handover_error = Some(format!("J{}: the hold does not fit the effort limit ({e})", j + 1));
                    self.mit.armed_ok = false;
                    return;
                }
            }
        }
        let mut hold = self.mit_hold_targets(&joints, &ff);
        for (j, frame) in frames { hold[j] = frame; }
        let modes: Vec<crate::driver::JointMode> = (0..self.config.dof)
            .map(|i| if joints.contains(&i) { crate::driver::JointMode::Mit } else { crate::driver::JointMode::Position }).collect();
        let holdoff = self.mit_holdoff();
        let (res, uncertain) = {
            let Some(Ok(mut drv)) = self.driver.as_ref().map(|d| d.lock()) else { return };
            // armed before the first MIT-capable switch frame: a lost acknowledgement or a slow
            // confirmation cannot outlast the deadline, and a rollback after a trip sends nothing
            drv.set_mit_deadline_fence(Some(MIT_DEADLINE), holdoff);
            let res = drv.set_joint_modes_confirmed(&modes, &hold);
            let uncertain = drv.joint_modes_uncertain();
            if res.is_err() && !uncertain && drv.mit_fence_trip().is_none() {
                // switched back and confirmed in position control: nothing to fence
                drv.set_mit_deadline_fence(None, Duration::ZERO);
            }
            (res, uncertain || drv.mit_fence_trip().is_some())
        };
        match res {
            Ok(()) => {
                for &j in &joints {
                    self.commanded_positions[j] = hold[j].position;
                    self.target_positions[j] = hold[j].position;
                }
                self.tracking.seed(&self.commanded_positions);
                self.mit.owner = MitOwner::Active;
                self.mit.joints = joints.clone();
                self.mit.last_ff = ff;
                self.mit.last_output = Some(self.mit_now());
                self.mit.handover_error = None;
                tracing::warn!("MIT tracking owns joints {:?}", joints.iter().map(|j| j + 1).collect::<Vec<_>>());
            }
            Err(e) => {
                self.mit.armed_ok = false;
                self.mit.handover_error = Some(format!("MIT handover failed: {e}"));
                self.tracking.invalidate();
                if uncertain {
                    self.mit.joints = joints;
                    let code = self.mit_loss_code();
                    self.mit_lose(format!("MIT handover failed and the switch back is not confirmed: {e}"), code);
                }
            }
        }
    }

    /// Hand the MIT joints back to confirmed stationary position control (gravity compensation
    /// entry, park, rearm). Needs fresh stationary feedback. Nothing to do when MIT is not active.
    pub fn mit_hand_back(&mut self, why: &str) -> Result<(), String> {
        if self.mit.owner != MitOwner::Active { return Ok(()); }
        if !self.mit_feedback_fresh() || !self.tracking_stationary_evidence() || self.commanded_velocities.iter().any(|v| *v != 0.0) {
            return Err(format!("MIT tracking hands back ({why}) only at rest with fresh feedback"));
        }
        let joints = self.mit.joints.clone();
        let mut hold = self.mit_hold_targets(&joints, &self.mit.last_ff.clone());
        // the MIT side of the hold is what a rollback sends while the mode is uncertain: the
        // same decoded-field effort bound as every MIT frame, slewed from the last tau_ff; when it
        // cannot be met nothing is sent and the group is cancelled
        let g = match self.mit_gravity() {
            Ok(g) => g,
            Err(e) => { self.mit_lose(format!("MIT hand back ({why}): {e}"), crate::safety::FaultCode::BusLoss); return Err(e); }
        };
        let dt = 1.0 / self.config.control_frequency_hz.max(1) as f32;
        for &j in &joints {
            let last = self.mit.last_ff.get(j).copied().unwrap_or(0.0);
            match self.mit_frame(j, self.measured_positions[j], 0.0, 0.0, true, g[j], Some((last, dt))) {
                Ok((frame, _)) => hold[j] = frame,
                Err(e) => {
                    let msg = format!("MIT hand back ({why}) refused: the rollback hold of J{} does not fit the effort bound ({e})", j + 1);
                    self.mit_lose(msg.clone(), crate::safety::FaultCode::BusLoss);
                    return Err(msg);
                }
            }
        }
        let modes = vec![crate::driver::JointMode::Position; self.config.dof];
        let (res, uncertain) = {
            let Some(d) = self.driver.as_ref() else { return Err("no driver".into()) };
            let mut drv = d.lock().map_err(|_| "driver lock poisoned".to_string())?;
            let res = drv.set_joint_modes_confirmed(&modes, &hold);
            if res.is_ok() { drv.set_mit_deadline_fence(None, Duration::ZERO); }
            (res, drv.joint_modes_uncertain())
        };
        match res {
            Ok(()) => {
                for &j in &joints {
                    self.commanded_positions[j] = hold[j].position;
                    self.target_positions[j] = hold[j].position;
                }
                self.tracking.seed(&self.commanded_positions);
                self.mit.owner = MitOwner::Off;
                self.mit.joints.clear();
                self.mit.last_output = None;
                tracing::warn!("MIT tracking handed back to position control ({why})");
                Ok(())
            }
            Err(e) => {
                let msg = format!("MIT hand back ({why}) failed: {e}");
                if uncertain {
                    let code = self.mit_loss_code();
                    self.mit_lose(msg.clone(), code);
                } else {
                    // switched back to MIT with the hold: still the owner; the rollback hold is what the
                    // motors now execute, so its tau_ff is the reference of the next frame's slew
                    for &j in &joints {
                        if let (Some(t), Some(slot)) = (hold[j].torque, self.mit.last_ff.get_mut(j)) { *slot = t; }
                    }
                    self.mit.last_output = Some(self.mit_now());
                    self.safety.latch(
                        crate::safety::FaultKind::Other,
                        crate::safety::FaultDetail::formatted(crate::safety::FaultCode::HardwareFault, |buf| {
                            let _ = core::fmt::write(buf, format_args!("{msg}"));
                        }),
                    );
                }
                self.mit.handover_error = Some(msg.clone());
                Err(msg)
            }
        }
    }

    /// Torque goes off: MIT owns nothing; the motors keep CTRL_MODE MIT in RAM until the next
    /// enable restores the base mode. A cancelled group stays cancelled until cleared.
    pub(crate) fn mit_torque_off(&mut self) {
        self.mit.armed_ok = false;
        if self.mit.owner == MitOwner::Active {
            // The driver fence stays armed through the disable (no recovery replay, no invented
            // hold even when the disable fails); a confirmed disable or enable disarms it.
            self.mit.owner = MitOwner::Off;
            self.mit.joints.clear();
            self.mit.last_output = None;
        }
    }
}

/// Mark the MIT qualification record of `c` invalidated when it holds one of `joints` (1-based)
/// and is not invalidated yet; an unparseable record is marked as well. Returns true on change.
pub fn invalidate_record(c: &mut crate::calibration::CalibrationFile, joints: &[usize], reason: &str, at: &str) -> bool {
    let touches = match c.mit_record_raw_joints() {
        Some(js) => js.iter().any(|j| joints.contains(j)),
        None => true,
    };
    let Some(v) = c.mit_qualification.as_mut() else { return false };
    let Some(obj) = v.as_object_mut() else { return false };
    if !touches || obj.get("invalidated").is_some_and(|i| !i.is_null()) { return false; }
    obj.insert("invalidated".into(), json!({"reason": reason, "at": at}));
    true
}

#[cfg(not(target_arch = "wasm32"))]
impl crate::supervisor::SupervisorInner {
    /// clear_fault after an MIT loss when fresh feedback shows a motor in an error state (its
    /// command timeout expired: 0xD) or the motors partly enabled (operator, explicit and confirmed): clear-error to the faulted
    /// motors, a disable of every motor confirmed per motor, then Idle with torque off and the
    /// MIT group reset (fence disarmed, history cleared; the qualification record is untouched
    /// and the next arm reads the TIMEOUT again). A motor that does not confirm keeps the fault.
    pub(crate) fn mit_expiry_recovery_for_epoch(
        &mut self,
        expected_epoch: Option<u64>,
    ) -> Result<crate::safety::SafetyState, crate::safety::SafetyError> {
        use crate::driver::EnableState;
        use crate::safety::{ClearRefusal, FaultDetail, FaultKind, SafetyError};
        // Single authoritative clear_check FIRST before issuing any recovery disable/reset commands.
        let mut pre_ctx = self.build_mit_recovery_clear_context();
        pre_ctx.expected_epoch = expected_epoch;
        pre_ctx.pending_estop = self.estop_request_unapplied();
        if let Err(refusal) = self.safety.clear_check(&pre_ctx) {
            if refusal == ClearRefusal::EStopActive {
                self.consume_pending_estop();
                if let Some(received) = expected_epoch {
                    if received != self.safety.epoch_id() {
                        return Err(SafetyError::EpochMismatch {
                            expected: self.safety.epoch_id(),
                            received,
                        });
                    }
                }
            }
            return Err(refusal.into_safety_error());
        }
        let (pre_recovery_seqs, res) = match self.driver.as_ref().and_then(|d| d.lock().ok()) {
            Some(mut drv) => {
                let pre = drv.feedback_seqs();
                let r = drv.mit_expiry_recover();
                (pre, r)
            }
            None => (Vec::new(), Err("no driver".into())),
        };
        if let Err(e) = res {
            let msg = format!("MIT recovery after a motor timeout failed: {e}; the fault stays latched. \
                The recovery turns torque off on every joint (support the arm); check the named joints, then clear again");
            tracing::error!("{msg}");
            return Err(SafetyError::ClearFaultRejected(msg));
        }
        tracing::warn!("MIT recovery after a motor timeout: every motor confirmed disabled; torque is off on all joints");
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        self.is_torqued = false;
        self.disable_pending = false;
        self.pending_rearm = false;

        // Refresh supervisor feedback from samples received strictly AFTER the recovery request
        // (verified by per-joint receive sequence advance AND driver post-recovery RX boundary) —
        // never evaluate final `clear_check` on cached or pre-recovery feedback.
        let post_read = self.driver.as_ref().and_then(|d| {
            d.lock().ok().and_then(|mut drv| {
                let states = drv.read_joint_states().ok()?;
                let post_seqs = drv.feedback_seqs();
                let seqs_after_recovery = pre_recovery_seqs.len() == self.config.dof
                    && post_seqs.len() == self.config.dof
                    && post_seqs
                        .iter()
                        .zip(&pre_recovery_seqs)
                        .all(|(&post, &pre)| post > pre);
                let rx_boundary_ok = drv.mit_feedback_after_recovery(self.config.dof);
                let ages = drv.feedback_ages();
                let loads = drv.joint_loads();
                let caps = drv.capabilities();
                let fb_fresh = seqs_after_recovery
                    && rx_boundary_ok
                    && drv.is_feedback_valid()
                    && ages.len() == self.config.dof
                    && ages
                        .iter()
                        .all(|a| a.is_some_and(|t| t <= crate::supervisor::THERMAL_FEEDBACK_FRESHNESS_LIMIT))
                    && Self::states_valid(&states, caps, self.config.dof, loads.as_deref());
                let hw_err = drv.check_hardware_status().err().map(|e| e.to_clear_fault_error());
                let hw_fault = drv.has_hardware_fault();
                let state = drv.enable_state();
                let partial = drv.has_known_partial_enable();
                let enable_known = state != EnableState::Unknown || partial;
                let oldest_age = ages
                    .iter()
                    .copied()
                    .try_fold(std::time::Duration::ZERO, |m, a| a.map(|t| m.max(t)));
                let threshold = self.config.max_temperature - crate::supervisor::THERMAL_CLEAR_MARGIN_C;
                let thermal_below = fb_fresh
                    && caps.temperature_feedback
                    && !states.is_empty()
                    && states.iter().all(|s| s.temperature.is_finite() && s.temperature <= threshold);
                let mut rx_buf = Vec::new();
                let next_cd = self.evaluate_thermal_cooldown_with_driver(
                    drv.as_ref(),
                    &ages,
                    thermal_below,
                    &mut rx_buf,
                );
                Some((
                    states,
                    loads,
                    caps.temperature_feedback,
                    fb_fresh,
                    hw_err,
                    hw_fault,
                    enable_known,
                    oldest_age,
                    next_cd,
                ))
            })
        });
        let (post_fb_fresh, post_temp_fb, post_hw_err, post_hw_fault, post_enable_known) =
            if let Some((states, loads, temp_fb, fb_fresh, hw_err, hw_fault, enable_known, oldest_age, next_cd)) = post_read {
                self.feedback_numeric_valid = fb_fresh;
                if fb_fresh {
                    self.last_can_feedback_instant =
                        oldest_age.and_then(|age| std::time::Instant::now().checked_sub(age));
                    Self::apply_feedback(
                        &mut self.measured_positions,
                        &mut self.measured_velocities,
                        &mut self.torques,
                        &mut self.temperatures,
                        &mut self.loads,
                        self.config.dof,
                        &states,
                        loads.as_deref(),
                    );
                    if temp_fb {
                        self.thermal_cooldown = next_cd;
                        let temps_copy = self.temperatures.clone();
                        if let Some((joint_idx, temperature)) = temps_copy
                            .iter()
                            .copied()
                            .enumerate()
                            .filter(|(_, t)| t.is_finite() && *t > self.config.max_temperature)
                            .max_by(|a, b| a.1.total_cmp(&b.1))
                        {
                            self.thermal_cooldown.reset();
                            self.safety.latch(
                                FaultKind::ThermalTrip,
                                FaultDetail::ThermalTrip {
                                    joint_idx,
                                    temperature,
                                    limit: self.config.max_temperature,
                                },
                            );
                        }
                    } else {
                        self.thermal_cooldown.reset();
                    }
                } else {
                    self.last_can_feedback_instant = None;
                    self.thermal_cooldown.reset();
                }
                (fb_fresh, temp_fb, hw_err, hw_fault, enable_known)
            } else {
                self.feedback_numeric_valid = false;
                self.last_can_feedback_instant = None;
                self.thermal_cooldown.reset();
                (false, false, None, false, false)
            };

        if post_fb_fresh && self.measured_positions.iter().all(|q| q.is_finite()) {
            crate::rt_core::scratch::copy_into(&mut self.commanded_positions, &self.measured_positions);
            crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.measured_positions);
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);

        // An E-stop that arrived while the disables were confirmed wins: never end in Idle.
        self.consume_pending_estop();
        let mut post_ctx = self.build_mit_recovery_clear_context();
        post_ctx.expected_epoch = expected_epoch;
        post_ctx.pending_estop = self.estop_request_unapplied();
        post_ctx.feedback_fresh = post_fb_fresh;
        post_ctx.thermal_sensor_available = post_temp_fb;
        post_ctx.hardware_error = post_hw_err;
        post_ctx.hardware_reset_required = post_hw_fault;
        post_ctx.enable_state_known = post_enable_known;
        if let Some(received) = expected_epoch {
            if received != self.safety.epoch_id() {
                return Err(SafetyError::EpochMismatch {
                    expected: self.safety.epoch_id(),
                    received,
                });
            }
        }
        if self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst) {
            let _ = self.safety.clear_with_check(&post_ctx);
            self.apply_stop_policy(crate::safety::StopPolicy::EStopLatch);
            return Err(SafetyError::EStopActive);
        }
        if let Err(refusal) = self.safety.clear_with_check(&post_ctx) {
            if refusal == ClearRefusal::EStopActive {
                self.consume_pending_estop();
                if let Some(received) = expected_epoch {
                    if received != self.safety.epoch_id() {
                        return Err(SafetyError::EpochMismatch {
                            expected: self.safety.epoch_id(),
                            received,
                        });
                    }
                }
            }
            return Err(refusal.into_safety_error());
        }
        self.mit.owner = MitOwner::Off;
        self.mit.joints.clear();
        self.mit.last_ff.clear();
        self.mit.last_output = None;
        self.mit.armed_ok = false;
        self.mit.revoke_pending = false;
        if let Some(mut drv) = self.driver.as_ref().and_then(|d| d.lock().ok()) {
            drv.acknowledge_mit_fence_trip();
        }
        let _ = self.safety.set_torque(false);
        if self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst) {
            self.apply_stop_policy(crate::safety::StopPolicy::EStopLatch);
            return Err(SafetyError::EStopActive);
        }
        Ok(self.safety.state().clone())
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl crate::supervisor::SupervisorInner {
    /// `clear_fault` with the operator's explicit confirmation of the MIT expiry recovery
    /// (`{"mit_recovery": "torque_off_all"}`). Without it, the recovery state is refused with
    /// [`RECOVERY_CONFIRM_CODE`]; every other state clears as `clear_fault`.
    pub fn clear_fault_confirmed(&mut self, mit_recovery: bool) -> Result<crate::safety::SafetyState, crate::supervisor::ArmError> {
        self.clear_fault_confirmed_for_epoch(mit_recovery, None)
    }

    pub fn clear_fault_confirmed_for_epoch(
        &mut self,
        mit_recovery: bool,
        expected_epoch: Option<u64>,
    ) -> Result<crate::safety::SafetyState, crate::supervisor::ArmError> {
        self.mit.recovery_confirmed = mit_recovery;
        let res = self.clear_fault_for_epoch(expected_epoch);
        self.mit.recovery_confirmed = false;
        res.map_err(|e| match e {
            crate::safety::SafetyError::ClearFaultRejected(m) if m.starts_with(RECOVERY_CONFIRM_CODE) =>
                crate::supervisor::ArmError::Rejected { status: 409, code: RECOVERY_CONFIRM_CODE.into(), message: RECOVERY_WARNING.into() },
            crate::safety::SafetyError::ClearFaultRejected(m) if m.contains(QUIET_PERIOD_CODE) =>
                crate::supervisor::ArmError::Rejected { status: 409, code: QUIET_PERIOD_CODE.into(), message: m },
            other => crate::supervisor::ArmError::from(other),
        })
    }
}
