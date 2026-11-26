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
/// What the confirmation accepts: torque goes off on every joint (rest pose or already disabled only).
pub const RECOVERY_WARNING: &str = "A motor's command timeout expired after the MIT loss. Clearing recovers by turning torque off on every joint at the resting pose (or when every arm motor is already disabled): \
    support the arm or bring it to rest, then clear again with {\"mit_recovery\": \"torque_off_all\"}; away from the resting pose while arm motors hold torque, recovery is refused with TORQUE_OFF_WOULD_DROP";
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

/// Zero-allocation equivalent of `!mit_joints(cfg).is_empty()` for RT control paths.
pub fn has_mit_joints(cfg: &ArmConfig) -> bool {
    let Some(jt) = cfg.joint_tracking.as_ref().filter(|jt| jt.mit_requested()) else {
        return false;
    };
    let arm_len = cfg.arm_joints().len();
    jt.joints
        .iter()
        .enumerate()
        .any(|(k, j)| j.strategy == crate::tracking::TrackingStrategy::Mit && k < arm_len)
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

/// Parses `/api/v1/calibration/mit_timeout` requests for arm joints (J1–J6) only.
///
/// J7 (the gripper, `CTRL_MODE_POS_FORCE = 4` or legacy `POS_VEL`) is intentionally excluded:
/// it never enters MIT mode, its `PosForceBootFallback::NoOutputUntilFresh` recovery and
/// MIT-fence hold contracts rely on `TIMEOUT = 0` so a held workpiece is not dropped on host
/// restart or bus pause, and `RID_TIMEOUT` expiry in `POS_FORCE` is uncharacterized on B601-DM.
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
        let unit = json!({"unit_us": damiao::TIMEOUT_UNIT_US_DOCUMENTED, "unit_verified": false,
            "note": "50 us per count is documented by Damiao and not verified on this firmware; RAM only (0x55), lost at power-off"});
        #[cfg(not(target_arch = "wasm32"))]
        let shadow_rt = &mut self.torque_shadow;
        let d = self.driver.as_mut().expect("checked above");
        let res = d.with_mut(|driver| {
            if cmd.dry_run {
                let rb = driver.read_motor_identity(&cmd.joints);
                #[cfg(not(target_arch = "wasm32"))]
                shadow_rt.drain_from_driver(driver);
                let rb = rb.map_err(|e| CalibError::new(502, "MIT_TIMEOUT_READ", e.to_string()))?;
                return Ok(json!({"dry_run": true, "readback": rb, "timeout_unit": unit}));
            }
            let res = driver.commission_motor_timeout(&cmd.joints, cmd.timeout_counts);
            #[cfg(not(target_arch = "wasm32"))]
            shadow_rt.drain_from_driver(driver);
            let res = res.map_err(|e| {
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
        })
        .ok_or_else(|| CalibError::new(500, "DRIVER", "driver lock poisoned"))?;
        #[cfg(not(target_arch = "wasm32"))]
        if !self.rt_executor_owned {
            self.flush_torque_shadow();
        }
        res
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
            let d = self.driver.as_mut().expect("checked above");
            d.with_mut(|driver| {
                driver.read_motor_identity(&joints).map_err(|e| CalibError::new(502, "MIT_TIMEOUT_READ", e.to_string()))
            })
            .ok_or_else(|| CalibError::new(500, "DRIVER", "driver lock poisoned"))??
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
    pub fn mit_gate(&mut self) -> Result<(), String> {
        let joints = mit_joints(&self.config);
        if joints.is_empty() { return Ok(()); }
        // fail-closed: no MIT unless the RT executor runs at SCHED_FIFO with locked memory
        if let Some(why) = crate::rt_core::sched::mit_refusal() { return Err(why); }
        let Some(slot) = self.driver.as_mut() else { return Err("MIT needs a motor driver".into()) };
        let (supports, rb_res) = slot
            .with_mut(|driver| {
                (driver.supports_mit_mode_switch(), driver.read_motor_identity(&joints))
            })
            .ok_or_else(|| "driver lock poisoned".to_string())?;
        let rb = rb_res.map_err(|e| format!("register readback failed: {e}"))?;
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
    /// Why the group was cancelled (bounded stack text — zero allocation on `armd-control`).
    pub lost_reason: Option<crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>>,
    /// A park was requested while MIT moved the arm: it starts after rest and the hand back.
    pub park_after_hand_back: bool,
    /// When true, MIT handback transitions `park_continuation` to `ReadyForPlan` instead of
    /// planning synchronously inside `tick_control`.
    pub deferred_park_off_owner: bool,
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
        if self.park_continuation.is_some() { return Some("a park continuation is pending".into()); }
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

    /// Zero-allocation gravity torque per joint into stack buffer `out` (Rule R1).
    fn mit_gravity_into(
        &self,
        out: &mut [f32; crate::rt_core::mailbox::MAX_DOF],
    ) -> Result<(), &'static str> {
        out.fill(0.0);
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        let arm_dof = self.config.arm_dof().min(dof);
        let needed = (0..dof).any(|j| self.mit_slot(j).is_some_and(|c| c.gravity_scale > 0.0));
        if !needed {
            return Ok(());
        }
        if self.config.gravity_model.is_none() || self.measured_positions.len() < arm_dof {
            return Err("gravity model unavailable");
        }
        let tau = self
            .config
            .arm_model
            .gravity_torques(&self.measured_positions[..arm_dof])
            .map_err(|_| "gravity torque computation failed")?;
        if !tau[..arm_dof].iter().all(|t| t.is_finite()) {
            return Err("gravity torque not finite");
        }
        out[..arm_dof].copy_from_slice(&tau[..arm_dof]);
        Ok(())
    }

    /// Zero-allocation gravity torque per joint from the fresh measured posture, when a selected
    /// joint uses it.
    #[allow(clippy::result_large_err)] // inline stack text avoids RT heap allocation
    fn mit_gravity_inline(&self) -> Result<[f32; crate::rt_core::mailbox::MAX_DOF], OutputErrorText> {
        let arm_dof = self.config.arm_dof();
        let arm = self.config.arm_joints();
        let needed = self
            .mit
            .joints
            .iter()
            .copied()
            .any(|j| self.mit_slot(j).is_some_and(|c| c.gravity_scale > 0.0))
            || self
                .config
                .joint_tracking
                .as_ref()
                .filter(|jt| jt.mit_requested())
                .is_some_and(|jt| {
                    jt.joints
                        .iter()
                        .take(arm.len())
                        .any(|c| c.strategy == TrackingStrategy::Mit && c.gravity_scale > 0.0)
                });
        let mut g = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        if !needed {
            return Ok(g);
        }
        if self.config.gravity_model.is_none()
            || self.measured_positions.len() < arm_dof
            || arm_dof > crate::rt_core::mailbox::MAX_DOF
        {
            return Err(OutputErrorText::new("gravity model unavailable"));
        }
        let tau = self
            .config
            .arm_model
            .gravity_torques(&self.measured_positions[..arm_dof])
            .map_err(|e| {
                let mut buf = OutputErrorText::empty();
                let _ = core::fmt::write(&mut buf, format_args!("gravity: {e:?}"));
                buf
            })?;
        if !tau[..arm_dof].iter().all(|t| t.is_finite()) {
            return Err(OutputErrorText::new("gravity torque not finite"));
        }
        g[..arm_dof].copy_from_slice(&tau[..arm_dof]);
        Ok(g)
    }

    /// Gravity torque per joint from the fresh measured posture, when a selected joint uses it.
    fn mit_gravity(&self) -> Result<Vec<f32>, String> {
        let g = self.mit_gravity_inline().map_err(|e| e.to_string())?;
        Ok(g[..self.config.dof.min(g.len())].to_vec())
    }

    /// Fresh feedback of every joint: no age past the deadline ([`past_deadline`]).
    fn mit_feedback_fresh(&self) -> bool {
        if !self.feedback_numeric_valid { return false; }
        let Some(d) = self.driver.as_ref() else { return true };
        d.try_with_ref(|drv| {
            drv.feedback_all_within(self.config.dof, MIT_DEADLINE.saturating_sub(Duration::from_nanos(1)))
        })
        .unwrap_or(false)
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
    /// Zero-allocation RT core of [`Self::mit_prepare`].
    #[allow(clippy::result_large_err)] // inline stack text avoids RT heap allocation
    fn mit_prepare_inline(
        &self,
        targets: &mut [JointTarget],
        dt: f64,
    ) -> Result<([f32; crate::rt_core::mailbox::MAX_DOF], usize), OutputErrorText> {
        if !self.mit_feedback_fresh() {
            return Err(OutputErrorText::new("feedback older than the MIT deadline"));
        }
        let g = self.mit_gravity_inline()?;
        let dof = self.config.dof;
        if dof > crate::rt_core::mailbox::MAX_DOF {
            return Err(OutputErrorText::new("dof exceeds MAX_DOF"));
        }
        let dt = (dt as f32).clamp(0.001, 0.1);
        let mut ff = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let copy_len = self.mit.last_ff.len().min(dof);
        ff[..copy_len].copy_from_slice(&self.mit.last_ff[..copy_len]);
        for &j in &self.mit.joints {
            let t = targets
                .get_mut(j)
                .ok_or_else(|| OutputErrorText::new("target vector too short"))?;
            let (q_d, v_d) = (t.position, t.velocity.unwrap_or(0.0));
            let a_d = self.commanded_accelerations.get(j).copied().unwrap_or(0.0);
            let hold = self.mit_in_hold(v_d, a_d);
            let last = self.mit.last_ff.get(j).copied().unwrap_or(0.0);
            let (frame, tau) = self.mit_frame_bounded(j, q_d, v_d, a_d, hold, g[j], Some((last, dt)))?;
            *t = frame;
            ff[j] = tau;
        }
        Ok((ff, dof))
    }

    /// Zero-allocation per-joint masked MIT hold preparation for stopped / `FailedHold` states
    /// (Rule R1): reads feedback ages from the owner's inline `owner_driver_snapshot` (no driver
    /// mutex lock and no feedback-age `Vec`), writes feed-forward torques into caller-owned
    /// stack array `ff_out` (no `last_ff.clone()`), and validates each joint in `hold_mask`
    /// independently. Returns `(filtered_mask, any_mit_prepared)`.
    pub fn mit_prepare_masked_into(
        &self,
        hold_mask: u64,
        targets: &mut [JointTarget],
        dt: f64,
        ff_out: &mut [f32; crate::rt_core::mailbox::MAX_DOF],
    ) -> (u64, bool) {
        let mut eff_mask = hold_mask;
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        ff_out.fill(0.0);
        let seed_n = self.mit.last_ff.len().min(dof);
        ff_out[..seed_n].copy_from_slice(&self.mit.last_ff[..seed_n]);
        let mut g_buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let g_res = self.mit_gravity_into(&mut g_buf);
        let dt = (dt as f32).clamp(0.001, 0.1);
        let has_driver = self.driver.is_some();
        let snap = self.owner_driver_snapshot;
        let mut any_mit_prepared = false;
        for &j in &self.mit.joints {
            if j >= crate::rt_core::mailbox::MAX_DOF || (eff_mask & (1u64 << j)) == 0 {
                continue;
            }
            let age_ok = if has_driver {
                snap.valid && snap.feedback_ages[j].is_some_and(|age| !past_deadline(age))
            } else {
                true
            };
            if !age_ok {
                eff_mask &= !(1u64 << j);
                continue;
            }
            let g_j = match (&g_res, self.mit_slot(j)) {
                (Ok(()), _) => g_buf[j],
                (Err(_), Some(c)) if c.gravity_scale <= 0.0 => 0.0,
                (Err(_), _) => {
                    eff_mask &= !(1u64 << j);
                    continue;
                }
            };
            let Some(t) = targets.get_mut(j) else {
                eff_mask &= !(1u64 << j);
                continue;
            };
            let (q_d, v_d) = (t.position, t.velocity.unwrap_or(0.0));
            let a_d = self.commanded_accelerations.get(j).copied().unwrap_or(0.0);
            let last = self.mit.last_ff.get(j).copied().unwrap_or(0.0);
            match self.mit_frame_core(j, q_d, v_d, a_d, true, g_j, Some((last, dt))) {
                Ok((frame, tau)) => {
                    *t = frame;
                    ff_out[j] = tau;
                    any_mit_prepared = true;
                }
                Err(_) => {
                    eff_mask &= !(1u64 << j);
                }
            }
        }
        (eff_mask, any_mit_prepared)
    }

    /// Per-joint masked MIT hold preparation for stopped / `FailedHold` states: validates each
    /// joint in `hold_mask` independently so an excluded or stale joint never vetoes peer joints.
    /// Returns `(filtered_mask, Option<ff>)` where failing MIT joints are cleared from `hold_mask`.
    pub fn mit_prepare_masked(
        &self,
        hold_mask: u64,
        targets: &mut [JointTarget],
        dt: f64,
    ) -> (u64, Option<Vec<f32>>) {
        let mut ff_buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let (eff_mask, any_mit) = self.mit_prepare_masked_into(hold_mask, targets, dt, &mut ff_buf);
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        (eff_mask, any_mit.then(|| ff_buf[..dof].to_vec()))
    }

    pub fn mit_prepare(&self, targets: &mut [JointTarget], dt: f64) -> Result<Vec<f32>, String> {
        let (ff, dof) = self.mit_prepare_inline(targets, dt).map_err(|e| e.to_string())?;
        Ok(ff[..dof].to_vec())
    }

    /// Zero-allocation MIT frame calculation used by stopped holds (Rule R1).
    #[allow(clippy::too_many_arguments)]
    fn mit_frame_core(
        &self,
        j: usize,
        q_d: f32,
        v_d: f32,
        a_d: f32,
        hold: bool,
        g_j: f32,
        slew: Option<(f32, f32)>,
    ) -> Result<(JointTarget, f32), &'static str> {
        let c = self.mit_slot(j).ok_or("no tracking entry")?;
        let (q_m, v_m) = (self.measured_positions[j], self.measured_velocities[j]);
        if ![q_d, v_d, a_d, q_m, v_m].iter().all(|x| x.is_finite()) {
            return Err("non-finite reference or feedback");
        }
        let (kp, kd) = if hold {
            (c.hold_kp.unwrap_or(0.0), c.hold_kd.unwrap_or(0.0))
        } else {
            (c.kp, c.kd)
        };
        let wl = wire_limits(&self.config, j);
        let off = self.joint_zero_offsets.get(j).copied().unwrap_or(0.0);
        if (q_d + off).abs() > wl.p_max
            || v_d.abs() > wl.v_max
            || kp > wl.kp_max
            || kd > wl.kd_max
            || kp <= 0.0
            || kd <= 0.0
        {
            return Err("MIT frame outside wire ranges");
        }
        let grid = |x: f32, lo: f32, hi: f32, bits: u32| {
            damiao::uint_to_float(damiao::float_to_uint(x, lo, hi, bits), lo, hi, bits)
        };
        let q_w = grid(q_d + off, wl.p_min, wl.p_max, 16) - off;
        let v_w = grid(v_d, wl.v_min, wl.v_max, 12);
        let kp_w = grid(kp, wl.kp_min, wl.kp_max, 12);
        let kd_w = grid(kd, wl.kd_min, wl.kd_max, 12);
        let pd = kp_w * (q_w - q_m) + kd_w * (v_w - v_m);
        let dynamic = if hold {
            0.0
        } else {
            c.coulomb_nm * (v_d / MIT_COULOMB_VEL_RAD_S).clamp(-1.0, 1.0)
                + c.viscous_nm_s_rad * v_d
                + c.inertia_kg_m2 * a_d
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
            if h.sign > 0.0 {
                hi = hi.min(-pd);
            } else {
                lo = lo.max(-pd);
            }
        }
        if lo.is_nan() || hi.is_nan() || lo > hi {
            return Err("empty effort interval");
        }
        let tau = quantize_into(raw.clamp(lo, hi), lo, hi, &wl)
            .ok_or("no wire torque value inside interval")?;
        let total = pd + tau;
        if total.abs() > l + 1e-6 || guard.is_some_and(|h| h.sign * total > 1e-6) {
            return Err("decoded effort outside limit or relieving side");
        }
        Ok((JointTarget::mit(q_w, v_w, kp_w, kd_w, tau), tau))
    }

    /// Zero-allocation RT core of [`Self::mit_frame`].
    #[allow(clippy::too_many_arguments, clippy::result_large_err)] // inline stack text avoids RT heap allocation
    fn mit_frame_bounded(
        &self,
        j: usize,
        q_d: f32,
        v_d: f32,
        a_d: f32,
        hold: bool,
        g_j: f32,
        slew: Option<(f32, f32)>,
    ) -> Result<(JointTarget, f32), OutputErrorText> {
        let fmt_err = |args: core::fmt::Arguments<'_>| {
            let mut buf = OutputErrorText::empty();
            let _ = core::fmt::write(&mut buf, args);
            buf
        };
        let c = self
            .mit_slot(j)
            .ok_or_else(|| fmt_err(format_args!("J{}: no tracking entry", j + 1)))?;
        let (q_m, v_m) = (self.measured_positions[j], self.measured_velocities[j]);
        if ![q_d, v_d, a_d, q_m, v_m].iter().all(|x| x.is_finite()) {
            return Err(fmt_err(format_args!("J{}: non-finite reference or feedback", j + 1)));
        }
        let (kp, kd) = if hold { (c.hold_kp.unwrap_or(0.0), c.hold_kd.unwrap_or(0.0)) } else { (c.kp, c.kd) };
        let wl = wire_limits(&self.config, j);
        let off = self.joint_zero_offsets.get(j).copied().unwrap_or(0.0);
        if (q_d + off).abs() > wl.p_max || v_d.abs() > wl.v_max || kp > wl.kp_max || kd > wl.kd_max || kp <= 0.0 || kd <= 0.0 {
            return Err(fmt_err(format_args!(
                "J{}: MIT frame outside the wire ranges (q {:.3}, v {v_d:.3}, kp {kp}, kd {kd})",
                j + 1,
                q_d + off
            )));
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
            return Err(fmt_err(format_args!(
                "J{}: empty effort interval [{lo:.3}, {hi:.3}] N·m (pd {pd:.3}, L {l})",
                j + 1
            )));
        }
        let tau = quantize_into(raw.clamp(lo, hi), lo, hi, &wl).ok_or_else(|| {
            fmt_err(format_args!("J{}: no wire torque value inside [{lo:.4}, {hi:.4}]", j + 1))
        })?;
        let total = pd + tau;
        if total.abs() > l + 1e-6 || guard.is_some_and(|h| h.sign * total > 1e-6) {
            return Err(fmt_err(format_args!(
                "J{}: decoded effort {total:.4} N·m outside the limit {l} or the relieving side",
                j + 1
            )));
        }
        Ok((JointTarget::mit(q_w, v_w, kp_w, kd_w, tau), tau))
    }

    /// One MIT frame for joint `j` on the decoded wire grid and its tau_ff (see
    /// [`Self::mit_prepare`]); `slew` = (last tau_ff, dt) limits the change, None (the handover's
    /// first frame) does not. Every bound is checked on the fields the motor decodes.
    #[allow(clippy::too_many_arguments)]
    fn mit_frame(&self, j: usize, q_d: f32, v_d: f32, a_d: f32, hold: bool, g_j: f32, slew: Option<(f32, f32)>)
        -> Result<(JointTarget, f32), String> {
        self.mit_frame_bounded(j, q_d, v_d, a_d, hold, g_j, slew).map_err(|e| e.to_string())
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
    pub(crate) fn active_joint_velocity_ceiling(&self, g: usize) -> Option<f32> {
        #[cfg(feature = "policy-runtime")]
        let policy_scale = self
            .policy_run
            .as_ref()
            .filter(|r| r.mode == crate::policy_runtime::PolicyMode::Live)
            .and_then(crate::policy_runtime::PolicyRun::release_gate_max_speed_scale)
            .map(|s| (s as f32).clamp(0.1, 1.0));
        #[cfg(not(feature = "policy-runtime"))]
        let policy_scale: Option<f32> = None;

        let base = if let Some(active) = &self.active_trajectory {
            if let crate::motion::Reference::Gripper(gm) = &active.reference {
                if gm.gripper_index == g {
                    return Some(gm.wire_velocity_ceiling());
                }
            }
            active
                .motion_limits
                .as_ref()
                .and_then(|ls| ls.get(g).map(|l| l.max_velocity))
                .or_else(|| active.motion_vel_caps.and_then(|(caps, dof)| (g < dof).then_some(caps[g])))
        } else if let Some(ramp) = &self.tracking.fallback_ramp {
            ramp.limits.get(g).map(|l| l.max_velocity)
        } else if let Some(lim) = self.tracking.retained_motion_limits.as_ref().and_then(|ls| ls.get(g)) {
            Some(lim.max_velocity)
        } else if let Some((caps, dof)) = self.tracking.retained_motion_vel_caps {
            (g < dof).then_some(caps[g])
        } else if self.servo_target.is_some() {
            let mut v = self.config.limits.get(g)?.max_velocity;
            if let Some(scale) = policy_scale {
                v *= scale;
            }
            Some(v)
        } else {
            None
        };
        let mut ceiling = base.or_else(|| self.config.limits.get(g).map(|l| l.max_velocity))?;
        if self.active_trajectory.is_none() && self.tracking.fallback_ramp.is_none() {
            if let Some(scale) = policy_scale {
                if let Some(prof_v) = self.config.limits.get(g).map(|l| l.max_velocity * scale) {
                    ceiling = ceiling.min(prof_v);
                }
            }
        }
        Some(self.gripper_hold_vel_ceiling.map_or(ceiling, |hold_v| ceiling.min(hold_v)))
    }

    pub(crate) fn write_output_slice(
        &mut self,
        targets: &mut [JointTarget],
        dt: f64,
        override_limits: Option<&[crate::model::JointLimit]>,
    ) -> crate::supervisor::TickOutput {
        use crate::driver::PosForceIntent;
        use crate::supervisor::TickOutput;
        // an E-stop or authorized stop requested since the tick applied the last one, or a
        // non-blocking disconnect lease fence / pending disconnect hold: no dynamic output
        // reaches the bus (the tick applies stop/fence policy right after, see `tick_control`).
        if self.urgent_stop_unapplied() || self.disconnect_fence_blocks_output() {
            return TickOutput::None;
        }
        let is_hold = self.safety.state().is_stopped();
        let non_target_stamp = self.current_j7_intent_stamp();
        // Capture the client lease generation BEFORE `post_stop_check_hook` and BEFORE locking
        // the driver so even a disconnect arriving in that exact window is caught at the driver's
        // single TX choke point (`DamiaoArmDriver::send`). Autonomous safety motions
        // (`watchdog_park`, `pending_normal_unload`) and stopped holds carry `0` (lease-exempt).
        // Live policy output NEVER downgrades to lease-exempt `0` on expiry: if the lease
        // expired between the top-of-tick fence and TX staging, fail closed and hold J7.
        let output_lease_gen = if !is_hold && self.watchdog_park.is_none() && !self.pending_normal_unload {
            #[cfg(feature = "policy-runtime")]
            if let Some(run) = self
                .policy_run
                .as_ref()
                .filter(|r| r.mode == crate::policy_runtime::PolicyMode::Live)
            {
                if self.lease_enabled {
                    let active_gen = self.current_active_lease_gen();
                    if run.lease_generation == 0 || active_gen != run.lease_generation {
                        self.j7_intent = Some(PosForceIntent::hold(non_target_stamp));
                        return TickOutput::None;
                    }
                    run.lease_generation
                } else {
                    self.current_active_lease_gen()
                }
            } else {
                self.current_active_lease_gen()
            }
            #[cfg(not(feature = "policy-runtime"))]
            {
                self.current_active_lease_gen()
            }
        } else {
            0
        };
        if let Some(ref hook) = self.post_stop_check_hook {
            hook();
        }
        // Every J7 `Target` carries the `(stop_gen, estop_epoch)` from the A1 `AdmissionStamp` of
        // the command that produced it (`self.j7_admitted_stamp`). It is NEVER re-stamped later
        // and NEVER stamped with `max(local, requested)`, so any STOP accepted after admission —
        // even right between the `urgent_stop_unapplied()` check above and intent creation — turns
        // the intent into `Hold` in the tick epilogue's single J7 emitter.
        let admitted_target_stamp = self.j7_admitted_stamp;
        let ff = match self.mit.owner {
            MitOwner::Off => None,
            MitOwner::Lost => {
                // A tripped MIT fence / cancelled MIT group suppresses ONLY the MIT joints; set
                // the J7 intent so the tick epilogue's ONE J7 emitter writes the hold/freeze frame.
                self.j7_intent = Some(if is_hold {
                    PosForceIntent::stop(non_target_stamp)
                } else {
                    PosForceIntent::hold(non_target_stamp)
                });
                return TickOutput::None;
            }
            MitOwner::Active => match self.mit_prepare_inline(targets, dt) {
                Ok(ff) => Some(ff),
                Err(e) => {
                    // Record J7 Hold/Stop intent so the tick epilogue's ONE J7 emitter still writes
                    // J7's freeze/hold frame while MIT joints remain suppressed.
                    self.j7_intent = Some(if is_hold {
                        PosForceIntent::stop(non_target_stamp)
                    } else {
                        PosForceIntent::hold(non_target_stamp)
                    });
                    let (now, max_age) = self
                        .driver
                        .as_ref()
                        .and_then(|d| d.try_with_ref(|drv| (drv.clock_now(), drv.tx_frame_max_age())).ok())
                        .unwrap_or_else(|| (self.mit_now(), crate::driver::DAMIAO_TX_FRAME_MAX_AGE));
                    let mut buf = OutputErrorText::empty();
                    let _ = core::fmt::write(&mut buf, format_args!("MIT output refused: {e}"));
                    return TickOutput::Error {
                        message: buf,
                        now,
                        max_age,
                        is_hold,
                        sent_any: false,
                        due_mask: 0,
                        sent_mask: 0,
                        pf_status: crate::driver::PosForceTxStatus::Unobserved,
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
        let mut computed_j7_intent: Option<PosForceIntent> = None;
        let cutoff = self.active_tx_cutoff;
        #[cfg(not(target_arch = "wasm32"))]
        let shadow_rt = &mut self.torque_shadow;
        let res = self.driver.as_mut().and_then(|d| {
            d.with_mut(|drv| {
                // Resolve the J7 PosForce intent (never writing a 0x307 frame here: the tick
                // epilogue's ONE J7 emitter writes every 0x307 frame after arm output finishes).
                // - In stopped hold (`is_hold`), caller's J7 target is ignored and intent is `Stop`.
                // - In motion (`!is_hold`), enforce the active authorized velocity ceiling and
                //   validate the caller's J7 target BEFORE writing arm frames: an invalid caller
                //   J7 target (non-finite or unrepresentable sub-quantum speed) refuses arm motion
                //   and degrades J7 intent to `Stop` so the epilogue still emits a hold/freeze.
                if let Some(cap) = drv.gripper_pos_force_info().filter(|c| c.active) {
                    if is_hold {
                        computed_j7_intent = Some(PosForceIntent::stop(non_target_stamp));
                    } else if let Some(t) = targets.get_mut(cap.joint) {
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
                        if let Err(err) = drv.validate_pos_force_intent_target(t.position, t.velocity, None) {
                            computed_j7_intent = Some(PosForceIntent::stop(non_target_stamp));
                            return crate::supervisor::SupervisorInner::tick_output_from_driver_write(
                                drv,
                                false,
                                Err(err),
                            );
                        }
                        computed_j7_intent = Some(match admitted_target_stamp {
                            Some(stamp) => PosForceIntent::target(t.position, t.velocity, None, stamp),
                            None => PosForceIntent::hold(non_target_stamp),
                        });
                    }
                }
                let j7_hold_hint = is_hold
                    || computed_j7_intent.is_some_and(|intent| {
                        !matches!(intent.kind, crate::driver::PosForceIntentKind::Target { .. })
                            || intent.stamp.is_stale_against(non_target_stamp)
                    });
                drv.set_pending_pos_force_hold_hint(j7_hold_hint);
                // Single TX cutoff choke point at the driver boundary:
                // - Safety/stopped holds restore the unclipped release cap so an action deadline
                //   never blocks a safety hold.
                // - Motion targets check the action cutoff right before calling the driver (guarding
                //   non-CAN/sim/mock drivers and any legacy lock wait), while `DamiaoArmDriver`
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
                        return TickOutput::ActionDeadlineRefused {
                            drv_now: pre_now,
                            pf_status: crate::driver::PosForceTxStatus::Unobserved,
                        };
                    }
                }
                let write_res = if is_hold {
                    drv.set_output_lease_gen(0);
                    drv.write_estop_hold_bounded(targets)
                } else {
                    drv.set_output_lease_gen(output_lease_gen);
                    let r = drv.write_joint_targets_bounded(targets);
                    drv.set_output_lease_gen(0);
                    r
                };
                let now = drv.clock_now();
                #[cfg(not(target_arch = "wasm32"))]
                shadow_rt.drain_from_driver(drv);
                if !is_hold
                    && matches!(
                        write_res,
                        Ok(crate::driver::TxBatchOutcome::Deferred { .. })
                            | Err(crate::driver::ArmDriverError::Transport(
                                crate::transport::TransportError::Timeout
                            ))
                    )
                    && action_cap.is_some_and(|cap| now >= cap)
                {
                    drv.discard_pending_tx();
                    return TickOutput::ActionDeadlineRefused {
                        drv_now: now,
                        pf_status: drv.take_last_pos_force_tx_status(),
                    };
                }
                crate::supervisor::SupervisorInner::tick_output_from_driver_write(drv, is_hold, write_res)
            })
        })
        .unwrap_or(TickOutput::None);
        if let Some(intent) = computed_j7_intent {
            self.j7_intent = Some(intent);
        }
        if let Some((ff, dof)) = ff {
            match &res {
                TickOutput::Complete { .. } => {
                    crate::rt_core::scratch::copy_into(&mut self.mit.last_ff, &ff[..dof]);
                    self.mit.last_output = Some(self.mit_now());
                }
                TickOutput::EstopInhibited { .. } => {
                    self.j7_intent = Some(PosForceIntent::stop(self.current_j7_intent_stamp()));
                    let mut reason = OutputErrorText::empty();
                    let _ = core::fmt::write(
                        &mut reason,
                        format_args!("MIT batch write failed: {}", crate::driver::ESTOP_INHIBITED),
                    );
                    self.mit_lose_bounded(reason, crate::safety::FaultCode::BusLoss);
                }
                TickOutput::LeaseFenced => {
                    self.j7_intent = Some(PosForceIntent::hold(self.current_j7_intent_stamp()));
                }
                TickOutput::Error { message, .. } => {
                    self.j7_intent = Some(PosForceIntent::stop(self.current_j7_intent_stamp()));
                    let mut reason = OutputErrorText::empty();
                    let _ = core::fmt::write(&mut reason, format_args!("MIT batch write failed: {message}"));
                    self.mit_lose_bounded(reason, crate::safety::FaultCode::BusLoss);
                }
                TickOutput::Deferred { pending, .. } => {
                    self.j7_intent = Some(PosForceIntent::stop(self.current_j7_intent_stamp()));
                    let mut reason = OutputErrorText::empty();
                    let _ = core::fmt::write(
                        &mut reason,
                        format_args!("MIT batch write deferred ({pending} frames queued)"),
                    );
                    self.mit_lose_bounded(reason, crate::safety::FaultCode::BusLoss);
                }
                TickOutput::Expired { dropped, .. } => {
                    self.j7_intent = Some(PosForceIntent::stop(self.current_j7_intent_stamp()));
                    let mut reason = OutputErrorText::empty();
                    let _ = core::fmt::write(
                        &mut reason,
                        format_args!("MIT batch write expired ({dropped} frames dropped)"),
                    );
                    self.mit_lose_bounded(reason, crate::safety::FaultCode::BusLoss);
                }
                TickOutput::ActionDeadlineRefused { .. } | TickOutput::None => {}
            }
        }
        res
    }

    /// Cancel the group without heap allocation on `armd-control`: no further output, latched
    /// fault, history and caches dropped, and deferred log event when RT deferred logging is on.
    pub(crate) fn mit_lose_bounded(
        &mut self,
        reason: OutputErrorText,
        code: crate::safety::FaultCode,
    ) {
        self.safety.log_mit_tracking_cancelled(reason);
        #[cfg(not(target_arch = "wasm32"))]
        if !self.mit.joints.is_empty() {
            let mut mask = crate::torque::ChannelMask::EMPTY;
            for &j in &self.mit.joints {
                if j < crate::torque::MAX_CHANNELS {
                    mask.insert(j as u8);
                }
            }
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::MitOwnershipChange {
                    channels: mask,
                    target_mode: crate::motor_mode::ControlMode::PositionVelocity,
                    device_wide: true,
                },
            );
        }
        self.mit.owner = MitOwner::Lost;
        self.mit.lost_reason = Some(reason);
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

    /// Cancel the group: no further output, latched fault, history and caches dropped.
    pub(crate) fn mit_lose(&mut self, reason: String, code: crate::safety::FaultCode) {
        self.mit_lose_bounded(OutputErrorText::new(&reason), code);
    }

    /// Deadline supervision while MIT owns joints: the driver fence, every joint's feedback age
    /// and the gap since the last flushed dynamic batch, each against [`MIT_DEADLINE`].
    /// Time of the MIT deadline bookkeeping: the driver's clock (the transport's deterministic
    /// clock in tests, the system clock otherwise), the same one that dates the feedback ages.
    pub(crate) fn mit_now(&self) -> std::time::Instant {
        self.driver
            .as_ref()
            .and_then(|d| d.try_with_ref(|drv| drv.clock_now()).ok())
            .unwrap_or_else(std::time::Instant::now)
    }

    /// Fault code of a cancelled group: BUS_LOSS when the driver's deadline fence tripped (a
    /// transport loss, wherever it was detected), otherwise a hardware fault (mode uncertain).
    fn mit_loss_code(&self) -> crate::safety::FaultCode {
        let tripped = self
            .driver
            .as_ref()
            .and_then(|d| d.try_with_ref(|drv| drv.mit_fence_trip_bounded().is_some()).ok())
            .unwrap_or(false);
        if tripped { crate::safety::FaultCode::BusLoss } else { crate::safety::FaultCode::HardwareFault }
    }

    pub(crate) fn mit_supervise(&mut self) {
        if self.mit.owner == MitOwner::Lost {
            // a trip seen after the group was cancelled for another reason is still a loss
            let tripped = self
                .driver
                .as_ref()
                .and_then(|d| d.try_with_ref(|drv| drv.mit_fence_trip_bounded()).ok())
                .flatten();
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
        let mut ages = std::mem::take(&mut self.scratch.ages);
        ages.clear();
        let (trip, now) = match self.driver.as_ref() {
            Some(d) => d
                .try_with_ref(|drv| {
                    drv.feedback_ages_into(&mut ages);
                    (drv.mit_fence_trip_bounded(), drv.clock_now())
                })
                .unwrap_or_else(|_| (None, std::time::Instant::now())),
            None => (None, std::time::Instant::now()),
        };
        let gap = self.mit.last_output.map(|t| now.saturating_duration_since(t));
        let mut reason: Option<OutputErrorText> = None;
        if let Some(t) = trip {
            let mut buf = OutputErrorText::empty();
            let _ = core::fmt::write(&mut buf, format_args!("driver deadline fence: {t}"));
            reason = Some(buf);
        } else if let Some((i, a)) = ages.iter().enumerate().find(|(_, a)| a.is_none_or(past_deadline)) {
            let mut buf = OutputErrorText::empty();
            let _ = core::fmt::write(
                &mut buf,
                format_args!("joint {} feedback age {a:?} reached {MIT_DEADLINE:?}", i + 1),
            );
            reason = Some(buf);
        } else if ages.len() != self.config.dof {
            reason = Some(OutputErrorText::new("feedback ages unavailable"));
        } else if let Some(g) = gap.filter(|g| past_deadline(*g)) {
            let mut buf = OutputErrorText::empty();
            let _ = core::fmt::write(
                &mut buf,
                format_args!("gap between dynamic outputs {g:?} reached {MIT_DEADLINE:?}"),
            );
            reason = Some(buf);
        }
        let worst = ages.iter().flatten().copied().chain(gap).max().unwrap_or_default();
        self.scratch.ages = ages;
        if let Some(r) = reason {
            self.mit_lose_bounded(r, crate::safety::FaultCode::BusLoss);
            return;
        }
        if worst >= MIT_NEAR_MISS {
            self.mit.near_misses += 1;
            if self.mit.near_misses.is_power_of_two() {
                self.safety.log_mit_deadline_near_miss(self.mit.near_misses, worst, MIT_DEADLINE);
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
            || self.disable_pending || self.pending_rearm || self.pending_normal_unload
            || self.park_continuation.is_some() || self.gravity_comp.is_some()
            || self.safety.state().is_stopped() || !self.safety.state().is_operational() || self.handover_settling_since.is_some()
            || self.active_trajectory.is_some() || self.servo_target.is_some() || self.servo_stream_active
            || self.collision.hold.is_some() || self.torque_guard.hold.is_some() || self.servo_blocked_hold
            || self.commanded_velocities.iter().any(|v| *v != 0.0)
            || self.exclusive_session_owns_output() {
            return;
        }
        let joints = mit_joints(&self.config);
        if joints.is_empty() || !self.mit_feedback_fresh() || !self.tracking_stationary_evidence() {
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
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let Some(out) = self.driver.as_mut().and_then(|d| {
                d.with_mut(|drv| {
                    // armed before the first MIT-capable switch frame: a lost acknowledgement or a slow
                    // confirmation cannot outlast the deadline, and a rollback after a trip sends nothing
                    drv.set_mit_deadline_fence(Some(MIT_DEADLINE), holdoff);
                    let res = drv.set_joint_modes_confirmed(&modes, &hold);
                    let uncertain = drv.joint_modes_uncertain();
                    if res.is_err() && !uncertain && drv.mit_fence_trip().is_none() {
                        // switched back and confirmed in position control: nothing to fence
                        drv.set_mit_deadline_fence(None, Duration::ZERO);
                    }
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    (res, uncertain || drv.mit_fence_trip().is_some())
                })
            }) else { return };
            out
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
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let mut mask = crate::torque::ChannelMask::EMPTY;
                    for &j in &joints {
                        if j < crate::torque::MAX_CHANNELS {
                            mask.insert(j as u8);
                        }
                    }
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::MitOwnershipChange {
                            channels: mask,
                            target_mode: crate::motor_mode::ControlMode::Impedance,
                            device_wide: false,
                        },
                    );
                }
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
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let Some(d) = self.driver.as_mut() else { return Err("no driver".into()) };
            d.with_mut(|drv| {
                let res = drv.set_joint_modes_confirmed(&modes, &hold);
                if res.is_ok() { drv.set_mit_deadline_fence(None, Duration::ZERO); }
                #[cfg(not(target_arch = "wasm32"))]
                shadow_rt.drain_from_driver(drv);
                (res, drv.joint_modes_uncertain())
            })
            .ok_or_else(|| "driver lock poisoned".to_string())?
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
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let mut mask = crate::torque::ChannelMask::EMPTY;
                    for &j in &joints {
                        if j < crate::torque::MAX_CHANNELS {
                            mask.insert(j as u8);
                        }
                    }
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::MitOwnershipChange {
                            channels: mask,
                            target_mode: crate::motor_mode::ControlMode::PositionVelocity,
                            device_wide: false,
                        },
                    );
                }
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
        self.mit_torque_off_inner(true);
    }

    /// Same as [`Self::mit_torque_off`], optionally deferring shadow emission (zero shadow clock
    /// sampling inside timed panic-disable closures before the live outcome is sealed).
    pub(crate) fn mit_torque_off_inner(&mut self, emit_shadow: bool) {
        self.mit.armed_ok = false;
        if self.mit.owner == MitOwner::Active {
            // The driver fence stays armed through the disable (no recovery replay, no invented
            // hold even when the disable fails); a confirmed disable or enable disarms it.
            #[cfg(not(target_arch = "wasm32"))]
            if !self.mit.joints.is_empty() {
                let mut mask = crate::torque::ChannelMask::EMPTY;
                for &j in &self.mit.joints {
                    if j < crate::torque::MAX_CHANNELS {
                        mask.insert(j as u8);
                    }
                }
                if emit_shadow {
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::MitOwnershipChange {
                            channels: mask,
                            target_mode: crate::motor_mode::ControlMode::PositionVelocity,
                            device_wide: false,
                        },
                    );
                } else {
                    self.torque_shadow.note_deferred_mit_off(mask);
                }
            }
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
        mut pre_ctx: crate::safety::ClearContext,
        commit: crate::supervisor::ClearFaultCommit,
        driver_err_override: Option<crate::safety::BoundedText<256>>,
    ) -> Result<crate::safety::SafetyState, crate::safety::SafetyError> {
        use crate::safety::{ClearRefusal, FaultDetail, FaultKind, SafetyError};
        // Recheck live J7 freshness, feedback validity, E-stop, and epoch under the SAME driver
        // lock guard that issues `drv.mit_expiry_recover()` so there is zero check-to-command gap
        // if J7 crosses 100 ms before the physical disable runs (ZERO commits & ZERO bus commands).
        pre_ctx.expected_epoch = expected_epoch;
        let (pre_recovery_seqs, res) = if let Some(ref mut d) = self.driver {
            let mut drv = match d.write() {
                Some(g) => g,
                None => {
                    pre_ctx.j7_feedback_fresh = false;
                    pre_ctx.feedback_fresh = false;
                    pre_ctx.pending_estop =
                        self.estop_state.requested_count() != self.applied_estop_count;
                    return Err(self
                        .safety
                        .clear_check(&pre_ctx)
                        .err()
                        .map_or(SafetyError::FeedbackUnavailable, ClearRefusal::into_safety_error));
                }
            };
            pre_ctx.j7_feedback_fresh = drv.pos_force_feedback_fresh();
            pre_ctx.feedback_fresh = pre_ctx.feedback_fresh && drv.is_feedback_valid();
            pre_ctx.pending_estop = self.estop_state.requested_count() != self.applied_estop_count;
            if let Err(refusal) = self.safety.clear_check(&pre_ctx) {
                drop(drv);
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
            if let Err(refusal) = Self::check_stopped_torque_off_with_driver(&self.config, drv.as_ref()) {
                drop(drv);
                self.last_stopped_torque_off_refusal = Some(refusal);
                return Err(refusal.into_safety_error(&self.config.joint_names));
            }
            let pre = drv.feedback_seqs();
            let r = drv.mit_expiry_recover();
            drop(drv);
            self.last_stopped_torque_off_refusal = None;
            (pre, r)
        } else {
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
            self.eval_stopped_torque_off()
                .map_err(|r| r.into_safety_error(&self.config.joint_names))?;
            (Vec::new(), Err("no driver".into()))
        };
        if let Err(e) = res {
            if self.estop_request_unapplied() {
                self.consume_pending_estop();
            }
            let msg = format!("MIT recovery after a motor timeout failed: {e}; the fault stays latched. \
                The recovery turns torque off on every joint (support the arm); check the named joints, then clear again");
            tracing::error!("{msg}");
            return Err(SafetyError::ClearFaultRejected(msg));
        }
        tracing::warn!("MIT recovery after a motor timeout: accepted disable sent; torque is off on all joints");
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        self.disable_accepted();

        // Read post-recovery feedback strictly AFTER the recovery disable (verified by per-joint
        // receive sequence advance AND driver post-recovery RX boundary), keeping telemetry,
        // cooldown, confirmation, and limits STAGED until `clear_with_check(&post_ctx)` commits.
        let dof = self.config.dof;
        let max_temp = self.config.max_temperature;
        let thermal_cooldown = self.thermal_cooldown;
        let post_read = self.driver.as_mut().and_then(|d| {
            d.with_mut(|drv| {
                let states = drv.read_joint_states().ok()?;
                let post_seqs = drv.feedback_seqs();
                let seqs_after_recovery = pre_recovery_seqs.len() == dof
                    && post_seqs.len() == dof
                    && post_seqs
                        .iter()
                        .zip(&pre_recovery_seqs)
                        .all(|(&post, &pre)| post > pre);
                let rx_boundary_ok = drv.mit_feedback_after_recovery(dof);
                let ages = drv.feedback_ages();
                let loads = drv.joint_loads();
                let caps = drv.capabilities();
                let fb_fresh = seqs_after_recovery
                    && rx_boundary_ok
                    && drv.is_feedback_valid()
                    && ages.len() == dof
                    && ages
                        .iter()
                        .all(|a| a.is_some_and(|t| t <= crate::supervisor::THERMAL_FEEDBACK_FRESHNESS_LIMIT))
                    && Self::states_valid(&states, caps, dof, loads.as_deref());
                let j7_fresh = drv.pos_force_feedback_fresh();
                let hw_err = drv.check_hardware_status().err().map(|e| e.to_clear_fault_error());
                let hw_fault = drv.has_hardware_fault();
                let enable_known = true;
                let oldest_age = ages
                    .iter()
                    .copied()
                    .try_fold(std::time::Duration::ZERO, |m, a| a.map(|t| m.max(t)));
                let threshold = max_temp - crate::supervisor::THERMAL_CLEAR_MARGIN_C;
                let thermal_below = fb_fresh
                    && caps.temperature_feedback
                    && !states.is_empty()
                    && states.iter().all(|s| s.temperature.is_finite() && s.temperature <= threshold);
                let mut rx_buf = Vec::new();
                let next_cd = Self::evaluate_thermal_cooldown_fields(
                    dof,
                    thermal_cooldown,
                    drv,
                    &ages,
                    thermal_below,
                    &mut rx_buf,
                );
                Some((
                    states,
                    loads,
                    caps.temperature_feedback,
                    fb_fresh,
                    j7_fresh,
                    thermal_below,
                    hw_err,
                    hw_fault,
                    enable_known,
                    oldest_age,
                    next_cd,
                ))
            })
            .flatten()
        });

        let mut post_ctx = self.build_mit_recovery_clear_context();
        post_ctx.driver_config_error = if commit.candidate_limits.is_some() || commit.reapply_legacy_limits {
            None
        } else {
            driver_err_override.or(post_ctx.driver_config_error)
        };
        if let Some((ref states, _, temp_fb, fb_fresh, j7_fresh, thermal_below, hw_err, hw_fault, enable_known, _, next_cd)) = post_read {
            if fb_fresh && temp_fb {
                if let Some((joint_idx, temperature)) = states
                    .iter()
                    .take(self.config.dof)
                    .map(|s| s.temperature)
                    .enumerate()
                    .filter(|(_, t)| t.is_finite() && *t > self.config.max_temperature)
                    .max_by(|a, b| a.1.total_cmp(&b.1))
                {
                    self.safety.latch(
                        FaultKind::ThermalTrip,
                        FaultDetail::ThermalTrip {
                            joint_idx,
                            temperature,
                            limit: self.config.max_temperature,
                        },
                    );
                }
            }
            let positions: Vec<f32> = states.iter().take(self.config.dof).map(|s| s.position).collect();
            post_ctx.feedback_fresh = fb_fresh;
            post_ctx.j7_feedback_fresh = post_ctx.j7_feedback_fresh && j7_fresh;
            post_ctx.thermal_sensor_available = temp_fb;
            post_ctx.thermal_below_threshold = thermal_below;
            post_ctx.thermal_cooldown_elapsed_ms = if thermal_below { next_cd.elapsed_ms() } else { 0 };
            post_ctx.joint_limit_violation = if fb_fresh {
                self.joint_limit_clear_violation_for_positions(&positions, commit.candidate_limits.as_ref())
            } else {
                self.joint_limit_clear_violation_with_candidates(commit.candidate_limits.as_ref())
            };
            post_ctx.hardware_error = hw_err;
            post_ctx.hardware_reset_required = hw_fault;
            post_ctx.enable_state_known = enable_known;
        } else {
            post_ctx.feedback_fresh = false;
            post_ctx.j7_feedback_fresh = false;
            post_ctx.thermal_sensor_available = false;
            post_ctx.thermal_below_threshold = false;
            post_ctx.thermal_cooldown_elapsed_ms = 0;
            post_ctx.hardware_error = None;
            post_ctx.hardware_reset_required = false;
            post_ctx.enable_state_known = false;
        }

        // An E-stop that arrived while the disables were confirmed wins: never end in Idle.
        self.consume_pending_estop();
        post_ctx.expected_epoch = expected_epoch;
        post_ctx.pending_estop = self.estop_request_unapplied();
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
        // Single authoritative clear commit FIRST — recheck live J7 freshness and feedback
        // validity under the driver lock guard across `clear_with_check(&post_ctx)` so an earlier
        // read can never clear a now-stale J7 fault (ZERO feedback/cooldown/limit/confirmation
        // commits on refusal).
        let clear_res = if let Some(ref d) = self.driver {
            match d.read() {
                Some(drv) => {
                    post_ctx.j7_feedback_fresh =
                        post_ctx.j7_feedback_fresh && drv.pos_force_feedback_fresh();
                    post_ctx.feedback_fresh = post_ctx.feedback_fresh && drv.is_feedback_valid();
                    post_ctx.pending_estop =
                        self.estop_state.requested_count() != self.applied_estop_count;
                    self.safety.clear_with_check(&post_ctx)
                }
                None => {
                    post_ctx.j7_feedback_fresh = false;
                    post_ctx.feedback_fresh = false;
                    post_ctx.pending_estop = self.estop_request_unapplied();
                    self.safety.clear_with_check(&post_ctx)
                }
            }
        } else {
            self.safety.clear_with_check(&post_ctx)
        };
        if let Err(refusal) = clear_res {
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

        // Clear committed: apply staged post-recovery feedback, cooldown, limits, and confirmation.
        self.mit.recovery_confirmed = false;
        if let Some((states, loads, temp_fb, fb_fresh, _, _, _, _, _, oldest_age, next_cd)) = post_read {
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
                if self.measured_positions.iter().all(|q| q.is_finite()) {
                    crate::rt_core::scratch::copy_into(&mut self.commanded_positions, &self.measured_positions);
                    crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.measured_positions);
                }
                if temp_fb {
                    self.thermal_cooldown = next_cd;
                } else {
                    self.thermal_cooldown.reset();
                }
            } else {
                self.last_can_feedback_instant = None;
                self.thermal_cooldown.reset();
            }
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        if let Some(ref cand) = commit.candidate_limits {
            self.commit_driver_limits_bounded(cand);
        } else if commit.reapply_legacy_limits {
            self.reapply_driver_limits();
        }
        self.mit.owner = MitOwner::Off;
        self.mit.joints.clear();
        self.mit.last_ff.clear();
        self.mit.last_output = None;
        self.mit.armed_ok = false;
        self.mit.revoke_pending = false;
        if let Some(d) = self.driver.as_mut() {
            let _ = d.with_mut(|drv| drv.acknowledge_mit_fence_trip());
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
        res.map_err(Self::map_clear_fault_mit_err)
    }

    pub(crate) fn clear_fault_confirmed_for_epoch_with_authority(
        &mut self,
        mit_recovery: bool,
        authority: crate::rt_core::requests::lease::ClearFaultAuthority,
    ) -> Result<crate::safety::SafetyState, crate::supervisor::ClearFaultLegacyWithAuthError> {
        self.mit.recovery_confirmed = mit_recovery;
        let res = self.clear_fault_for_epoch_with_authority(Some(authority.expected_epoch), Some(authority));
        self.mit.recovery_confirmed = false;
        res.map_err(|e| match e {
            crate::supervisor::ClearFaultLegacyInnerError::Authority(a) => {
                crate::supervisor::ClearFaultLegacyWithAuthError::Authority(a)
            }
            crate::supervisor::ClearFaultLegacyInnerError::Safety(s) => {
                crate::supervisor::ClearFaultLegacyWithAuthError::Arm(Self::map_clear_fault_mit_err(s))
            }
        })
    }

    fn map_clear_fault_mit_err(e: crate::safety::SafetyError) -> crate::supervisor::ArmError {
        match e {
            crate::safety::SafetyError::ClearFaultRejected(m) if m.starts_with(RECOVERY_CONFIRM_CODE) =>
                crate::supervisor::ArmError::Rejected { status: 409, code: RECOVERY_CONFIRM_CODE.into(), message: RECOVERY_WARNING.into() },
            crate::safety::SafetyError::ClearFaultRejected(m) if m.contains(QUIET_PERIOD_CODE) =>
                crate::supervisor::ArmError::Rejected { status: 409, code: QUIET_PERIOD_CODE.into(), message: m },
            other => crate::supervisor::ArmError::from(other),
        }
    }
}

// ============================================================================
// C-F1 part 2: Executor-side tick-driven hardware commissioning state machine
// ============================================================================

/// Stack-only, zero-allocation register reply entry recorded by `DamiaoArmDriver`
/// for causal per-request verification during RT commissioning.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy)]
pub struct CommissioningRegReply {
    pub motor_id: u16,
    pub write: bool,
    pub rid: u8,
    pub reply_id: u32,
    pub value: [u8; 4],
    pub rx_at: std::time::Instant,
    pub rx_seq: u64,
    pub rx_empty_seq: u64,
    pub decode_offset_ns: Option<i128>,
}

/// Outcome of a single non-blocking commissioning TX attempt at the driver boundary.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommissioningTxStatus {
    /// Transport accepted and (when required) flushed the frame.
    Sent {
        tx_seq: u64,
        tx_started: std::time::Instant,
        tx_offset_ns: Option<i128>,
    },
    /// Transport accepted the frame, but flush failed or release cutoff expired before flush.
    /// Irreversible writes MUST mark the joint `Uncertain` and NEVER auto-retry.
    SentFlushFailed {
        tx_seq: u64,
        tx_started: std::time::Instant,
        tx_offset_ns: Option<i128>,
    },
    /// Transient TX backpressure or tick budget cutoff before transport accepted the frame.
    /// Frame never left host; safe to retry on a later release after rechecking live authority.
    Backpressure,
    /// Pre-transmit live authority/stop/lease/operator gate refused immediately before `transmit`.
    Revoked(&'static str),
    /// Driver refused before transmit (e.g. torque not confirmed off, rest gate not confirmed).
    Refused(&'static str),
}

/// Outcome of one bounded non-blocking disable-all step during hardware motor zero.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommissioningDisableStep {
    /// Every motor has received a disable frame and the bus was flushed.
    AllDispatched {
        boundary_seq: u64,
        tx_started: std::time::Instant,
        tx_offset_ns: Option<i128>,
    },
    /// Some motors still need a disable frame on a later release (cutoff/backpressure).
    InProgress { dispatched_mask: u64 },
    /// Waiting for rest-pose evidence window/drain within the disable request lifetime.
    WaitEvidence,
    /// Live authority/stop gate revoked before completing disable dispatch.
    Revoked(&'static str),
    /// Refused (e.g. not at rest, expired, fatal transport error).
    Refused(&'static str),
}

/// Per-joint fresh feedback snapshot for commissioning verification (raw + calibrated).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy)]
pub struct CommissioningFeedback {
    /// Raw motor encoder position (rad), before `ZeroOffsetDriver` offset subtraction.
    pub raw_position: f32,
    /// Calibrated joint position (rad), after `ZeroOffsetDriver` offset subtraction.
    pub joint_position: f32,
    pub velocity: f32,
    pub motor_status: u8,
    pub rx_seq: u64,
    pub rx_at: std::time::Instant,
    pub queue_bound: Option<std::time::Instant>,
}

/// Fail-closed causal match result for a single commissioning register request (B1).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy)]
pub enum CommissioningRegMatch {
    /// No matching reply dequeued after `after_tx_seq` yet.
    Pending,
    /// Exactly one reply matched `(motor_id, op, rid)` strictly after `(after_tx_seq, min_rx_at)`
    /// with bounded realtime-to-monotonic clock offset drift.
    Matched(CommissioningRegReply),
    /// Multiple replies for `(motor_id, op, rid)` or a stale/drifted reply arrived after the
    /// empty-queue TX boundary: causal attribution cannot prove this reply belongs to the current
    /// request, so callers MUST fail closed (preserving `Uncertain` on writes).
    Ambiguous,
}

/// Copyable live authority binding checked on the RT owner before EVERY irreversible TX.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy)]
pub struct CommissioningAuthority {
    pub conn_id: u64,
    pub lease_holder: Option<(crate::model::HolderLabel, u64)>,
    pub operator_grant: crate::auth::OperatorGrant,
    pub expected_epoch: u64,
    pub expected_stop_gen: u64,
    pub grant_id: Option<crate::rt_core::config_iface::GrantId>,
    pub txn_id: u64,
}

/// Three-valued per-joint commissioning outcome.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum JointCommissioningDisposition {
    /// Fresh post-command verification confirmed the new state.
    Applied,
    /// Transport accepted an irreversible write, but verification did not succeed.
    /// NEVER auto-retry; fences motion + rearm until reconciled.
    Uncertain,
    /// No irreversible write was accepted by the transport for this joint.
    #[default]
    Unattempted,
}

/// Stack-only copyable motor identity snapshot (zero heap alloc on RT).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommissioningIdentity {
    pub joint: usize,
    pub motor_id: u16,
    pub reply_id: u32,
    pub esc_id: u32,
    pub mst_id: u32,
    pub sw_ver: u32,
    pub ctrl_mode: u32,
    pub pmax: f32,
    pub vmax: f32,
    pub tmax: f32,
    pub timeout_raw: u32,
}

#[cfg(not(target_arch = "wasm32"))]
impl CommissioningIdentity {
    pub fn into_motor_identity(self) -> MotorIdentity {
        MotorIdentity {
            joint: self.joint,
            motor_id: self.motor_id,
            reply_id: self.reply_id,
            esc_id: self.esc_id,
            mst_id: self.mst_id,
            sw_ver: self.sw_ver,
            ctrl_mode: self.ctrl_mode,
            pmax: self.pmax,
            vmax: self.vmax,
            tmax: self.tmax,
            timeout_raw: self.timeout_raw,
        }
    }
}

/// Fixed-size per-joint commissioning result (Copy, zero heap alloc on RT).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy)]
pub struct CommissioningJointOutcome {
    pub joint: usize,
    pub disposition: JointCommissioningDisposition,
    pub before: Option<CommissioningIdentity>,
    pub written_raw: u32,
    pub ack_raw: u32,
    pub readback: Option<CommissioningIdentity>,
    pub raw_after: f32,
    pub joint_after: f32,
    pub fresh: bool,
    pub verified: bool,
}

#[cfg(not(target_arch = "wasm32"))]
impl Default for CommissioningJointOutcome {
    fn default() -> Self {
        Self {
            joint: 0,
            disposition: JointCommissioningDisposition::Unattempted,
            before: None,
            written_raw: 0,
            ack_raw: 0,
            readback: None,
            raw_after: f32::NAN,
            joint_after: f32::NAN,
            fresh: false,
            verified: false,
        }
    }
}

/// Kind of RT tick-driven hardware commissioning operation.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommissioningOpKind {
    ReadIdentity,
    WriteTimeout,
    MotorZeroDisable,
    MotorZeroWriteVerify,
}

#[cfg(not(target_arch = "wasm32"))]
impl CommissioningOpKind {
    pub fn is_mutating(self) -> bool {
        matches!(self, Self::WriteTimeout | Self::MotorZeroWriteVerify)
    }
}

/// Terminal / active state of the single commissioning slot.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CommissioningOpState {
    #[default]
    Idle,
    Running,
    Done,
    Failed,
    Cancelled,
}

/// Start parameters for one RT commissioning operation (stack-only, 0 alloc on RT).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy)]
pub struct CommissioningStartRequest {
    pub kind: CommissioningOpKind,
    pub joints: [usize; crate::rt_core::mailbox::MAX_DOF],
    pub joint_len: usize,
    pub timeout_counts: u32,
    pub references: [f32; crate::rt_core::mailbox::MAX_DOF],
    pub authority: CommissioningAuthority,
    pub txn_deadline: std::time::Instant,
    /// When true, the caller already persisted a durable config mutation (e.g. MIT invalidation)
    /// under this transaction that must stay consistent on failure/cancellation.
    pub prior_disk_mutated: bool,
}

/// Stack-only snapshot returned by `PollCommissioning` / `CancelCommissioning` (0 alloc on RT).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy)]
pub struct CommissioningSnapshot {
    pub op_id: u64,
    pub txn_id: u64,
    pub kind: CommissioningOpKind,
    pub state: CommissioningOpState,
    pub joint_len: usize,
    pub outcomes: [CommissioningJointOutcome; crate::rt_core::mailbox::MAX_DOF],
    pub hw_mutated: bool,
    pub disk_mutated: bool,
    pub supports_mit_switch: bool,
    pub error_status: u16,
    pub error_code: &'static str,
    pub error_msg: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
}

#[cfg(not(target_arch = "wasm32"))]
const COMM_IDENTITY_RIDS: [u8; 8] = [
    damiao::RID_ESC_ID,
    damiao::RID_MST_ID,
    damiao::RID_SW_VER,
    damiao::RID_CTRL_MODE,
    damiao::RID_PMAX,
    damiao::RID_VMAX,
    damiao::RID_TMAX,
    damiao::RID_TIMEOUT,
];

/// Bounded per-request register reply wait timeout on the driver clock.
#[cfg(not(target_arch = "wasm32"))]
const COMM_REG_REPLY_TIMEOUT: Duration = Duration::from_millis(50);
/// Documented quiet-bus latency bound after an unanswered or ambiguous register request on 1 Mbps
/// Damiao CAN (2x `COMM_REG_REPLY_TIMEOUT`): no subsequent register request may start until the bus
/// stays quiet past this bound.
#[cfg(not(target_arch = "wasm32"))]
const COMM_REG_QUIET_BOUND: Duration = Duration::from_millis(100);
/// Maximum non-blocking TX attempts for one register or motor-zero frame before failing.
#[cfg(not(target_arch = "wasm32"))]
const COMM_MAX_FRAME_ATTEMPTS: u8 = 16;
/// Maximum commissioning register TX attempts per control release.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) const COMM_MAX_TX_PER_RELEASE: u8 = 4;

/// Zero-allocation summary of one bounded commissioning step (used by the legacy post-tick
/// scheduler to split commissioning across short `<= 2 ms` `SupervisorLock::write` guards
/// inside the remaining release budget).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct CommissioningStepProgress {
    /// Commissioning operation remains `Running` after this step.
    pub still_running: bool,
    /// Register read/write (`0x33`/`0x55`) or set-zero TX frames attempted in this step.
    pub reg_tx_attempted: u8,
    /// Read-only `0xCC` rest-pose probe TX frames attempted in this step.
    pub probe_tx_attempted: u8,
    /// True when a register read or write is currently in flight awaiting a reply.
    pub awaiting_reply: bool,
    /// True when `GatherRestEvidence` still has unsent `0xCC` probes in the current release burst.
    pub rest_probe_pending: bool,
    /// True when the state machine has immediate non-waiting register work ready without sleeping.
    pub more_immediate_work: bool,
    /// True when CAN TX backpressure or driver TX budget refusal stopped this step.
    pub backpressured: bool,
    /// True when `DriverSlot::Legacy::try_lock` returned `WouldBlock`.
    pub contended: bool,
}
/// Maximum stationary velocity tolerance (rad/s) for commissioning writes.
#[cfg(not(target_arch = "wasm32"))]
const COMM_STATIONARY_VEL_RAD_S: f32 = 0.05;
/// Bounded wall/driver-clock window for active read-only `0xCC` rest-pose evidence gathering
/// while disarmed before a TIMEOUT commissioning read/write step.
#[cfg(not(target_arch = "wasm32"))]
const COMM_REST_EVIDENCE_TIMEOUT: Duration = Duration::from_millis(500);
/// Minimum spacing between read-only `0xCC` rest-pose probe bursts during commissioning precheck
/// (aligned with `EVIDENCE_PROBE_INTERVAL` so K=3 causal samples span >= 20 ms across >= 3 ticks).
#[cfg(not(target_arch = "wasm32"))]
const COMM_REST_PROBE_INTERVAL: Duration = Duration::from_millis(10);

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RegReadPhase {
    QuietWait,
    DrainBeforeReq,
    SendReq,
    AwaitReply,
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WriteTimeoutPhase {
    GatherRestEvidence,
    ReadBefore,
    DrainBeforeWrite,
    SendWrite,
    AwaitWriteAck,
    ReadAfter,
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MotorZeroPhase {
    DisableDispatch,
    DisableAwaitConfirm,
    ZeroSendLoop,
    ZeroVerify,
}

/// Preallocated, tick-driven, zero-allocation commissioning state machine embedded inline
/// inside `SupervisorInner` (at most one active operation at a time).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone)]
pub struct CommissioningMachine {
    active_op_id: u64,
    next_op_id: u64,
    session_id: Option<crate::rt_core::session::SessionId>,
    txn_id: u64,
    kind: CommissioningOpKind,
    state: CommissioningOpState,
    joints: [usize; crate::rt_core::mailbox::MAX_DOF],
    joint_len: usize,
    timeout_counts: u32,
    references: [f32; crate::rt_core::mailbox::MAX_DOF],
    authority: Option<CommissioningAuthority>,
    txn_deadline: Option<std::time::Instant>,
    hw_mutated: bool,
    disk_mutated: bool,
    supports_mit_switch: bool,
    outcomes: [CommissioningJointOutcome; crate::rt_core::mailbox::MAX_DOF],
    error_status: u16,
    error_code: &'static str,
    error_msg: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
    /// Quiet-window barrier after a timed-out or cancelled register request so a late
    /// reply from the abandoned request can never match a subsequent request for the same tuple.
    unmatched_reply_quiet_until: Option<std::time::Instant>,
    // Per-joint & per-request cursor state (all Copy / fixed-size, 0 heap)
    joint_idx: usize,
    rid_idx: usize,
    reg_phase: RegReadPhase,
    write_phase: WriteTimeoutPhase,
    mz_phase: MotorZeroPhase,
    read_vals: [([u8; 4], u32); 8],
    req_tx_seq: u64,
    req_tx_started: Option<std::time::Instant>,
    req_tx_offset_ns: Option<i128>,
    req_deadline: Option<std::time::Instant>,
    req_attempts: u8,
    req_in_flight: bool,
    // Bounded read-only 0xCC rest-pose evidence gather state for disarmed WriteTimeout precheck
    release_step_count: u64,
    rest_gather_deadline: Option<std::time::Instant>,
    rest_probe_next_at: Option<std::time::Instant>,
    rest_probe_pending_mask: u64,
    rest_probe_last_release_seq: u64,
    rest_probe_tx_seq: [u64; crate::rt_core::mailbox::MAX_DOF],
    rest_probe_tx_at: [Option<std::time::Instant>; crate::rt_core::mailbox::MAX_DOF],
    rest_probe_confirmed_mask: u64,
    rest_gather_resume_phase: WriteTimeoutPhase,
    // Motor-zero disable & write-verify state
    disable_started_at: Option<std::time::Instant>,
    disable_boundary_seq: u64,
    disable_tx_started: Option<std::time::Instant>,
    disable_deadline: Option<std::time::Instant>,
    next_zero_send_at: Option<std::time::Instant>,
    zero_boundary_seq: [u64; crate::rt_core::mailbox::MAX_DOF],
    zero_tx_started: [Option<std::time::Instant>; crate::rt_core::mailbox::MAX_DOF],
    zero_sent_mask: u64,
    /// Bitmask indexed by physical 0-based joint index (`1u64 << joint`) of joints that already
    /// received `CMD_SET_ZERO` in this batch (used by B4 mid-batch whole-arm rest-pose verification).
    zero_sent_joint_mask: u64,
    zero_verify_deadline: Option<std::time::Instant>,
}

#[cfg(not(target_arch = "wasm32"))]
impl Default for CommissioningMachine {
    fn default() -> Self {
        Self {
            active_op_id: 0,
            next_op_id: 1,
            session_id: None,
            txn_id: 0,
            kind: CommissioningOpKind::ReadIdentity,
            state: CommissioningOpState::Idle,
            joints: [0; crate::rt_core::mailbox::MAX_DOF],
            joint_len: 0,
            timeout_counts: 0,
            references: [0.0; crate::rt_core::mailbox::MAX_DOF],
            authority: None,
            txn_deadline: None,
            hw_mutated: false,
            disk_mutated: false,
            supports_mit_switch: false,
            outcomes: [CommissioningJointOutcome::default(); crate::rt_core::mailbox::MAX_DOF],
            error_status: 0,
            error_code: "",
            error_msg: crate::safety::BoundedText::empty(),
            unmatched_reply_quiet_until: None,
            joint_idx: 0,
            rid_idx: 0,
            reg_phase: RegReadPhase::QuietWait,
            write_phase: WriteTimeoutPhase::ReadBefore,
            mz_phase: MotorZeroPhase::DisableDispatch,
            read_vals: [([0; 4], 0); 8],
            req_tx_seq: 0,
            req_tx_started: None,
            req_tx_offset_ns: None,
            req_deadline: None,
            req_attempts: 0,
            req_in_flight: false,
            release_step_count: 0,
            rest_gather_deadline: None,
            rest_probe_next_at: None,
            rest_probe_pending_mask: 0,
            rest_probe_last_release_seq: 0,
            rest_probe_tx_seq: [0; crate::rt_core::mailbox::MAX_DOF],
            rest_probe_tx_at: [None; crate::rt_core::mailbox::MAX_DOF],
            rest_probe_confirmed_mask: 0,
            rest_gather_resume_phase: WriteTimeoutPhase::ReadBefore,
            disable_started_at: None,
            disable_boundary_seq: 0,
            disable_tx_started: None,
            disable_deadline: None,
            next_zero_send_at: None,
            zero_boundary_seq: [0; crate::rt_core::mailbox::MAX_DOF],
            zero_tx_started: [None; crate::rt_core::mailbox::MAX_DOF],
            zero_sent_mask: 0,
            zero_sent_joint_mask: 0,
            zero_verify_deadline: None,
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl CommissioningMachine {
    pub fn is_active(&self) -> bool {
        self.state == CommissioningOpState::Running
    }

    pub fn is_occupied(&self) -> bool {
        self.state != CommissioningOpState::Idle
    }

    pub(crate) fn is_running(&self) -> bool {
        self.state == CommissioningOpState::Running
    }

    pub(crate) fn is_running_op(&self, op_id: u64) -> bool {
        self.state == CommissioningOpState::Running && self.active_op_id == op_id
    }

    pub fn active_op_id(&self) -> Option<u64> {
        (self.state != CommissioningOpState::Idle).then_some(self.active_op_id)
    }

    pub fn session_id(&self) -> Option<crate::rt_core::session::SessionId> {
        (self.state != CommissioningOpState::Idle)
            .then_some(self.session_id)
            .flatten()
    }

    pub(crate) fn has_uncertain_outcome(&self) -> bool {
        self.outcomes[..self.joint_len]
            .iter()
            .any(|o| o.disposition == JointCommissioningDisposition::Uncertain)
    }

    pub fn hw_mutated(&self) -> bool {
        self.hw_mutated
    }

    pub fn disk_mutated(&self) -> bool {
        self.disk_mutated
    }

    pub fn snapshot(&self, op_id: u64) -> Option<CommissioningSnapshot> {
        if self.state == CommissioningOpState::Idle || self.active_op_id != op_id {
            return None;
        }
        Some(CommissioningSnapshot {
            op_id: self.active_op_id,
            txn_id: self.txn_id,
            kind: self.kind,
            state: self.state,
            joint_len: self.joint_len,
            outcomes: self.outcomes,
            hw_mutated: self.hw_mutated,
            disk_mutated: self.disk_mutated,
            supports_mit_switch: self.supports_mit_switch,
            error_status: self.error_status,
            error_code: self.error_code,
            error_msg: self.error_msg,
        })
    }

    fn set_error(&mut self, state: CommissioningOpState, status: u16, code: &'static str, msg: &str) {
        self.state = state;
        self.error_status = status;
        self.error_code = code;
        self.error_msg = crate::safety::BoundedText::new(msg);
    }

    fn build_identity(&self, joint: usize, motor_id: u16) -> Result<CommissioningIdentity, &'static str> {
        let reply_id = self.read_vals[0].1;
        for &(_, fid) in &self.read_vals {
            if fid != reply_id {
                return Err("register replies arrived on inconsistent CAN IDs");
            }
        }
        let u = |k: usize| u32::from_le_bytes(self.read_vals[k].0);
        let f = |k: usize| f32::from_le_bytes(self.read_vals[k].0);
        Ok(CommissioningIdentity {
            joint: joint + 1,
            motor_id,
            reply_id,
            esc_id: u(0),
            mst_id: u(1),
            sw_ver: u(2),
            ctrl_mode: u(3),
            pmax: f(4),
            vmax: f(5),
            tmax: f(6),
            timeout_raw: u(7),
        })
    }

    fn mark_quiet_if_in_flight(&mut self, drv_now: std::time::Instant) {
        if self.req_in_flight {
            let until = drv_now + COMM_REG_QUIET_BOUND;
            self.unmatched_reply_quiet_until = Some(
                self.unmatched_reply_quiet_until
                    .map_or(until, |prev| prev.max(until)),
            );
            self.req_in_flight = false;
        }
    }

    fn reset_reg_read(&mut self) {
        self.rid_idx = 0;
        self.reg_phase = RegReadPhase::QuietWait;
        self.req_tx_seq = 0;
        self.req_tx_started = None;
        self.req_tx_offset_ns = None;
        self.req_deadline = None;
        self.req_attempts = 0;
        self.req_in_flight = false;
    }

    fn reset_rest_gather(&mut self, resume_phase: WriteTimeoutPhase) {
        self.rest_gather_deadline = None;
        self.rest_probe_next_at = None;
        self.rest_probe_pending_mask = 0;
        self.rest_probe_last_release_seq = 0;
        self.rest_probe_tx_seq = [0; crate::rt_core::mailbox::MAX_DOF];
        self.rest_probe_tx_at = [None; crate::rt_core::mailbox::MAX_DOF];
        self.rest_probe_confirmed_mask = 0;
        self.rest_gather_resume_phase = resume_phase;
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl crate::supervisor::SupervisorInner {
    /// Zero-allocation live authority recheck for RT commissioning operations, using a fresh
    /// owner clock read (`owner_now`) and live atomic stop/E-stop/disconnect/operator grants.
    ///
    /// B2: All mutating/hardware-commissioning ops (`WriteTimeout`, `MotorZeroDisable`,
    /// `MotorZeroWriteVerify`) strictly require an owner-validated `txn_id` and active
    /// `HoldState::Reserved` calibration reservation; irreversible write ops (`WriteTimeout`,
    /// `MotorZeroWriteVerify`) additionally require durable pending-marker proof bound to that
    /// reservation on the RT owner.
    pub(crate) fn commissioning_live_authority_ok(
        &self,
        kind: CommissioningOpKind,
        auth: &CommissioningAuthority,
        owner_now: std::time::Instant,
    ) -> Result<(), (u16, &'static str, &'static str)> {
        if auth.conn_id != 0 && self.is_connection_closed(auth.conn_id) {
            return Err((409, "LEASE_LOST", "This connection closed during commissioning"));
        }
        if self.estop_request_unapplied() || self.safety.epoch_id() != auth.expected_epoch {
            return Err((
                409,
                "EPOCH_MISMATCH",
                "The epoch changed during commissioning (stop or e-stop)",
            ));
        }
        if self.urgent_stop_unapplied() || self.current_stop_gen() != auth.expected_stop_gen {
            return Err((409, "STOPPED", "Stopped during commissioning"));
        }
        match auth.lease_holder {
            Some((conn, gen))
                if gen != 0
                    && self.is_lease_valid_for_conn_at(conn.as_str(), gen, owner_now) => {}
            _ => {
                return Err((
                    409,
                    "LEASE_LOST",
                    "This connection lost the lease during commissioning",
                ));
            }
        }
        if !auth
            .operator_grant
            .is_valid_at(owner_now, self.operator_auth_enforced)
        {
            return Err((
                403,
                "OPERATOR_REQUIRED",
                "Operator session expired or locked during commissioning",
            ));
        }
        let requires_txn_and_hold = matches!(
            kind,
            CommissioningOpKind::WriteTimeout
                | CommissioningOpKind::MotorZeroDisable
                | CommissioningOpKind::MotorZeroWriteVerify
        );
        if requires_txn_and_hold {
            if auth.txn_id == 0 || self.calibration_txn_id != Some(auth.txn_id) {
                return Err((
                    409,
                    "CONFIG_BUSY",
                    "Commissioning requires an active owner-validated calibration transaction",
                ));
            }
            let Some(grant) = auth.grant_id else {
                return Err((
                    409,
                    "CONFIG_BUSY",
                    "Commissioning requires an active Reserved calibration hold",
                ));
            };
            let hold_ok = self.core.config().held().is_some_and(|h| {
                h.domain == crate::rt_core::config_iface::ConfigDomain::Calibration
                    && h.grant == Some(grant)
                    && h.state == crate::rt_core::config_iface::HoldState::Reserved
            });
            if !hold_ok {
                return Err((
                    409,
                    "CONFIG_BUSY",
                    "Commissioning reservation is missing or no longer Reserved",
                ));
            }
            if kind.is_mutating() && !self.core.config().is_commissioning_marker_bound(grant) {
                return Err((
                    409,
                    "CONFIG_BUSY",
                    "Commissioning write requires durable pending-marker binding on the active reservation",
                ));
            }
        } else {
            if auth.txn_id != 0 && self.calibration_txn_id != Some(auth.txn_id) {
                return Err((
                    409,
                    "CONFIG_BUSY",
                    "Commissioning transaction ownership was lost",
                ));
            }
            if let Some(grant) = auth.grant_id {
                if !self
                    .core
                    .config()
                    .held()
                    .is_some_and(|h| h.grant == Some(grant))
                {
                    return Err((
                        409,
                        "CONFIG_BUSY",
                        "Commissioning reservation was lost",
                    ));
                }
            }
        }
        Ok(())
    }

    /// Zero-allocation safety/hardware admission check for commissioning.
    /// Irreversible writes require stationary arm, supported HW, torque confirmed off,
    /// no active fault/E-stop, and fresh rest-pose evidence (NO relaxation for `force`).
    fn commissioning_safety_check_with_driver(
        &self,
        kind: CommissioningOpKind,
        joints: &[usize],
        drv: Option<&dyn crate::driver::ArmDriver>,
    ) -> Result<(), (u16, &'static str, &'static str)> {
        self.commissioning_safety_check_with_driver_inner(kind, joints, drv, false)
    }

    /// Inner zero-allocation safety/hardware check.
    /// When `allow_rest_evidence_gather` is true (ONLY for `WriteTimeout` admission and while
    /// actively polling read-only `0xCC` rest-pose evidence before any register read/write),
    /// hard safety refusals (fault, torque on, motion, non-stationary, fresh non-disabled motor,
    /// or `RestPoseGateStatus::RefusedNonRest`) still fail immediately, while stale/incomplete
    /// rest-pose evidence or stale disabled feedback is permitted so `GatherRestEvidence` can
    /// actively collect K=3 causal fresh samples while disarmed.
    fn commissioning_safety_check_with_driver_inner(
        &self,
        kind: CommissioningOpKind,
        joints: &[usize],
        drv: Option<&dyn crate::driver::ArmDriver>,
        allow_rest_evidence_gather: bool,
    ) -> Result<(), (u16, &'static str, &'static str)> {
        let can_gather = allow_rest_evidence_gather && kind == CommissioningOpKind::WriteTimeout;
        let is_zero = matches!(
            kind,
            CommissioningOpKind::MotorZeroDisable | CommissioningOpKind::MotorZeroWriteVerify
        );
        if self.estop_pending() || self.safety.state().is_stopped() || self.driver_config_error.is_some() {
            return Err(if is_zero {
                (409, "MOTOR_ZERO_FAULT", "the arm is stopped or faulted; clear the fault first")
            } else {
                (409, "MIT_TIMEOUT_ARMED", "the arm is stopped or faulted; clear the fault first")
            });
        }
        if self.is_torqued
            || self.disable_pending
            || self.torque_transition.is_some()
            || self.handover_settling_since.is_some()
        {
            return Err(if is_zero {
                (409, "MOTOR_ZERO_ARMED", "motor zero needs torque off and no action running (torque is on); park or disable torque first")
            } else {
                (409, "MIT_TIMEOUT_ARMED", "torque must be off (load mechanically supported) for the TIMEOUT commissioning")
            });
        }
        if self.active_trajectory.is_some()
            || self.servo_target.is_some()
            || self.gravity_comp.is_some()
            || self.last_action.as_ref().is_some_and(|a| a.is_running())
            || self.macro_run.is_some()
        {
            return Err(if is_zero {
                (409, "MOTOR_ZERO_ARMED", "motor zero needs torque off and no action running (a motion is running); park or disable torque first")
            } else {
                (409, "MIT_TIMEOUT_ARMED", "a motion is running")
            });
        }
        if is_zero && !matches!(self.safety.state(), crate::safety::SafetyState::Idle) {
            return Err((409, "MOTOR_ZERO_ARMED", "motor zero needs torque off and Idle state"));
        }
        let Some(drv) = drv else {
            return if is_zero {
                Ok(())
            } else {
                Err((400, "UNSUPPORTED", "no motor driver"))
            };
        };
        if drv.has_hardware_fault() {
            return Err(if is_zero {
                (409, "MOTOR_ZERO_FAULT", "motor hardware fault active; clear the fault first")
            } else {
                (409, "MIT_TIMEOUT_ARMED", "motor hardware fault active; clear the fault first")
            });
        }
        match kind {
            CommissioningOpKind::ReadIdentity | CommissioningOpKind::WriteTimeout => {
                if !drv.supports_nonblocking_commissioning() {
                    return Err((400, "UNSUPPORTED", "this driver does not support non-blocking register commissioning"));
                }
            }
            CommissioningOpKind::MotorZeroDisable | CommissioningOpKind::MotorZeroWriteVerify => {
                if !drv.supports_nonblocking_motor_zero() {
                    return Err((400, "UNSUPPORTED", "this driver does not support non-blocking motor zero"));
                }
            }
        }
        if !kind.is_mutating() {
            return Ok(());
        }
        // Stationary check: every joint velocity (both supervisor snapshot and any fresh driver
        // feedback decoded in the current release) must be finite and within resting velocity bound.
        let dof = self.config.dof.min(self.measured_velocities.len());
        let drv_now = drv.clock_now();
        for j in 0..dof {
            let v = self.measured_velocities[j];
            let vlim = self
                .config
                .resting_pose
                .as_ref()
                .map_or(COMM_STATIONARY_VEL_RAD_S, |rp| {
                    rp.max_velocity_of(j).min(COMM_STATIONARY_VEL_RAD_S)
                });
            if !v.is_finite() || v.abs() > vlim {
                return Err(if is_zero {
                    (409, "MOTOR_ZERO_ARMED", "arm must be stationary for motor zero")
                } else {
                    (409, "MIT_TIMEOUT_ARMED", "arm must be stationary for TIMEOUT commissioning")
                });
            }
            if let Some(fb) = drv.commissioning_raw_feedback(j) {
                let age = drv_now.saturating_duration_since(fb.rx_at);
                if age <= crate::driver::ZERO_FEEDBACK_MAX_AGE
                    && (!fb.velocity.is_finite() || fb.velocity.abs() > vlim)
                {
                    return Err(if is_zero {
                        (409, "MOTOR_ZERO_ARMED", "arm must be stationary for motor zero")
                    } else {
                        (409, "MIT_TIMEOUT_ARMED", "arm must be stationary for TIMEOUT commissioning")
                    });
                }
            }
        }
        // Torque confirmed off at the driver.
        if drv.is_enabled() || drv.has_any_joint_enabled() {
            return Err(if is_zero {
                (409, "MOTOR_ZERO_ARMED", "driver reports torque enabled")
            } else {
                (409, "MIT_TIMEOUT_ARMED", "driver reports torque enabled")
            });
        }
        if drv.capabilities().enable_feedback {
            // Any fresh non-zero motor_status is an immediate hard refusal (even when gathering).
            for j in 0..dof {
                if let Some(fb) = drv.commissioning_raw_feedback(j) {
                    let age = drv_now.saturating_duration_since(fb.rx_at);
                    if age <= crate::driver::ZERO_FEEDBACK_MAX_AGE && fb.motor_status != 0 {
                        return Err(if is_zero {
                            (409, "MOTOR_ZERO_NOT_DISABLED", "joint is not confirmed disabled from fresh feedback")
                        } else {
                            (502, "MIT_TIMEOUT_FAILED", "joint is not confirmed disabled from fresh feedback")
                        });
                    }
                }
            }
            if !can_gather && drv.enable_state() != crate::model::EnableState::Disabled {
                return Err(if is_zero {
                    (409, "MOTOR_ZERO_NOT_DISABLED", "motors are not confirmed disabled from fresh feedback")
                } else {
                    (502, "MIT_TIMEOUT_FAILED", "motors are not confirmed disabled from fresh feedback")
                });
            }
        }
        if !can_gather {
            for &j in joints {
                let Some(fb) = drv.commissioning_raw_feedback(j) else {
                    return Err(if is_zero {
                        (409, "MOTOR_ZERO_NO_FEEDBACK", "the motor feedback is not fresh; check the bus and the motor power")
                    } else {
                        (502, "MIT_TIMEOUT_FAILED", "the motor feedback is not fresh; check the bus and the motor power")
                    });
                };
                let age = drv_now.saturating_duration_since(fb.rx_at);
                if age > crate::driver::ZERO_FEEDBACK_MAX_AGE
                    || (drv.capabilities().enable_feedback && fb.motor_status != 0)
                {
                    return Err(if is_zero {
                        (409, "MOTOR_ZERO_NOT_DISABLED", "joint is not confirmed disabled from fresh feedback")
                    } else {
                        (502, "MIT_TIMEOUT_FAILED", "joint is not confirmed disabled from fresh feedback")
                    });
                }
            }
        }
        // Fresh rest evidence: ALWAYS required before EVERY irreversible hardware write
        // (fail-closed; no relaxation for `force`).
        // B4: During a multi-joint `MotorZeroWriteVerify` batch, once >=1 joint already received
        // `CMD_SET_ZERO` (whose raw motor reading jumped to ~0 while `ZeroOffsetDriver` still
        // holds the old offsets until atomic commit per M2), evaluate fresh whole-arm rest-pose
        // evidence across EVERY arm joint in its exact coordinate frame (raw `0.0` for already-
        // zeroed joints, `raw - old_offset` vs `rest_ref` for unzeroed joints).
        if kind == CommissioningOpKind::MotorZeroWriteVerify
            && self.commissioning.zero_sent_joint_mask != 0
        {
            self.verify_mid_zero_batch_rest_pose_evidence(
                drv,
                self.commissioning.zero_sent_joint_mask,
            )?;
        } else {
            let gate_status = drv.rest_pose_gate_status();
            let rest_ok = if can_gather {
                // While disarmed and idle, only 1-slot RefreshScheduler runs so rest evidence may
                // be PendingDrainOrWindow / StaleOrMissing until GatherRestEvidence polls 0xCC.
                // RefusedNonRest (no resting pose or observed off-rest) still refuses immediately.
                gate_status != crate::driver::RestPoseGateStatus::RefusedNonRest
            } else {
                gate_status == crate::driver::RestPoseGateStatus::Confirmed
            };
            if !rest_ok {
                return Err(if is_zero {
                    (409, "ARM_NOT_AT_RESTING_POSE", "hardware motor zero requires fresh confirmed rest-pose evidence")
                } else {
                    (409, "ARM_NOT_AT_RESTING_POSE", "TIMEOUT commissioning requires fresh confirmed rest-pose evidence")
                });
            }
        }
        Ok(())
    }

    /// B4: Zero-allocation fresh whole-arm rest-pose verification before every subsequent
    /// `CMD_SET_ZERO` in a multi-joint batch (using raw `0.0` for joints already zeroed in this
    /// batch and `raw - joint_zero_offsets[i]` vs `rest_ref[i]` for joints not yet zeroed).
    fn verify_mid_zero_batch_rest_pose_evidence(
        &self,
        drv: &dyn crate::driver::ArmDriver,
        zero_sent_joint_mask: u64,
    ) -> Result<(), (u16, &'static str, &'static str)> {
        let Some(rp) = self.config.resting_pose.as_ref() else {
            return Err((
                409,
                "ARM_NOT_AT_RESTING_POSE",
                "hardware motor zero requires a configured resting pose",
            ));
        };
        if !drv.is_feedback_valid() {
            return Err((
                409,
                "ARM_NOT_AT_RESTING_POSE",
                "fresh whole-arm feedback lost during motor zero batch",
            ));
        }
        let rest_ref: &[f32] = rp
            .positions
            .as_deref()
            .unwrap_or(self.config.home_positions.as_slice());
        let gripper_idx = self.config.gripper_index();
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        let now = drv.clock_now();
        let max_age = crate::driver::STOPPED_DISABLE_FEEDBACK_MAX_AGE;
        let enable_fb = drv.capabilities().enable_feedback;
        let mut arm_count = 0usize;
        let mut total_dev = 0.0f32;
        for i in 0..dof {
            if gripper_idx == Some(i) {
                continue;
            }
            arm_count += 1;
            let Some(fb) = drv.commissioning_raw_feedback(i) else {
                return Err((
                    409,
                    "ARM_NOT_AT_RESTING_POSE",
                    "missing fresh feedback for arm joint during motor zero batch",
                ));
            };
            if now.saturating_duration_since(fb.rx_at) > max_age {
                return Err((
                    409,
                    "ARM_NOT_AT_RESTING_POSE",
                    "arm joint feedback exceeded rest-pose freshness bound during motor zero batch",
                ));
            }
            if !fb
                .queue_bound
                .is_some_and(|b| now.saturating_duration_since(b) <= max_age)
            {
                return Err((
                    409,
                    "ARM_NOT_AT_RESTING_POSE",
                    "arm joint receive-queue bound exceeded rest-pose freshness window during motor zero batch",
                ));
            }
            if !fb.raw_position.is_finite() || !fb.velocity.is_finite() {
                return Err((
                    409,
                    "ARM_NOT_AT_RESTING_POSE",
                    "non-finite feedback during motor zero batch",
                ));
            }
            if enable_fb && fb.motor_status != 0 {
                return Err((
                    409,
                    "MOTOR_ZERO_NOT_DISABLED",
                    "arm joint no longer confirmed disabled during motor zero batch",
                ));
            }
            let vlim = rp.max_velocity_of(i).min(COMM_STATIONARY_VEL_RAD_S);
            if fb.velocity.abs() > vlim {
                return Err((
                    409,
                    "ARM_NOT_AT_RESTING_POSE",
                    "arm moved during motor zero batch",
                ));
            }
            let dev = if (zero_sent_joint_mask & (1u64 << i)) != 0 {
                // Already zeroed in this batch: motor raw position must stay near 0.0.
                fb.raw_position.abs()
            } else {
                let Some(&target_rest) = rest_ref.get(i) else {
                    return Err((
                        409,
                        "ARM_NOT_AT_RESTING_POSE",
                        "missing resting reference for arm joint",
                    ));
                };
                let offset = self.joint_zero_offsets.get(i).copied().unwrap_or(0.0);
                ((fb.raw_position - offset) - target_rest).abs()
            };
            let tol = self.config.rest_pose_tolerance_rad;
            if !dev.is_finite() || dev > tol {
                return Err((
                    409,
                    "ARM_NOT_AT_RESTING_POSE",
                    "arm left resting pose during motor zero batch",
                ));
            }
            total_dev += dev;
        }
        if arm_count == 0 || !total_dev.is_finite() || total_dev > rp.max_total_deviation {
            return Err((
                409,
                "ARM_NOT_AT_RESTING_POSE",
                "whole-arm deviation exceeded resting pose limit during motor zero batch",
            ));
        }
        Ok(())
    }

    /// Starts one tick-driven commissioning operation on the exclusive RT owner (zero alloc).
    #[allow(clippy::result_large_err)]
    pub(crate) fn start_commissioning_rt(
        &mut self,
        req: CommissioningStartRequest,
        owner_now: std::time::Instant,
    ) -> Result<u64, crate::rt_core::requests::config::ConfigError> {
        use crate::rt_core::requests::config::ConfigError;
        use crate::rt_core::session::{
            PreparedSession, SessionAuthority, SessionKind, SessionSlot,
        };
        if self.commissioning.is_occupied()
            || self.sessions.is_slot_occupied(SessionSlot::ExclusiveOutput)
        {
            return Err(ConfigError::Busy("a hardware commissioning operation is already running"));
        }
        if req.joint_len == 0 || req.joint_len > crate::rt_core::mailbox::MAX_DOF {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "invalid commissioning joint list",
            });
        }
        if let Err((status, code, message)) =
            self.commissioning_live_authority_ok(req.kind, &req.authority, owner_now)
        {
            return Err(match code {
                "LEASE_LOST" => ConfigError::LeaseLost,
                "EPOCH_MISMATCH" => ConfigError::EpochMismatch {
                    current_epoch: self.safety.epoch_id(),
                },
                "STOPPED" => ConfigError::Stopped,
                _ => ConfigError::CalibPrecondition { status, code, message },
            });
        }
        if req.kind == CommissioningOpKind::WriteTimeout
            && (req.timeout_counts == 0 || req.timeout_counts > MAX_TIMEOUT_COUNTS)
        {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "timeout_counts out of range",
            });
        }
        let supports_mit_switch = match self.driver.as_ref() {
            Some(slot) => {
                let mut check_res = Ok(());
                let mut mit_sw = false;
                let locked = slot.try_with_ref(|drv| {
                    mit_sw = drv.supports_mit_mode_switch();
                    check_res = self.commissioning_safety_check_with_driver_inner(
                        req.kind,
                        &req.joints[..req.joint_len],
                        Some(drv),
                        req.kind == CommissioningOpKind::WriteTimeout,
                    );
                });
                if locked.is_err() {
                    return Err(ConfigError::CalibPrecondition {
                        status: 500,
                        code: "DRIVER",
                        message: "driver unavailable",
                    });
                }
                if let Err((status, code, message)) = check_res {
                    return Err(ConfigError::CalibPrecondition { status, code, message });
                }
                mit_sw
            }
            None => {
                if let Err((status, code, message)) = self.commissioning_safety_check_with_driver(
                    req.kind,
                    &req.joints[..req.joint_len],
                    None,
                ) {
                    return Err(ConfigError::CalibPrecondition { status, code, message });
                }
                false
            }
        };
        let mut prepared = PreparedSession::exclusive(
            SessionKind::Commissioning,
            crate::rt_core::broker::ClientId(req.authority.conn_id),
        );
        prepared.deadline = Some(req.txn_deadline);
        prepared.timeout_code = Self::commissioning_deadline_error_code(req.kind);
        prepared.operator_required = self.operator_auth_enforced;
        prepared.operator_grant = Some(req.authority.operator_grant);
        prepared.revoke_on_stop = true;
        prepared.escalate_safe_hold_on_uncertain = true;
        let session_auth = SessionAuthority {
            baseline: self.capture_baseline(),
            connection_id: req.authority.conn_id,
            lease_generation: req.authority.lease_holder.map_or(0, |(_, gen)| gen),
            epoch: req.authority.expected_epoch,
            stop_generation: req.authority.expected_stop_gen,
            require_lease: self.lease_enabled,
            operator_required: self.operator_auth_enforced,
            operator_grant: Some(req.authority.operator_grant),
            revoke_on_stop: true,
            revoke_on_epoch_change: true,
        };
        let session_id = self
            .sessions
            .admit_with_authority(prepared, session_auth, owner_now, false)
            .map_err(|_| {
                ConfigError::Busy("a hardware commissioning operation is already running")
            })?;
        let op_id = self.commissioning.next_op_id;
        self.commissioning.next_op_id = op_id.wrapping_add(1).max(1);
        self.commissioning.active_op_id = op_id;
        self.commissioning.session_id = Some(session_id);
        self.commissioning.txn_id = req.authority.txn_id;
        self.commissioning.kind = req.kind;
        self.commissioning.state = CommissioningOpState::Running;
        self.commissioning.joints = req.joints;
        self.commissioning.joint_len = req.joint_len;
        self.commissioning.timeout_counts = req.timeout_counts;
        self.commissioning.references = req.references;
        self.commissioning.authority = Some(req.authority);
        self.commissioning.txn_deadline = Some(req.txn_deadline);
        self.commissioning.hw_mutated = false;
        self.commissioning.disk_mutated = req.prior_disk_mutated;
        self.commissioning.supports_mit_switch = supports_mit_switch;
        self.commissioning.error_status = 0;
        self.commissioning.error_code = "";
        self.commissioning.error_msg = crate::safety::BoundedText::empty();
        for i in 0..crate::rt_core::mailbox::MAX_DOF {
            self.commissioning.outcomes[i] = CommissioningJointOutcome {
                joint: if i < req.joint_len { req.joints[i] } else { 0 },
                ..CommissioningJointOutcome::default()
            };
        }
        self.commissioning.joint_idx = 0;
        self.commissioning.reset_reg_read();
        self.commissioning.write_phase = WriteTimeoutPhase::GatherRestEvidence;
        self.commissioning.reset_rest_gather(WriteTimeoutPhase::ReadBefore);
        self.commissioning.mz_phase = match req.kind {
            CommissioningOpKind::MotorZeroDisable => MotorZeroPhase::DisableDispatch,
            CommissioningOpKind::MotorZeroWriteVerify => MotorZeroPhase::ZeroSendLoop,
            _ => MotorZeroPhase::DisableDispatch,
        };
        self.commissioning.disable_started_at = None;
        self.commissioning.disable_boundary_seq = 0;
        self.commissioning.disable_tx_started = None;
        self.commissioning.disable_deadline = None;
        self.commissioning.next_zero_send_at = None;
        self.commissioning.zero_boundary_seq = [0; crate::rt_core::mailbox::MAX_DOF];
        self.commissioning.zero_tx_started = [None; crate::rt_core::mailbox::MAX_DOF];
        self.commissioning.zero_sent_mask = 0;
        self.commissioning.zero_sent_joint_mask = 0;
        self.commissioning.zero_verify_deadline = None;
        Ok(op_id)
    }

    pub(crate) fn poll_commissioning_rt(&self, op_id: u64) -> Option<CommissioningSnapshot> {
        self.commissioning.snapshot(op_id)
    }

    /// Cancels an active commissioning operation matching `op_id` (stale ids are ignored).
    /// If any hardware or durable disk mutation already occurred, immediately establishes
    /// `HoldState::Reconciling` so motion and rearm remain fenced.
    pub(crate) fn cancel_commissioning_rt(
        &mut self,
        op_id: u64,
        reason: &'static str,
    ) -> Option<CommissioningSnapshot> {
        if self.commissioning.active_op_id != op_id || self.commissioning.state == CommissioningOpState::Idle {
            return None;
        }
        if self.commissioning.state == CommissioningOpState::Running {
            let drv_now = self.driver_clock_now_or(std::time::Instant::now());
            self.commissioning.mark_quiet_if_in_flight(drv_now);
            let (status, code) = match reason {
                "STOPPED" => (409, "STOPPED"),
                "ESTOP_ACTIVE" | "EPOCH_MISMATCH" => (409, "EPOCH_MISMATCH"),
                "LEASE_LOST" => (409, "LEASE_LOST"),
                "OPERATOR_REQUIRED" => (403, "OPERATOR_REQUIRED"),
                _ => (409, "CANCELLED"),
            };
            self.commissioning
                .set_error(CommissioningOpState::Cancelled, status, code, reason);
            if self.commissioning.hw_mutated || self.commissioning.disk_mutated {
                let active_rev = crate::rt_core::config_iface::ConfigAuthority::revision(
                    self,
                    crate::rt_core::config_iface::ConfigDomain::Calibration,
                );
                self.core
                    .config_mut()
                    .force_reconcile(crate::rt_core::config_iface::ConfigDomain::Calibration, active_rev);
            }
            self.sync_commissioning_session_terminal();
        }
        self.commissioning.snapshot(op_id)
    }

    /// Clears a terminal commissioning slot owned by `op_id` once its off-thread caller has
    /// read the snapshot. When `force_reconcile` is true (or `hw_mutated && !clean_complete`),
    /// forces `HoldState::Reconciling` atomically before freeing the slot.
    pub(crate) fn clear_commissioning_rt(
        &mut self,
        op_id: u64,
        clean_complete: bool,
        force_reconcile: bool,
    ) -> bool {
        if self.commissioning.active_op_id != op_id || self.commissioning.state == CommissioningOpState::Idle {
            return false;
        }
        if self.commissioning.state == CommissioningOpState::Running {
            let drv_now = self.driver_clock_now_or(std::time::Instant::now());
            self.commissioning.mark_quiet_if_in_flight(drv_now);
            if let Some(sid) = self.commissioning.session_id {
                let uncertain = self.commissioning.has_uncertain_outcome()
                    || self.commissioning.hw_mutated
                    || self.commissioning.disk_mutated;
                let _ = self.sessions.cancel_slot(
                    crate::rt_core::session::SessionSlot::ExclusiveOutput,
                    sid,
                    crate::rt_core::session::SessionTerminalReason::Cancelled("CANCELLED"),
                    uncertain,
                );
            }
        }
        let must_reconcile = force_reconcile
            || (!clean_complete && (self.commissioning.hw_mutated || self.commissioning.disk_mutated));
        if must_reconcile {
            let active_rev = crate::rt_core::config_iface::ConfigAuthority::revision(
                self,
                crate::rt_core::config_iface::ConfigDomain::Calibration,
            );
            self.core
                .config_mut()
                .force_reconcile(crate::rt_core::config_iface::ConfigDomain::Calibration, active_rev);
        }
        if let Some(sid) = self.commissioning.session_id.take() {
            let _ = self
                .sessions
                .clear_terminal_slot(crate::rt_core::session::SessionSlot::ExclusiveOutput, sid);
        }
        self.commissioning.state = CommissioningOpState::Idle;
        self.commissioning.authority = None;
        self.commissioning.txn_deadline = None;
        self.commissioning.hw_mutated = false;
        self.commissioning.disk_mutated = false;
        true
    }

    /// Immediate owner-side cancellation hook invoked by urgent Stop, E-stop, Disconnect/Lease
    /// eviction, or `CommitShutdown`. Cancels any running commissioning op before another TX
    /// can be attempted, and forces `HoldState::Reconciling` if hardware or disk was mutated.
    pub(crate) fn cancel_active_commissioning_on_stop(&mut self, reason: &'static str) {
        if let Some(op_id) = self.commissioning.active_op_id() {
            if self.commissioning.state == CommissioningOpState::Running {
                let _ = self.cancel_commissioning_rt(op_id, reason);
            }
        }
    }

    /// Advances the single preallocated commissioning state machine by one bounded non-blocking
    /// step inside Step 2b of a control release (strictly under `release_deadline - publish_margin`,
    /// sharing Step 2's RX/TX pass budgets; 0 heap allocs and 0 heap frees on `armd-control`).
    pub(crate) fn service_commissioning_release(
        &mut self,
        ctx: &crate::rt_core::ops::CommitCtx,
        release_deadline: std::time::Instant,
        owner_now_fn: &dyn Fn() -> std::time::Instant,
    ) {
        if self.commissioning.state != CommissioningOpState::Running {
            return;
        }
        let owner_now = owner_now_fn();
        let total_span = release_deadline.saturating_duration_since(owner_now);
        let publish_margin = self.tx_release_publish_margin().min(total_span / 2);
        let owner_cutoff = release_deadline
            .checked_sub(publish_margin)
            .unwrap_or(owner_now);
        let mut now_mut = || owner_now_fn();
        let _ = self.service_commissioning_step_with_cutoff(
            ctx,
            owner_cutoff,
            COMM_MAX_TX_PER_RELEASE,
            None,
            true,
            &mut now_mut,
        );
    }

    /// Advances the single preallocated commissioning state machine by one bounded non-blocking
    /// step using an already-computed `owner_cutoff` (no second `publish_margin` subtraction)
    /// and explicit per-step TX bounds (`max_tx` for register I/O, optional `probe_tx_cap` for
    /// `GatherRestEvidence` `0xCC` bursts). Only increments `release_step_count` when
    /// `is_new_release` is true so multi-guard legacy post-tick stepping shares one release index.
    pub(crate) fn service_commissioning_step_with_cutoff(
        &mut self,
        ctx: &crate::rt_core::ops::CommitCtx,
        mut owner_cutoff: std::time::Instant,
        max_tx: u8,
        probe_tx_cap: Option<u8>,
        is_new_release: bool,
        owner_now_fn: &mut dyn FnMut() -> std::time::Instant,
    ) -> CommissioningStepProgress {
        if self.commissioning.state != CommissioningOpState::Running {
            return CommissioningStepProgress::default();
        }
        if is_new_release {
            self.commissioning.release_step_count =
                self.commissioning.release_step_count.wrapping_add(1).max(1);
        }
        let Some(auth) = self.commissioning.authority else {
            self.commissioning.set_error(
                CommissioningOpState::Failed,
                500,
                "INTERNAL",
                "missing commissioning authority",
            );
            return CommissioningStepProgress::default();
        };
        let owner_now = owner_now_fn();
        let kind = self.commissioning.kind;
        if let Err((status, code, msg)) = self.commissioning_live_authority_ok(kind, &auth, owner_now) {
            self.abort_running_commissioning(status, code, msg);
            return CommissioningStepProgress::default();
        }
        if let Err((status, code, msg)) = self.commissioning_txn_deadline_ok(kind, owner_now) {
            self.abort_running_commissioning(status, code, msg);
            return CommissioningStepProgress::default();
        }

        // Clamp caller-derived cutoff through the session lifecycle driver (session + child
        // deadlines) as well as the commissioning transaction deadline (no second margin subtraction).
        owner_cutoff = self
            .sessions
            .effective_tx_cutoff(crate::rt_core::session::SessionSlot::ExclusiveOutput, owner_cutoff);
        if let Some(txn_dl) = self.commissioning.txn_deadline {
            owner_cutoff = owner_cutoff.min(txn_dl);
        }
        if owner_now_fn() >= owner_cutoff {
            if let Err((status, code, msg)) =
                self.commissioning_txn_deadline_ok(kind, owner_now_fn())
            {
                self.abort_running_commissioning(status, code, msg);
            }
            return CommissioningStepProgress {
                still_running: self.commissioning.state == CommissioningOpState::Running,
                ..CommissioningStepProgress::default()
            };
        }
        let remaining_rx = crate::rt_core::ops::RX_FRAME_BUDGET
            .saturating_sub(self.pending_rx_poll.rx_frames)
            .min(16);

        let kind = self.commissioning.kind;
        let joint_len = self.commissioning.joint_len;
        let joints = self.commissioning.joints;

        // Driverless mock follower `MotorZeroWriteVerify`: no CAN bus needed.
        if self.driver.is_none() {
            if kind == CommissioningOpKind::MotorZeroDisable {
                self.commissioning.state = CommissioningOpState::Done;
                self.sync_commissioning_session_terminal();
                return CommissioningStepProgress::default();
            }
            if kind == CommissioningOpKind::MotorZeroWriteVerify {
                if let Err((status, code, msg)) =
                    self.commissioning_safety_check_with_driver(kind, &joints[..joint_len], None)
                {
                    self.abort_running_commissioning(status, code, msg);
                    return CommissioningStepProgress::default();
                }
                let now = owner_now_fn();
                for k in 0..joint_len {
                    if let Err((status, code, msg)) =
                        self.commissioning_live_authority_ok(kind, &auth, owner_now_fn())
                    {
                        self.abort_running_commissioning(status, code, msg);
                        return CommissioningStepProgress::default();
                    }
                    if k > 0 {
                        if let Some(next_at) = self.commissioning.next_zero_send_at {
                            if now < next_at {
                                return CommissioningStepProgress {
                                    still_running: true,
                                    ..CommissioningStepProgress::default()
                                };
                            }
                        }
                    }
                    if self.commissioning.joint_idx > k {
                        continue;
                    }
                    let j = joints[k];
                    self.commissioning.hw_mutated = true;
                    self.commissioning.outcomes[k].disposition = JointCommissioningDisposition::Applied;
                    self.commissioning.outcomes[k].raw_after = 0.0;
                    self.commissioning.outcomes[k].joint_after = self.commissioning.references[j];
                    self.commissioning.outcomes[k].fresh = true;
                    self.commissioning.outcomes[k].verified = true;
                    self.commissioning.joint_idx = k + 1;
                    self.commissioning.next_zero_send_at = Some(now + crate::motor_zero::COMMAND_SPACING);
                    return CommissioningStepProgress {
                        still_running: true,
                        ..CommissioningStepProgress::default()
                    };
                }
                if self.commissioning.joint_idx >= joint_len {
                    self.commissioning.state = CommissioningOpState::Done;
                    self.sync_commissioning_session_terminal();
                }
                return CommissioningStepProgress {
                    still_running: self.commissioning.state == CommissioningOpState::Running,
                    ..CommissioningStepProgress::default()
                };
            }
            self.abort_running_commissioning(400, "UNSUPPORTED", "no motor driver");
            return CommissioningStepProgress::default();
        }

        // Temporarily take driver out so the live submission gate closure can borrow `&*self`
        // right before `can.transmit` (closing B2 with zero allocation). Wrap in `catch_unwind`
        // so ANY panic in `drv.clock_now()` or CAN I/O unconditionally restores `self.driver`
        // before unwinding (never dropping the sole driver or leaving `self.driver == None`).
        // Supports both `DriverSlot::Exclusive` (RT executor) and `DriverSlot::Legacy`
        // (non-blocking `try_lock`, yielding on contention so write-guard hold stays <= 2 ms).
        #[allow(clippy::large_enum_variant)] // RT zero-alloc path; stack-only enum
        enum DriverCommStep {
            Ran(
                Result<
                    (),
                    (
                        u16,
                        &'static str,
                        crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
                    ),
                >,
            ),
            Contended,
            Unavailable,
        }
        let mut reg_tx_attempted = 0u8;
        let mut probe_tx_attempted = 0u8;
        let mut backpressured = false;
        let mut taken = self.driver.take();
        let step_outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            #[allow(clippy::result_large_err)] // RT zero-alloc path; BoundedText stays inline
            let mut run_with_drv = |drv: &mut dyn crate::driver::ArmDriver| {
                let drv_now = drv.clock_now();
                let span = owner_cutoff.saturating_duration_since(owner_now_fn());
                let drv_cutoff = drv_now + span;
                drv.set_tx_release_deadline(Some(drv_cutoff));

                let res = match kind {
                    CommissioningOpKind::ReadIdentity => self.step_read_identity_with_drv(
                        drv,
                        &auth,
                        drv_cutoff,
                        remaining_rx,
                        max_tx,
                        &mut reg_tx_attempted,
                        &mut backpressured,
                        owner_now_fn,
                    ),
                    CommissioningOpKind::WriteTimeout => self.step_write_timeout_with_drv(
                        drv,
                        &auth,
                        drv_cutoff,
                        remaining_rx,
                        max_tx,
                        probe_tx_cap,
                        &mut reg_tx_attempted,
                        &mut probe_tx_attempted,
                        &mut backpressured,
                        owner_now_fn,
                    ),
                    CommissioningOpKind::MotorZeroDisable => {
                        self.step_motor_zero_disable_with_drv(drv, &auth, drv_cutoff, remaining_rx, owner_now_fn)
                    }
                    CommissioningOpKind::MotorZeroWriteVerify => {
                        self.step_motor_zero_write_verify_with_drv(
                            drv,
                            &auth,
                            drv_cutoff,
                            remaining_rx,
                            &mut reg_tx_attempted,
                            &mut backpressured,
                            owner_now_fn,
                        )
                    }
                };

                drv.set_tx_release_deadline(None);
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow.drain_from_driver(drv);
                res
            };
            match taken.as_mut() {
                Some(slot) => {
                    if let Some(drv) = slot.as_exclusive_mut() {
                        DriverCommStep::Ran(run_with_drv(drv))
                    } else if let Some(lock) = slot.as_legacy_lock() {
                        match lock.try_lock() {
                            Ok(mut guard) => DriverCommStep::Ran(run_with_drv(&mut **guard)),
                            Err(std::sync::TryLockError::WouldBlock) => DriverCommStep::Contended,
                            Err(std::sync::TryLockError::Poisoned(_)) => DriverCommStep::Unavailable,
                        }
                    } else {
                        DriverCommStep::Unavailable
                    }
                }
                None => DriverCommStep::Unavailable,
            }
        }));
        self.driver = taken;
        let step_result = match step_outcome {
            Ok(r) => r,
            Err(payload) => std::panic::resume_unwind(payload),
        };
        let _ = ctx;
        let mut contended = false;
        match step_result {
            DriverCommStep::Ran(Ok(())) => {}
            DriverCommStep::Contended => {
                contended = true;
            }
            DriverCommStep::Ran(Err((status, code, msg_buf))) => {
                self.abort_running_commissioning_bounded(status, code, msg_buf);
            }
            DriverCommStep::Unavailable => {
                self.abort_running_commissioning(500, "DRIVER", "driver unavailable");
            }
        }
        let still_running = self.commissioning.state == CommissioningOpState::Running;
        if !still_running {
            self.sync_commissioning_session_terminal();
        }
        let rest_probe_pending = still_running
            && kind == CommissioningOpKind::WriteTimeout
            && self.commissioning.write_phase == WriteTimeoutPhase::GatherRestEvidence
            && self.commissioning.rest_probe_pending_mask != 0;
        let awaiting_reply = still_running && self.commissioning.req_in_flight;
        let quiet_active = self.commissioning.unmatched_reply_quiet_until.is_some();
        let more_immediate_work = still_running
            && !contended
            && !backpressured
            && !awaiting_reply
            && !quiet_active
            && (rest_probe_pending
                || (kind == CommissioningOpKind::ReadIdentity
                    && !matches!(self.commissioning.reg_phase, RegReadPhase::QuietWait))
                || (kind == CommissioningOpKind::WriteTimeout
                    && self.commissioning.write_phase != WriteTimeoutPhase::GatherRestEvidence));
        CommissioningStepProgress {
            still_running,
            reg_tx_attempted,
            probe_tx_attempted,
            awaiting_reply,
            rest_probe_pending,
            more_immediate_work,
            backpressured,
            contended,
        }
    }

    fn commissioning_deadline_error_code(kind: CommissioningOpKind) -> &'static str {
        match kind {
            CommissioningOpKind::ReadIdentity => "MIT_TIMEOUT_READ",
            CommissioningOpKind::WriteTimeout => "MIT_TIMEOUT_FAILED",
            CommissioningOpKind::MotorZeroDisable => "MOTOR_ZERO_NOT_DISABLED",
            CommissioningOpKind::MotorZeroWriteVerify => "MOTOR_ZERO_UNVERIFIED",
        }
    }

    fn commissioning_txn_deadline_ok(
        &self,
        kind: CommissioningOpKind,
        owner_now: std::time::Instant,
    ) -> Result<(), (u16, &'static str, &'static str)> {
        if self
            .commissioning
            .txn_deadline
            .is_some_and(|dl| owner_now >= dl)
        {
            return Err((
                502,
                Self::commissioning_deadline_error_code(kind),
                "commissioning transaction deadline expired",
            ));
        }
        Ok(())
    }

    /// Combined terminal pre-TX gate re-checking BOTH live authority (operator, lease, epoch,
    /// stop, reservation, durable marker binding) AND `txn_deadline` immediately before each TX.
    fn commissioning_pre_tx_gate_ok(
        &self,
        kind: CommissioningOpKind,
        auth: &CommissioningAuthority,
        owner_now: std::time::Instant,
    ) -> Result<(), &'static str> {
        self.commissioning_live_authority_ok(kind, auth, owner_now)
            .map_err(|(_, code, _)| code)?;
        if self
            .commissioning
            .txn_deadline
            .is_some_and(|dl| owner_now >= dl)
        {
            return Err("TXN_DEADLINE_EXPIRED");
        }
        Ok(())
    }

    fn map_commissioning_revoked(
        kind: CommissioningOpKind,
        code: &'static str,
        revoked_msg: &'static str,
    ) -> (u16, &'static str, crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>) {
        if code == "TXN_DEADLINE_EXPIRED" {
            return (
                502,
                Self::commissioning_deadline_error_code(kind),
                crate::safety::BoundedText::new("commissioning transaction deadline expired"),
            );
        }
        let status = if code == "OPERATOR_REQUIRED" { 403 } else { 409 };
        (status, code, crate::safety::BoundedText::new(revoked_msg))
    }

    fn abort_running_commissioning(&mut self, status: u16, code: &'static str, msg: &str) {
        self.abort_running_commissioning_bounded(status, code, crate::safety::BoundedText::new(msg));
    }

    fn abort_running_commissioning_bounded(
        &mut self,
        status: u16,
        code: &'static str,
        msg: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
    ) {
        let drv_now = self.driver_clock_now_or(std::time::Instant::now());
        self.commissioning.mark_quiet_if_in_flight(drv_now);
        let state = if matches!(code, "STOPPED" | "EPOCH_MISMATCH" | "LEASE_LOST" | "OPERATOR_REQUIRED") {
            CommissioningOpState::Cancelled
        } else {
            CommissioningOpState::Failed
        };
        self.commissioning.state = state;
        self.commissioning.error_status = status;
        self.commissioning.error_code = code;
        self.commissioning.error_msg = msg;
        if self.commissioning.hw_mutated || self.commissioning.disk_mutated {
            let active_rev = crate::rt_core::config_iface::ConfigAuthority::revision(
                self,
                crate::rt_core::config_iface::ConfigDomain::Calibration,
            );
            self.core
                .config_mut()
                .force_reconcile(crate::rt_core::config_iface::ConfigDomain::Calibration, active_rev);
        }
        self.sync_commissioning_session_terminal();
    }

    fn sync_commissioning_session_terminal(&mut self) {
        use crate::rt_core::session::{SessionSlot, SessionTerminalReason};
        let Some(sid) = self.commissioning.session_id else {
            return;
        };
        if self
            .sessions
            .slot(SessionSlot::ExclusiveOutput)
            .is_some_and(|s| s.id == sid && !s.is_live())
        {
            return;
        }
        let reason = match self.commissioning.state {
            CommissioningOpState::Done => SessionTerminalReason::Completed,
            CommissioningOpState::Cancelled => {
                SessionTerminalReason::Cancelled(if self.commissioning.error_code.is_empty() {
                    "CANCELLED"
                } else {
                    self.commissioning.error_code
                })
            }
            CommissioningOpState::Failed => {
                let code = if self.commissioning.error_code.is_empty() {
                    "FAILED"
                } else {
                    self.commissioning.error_code
                };
                if code == Self::commissioning_deadline_error_code(self.commissioning.kind)
                    && self.commissioning.error_status == 502
                {
                    SessionTerminalReason::DeadlineExpired(code)
                } else {
                    SessionTerminalReason::Failed(code)
                }
            }
            CommissioningOpState::Idle | CommissioningOpState::Running => return,
        };
        let uncertain = self.commissioning.has_uncertain_outcome()
            || (self.commissioning.state != CommissioningOpState::Done
                && (self.commissioning.hw_mutated || self.commissioning.disk_mutated));
        if let Some(outcome) = self
            .sessions
            .finish_slot(SessionSlot::ExclusiveOutput, sid, reason, uncertain)
        {
            self.maybe_escalate_session_safe_hold(outcome, true);
        }
    }

    /// Drives one joint's 8-register identity read (`COMM_IDENTITY_RIDS`), sending ONE register
    /// read at a time with a fresh post-drain submission mark `(tx_seq, tx_started, tx_offset_ns)`
    /// per accepted request so a delayed reply from an earlier request or retry can NEVER match.
    #[allow(clippy::result_large_err)]
    fn step_one_identity_read(
        &mut self,
        drv: &mut dyn crate::driver::ArmDriver,
        joint: usize,
        auth: &CommissioningAuthority,
        drv_cutoff: std::time::Instant,
        remaining_rx: &mut u16,
        tx_budget_left: &mut u8,
        reg_tx_attempted: &mut u8,
        backpressured: &mut bool,
        owner_now_fn: &mut dyn FnMut() -> std::time::Instant,
    ) -> Result<Option<CommissioningIdentity>, (u16, &'static str, crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>)> {
        let Some(motor_id) = drv.commissioning_motor_id(joint) else {
            return Err((
                502,
                "MIT_TIMEOUT_READ",
                crate::safety::BoundedText::new("joint index out of driver range"),
            ));
        };
        loop {
            if let Err((status, code, msg)) =
                self.commissioning_live_authority_ok(self.commissioning.kind, auth, owner_now_fn())
            {
                return Err((status, code, crate::safety::BoundedText::new(msg)));
            }
            if let Err((status, code, msg)) =
                self.commissioning_txn_deadline_ok(self.commissioning.kind, owner_now_fn())
            {
                return Err((status, code, crate::safety::BoundedText::new(msg)));
            }
            if drv.clock_now() >= drv_cutoff {
                return Ok(None);
            }
            match self.commissioning.reg_phase {
                RegReadPhase::QuietWait => {
                    if let Some(until) = self.commissioning.unmatched_reply_quiet_until {
                        if drv.clock_now() < until {
                            return Ok(None);
                        }
                        self.commissioning.unmatched_reply_quiet_until = None;
                    }
                    self.commissioning.reg_phase = RegReadPhase::DrainBeforeReq;
                }
                RegReadPhase::DrainBeforeReq => {
                    if *remaining_rx == 0 {
                        return Ok(None);
                    }
                    match drv.commissioning_drain_rx_step(*remaining_rx, drv_cutoff) {
                        Ok((frames, drained)) => {
                            *remaining_rx = remaining_rx.saturating_sub(frames);
                            if !drained {
                                return Ok(None);
                            }
                            drv.commissioning_clear_register_log();
                            self.commissioning.req_attempts = 0;
                            self.commissioning.reg_phase = RegReadPhase::SendReq;
                        }
                        Err(_) => {
                            return Err((
                                502,
                                "MIT_TIMEOUT_READ",
                                crate::safety::BoundedText::new("receive error while draining CAN RX"),
                            ));
                        }
                    }
                }
                RegReadPhase::SendReq => {
                    if *tx_budget_left == 0 {
                        return Ok(None);
                    }
                    let rid = COMM_IDENTITY_RIDS[self.commissioning.rid_idx];
                    let op_kind = self.commissioning.kind;
                    let mut gate = || -> Result<(), &'static str> {
                        self.commissioning_pre_tx_gate_ok(op_kind, auth, owner_now_fn())
                    };
                    *tx_budget_left = tx_budget_left.saturating_sub(1);
                    *reg_tx_attempted = reg_tx_attempted.saturating_add(1);
                    match drv.commissioning_try_send_register_read(joint, rid, drv_cutoff, &mut gate) {
                        CommissioningTxStatus::Sent {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        } => {
                            self.commissioning.req_tx_seq = tx_seq;
                            self.commissioning.req_tx_started = Some(tx_started);
                            self.commissioning.req_tx_offset_ns = tx_offset_ns;
                            self.commissioning.req_deadline = Some(drv.clock_now() + COMM_REG_REPLY_TIMEOUT);
                            self.commissioning.req_in_flight = true;
                            self.commissioning.reg_phase = RegReadPhase::AwaitReply;
                        }
                        CommissioningTxStatus::SentFlushFailed { .. } => {
                            self.commissioning.req_in_flight = true;
                            self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                            return Err((
                                502,
                                "MIT_TIMEOUT_READ",
                                crate::safety::BoundedText::new("CAN flush failed during register read"),
                            ));
                        }
                        CommissioningTxStatus::Backpressure => {
                            *backpressured = true;
                            self.commissioning.req_attempts = self.commissioning.req_attempts.saturating_add(1);
                            if self.commissioning.req_attempts >= COMM_MAX_FRAME_ATTEMPTS {
                                return Err((
                                    502,
                                    "MIT_TIMEOUT_READ",
                                    crate::safety::BoundedText::new("CAN TX backpressure exhausted during register read"),
                                ));
                            }
                            // Re-drain before the next retry so any delayed frame arriving during
                            // backpressure cannot precede the retry's submission mark.
                            self.commissioning.reg_phase = RegReadPhase::DrainBeforeReq;
                            return Ok(None);
                        }
                        CommissioningTxStatus::Revoked(code) => {
                            return Err(Self::map_commissioning_revoked(
                                op_kind,
                                code,
                                "authority revoked before register read",
                            ));
                        }
                        CommissioningTxStatus::Refused(msg) => {
                            return Err((502, "MIT_TIMEOUT_READ", crate::safety::BoundedText::new(msg)));
                        }
                    }
                }
                RegReadPhase::AwaitReply => {
                    let rid = COMM_IDENTITY_RIDS[self.commissioning.rid_idx];
                    let tx_started = self.commissioning.req_tx_started.unwrap_or_else(|| drv.clock_now());
                    let req_deadline = self.commissioning.req_deadline;
                    let mut drained = false;
                    if *remaining_rx > 0 && drv.clock_now() < drv_cutoff {
                        match drv.commissioning_drain_rx_step(*remaining_rx, drv_cutoff) {
                            Ok((frames, d)) => {
                                *remaining_rx = remaining_rx.saturating_sub(frames);
                                drained = d;
                            }
                            Err(_) => {
                                self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                                return Err((
                                    502,
                                    "MIT_TIMEOUT_READ",
                                    crate::safety::BoundedText::new("CAN RX error while awaiting register reply"),
                                ));
                            }
                        }
                    }
                    if let Err((status, code, msg)) =
                        self.commissioning_txn_deadline_ok(self.commissioning.kind, owner_now_fn())
                    {
                        self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                        return Err((status, code, crate::safety::BoundedText::new(msg)));
                    }
                    let reply_expired = req_deadline.is_some_and(|dl| drv.clock_now() >= dl);
                    let match_res = drv.commissioning_find_register_reply(
                        joint,
                        false,
                        rid,
                        self.commissioning.req_tx_seq,
                        tx_started,
                        req_deadline,
                        self.commissioning.req_tx_offset_ns,
                    );
                    if matches!(match_res, CommissioningRegMatch::Ambiguous) {
                        self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                        let mut buf = crate::safety::BoundedText::empty();
                        let _ = core::fmt::write(
                            &mut buf,
                            format_args!(
                                "Joint {} (CAN ID 0x{:X}) received ambiguous/stale register reply for RID {rid}; failing closed",
                                joint + 1,
                                motor_id
                            ),
                        );
                        return Err((502, "MIT_TIMEOUT_READ", buf));
                    }
                    if reply_expired {
                        self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                        let mut buf = crate::safety::BoundedText::empty();
                        let _ = core::fmt::write(
                            &mut buf,
                            format_args!(
                                "Joint {} (CAN ID 0x{:X}) did not answer the read of register {rid}",
                                joint + 1,
                                motor_id
                            ),
                        );
                        return Err((502, "MIT_TIMEOUT_READ", buf));
                    }
                    // Blocker 4: Complete the RX drain before accepting any matched reply so a
                    // duplicate reply sitting beyond this release's RX drain budget cannot slip past.
                    if !drained {
                        return Ok(None);
                    }
                    if let CommissioningRegMatch::Matched(rep) = match_res {
                        self.commissioning.req_in_flight = false;
                        self.commissioning.read_vals[self.commissioning.rid_idx] = (rep.value, rep.reply_id);
                        self.commissioning.rid_idx += 1;
                        if self.commissioning.rid_idx >= COMM_IDENTITY_RIDS.len() {
                            let ident = self.commissioning.build_identity(joint, motor_id).map_err(|m| {
                                (502, "MIT_TIMEOUT_READ", crate::safety::BoundedText::new(m))
                            })?;
                            self.commissioning.reset_reg_read();
                            return Ok(Some(ident));
                        }
                        self.commissioning.reg_phase = RegReadPhase::DrainBeforeReq;
                        continue;
                    }
                    return Ok(None);
                }
            }
        }
    }

    #[allow(clippy::result_large_err)]
    fn step_read_identity_with_drv(
        &mut self,
        drv: &mut dyn crate::driver::ArmDriver,
        auth: &CommissioningAuthority,
        drv_cutoff: std::time::Instant,
        mut remaining_rx: u16,
        max_tx: u8,
        reg_tx_attempted: &mut u8,
        backpressured: &mut bool,
        owner_now_fn: &mut dyn FnMut() -> std::time::Instant,
    ) -> Result<(), (u16, &'static str, crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>)> {
        let mut tx_budget = max_tx;
        while self.commissioning.joint_idx < self.commissioning.joint_len {
            let idx = self.commissioning.joint_idx;
            let joint = self.commissioning.joints[idx];
            if let Err((status, code, msg)) =
                self.commissioning_safety_check_with_driver(CommissioningOpKind::ReadIdentity, &[joint], Some(drv))
            {
                return Err((status, code, crate::safety::BoundedText::new(msg)));
            }
            match self.step_one_identity_read(
                drv,
                joint,
                auth,
                drv_cutoff,
                &mut remaining_rx,
                &mut tx_budget,
                reg_tx_attempted,
                backpressured,
                owner_now_fn,
            )? {
                Some(ident) => {
                    self.commissioning.outcomes[idx].disposition = JointCommissioningDisposition::Applied;
                    self.commissioning.outcomes[idx].before = Some(ident);
                    self.commissioning.outcomes[idx].readback = Some(ident);
                    self.commissioning.joint_idx += 1;
                }
                None => return Ok(()),
            }
        }
        self.commissioning.state = CommissioningOpState::Done;
        Ok(())
    }

    #[allow(clippy::result_large_err)]
    fn step_write_timeout_with_drv(
        &mut self,
        drv: &mut dyn crate::driver::ArmDriver,
        auth: &CommissioningAuthority,
        drv_cutoff: std::time::Instant,
        mut remaining_rx: u16,
        max_tx: u8,
        probe_tx_cap: Option<u8>,
        reg_tx_attempted: &mut u8,
        probe_tx_attempted: &mut u8,
        backpressured: &mut bool,
        owner_now_fn: &mut dyn FnMut() -> std::time::Instant,
    ) -> Result<(), (u16, &'static str, crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>)> {
        let mut tx_budget = max_tx;
        let counts = self.commissioning.timeout_counts;
        while self.commissioning.joint_idx < self.commissioning.joint_len {
            if let Err((status, code, msg)) = self.commissioning_txn_deadline_ok(
                CommissioningOpKind::WriteTimeout,
                owner_now_fn(),
            ) {
                return Err((status, code, crate::safety::BoundedText::new(msg)));
            }
            if drv.clock_now() >= drv_cutoff {
                return Ok(());
            }
            let idx = self.commissioning.joint_idx;
            let joint = self.commissioning.joints[idx];
            let Some(motor_id) = drv.commissioning_motor_id(joint) else {
                return Err((
                    502,
                    "MIT_TIMEOUT_FAILED",
                    crate::safety::BoundedText::new("joint index out of driver range"),
                ));
            };
            match self.commissioning.write_phase {
                WriteTimeoutPhase::GatherRestEvidence => {
                    // Reject any hard safety/hardware/non-rest refusal immediately with 0 TX.
                    let active_joints = self.commissioning.joints;
                    let active_len = self.commissioning.joint_len;
                    if let Err((status, code, msg)) = self.commissioning_safety_check_with_driver_inner(
                        CommissioningOpKind::WriteTimeout,
                        &active_joints[..active_len],
                        Some(drv),
                        true,
                    ) {
                        return Err((status, code, crate::safety::BoundedText::new(msg)));
                    }
                    // Arm the bounded gather deadline immediately on phase entry so continuous RX
                    // traffic cannot stretch GatherRestEvidence to the 15 s transaction deadline.
                    let gather_dl = *self
                        .commissioning
                        .rest_gather_deadline
                        .get_or_insert_with(|| drv.clock_now() + COMM_REST_EVIDENCE_TIMEOUT);
                    // Drain any queued feedback to an observed-empty boundary before judging or
                    // transmitting read-only 0xCC probes.
                    let mut drained_now = false;
                    if remaining_rx > 0 && drv.clock_now() < drv_cutoff {
                        match drv.commissioning_drain_rx_step(remaining_rx, drv_cutoff) {
                            Ok((frames, drained)) => {
                                remaining_rx = remaining_rx.saturating_sub(frames);
                                drained_now = drained;
                            }
                            Err(_) => {
                                return Err((
                                    502,
                                    "MIT_TIMEOUT_FAILED",
                                    crate::safety::BoundedText::new(
                                        "CAN RX error while gathering rest-pose evidence",
                                    ),
                                ));
                            }
                        }
                    }
                    // Enforce the gather deadline BEFORE accepting evidence or advancing.
                    let drv_now = drv.clock_now();
                    if drv_now >= gather_dl {
                        let (status, code, msg) = self
                            .commissioning_safety_check_with_driver(
                                CommissioningOpKind::WriteTimeout,
                                &[joint],
                                Some(drv),
                            )
                            .err()
                            .unwrap_or((
                                409,
                                "ARM_NOT_AT_RESTING_POSE",
                                "TIMEOUT commissioning requires fresh confirmed rest-pose evidence",
                            ));
                        return Err((status, code, crate::safety::BoundedText::new(msg)));
                    }
                    if !drained_now {
                        return Ok(());
                    }
                    // Re-check hard safety/non-rest preconditions after draining RX.
                    if let Err((h_st, h_code, h_msg)) = self
                        .commissioning_safety_check_with_driver_inner(
                            CommissioningOpKind::WriteTimeout,
                            &[joint],
                            Some(drv),
                            true,
                        )
                    {
                        return Err((h_st, h_code, crate::safety::BoundedText::new(h_msg)));
                    }
                    // Build required arm-joint mask (J1-J6) and update per-axis causal reply proof:
                    // every arm axis MUST deliver at least one fresh, queue-bounded, disabled reply
                    // causally AFTER this gather's 0xCC probe (preventing pre-gather cached samples
                    // on a newly silent axis from passing while other axes gather).
                    let gripper_idx = self.config.gripper_index();
                    let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
                    let enable_fb = drv.capabilities().enable_feedback;
                    let mut arm_mask = 0u64;
                    for i in 0..dof {
                        if gripper_idx != Some(i) {
                            arm_mask |= 1u64 << i;
                        }
                        if let Some(tx_at) = self.commissioning.rest_probe_tx_at[i] {
                            let tx_seq = self.commissioning.rest_probe_tx_seq[i];
                            let causal_reply = drv.commissioning_raw_feedback(i).is_some_and(|fb| {
                                fb.rx_seq > tx_seq
                                    && fb.rx_at >= tx_at
                                    && fb.queue_bound.is_some()
                                    && drv_now.saturating_duration_since(fb.rx_at)
                                        <= crate::driver::STOPPED_DISABLE_FEEDBACK_MAX_AGE
                                    && (!enable_fb || fb.motor_status == 0)
                            });
                            if causal_reply {
                                self.commissioning.rest_probe_confirmed_mask |= 1u64 << i;
                            }
                        }
                    }
                    let causal_all_arm_replied =
                        arm_mask != 0 && (self.commissioning.rest_probe_confirmed_mask & arm_mask) == arm_mask;
                    // Strict check: only proceed once EVERY arm axis delivered a causal post-probe
                    // reply in this gather AND K=3 causal fresh rest-pose evidence on J1-J6, fresh
                    // disabled feedback, and stationary checks ALL pass.
                    if causal_all_arm_replied
                        && self
                            .commissioning_safety_check_with_driver(
                                CommissioningOpKind::WriteTimeout,
                                &[joint],
                                Some(drv),
                            )
                            .is_ok()
                    {
                        let resume = self.commissioning.rest_gather_resume_phase;
                        self.commissioning.reset_rest_gather(WriteTimeoutPhase::ReadBefore);
                        self.commissioning.write_phase = resume;
                        continue;
                    }
                    // Issue at most one bounded read-only 0xCC burst per control release
                    // (spaced by COMM_REST_PROBE_INTERVAL) so CausalWindow accumulates K=3
                    // distinct control releases spanning >= 20 ms within 100 ms.
                    if self.commissioning.rest_probe_pending_mask == 0 {
                        if self.commissioning.rest_probe_last_release_seq
                            == self.commissioning.release_step_count
                            || self
                                .commissioning
                                .rest_probe_next_at
                                .is_some_and(|t| drv_now < t)
                        {
                            return Ok(());
                        }
                        let mut mask = arm_mask;
                        if let Some(gi) = gripper_idx {
                            if gi < dof && enable_fb {
                                let stale = drv.commissioning_raw_feedback(gi).is_none_or(|fb| {
                                    drv_now.saturating_duration_since(fb.rx_at)
                                        > crate::driver::STOPPED_DISABLE_FEEDBACK_MAX_AGE
                                });
                                if stale
                                    || drv.enable_state() != crate::model::EnableState::Disabled
                                {
                                    mask |= 1u64 << gi;
                                }
                            }
                        }
                        self.commissioning.rest_probe_pending_mask = mask;
                    }
                    let probe_dl = drv_cutoff.min(gather_dl);
                    let mut sent_any = false;
                    let mut probes_sent = 0u8;
                    for i in 0..dof {
                        if (self.commissioning.rest_probe_pending_mask & (1u64 << i)) == 0 {
                            continue;
                        }
                        if probe_tx_cap.is_some_and(|cap| probes_sent >= cap) {
                            break;
                        }
                        if drv.clock_now() >= probe_dl {
                            break;
                        }
                        let mut gate = || -> Result<(), &'static str> {
                            self.commissioning_pre_tx_gate_ok(
                                CommissioningOpKind::WriteTimeout,
                                auth,
                                owner_now_fn(),
                            )
                        };
                        match drv.commissioning_try_send_rest_probe(i, probe_dl, &mut gate) {
                            CommissioningTxStatus::Sent {
                                tx_seq,
                                tx_started,
                                ..
                            } => {
                                self.commissioning.rest_probe_pending_mask &= !(1u64 << i);
                                if self.commissioning.rest_probe_tx_at[i].is_none() {
                                    self.commissioning.rest_probe_tx_seq[i] = tx_seq;
                                    self.commissioning.rest_probe_tx_at[i] = Some(tx_started);
                                }
                                probes_sent = probes_sent.saturating_add(1);
                                *probe_tx_attempted = probe_tx_attempted.saturating_add(1);
                                sent_any = true;
                            }
                            CommissioningTxStatus::SentFlushFailed { .. } => {
                                return Err((
                                    502,
                                    "MIT_TIMEOUT_FAILED",
                                    crate::safety::BoundedText::new(
                                        "CAN flush failed during rest-pose probe",
                                    ),
                                ));
                            }
                            CommissioningTxStatus::Backpressure => {
                                *backpressured = true;
                                break;
                            }
                            CommissioningTxStatus::Revoked(rc) => {
                                return Err(Self::map_commissioning_revoked(
                                    CommissioningOpKind::WriteTimeout,
                                    rc,
                                    "authority revoked before rest-pose probe",
                                ));
                            }
                            CommissioningTxStatus::Refused(rm) => {
                                return Err((
                                    502,
                                    "MIT_TIMEOUT_FAILED",
                                    crate::safety::BoundedText::new(rm),
                                ));
                            }
                        }
                    }
                    if sent_any && self.commissioning.rest_probe_pending_mask == 0 {
                        self.commissioning.rest_probe_last_release_seq =
                            self.commissioning.release_step_count;
                        self.commissioning.rest_probe_next_at =
                            Some(drv.clock_now() + COMM_REST_PROBE_INTERVAL);
                    }
                    // Yield to the next control release so Step 2 decodes the 0xCC replies
                    // under a fresh control_release_id and updates measured_velocities.
                    return Ok(());
                }
                WriteTimeoutPhase::ReadBefore => {
                    // `ReadBefore` issues read-only `0x33` register reads only. Enforce all hard
                    // safety/non-rest preconditions on every tick without interrupting an in-flight
                    // 50 ms register read if the 100 ms rest window ages during multi-register I/O;
                    // `SendWrite` strictly enforces (and re-gathers if aged) K=3 rest evidence
                    // immediately before any `0x55` `REG_WRITE` leaves the host.
                    if let Err((status, code, msg)) = self.commissioning_safety_check_with_driver_inner(
                        CommissioningOpKind::WriteTimeout,
                        &[joint],
                        Some(drv),
                        true,
                    ) {
                        return Err((status, code, crate::safety::BoundedText::new(msg)));
                    }
                    let Some(before) = self
                        .step_one_identity_read(
                            drv,
                            joint,
                            auth,
                            drv_cutoff,
                            &mut remaining_rx,
                            &mut tx_budget,
                            reg_tx_attempted,
                            backpressured,
                            owner_now_fn,
                        )
                        .map_err(|(st, code, m)| {
                            let mapped_code = if code == "MIT_TIMEOUT_READ" { "MIT_TIMEOUT_FAILED" } else { code };
                            (st, mapped_code, m)
                        })?
                    else {
                        return Ok(());
                    };
                    if before.reply_id != before.mst_id {
                        let mut buf = crate::safety::BoundedText::empty();
                        let _ = core::fmt::write(
                            &mut buf,
                            format_args!(
                                "joint {}: register replies on 0x{:X} but MST_ID {}",
                                joint + 1,
                                before.reply_id,
                                before.mst_id
                            ),
                        );
                        return Err((502, "MIT_TIMEOUT_FAILED", buf));
                    }
                    self.commissioning.outcomes[idx].before = Some(before);
                    self.commissioning.req_attempts = 0;
                    self.commissioning.write_phase = WriteTimeoutPhase::DrainBeforeWrite;
                }
                WriteTimeoutPhase::DrainBeforeWrite => {
                    if let Some(until) = self.commissioning.unmatched_reply_quiet_until {
                        if drv.clock_now() < until {
                            return Ok(());
                        }
                        self.commissioning.unmatched_reply_quiet_until = None;
                    }
                    if remaining_rx == 0 {
                        return Ok(());
                    }
                    match drv.commissioning_drain_rx_step(remaining_rx, drv_cutoff) {
                        Ok((frames, drained)) => {
                            remaining_rx = remaining_rx.saturating_sub(frames);
                            if !drained {
                                return Ok(());
                            }
                            drv.commissioning_clear_register_log();
                            self.commissioning.write_phase = WriteTimeoutPhase::SendWrite;
                        }
                        Err(_) => {
                            return Err((
                                502,
                                "MIT_TIMEOUT_FAILED",
                                crate::safety::BoundedText::new("CAN RX error before TIMEOUT write"),
                            ));
                        }
                    }
                }
                WriteTimeoutPhase::SendWrite => {
                    if tx_budget == 0 {
                        return Ok(());
                    }
                    // Recheck stationary + torque confirmed off + fresh rest evidence BEFORE TX.
                    if let Err((status, code, msg)) = self.commissioning_safety_check_with_driver(
                        CommissioningOpKind::WriteTimeout,
                        &[joint],
                        Some(drv),
                    ) {
                        if self
                            .commissioning_safety_check_with_driver_inner(
                                CommissioningOpKind::WriteTimeout,
                                &[joint],
                                Some(drv),
                                true,
                            )
                            .is_ok()
                        {
                            self.commissioning
                                .reset_rest_gather(WriteTimeoutPhase::DrainBeforeWrite);
                            self.commissioning.write_phase = WriteTimeoutPhase::GatherRestEvidence;
                            continue;
                        }
                        return Err((status, code, crate::safety::BoundedText::new(msg)));
                    }
                    let mut gate = || -> Result<(), &'static str> {
                        self.commissioning_pre_tx_gate_ok(
                            CommissioningOpKind::WriteTimeout,
                            auth,
                            owner_now_fn(),
                        )
                    };
                    tx_budget = tx_budget.saturating_sub(1);
                    *reg_tx_attempted = reg_tx_attempted.saturating_add(1);
                    match drv.commissioning_try_send_timeout_write(joint, counts, drv_cutoff, &mut gate) {
                        CommissioningTxStatus::Sent {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        } => {
                            // Irreversible TX accepted by transport -> immediately mark Uncertain
                            // and hw_mutated (NEVER auto-retry once accepted!).
                            self.commissioning.hw_mutated = true;
                            self.commissioning.outcomes[idx].disposition = JointCommissioningDisposition::Uncertain;
                            self.commissioning.outcomes[idx].written_raw = counts;
                            self.commissioning.req_tx_seq = tx_seq;
                            self.commissioning.req_tx_started = Some(tx_started);
                            self.commissioning.req_tx_offset_ns = tx_offset_ns;
                            self.commissioning.req_deadline = Some(drv.clock_now() + COMM_REG_REPLY_TIMEOUT);
                            self.commissioning.req_in_flight = true;
                            self.commissioning.write_phase = WriteTimeoutPhase::AwaitWriteAck;
                        }
                        CommissioningTxStatus::SentFlushFailed { .. } => {
                            self.commissioning.hw_mutated = true;
                            self.commissioning.outcomes[idx].disposition = JointCommissioningDisposition::Uncertain;
                            self.commissioning.outcomes[idx].written_raw = counts;
                            self.commissioning.req_in_flight = true;
                            self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                            let mut buf = crate::safety::BoundedText::empty();
                            let _ = core::fmt::write(
                                &mut buf,
                                format_args!("joint {}: TIMEOUT write accepted but CAN flush failed or cutoff expired", joint + 1),
                            );
                            return Err((502, "MIT_TIMEOUT_FAILED", buf));
                        }
                        CommissioningTxStatus::Backpressure => {
                            *backpressured = true;
                            // Transport did NOT accept: joint remains Unattempted, retry after re-drain
                            // and fresh live authority recheck.
                            self.commissioning.req_attempts = self.commissioning.req_attempts.saturating_add(1);
                            if self.commissioning.req_attempts >= COMM_MAX_FRAME_ATTEMPTS {
                                let mut buf = crate::safety::BoundedText::empty();
                                let _ = core::fmt::write(
                                    &mut buf,
                                    format_args!("joint {}: CAN TX backpressure exhausted before TIMEOUT write", joint + 1),
                                );
                                return Err((502, "MIT_TIMEOUT_FAILED", buf));
                            }
                            self.commissioning.write_phase = WriteTimeoutPhase::DrainBeforeWrite;
                            return Ok(());
                        }
                        CommissioningTxStatus::Revoked(code) => {
                            return Err(Self::map_commissioning_revoked(
                                CommissioningOpKind::WriteTimeout,
                                code,
                                "authority revoked before TIMEOUT write",
                            ));
                        }
                        CommissioningTxStatus::Refused(msg) => {
                            return Err((502, "MIT_TIMEOUT_FAILED", crate::safety::BoundedText::new(msg)));
                        }
                    }
                }
                WriteTimeoutPhase::AwaitWriteAck => {
                    let Some(before) = self.commissioning.outcomes[idx].before else {
                        return Err((500, "INTERNAL", crate::safety::BoundedText::new("missing before identity")));
                    };
                    let tx_started = self.commissioning.req_tx_started.unwrap_or_else(|| drv.clock_now());
                    let req_deadline = self.commissioning.req_deadline;
                    let mut drained = false;
                    if remaining_rx > 0 && drv.clock_now() < drv_cutoff {
                        match drv.commissioning_drain_rx_step(remaining_rx, drv_cutoff) {
                            Ok((frames, d)) => {
                                remaining_rx = remaining_rx.saturating_sub(frames);
                                drained = d;
                            }
                            Err(_) => {
                                self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                                return Err((
                                    502,
                                    "MIT_TIMEOUT_FAILED",
                                    crate::safety::BoundedText::new("CAN RX error while awaiting TIMEOUT write ack"),
                                ));
                            }
                        }
                    }
                    if let Err((status, code, msg)) = self.commissioning_txn_deadline_ok(
                        CommissioningOpKind::WriteTimeout,
                        owner_now_fn(),
                    ) {
                        self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                        return Err((status, code, crate::safety::BoundedText::new(msg)));
                    }
                    let reply_expired = req_deadline.is_some_and(|dl| drv.clock_now() >= dl);
                    let match_res = drv.commissioning_find_register_reply(
                        joint,
                        true,
                        damiao::RID_TIMEOUT,
                        self.commissioning.req_tx_seq,
                        tx_started,
                        req_deadline,
                        self.commissioning.req_tx_offset_ns,
                    );
                    if matches!(match_res, CommissioningRegMatch::Ambiguous) {
                        self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                        let mut buf = crate::safety::BoundedText::empty();
                        let _ = core::fmt::write(
                            &mut buf,
                            format_args!(
                                "joint {}: ambiguous or stale TIMEOUT write ack observed; failing closed as Uncertain",
                                joint + 1
                            ),
                        );
                        return Err((502, "MIT_TIMEOUT_FAILED", buf));
                    }
                    if reply_expired {
                        self.commissioning.mark_quiet_if_in_flight(drv.clock_now());
                        let mut buf = crate::safety::BoundedText::empty();
                        let _ = core::fmt::write(
                            &mut buf,
                            format_args!(
                                "joint {} (CAN ID 0x{:X}) did not acknowledge the TIMEOUT write",
                                joint + 1,
                                motor_id
                            ),
                        );
                        return Err((502, "MIT_TIMEOUT_FAILED", buf));
                    }
                    // Blocker 4: Complete the RX drain before accepting the write ACK so a
                    // duplicate ACK beyond this release's RX drain budget cannot slip past.
                    if !drained {
                        return Ok(());
                    }
                    if let CommissioningRegMatch::Matched(ack) = match_res {
                        self.commissioning.req_in_flight = false;
                        let ack_val = u32::from_le_bytes(ack.value);
                        if ack.reply_id != before.mst_id || ack_val != counts {
                            let mut buf = crate::safety::BoundedText::empty();
                            let _ = core::fmt::write(
                                &mut buf,
                                format_args!(
                                    "joint {}: TIMEOUT write acknowledged with {} on 0x{:X} (wrote {counts}, MST_ID {})",
                                    joint + 1,
                                    ack_val,
                                    ack.reply_id,
                                    before.mst_id
                                ),
                            );
                            return Err((502, "MIT_TIMEOUT_FAILED", buf));
                        }
                        self.commissioning.outcomes[idx].ack_raw = ack_val;
                        self.commissioning.reset_reg_read();
                        self.commissioning.write_phase = WriteTimeoutPhase::ReadAfter;
                        continue;
                    }
                    return Ok(());
                }
                WriteTimeoutPhase::ReadAfter => {
                    let Some(before) = self.commissioning.outcomes[idx].before else {
                        return Err((500, "INTERNAL", crate::safety::BoundedText::new("missing before identity")));
                    };
                    let Some(readback) = self
                        .step_one_identity_read(
                            drv,
                            joint,
                            auth,
                            drv_cutoff,
                            &mut remaining_rx,
                            &mut tx_budget,
                            reg_tx_attempted,
                            backpressured,
                            owner_now_fn,
                        )
                        .map_err(|(st, code, m)| {
                            let mapped_code = if code == "MIT_TIMEOUT_READ" { "MIT_TIMEOUT_FAILED" } else { code };
                            (st, mapped_code, m)
                        })?
                    else {
                        return Ok(());
                    };
                    let same = readback.motor_id == before.motor_id
                        && readback.reply_id == before.reply_id
                        && readback.reply_id == readback.mst_id
                        && readback.esc_id == before.esc_id
                        && readback.mst_id == before.mst_id
                        && readback.sw_ver == before.sw_ver;
                    if !same || readback.timeout_raw != counts {
                        let mut buf = crate::safety::BoundedText::empty();
                        let _ = core::fmt::write(
                            &mut buf,
                            format_args!(
                                "joint {}: TIMEOUT readback {} (wrote {counts}) or identity changed",
                                joint + 1,
                                readback.timeout_raw
                            ),
                        );
                        return Err((502, "MIT_TIMEOUT_FAILED", buf));
                    }
                    self.commissioning.outcomes[idx].readback = Some(readback);
                    self.commissioning.outcomes[idx].disposition = JointCommissioningDisposition::Applied;
                    self.commissioning.joint_idx += 1;
                    self.commissioning.reset_reg_read();
                    self.commissioning.write_phase = WriteTimeoutPhase::ReadBefore;
                }
            }
        }
        self.commissioning.state = CommissioningOpState::Done;
        Ok(())
    }

    #[allow(clippy::result_large_err)]
    fn step_motor_zero_disable_with_drv(
        &mut self,
        drv: &mut dyn crate::driver::ArmDriver,
        auth: &CommissioningAuthority,
        drv_cutoff: std::time::Instant,
        mut remaining_rx: u16,
        owner_now_fn: &mut dyn FnMut() -> std::time::Instant,
    ) -> Result<(), (u16, &'static str, crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>)> {
        let now = drv.clock_now();
        let first_attempt = self.commissioning.disable_started_at.is_none();
        let started_at = *self.commissioning.disable_started_at.get_or_insert(now);
        #[cfg(not(target_arch = "wasm32"))]
        if first_attempt {
            let live_op_id = self.torque_shadow.alloc_live_op_id();
            let live_intent = self.shadow_live_intent();
            let live_is_torqued = self.is_torqued;
            let live_stopped = self.safety.state().is_stopped();
            self.torque_shadow.enqueue_at(
                started_at,
                crate::torque::shadow::ShadowInputKind::DisableRequest {
                    live_op_id,
                    explicit_release_authorized: false,
                    live_refusal: None,
                    live_intent,
                    live_is_torqued,
                    live_stopped,
                },
            );
        }
        let deadline = *self
            .commissioning
            .disable_deadline
            .get_or_insert(started_at + crate::motor_zero::DISABLE_CONFIRM_TIMEOUT);
        if now >= deadline {
            return Err((
                409,
                "MOTOR_ZERO_NOT_DISABLED",
                crate::safety::BoundedText::new("motors did not confirm disabled within 1000 ms; nothing was changed"),
            ));
        }
        match self.commissioning.mz_phase {
            MotorZeroPhase::DisableDispatch => {
                let mut gate = || -> Result<(), &'static str> {
                    self.commissioning_pre_tx_gate_ok(
                        CommissioningOpKind::MotorZeroDisable,
                        auth,
                        owner_now_fn(),
                    )
                };
                match drv.commissioning_disable_step(started_at, drv_cutoff, &mut gate) {
                    CommissioningDisableStep::AllDispatched {
                        boundary_seq,
                        tx_started,
                        ..
                    } => {
                        self.commissioning.disable_boundary_seq = boundary_seq;
                        self.commissioning.disable_tx_started = Some(tx_started);
                        self.commissioning.mz_phase = MotorZeroPhase::DisableAwaitConfirm;
                    }
                    CommissioningDisableStep::InProgress { .. }
                    | CommissioningDisableStep::WaitEvidence => return Ok(()),
                    CommissioningDisableStep::Revoked(code) => {
                        return Err(Self::map_commissioning_revoked(
                            CommissioningOpKind::MotorZeroDisable,
                            code,
                            "authority revoked before disable",
                        ));
                    }
                    CommissioningDisableStep::Refused(msg) => {
                        let code = if msg == "ARM_NOT_AT_RESTING_POSE" || msg == "TORQUE_OFF_WOULD_DROP" {
                            "ARM_NOT_AT_RESTING_POSE"
                        } else {
                            "MOTOR_ZERO_NOT_DISABLED"
                        };
                        return Err((409, code, crate::safety::BoundedText::new(msg)));
                    }
                }
            }
            MotorZeroPhase::DisableAwaitConfirm => {}
            _ => return Ok(()),
        }

        // Confirm ALL motors (matching DamiaoArmDriver::enable_state() == Disabled) have fresh
        // post-command disabled feedback before allowing any motor-zero write.
        if remaining_rx > 0 && drv.clock_now() < drv_cutoff {
            if let Ok((frames, _)) = drv.commissioning_drain_rx_step(remaining_rx, drv_cutoff) {
                remaining_rx = remaining_rx.saturating_sub(frames);
            }
        }
        let _ = remaining_rx;
        let tx_started = self.commissioning.disable_tx_started.unwrap_or(started_at);
        let boundary_seq = self.commissioning.disable_boundary_seq;
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        let mut all_disabled = drv.enable_state() == crate::model::EnableState::Disabled;
        if all_disabled {
            for j in 0..dof {
                let ok = drv.commissioning_raw_feedback(j).is_some_and(|fb| {
                    fb.rx_seq > boundary_seq
                        && fb.rx_at >= tx_started
                        && fb.queue_bound.is_some()
                        && (!drv.capabilities().enable_feedback || fb.motor_status == 0)
                });
                if !ok {
                    all_disabled = false;
                    break;
                }
            }
        }
        if all_disabled {
            self.commissioning.state = CommissioningOpState::Done;
        } else if drv.clock_now() >= deadline {
            return Err((
                409,
                "MOTOR_ZERO_NOT_DISABLED",
                crate::safety::BoundedText::new("motors did not confirm disabled within 1000 ms; nothing was changed"),
            ));
        }
        Ok(())
    }

    #[allow(clippy::result_large_err)]
    fn step_motor_zero_write_verify_with_drv(
        &mut self,
        drv: &mut dyn crate::driver::ArmDriver,
        auth: &CommissioningAuthority,
        drv_cutoff: std::time::Instant,
        mut remaining_rx: u16,
        reg_tx_attempted: &mut u8,
        backpressured: &mut bool,
        owner_now_fn: &mut dyn FnMut() -> std::time::Instant,
    ) -> Result<(), (u16, &'static str, crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>)> {
        let joint_len = self.commissioning.joint_len;
        match self.commissioning.mz_phase {
            MotorZeroPhase::ZeroSendLoop => {
                let idx = self.commissioning.joint_idx;
                if idx >= joint_len {
                    self.commissioning.zero_verify_deadline =
                        Some(drv.clock_now() + crate::motor_zero::VERIFY_TIMEOUT);
                    self.commissioning.mz_phase = MotorZeroPhase::ZeroVerify;
                } else {
                    let now = drv.clock_now();
                    if let Some(next_at) = self.commissioning.next_zero_send_at {
                        if now < next_at {
                            return Ok(());
                        }
                    }
                    let joint = self.commissioning.joints[idx];
                    // Recheck safety, stationary, torque confirmed off, and fresh rest evidence BEFORE TX.
                    if let Err((status, code, msg)) = self.commissioning_safety_check_with_driver(
                        CommissioningOpKind::MotorZeroWriteVerify,
                        &[joint],
                        Some(drv),
                    ) {
                        return Err((status, code, crate::safety::BoundedText::new(msg)));
                    }
                    let mut gate = || -> Result<(), &'static str> {
                        self.commissioning_pre_tx_gate_ok(
                            CommissioningOpKind::MotorZeroWriteVerify,
                            auth,
                            owner_now_fn(),
                        )
                    };
                    *reg_tx_attempted = reg_tx_attempted.saturating_add(1);
                    match drv.commissioning_try_send_set_zero(joint, drv_cutoff, &mut gate) {
                        CommissioningTxStatus::Sent {
                            tx_seq,
                            tx_started,
                            ..
                        } => {
                            // Mark hw_mutated + Uncertain immediately at wire acceptance.
                            // Do NOT mutate joint_zero_offsets or ZeroOffsetDriver yet (M2).
                            self.commissioning.hw_mutated = true;
                            self.commissioning.zero_sent_mask |= 1u64 << idx;
                            self.commissioning.zero_sent_joint_mask |= 1u64 << joint;
                            self.commissioning.zero_boundary_seq[idx] = tx_seq;
                            self.commissioning.zero_tx_started[idx] = Some(tx_started);
                            self.commissioning.outcomes[idx].disposition = JointCommissioningDisposition::Uncertain;
                            self.commissioning.req_attempts = 0;
                            self.commissioning.joint_idx = idx + 1;
                            if self.commissioning.joint_idx >= joint_len {
                                self.commissioning.zero_verify_deadline =
                                    Some(drv.clock_now() + crate::motor_zero::VERIFY_TIMEOUT);
                                self.commissioning.mz_phase = MotorZeroPhase::ZeroVerify;
                            } else {
                                self.commissioning.next_zero_send_at =
                                    Some(drv.clock_now() + crate::motor_zero::COMMAND_SPACING);
                            }
                        }
                        CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            ..
                        } => {
                            self.commissioning.hw_mutated = true;
                            self.commissioning.zero_sent_mask |= 1u64 << idx;
                            self.commissioning.zero_sent_joint_mask |= 1u64 << joint;
                            self.commissioning.zero_boundary_seq[idx] = tx_seq;
                            self.commissioning.zero_tx_started[idx] = Some(tx_started);
                            self.commissioning.outcomes[idx].disposition = JointCommissioningDisposition::Uncertain;
                            let mut buf = crate::safety::BoundedText::empty();
                            let _ = core::fmt::write(
                                &mut buf,
                                format_args!("J{}: set-zero accepted but CAN flush failed or cutoff expired", joint + 1),
                            );
                            return Err((500, "MOTOR_ZERO_FAILED", buf));
                        }
                        CommissioningTxStatus::Backpressure => {
                            *backpressured = true;
                            self.commissioning.req_attempts = self.commissioning.req_attempts.saturating_add(1);
                            if self.commissioning.req_attempts >= COMM_MAX_FRAME_ATTEMPTS {
                                let mut buf = crate::safety::BoundedText::empty();
                                let _ = core::fmt::write(
                                    &mut buf,
                                    format_args!("J{}: CAN TX backpressure exhausted before set-zero", joint + 1),
                                );
                                return Err((500, "MOTOR_ZERO_FAILED", buf));
                            }
                            return Ok(());
                        }
                        CommissioningTxStatus::Revoked(code) => {
                            return Err(Self::map_commissioning_revoked(
                                CommissioningOpKind::MotorZeroWriteVerify,
                                code,
                                "authority revoked before set-zero",
                            ));
                        }
                        CommissioningTxStatus::Refused(msg) => {
                            let mut buf = crate::safety::BoundedText::empty();
                            let _ = core::fmt::write(&mut buf, format_args!("J{}: {msg}", joint + 1));
                            return Err((500, "MOTOR_ZERO_FAILED", buf));
                        }
                    }
                }
            }
            MotorZeroPhase::ZeroVerify => {}
            _ => return Ok(()),
        }

        if self.commissioning.mz_phase != MotorZeroPhase::ZeroVerify {
            return Ok(());
        }
        if remaining_rx > 0 && drv.clock_now() < drv_cutoff {
            if let Ok((frames, _)) = drv.commissioning_drain_rx_step(remaining_rx, drv_cutoff) {
                remaining_rx = remaining_rx.saturating_sub(frames);
            }
        }
        let _ = remaining_rx;
        let verify_tol = crate::model::MOTOR_ZERO_VERIFY_MRAD as f32 / 1000.0;
        let mut all_verified = true;
        for idx in 0..joint_len {
            if (self.commissioning.zero_sent_mask & (1u64 << idx)) == 0 {
                continue;
            }
            let joint = self.commissioning.joints[idx];
            let boundary_seq = self.commissioning.zero_boundary_seq[idx];
            let Some(tx_started) = self.commissioning.zero_tx_started[idx] else {
                all_verified = false;
                continue;
            };
            if let Some(fb) = drv.commissioning_raw_feedback(joint) {
                let fresh = fb.rx_seq > boundary_seq
                    && fb.rx_at >= tx_started
                    && fb.queue_bound.is_some();
                let raw_ok = fresh && fb.raw_position.is_finite() && fb.raw_position.abs() < verify_tol;
                self.commissioning.outcomes[idx].raw_after = fb.raw_position;
                self.commissioning.outcomes[idx].joint_after =
                    fb.raw_position + self.commissioning.references[joint];
                self.commissioning.outcomes[idx].fresh = fresh;
                self.commissioning.outcomes[idx].verified = raw_ok;
                if raw_ok {
                    self.commissioning.outcomes[idx].disposition = JointCommissioningDisposition::Applied;
                } else {
                    all_verified = false;
                }
            } else {
                all_verified = false;
            }
        }
        if all_verified {
            self.commissioning.state = CommissioningOpState::Done;
        } else if self
            .commissioning
            .zero_verify_deadline
            .is_some_and(|dl| drv.clock_now() >= dl)
        {
            return Err((
                500,
                "MOTOR_ZERO_UNVERIFIED",
                crate::safety::BoundedText::new("not every zeroed joint verified within tolerance of expected zero"),
            ));
        }
        Ok(())
    }
}

// ============================================================================
// C-F1 part 2: Off-thread executor orchestrators (TIMEOUT & MIT qualification)
// ============================================================================

#[cfg(not(target_arch = "wasm32"))]
const COMMISSIONING_POLL_INTERVAL: Duration = Duration::from_millis(5);
#[cfg(not(target_arch = "wasm32"))]
const COMMISSIONING_TXN_TIMEOUT: Duration = Duration::from_secs(15);

#[cfg(not(target_arch = "wasm32"))]
struct CommissioningOpGuard {
    sup: Option<std::sync::Arc<crate::supervisor::ArmSupervisor>>,
    op_id: u64,
}

#[cfg(not(target_arch = "wasm32"))]
impl CommissioningOpGuard {
    fn new(sup: std::sync::Arc<crate::supervisor::ArmSupervisor>, op_id: u64) -> Self {
        Self {
            sup: Some(sup),
            op_id,
        }
    }

    async fn clear(mut self, clean_complete: bool, force_reconcile: bool) {
        if let Some(sup) = self.sup.take() {
            let op_id = self.op_id;
            let _ = sup
                .dispatch_config_request_reliable(|| {
                    crate::rt_core::requests::config::ConfigRequest::ClearCommissioning {
                        op_id,
                        clean_complete,
                        force_reconcile,
                    }
                })
                .await;
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl Drop for CommissioningOpGuard {
    fn drop(&mut self) {
        let Some(sup) = self.sup.take() else {
            return;
        };
        let op_id = self.op_id;
        match tokio::runtime::Handle::try_current() {
            Ok(h) => {
                h.spawn(async move {
                    let _ = sup
                        .dispatch_config_request_reliable(|| {
                            crate::rt_core::requests::config::ConfigRequest::CancelCommissioning {
                                op_id,
                                reason: "CANCELLED",
                            }
                        })
                        .await;
                    let _ = sup
                        .dispatch_config_request_reliable(|| {
                            crate::rt_core::requests::config::ConfigRequest::ClearCommissioning {
                                op_id,
                                clean_complete: false,
                                force_reconcile: false,
                            }
                        })
                        .await;
                });
            }
            Err(_) => {
                if let Some(h) = sup.executor_handle() {
                    let _ = h.call_config(
                        crate::rt_core::requests::config::ConfigRequest::CancelCommissioning {
                            op_id,
                            reason: "CANCELLED",
                        },
                    );
                    let _ = h.call_config(
                        crate::rt_core::requests::config::ConfigRequest::ClearCommissioning {
                            op_id,
                            clean_complete: false,
                            force_reconcile: false,
                        },
                    );
                }
            }
        }
    }
}

/// Distinguishes a definitive pre-TX start refusal from an unknown post-submission outcome (B3).
/// Callers MUST preserve `.calibration_commit.pending` and force `HoldState::Reconciling` on
/// `UnknownPostStartOutcome` and may unlink the marker only on `DefinitivePreTxRefusal`.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) enum CommissioningRunError {
    DefinitivePreTxRefusal(CalibError),
    UnknownPostStartOutcome(CalibError),
}

#[cfg(not(target_arch = "wasm32"))]
impl CommissioningRunError {
    pub(crate) fn into_calib_error(self) -> CalibError {
        match self {
            Self::DefinitivePreTxRefusal(e) | Self::UnknownPostStartOutcome(e) => e,
        }
    }
}

/// Runs one tick-driven RT commissioning operation on the exclusive RT owner, polling off-thread
/// while checking live off-thread operator/lease authority, cancelling on revocation or drop,
/// and clearing the RT slot once the terminal snapshot is captured.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn run_rt_commissioning_op(
    sup: &std::sync::Arc<crate::supervisor::ArmSupervisor>,
    req: CommissioningStartRequest,
    auth_check: &(dyn Fn() -> Result<(), (&'static str, crate::supervisor::ArmError)> + Send + Sync),
) -> Result<CommissioningSnapshot, CommissioningRunError> {
    use crate::rt_core::requests::config::{ConfigReply, ConfigRequest};
    if let Err((_, arm_err)) = (auth_check)() {
        return Err(CommissioningRunError::DefinitivePreTxRefusal(CalibError::from(
            arm_err,
        )));
    }
    let op_id = match sup
        .dispatch_config_request_owned_reliable(ConfigRequest::StartCommissioning(req))
        .await
    {
        Ok(ConfigReply::CommissioningStarted(Ok(id))) => id,
        Ok(ConfigReply::CommissioningStarted(Err(e))) => {
            return Err(CommissioningRunError::DefinitivePreTxRefusal(
                crate::calibration::calib_error_from_config(e),
            ));
        }
        Err(e) => {
            return Err(CommissioningRunError::UnknownPostStartOutcome(
                crate::calibration::calib_error_from_config(e),
            ));
        }
        _ => {
            return Err(CommissioningRunError::UnknownPostStartOutcome(CalibError::new(
                500,
                "INTERNAL",
                "unexpected start commissioning reply",
            )));
        }
    };
    let op_guard = CommissioningOpGuard::new(std::sync::Arc::clone(sup), op_id);

    // Legacy headless fallback tracking (matches TQ-LOCK pattern):
    // When a live 50 Hz legacy control loop is ticking, `tick_shared` services commissioning
    // strictly after the tick's own output & publish in short `<= 2 ms` write guards bounded by
    // `release_deadline - publish_margin` with zero headless interference.
    // When no background ticker is running (e.g. headless tests), step every 5 ms after 25 ms
    // of idle detection while keeping every SupervisorLock write hold <= 2 ms.
    const FALLBACK_IDLE_DETECT: std::time::Duration = std::time::Duration::from_millis(25);
    const FALLBACK_HEADLESS_INTERVAL: std::time::Duration = std::time::Duration::from_millis(5);
    let legacy_tick_counter = if sup.owner_mode() == crate::supervisor::OWNER_LEGACY {
        sup.legacy_tick_seq_counter().await
    } else {
        None
    };
    let mut last_tick_seq = legacy_tick_counter.as_ref().map_or(0, |(_, s)| *s);
    let mut last_fallback_step = std::time::Instant::now();
    let mut headless_active = false;

    let snap = loop {
        if let Err((code, _)) = (auth_check)() {
            let reason = match code {
                "OPERATOR_REQUIRED" => "OPERATOR_REQUIRED",
                "LEASE_LOST" => "LEASE_LOST",
                _ => "CANCELLED",
            };
            let cancelled = match sup
                .dispatch_config_request_reliable(|| ConfigRequest::CancelCommissioning {
                    op_id,
                    reason,
                })
                .await
            {
                Ok(ConfigReply::CommissioningCancelled(Some(s))) => s,
                _ => {
                    op_guard.clear(false, req.kind.is_mutating()).await;
                    let ce = match code {
                        "OPERATOR_REQUIRED" => CalibError::new(
                            403,
                            "OPERATOR_REQUIRED",
                            "Operator session expired or locked during commissioning",
                        ),
                        "LEASE_LOST" => CalibError::new(
                            409,
                            "LEASE_LOST",
                            "This connection lost the lease during commissioning",
                        ),
                        _ => CalibError::new(409, "CANCELLED", "commissioning cancelled"),
                    };
                    return Err(CommissioningRunError::UnknownPostStartOutcome(ce));
                }
            };
            break cancelled;
        }
        match sup
            .dispatch_config_request_reliable(|| ConfigRequest::PollCommissioning { op_id })
            .await
        {
            Ok(ConfigReply::CommissioningPolled(Some(s))) => {
                if s.state != CommissioningOpState::Running {
                    break s;
                }
            }
            Ok(ConfigReply::CommissioningPolled(None)) => {
                op_guard.clear(false, req.kind.is_mutating()).await;
                return Err(CommissioningRunError::UnknownPostStartOutcome(CalibError::new(
                    500,
                    "INTERNAL",
                    "commissioning operation vanished before completion",
                )));
            }
            Err(e) => {
                op_guard.clear(false, req.kind.is_mutating()).await;
                return Err(CommissioningRunError::UnknownPostStartOutcome(
                    crate::calibration::calib_error_from_deferred_config(e),
                ));
            }
            _ => {
                op_guard.clear(false, req.kind.is_mutating()).await;
                return Err(CommissioningRunError::UnknownPostStartOutcome(CalibError::new(
                    500,
                    "INTERNAL",
                    "unexpected poll commissioning reply",
                )));
            }
        }
        if let Some((ref counter, _)) = legacy_tick_counter {
            if sup.owner_mode() == crate::supervisor::OWNER_LEGACY {
                let cur_seq = counter.load(std::sync::atomic::Ordering::Acquire);
                let step_interval = if headless_active {
                    FALLBACK_HEADLESS_INTERVAL
                } else {
                    FALLBACK_IDLE_DETECT
                };
                if cur_seq != last_tick_seq {
                    last_tick_seq = cur_seq;
                    last_fallback_step = std::time::Instant::now();
                    headless_active = false;
                } else if last_fallback_step.elapsed() >= step_interval {
                    last_fallback_step = std::time::Instant::now();
                    let (obs_seq, stepped) = sup
                        .step_legacy_commissioning_headless(op_id, last_tick_seq)
                        .await;
                    headless_active = stepped;
                    last_tick_seq = obs_seq;
                }
            }
        }
        tokio::time::sleep(COMMISSIONING_POLL_INTERVAL).await;
    };

    let clean_complete = snap.state == CommissioningOpState::Done
        && (0..snap.joint_len).all(|i| snap.outcomes[i].disposition == JointCommissioningDisposition::Applied);
    let force_reconcile = !clean_complete && (snap.hw_mutated || snap.disk_mutated);
    op_guard.clear(clean_complete, force_reconcile).await;
    Ok(snap)
}

#[cfg(not(target_arch = "wasm32"))]
fn timeout_unit_json() -> Value {
    json!({
        "unit_us": damiao::TIMEOUT_UNIT_US_DOCUMENTED,
        "unit_verified": false,
        "note": "50 us per count is documented by Damiao and not verified on this firmware; RAM only (0x55), lost at power-off",
    })
}

/// Synchronous under-lock preflight for legacy WS `/api/v1/calibration/mit_timeout`.
/// Performs zero CAN or disk I/O: validates the request body, checks driver/torque-off/busy
/// preconditions, and reserves a `calibration_txn_id` for non-dry-run RAM writes so the actual
/// register I/O runs off `SupervisorLock` through the ticked C-F1 commissioning engine.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn check_mit_timeout_legacy_preflight(
    inner: &mut crate::supervisor::SupervisorInner,
    body: &Value,
) -> Result<(TimeoutCommand, Option<u64>), CalibError> {
    let cmd = parse_timeout_command(&inner.config, body)?;
    if inner.driver.is_none() {
        return Err(CalibError::new(400, "UNSUPPORTED", "no motor driver"));
    }
    if inner.commissioning.is_occupied() {
        return Err(CalibError::new(
            409,
            "CONFIG_BUSY",
            "a hardware commissioning operation is already running",
        ));
    }
    if cmd.dry_run {
        return Ok((cmd, None));
    }
    if inner.calibration_busy || inner.core.config().held().is_some() {
        return Err(CalibError::new(
            409,
            "CONFIG_BUSY",
            "a calibration apply, restore or motor zero is running",
        ));
    }
    if inner.estop_pending() || inner.safety.state().is_stopped() {
        return Err(CalibError::new(
            409,
            "MIT_TIMEOUT_ARMED",
            "the arm is stopped or faulted; clear the fault first",
        ));
    }
    if inner.is_torqued
        || inner.disable_pending
        || inner.torque_transition.is_some()
        || inner.handover_settling_since.is_some()
    {
        return Err(CalibError::new(
            409,
            "MIT_TIMEOUT_ARMED",
            "torque must be off (load mechanically supported) for the TIMEOUT commissioning",
        ));
    }
    if inner.active_trajectory.is_some()
        || inner.servo_target.is_some()
        || inner.gravity_comp.is_some()
        || inner.last_action.as_ref().is_some_and(|a| a.is_running())
        || inner.macro_run.is_some()
    {
        return Err(CalibError::new(409, "MIT_TIMEOUT_ARMED", "a motion is running"));
    }
    let txn_id = crate::rt_core::requests::config::next_calibration_txn_id();
    inner.calibration_busy = true;
    inner.calibration_txn_id = Some(txn_id);
    Ok((cmd, Some(txn_id)))
}

/// Synchronous preflight for executor-mode WS `/api/v1/calibration/mit_timeout`.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn check_mit_timeout_executor(
    sup: &std::sync::Arc<crate::supervisor::ArmSupervisor>,
    body: &Value,
    auth: &crate::calibration::ExecutorCalibAuth,
) -> Result<(TimeoutCommand, Option<crate::calibration::ExecutorTxnGuard>), CalibError> {
    use crate::rt_core::requests::config::{
        next_calibration_txn_id, CalibrationTxnKind, ConfigAuthContext, ConfigReply, ConfigRequest,
    };
    let pub_cfg = sup.published_config().await.map_err(CalibError::from)?;
    let cmd = parse_timeout_command(&pub_cfg.config, body)?;
    if !pub_cfg.has_driver {
        return Err(CalibError::new(400, "UNSUPPORTED", "no motor driver"));
    }
    if let Err((_, arm_err)) = (auth.auth_check)() {
        return Err(CalibError::from(arm_err));
    }
    let txn_id = next_calibration_txn_id();
    let cfg_auth = ConfigAuthContext {
        admission: auth.admission,
        lease_holder: auth.lease_holder,
    };
    let mut pre_guard = (!cmd.dry_run)
        .then(|| crate::calibration::ExecutorTxnGuard::new(std::sync::Arc::clone(sup), txn_id));
    match sup
        .dispatch_config_request(ConfigRequest::BeginCalibrationTxn {
            txn_id,
            auth: cfg_auth,
            operator_grant: auth.operator_grant,
            expected_epoch: auth.admission.estop_epoch,
            expected_stop_gen: auth.admission.stop_gen,
            kind: CalibrationTxnKind::MitTimeout {
                dry_run: cmd.dry_run,
            },
        })
        .await
    {
        Ok(ConfigReply::CalibrationTxnBegan(Ok(_))) => {
            if !cmd.dry_run {
                sup.published_config_hub().update(|cfg| {
                    cfg.calibration_busy = true;
                    cfg.calibration_txn_id = Some(txn_id);
                });
            }
            Ok((cmd, pre_guard))
        }
        Ok(ConfigReply::CalibrationTxnBegan(Err(e))) => {
            if let Some(g) = pre_guard.take() {
                let _ = g.disarm();
            }
            Err(crate::calibration::calib_error_from_config(e))
        }
        Err(e) => {
            if let Some(g) = pre_guard.take() {
                g.release().await;
            }
            Err(crate::calibration::calib_error_from_config(e))
        }
        _ => {
            if let Some(g) = pre_guard.take() {
                g.release().await;
            }
            Err(CalibError::new(
                500,
                "INTERNAL",
                "unexpected begin mit_timeout reply",
            ))
        }
    }
}

/// Executor-mode WS `/api/v1/calibration/mit_timeout` worker (both `dry_run` readback and RAM write).
#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn run_mit_timeout_executor_with_guard(
    sup: &std::sync::Arc<crate::supervisor::ArmSupervisor>,
    deps: &crate::calibration::CalibrationDeps,
    cmd: TimeoutCommand,
    auth: crate::calibration::ExecutorCalibAuth,
    existing_guard: Option<crate::calibration::ExecutorTxnGuard>,
) -> Result<Value, CalibError> {
    use crate::rt_core::config_iface::ConfigDomain;
    use crate::rt_core::requests::config::{
        build_config_proposal_with_deadline, pending_commit_marker_path,
        remove_pending_commit_marker, write_pending_commit_marker, ConfigAuthContext, ConfigReply,
        ConfigRequest, ReleaseConfig, ReservationGuard, ReserveConfig, ReservePrecheck,
    };

    let mut joints_arr = [0usize; crate::rt_core::mailbox::MAX_DOF];
    if cmd.joints.len() > crate::rt_core::mailbox::MAX_DOF {
        if let Some(g) = existing_guard {
            g.release().await;
        }
        return Err(invalid("too many joints"));
    }
    for (i, &j) in cmd.joints.iter().enumerate() {
        joints_arr[i] = j;
    }

    if cmd.dry_run {
        if let Some(g) = existing_guard {
            g.release().await;
        }
        let start_req = CommissioningStartRequest {
            kind: CommissioningOpKind::ReadIdentity,
            joints: joints_arr,
            joint_len: cmd.joints.len(),
            timeout_counts: cmd.timeout_counts,
            references: [0.0; crate::rt_core::mailbox::MAX_DOF],
            authority: CommissioningAuthority {
                conn_id: auth.admission.connection_id,
                lease_holder: auth.lease_holder,
                operator_grant: auth.operator_grant,
                expected_epoch: auth.admission.estop_epoch,
                expected_stop_gen: auth.admission.stop_gen,
                grant_id: None,
                txn_id: 0,
            },
            txn_deadline: std::time::Instant::now() + COMMISSIONING_TXN_TIMEOUT,
            prior_disk_mutated: false,
        };
        let snap = run_rt_commissioning_op(sup, start_req, auth.auth_check.as_ref())
            .await
            .map_err(CommissioningRunError::into_calib_error)?;
        if snap.state != CommissioningOpState::Done {
            let status = if snap.error_status != 0 {
                snap.error_status
            } else {
                502
            };
            let code = if snap.error_code.is_empty() {
                "MIT_TIMEOUT_READ"
            } else {
                snap.error_code
            };
            let msg = if snap.error_msg.is_empty() {
                "register readback failed".to_string()
            } else {
                snap.error_msg.as_str().to_string()
            };
            return Err(CalibError::new(status, code, msg));
        }
        let mut rb = Vec::with_capacity(snap.joint_len);
        for i in 0..snap.joint_len {
            let Some(id) = snap.outcomes[i].before else {
                return Err(CalibError::new(
                    502,
                    "MIT_TIMEOUT_READ",
                    "incomplete motor identity readback",
                ));
            };
            rb.push(id.into_motor_identity());
        }
        return Ok(json!({
            "dry_run": true,
            "readback": rb,
            "timeout_unit": timeout_unit_json(),
        }));
    }

    // Non-dry-run RAM TIMEOUT commissioning: hold single Calibration reservation + durable marker
    // before any REG_WRITE(RID_TIMEOUT) frame leaves the host.
    let mut txn_guard = existing_guard.ok_or_else(|| {
        CalibError::new(500, "INTERNAL", "missing calibration transaction guard")
    })?;
    let txn_id = txn_guard.txn_id();
    let cfg_auth = ConfigAuthContext {
        admission: auth.admission,
        lease_holder: auth.lease_holder,
    };
    let pub_cfg = match sup.published_config().await {
        Ok(c) => c,
        Err(e) => {
            txn_guard.release().await;
            return Err(CalibError::from(e));
        }
    };
    let lease_gen = auth.lease_holder.map_or(0, |(_, g)| g);
    let proposal = build_config_proposal_with_deadline(
        sup,
        ConfigDomain::Calibration,
        pub_cfg.calibration_revision,
        auth.admission,
        lease_gen,
        pub_cfg.workspace_revision,
        std::time::Instant::now() + COMMISSIONING_TXN_TIMEOUT,
    );
    let reservation = match sup
        .dispatch_config_request_owned_reliable(ConfigRequest::Reserve(ReserveConfig {
            proposal,
            auth: cfg_auth,
            precheck: ReservePrecheck::MitCommissioning {
                allow_during_calibration_busy: true,
                expected_txn_id: Some(txn_id),
                operator_grant: auth.operator_grant,
                require_driver: true,
            },
            expected_workspace_revision: None,
            expected_poses_revision: None,
        }))
        .await
    {
        Ok(ConfigReply::Reserved(Ok(r))) => r,
        Ok(ConfigReply::Reserved(Err(e))) => {
            txn_guard.release().await;
            return Err(crate::calibration::calib_error_from_config(e));
        }
        Err(e) => {
            txn_guard.release().await;
            return Err(crate::calibration::calib_error_from_config(e));
        }
        _ => {
            txn_guard.release().await;
            return Err(CalibError::new(
                500,
                "INTERNAL",
                "unexpected calibration reserve reply",
            ));
        }
    };
    let grant_id = reservation.grant();
    let mut res_guard = ReservationGuard::new(std::sync::Arc::clone(sup), reservation);

    // Write durable pending marker BEFORE any irreversible RAM TIMEOUT write.
    let marker_path = pending_commit_marker_path(&deps.path);
    let marker_text = json!({
        "reason": "mit_timeout_commissioning",
        "joints": cmd.joints.iter().map(|j| j + 1).collect::<Vec<_>>(),
        "timeout_counts": cmd.timeout_counts,
    })
    .to_string();
    let mp_write = marker_path.clone();
    if let Err(e) = tokio::task::spawn_blocking(move || write_pending_commit_marker(&mp_write, &marker_text))
        .await
        .map_err(|e| e.to_string())
        .and_then(|r| r)
    {
        if let Some(r) = res_guard.take() {
            let _ = sup
                .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                    reservation: r,
                }))
                .await;
        }
        txn_guard.release().await;
        return Err(CalibError::new(
            500,
            "PERSIST_FAILED",
            format!("pending commissioning marker could not be written: {e}; nothing was changed"),
        ));
    }

    // B2: Bind durable pending-marker proof to the active Reserved hold on the RT owner
    // BEFORE dispatching `StartCommissioning(WriteTimeout)`.
    let bind_res = sup
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
            if let Some(r) = res_guard.take() {
                let _ = sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                        reservation: r,
                    }))
                    .await;
            }
        } else {
            res_guard.mark_hw_mutated();
            txn_guard.mark_remote_mutated();
        }
        txn_guard.release().await;
        return Err(CalibError::new(
            409,
            "CONFIG_BUSY",
            "commissioning reservation lost before binding durable pending marker; nothing was changed",
        ));
    }

    let start_req = CommissioningStartRequest {
        kind: CommissioningOpKind::WriteTimeout,
        joints: joints_arr,
        joint_len: cmd.joints.len(),
        timeout_counts: cmd.timeout_counts,
        references: [0.0; crate::rt_core::mailbox::MAX_DOF],
        authority: CommissioningAuthority {
            conn_id: auth.admission.connection_id,
            lease_holder: auth.lease_holder,
            operator_grant: auth.operator_grant,
            expected_epoch: auth.admission.estop_epoch,
            expected_stop_gen: auth.admission.stop_gen,
            grant_id: Some(grant_id),
            txn_id,
        },
        txn_deadline: std::time::Instant::now() + COMMISSIONING_TXN_TIMEOUT,
        prior_disk_mutated: false,
    };

    let snap_res = run_rt_commissioning_op(sup, start_req, auth.auth_check.as_ref()).await;
    let snap = match snap_res {
        Ok(s) => s,
        Err(CommissioningRunError::DefinitivePreTxRefusal(err)) => {
            // RT owner definitively refused StartCommissioning before any TX could occur:
            // safe to unlink marker & release reservation.
            let mp_rm = marker_path.clone();
            let rm_ok = tokio::task::spawn_blocking(move || remove_pending_commit_marker(&mp_rm))
                .await
                .map_err(|e| e.to_string())
                .and_then(|r| r)
                .is_ok();
            if rm_ok {
                if let Some(r) = res_guard.take() {
                    let _ = sup
                        .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                            ReleaseConfig { reservation: r },
                        ))
                        .await;
                }
            } else {
                res_guard.mark_hw_mutated();
                txn_guard.mark_remote_mutated();
            }
            txn_guard.release().await;
            return Err(err);
        }
        Err(CommissioningRunError::UnknownPostStartOutcome(err)) => {
            // B3: Outcome after start submission is unknown — NEVER unlink `.calibration_commit.pending`;
            // force Reconciling fail-closed so the restart fence survives process exit.
            res_guard.mark_hw_mutated();
            txn_guard.mark_remote_mutated();
            let _ = sup
                .dispatch_config_request_reliable(|| {
                    ConfigRequest::ForceReconcile(ConfigDomain::Calibration)
                })
                .await;
            txn_guard.release().await;
            return Err(err);
        }
    };

    let mut applied_joints = Vec::new();
    let mut comm_rows = Vec::new();
    let mut uncertain_joints = Vec::new();
    let mut not_attempted_joints = Vec::new();
    for i in 0..snap.joint_len {
        let o = &snap.outcomes[i];
        let j1 = o.joint + 1;
        match o.disposition {
            JointCommissioningDisposition::Applied => {
                applied_joints.push(j1);
                if let (Some(before), Some(readback)) = (o.before, o.readback) {
                    comm_rows.push(TimeoutCommissioning {
                        before: before.into_motor_identity(),
                        written_raw: o.written_raw,
                        ack_raw: o.ack_raw,
                        readback: readback.into_motor_identity(),
                        unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
                        unit_verified: false,
                    });
                }
            }
            JointCommissioningDisposition::Uncertain => uncertain_joints.push(j1),
            JointCommissioningDisposition::Unattempted => not_attempted_joints.push(j1),
        }
    }

    let all_applied = snap.state == CommissioningOpState::Done
        && uncertain_joints.is_empty()
        && not_attempted_joints.is_empty()
        && comm_rows.len() == snap.joint_len;

    if all_applied {
        let mp_rm = marker_path.clone();
        if let Err(rm_err) = tokio::task::spawn_blocking(move || remove_pending_commit_marker(&mp_rm))
            .await
            .map_err(|e| e.to_string())
            .and_then(|r| r)
        {
            res_guard.mark_hw_mutated();
            txn_guard.mark_remote_mutated();
            let _ = sup
                .dispatch_config_request_reliable(|| {
                    ConfigRequest::ForceReconcile(ConfigDomain::Calibration)
                })
                .await;
            txn_guard.release().await;
            return Err(CalibError::new(
                500,
                "PERSIST_FAILED",
                format!("TIMEOUT commissioned but marker cleanup failed ({rm_err}); reconciling required"),
            ));
        }
        if let Some(r) = res_guard.take() {
            let _ = sup
                .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                    reservation: r,
                }))
                .await;
        }
        txn_guard.release().await;
        tracing::warn!(
            "MIT TIMEOUT commissioned (RAM, executor) for joints {:?}: {} counts",
            applied_joints,
            cmd.timeout_counts
        );
        return Ok(json!({
            "dry_run": false,
            "timeout_counts": cmd.timeout_counts,
            "commissioning": comm_rows,
            "timeout_unit": timeout_unit_json(),
        }));
    }

    // Failure, partial batch, cancellation, or uncertain TX.
    if snap.hw_mutated {
        // B3: Hardware RAM TIMEOUT may have mutated on >=1 joint. Keep pending marker AND
        // HoldState::Reconciling so motion & rearm stay fenced until reconciled.
        res_guard.mark_hw_mutated();
        txn_guard.mark_remote_mutated();
        let _ = sup
            .dispatch_config_request_reliable(|| {
                ConfigRequest::ForceReconcile(ConfigDomain::Calibration)
            })
            .await;
        if !uncertain_joints.is_empty() || (!applied_joints.is_empty() && !not_attempted_joints.is_empty()) {
            let fault_msg = format!(
                "MIT TIMEOUT commissioning incomplete: applied {applied_joints:?}, uncertain {uncertain_joints:?}, not attempted {not_attempted_joints:?}; power-cycle or re-commission"
            );
            let bounded = crate::safety::BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::new(&fault_msg);
            let _ = sup
                .dispatch_config_request_reliable(|| ConfigRequest::LatchConfigReconcileFault {
                    domain: ConfigDomain::Calibration,
                    message: bounded,
                })
                .await;
        }
        drop(res_guard);
    } else {
        // Zero irreversible TX left host: clean up pending marker and release hold cleanly.
        let mp_rm = marker_path.clone();
        let rm_ok = tokio::task::spawn_blocking(move || remove_pending_commit_marker(&mp_rm))
            .await
            .map_err(|e| e.to_string())
            .and_then(|r| r)
            .is_ok();
        if rm_ok {
            if let Some(r) = res_guard.take() {
                let _ = sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                        reservation: r,
                    }))
                    .await;
            }
        } else {
            res_guard.mark_hw_mutated();
            txn_guard.mark_remote_mutated();
            drop(res_guard);
        }
    }
    txn_guard.release().await;

    let base_err = if snap.error_msg.is_empty() {
        "MIT TIMEOUT commissioning failed".to_string()
    } else {
        snap.error_msg.as_str().to_string()
    };
    let status = if snap.error_status != 0 {
        snap.error_status
    } else {
        502
    };
    let code = if snap.error_code.is_empty() {
        "MIT_TIMEOUT_FAILED"
    } else {
        snap.error_code
    };
    let message = if code == "MIT_TIMEOUT_FAILED" || snap.hw_mutated {
        format!(
            "{base_err}; applied {applied_joints:?}, uncertain {uncertain_joints:?} (write sent, not confirmed), not attempted {not_attempted_joints:?}"
        )
    } else {
        base_err
    };
    let mut err = CalibError::new(status, code, message);
    if code == "MIT_TIMEOUT_FAILED" || snap.hw_mutated {
        err.extra.insert("applied".into(), json!(applied_joints));
        err.extra.insert("commissioning".into(), json!(comm_rows));
        err.extra.insert("uncertain".into(), json!(uncertain_joints));
        err.extra
            .insert("not_attempted".into(), json!(not_attempted_joints));
    }
    Err(err)
}

/// Synchronous preflight for executor-mode WS `/api/v1/calibration/mit_qualification`.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn check_mit_qualification_executor(
    sup: &std::sync::Arc<crate::supervisor::ArmSupervisor>,
    body: &Value,
    auth: &crate::calibration::ExecutorCalibAuth,
) -> Result<(MitQualificationFile, crate::calibration::ExecutorTxnGuard), CalibError> {
    use crate::rt_core::requests::config::{
        next_calibration_txn_id, CalibrationTxnKind, ConfigAuthContext, ConfigReply, ConfigRequest,
    };
    let o = body
        .as_object()
        .ok_or_else(|| invalid("Command body must be an object"))?;
    if o.keys().any(|k| k != "record") {
        return Err(invalid("the body holds only `record`"));
    }
    let rec: MitQualificationFile =
        serde_json::from_value(o.get("record").cloned().unwrap_or(Value::Null))
            .map_err(|e| invalid(format!("record: {e}")))?;
    rec.validate().map_err(invalid)?;
    if rec.invalidated.is_some() {
        return Err(invalid("record: invalidated records cannot be stored"));
    }
    let pub_cfg = sup.published_config().await.map_err(CalibError::from)?;
    if !pub_cfg.has_driver {
        return Err(CalibError::new(400, "UNSUPPORTED", "no motor driver"));
    }
    let arm = pub_cfg.config.arm_joint_indices();
    for j in &rec.joints {
        let idx = j.joint.saturating_sub(1);
        if j.joint == 0 || !arm.contains(&idx) {
            return Err(invalid(format!("record: J{} is not an arm joint", j.joint)));
        }
    }
    if let Err((_, arm_err)) = (auth.auth_check)() {
        return Err(CalibError::from(arm_err));
    }
    let txn_id = next_calibration_txn_id();
    let cfg_auth = ConfigAuthContext {
        admission: auth.admission,
        lease_holder: auth.lease_holder,
    };
    let pre_guard = crate::calibration::ExecutorTxnGuard::new(std::sync::Arc::clone(sup), txn_id);
    match sup
        .dispatch_config_request(ConfigRequest::BeginCalibrationTxn {
            txn_id,
            auth: cfg_auth,
            operator_grant: auth.operator_grant,
            expected_epoch: auth.admission.estop_epoch,
            expected_stop_gen: auth.admission.stop_gen,
            kind: CalibrationTxnKind::MitQualification,
        })
        .await
    {
        Ok(ConfigReply::CalibrationTxnBegan(Ok(_))) => {
            sup.published_config_hub().update(|cfg| {
                cfg.calibration_busy = true;
                cfg.calibration_txn_id = Some(txn_id);
            });
            Ok((rec, pre_guard))
        }
        Ok(ConfigReply::CalibrationTxnBegan(Err(e))) => {
            let _ = pre_guard.disarm();
            Err(crate::calibration::calib_error_from_config(e))
        }
        Err(e) => {
            pre_guard.release().await;
            Err(crate::calibration::calib_error_from_config(e))
        }
        _ => {
            pre_guard.release().await;
            Err(CalibError::new(
                500,
                "INTERNAL",
                "unexpected begin mit_qualification reply",
            ))
        }
    }
}

/// Executor-mode WS `/api/v1/calibration/mit_qualification` worker:
/// Holds ONE single `ConfigDomain::Calibration` reservation across RT identity readback,
/// `check_gate` (using actual driver `supports_mit_switch`), staging, both commit boundaries,
/// and `PublishedConfigHub` update, releasing `calibration_busy` only after publication.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn run_mit_qualification_executor_with_guard(
    sup: &std::sync::Arc<crate::supervisor::ArmSupervisor>,
    deps: &crate::calibration::CalibrationDeps,
    rec: MitQualificationFile,
    auth: crate::calibration::ExecutorCalibAuth,
    existing_guard: Option<crate::calibration::ExecutorTxnGuard>,
) -> Result<Value, CalibError> {
    use crate::rt_core::config_iface::ConfigDomain;
    use crate::rt_core::requests::config::{
        build_config_proposal_with_deadline, pending_commit_marker_path,
        run_config_commit_on_held_reservation, ConfigAuthContext, ConfigCandidate, ConfigCommitPlan,
        ConfigReply, ConfigRequest, ConfigWriteOp, FileWriteOp, ReleaseConfig, ReservationGuard,
        ReserveConfig, ReservePrecheck,
    };

    let txn_guard = existing_guard.ok_or_else(|| {
        CalibError::new(500, "INTERNAL", "missing calibration transaction guard")
    })?;
    let txn_id = txn_guard.txn_id();
    let cfg_auth = ConfigAuthContext {
        admission: auth.admission,
        lease_holder: auth.lease_holder,
    };

    let run_res = async {
        let pub_cfg = sup.published_config().await.map_err(CalibError::from)?;
        let joints: Vec<usize> = rec.joints.iter().map(|j| j.joint - 1).collect();
        if joints.len() > crate::rt_core::mailbox::MAX_DOF {
            return Err(invalid("too many joints in qualification record"));
        }
        let mut joints_arr = [0usize; crate::rt_core::mailbox::MAX_DOF];
        for (i, &j) in joints.iter().enumerate() {
            joints_arr[i] = j;
        }

        // Acquire single uninterrupted ConfigDomain::Calibration reservation BEFORE readback (M4).
        let lease_gen = auth.lease_holder.map_or(0, |(_, g)| g);
        let proposal = build_config_proposal_with_deadline(
            sup,
            ConfigDomain::Calibration,
            pub_cfg.calibration_revision,
            auth.admission,
            lease_gen,
            pub_cfg.workspace_revision,
            std::time::Instant::now() + COMMISSIONING_TXN_TIMEOUT,
        );
        let reservation = match sup
            .dispatch_config_request_owned_reliable(ConfigRequest::Reserve(ReserveConfig {
                proposal,
                auth: cfg_auth,
                precheck: ReservePrecheck::MitCommissioning {
                    allow_during_calibration_busy: true,
                    expected_txn_id: Some(txn_id),
                    operator_grant: auth.operator_grant,
                    require_driver: true,
                },
                expected_workspace_revision: None,
                expected_poses_revision: None,
            }))
            .await
        {
            Ok(ConfigReply::Reserved(Ok(r))) => r,
            Ok(ConfigReply::Reserved(Err(e))) => {
                return Err(crate::calibration::calib_error_from_config(e));
            }
            Err(e) => return Err(crate::calibration::calib_error_from_config(e)),
            _ => {
                return Err(CalibError::new(
                    500,
                    "INTERNAL",
                    "unexpected calibration reserve reply",
                ));
            }
        };
        let grant_id = reservation.grant();
        let mut res_guard = ReservationGuard::new(std::sync::Arc::clone(sup), reservation);

        // Fresh RT register readback under the held reservation.
        let start_req = CommissioningStartRequest {
            kind: CommissioningOpKind::ReadIdentity,
            joints: joints_arr,
            joint_len: joints.len(),
            timeout_counts: 0,
            references: [0.0; crate::rt_core::mailbox::MAX_DOF],
            authority: CommissioningAuthority {
                conn_id: auth.admission.connection_id,
                lease_holder: auth.lease_holder,
                operator_grant: auth.operator_grant,
                expected_epoch: auth.admission.estop_epoch,
                expected_stop_gen: auth.admission.stop_gen,
                grant_id: Some(grant_id),
                txn_id,
            },
            txn_deadline: std::time::Instant::now() + COMMISSIONING_TXN_TIMEOUT,
            prior_disk_mutated: false,
        };
        let snap = match run_rt_commissioning_op(sup, start_req, auth.auth_check.as_ref()).await {
            Ok(s) => s,
            Err(e) => {
                if let Some(r) = res_guard.take() {
                    let _ = sup
                        .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                            ReleaseConfig { reservation: r },
                        ))
                        .await;
                }
                return Err(e.into_calib_error());
            }
        };
        if snap.state != CommissioningOpState::Done {
            if let Some(r) = res_guard.take() {
                let _ = sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                        reservation: r,
                    }))
                    .await;
            }
            let status = if snap.error_status != 0 {
                snap.error_status
            } else {
                502
            };
            let code = if snap.error_code.is_empty() {
                "MIT_TIMEOUT_READ"
            } else {
                snap.error_code
            };
            let msg = if snap.error_msg.is_empty() {
                "register readback failed".to_string()
            } else {
                snap.error_msg.as_str().to_string()
            };
            return Err(CalibError::new(status, code, msg));
        }
        let mut rb = Vec::with_capacity(snap.joint_len);
        for i in 0..snap.joint_len {
            let Some(id) = snap.outcomes[i].before else {
                if let Some(r) = res_guard.take() {
                    let _ = sup
                        .dispatch_config_request_owned_reliable(ConfigRequest::Release(
                            ReleaseConfig { reservation: r },
                        ))
                        .await;
                }
                return Err(CalibError::new(
                    502,
                    "MIT_TIMEOUT_READ",
                    "incomplete motor identity readback",
                ));
            };
            rb.push(id.into_motor_identity());
        }

        // Validate qualification gate against fresh readback, active profile, active zero offsets,
        // and actual driver `supports_mit_switch` (M4).
        let pub_now = sup.published_config().await.map_err(CalibError::from)?;
        let mut cfg = pub_now.config.clone();
        if let Some(jt) = cfg.joint_tracking.as_mut() {
            for j in jt.joints.iter_mut() {
                j.strategy = TrackingStrategy::Position;
            }
        }
        if let Err(e) = check_gate(
            &cfg,
            Some(&rec),
            &pub_now.joint_zero_offsets,
            &joints,
            &rb,
            snap.supports_mit_switch,
        ) {
            if let Some(r) = res_guard.take() {
                let _ = sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                        reservation: r,
                    }))
                    .await;
            }
            return Err(CalibError::new(409, "MIT_QUALIFICATION_MISMATCH", e));
        }

        let mut file = pub_now.calibration.clone().unwrap_or_default();
        file.set_mit_record(&rec);
        if let Err(e) = file.validate() {
            if let Some(r) = res_guard.take() {
                let _ = sup
                    .dispatch_config_request_owned_reliable(ConfigRequest::Release(ReleaseConfig {
                        reservation: r,
                    }))
                    .await;
            }
            return Err(invalid(e));
        }
        let text = serde_json::to_string_pretty(&file)
            .map_err(|e| CalibError::new(500, "INTERNAL", e.to_string()))?;
        let marker_path = pending_commit_marker_path(&deps.path);
        let marker_text = json!({
            "reason": "mit_qualification",
            "files": [deps.path.display().to_string()],
        })
        .to_string();

        let pub_cal = Some(file.clone());
        #[cfg(feature = "policy-runtime")]
        let calibration_digest =
            crate::supervisor::SupervisorInner::calibration_digest_of(pub_cal.as_ref());

        let held_reservation = res_guard
            .take()
            .expect("reservation held through commit handoff");
        let plan = ConfigCommitPlan {
            proposal,
            auth: cfg_auth,
            precheck: ReservePrecheck::MitCommissioning {
                allow_during_calibration_busy: true,
                expected_txn_id: Some(txn_id),
                operator_grant: auth.operator_grant,
                require_driver: true,
            },
            candidate: ConfigCandidate::MitQualification {
                calibration: Some(file),
                #[cfg(feature = "policy-runtime")]
                calibration_digest,
                expected_txn_id: Some(txn_id),
                operator_grant: auth.operator_grant,
            },
            write: ConfigWriteOp::Batch {
                ops: vec![FileWriteOp::Replace(crate::persist::PendingWrite::new(
                    deps.path.clone(),
                    text,
                ))],
                pending_marker: Some((marker_path, marker_text)),
                existing_marker: None,
            },
            expected_workspace_revision: None,
            expected_poses_revision: None,
        };

        let auth_cb = std::sync::Arc::clone(&auth.auth_check);
        run_config_commit_on_held_reservation(
            sup,
            plan,
            held_reservation,
            false,
            move || (auth_cb)(),
            move |cfg, act| {
                cfg.calibration = pub_cal;
                cfg.calibration_load_error = None;
                cfg.calibration_revision = act.calibration_revision;
            },
        )
        .await
        .map_err(|e| crate::calibration::calib_error_from_commit(e, "mit_qualification", None))?;

        let one_based: Vec<usize> = rec.joints.iter().map(|j| j.joint).collect();
        tracing::warn!("MIT qualification stored (executor) for joints {one_based:?}");
        Ok(json!({"stored": true, "joints": one_based}))
    }
    .await;

    txn_guard.release().await;
    run_res
}
