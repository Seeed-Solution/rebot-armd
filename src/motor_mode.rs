//! Semantic motor control modes, typed per-mode limits, and the vendor capability table.
//!
//! This module is intentionally transport- and `wasm32`-independent so profile parsing,
//! load-time validation, and vendor capability queries share one authoritative model across
//! `armd`, tests, and `armd.wasm`.

use std::f32::consts::PI;
use std::fmt;

use serde::de::{self, Deserializer, MapAccess, Visitor};
use serde::ser::{SerializeStruct, Serializer};
use serde::{Deserialize, Serialize};
use serde_json::value::RawValue;

use crate::model::{DamiaoGripperDrive, DamiaoGripperMode};

/// Command-semantic control mode of one joint, independent of motor vendor or model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode {
    /// Position setpoint (e.g. Feetech STS3215 goal position, FashionStar set-angle).
    Position,
    /// Position setpoint with a per-frame velocity ceiling (e.g. Damiao `POS_VEL`, `CTRL_MODE = 2`).
    PositionVelocity,
    /// Joint-space impedance / MIT command carrying position, velocity, `kp`, `kd`, and
    /// feedforward torque (e.g. Damiao `MIT` `CTRL_MODE = 1`, RobStride Type-1 motion control).
    Impedance,
    /// Position setpoint with per-frame velocity and motor current/force ceilings
    /// (e.g. Damiao `POS_FORCE`, `CTRL_MODE = 4`).
    PositionForce,
}

impl ControlMode {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Position => "position",
            Self::PositionVelocity => "position_velocity",
            Self::Impedance => "impedance",
            Self::PositionForce => "position_force",
        }
    }
}

impl fmt::Display for ControlMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

// =========================================================================
// Typed per-mode limits
// =========================================================================

/// Maximum 12-bit bus-servo encoder step (`0..=4095`, Feetech STS3215 / FashionStar magnetic encoder).
pub const MAX_12BIT_STEP: u16 = 4095;

/// Default FashionStar `CMD_SET_SERVO_ANGLE` move interval (ms) used by `FashionStarArmDriver`.
pub const FASHIONSTAR_DEFAULT_INTERVAL_MS: u16 = 20;

/// Default FashionStar `CMD_SET_SERVO_ANGLE` power parameter used by `FashionStarArmDriver`.
pub const FASHIONSTAR_DEFAULT_POWER: u16 = 1000;

/// Maximum wire magnitude for FashionStar `i16` angle in 0.1 degree units (rad, `3276.7 deg`).
pub const FASHIONSTAR_MAX_ANGLE_RAD: f32 = 3276.7 * PI / 180.0;

/// Default Damiao `POS_VEL` velocity ceiling (`DamiaoArmDriver::new`, rad/s).
pub const DAMIAO_DEFAULT_POS_VEL_RAD_S: f32 = 2.0;

/// Default Damiao wire mapping ranges (`DamiaoLimits::default()`).
pub const DAMIAO_DEFAULT_P_MAX: f32 = 12.5;
pub const DAMIAO_DEFAULT_V_MAX: f32 = 30.0;
pub const DAMIAO_DEFAULT_T_MAX: f32 = 18.0;

/// Default RobStride Type-1 wire mapping ranges (`RobStrideLimits::default()`).
pub const ROBSTRIDE_DEFAULT_P_MAX: f32 = 4.0 * PI;
pub const ROBSTRIDE_DEFAULT_V_MAX: f32 = 30.0;
pub const ROBSTRIDE_DEFAULT_T_MAX: f32 = 18.0;

/// Wire quantization ceilings for `kp` and `kd` shared by Damiao MIT and RobStride Type-1.
pub const IMPEDANCE_WIRE_KP_MAX: f32 = 500.0;
pub const IMPEDANCE_WIRE_KD_MAX: f32 = 5.0;

/// Full scale of Damiao `POS_FORCE` `i_des` field (`1/10000` of maximum phase current).
pub const POS_FORCE_CURRENT_MAX_RAW: u16 = 10_000;

/// Highest `f32` speed representable on the Damiao `POS_FORCE` `u16` wire field (`vel_raw = 65535`).
///
/// Defined via [`DamiaoGripperDrive::wire_speed_f32`] so `u16::MAX` (`0xFFFF`) is reachable in `f32`
/// without rejecting the smallest `f32` that floors back to `65535` (whereas `655.35_f32` is
/// slightly below `655.35` and floors to `65534`).
pub fn pos_force_max_wire_speed_f32() -> f32 {
    DamiaoGripperDrive::wire_speed_f32(u16::MAX)
}

/// Predicate verifying that `[min_val, max_val]` will not overflow `span = max_val - min_val`
/// or the intermediate `span * ((1 << bits) - 1)` in `f32` linear quantization / de-quantization.
pub fn is_valid_quantized_range(min_val: f32, max_val: f32, bits: u32) -> bool {
    if !(min_val.is_finite() && max_val.is_finite() && min_val < max_val) {
        return false;
    }
    if bits == 0 || bits > 16 {
        return false;
    }
    let span = max_val - min_val;
    if !span.is_finite() || span <= 0.0 {
        return false;
    }
    let max_int = ((1u32 << bits) - 1) as f32;
    (span * max_int).is_finite()
}

/// Typed limits for [`ControlMode::Position`].
///
/// All fields are optional: when omitted, the vendor binding supplies its defaults (e.g.
/// Feetech step limits from `feetech.joints[i]`, FashionStar `interval_ms = 20` / `power = 1000`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PositionLimits {
    /// Optional command velocity bound (rad/s, positive finite).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_velocity_rad_s: Option<f32>,
    /// Optional 12-bit bus-servo step range `[min_step, max_step]` (`0 <= min < max <= 4095`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step_range: Option<[u16; 2]>,
    /// Optional move duration interval (ms, `>= 1`) for interval-timed position servos (FashionStar).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub interval_ms: Option<u16>,
    /// Optional servo drive power limit (`>= 1`) for power-capped position servos (FashionStar).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub power_limit: Option<u16>,
}

impl PositionLimits {
    pub fn validate(&self) -> Result<(), String> {
        if let Some(v) = self.max_velocity_rad_s {
            if !v.is_finite() || v <= 0.0 {
                return Err(format!("position.max_velocity_rad_s must be positive finite, got {v}"));
            }
        }
        if let Some([min_s, max_s]) = self.step_range {
            if min_s >= max_s || max_s > MAX_12BIT_STEP {
                return Err(format!(
                    "position.step_range [{min_s}, {max_s}] invalid (need 0 <= min < max <= {MAX_12BIT_STEP})"
                ));
            }
        }
        if let Some(ms) = self.interval_ms {
            if ms == 0 {
                return Err("position.interval_ms must be >= 1".into());
            }
        }
        if let Some(p) = self.power_limit {
            if p == 0 {
                return Err("position.power_limit must be >= 1".into());
            }
        }
        Ok(())
    }
}

/// Typed limits for [`ControlMode::PositionVelocity`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PositionVelocityLimits {
    /// Per-frame velocity ceiling carried on `POS_VEL` frames (rad/s, positive finite).
    pub max_velocity_rad_s: f32,
}

impl PositionVelocityLimits {
    pub const fn damiao_default() -> Self {
        Self {
            max_velocity_rad_s: DAMIAO_DEFAULT_POS_VEL_RAD_S,
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        if !self.max_velocity_rad_s.is_finite() || self.max_velocity_rad_s <= 0.0 {
            return Err(format!(
                "position_velocity.max_velocity_rad_s must be positive finite, got {}",
                self.max_velocity_rad_s
            ));
        }
        Ok(())
    }
}

/// Typed limits for [`ControlMode::Impedance`].
///
/// `p_max`, `v_max`, and `t_max` are the symmetric wire mapping ranges (`[-max, +max]`);
/// `kp_max` and `kd_max` are operational gain ceilings within the wire range
/// (`[0, 500]` and `[0, 5]`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImpedanceLimits {
    /// Symmetric wire position mapping range `[-p_max, +p_max]` (rad, positive finite).
    pub p_max: f32,
    /// Symmetric wire velocity mapping range `[-v_max, +v_max]` (rad/s, positive finite).
    pub v_max: f32,
    /// Symmetric wire torque mapping range `[-t_max, +t_max]` (N·m, positive finite).
    pub t_max: f32,
    /// Operational stiffness gain ceiling (`0.0..=500.0`).
    pub kp_max: f32,
    /// Operational damping gain ceiling (`0.0..=5.0`).
    pub kd_max: f32,
    /// Optional operational torque/effort limit (N·m, positive finite `<= t_max`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub effort_limit_nm: Option<f32>,
}

impl ImpedanceLimits {
    pub const fn damiao_default() -> Self {
        Self {
            p_max: DAMIAO_DEFAULT_P_MAX,
            v_max: DAMIAO_DEFAULT_V_MAX,
            t_max: DAMIAO_DEFAULT_T_MAX,
            kp_max: IMPEDANCE_WIRE_KP_MAX,
            kd_max: IMPEDANCE_WIRE_KD_MAX,
            effort_limit_nm: None,
        }
    }

    pub const fn robstride_default() -> Self {
        Self {
            p_max: ROBSTRIDE_DEFAULT_P_MAX,
            v_max: ROBSTRIDE_DEFAULT_V_MAX,
            t_max: ROBSTRIDE_DEFAULT_T_MAX,
            kp_max: IMPEDANCE_WIRE_KP_MAX,
            kd_max: IMPEDANCE_WIRE_KD_MAX,
            effort_limit_nm: None,
        }
    }

    /// Effective operational torque/effort ceiling (`effort_limit_nm` clamped to `t_max`).
    pub fn effective_effort_limit_nm(&self) -> f32 {
        self.effort_limit_nm
            .map_or(self.t_max, |eff| eff.min(self.t_max))
    }

    pub fn validate(&self) -> Result<(), String> {
        if !is_valid_quantized_range(-self.p_max, self.p_max, 16)
            || !is_valid_quantized_range(-self.v_max, self.v_max, 16)
            || !is_valid_quantized_range(-self.t_max, self.t_max, 16)
        {
            return Err(format!(
                "impedance p_max/v_max/t_max must be positive finite quantized ranges, got ({}, {}, {})",
                self.p_max, self.v_max, self.t_max
            ));
        }
        if !self.kp_max.is_finite() || self.kp_max < 0.0 || self.kp_max > IMPEDANCE_WIRE_KP_MAX {
            return Err(format!(
                "impedance.kp_max must be in 0..={IMPEDANCE_WIRE_KP_MAX}, got {}",
                self.kp_max
            ));
        }
        if !self.kd_max.is_finite() || self.kd_max < 0.0 || self.kd_max > IMPEDANCE_WIRE_KD_MAX {
            return Err(format!(
                "impedance.kd_max must be in 0..={IMPEDANCE_WIRE_KD_MAX}, got {}",
                self.kd_max
            ));
        }
        if let Some(eff) = self.effort_limit_nm {
            if !eff.is_finite() || eff <= 0.0 || eff > self.t_max {
                return Err(format!(
                    "impedance.effort_limit_nm ({eff}) must be in (0, t_max={}]",
                    self.t_max
                ));
            }
        }
        Ok(())
    }
}

/// Typed limits for [`ControlMode::PositionForce`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PositionForceLimits {
    /// Current ceiling on every `POS_FORCE` frame in `1/10000` of maximum phase current (`1..=10000`).
    pub current_ceiling_raw: u16,
    /// Closing speed ceiling (rad/s); must quantize to `>= 0.01 rad/s` and fit the `u16` wire field (`<= 655.35 rad/s`).
    pub close_speed_rad_s: f32,
    /// Optional overall speed ceiling (e.g. opening speed limit, rad/s); when set must be `>= close_speed_rad_s` and `<= 655.35 rad/s`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_speed_rad_s: Option<f32>,
    /// Whether the torque equivalent of `current_ceiling_raw` was commissioned on hardware.
    #[serde(default)]
    pub calibrated: bool,
    /// Measured torque equivalent of `current_ceiling_raw` (N·m); required positive finite when `calibrated == true`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ceiling_torque_nm: Option<f32>,
    /// Torque reference class: `"external"` or `"feedback_equivalent"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub torque_reference: Option<String>,
    /// Bounded staleness fault limit (ms, `200..=450` ms; default `250` ms).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feedback_stale_fault_ms: Option<u64>,
}

impl PositionForceLimits {
    pub fn from_damiao_gripper(g: &DamiaoGripperDrive) -> Self {
        Self {
            current_ceiling_raw: g.current_ceiling_raw,
            close_speed_rad_s: g.close_speed_rad_s,
            max_speed_rad_s: None,
            calibrated: g.calibrated,
            ceiling_torque_nm: g.ceiling_torque_nm,
            torque_reference: g.torque_reference.clone(),
            feedback_stale_fault_ms: g.feedback_stale_fault_ms,
        }
    }

    pub fn to_damiao_gripper(&self) -> DamiaoGripperDrive {
        DamiaoGripperDrive {
            mode: DamiaoGripperMode::PosForce,
            current_ceiling_raw: self.current_ceiling_raw,
            close_speed_rad_s: self.close_speed_rad_s,
            calibrated: self.calibrated,
            ceiling_torque_nm: self.ceiling_torque_nm,
            torque_reference: self.torque_reference.clone(),
            feedback_stale_fault_ms: self.feedback_stale_fault_ms,
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        if !(1..=POS_FORCE_CURRENT_MAX_RAW).contains(&self.current_ceiling_raw) {
            return Err(format!(
                "position_force.current_ceiling_raw must be 1..={POS_FORCE_CURRENT_MAX_RAW}, got {}",
                self.current_ceiling_raw
            ));
        }
        if !self.close_speed_rad_s.is_finite() || self.close_speed_rad_s <= 0.0 {
            return Err(format!(
                "position_force.close_speed_rad_s must be positive finite, got {}",
                self.close_speed_rad_s
            ));
        }
        let max_wire_speed = pos_force_max_wire_speed_f32();
        if self.close_speed_rad_s > max_wire_speed {
            return Err(format!(
                "position_force.close_speed_rad_s ({}) exceeds wire maximum {max_wire_speed} rad/s",
                self.close_speed_rad_s
            ));
        }
        if DamiaoGripperDrive::quantize_vel_raw(self.close_speed_rad_s) == 0 {
            return Err(
                "position_force.close_speed_rad_s encodes to 0 on the wire (must quantize to >= 0.01 rad/s)".into(),
            );
        }
        if let Some(max_v) = self.max_speed_rad_s {
            if !max_v.is_finite() || max_v <= 0.0 {
                return Err(format!(
                    "position_force.max_speed_rad_s must be positive finite, got {max_v}"
                ));
            }
            if max_v > max_wire_speed {
                return Err(format!(
                    "position_force.max_speed_rad_s ({max_v}) exceeds wire maximum {max_wire_speed} rad/s"
                ));
            }
            if DamiaoGripperDrive::quantize_vel_raw(max_v) == 0 {
                return Err(
                    "position_force.max_speed_rad_s encodes to 0 on the wire (must quantize to >= 0.01 rad/s)".into(),
                );
            }
            if max_v < self.close_speed_rad_s {
                return Err(format!(
                    "position_force.max_speed_rad_s ({max_v}) must be >= close_speed_rad_s ({})",
                    self.close_speed_rad_s
                ));
            }
        }
        if self.calibrated && !self.ceiling_torque_nm.is_some_and(|t| t.is_finite() && t > 0.0) {
            return Err("position_force: calibrated needs a positive finite ceiling_torque_nm".into());
        }
        if let Some(t) = self.ceiling_torque_nm {
            if !t.is_finite() || t <= 0.0 {
                return Err(format!("position_force.ceiling_torque_nm must be positive finite, got {t}"));
            }
        }
        if let Some(r) = &self.torque_reference {
            if r != "external" && r != "feedback_equivalent" {
                return Err(
                    "position_force.torque_reference must be \"external\" or \"feedback_equivalent\"".into(),
                );
            }
        }
        if let Some(ms) = self.feedback_stale_fault_ms {
            if !(DamiaoGripperDrive::MIN_FEEDBACK_STALE_FAULT_MS
                ..=DamiaoGripperDrive::MAX_FEEDBACK_STALE_FAULT_MS)
                .contains(&ms)
            {
                return Err(format!(
                    "position_force.feedback_stale_fault_ms must be {}..={} ms, got {ms}",
                    DamiaoGripperDrive::MIN_FEEDBACK_STALE_FAULT_MS,
                    DamiaoGripperDrive::MAX_FEEDBACK_STALE_FAULT_MS
                ));
            }
        }
        Ok(())
    }
}

/// Discriminated union binding each [`ControlMode`] to its typed limit structure.
#[derive(Debug, Clone, PartialEq)]
pub enum ModeLimits {
    Position(PositionLimits),
    PositionVelocity(PositionVelocityLimits),
    Impedance(ImpedanceLimits),
    PositionForce(PositionForceLimits),
}

impl ModeLimits {
    pub const fn mode(&self) -> ControlMode {
        match self {
            Self::Position(_) => ControlMode::Position,
            Self::PositionVelocity(_) => ControlMode::PositionVelocity,
            Self::Impedance(_) => ControlMode::Impedance,
            Self::PositionForce(_) => ControlMode::PositionForce,
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        match self {
            Self::Position(l) => l.validate(),
            Self::PositionVelocity(l) => l.validate(),
            Self::Impedance(l) => l.validate(),
            Self::PositionForce(l) => l.validate(),
        }
    }
}

// =========================================================================
// Profile per-joint configuration schema
// =========================================================================

/// Optional per-joint control-mode declaration in a profile (`ArmConfig::control_mode`).
///
/// JSON layout:
/// ```json
/// {
///   "mode": "position_force",
///   "vendor": "damiao",
///   "runtime_modes": [],
///   "limits": {
///     "current_ceiling_raw": 460,
///     "close_speed_rad_s": 0.3
///   }
/// }
/// ```
///
/// `limits` is parsed strictly according to `mode`. Omitting `limits` is permitted only for
/// `position` (which defaults to [`PositionLimits::default()`]); `position_velocity`, `impedance`,
/// and `position_force` require explicit mode-specific limits and fail validation when omitted.
#[derive(Debug, Clone, PartialEq)]
pub struct JointControlModeConfig {
    pub mode: ControlMode,
    pub limits: Option<ModeLimits>,
    /// Optional explicit vendor binding; checked against the active follower and capability table.
    pub vendor: Option<VendorBinding>,
    /// Optional permitted runtime mode switches from this base mode.
    /// `None` inherits from existing profile policies (`gravity_comp` / `joint_tracking`);
    /// `Some(vec![])` explicitly forbids runtime switching on this joint.
    pub runtime_modes: Option<Vec<ControlMode>>,
}

impl JointControlModeConfig {
    pub fn position(limits: PositionLimits) -> Self {
        Self {
            mode: ControlMode::Position,
            limits: Some(ModeLimits::Position(limits)),
            vendor: None,
            runtime_modes: None,
        }
    }

    pub fn position_velocity(limits: PositionVelocityLimits) -> Self {
        Self {
            mode: ControlMode::PositionVelocity,
            limits: Some(ModeLimits::PositionVelocity(limits)),
            vendor: None,
            runtime_modes: None,
        }
    }

    pub fn impedance(limits: ImpedanceLimits) -> Self {
        Self {
            mode: ControlMode::Impedance,
            limits: Some(ModeLimits::Impedance(limits)),
            vendor: None,
            runtime_modes: None,
        }
    }

    pub fn position_force(limits: PositionForceLimits) -> Self {
        Self {
            mode: ControlMode::PositionForce,
            limits: Some(ModeLimits::PositionForce(limits)),
            vendor: None,
            runtime_modes: None,
        }
    }

    pub fn with_vendor(mut self, vendor: VendorBinding) -> Self {
        self.vendor = Some(vendor);
        self
    }

    pub fn with_runtime_modes(mut self, runtime_modes: Vec<ControlMode>) -> Self {
        self.runtime_modes = Some(runtime_modes);
        self
    }
}

impl<'de> Deserialize<'de> for JointControlModeConfig {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct JointControlModeVisitor;

        impl<'de> Visitor<'de> for JointControlModeVisitor {
            type Value = JointControlModeConfig;

            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("a joint control_mode object")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: MapAccess<'de>,
            {
                let mut mode: Option<ControlMode> = None;
                let mut raw_limits: Option<Option<Box<RawValue>>> = None;
                let mut vendor: Option<Option<VendorBinding>> = None;
                let mut runtime_modes: Option<Option<Vec<ControlMode>>> = None;

                while let Some(key) = map.next_key::<String>()? {
                    match key.as_str() {
                        "mode" => {
                            if mode.is_some() {
                                return Err(de::Error::duplicate_field("mode"));
                            }
                            mode = Some(map.next_value()?);
                        }
                        "limits" => {
                            if raw_limits.is_some() {
                                return Err(de::Error::duplicate_field("limits"));
                            }
                            raw_limits = Some(map.next_value()?);
                        }
                        "vendor" => {
                            if vendor.is_some() {
                                return Err(de::Error::duplicate_field("vendor"));
                            }
                            vendor = Some(map.next_value()?);
                        }
                        "runtime_modes" => {
                            if runtime_modes.is_some() {
                                return Err(de::Error::duplicate_field("runtime_modes"));
                            }
                            runtime_modes = Some(map.next_value()?);
                        }
                        other => {
                            return Err(de::Error::unknown_field(
                                other,
                                &["mode", "limits", "vendor", "runtime_modes"],
                            ));
                        }
                    }
                }

                let mode = mode.ok_or_else(|| de::Error::missing_field("mode"))?;
                let limits = match raw_limits.flatten() {
                    None => None,
                    Some(raw) => {
                        let json = raw.get();
                        let parsed = match mode {
                            ControlMode::Position => serde_json::from_str::<PositionLimits>(json)
                                .map(ModeLimits::Position),
                            ControlMode::PositionVelocity => {
                                serde_json::from_str::<PositionVelocityLimits>(json)
                                    .map(ModeLimits::PositionVelocity)
                            }
                            ControlMode::Impedance => {
                                serde_json::from_str::<ImpedanceLimits>(json)
                                    .map(ModeLimits::Impedance)
                            }
                            ControlMode::PositionForce => {
                                serde_json::from_str::<PositionForceLimits>(json)
                                    .map(ModeLimits::PositionForce)
                            }
                        }
                        .map_err(|e| {
                            de::Error::custom(format!(
                                "invalid limits for control_mode '{mode}': {e}"
                            ))
                        })?;
                        Some(parsed)
                    }
                };
                Ok(JointControlModeConfig {
                    mode,
                    limits,
                    vendor: vendor.flatten(),
                    runtime_modes: runtime_modes.flatten(),
                })
            }
        }

        deserializer.deserialize_struct(
            "JointControlModeConfig",
            &["mode", "limits", "vendor", "runtime_modes"],
            JointControlModeVisitor,
        )
    }
}

impl Serialize for JointControlModeConfig {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut fields = 1;
        if self.limits.is_some() {
            fields += 1;
        }
        if self.vendor.is_some() {
            fields += 1;
        }
        if self.runtime_modes.is_some() {
            fields += 1;
        }
        let mut s = serializer.serialize_struct("JointControlModeConfig", fields)?;
        s.serialize_field("mode", &self.mode)?;
        if let Some(lim) = &self.limits {
            match lim {
                ModeLimits::Position(l) => s.serialize_field("limits", l)?,
                ModeLimits::PositionVelocity(l) => s.serialize_field("limits", l)?,
                ModeLimits::Impedance(l) => s.serialize_field("limits", l)?,
                ModeLimits::PositionForce(l) => s.serialize_field("limits", l)?,
            }
        }
        if let Some(vendor) = &self.vendor {
            s.serialize_field("vendor", vendor)?;
        }
        if let Some(rt) = &self.runtime_modes {
            s.serialize_field("runtime_modes", rt)?;
        }
        s.end()
    }
}

/// Fully resolved per-joint control-mode binding produced at profile load before transport open.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedJointMode {
    pub joint: usize,
    /// Bound hardware vendor (`None` only for generic `--follower mock` profiles without vendor hints).
    pub vendor: Option<VendorBinding>,
    /// Base control mode active after enable.
    pub mode: ControlMode,
    /// Validated typed limits for `mode`.
    pub limits: ModeLimits,
    /// Validated runtime mode switch targets permitted from `mode` (e.g. `[Impedance]` on
    /// Damiao arm joints when `gravity_comp` or MIT `joint_tracking` is configured).
    pub runtime_modes: Vec<ControlMode>,
    /// Active impedance wire mapping limits when `runtime_modes` permits [`ControlMode::Impedance`]
    /// (derived from commissioned `damiao_ranges[joint]` or Damiao defaults).
    pub runtime_impedance_limits: Option<ImpedanceLimits>,
}

// =========================================================================
// Vendor bindings and static capability table (from src/protocol/*.rs)
// =========================================================================

/// Supported motor vendor protocol bindings in `armd`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VendorBinding {
    #[serde(rename = "damiao")]
    Damiao,
    #[serde(rename = "feetech")]
    Feetech,
    #[serde(rename = "robstride")]
    RobStride,
    #[serde(rename = "fashionstar")]
    FashionStar,
}

impl VendorBinding {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Damiao => "damiao",
            Self::Feetech => "feetech",
            Self::RobStride => "robstride",
            Self::FashionStar => "fashionstar",
        }
    }

    /// Map CLI `--follower <MODEL>` to its hardware vendor binding.
    /// Returns `None` for virtual followers (`"sim"`, `"mock"`) and unknown names.
    pub fn from_follower_name(follower: &str) -> Option<Self> {
        match follower {
            "rebot-dm" => Some(Self::Damiao),
            "so-arm101" | "so-arm100" => Some(Self::Feetech),
            "rebot-rs" => Some(Self::RobStride),
            "fashionstar" | "star-arm102" => Some(Self::FashionStar),
            _ => None,
        }
    }

    pub const fn capability(self) -> &'static VendorCapability {
        match self {
            Self::Damiao => &DAMIAO_CAPABILITY,
            Self::Feetech => &FEETECH_CAPABILITY,
            Self::RobStride => &ROBSTRIDE_CAPABILITY,
            Self::FashionStar => &FASHIONSTAR_CAPABILITY,
        }
    }

    pub fn supports_mode(self, mode: ControlMode) -> bool {
        self.capability().supported_modes.contains(&mode)
    }

    pub fn supports_runtime_switch(self, from: ControlMode, to: ControlMode) -> bool {
        self.capability()
            .supported_runtime_switches
            .contains(&(from, to))
    }

    pub const fn all_capabilities() -> &'static [VendorCapability] {
        &VENDOR_CAPABILITIES
    }
}

impl fmt::Display for VendorBinding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Feedback acquisition mechanism implemented in `src/protocol/*.rs` and `src/driver.rs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeedbackSource {
    /// Motor replies to command frames plus explicit read-only `0xCC` refresh (`0x7FF`).
    CommandReplyAndRefresh,
    /// Host polls registers via bounded chunked `INST_SYNC_READ` (`0x82`).
    SyncReadPoll,
    /// Driver drains Type-2 extended CAN feedback frames (`TYPE_FEEDBACK = 2`).
    CanFeedbackDrain,
    /// Host polls explicit single-servo angle queries (`CMD_QUERY_SERVO_ANGLE = 0x0A`).
    ExplicitAngleQuery,
}

/// Hardware mode selection / readback mechanism verified in `src/protocol/*.rs` and `src/driver.rs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeSelectionMechanism {
    /// RAM register write (`0x7FF`, `REG_WRITE = 0x55`, `RID_CTRL_MODE = 10`) with ACK + readback (`REG_READ = 0x33`).
    RegisterWriteAndReadback,
    /// Provisioning-time register check/write (`REG_OPERATING_MODE = 0x21`, value `0`) with EEPROM unlock/verify/lock.
    ProvisioningRegisterReadback,
    /// Fixed extended CAN motion-control frame (`TYPE_MOTION_CONTROL = 1`).
    FixedMotionControlType,
    /// Implicit angle command (`CMD_SET_SERVO_ANGLE = 0x08`).
    ImplicitAngleCommand,
}

/// Verified capability table entry for one vendor protocol binding.
///
/// Built strictly from `src/protocol/*.rs` and the current drivers in `src/driver.rs`.
/// Capabilities or wire guarantees not confirmed in this repository are marked `needs-vendor-doc`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VendorCapability {
    pub vendor: VendorBinding,
    pub supported_modes: &'static [ControlMode],
    pub default_base_mode: ControlMode,
    pub supports_runtime_mode_switch: bool,
    pub supported_runtime_switches: &'static [(ControlMode, ControlMode)],
    pub mode_selection: ModeSelectionMechanism,
    pub feedback_source: FeedbackSource,
    pub feedback_position: bool,
    pub feedback_velocity: bool,
    pub feedback_torque: bool,
    pub feedback_load: bool,
    pub feedback_temperature: bool,
    pub feedback_enable_state: bool,
    /// General driver feedback freshness bound (ms) where defined (`None` = unverified / needs-vendor-doc).
    pub default_feedback_max_age_ms: Option<u64>,
    /// Mode-specific stricter feedback freshness bounds (ms), e.g. Damiao J7 `PositionForce` (`100 ms`).
    pub mode_feedback_max_age_ms: &'static [(ControlMode, u64)],
    pub needs_vendor_doc: &'static [&'static str],
}

impl VendorCapability {
    /// Effective feedback freshness bound (ms) for `mode`: mode-specific override if present,
    /// otherwise the vendor's general `default_feedback_max_age_ms`.
    pub fn feedback_max_age_ms(&self, mode: ControlMode) -> Option<u64> {
        self.mode_feedback_max_age_ms
            .iter()
            .find_map(|&(m, ms)| (m == mode).then_some(ms))
            .or(self.default_feedback_max_age_ms)
    }
}

/// Damiao capability record (`src/protocol/damiao.rs`).
///
/// Confirmed in repo:
/// - `RID_CTRL_MODE = 10`: `CTRL_MODE_MIT = 1` (`Impedance`), `CTRL_MODE_POS_VEL = 2`
///   (`PositionVelocity`), `CTRL_MODE_POS_FORCE = 4` (`PositionForce`).
/// - RAM register write (`0x55`) + ACK + separate readback (`0x33`) on `0x7FF`.
/// - Feedback unpacked from command replies and `0xCC` refresh (`position`, `velocity`, `torque`,
///   `t_mos`, `t_rotor`, `state_or_error`).
///
/// needs-vendor-doc:
/// - `CTRL_MODE = 3` (VEL) command frame layout and semantics.
/// - Per-motor-model supported `CTRL_MODE` matrix.
/// - Feedback reply cadence and delivery guarantees across modes and firmware versions.
/// - `RID_TIMEOUT` (register 9) count unit (`50 us` documented), expiry latency, and motor
///   response on expiry on B601-DM firmware (`sw_ver 942747703`).
pub const DAMIAO_CAPABILITY: VendorCapability = VendorCapability {
    vendor: VendorBinding::Damiao,
    supported_modes: &[
        ControlMode::PositionVelocity,
        ControlMode::Impedance,
        ControlMode::PositionForce,
    ],
    default_base_mode: ControlMode::PositionVelocity,
    supports_runtime_mode_switch: true,
    supported_runtime_switches: &[
        (ControlMode::PositionVelocity, ControlMode::Impedance),
        (ControlMode::Impedance, ControlMode::PositionVelocity),
    ],
    mode_selection: ModeSelectionMechanism::RegisterWriteAndReadback,
    feedback_source: FeedbackSource::CommandReplyAndRefresh,
    feedback_position: true,
    feedback_velocity: true,
    feedback_torque: true,
    feedback_load: false,
    feedback_temperature: true,
    feedback_enable_state: true,
    default_feedback_max_age_ms: Some(250),
    mode_feedback_max_age_ms: &[(ControlMode::PositionForce, 100)],
    needs_vendor_doc: &[
        "CTRL_MODE 3 (VEL) command frame layout and semantics",
        "Per-motor-model supported CTRL_MODE matrix",
        "Feedback reply cadence and guarantees across modes and firmware versions",
        "RID_TIMEOUT (register 9) count unit, expiry latency, and motor response on B601-DM firmware",
    ],
};

/// Feetech STS3215 capability record (`src/protocol/feetech.rs`).
///
/// Confirmed in repo:
/// - `INST_SYNC_WRITE` (`0x83`) to `REG_GOAL_POSITION` (`0x2A`) in operating mode `0` (`Position`).
/// - Provisioning reads `REG_OPERATING_MODE` (`0x21`), writes `0` when different (torque off,
///   EEPROM unlocked via `REG_LOCK = 0x37`), and verifies changed registers by readback.
/// - Bounded chunked `INST_SYNC_READ` (`0x82`) polls 8 bytes from `REG_PRESENT_POSITION_L` (`0x38`):
///   position, speed, load, voltage, temperature; torque-enable (`0x28`) has a separate readback.
///
/// needs-vendor-doc:
/// - `REG_OPERATING_MODE` (`0x21`) values and command semantics for wheel/speed and PWM modes.
/// - `REG_PRESENT_CURRENT_L` (`0x45`) scaling and latency (defined in protocol, outside the 8-byte poll).
pub const FEETECH_CAPABILITY: VendorCapability = VendorCapability {
    vendor: VendorBinding::Feetech,
    supported_modes: &[ControlMode::Position],
    default_base_mode: ControlMode::Position,
    supports_runtime_mode_switch: false,
    supported_runtime_switches: &[],
    mode_selection: ModeSelectionMechanism::ProvisioningRegisterReadback,
    feedback_source: FeedbackSource::SyncReadPoll,
    feedback_position: true,
    feedback_velocity: true,
    feedback_torque: false,
    feedback_load: true,
    feedback_temperature: true,
    feedback_enable_state: true,
    default_feedback_max_age_ms: Some(250),
    mode_feedback_max_age_ms: &[],
    needs_vendor_doc: &[
        "REG_OPERATING_MODE (0x21) values and command semantics for wheel/speed and PWM modes",
        "REG_PRESENT_CURRENT_L (0x45) scaling and latency (outside the 8-byte sync-read poll)",
    ],
};

/// RobStride capability record (`src/protocol/robstride.rs`).
///
/// Confirmed in repo:
/// - `TYPE_MOTION_CONTROL = 1` 29-bit extended CAN MIT-like frame (`Impedance`):
///   16-bit `pos`, `vel`, `kp`, `kd` in payload and 16-bit `torque` in CAN ID bits 23..8.
/// - `TYPE_FEEDBACK = 2` 29-bit extended CAN feedback (`position`, `velocity`, `torque`,
///   `mos_temperature` in 0.1 C, `fault_flags`).
/// - `TYPE_ENABLE = 3`, `TYPE_STOP = 4`, `TYPE_SET_ZERO = 6`, `TYPE_PARAM_READ = 17`,
///   `TYPE_PARAM_WRITE = 18` constants declared.
///
/// needs-vendor-doc:
/// - Position, speed, and current operating modes, selection registers, and readback via
///   `TYPE_PARAM_READ` (17) / `TYPE_PARAM_WRITE` (18).
/// - Type-2 feedback reply pairing, periodic reporting cadence, and current telemetry.
/// - Whether `TYPE_ENABLE` (3) / `TYPE_STOP` (4) yield causal torque-state evidence in Type-2 feedback.
/// - `TYPE_SET_ZERO` (6) persistence and parameter-save sequence (type 22).
pub const ROBSTRIDE_CAPABILITY: VendorCapability = VendorCapability {
    vendor: VendorBinding::RobStride,
    supported_modes: &[ControlMode::Impedance],
    default_base_mode: ControlMode::Impedance,
    supports_runtime_mode_switch: false,
    supported_runtime_switches: &[],
    mode_selection: ModeSelectionMechanism::FixedMotionControlType,
    feedback_source: FeedbackSource::CanFeedbackDrain,
    feedback_position: true,
    feedback_velocity: true,
    feedback_torque: true,
    feedback_load: false,
    feedback_temperature: true,
    feedback_enable_state: false,
    default_feedback_max_age_ms: Some(250),
    mode_feedback_max_age_ms: &[],
    needs_vendor_doc: &[
        "Position, speed, and current operating modes, selection registers, and readback via TYPE_PARAM_READ/TYPE_PARAM_WRITE",
        "Type-2 feedback reply pairing, periodic reporting cadence, and current telemetry",
        "Whether TYPE_ENABLE (3) / TYPE_STOP (4) yield causal torque-state evidence in Type-2 feedback",
        "TYPE_SET_ZERO (6) persistence and parameter-save sequence (type 22)",
    ],
};

/// FashionStar capability record (`src/protocol/fashionstar.rs`).
///
/// Confirmed in repo:
/// - `CMD_SET_SERVO_ANGLE = 0x08` (`Position`): angle in 0.1 deg (`i16`), interval (ms, `u16`),
///   power (`u16`); separate `CMD_SET_TORQUE = 0x1A`.
/// - Runtime polling uses round-robin `CMD_QUERY_SERVO_ANGLE = 0x0A` (position only);
///   `CMD_QUERY_STATUS = 0x1E` decoder exists in protocol for voltage/temp/current/status.
///
/// needs-vendor-doc:
/// - Mode selection/readback and any non-position operating modes.
/// - `CMD_SET_SERVO_ANGLE` (`0x08`) ACK semantics and `power == 0` meaning.
/// - Velocity and torque telemetry availability.
pub const FASHIONSTAR_CAPABILITY: VendorCapability = VendorCapability {
    vendor: VendorBinding::FashionStar,
    supported_modes: &[ControlMode::Position],
    default_base_mode: ControlMode::Position,
    supports_runtime_mode_switch: false,
    supported_runtime_switches: &[],
    mode_selection: ModeSelectionMechanism::ImplicitAngleCommand,
    feedback_source: FeedbackSource::ExplicitAngleQuery,
    feedback_position: true,
    feedback_velocity: false,
    feedback_torque: false,
    feedback_load: false,
    feedback_temperature: false,
    feedback_enable_state: false,
    default_feedback_max_age_ms: None,
    mode_feedback_max_age_ms: &[],
    needs_vendor_doc: &[
        "Mode selection/readback and any non-position operating modes",
        "CMD_SET_SERVO_ANGLE (0x08) ACK semantics and power=0 meaning",
        "Velocity and torque telemetry availability",
    ],
};

/// Complete static capability table across all supported vendor bindings.
pub const VENDOR_CAPABILITIES: [VendorCapability; 4] = [
    DAMIAO_CAPABILITY,
    FEETECH_CAPABILITY,
    ROBSTRIDE_CAPABILITY,
    FASHIONSTAR_CAPABILITY,
];
