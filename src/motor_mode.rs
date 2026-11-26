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

// =========================================================================
// PR0C: Tick-driven ModeSwitch service (executor services layer, spec item 6)
// =========================================================================

#[cfg(not(target_arch = "wasm32"))]
pub mod switch {
    use std::time::{Duration, Instant};

    use super::{ControlMode, VendorBinding};

    /// Maximum axes tracked inline by one [`ModeSwitch`] transaction (matches `MAX_MODE_JOINTS`).
    pub const MAX_MODE_SWITCH_AXES: usize = 16;

    /// Default maximum control-frame watchdog gap tolerated on energized watchdog-bearing axes.
    pub const DEFAULT_MAX_WATCHDOG_GAP: Duration = Duration::from_millis(40);

    /// Default proactive watchdog hold interval matching Damiao `WATCHDOG_HOLD_PROACTIVE_INTERVAL`.
    pub const DEFAULT_WATCHDOG_PROACTIVE_INTERVAL: Duration = Duration::from_millis(15);

    /// Conservative worst-case allowance for RX drain + hold TX flush + scheduling jitter on the
    /// next release when checking whether waiting another tick would breach `max_watchdog_gap`.
    pub const DEFAULT_HOLD_DELIVERY_MARGIN: Duration = Duration::from_millis(5);

    /// Default per-axis confirmation timeout for disarmed or non-watchdog axes (e.g. J7 PosForce).
    pub const DEFAULT_PER_AXIS_TIMEOUT: Duration = Duration::from_millis(50);

    /// Minimum allowed caller-configured per-axis timeout.
    pub const MIN_PER_AXIS_TIMEOUT: Duration = Duration::from_millis(1);

    /// Maximum allowed caller-configured per-axis timeout (prevents overflow / RT panic).
    pub const MAX_PER_AXIS_TIMEOUT: Duration = Duration::from_secs(5);

    /// Maximum allowed caller-configured watchdog gap bound.
    pub const MAX_WATCHDOG_GAP_LIMIT: Duration = Duration::from_secs(5);

    /// Per-release execution budget passed into [`ModeSwitch::step`].
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ReleaseBudget {
        pub now: Instant,
        pub cutoff: Instant,
        pub period: Duration,
        pub max_tx_frames: u8,
        pub max_rx_frames: u16,
        pub watchdog_proactive_interval: Duration,
        pub max_watchdog_gap: Duration,
        pub hold_delivery_margin: Duration,
    }

    impl ReleaseBudget {
        pub fn new(now: Instant, cutoff: Instant, period: Duration, max_tx_frames: u8) -> Self {
            Self {
                now,
                cutoff,
                period,
                max_tx_frames,
                max_rx_frames: 64,
                watchdog_proactive_interval: DEFAULT_WATCHDOG_PROACTIVE_INTERVAL,
                max_watchdog_gap: DEFAULT_MAX_WATCHDOG_GAP,
                hold_delivery_margin: DEFAULT_HOLD_DELIVERY_MARGIN,
            }
        }

        /// Builds a tightened mode-switch budget from the shared per-release
        /// [`crate::rt_core::budget::ReleaseBudget`] ledger, clamping both RX and TX caps
        /// to the ledger's remaining allowance (sub-budgets may only tighten).
        pub fn from_ledger(
            ledger: &crate::rt_core::budget::ReleaseBudget,
            now: Instant,
            cutoff: Instant,
            period: Duration,
            max_tx_frames: u8,
        ) -> Self {
            let mut b = Self::new(
                now,
                cutoff,
                period,
                max_tx_frames.min(ledger.tx_remaining().min(u16::from(u8::MAX)) as u8),
            );
            b.max_rx_frames = ledger.rx_remaining();
            b
        }

        /// Converts this standalone budget into a driver-clock
        /// [`crate::rt_core::budget::ReleaseBudget`] ledger.
        pub fn to_ledger(&self) -> crate::rt_core::budget::ReleaseBudget {
            crate::rt_core::budget::ReleaseBudget::new(
                0,
                self.now,
                self.cutoff,
                self.max_rx_frames,
                u16::from(self.max_tx_frames),
            )
        }
    }

    /// Operation intent of a [`ModeSwitchPlan`].
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ModeSwitchIntent {
        /// Read-only causal confirmation (`REG_READ(RID_CTRL_MODE)`) that every target axis is in
        /// its requested mode (never emits `REG_WRITE`).
        ConfirmOnly,
        /// Switch each target axis whose verified mode differs from `target_mode` via mandatory
        /// wire-range verification (for `Impedance`), causal `REG_WRITE`, ACK, optional separate
        /// readback, and optional post-confirm causal feedback.
        Switch,
    }

    /// Recovery policy when a forward switch fails cleanly before any axis becomes ambiguous.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ModeSwitchRecovery {
        /// While session authority remains live (not stopped/E-stopped), roll back any
        /// already-switched axes to their pre-switch confirmed modes via the same tick-driven
        /// watchdog-first causal confirmation path.
        RollbackToPrevious,
        /// Do not issue rollback writes; terminate immediately.
        FailClosed,
    }

    /// Typed static failure/abort reason for [`ModeSwitch`] (zero heap allocation on RT).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ModeSwitchFailure {
        InvalidPlan,
        UnsupportedMode,
        RangeMismatch,
        RangeCheckTimeout,
        ReadbackMismatch,
        ReadbackTimeout,
        WriteAckMismatch,
        WriteAckTimeout,
        FeedbackTimeout,
        FeedbackUnsafe,
        AmbiguousReply,
        TxFailed,
        WatchdogBudgetBreached,
        Stopped,
        Revoked,
        DeadlineExpired,
        NoConfirmedMode,
        /// The plan's watchdog gap is shorter than one release period plus the hold delivery
        /// margin, so no hold schedule can keep it.
        WatchdogGapInfeasible,
    }

    impl ModeSwitchFailure {
        pub const fn code(self) -> &'static str {
            match self {
                Self::InvalidPlan => "INVALID_COMMAND",
                Self::UnsupportedMode => "UNSUPPORTED_MODE",
                Self::RangeMismatch => "RANGE_MISMATCH",
                Self::RangeCheckTimeout => "RANGE_CHECK_TIMEOUT",
                Self::ReadbackMismatch => "MODE_READBACK_MISMATCH",
                Self::ReadbackTimeout => "MODE_READBACK_TIMEOUT",
                Self::WriteAckMismatch => "MODE_WRITE_MISMATCH",
                Self::WriteAckTimeout => "MODE_WRITE_TIMEOUT",
                Self::FeedbackTimeout => "MODE_FEEDBACK_TIMEOUT",
                Self::FeedbackUnsafe => "MODE_FEEDBACK_UNSAFE",
                Self::AmbiguousReply => "MODE_REPLY_AMBIGUOUS",
                Self::TxFailed => "MODE_TX_FAILED",
                Self::WatchdogBudgetBreached => "WATCHDOG_CONTINUITY_ABORT",
                Self::Stopped => "STOPPED",
                Self::Revoked => "REVOKED",
                Self::DeadlineExpired => "TIMEOUT",
                Self::NoConfirmedMode => "MODE_UNVERIFIED",
                Self::WatchdogGapInfeasible => "WATCHDOG_GAP_INFEASIBLE",
            }
        }

        pub fn from_revoke_code(code: &'static str) -> Self {
            match code {
                "STOPPED" | "ESTOP_ACTIVE" | "EPOCH_MISMATCH" => Self::Stopped,
                "TIMEOUT" | "DEADLINE_EXPIRED" => Self::DeadlineExpired,
                _ => Self::Revoked,
            }
        }
    }

    /// Target mode and confirmation requirements for one joint axis.
    ///
    /// Wire-range verification (`PMAX`/`VMAX`/`TMAX`) is unconditionally mandatory before ANY
    /// transition into [`ControlMode::Impedance`] (both forward and rollback) and cannot be
    /// opted out of here.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct AxisTarget {
        pub joint: u8,
        pub target_mode: ControlMode,
        /// Require a separate `REG_READ(RID_CTRL_MODE)` after `REG_WRITE` ACK before considering
        /// the new mode confirmed. On energized watchdog-bearing axes this is admitted ONLY when
        /// `2 * period + hold_delivery_margin <= max_watchdog_gap`.
        pub require_post_write_readback: bool,
        /// Require fresh causal feedback (`feedback_seq > confirm_seq`, `rx_at >= confirm_at`)
        /// after mode confirmation.
        pub require_post_confirm_feedback: bool,
        /// Require the register reply's CAN ID to match the motor's expected master reply CAN ID
        /// (always enforced for `PositionForce` J7).
        pub require_expected_reply_id: bool,
        /// When confirming/switching disarmed `PositionForce`, require stationary (`|v| <= 0.05`)
        /// and disabled (`state == 0`) fresh (`<= 100 ms`) feedback.
        pub require_disarmed_stationary: bool,
    }

    impl AxisTarget {
        pub const fn new(joint: u8, target_mode: ControlMode) -> Self {
            let is_pf = matches!(target_mode, ControlMode::PositionForce);
            Self {
                joint,
                target_mode,
                require_post_write_readback: false,
                require_post_confirm_feedback: true,
                require_expected_reply_id: is_pf,
                require_disarmed_stationary: is_pf,
            }
        }
    }

    /// Fixed-capacity `Copy` plan submitted to [`ModeSwitch`].
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ModeSwitchPlan {
        pub axes: [AxisTarget; MAX_MODE_SWITCH_AXES],
        pub len: u8,
        pub intent: ModeSwitchIntent,
        pub recovery: ModeSwitchRecovery,
        pub per_axis_timeout: Duration,
        pub max_watchdog_gap: Duration,
    }

    impl ModeSwitchPlan {
        pub fn empty(intent: ModeSwitchIntent) -> Self {
            Self {
                axes: [AxisTarget::new(0, ControlMode::PositionVelocity); MAX_MODE_SWITCH_AXES],
                len: 0,
                intent,
                recovery: ModeSwitchRecovery::RollbackToPrevious,
                per_axis_timeout: DEFAULT_PER_AXIS_TIMEOUT,
                max_watchdog_gap: DEFAULT_MAX_WATCHDOG_GAP,
            }
        }

        /// Validates plan bounds (axis count, unique in-range joints, non-overflowing timeout and
        /// watchdog bounds) before admission or hardware TX.
        pub fn validate(&self) -> Result<(), ModeSwitchFailure> {
            let len = usize::from(self.len);
            if len == 0 || len > MAX_MODE_SWITCH_AXES {
                return Err(ModeSwitchFailure::InvalidPlan);
            }
            if self.per_axis_timeout < MIN_PER_AXIS_TIMEOUT
                || self.per_axis_timeout > MAX_PER_AXIS_TIMEOUT
            {
                return Err(ModeSwitchFailure::InvalidPlan);
            }
            if self.max_watchdog_gap.is_zero()
                || self.max_watchdog_gap > MAX_WATCHDOG_GAP_LIMIT
            {
                return Err(ModeSwitchFailure::InvalidPlan);
            }
            let mut seen = 0u32;
            for i in 0..len {
                let j = usize::from(self.axes[i].joint);
                if j >= MAX_MODE_SWITCH_AXES || (seen & (1u32 << j)) != 0 {
                    return Err(ModeSwitchFailure::InvalidPlan);
                }
                seen |= 1u32 << j;
            }
            Ok(())
        }

        /// Reference adapter plan: read-only causal J7 gripper mode confirmation (`PositionVelocity`
        /// or `PositionForce`). Rejects out-of-range joints or non-gripper modes with
        /// [`ModeSwitchFailure::InvalidPlan`].
        pub fn confirm_gripper(joint: usize, mode: ControlMode) -> Result<Self, ModeSwitchFailure> {
            if joint >= MAX_MODE_SWITCH_AXES
                || !matches!(mode, ControlMode::PositionVelocity | ControlMode::PositionForce)
            {
                return Err(ModeSwitchFailure::InvalidPlan);
            }
            let mut plan = Self::empty(ModeSwitchIntent::ConfirmOnly);
            plan.recovery = ModeSwitchRecovery::FailClosed;
            let mut t = AxisTarget::new(joint as u8, mode);
            t.require_post_confirm_feedback = matches!(mode, ControlMode::PositionForce);
            t.require_expected_reply_id = matches!(mode, ControlMode::PositionForce);
            t.require_disarmed_stationary = matches!(mode, ControlMode::PositionForce);
            plan.axes[0] = t;
            plan.len = 1;
            Ok(plan)
        }

        /// Builds a validated switch plan over `targets`, rejecting empty, over-capacity,
        /// out-of-range, or duplicate joint indices with [`ModeSwitchFailure::InvalidPlan`].
        pub fn switch_axes(
            targets: &[(usize, ControlMode)],
            recovery: ModeSwitchRecovery,
        ) -> Result<Self, ModeSwitchFailure> {
            if targets.is_empty() || targets.len() > MAX_MODE_SWITCH_AXES {
                return Err(ModeSwitchFailure::InvalidPlan);
            }
            let mut plan = Self::empty(ModeSwitchIntent::Switch);
            plan.recovery = recovery;
            let mut seen = 0u32;
            for &(joint, mode) in targets {
                if joint >= MAX_MODE_SWITCH_AXES || (seen & (1u32 << joint)) != 0 {
                    return Err(ModeSwitchFailure::InvalidPlan);
                }
                seen |= 1u32 << joint;
                plan.axes[usize::from(plan.len)] = AxisTarget::new(joint as u8, mode);
                plan.len += 1;
            }
            Ok(plan)
        }

        pub fn axis_slice(&self) -> &[AxisTarget] {
            &self.axes[..usize::from(self.len).min(MAX_MODE_SWITCH_AXES)]
        }
    }

    /// Result of one tick-driven [`ModeSwitch::step`] call.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ModeProgress {
        /// Still advancing across releases; caller keeps stepping on subsequent ticks.
        Pending { wake: Option<Instant> },
        /// Every requested axis is causally confirmed in its target mode.
        Confirmed,
        /// Forward switch failed cleanly before ambiguity and every previously switched axis was
        /// causally restored to its pre-switch confirmed mode (`uncertain_hw_state == false`).
        RolledBack {
            failed_joint: u8,
            reason: ModeSwitchFailure,
        },
        /// Stopped/cancelled/revoked while every axis still holds a causally confirmed mode
        /// (`mode_write_uncertain_mask == 0`, `uncertain_hw_state == false`); stopped holds keep
        /// holding each axis in its last confirmed mode without issuing further `REG_WRITE`s.
        StoppedRecovered { reason: ModeSwitchFailure },
        /// Clean failure with no hardware ambiguity and no rollback needed
        /// (`uncertain_hw_state == false`).
        FailedClean {
            failed_joint: u8,
            reason: ModeSwitchFailure,
        },
        /// Hardware mode is ambiguous (unconfirmed write, failed flush after write, duplicate or
        /// drifted reply, contradicted energized `ConfirmOnly` cache, failed rollback, or
        /// energized watchdog continuity breach). Callers MUST escalate energized arms to
        /// SAFE-HOLD (`SafeHoldReason::SessionUncertainState`, 0 `CMD_DISABLE` frames).
        Ambiguous {
            failed_joint: u8,
            reason: ModeSwitchFailure,
        },
    }

    impl ModeProgress {
        #[inline]
        pub const fn is_terminal(self) -> bool {
            !matches!(self, Self::Pending { .. })
        }

        #[inline]
        pub const fn is_ambiguous(self) -> bool {
            matches!(self, Self::Ambiguous { .. })
        }

        #[inline]
        pub const fn is_confirmed(self) -> bool {
            matches!(self, Self::Confirmed)
        }

        pub const fn failure_reason(self) -> Option<ModeSwitchFailure> {
            match self {
                Self::Pending { .. } | Self::Confirmed => None,
                Self::RolledBack { reason, .. }
                | Self::StoppedRecovered { reason }
                | Self::FailedClean { reason, .. }
                | Self::Ambiguous { reason, .. } => Some(reason),
            }
        }
    }

    /// Outcome of servicing energized watchdog holds before mode-switch register TX.
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct WatchdogHoldResult {
        /// Number of CAN frames transmitted (including J7 `PosForce` hold/freeze if sent).
        pub frames_sent: u8,
        /// Bitmask of axes whose control frame was delivered & committed in this pass.
        pub refreshed_mask: u32,
        /// True iff every due energized confirmed axis (outside `exclude_mask`) was refreshed.
        pub all_due_serviced: bool,
        /// Largest remaining elapsed control-frame age across all energized watchdog-bearing
        /// confirmed axes after this pass.
        pub max_energized_age: Duration,
        /// Joint index achieving `max_energized_age`, if any energized watchdog-bearing axis exists.
        pub oldest_joint: Option<u8>,
    }

    /// Outcome of one non-blocking register or refresh TX attempt.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ModeTxResult {
        /// Frame transmitted and flushed; causal reply window opens strictly after `(boundary_seq, tx_at)`.
        Sent {
            boundary_seq: u64,
            tx_at: Instant,
            tx_offset_ns: Option<i128>,
        },
        /// Frame was handed to transport `transmit()`, then flush failed or cutoff elapsed.
        /// For `REG_WRITE(RID_CTRL_MODE)`, hardware state is immediately ambiguous.
        SentFlushFailed {
            boundary_seq: u64,
            tx_at: Instant,
            tx_offset_ns: Option<i128>,
        },
        /// Deferred due to TX budget, cutoff, or transport backpressure (frame never left).
        Backpressure,
        /// Pre-TX authority / stop gate revoked before transmitting (frame never left).
        Revoked(&'static str),
        /// Hard refusal before transmitting (frame never left).
        Refused(&'static str),
    }

    /// Causal match result for a `RID_CTRL_MODE` register reply.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ModeRegMatch {
        /// No complete causal reply observed yet (or RX queue not yet drained empty).
        Pending,
        /// Single causal post-boundary reply observed through an empty RX queue.
        Matched {
            mode: Option<ControlMode>,
            raw_value: u32,
            rx_seq: u64,
            rx_at: Instant,
            decode_offset_ns: Option<i128>,
        },
        /// Multiple replies, buffer overflow, pre-TX timestamp, or clock-offset drift detected.
        Ambiguous,
    }

    /// Causal match result for a `PMAX`/`VMAX`/`TMAX` range register read reply.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum ModeRangeMatch {
        Pending,
        Matched(f32),
        Ambiguous,
    }

    /// Causal post-confirmation feedback evaluation.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ModeFeedbackMatch {
        Pending,
        FreshConfirmed,
        Unsafe,
    }

    /// Compact `Copy` diagnostic/observability event pushed over `rt_core/stream` off-RT.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ModeSwitchEvent {
        pub session_id: u64,
        pub kind: ModeSwitchEventKind,
        pub joint: u8,
        pub mode: Option<ControlMode>,
        pub reason: Option<ModeSwitchFailure>,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ModeSwitchEventKind {
        Started,
        AxisConfirmed,
        RollbackStarted,
        TerminalConfirmed,
        TerminalRolledBack,
        TerminalStoppedRecovered,
        TerminalFailedClean,
        TerminalAmbiguous,
    }

    /// Authoritative `Copy` snapshot returned by `PollModeSwitch` (unaffected by stream drops).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ModeSwitchSnapshot {
        pub session_id: u64,
        pub intent: ModeSwitchIntent,
        pub running: bool,
        pub recovering: bool,
        pub progress: ModeProgress,
        pub axis_count: u8,
        pub confirmed_mask: u32,
        pub uncertain_mask: u32,
        pub steps: u32,
    }

    /// Narrow zero-allocation driver abstraction consumed by [`ModeSwitch::step_driver`].
    pub trait ModeSwitchDriver {
        fn clock_now(&self) -> Instant;
        fn dof(&self) -> usize;
        fn vendor(&self) -> Option<VendorBinding>;
        fn supports_nonblocking_mode_switch(&self) -> bool;
        fn axis_is_energized(&self, joint: usize) -> bool;
        fn any_axis_energized(&self) -> bool;
        /// True when `joint` carries a hardware control-frame watchdog (e.g. Damiao J1–J6;
        /// false for J7 `PosForce` which keeps `TIMEOUT = 0`).
        fn axis_has_watchdog(&self, joint: usize) -> bool;
        /// True when `joint` is configured to support `ControlMode::PositionForce`.
        fn axis_supports_pos_force(&self, joint: usize) -> bool;
        fn remaining_rx_budget(&self, default_max: u16) -> u16;
        fn axis_confirmed_mode(&self, joint: usize) -> Option<ControlMode>;
        fn set_axis_confirmed_mode(&mut self, joint: usize, mode: Option<ControlMode>);
        fn latch_axis_mode_uncertain(&mut self, joint: usize);
        fn mode_write_uncertain_mask(&self) -> u32;
        fn set_mode_switch_active_mask(&mut self, mask: u32);
        /// Owner proof that a stop/E-stop has been APPLIED (not merely pending). Defaults to
        /// `false`: without proof a revoked energized partial switch escalates to SAFE-HOLD.
        fn stop_applied(&self) -> bool { false }
        fn axis_last_ctrl_tx(&self, joint: usize) -> Option<Instant>;
        fn axis_impedance_ranges_verified(&self, joint: usize) -> bool;
        fn axis_configured_impedance_ranges(&self, joint: usize) -> Option<[f32; 3]>;
        fn set_axis_impedance_ranges_verified(&mut self, joint: usize, verified: bool);
        fn rx_drained(&self) -> bool;
        fn drain_rx_step(&mut self, max_frames: u16, deadline: Instant) -> Result<(u16, bool), ModeSwitchFailure>;
        fn clear_register_reply_log(&mut self);
        fn service_watchdog_holds(
            &mut self,
            min_age: Duration,
            exclude_mask: u32,
            force_mask: u32,
            deadline: Instant,
            max_frames: u8,
        ) -> Result<WatchdogHoldResult, ModeSwitchFailure>;
        fn try_send_ctrl_mode_read(&mut self, joint: usize, deadline: Instant) -> ModeTxResult;
        fn try_send_ctrl_mode_write(
            &mut self,
            joint: usize,
            mode: ControlMode,
            deadline: Instant,
        ) -> ModeTxResult;
        fn try_send_range_read(
            &mut self,
            joint: usize,
            rid_idx: u8,
            deadline: Instant,
        ) -> ModeTxResult;
        fn try_send_status_refresh(&mut self, joint: usize, deadline: Instant) -> ModeTxResult;
        #[allow(clippy::too_many_arguments)]
        fn find_ctrl_mode_reply(
            &self,
            joint: usize,
            write: bool,
            after_seq: u64,
            min_rx_at: Instant,
            max_rx_at: Instant,
            tx_offset_ns: Option<i128>,
            require_expected_id: bool,
        ) -> ModeRegMatch;
        fn find_range_reply(
            &self,
            joint: usize,
            rid_idx: u8,
            after_seq: u64,
            min_rx_at: Instant,
            max_rx_at: Instant,
            tx_offset_ns: Option<i128>,
        ) -> ModeRangeMatch;
        #[allow(clippy::too_many_arguments)]
        fn post_confirm_feedback(
            &self,
            joint: usize,
            mode: ControlMode,
            after_seq: u64,
            after_at: Instant,
            max_rx_at: Instant,
            confirm_offset_ns: Option<i128>,
            now: Instant,
            require_disarmed_stationary: bool,
        ) -> ModeFeedbackMatch;
    }

    /// Lightweight zero-alloc adapter binding `&mut dyn ArmDriver` and a live pre-TX gate to
    /// [`ModeSwitchDriver`].
    pub struct ArmDriverModeAdapter<'a> {
        drv: &'a mut dyn crate::driver::ArmDriver,
        pre_tx_gate: &'a mut dyn FnMut() -> Result<(), &'static str>,
        stop_applied: Option<&'a dyn Fn() -> bool>,
    }

    impl<'a> ArmDriverModeAdapter<'a> {
        pub fn new(
            drv: &'a mut dyn crate::driver::ArmDriver,
            pre_tx_gate: &'a mut dyn FnMut() -> Result<(), &'static str>,
        ) -> Self {
            Self { drv, pre_tx_gate, stop_applied: None }
        }

        /// Attaches the owner's "stop applied" proof consulted when the pre-TX gate revokes.
        pub fn with_stop_proof(mut self, stop_applied: &'a dyn Fn() -> bool) -> Self {
            self.stop_applied = Some(stop_applied);
            self
        }
    }

    impl ModeSwitchDriver for ArmDriverModeAdapter<'_> {
        fn clock_now(&self) -> Instant { self.drv.clock_now() }
        fn dof(&self) -> usize { self.drv.mode_switch_dof() }
        fn vendor(&self) -> Option<VendorBinding> { self.drv.driver_vendor_binding() }
        fn supports_nonblocking_mode_switch(&self) -> bool { self.drv.supports_nonblocking_mode_switch() }
        fn axis_is_energized(&self, joint: usize) -> bool { self.drv.axis_is_energized(joint) }
        fn any_axis_energized(&self) -> bool { self.drv.any_axis_energized() }
        fn axis_has_watchdog(&self, joint: usize) -> bool { self.drv.axis_has_watchdog(joint) }
        fn axis_supports_pos_force(&self, joint: usize) -> bool { self.drv.axis_supports_pos_force(joint) }
        fn remaining_rx_budget(&self, default_max: u16) -> u16 {
            self.drv.mode_switch_remaining_rx_budget(default_max)
        }
        fn axis_confirmed_mode(&self, joint: usize) -> Option<ControlMode> { self.drv.axis_confirmed_mode(joint) }
        fn set_axis_confirmed_mode(&mut self, joint: usize, mode: Option<ControlMode>) {
            self.drv.set_axis_confirmed_mode(joint, mode);
        }
        fn latch_axis_mode_uncertain(&mut self, joint: usize) {
            self.drv.latch_axis_mode_uncertain(joint);
        }
        fn mode_write_uncertain_mask(&self) -> u32 { self.drv.mode_write_uncertain_mask() }
        fn set_mode_switch_active_mask(&mut self, mask: u32) { self.drv.set_mode_switch_active_mask(mask); }
        fn stop_applied(&self) -> bool { self.stop_applied.is_some_and(|f| f()) }
        fn axis_last_ctrl_tx(&self, joint: usize) -> Option<Instant> { self.drv.axis_last_ctrl_tx(joint) }
        fn axis_impedance_ranges_verified(&self, joint: usize) -> bool { self.drv.axis_impedance_ranges_verified(joint) }
        fn axis_configured_impedance_ranges(&self, joint: usize) -> Option<[f32; 3]> { self.drv.axis_configured_impedance_ranges(joint) }
        fn set_axis_impedance_ranges_verified(&mut self, joint: usize, verified: bool) {
            self.drv.set_axis_impedance_ranges_verified(joint, verified);
        }
        fn rx_drained(&self) -> bool { self.drv.mode_switch_rx_drained() }
        fn drain_rx_step(&mut self, max_frames: u16, deadline: Instant) -> Result<(u16, bool), ModeSwitchFailure> {
            self.drv
                .commissioning_drain_rx_step(max_frames, deadline)
                .map_err(|_| ModeSwitchFailure::TxFailed)
        }
        fn clear_register_reply_log(&mut self) { self.drv.commissioning_clear_register_log(); }
        fn service_watchdog_holds(
            &mut self,
            min_age: Duration,
            exclude_mask: u32,
            force_mask: u32,
            deadline: Instant,
            max_frames: u8,
        ) -> Result<WatchdogHoldResult, ModeSwitchFailure> {
            self.drv.mode_switch_service_watchdog_holds(
                min_age,
                exclude_mask,
                force_mask,
                deadline,
                max_frames,
            )
        }
        fn try_send_ctrl_mode_read(&mut self, joint: usize, deadline: Instant) -> ModeTxResult {
            self.drv.mode_switch_try_send_ctrl_mode_read(joint, deadline, self.pre_tx_gate)
        }
        fn try_send_ctrl_mode_write(
            &mut self,
            joint: usize,
            mode: ControlMode,
            deadline: Instant,
        ) -> ModeTxResult {
            self.drv.mode_switch_try_send_ctrl_mode_write(joint, mode, deadline, self.pre_tx_gate)
        }
        fn try_send_range_read(&mut self, joint: usize, rid_idx: u8, deadline: Instant) -> ModeTxResult {
            self.drv.mode_switch_try_send_range_read(joint, rid_idx, deadline, self.pre_tx_gate)
        }
        fn try_send_status_refresh(&mut self, joint: usize, deadline: Instant) -> ModeTxResult {
            self.drv.mode_switch_try_send_status_refresh(joint, deadline, self.pre_tx_gate)
        }
        fn find_ctrl_mode_reply(
            &self,
            joint: usize,
            write: bool,
            after_seq: u64,
            min_rx_at: Instant,
            max_rx_at: Instant,
            tx_offset_ns: Option<i128>,
            require_expected_id: bool,
        ) -> ModeRegMatch {
            self.drv.mode_switch_find_ctrl_mode_reply(
                joint,
                write,
                after_seq,
                min_rx_at,
                max_rx_at,
                tx_offset_ns,
                require_expected_id,
            )
        }
        fn find_range_reply(
            &self,
            joint: usize,
            rid_idx: u8,
            after_seq: u64,
            min_rx_at: Instant,
            max_rx_at: Instant,
            tx_offset_ns: Option<i128>,
        ) -> ModeRangeMatch {
            self.drv.mode_switch_find_range_reply(
                joint,
                rid_idx,
                after_seq,
                min_rx_at,
                max_rx_at,
                tx_offset_ns,
            )
        }
        fn post_confirm_feedback(
            &self,
            joint: usize,
            mode: ControlMode,
            after_seq: u64,
            after_at: Instant,
            max_rx_at: Instant,
            confirm_offset_ns: Option<i128>,
            now: Instant,
            require_disarmed_stationary: bool,
        ) -> ModeFeedbackMatch {
            self.drv.mode_switch_post_confirm_feedback(
                joint,
                mode,
                after_seq,
                after_at,
                max_rx_at,
                confirm_offset_ns,
                now,
                require_disarmed_stationary,
            )
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    enum AxisPhase {
        Pending,
        RangeReadNeed {
            rid_idx: u8,
            got: [Option<f32>; 3],
        },
        RangeReadAwait {
            rid_idx: u8,
            boundary_seq: u64,
            tx_at: Instant,
            tx_offset_ns: Option<i128>,
            deadline: Instant,
            got: [Option<f32>; 3],
        },
        NeedWrite,
        AwaitingWriteAck {
            boundary_seq: u64,
            tx_at: Instant,
            tx_offset_ns: Option<i128>,
            deadline: Instant,
            last_ctrl_at: Option<Instant>,
            written_mode: ControlMode,
        },
        NeedReadback {
            expected_mode: ControlMode,
            from_write: bool,
            last_ctrl_at: Option<Instant>,
            post_write_deadline: Option<Instant>,
        },
        AwaitingReadback {
            boundary_seq: u64,
            tx_at: Instant,
            tx_offset_ns: Option<i128>,
            deadline: Instant,
            expected_mode: ControlMode,
            from_write: bool,
            last_ctrl_at: Option<Instant>,
        },
        AwaitingFeedback {
            after_seq: u64,
            after_at: Instant,
            confirm_offset_ns: Option<i128>,
            deadline: Instant,
            confirmed_mode: ControlMode,
            from_write: bool,
            fenced_pending_feedback: bool,
            refresh_sent: bool,
        },
        Confirmed {
            mode: ControlMode,
        },
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum RunnerPhase {
        Init,
        Forward,
        Recovering {
            failed_joint: u8,
            reason: ModeSwitchFailure,
        },
        Done(ModeProgress),
    }

    #[derive(Debug, Clone, Copy)]
    struct AxisSlot {
        target: AxisTarget,
        active_goal: ControlMode,
        previous_mode: Option<ControlMode>,
        switched_forward: bool,
        started_energized: bool,
        phase: AxisPhase,
    }

    impl AxisSlot {
        const fn empty() -> Self {
            Self {
                target: AxisTarget::new(0, ControlMode::PositionVelocity),
                active_goal: ControlMode::PositionVelocity,
                previous_mode: None,
                switched_forward: false,
                started_energized: false,
                phase: AxisPhase::Pending,
            }
        }
    }

    /// Tick-driven, zero-allocation motor control-mode switch & confirmation state machine.
    ///
    /// Guarantees:
    /// - Holds all energized confirmed axes in their **last confirmed mode** first on every tick.
    /// - Enforces watchdog budget first (`<= max_watchdog_gap`, default 40 ms) via predictive
    ///   write admission and confirmation deadline clamping against the confirmed control-TX
    ///   delivery timestamp.
    /// - Unconditionally verifies `PMAX`/`VMAX`/`TMAX` wire ranges before any `Impedance` write
    ///   (forward or rollback).
    /// - Requires causal empty-queue RX boundaries, post-boundary timestamps, no duplicates or
    ///   clock-offset drift, and expected master CAN IDs for `PositionForce`.
    /// - Returns clean recovery (`RolledBack` / `StoppedRecovered` / `FailedClean`) when zero axes
    ///   are mode-ambiguous, and `Ambiguous` (`uncertain_hw_state == true`) whenever a write may
    ///   have reached the wire without causal confirmation or an energized cache is contradicted.
    pub struct ModeSwitch {
        plan: ModeSwitchPlan,
        phase: RunnerPhase,
        slots: [AxisSlot; MAX_MODE_SWITCH_AXES],
        active_idx: usize,
        active_mask: u32,
        confirmed_mask: u32,
        ambiguous_mask: u32,
        steps: u32,
        /// Inline staging ring of recent events for the caller to drain into `rt_core/stream`.
        pending_events: [Option<ModeSwitchEvent>; 8],
        pending_event_len: usize,
    }

    impl ModeSwitch {
        pub fn new(plan: ModeSwitchPlan) -> Self {
            let mut slots = [AxisSlot::empty(); MAX_MODE_SWITCH_AXES];
            let len = usize::from(plan.len).min(MAX_MODE_SWITCH_AXES);
            let mut active_mask = 0u32;
            for i in 0..len {
                let target = plan.axes[i];
                if usize::from(target.joint) < MAX_MODE_SWITCH_AXES {
                    active_mask |= 1u32 << target.joint;
                }
                slots[i] = AxisSlot {
                    target,
                    active_goal: target.target_mode,
                    previous_mode: None,
                    switched_forward: false,
                    started_energized: false,
                    phase: AxisPhase::Pending,
                };
            }
            Self {
                plan,
                phase: RunnerPhase::Init,
                slots,
                active_idx: 0,
                active_mask,
                confirmed_mask: 0,
                ambiguous_mask: 0,
                steps: 0,
                pending_events: [None; 8],
                pending_event_len: 0,
            }
        }

        #[inline]
        pub fn plan(&self) -> &ModeSwitchPlan { &self.plan }

        #[inline]
        pub fn is_running(&self) -> bool { !matches!(self.phase, RunnerPhase::Done(_)) }

        #[inline]
        pub fn progress(&self) -> ModeProgress {
            match self.phase {
                RunnerPhase::Done(p) => p,
                _ => ModeProgress::Pending { wake: None },
            }
        }

        #[inline]
        pub fn has_ambiguous_axis(&self) -> bool {
            self.ambiguous_mask != 0
                || matches!(self.phase, RunnerPhase::Done(ModeProgress::Ambiguous { .. }))
        }

        pub fn snapshot(&self, session_id: u64, driver_uncertain_mask: u32) -> ModeSwitchSnapshot {
            ModeSwitchSnapshot {
                session_id,
                intent: self.plan.intent,
                running: self.is_running(),
                recovering: matches!(self.phase, RunnerPhase::Recovering { .. }),
                progress: self.progress(),
                axis_count: self.plan.len,
                confirmed_mask: self.confirmed_mask,
                uncertain_mask: self.ambiguous_mask | (driver_uncertain_mask & self.active_mask),
                steps: self.steps,
            }
        }

        /// Drains inline staged events (zero heap alloc) into a caller closure (e.g. `StreamProducer::try_push`).
        pub fn drain_events(&mut self, session_id: u64, mut sink: impl FnMut(ModeSwitchEvent)) {
            let n = self.pending_event_len;
            for slot in &mut self.pending_events[..n] {
                if let Some(mut ev) = slot.take() {
                    ev.session_id = session_id;
                    sink(ev);
                }
            }
            self.pending_event_len = 0;
        }

        fn push_event(
            &mut self,
            kind: ModeSwitchEventKind,
            joint: u8,
            mode: Option<ControlMode>,
            reason: Option<ModeSwitchFailure>,
        ) {
            if self.pending_event_len < self.pending_events.len() {
                self.pending_events[self.pending_event_len] = Some(ModeSwitchEvent {
                    session_id: 0,
                    kind,
                    joint,
                    mode,
                    reason,
                });
                self.pending_event_len += 1;
            }
        }

        fn first_switched_forward_joint(&self) -> Option<u8> {
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            for i in 0..len {
                if self.slots[i].switched_forward {
                    return Some(self.slots[i].target.joint);
                }
            }
            None
        }

        /// Single revocation decision shared by in-step pre-TX gate revocation and external
        /// revocation. `stop_applied` must be owner proof that the stop/E-stop was APPLIED; a
        /// pending stop (or a bare `"STOPPED"` code) never qualifies. Without that proof an
        /// energized partial switch escalates to `Ambiguous` (SAFE-HOLD).
        fn revocation_outcome(
            &mut self,
            drv_energized: bool,
            drv_uncertain: u32,
            reason: ModeSwitchFailure,
            stop_applied: bool,
        ) -> ModeProgress {
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            let mut in_flight_write_joint: Option<u8> = None;
            for i in 0..len {
                match self.slots[i].phase {
                    AxisPhase::AwaitingWriteAck { .. }
                    | AxisPhase::NeedReadback { from_write: true, .. }
                    | AxisPhase::AwaitingReadback { from_write: true, .. }
                    | AxisPhase::AwaitingFeedback { from_write: true, .. } => {
                        let j = self.slots[i].target.joint;
                        self.ambiguous_mask |= 1u32 << j;
                        in_flight_write_joint.get_or_insert(j);
                    }
                    AxisPhase::AwaitingFeedback {
                        fenced_pending_feedback: true,
                        ..
                    } if drv_energized => {
                        let j = self.slots[i].target.joint;
                        self.ambiguous_mask |= 1u32 << j;
                        in_flight_write_joint.get_or_insert(j);
                    }
                    _ => {}
                }
            }
            if !stop_applied && drv_energized {
                if let Some(j) = self.first_switched_forward_joint() {
                    self.ambiguous_mask |= 1u32 << j;
                    in_flight_write_joint.get_or_insert(j);
                }
            }
            self.ambiguous_mask |= drv_uncertain & self.active_mask;
            if self.ambiguous_mask != 0 {
                let fj = in_flight_write_joint
                    .unwrap_or_else(|| self.ambiguous_mask.trailing_zeros() as u8);
                ModeProgress::Ambiguous {
                    failed_joint: fj,
                    reason,
                }
            } else {
                ModeProgress::StoppedRecovered { reason }
            }
        }

        /// Reconciles an external revocation (stop, cancel, disconnect, deadline expiry, epoch or
        /// operator revocation, fault, SAFE-HOLD, handback) even if `step` does not run again.
        ///
        /// Checks runner state, the driver's authoritative `mode_write_uncertain_mask`, and
        /// whether an energized partial switch has already committed without an applied stop.
        pub fn reconcile_external_revocation(
            &mut self,
            drv: Option<&mut dyn crate::driver::ArmDriver>,
            reason: ModeSwitchFailure,
            stop_applied: bool,
        ) -> ModeProgress {
            if let RunnerPhase::Done(p) = self.phase {
                if let Some(d) = drv {
                    d.set_mode_switch_active_mask(0);
                }
                return p;
            }
            let drv_energized = drv.as_ref().is_some_and(|d| d.any_axis_energized());
            let drv_uncertain = if let Some(d) = drv {
                d.set_mode_switch_active_mask(0);
                d.mode_write_uncertain_mask()
            } else {
                0
            };
            let outcome = self.revocation_outcome(drv_energized, drv_uncertain, reason, stop_applied);
            match outcome {
                ModeProgress::Ambiguous { failed_joint, .. } => self.push_event(
                    ModeSwitchEventKind::TerminalAmbiguous,
                    failed_joint,
                    None,
                    Some(reason),
                ),
                _ => self.push_event(
                    ModeSwitchEventKind::TerminalStoppedRecovered,
                    0,
                    None,
                    Some(reason),
                ),
            }
            self.phase = RunnerPhase::Done(outcome);
            outcome
        }

        /// Spec entry point: advances the mode switch by one bounded release tick on an
        /// [`ExclusiveDriver`](crate::supervisor_lock::ExclusiveDriver).
        pub fn step(
            &mut self,
            drv: &mut crate::supervisor_lock::ExclusiveDriver,
            budget: ReleaseBudget,
        ) -> ModeProgress {
            let mut allow = || Ok(());
            self.step_with_gate(drv.get_mut().as_mut(), &mut allow, budget)
        }

        /// Advances the mode switch by one bounded release tick on `&mut dyn ArmDriver` with a
        /// live pre-TX authority gate (e.g. `SupervisorInner::check_session_pre_tx`).
        pub fn step_with_gate(
            &mut self,
            drv: &mut dyn crate::driver::ArmDriver,
            pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
            budget: ReleaseBudget,
        ) -> ModeProgress {
            let mut adapter = ArmDriverModeAdapter::new(drv, pre_tx_gate);
            self.step_driver(&mut adapter, budget)
        }

        /// [`Self::step_with_gate`] plus the owner's "stop applied" proof (see
        /// [`ModeSwitchDriver::stop_applied`]); a gate revocation without proof is treated as a
        /// pending stop.
        pub fn step_with_gate_and_stop_proof(
            &mut self,
            drv: &mut dyn crate::driver::ArmDriver,
            pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
            stop_applied: &dyn Fn() -> bool,
            budget: ReleaseBudget,
        ) -> ModeProgress {
            let mut adapter = ArmDriverModeAdapter::new(drv, pre_tx_gate).with_stop_proof(stop_applied);
            self.step_driver(&mut adapter, budget)
        }

        /// Advances the mode switch by one bounded release tick while drawing RX frames, TX
        /// attempts, and the cutoff from a shared driver-clock
        /// [`crate::rt_core::budget::ReleaseBudget`] ledger (tightening only; charges consumed
        /// RX/TX back into `ledger`).
        pub fn step_with_gate_and_stop_proof_ledger(
            &mut self,
            drv: &mut dyn crate::driver::ArmDriver,
            pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
            stop_applied: &dyn Fn() -> bool,
            budget: ReleaseBudget,
            ledger: &mut crate::rt_core::budget::ReleaseBudget,
        ) -> ModeProgress {
            let mut adapter = ArmDriverModeAdapter::new(drv, pre_tx_gate).with_stop_proof(stop_applied);
            self.step_driver_with_ledger(&mut adapter, budget, ledger)
        }

        /// Evaluates final watchdog holds on an energized terminal transition and commits the
        /// SINGLE terminal outcome via [`Self::finish`] (R3 Major 4: never emits two contradictory
        /// terminal events for one session).
        fn finalize_with_watchdog_check(
            &mut self,
            drv: &mut dyn ModeSwitchDriver,
            budget: ReleaseBudget,
            sub: &mut crate::rt_core::budget::SubBudget<'_>,
            candidate: ModeProgress,
        ) -> ModeProgress {
            if !candidate.is_ambiguous() && drv.any_axis_energized() {
                let exclude_mask = drv.mode_write_uncertain_mask();
                let effective_gap = self.effective_max_watchdog_gap(budget);
                let tx_cap = sub.tx_remaining().min(u16::from(u8::MAX)) as u8;
                match drv.service_watchdog_holds(
                    self.hold_due_age(budget),
                    exclude_mask,
                    0,
                    sub.deadline(),
                    tx_cap,
                ) {
                    Ok(wd) => {
                        sub.charge_tx(u16::from(wd.frames_sent));
                        if !wd.all_due_serviced
                            && wd
                                .max_energized_age
                                .saturating_add(budget.period)
                                .saturating_add(budget.hold_delivery_margin)
                                > effective_gap
                        {
                            let fj = wd.oldest_joint.unwrap_or_else(|| self.current_joint());
                            self.ambiguous_mask |= 1u32 << fj;
                            return self.finish(
                                drv,
                                ModeProgress::Ambiguous {
                                    failed_joint: fj,
                                    reason: ModeSwitchFailure::WatchdogBudgetBreached,
                                },
                            );
                        }
                    }
                    Err(reason) => {
                        let fj = self.current_joint();
                        return self.finish(
                            drv,
                            ModeProgress::Ambiguous {
                                failed_joint: fj,
                                reason,
                            },
                        );
                    }
                }
            }
            self.finish(drv, candidate)
        }

        /// Advances the mode switch by one bounded release tick against any [`ModeSwitchDriver`].
        pub fn step_driver(
            &mut self,
            drv: &mut dyn ModeSwitchDriver,
            budget: ReleaseBudget,
        ) -> ModeProgress {
            let mut local_ledger = budget.to_ledger();
            self.step_driver_with_ledger(drv, budget, &mut local_ledger)
        }

        /// Advances the mode switch by one bounded release tick against any [`ModeSwitchDriver`],
        /// tightening a sub-budget from `ledger` and charging all RX frames and TX attempts back
        /// into `ledger`.
        pub fn step_driver_with_ledger(
            &mut self,
            drv: &mut dyn ModeSwitchDriver,
            mut budget: ReleaseBudget,
            ledger: &mut crate::rt_core::budget::ReleaseBudget,
        ) -> ModeProgress {
            if let RunnerPhase::Done(done) = self.phase {
                return done;
            }
            let mut sub = ledger.sub_budget(
                budget.cutoff,
                budget.max_rx_frames,
                u16::from(budget.max_tx_frames),
            );
            budget.cutoff = sub.deadline();
            budget.max_rx_frames = sub.rx_remaining();
            budget.max_tx_frames = sub.tx_remaining().min(u16::from(u8::MAX)) as u8;
            self.steps = self.steps.saturating_add(1);
            // R2 Minor 1: validate plan BEFORE the empty-plan branch so direct callers receive
            // `InvalidPlan` rather than `Confirmed` on an empty or malformed plan.
            if matches!(self.phase, RunnerPhase::Init) && self.plan.validate().is_err() {
                return self.finish(
                    drv,
                    ModeProgress::FailedClean {
                        failed_joint: self.slots[0].target.joint,
                        reason: ModeSwitchFailure::InvalidPlan,
                    },
                );
            }
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            if len == 0 {
                return self.finish(
                    drv,
                    ModeProgress::FailedClean {
                        failed_joint: 0,
                        reason: ModeSwitchFailure::InvalidPlan,
                    },
                );
            }

            // 0. One-time preflight & vendor capability validation before any hardware TX.
            if matches!(self.phase, RunnerPhase::Init) {
                if let Err(prog) = self.initialize(drv, budget) {
                    return self.finish(drv, prog);
                }
                drv.set_mode_switch_active_mask(self.active_mask);
                self.push_event(ModeSwitchEventKind::Started, self.slots[0].target.joint, None, None);
                self.phase = RunnerPhase::Forward;
                self.advance_completed_slots(drv);
                if self.active_idx >= len {
                    return self.finalize_with_watchdog_check(drv, budget, &mut sub, ModeProgress::Confirmed);
                }
            }

            // 1. Drain any newly arrived RX frames up to the shared per-release RX budget so
            //    causal replies arriving between Step 2 RX and Step 2b (and confirmation closure
            //    via `last_read_drained`) see all pending wire replies before evaluation.
            let rx_budget = sub
                .rx_remaining()
                .min(drv.remaining_rx_budget(sub.rx_remaining()));
            if rx_budget > 0 && drv.clock_now() < sub.deadline() {
                match drv.drain_rx_step(rx_budget, sub.deadline()) {
                    Ok((frames, drained)) => {
                        sub.charge_rx(frames, !drained);
                    }
                    Err(reason) => {
                        return self.handle_axis_failure(drv, self.current_joint(), reason, false);
                    }
                }
            }

            // 2. Evaluate causal replies and post-write deadlines for the active in-flight axis
            //    BEFORE watchdog holds, so a newly confirmed axis immediately receives its first
            //    hold in its new mode on THIS tick (keeping watchdog gap <= 1 period = 20 ms at 50 Hz).
            if let Some(candidate) = self.poll_active_inflight(drv, budget) {
                if matches!(self.phase, RunnerPhase::Done(_)) || !candidate.is_terminal() {
                    return candidate;
                }
                return self.finalize_with_watchdog_check(drv, budget, &mut sub, candidate);
            }
            if matches!(self.phase, RunnerPhase::Done(_)) {
                return self.progress();
            }

            // 3. WATCHDOG BUDGET FIRST: service all due energized confirmed axes before spending
            //    any TX budget on mode-switch register frames. If the active axis is about to
            //    transmit `REG_WRITE` while energized with a watchdog, proactively refresh it in
            //    its last confirmed mode (`force_mask`) so its watchdog window starts fresh.
            if drv.any_axis_energized() {
                let exclude_mask = drv.mode_write_uncertain_mask();
                let rem_tx = sub.tx_remaining().min(u16::from(u8::MAX)) as u8;
                let force_mask = self.next_write_force_refresh_mask(drv, rem_tx);
                let wd = match drv.service_watchdog_holds(
                    self.hold_due_age(budget),
                    exclude_mask,
                    force_mask,
                    sub.deadline(),
                    rem_tx,
                ) {
                    Ok(res) => res,
                    Err(reason) => {
                        let fj = self.current_joint();
                        let uncertain = drv.mode_write_uncertain_mask() != 0;
                        return self.handle_axis_failure(drv, fj, reason, uncertain);
                    }
                };
                sub.charge_tx(u16::from(wd.frames_sent));
                // Predictive starvation guard: if any energized watchdog-bearing confirmed axis
                // was NOT refreshed and waiting until the next release would breach `max_watchdog_gap`,
                // abort fail-closed BEFORE the 40 ms window closes.
                let effective_gap = self.effective_max_watchdog_gap(budget);
                if !wd.all_due_serviced
                    && wd.max_energized_age.saturating_add(budget.period).saturating_add(budget.hold_delivery_margin)
                        > effective_gap
                {
                    let fj = wd.oldest_joint.unwrap_or_else(|| self.current_joint());
                    return self.handle_axis_failure(
                        drv,
                        fj,
                        ModeSwitchFailure::WatchdogBudgetBreached,
                        true,
                    );
                }
                if !wd.all_due_serviced || !sub.can_tx(drv.clock_now()) {
                    return ModeProgress::Pending { wake: None };
                }
            }

            if !sub.can_tx(drv.clock_now()) {
                return ModeProgress::Pending { wake: None };
            }

            // 4. Advance the active axis's register/refresh TX using remaining budget.
            self.step_active_tx(drv, budget, &mut sub)
        }

        fn effective_max_watchdog_gap(&self, budget: ReleaseBudget) -> Duration {
            self.plan.max_watchdog_gap.min(budget.max_watchdog_gap)
        }

        /// Control-frame age at which an energized watchdog axis is due for a hold on THIS release
        /// (Major 1, R5): the latest age from which waiting one more release period plus the
        /// delivery margin still lands inside the effective gap, capped by the proactive interval.
        /// Applies to every armed axis the driver services, inside or outside the plan.
        fn hold_due_age(&self, budget: ReleaseBudget) -> Duration {
            let latest_safe = self
                .effective_max_watchdog_gap(budget)
                .saturating_sub(budget.period)
                .saturating_sub(budget.hold_delivery_margin);
            budget.watchdog_proactive_interval.min(latest_safe)
        }

        fn current_joint(&self) -> u8 {
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            if self.active_idx < len {
                self.slots[self.active_idx].target.joint
            } else {
                0
            }
        }

        fn initialize(
            &mut self,
            drv: &dyn ModeSwitchDriver,
            budget: ReleaseBudget,
        ) -> Result<(), ModeProgress> {
            if self.plan.validate().is_err() {
                return Err(ModeProgress::FailedClean {
                    failed_joint: self.slots[0].target.joint,
                    reason: ModeSwitchFailure::InvalidPlan,
                });
            }
            if !drv.supports_nonblocking_mode_switch() {
                return Err(ModeProgress::FailedClean {
                    failed_joint: self.slots[0].target.joint,
                    reason: ModeSwitchFailure::UnsupportedMode,
                });
            }
            let dof = drv.dof();
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            let vendor = drv.vendor();
            let mut seen_mask = 0u32;
            let effective_gap = self.effective_max_watchdog_gap(budget);
            // Major M1 (R6): refuse an infeasible gap before any TX; holds would come due on
            // every release and the overrun check could not catch the breach.
            if effective_gap < budget.period.saturating_add(budget.hold_delivery_margin) {
                return Err(ModeProgress::FailedClean {
                    failed_joint: self.slots[0].target.joint,
                    reason: ModeSwitchFailure::WatchdogGapInfeasible,
                });
            }

            for i in 0..len {
                let mut target = self.slots[i].target;
                let j = usize::from(target.joint);
                if j >= dof || j >= MAX_MODE_SWITCH_AXES || (seen_mask & (1u32 << j)) != 0 {
                    return Err(ModeProgress::FailedClean {
                        failed_joint: target.joint,
                        reason: ModeSwitchFailure::InvalidPlan,
                    });
                }
                seen_mask |= 1u32 << j;
                if let Some(v) = vendor {
                    if !v.supports_mode(target.target_mode) {
                        return Err(ModeProgress::FailedClean {
                            failed_joint: target.joint,
                            reason: ModeSwitchFailure::UnsupportedMode,
                        });
                    }
                }
                // Blocker 2: PositionForce is strictly bound to the configured PosForce joint,
                // and its causal feedback / expected reply ID / disarmed stationary checks are
                // mandatory (cannot be disabled via public AxisTarget fields).
                if target.target_mode == ControlMode::PositionForce {
                    if !drv.axis_supports_pos_force(j) {
                        return Err(ModeProgress::FailedClean {
                            failed_joint: target.joint,
                            reason: ModeSwitchFailure::UnsupportedMode,
                        });
                    }
                    target.require_post_confirm_feedback = true;
                    target.require_expected_reply_id = true;
                    target.require_disarmed_stationary = true;
                    self.slots[i].target = target;
                }
                let prev = drv.axis_confirmed_mode(j);
                let energized = drv.axis_is_energized(j);
                self.slots[i].previous_mode = prev;
                self.slots[i].active_goal = target.target_mode;
                self.slots[i].switched_forward = false;
                self.slots[i].started_energized = energized;

                match self.plan.intent {
                    ModeSwitchIntent::ConfirmOnly => {
                        // Blocker 3: Even read-only ConfirmOnly(Impedance) must verify wire
                        // PMAX/VMAX/TMAX ranges before Impedance becomes eligible for MIT holds.
                        if target.target_mode == ControlMode::Impedance
                            && !drv.axis_impedance_ranges_verified(j)
                        {
                            self.slots[i].phase = AxisPhase::RangeReadNeed {
                                rid_idx: 0,
                                got: [None; 3],
                            };
                        } else {
                            self.slots[i].phase = AxisPhase::NeedReadback {
                                expected_mode: target.target_mode,
                                from_write: false,
                                last_ctrl_at: None,
                                post_write_deadline: None,
                            };
                        }
                    }
                    ModeSwitchIntent::Switch => {
                        // R2 Blocker 2: PositionForce ALWAYS requires fresh causal readback +
                        // master-ID feedback (never shortcut from a cached PosForce flag).
                        if target.target_mode == ControlMode::PositionForce
                            && prev == Some(ControlMode::PositionForce)
                        {
                            self.slots[i].phase = AxisPhase::NeedReadback {
                                expected_mode: ControlMode::PositionForce,
                                from_write: false,
                                last_ctrl_at: None,
                                post_write_deadline: None,
                            };
                            continue;
                        }
                        if prev == Some(target.target_mode)
                            && (target.target_mode != ControlMode::Impedance
                                || drv.axis_impedance_ranges_verified(j))
                        {
                            self.slots[i].phase = AxisPhase::Confirmed {
                                mode: target.target_mode,
                            };
                            self.confirmed_mask |= 1u32 << j;
                            continue;
                        }
                        if energized {
                            let Some(from_mode) = prev else {
                                return Err(ModeProgress::FailedClean {
                                    failed_joint: target.joint,
                                    reason: ModeSwitchFailure::NoConfirmedMode,
                                });
                            };
                            if let Some(v) = vendor {
                                if !v.supports_runtime_switch(from_mode, target.target_mode) {
                                    return Err(ModeProgress::FailedClean {
                                        failed_joint: target.joint,
                                        reason: ModeSwitchFailure::UnsupportedMode,
                                    });
                                }
                            }
                            // Predictive admission check: verify the release period can fit a
                            // full write -> causal confirmation -> next hold delivery inside
                            // `effective_gap` for watchdog-bearing axes BEFORE touching hardware.
                            if drv.axis_has_watchdog(j) {
                                let ticks_needed: u32 = if target.require_post_write_readback { 2 } else { 1 };
                                let min_needed = budget
                                    .period
                                    .saturating_mul(ticks_needed)
                                    .saturating_add(budget.hold_delivery_margin);
                                if min_needed > effective_gap {
                                    return Err(ModeProgress::FailedClean {
                                        failed_joint: target.joint,
                                        reason: ModeSwitchFailure::WatchdogBudgetBreached,
                                    });
                                }
                            }
                        }
                        // Unconditional wire-range verification before ANY Impedance write.
                        if target.target_mode == ControlMode::Impedance
                            && !drv.axis_impedance_ranges_verified(j)
                        {
                            self.slots[i].phase = AxisPhase::RangeReadNeed {
                                rid_idx: 0,
                                got: [None; 3],
                            };
                        } else {
                            self.slots[i].phase = AxisPhase::NeedWrite;
                        }
                    }
                }
            }
            Ok(())
        }

        fn next_write_force_refresh_mask(&self, drv: &dyn ModeSwitchDriver, remaining_tx: u8) -> u32 {
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            if self.active_idx >= len || remaining_tx == 0 {
                return 0;
            }
            let slot = &self.slots[self.active_idx];
            if matches!(slot.phase, AxisPhase::NeedWrite) {
                let j = usize::from(slot.target.joint);
                if drv.axis_is_energized(j) && drv.axis_has_watchdog(j) {
                    return 1u32 << j;
                }
            }
            0
        }

        fn advance_completed_slots(&mut self, drv: &dyn ModeSwitchDriver) {
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            let _ = drv;
            while self.active_idx < len {
                if matches!(self.slots[self.active_idx].phase, AxisPhase::Confirmed { .. }) {
                    self.active_idx += 1;
                } else {
                    break;
                }
            }
        }

        fn poll_active_inflight(
            &mut self,
            drv: &mut dyn ModeSwitchDriver,
            budget: ReleaseBudget,
        ) -> Option<ModeProgress> {
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            if self.active_idx >= len {
                return None;
            }
            let idx = self.active_idx;
            let slot = self.slots[idx];
            let j = usize::from(slot.target.joint);
            let now = drv.clock_now();
            let effective_gap = self.effective_max_watchdog_gap(budget);

            match slot.phase {
                AxisPhase::RangeReadAwait {
                    rid_idx,
                    boundary_seq,
                    tx_at,
                    tx_offset_ns,
                    deadline,
                    mut got,
                } => {
                    // R2 Major 1: check live deadline BEFORE accepting a reply.
                    if now >= deadline {
                        return Some(self.handle_axis_failure(
                            drv,
                            slot.target.joint,
                            ModeSwitchFailure::RangeCheckTimeout,
                            false,
                        ));
                    }
                    match drv.find_range_reply(j, rid_idx, boundary_seq, tx_at, deadline, tx_offset_ns) {
                        ModeRangeMatch::Matched(val) => {
                            got[usize::from(rid_idx)] = Some(val);
                            let Some(want) = drv.axis_configured_impedance_ranges(j) else {
                                return Some(self.handle_axis_failure(
                                    drv,
                                    slot.target.joint,
                                    ModeSwitchFailure::RangeMismatch,
                                    false,
                                ));
                            };
                            let w = want[usize::from(rid_idx)];
                            if !val.is_finite() || (val - w).abs() > 0.01 + w.abs() * 1e-3 {
                                return Some(self.handle_axis_failure(
                                    drv,
                                    slot.target.joint,
                                    ModeSwitchFailure::RangeMismatch,
                                    false,
                                ));
                            }
                            if rid_idx + 1 < 3 {
                                self.slots[idx].phase = AxisPhase::RangeReadNeed {
                                    rid_idx: rid_idx + 1,
                                    got,
                                };
                            } else {
                                drv.set_axis_impedance_ranges_verified(j, true);
                                self.slots[idx].phase = if matches!(self.plan.intent, ModeSwitchIntent::ConfirmOnly) {
                                    AxisPhase::NeedReadback {
                                        expected_mode: slot.active_goal,
                                        from_write: false,
                                        last_ctrl_at: None,
                                        post_write_deadline: None,
                                    }
                                } else {
                                    AxisPhase::NeedWrite
                                };
                            }
                            None
                        }
                        ModeRangeMatch::Ambiguous => Some(self.handle_axis_failure(
                            drv,
                            slot.target.joint,
                            ModeSwitchFailure::AmbiguousReply,
                            false,
                        )),
                        ModeRangeMatch::Pending => None,
                    }
                }
                AxisPhase::AwaitingWriteAck {
                    boundary_seq,
                    tx_at,
                    tx_offset_ns,
                    deadline,
                    last_ctrl_at,
                    written_mode,
                } => {
                    // R2 Major 1: check absolute deadline BEFORE accepting a reply drained late!
                    if now >= deadline {
                        drv.set_axis_confirmed_mode(j, None);
                        return Some(self.handle_axis_failure(
                            drv,
                            slot.target.joint,
                            ModeSwitchFailure::WriteAckTimeout,
                            true,
                        ));
                    }
                    let req_id = slot.target.require_expected_reply_id
                        || written_mode == ControlMode::PositionForce;
                    match drv.find_ctrl_mode_reply(
                        j,
                        true,
                        boundary_seq,
                        tx_at,
                        deadline,
                        tx_offset_ns,
                        req_id,
                    ) {
                        ModeRegMatch::Matched {
                            mode: Some(got),
                            rx_seq,
                            rx_at,
                            decode_offset_ns,
                            ..
                        } if got == written_mode => {
                            if slot.target.require_post_write_readback
                                && matches!(self.phase, RunnerPhase::Forward)
                            {
                                // Blocker 6: preserve the SINGLE absolute post-write deadline across
                                // `NeedReadback` and `AwaitingReadback` so backpressure can never
                                // strand `NeedReadback`.
                                self.slots[idx].phase = AxisPhase::NeedReadback {
                                    expected_mode: written_mode,
                                    from_write: true,
                                    last_ctrl_at,
                                    post_write_deadline: Some(deadline),
                                };
                            } else if written_mode == ControlMode::PositionForce {
                                // R2 Blocker 2: NEVER publish `PositionForce` (nor clear uncertainty
                                // / enable `pos_force_verified`) before causal master-ID feedback passes!
                                drv.latch_axis_mode_uncertain(j);
                                let Some(fb_dl) = now.checked_add(self.plan.per_axis_timeout) else {
                                    return Some(self.handle_axis_failure(
                                        drv,
                                        slot.target.joint,
                                        ModeSwitchFailure::InvalidPlan,
                                        true,
                                    ));
                                };
                                self.slots[idx].phase = AxisPhase::AwaitingFeedback {
                                    after_seq: rx_seq,
                                    after_at: rx_at,
                                    confirm_offset_ns: decode_offset_ns,
                                    deadline: fb_dl,
                                    confirmed_mode: written_mode,
                                    from_write: true,
                                    fenced_pending_feedback: true,
                                    refresh_sent: false,
                                };
                            } else {
                                drv.set_axis_confirmed_mode(j, Some(written_mode));
                                if matches!(self.phase, RunnerPhase::Forward) {
                                    self.slots[idx].switched_forward = true;
                                }
                                if slot.target.require_post_confirm_feedback {
                                    let Some(fb_dl) = now.checked_add(self.plan.per_axis_timeout) else {
                                        return Some(self.handle_axis_failure(
                                            drv,
                                            slot.target.joint,
                                            ModeSwitchFailure::InvalidPlan,
                                            false,
                                        ));
                                    };
                                    self.slots[idx].phase = AxisPhase::AwaitingFeedback {
                                        after_seq: rx_seq,
                                        after_at: rx_at,
                                        confirm_offset_ns: decode_offset_ns,
                                        deadline: fb_dl,
                                        confirmed_mode: written_mode,
                                        from_write: false,
                                        fenced_pending_feedback: false,
                                        refresh_sent: false,
                                    };
                                } else {
                                    return self.mark_slot_confirmed(drv, idx, written_mode);
                                }
                            }
                            None
                        }
                        ModeRegMatch::Matched { .. } => {
                            // Causal write ACK reported a different/unknown mode -> ambiguous!
                            drv.set_axis_confirmed_mode(j, None);
                            Some(self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::WriteAckMismatch,
                                true,
                            ))
                        }
                        ModeRegMatch::Ambiguous => {
                            drv.set_axis_confirmed_mode(j, None);
                            Some(self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::AmbiguousReply,
                                true,
                            ))
                        }
                        ModeRegMatch::Pending => {
                            // Predictive watchdog bound check: if waiting until next release
                            // would breach `effective_gap` from `last_ctrl_at`, fail closed NOW
                            // (on this release) instead of letting the gap exceed 40 ms!
                            let wd_expired = drv.axis_is_energized(j)
                                && drv.axis_has_watchdog(j)
                                && last_ctrl_at.is_some_and(|t0| {
                                    now.saturating_duration_since(t0)
                                        .saturating_add(budget.period)
                                        .saturating_add(budget.hold_delivery_margin)
                                        > effective_gap
                                });
                            if wd_expired {
                                drv.set_axis_confirmed_mode(j, None);
                                return Some(self.handle_axis_failure(
                                    drv,
                                    slot.target.joint,
                                    ModeSwitchFailure::WriteAckTimeout,
                                    true,
                                ));
                            }
                            None
                        }
                    }
                }
                AxisPhase::NeedReadback {
                    from_write: true,
                    last_ctrl_at,
                    post_write_deadline,
                    ..
                } => {
                    // Blocker 6: evaluate the single absolute post-write deadline BEFORE any
                    // budget/backpressure/RX-drain return so `NeedReadback` can never be stranded!
                    let wd_expired = drv.axis_is_energized(j)
                        && drv.axis_has_watchdog(j)
                        && last_ctrl_at.is_some_and(|t0| {
                            now.saturating_duration_since(t0)
                                .saturating_add(budget.period)
                                .saturating_add(budget.hold_delivery_margin)
                                > effective_gap
                        });
                    if post_write_deadline.is_some_and(|dl| now >= dl) || wd_expired {
                        drv.set_axis_confirmed_mode(j, None);
                        return Some(self.handle_axis_failure(
                            drv,
                            slot.target.joint,
                            ModeSwitchFailure::ReadbackTimeout,
                            true,
                        ));
                    }
                    None
                }
                AxisPhase::AwaitingReadback {
                    boundary_seq,
                    tx_at,
                    tx_offset_ns,
                    deadline,
                    expected_mode,
                    from_write,
                    last_ctrl_at,
                } => {
                    // R2 Major 1: check live deadline BEFORE accepting a reply!
                    if now >= deadline {
                        if from_write {
                            drv.set_axis_confirmed_mode(j, None);
                        }
                        return Some(self.handle_axis_failure(
                            drv,
                            slot.target.joint,
                            ModeSwitchFailure::ReadbackTimeout,
                            from_write,
                        ));
                    }
                    let req_id = slot.target.require_expected_reply_id
                        || expected_mode == ControlMode::PositionForce;
                    match drv.find_ctrl_mode_reply(
                        j,
                        false,
                        boundary_seq,
                        tx_at,
                        deadline,
                        tx_offset_ns,
                        req_id,
                    ) {
                        ModeRegMatch::Matched {
                            mode: Some(got),
                            rx_seq,
                            rx_at,
                            decode_offset_ns,
                            ..
                        } if got == expected_mode => {
                            // Blocker 2 (R5): a causal ConfirmOnly reply is published only when it
                            // agrees with the cached mode; on an energized axis a contradiction
                            // fences the axis and escalates to SAFE-HOLD.
                            if !from_write
                                && drv.axis_is_energized(j)
                                && drv.axis_confirmed_mode(j).is_some_and(|c| c != got)
                            {
                                drv.latch_axis_mode_uncertain(j);
                                return Some(self.handle_axis_failure(
                                    drv,
                                    slot.target.joint,
                                    ModeSwitchFailure::ReadbackMismatch,
                                    true,
                                ));
                            }
                            if expected_mode == ControlMode::PositionForce {
                                // R2 Blocker 2: ALWAYS fence the axis (`latch_axis_mode_uncertain`)
                                // while mandatory PositionForce causal master-ID feedback is pending,
                                // so neither stale cached PosVel holds nor premature PosForce holds
                                // can be transmitted before causal feedback succeeds!
                                drv.latch_axis_mode_uncertain(j);
                                let Some(fb_dl) = now.checked_add(self.plan.per_axis_timeout) else {
                                    let ambiguous = from_write || drv.axis_is_energized(j);
                                    return Some(self.handle_axis_failure(
                                        drv,
                                        slot.target.joint,
                                        ModeSwitchFailure::InvalidPlan,
                                        ambiguous,
                                    ));
                                };
                                self.slots[idx].phase = AxisPhase::AwaitingFeedback {
                                    after_seq: rx_seq,
                                    after_at: rx_at,
                                    confirm_offset_ns: decode_offset_ns,
                                    deadline: fb_dl,
                                    confirmed_mode: expected_mode,
                                    from_write,
                                    fenced_pending_feedback: true,
                                    refresh_sent: false,
                                };
                                None
                            } else {
                                drv.set_axis_confirmed_mode(j, Some(expected_mode));
                                if from_write && matches!(self.phase, RunnerPhase::Forward) {
                                    self.slots[idx].switched_forward = true;
                                }
                                if slot.target.require_post_confirm_feedback {
                                    let Some(fb_dl) = now.checked_add(self.plan.per_axis_timeout) else {
                                        return Some(self.handle_axis_failure(
                                            drv,
                                            slot.target.joint,
                                            ModeSwitchFailure::InvalidPlan,
                                            false,
                                        ));
                                    };
                                    self.slots[idx].phase = AxisPhase::AwaitingFeedback {
                                        after_seq: rx_seq,
                                        after_at: rx_at,
                                        confirm_offset_ns: decode_offset_ns,
                                        deadline: fb_dl,
                                        confirmed_mode: expected_mode,
                                        from_write: false,
                                        fenced_pending_feedback: false,
                                        refresh_sent: false,
                                    };
                                    None
                                } else {
                                    self.mark_slot_confirmed(drv, idx, expected_mode)
                                }
                            }
                        }
                        ModeRegMatch::Matched { mode: got_mode, .. } => {
                            if from_write {
                                drv.set_axis_confirmed_mode(j, None);
                                return Some(self.handle_axis_failure(
                                    drv,
                                    slot.target.joint,
                                    ModeSwitchFailure::ReadbackMismatch,
                                    true,
                                ));
                            }
                            // Blocker 3: Distinguish ConfirmOnly mismatch that agrees with cached
                            // verified state vs one that CONTRADICTS cached verified state!
                            let cached = drv.axis_confirmed_mode(j);
                            let contradicts_cached = match (cached, got_mode) {
                                (Some(c), Some(g)) => c != g,
                                (Some(_), None) => true,
                                (None, _) => false,
                            };
                            if contradicts_cached {
                                drv.set_axis_confirmed_mode(j, None);
                                let ambiguous = drv.axis_is_energized(j);
                                return Some(self.handle_axis_failure(
                                    drv,
                                    slot.target.joint,
                                    ModeSwitchFailure::ReadbackMismatch,
                                    ambiguous,
                                ));
                            }
                            Some(self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::ReadbackMismatch,
                                false,
                            ))
                        }
                        ModeRegMatch::Ambiguous => {
                            // Blocker 4: invalidate cached evidence on ambiguous confirmation
                            // (duplicate, overflow, clock-drift, or unexpected reply ID) and
                            // escalate to same-release SAFE-HOLD whenever from_write or energized!
                            drv.set_axis_confirmed_mode(j, None);
                            let ambiguous = from_write || drv.axis_is_energized(j);
                            Some(self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::AmbiguousReply,
                                ambiguous,
                            ))
                        }
                        ModeRegMatch::Pending => {
                            let wd_expired = from_write
                                && drv.axis_is_energized(j)
                                && drv.axis_has_watchdog(j)
                                && last_ctrl_at.is_some_and(|t0| {
                                    now.saturating_duration_since(t0)
                                        .saturating_add(budget.period)
                                        .saturating_add(budget.hold_delivery_margin)
                                        > effective_gap
                                });
                            if wd_expired {
                                drv.set_axis_confirmed_mode(j, None);
                                return Some(self.handle_axis_failure(
                                    drv,
                                    slot.target.joint,
                                    ModeSwitchFailure::ReadbackTimeout,
                                    true,
                                ));
                            }
                            None
                        }
                    }
                }
                AxisPhase::AwaitingFeedback {
                    after_seq,
                    after_at,
                    confirm_offset_ns,
                    deadline,
                    confirmed_mode,
                    from_write,
                    fenced_pending_feedback,
                    ..
                } => {
                    // R3 Blocker 2: evaluate disarmed/stationary against `slot.started_energized`
                    // captured BEFORE feedback RX, so a contradictory sample reporting `state == 1`
                    // cannot flip `axis_is_energized(j)` and bypass its own disarmed/stationary gate!
                    let disarmed_stat = (slot.target.require_disarmed_stationary
                        || confirmed_mode == ControlMode::PositionForce)
                        && !slot.started_energized;
                    let fb = if now >= deadline {
                        ModeFeedbackMatch::Pending
                    } else {
                        drv.post_confirm_feedback(
                            j,
                            confirmed_mode,
                            after_seq,
                            after_at,
                            deadline,
                            confirm_offset_ns,
                            now,
                            disarmed_stat,
                        )
                    };
                    match fb {
                        ModeFeedbackMatch::FreshConfirmed => {
                            if confirmed_mode == ControlMode::PositionForce {
                                // R2 Blocker 2: release fence & publish PositionForce ONLY after
                                // causal master-ID feedback!
                                drv.set_axis_confirmed_mode(j, Some(confirmed_mode));
                                if from_write && matches!(self.phase, RunnerPhase::Forward) {
                                    self.slots[idx].switched_forward = true;
                                }
                            }
                            self.mark_slot_confirmed(drv, idx, confirmed_mode)
                        }
                        ModeFeedbackMatch::Unsafe => {
                            if confirmed_mode == ControlMode::PositionForce || fenced_pending_feedback {
                                drv.set_axis_confirmed_mode(j, None);
                            }
                            let ambiguous = from_write || drv.axis_is_energized(j);
                            Some(self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::FeedbackUnsafe,
                                ambiguous,
                            ))
                        }
                        ModeFeedbackMatch::Pending => {
                            if now >= deadline {
                                // R2 Blocker 2: energized PosForce feedback timeout with incomplete
                                // proof MUST escalate as Ambiguous (never FailedClean).
                                if confirmed_mode == ControlMode::PositionForce || fenced_pending_feedback {
                                    drv.set_axis_confirmed_mode(j, None);
                                }
                                let ambiguous = from_write
                                    || (fenced_pending_feedback && drv.axis_is_energized(j));
                                return Some(self.handle_axis_failure(
                                    drv,
                                    slot.target.joint,
                                    ModeSwitchFailure::FeedbackTimeout,
                                    ambiguous,
                                ));
                            }
                            None
                        }
                    }
                }
                _ => None,
            }
        }

        fn step_active_tx(
            &mut self,
            drv: &mut dyn ModeSwitchDriver,
            budget: ReleaseBudget,
            sub: &mut crate::rt_core::budget::SubBudget<'_>,
        ) -> ModeProgress {
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            if self.active_idx >= len || sub.tx_remaining() == 0 {
                return ModeProgress::Pending { wake: None };
            }
            let idx = self.active_idx;
            let slot = self.slots[idx];
            let j = usize::from(slot.target.joint);
            let now = drv.clock_now();
            if sub.deadline_exceeded(now) {
                return ModeProgress::Pending { wake: None };
            }
            let effective_gap = self.effective_max_watchdog_gap(budget);

            match slot.phase {
                AxisPhase::RangeReadNeed { rid_idx, got } => {
                    if !drv.rx_drained() {
                        return ModeProgress::Pending { wake: None };
                    }
                    if now.checked_add(self.plan.per_axis_timeout).is_none() {
                        return self.handle_axis_failure(
                            drv,
                            slot.target.joint,
                            ModeSwitchFailure::InvalidPlan,
                            false,
                        );
                    }
                    drv.clear_register_reply_log();
                    match drv.try_send_range_read(j, rid_idx, sub.deadline()) {
                        ModeTxResult::Sent {
                            boundary_seq,
                            tx_at,
                            tx_offset_ns,
                        } => {
                            sub.charge_tx(1);
                            let Some(deadline) = tx_at.checked_add(self.plan.per_axis_timeout) else {
                                return self.handle_axis_failure(
                                    drv,
                                    slot.target.joint,
                                    ModeSwitchFailure::InvalidPlan,
                                    false,
                                );
                            };
                            self.slots[idx].phase = AxisPhase::RangeReadAwait {
                                rid_idx,
                                boundary_seq,
                                tx_at,
                                tx_offset_ns,
                                deadline,
                                got,
                            };
                            ModeProgress::Pending { wake: Some(deadline) }
                        }
                        ModeTxResult::Backpressure => {
                            sub.charge_tx(1);
                            ModeProgress::Pending { wake: None }
                        }
                        ModeTxResult::Revoked(code) => self.handle_revocation(drv, code),
                        ModeTxResult::SentFlushFailed { .. } => {
                            sub.charge_tx(1);
                            self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::TxFailed,
                                false,
                            )
                        }
                        ModeTxResult::Refused(_) => {
                            self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::TxFailed,
                                false,
                            )
                        }
                    }
                }
                AxisPhase::NeedWrite => {
                    // For energized watchdog-bearing axes, ensure `last_ctrl_tx` was freshly
                    // delivered so `now + needed_periods + hold_delivery_margin <= last_ctrl_at + effective_gap`.
                    let last_ctrl_at = drv.axis_last_ctrl_tx(j);
                    let energized_wd = drv.axis_is_energized(j) && drv.axis_has_watchdog(j);
                    if energized_wd {
                        let ticks_needed: u32 = if slot.target.require_post_write_readback
                            && matches!(self.phase, RunnerPhase::Forward)
                        {
                            2
                        } else {
                            1
                        };
                        let needed = budget
                            .period
                            .saturating_mul(ticks_needed)
                            .saturating_add(budget.hold_delivery_margin);
                        if needed > effective_gap {
                            return self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::WatchdogBudgetBreached,
                                false,
                            );
                        }
                        let Some(t0) = last_ctrl_at else {
                            return ModeProgress::Pending { wake: None };
                        };
                        if now.saturating_duration_since(t0).saturating_add(needed) > effective_gap {
                            // Wait for next tick's watchdog-first pass to refresh this axis first.
                            return ModeProgress::Pending { wake: None };
                        }
                    }
                    // Major 7: verify deadline arithmetic BEFORE issuing an irreversible REG_WRITE!
                    if now.checked_add(self.plan.per_axis_timeout).is_none() {
                        return self.handle_axis_failure(
                            drv,
                            slot.target.joint,
                            ModeSwitchFailure::InvalidPlan,
                            false,
                        );
                    }
                    if !drv.rx_drained() {
                        return ModeProgress::Pending { wake: None };
                    }
                    drv.clear_register_reply_log();
                    let goal = slot.active_goal;
                    match drv.try_send_ctrl_mode_write(j, goal, sub.deadline()) {
                        ModeTxResult::Sent {
                            boundary_seq,
                            tx_at,
                            tx_offset_ns,
                        } => {
                            sub.charge_tx(1);
                            let mut deadline = tx_at
                                .checked_add(self.plan.per_axis_timeout)
                                .unwrap_or(sub.deadline());
                            if energized_wd {
                                if let Some(t0) = last_ctrl_at {
                                    let allowed = effective_gap
                                        .saturating_sub(budget.hold_delivery_margin);
                                    if let Some(wd_dl) = t0.checked_add(allowed) {
                                        deadline = deadline.min(wd_dl);
                                    }
                                }
                            }
                            self.slots[idx].phase = AxisPhase::AwaitingWriteAck {
                                boundary_seq,
                                tx_at,
                                tx_offset_ns,
                                deadline,
                                last_ctrl_at,
                                written_mode: goal,
                            };
                            ModeProgress::Pending { wake: Some(deadline) }
                        }
                        ModeTxResult::SentFlushFailed { .. } => {
                            sub.charge_tx(1);
                            // Blocker 2: Frame was handed to transport before flush failed ->
                            // driver already marked `mode_write_uncertain_mask`; fail closed as Ambiguous!
                            self.ambiguous_mask |= 1u32 << j;
                            self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::TxFailed,
                                true,
                            )
                        }
                        ModeTxResult::Backpressure => {
                            sub.charge_tx(1);
                            ModeProgress::Pending { wake: None }
                        }
                        ModeTxResult::Revoked(code) => self.handle_revocation(drv, code),
                        ModeTxResult::Refused(_) => self.handle_axis_failure(
                            drv,
                            slot.target.joint,
                            ModeSwitchFailure::TxFailed,
                            false,
                        ),
                    }
                }
                AxisPhase::NeedReadback {
                    expected_mode,
                    from_write,
                    last_ctrl_at,
                    post_write_deadline,
                } => {
                    if !drv.rx_drained() {
                        return ModeProgress::Pending { wake: None };
                    }
                    if !from_write && now.checked_add(self.plan.per_axis_timeout).is_none() {
                        return self.handle_axis_failure(
                            drv,
                            slot.target.joint,
                            ModeSwitchFailure::InvalidPlan,
                            false,
                        );
                    }
                    // R2 Major 1: clamp actual readback TX deadline to `post_write_deadline`!
                    let tx_cutoff = post_write_deadline.map_or(sub.deadline(), |dl| sub.deadline().min(dl));
                    if now >= tx_cutoff {
                        if from_write {
                            drv.set_axis_confirmed_mode(j, None);
                            return self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::ReadbackTimeout,
                                true,
                            );
                        }
                        return ModeProgress::Pending { wake: None };
                    }
                    drv.clear_register_reply_log();
                    match drv.try_send_ctrl_mode_read(j, tx_cutoff) {
                        ModeTxResult::Sent {
                            boundary_seq,
                            tx_at,
                            tx_offset_ns,
                        } => {
                            sub.charge_tx(1);
                            let base_dl = tx_at
                                .checked_add(self.plan.per_axis_timeout)
                                .unwrap_or(sub.deadline());
                            // Blocker 6: clamp readback deadline to the single pre-computed
                            // `post_write_deadline` carried from `REG_WRITE` acceptance!
                            let mut deadline = post_write_deadline.map_or(base_dl, |dl| base_dl.min(dl));
                            if from_write && drv.axis_is_energized(j) && drv.axis_has_watchdog(j) {
                                if let Some(t0) = last_ctrl_at {
                                    let allowed = effective_gap
                                        .saturating_sub(budget.hold_delivery_margin);
                                    if let Some(wd_dl) = t0.checked_add(allowed) {
                                        deadline = deadline.min(wd_dl);
                                    }
                                }
                            }
                            self.slots[idx].phase = AxisPhase::AwaitingReadback {
                                boundary_seq,
                                tx_at,
                                tx_offset_ns,
                                deadline,
                                expected_mode,
                                from_write,
                                last_ctrl_at,
                            };
                            ModeProgress::Pending { wake: Some(deadline) }
                        }
                        ModeTxResult::Backpressure => {
                            sub.charge_tx(1);
                            ModeProgress::Pending { wake: None }
                        }
                        ModeTxResult::Revoked(code) => self.handle_revocation(drv, code),
                        ModeTxResult::SentFlushFailed { .. } => {
                            sub.charge_tx(1);
                            self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::TxFailed,
                                from_write,
                            )
                        }
                        ModeTxResult::Refused(_) => {
                            self.handle_axis_failure(
                                drv,
                                slot.target.joint,
                                ModeSwitchFailure::TxFailed,
                                from_write,
                            )
                        }
                    }
                }
                AxisPhase::AwaitingFeedback {
                    after_seq,
                    after_at,
                    confirm_offset_ns,
                    deadline,
                    confirmed_mode,
                    from_write,
                    fenced_pending_feedback,
                    refresh_sent,
                } => {
                    // For disarmed or PosForce joints that don't receive an arm watchdog hold,
                    // send one bounded read-only `0xCC` status refresh so feedback arrives causally.
                    if !refresh_sent
                        && (!drv.axis_is_energized(j) || confirmed_mode == ControlMode::PositionForce)
                    {
                        match drv.try_send_status_refresh(j, sub.deadline()) {
                            ModeTxResult::Sent { .. } => {
                                sub.charge_tx(1);
                                self.slots[idx].phase = AxisPhase::AwaitingFeedback {
                                    after_seq,
                                    after_at,
                                    confirm_offset_ns,
                                    deadline,
                                    confirmed_mode,
                                    from_write,
                                    fenced_pending_feedback,
                                    refresh_sent: true,
                                };
                            }
                            ModeTxResult::Backpressure | ModeTxResult::SentFlushFailed { .. } => {
                                sub.charge_tx(1);
                            }
                            ModeTxResult::Revoked(code) => return self.handle_revocation(drv, code),
                            _ => {}
                        }
                    }
                    ModeProgress::Pending { wake: Some(deadline) }
                }
                AxisPhase::RangeReadAwait { deadline, .. }
                | AxisPhase::AwaitingWriteAck { deadline, .. }
                | AxisPhase::AwaitingReadback { deadline, .. } => {
                    ModeProgress::Pending { wake: Some(deadline) }
                }
                AxisPhase::Pending | AxisPhase::Confirmed { .. } => {
                    ModeProgress::Pending { wake: None }
                }
            }
        }

        fn mark_slot_confirmed(
            &mut self,
            drv: &mut dyn ModeSwitchDriver,
            idx: usize,
            mode: ControlMode,
        ) -> Option<ModeProgress> {
            let j = self.slots[idx].target.joint;
            self.slots[idx].phase = AxisPhase::Confirmed { mode };
            self.confirmed_mask |= 1u32 << j;
            self.push_event(ModeSwitchEventKind::AxisConfirmed, j, Some(mode), None);
            self.advance_completed_slots(drv);
            let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
            if self.active_idx >= len {
                let term = match self.phase {
                    RunnerPhase::Recovering {
                        failed_joint,
                        reason,
                    } => ModeProgress::RolledBack {
                        failed_joint,
                        reason,
                    },
                    _ => ModeProgress::Confirmed,
                };
                // R3 Major 4: return candidate terminal to `step_driver` so final watchdog holds
                // are evaluated BEFORE `self.finish(...)` commits and stages the single terminal event.
                return Some(term);
            }
            None
        }

        fn handle_revocation(
            &mut self,
            drv: &mut dyn ModeSwitchDriver,
            code: &'static str,
        ) -> ModeProgress {
            // Blocker 1 (R5): the gate code says only that a stop is pending or applied; the
            // "applied" proof comes from the owner via `stop_applied()`, through the same
            // decision as external revocation.
            let reason = ModeSwitchFailure::from_revoke_code(code);
            let drv_energized = drv.any_axis_energized();
            let drv_uncertain = drv.mode_write_uncertain_mask();
            let stop_applied = drv.stop_applied();
            let outcome = self.revocation_outcome(drv_energized, drv_uncertain, reason, stop_applied);
            self.finish(drv, outcome)
        }

        fn handle_axis_failure(
            &mut self,
            drv: &mut dyn ModeSwitchDriver,
            failed_joint: u8,
            reason: ModeSwitchFailure,
            ambiguous: bool,
        ) -> ModeProgress {
            let drv_uncertain = drv.mode_write_uncertain_mask() & self.active_mask;
            if ambiguous || drv_uncertain != 0 {
                if ambiguous {
                    self.ambiguous_mask |= 1u32 << failed_joint;
                }
                self.ambiguous_mask |= drv_uncertain;
                return self.finish(
                    drv,
                    ModeProgress::Ambiguous {
                        failed_joint,
                        reason,
                    },
                );
            }

            // Clean failure with zero ambiguous axes: if we are in Forward with
            // `RollbackToPrevious` and at least one prior axis switched, enter `Recovering`.
            if matches!(self.phase, RunnerPhase::Forward)
                && self.plan.recovery == ModeSwitchRecovery::RollbackToPrevious
            {
                let len = usize::from(self.plan.len).min(MAX_MODE_SWITCH_AXES);
                let mut any_to_rollback = false;
                for i in 0..len {
                    if self.slots[i].switched_forward {
                        let Some(prev) = self.slots[i].previous_mode else {
                            self.ambiguous_mask |= 1u32 << self.slots[i].target.joint;
                            return self.finish(
                                drv,
                                ModeProgress::Ambiguous {
                                    failed_joint: self.slots[i].target.joint,
                                    reason,
                                },
                            );
                        };
                        let j = usize::from(self.slots[i].target.joint);
                        self.slots[i].active_goal = prev;
                        self.confirmed_mask &= !(1u32 << j);
                        // Unconditional wire-range check even on rollback into Impedance!
                        if prev == ControlMode::Impedance && !drv.axis_impedance_ranges_verified(j) {
                            self.slots[i].phase = AxisPhase::RangeReadNeed {
                                rid_idx: 0,
                                got: [None; 3],
                            };
                        } else {
                            self.slots[i].phase = AxisPhase::NeedWrite;
                        }
                        any_to_rollback = true;
                    } else {
                        let mode = self.slots[i]
                            .previous_mode
                            .unwrap_or(self.slots[i].target.target_mode);
                        self.slots[i].phase = AxisPhase::Confirmed { mode };
                    }
                }
                if any_to_rollback {
                    self.phase = RunnerPhase::Recovering {
                        failed_joint,
                        reason,
                    };
                    self.active_idx = 0;
                    self.advance_completed_slots(drv);
                    self.push_event(
                        ModeSwitchEventKind::RollbackStarted,
                        failed_joint,
                        None,
                        Some(reason),
                    );
                    return ModeProgress::Pending { wake: None };
                }
            }

            // R3 Blocker 1: NEVER release an energized partial switch back to ordinary output!
            // If `>= 1` axis already switched forward and we did not roll back (e.g. `FailClosed`
            // policy or a failure while already in `Recovering`), escalate to `Ambiguous`!
            if drv.any_axis_energized()
                && (matches!(self.phase, RunnerPhase::Recovering { .. })
                    || self.first_switched_forward_joint().is_some())
            {
                self.ambiguous_mask |= 1u32 << failed_joint;
                return self.finish(
                    drv,
                    ModeProgress::Ambiguous {
                        failed_joint,
                        reason,
                    },
                );
            }

            self.finish(
                drv,
                ModeProgress::FailedClean {
                    failed_joint,
                    reason,
                },
            )
        }

        fn finish(&mut self, drv: &mut dyn ModeSwitchDriver, progress: ModeProgress) -> ModeProgress {
            drv.set_mode_switch_active_mask(0);
            self.phase = RunnerPhase::Done(progress);
            match progress {
                ModeProgress::Confirmed => {
                    self.push_event(ModeSwitchEventKind::TerminalConfirmed, 0, None, None);
                }
                ModeProgress::RolledBack {
                    failed_joint,
                    reason,
                } => {
                    self.push_event(
                        ModeSwitchEventKind::TerminalRolledBack,
                        failed_joint,
                        None,
                        Some(reason),
                    );
                }
                ModeProgress::StoppedRecovered { reason } => {
                    self.push_event(
                        ModeSwitchEventKind::TerminalStoppedRecovered,
                        0,
                        None,
                        Some(reason),
                    );
                }
                ModeProgress::FailedClean {
                    failed_joint,
                    reason,
                } => {
                    self.push_event(
                        ModeSwitchEventKind::TerminalFailedClean,
                        failed_joint,
                        None,
                        Some(reason),
                    );
                }
                ModeProgress::Ambiguous {
                    failed_joint,
                    reason,
                } => {
                    self.push_event(
                        ModeSwitchEventKind::TerminalAmbiguous,
                        failed_joint,
                        None,
                        Some(reason),
                    );
                }
                ModeProgress::Pending { .. } => {}
            }
            progress
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub use switch::{
    ArmDriverModeAdapter, AxisTarget, ModeFeedbackMatch, ModeProgress, ModeRangeMatch,
    ModeRegMatch, ModeSwitch, ModeSwitchDriver, ModeSwitchEvent, ModeSwitchEventKind,
    ModeSwitchFailure, ModeSwitchIntent, ModeSwitchPlan, ModeSwitchRecovery, ModeSwitchSnapshot,
    ModeTxResult, ReleaseBudget, WatchdogHoldResult, DEFAULT_HOLD_DELIVERY_MARGIN,
    DEFAULT_MAX_WATCHDOG_GAP, DEFAULT_PER_AXIS_TIMEOUT, DEFAULT_WATCHDOG_PROACTIVE_INTERVAL,
    MAX_MODE_SWITCH_AXES, MAX_PER_AXIS_TIMEOUT, MAX_WATCHDOG_GAP_LIMIT, MIN_PER_AXIS_TIMEOUT,
};
