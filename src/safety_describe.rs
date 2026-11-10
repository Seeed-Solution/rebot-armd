//! `GET /api/v1/safety/describe` (capability `safety_describe`): the limits, stops and faults of
//! this armd, generated from the running config (runtime keep-out and collision settings
//! included) and from the constants and enums the control code uses. No value here is a
//! hand-written copy of a number used elsewhere.
//!
//! Strings are fixed English identifiers or sentences; clients translate them.

use serde::Serialize;
use serde_json::{json, Value};

use crate::motion::{MAX_ANGULAR_RPS, MAX_LINEAR_MPS};
use crate::safety::FaultCode;
use crate::supervisor::{
    SupervisorInner, BUS_LOSS_TIMEOUT, CONTROL_DEADLINE, PREEMPT_COOLDOWN, THERMAL_CLEAR_MARGIN_C,
    UNLOAD_SETTLE_HOLD, WATCHDOG_PARK_MARGIN, WATCHDOG_PARK_SETTLE_GRACE, WATCHDOG_PARK_TIMEOUT,
};

/// Version of the `safety/describe` body.
pub const DESCRIBE_VERSION: u32 = 1;
/// Heartbeat interval recommended to lease holders (ms), the cockpit's interval. The
/// published value never exceeds a quarter of the lease timeout.
pub const HEARTBEAT_RECOMMENDED_MS: u64 = 150;

/// Recovery steps (identifiers):
/// - `clear_fault`: WS command `/api/v1/safety/clear_fault` (lease holder).
/// - `torque_on`: WS command `/api/v1/torque` `{"enabled": true}` (re-arm).
/// - `wait_cooldown`: wait until every joint is at or below `stops[thermal].clear_below_c`.
/// - `restore_bus`: restore CAN wiring / motor power until every joint reports fresh feedback.
/// - `remove_obstruction`: free the arm from whatever stopped it.
/// - `inspect_message`: read `observation.fault.message` for the cause.
/// - `clear_fault_again`: the first clear_fault sends a hardware reset; repeat after fresh feedback.
pub const RECOVERY_STEPS: &[&str] = &[
    "clear_fault", "torque_on", "wait_cooldown", "restore_bus", "remove_obstruction", "inspect_message", "clear_fault_again",
];

/// One `faults[]` entry.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct FaultDescription {
    pub code: FaultCode,
    /// [`FaultCode::recoverable`]: hint for client recovery policies.
    pub recoverable: bool,
    pub recovery: &'static [&'static str],
    /// armd never clears a fault by itself.
    pub auto_clear: bool,
}

/// Recovery steps for `code`. The match is exhaustive: a new [`FaultCode`] variant does not
/// compile until it is described here.
pub fn fault_recovery(code: FaultCode) -> &'static [&'static str] {
    use FaultCode::*;
    match code {
        Estop | WatchdogTimeout | ControlDeadline | JointLimit | EnableFailed | ServoNoBrake | HandoverUnsettled
        | GravityCompFailed => &["clear_fault", "torque_on"],
        BusLoss | InvalidTelemetry | DriverWriteFailed => &["restore_bus", "clear_fault", "torque_on"],
        ThermalTrip => &["wait_cooldown", "clear_fault", "torque_on"],
        FollowingError | Collision => &["remove_obstruction", "clear_fault", "torque_on"],
        HardwareFault => &["clear_fault", "clear_fault_again", "torque_on"],
        Other => &["inspect_message", "clear_fault", "torque_on"],
    }
}

/// `faults[]`: one entry per [`FaultCode`] variant, in declaration order.
pub fn faults() -> Vec<FaultDescription> {
    FaultCode::ALL.iter().map(|&code| FaultDescription {
        code,
        recoverable: code.recoverable(),
        recovery: fault_recovery(code),
        auto_clear: false,
    }).collect()
}

fn ms(d: std::time::Duration) -> u64 { d.as_millis() as u64 }

impl SupervisorInner {
    /// Watchdog outcome for this config: `park_then_torque_off` when the driver supports the
    /// automatic unload and the profile has a resting pose, else `hold_and_fault`.
    pub fn watchdog_effect(&self) -> &'static str {
        if self.config.resting_pose.is_some() && self.supports_automatic_unload() { "park_then_torque_off" } else { "hold_and_fault" }
    }

    /// Body of `GET /api/v1/safety/describe`.
    pub fn safety_describe(&self) -> Value {
        let c = &self.config;
        let capabilities = self.schema().capabilities;
        let recovery = fault_recovery;
        let temperature_feedback = self.driver.as_ref()
            .is_none_or(|d| d.lock().is_ok_and(|drv| drv.capabilities().temperature_feedback));
        let watchdog_effect = self.watchdog_effect();

        let mut stops = vec![
            json!({
                "id": "e_stop",
                "trigger": {"ws": {"type": "estop"}, "rest": "POST /api/v1/safety/e_stop"},
                "lease_required": false,
                "effect": "hold_position_torque_on",
                "invalidates_commands": true,
                "fault_code": FaultCode::Estop,
                "clear": recovery(FaultCode::Estop),
            }),
            json!({
                "id": "watchdog",
                "trigger": "no heartbeat from the lease holder for lease_timeout_ms while torqued",
                "timeout_ms": c.lease_timeout_ms,
                "lease_required": false,
                "effect": watchdog_effect,
                "outcomes": {
                    "park_then_torque_off": "driver supports automatic unload and the profile has a resting pose: park at the resting pose, settle, torque off",
                    "hold_and_fault": "otherwise, or when the park cannot start or exceeds its budget: hold position with torque on and latch WATCHDOG_TIMEOUT",
                },
                "park_budget": {
                    "min_ms": ms(WATCHDOG_PARK_TIMEOUT),
                    "margin_ms": ms(WATCHDOG_PARK_MARGIN),
                    "settle_hold_ms": ms(UNLOAD_SETTLE_HOLD),
                    "settle_grace_ms": ms(WATCHDOG_PARK_SETTLE_GRACE),
                },
                "invalidates_commands": true,
                "fault_code": FaultCode::WatchdogTimeout,
                "clear": recovery(FaultCode::WatchdogTimeout),
            }),
            json!({
                "id": "bus_loss",
                "trigger": "no fresh feedback from every joint for bus_loss_timeout_ms while torqued",
                "timeout_ms": ms(BUS_LOSS_TIMEOUT),
                "lease_required": false,
                "effect": "hold_position_torque_on",
                "invalidates_commands": true,
                "fault_code": FaultCode::BusLoss,
                "clear": recovery(FaultCode::BusLoss),
            }),
            json!({
                "id": "control_deadline",
                "trigger": "control tick gap above tick_deadline_ms while torqued and armed",
                "deadline_ms": ms(CONTROL_DEADLINE),
                "lease_required": false,
                "effect": "hold_position_torque_on",
                "invalidates_commands": true,
                "fault_code": FaultCode::ControlDeadline,
                "clear": recovery(FaultCode::ControlDeadline),
            }),
            json!({
                "id": "following_error",
                "trigger": "joint tracking error above max_error_rad for persistence_ms (7-DOF gripper excluded)",
                "max_error_rad": c.following_error.max_error_rad,
                "persistence_ms": c.following_error.persistence_ms,
                "lease_required": false,
                "effect": "hold_position_torque_on",
                "servo_stream_effect": "servo_blocked_hold",
                "invalidates_commands": true,
                "fault_code": FaultCode::FollowingError,
                "clear": recovery(FaultCode::FollowingError),
            }),
            json!({
                "id": "thermal",
                "trigger": "joint temperature above max_temperature_c",
                "available": temperature_feedback,
                "max_temperature_c": c.max_temperature,
                "clear_below_c": c.max_temperature - THERMAL_CLEAR_MARGIN_C,
                "lease_required": false,
                "effect": "stop_then_torque_off",
                "invalidates_commands": true,
                "fault_code": FaultCode::ThermalTrip,
                "clear": recovery(FaultCode::ThermalTrip),
            }),
        ];
        if capabilities.collision_guard {
            let p = self.collision_policy();
            stops.push(json!({
                "id": "collision_protective_stop",
                "trigger": "residual torque above threshold_nm for persist_ticks consecutive ticks on any of J1..J6",
                "enabled": self.collision.enabled,
                "sensitivity": self.collision.sensitivity,
                "source": if self.collision.runtime { "runtime" } else { "profile" },
                "thresholds_nm": self.collision_thresholds(),
                "persist_ticks": p.persist_ticks,
                "lease_required": false,
                "effect": "cancel_motion_hold_torque_on",
                "rejects_motion_with": "COLLISION_HOLD",
                "invalidates_commands": false,
                "auto_resume_s": p.resume_after_s,
                "clear_ratio": p.clear_ratio,
                "escalation": {
                    "max_auto_resumes": p.max_auto_resumes,
                    "window_s": p.escalate_window_s,
                    "fault_code": FaultCode::Collision,
                },
                "clear": ["auto_resume"],
            }));
        }

        let cap = c.speed_cap;
        let workspace = self.workspace_policy().map(|(ws, _)| json!({
            "floor_z_m": ws.floor_z_m,
            "box_min_m": ws.box_min_m,
            "box_max_m": ws.box_max_m,
            "max_reach_m": ws.max_reach_m,
            "check_points_end_m": ws.check_points_end_m,
        }));
        let keepout = self.keepout_state();

        json!({
            "describe_version": DESCRIBE_VERSION,
            "device": {
                "armd_version": env!("CARGO_PKG_VERSION"),
                "follower": self.follower,
                "profile_version": c.profile_version,
                "dof": c.dof,
                "joint_names": c.joint_names,
            },
            "timing": {
                "control_hz": c.control_frequency_hz,
                "lease_timeout_ms": c.lease_timeout_ms,
                "heartbeat_recommended_ms": HEARTBEAT_RECOMMENDED_MS.min(c.lease_timeout_ms / 4),
                "bus_loss_timeout_ms": ms(BUS_LOSS_TIMEOUT),
                "tick_deadline_ms": ms(CONTROL_DEADLINE),
                "preemption_cooldown_ms": ms(PREEMPT_COOLDOWN),
                "following_error": {
                    "max_error_rad": c.following_error.max_error_rad,
                    "persistence_ms": c.following_error.persistence_ms,
                },
            },
            "limits": {
                "joints": c.limits.iter().map(|l| json!({
                    "name": l.name, "min": l.min_position, "max": l.max_position,
                    "max_velocity": l.max_velocity, "max_acceleration": l.max_acceleration,
                })).collect::<Vec<_>>(),
                "planned_max_velocity": c.planned_max_velocity,
                "servo_max_velocity": c.servo_max_velocity,
                "workspace": workspace,
                "keepout_boxes": keepout.boxes,
                "keepout_source": keepout.source,
                "speed_cap": cap,
                "action_speed_max": {
                    "linear_mps": cap.and_then(|s| s.linear_mps).map_or(MAX_LINEAR_MPS, |v| MAX_LINEAR_MPS.min(f64::from(v))),
                    "angular_rps": cap.and_then(|s| s.angular_rps).map_or(MAX_ANGULAR_RPS, |v| MAX_ANGULAR_RPS.min(f64::from(v))),
                },
            },
            "stops": stops,
            "faults": faults(),
            "hardware": {
                "physical_estop": false,
                "torque_off_on_fault": false,
                "notes": [
                    "software stops only; no wired emergency stop",
                    "motor behaviour when CAN is physically lost depends on the motor's own timeout (unverified)",
                    "thermal trip and a completed watchdog park turn torque off; every other stop holds position with torque on",
                ],
            },
            "not_a_safety_function": true,
        })
    }
}
