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
    SupervisorInner, BUS_LOSS_TIMEOUT, CONTROL_DEADLINE, GRIPPER_FORCE_CAP_TICKS, GRIPPER_STALL_RELEASE_NM,
    PREEMPT_COOLDOWN, THERMAL_CLEAR_MARGIN_C,
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
        Estop | WatchdogTimeout | ControlDeadline | JointLimit | EnableFailed | ServoNoBrake
        | HandoverUnsettled | GravityCompFailed => &["clear_fault", "torque_on"],
        BusLoss | InvalidTelemetry | DriverWriteFailed | CanTxStalled => &["restore_bus", "clear_fault", "torque_on"],
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
                    "park_then_torque_off": "driver supports automatic unload and the profile has a resting pose: park at the resting pose (checking all enabled workspace constraints), settle, torque off",
                    "hold_and_fault": "otherwise, or when the park cannot start (no plan, or its path/destination violates the workspace) or exceeds its budget: hold position with torque on and latch WATCHDOG_TIMEOUT",
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
        let gripper = c.gripper_index().zip(c.gripper_policy.as_ref());
        if let Some((joint, g)) = gripper.filter(|(_, g)| g.max_close_torque_nm.is_some()) {
            stops.push(json!({
                "id": "gripper_force_limit",
                "trigger": "|gripper torque| above max_close_torque_nm for persist_ticks consecutive ticks on fresh feedback while the jaw is driven closed, or the torque pushing open above it while the jaw is driven open, whatever its speed",
                "joint": joint,
                "max_close_torque_nm": g.max_close_torque_nm,
                "persist_ticks": GRIPPER_FORCE_CAP_TICKS,
                "calibrated": false,
                "applies_to": ["gripper_action", "grasp", "servo_stream", "joint_motion", "replay", "gravity_comp"],
                "lease_required": false,
                "effect": "hold_gripper_at_measured_position",
                "motion_limit_reason": "gripper_force",
                "observation": "gripper.force_limited, gripper.force_limit_trips",
                "while_held": "gripper targets that close further are held; arm joints keep moving",
                "release_below_nm": GRIPPER_STALL_RELEASE_NM,
                "stream_release": "under a servo stream or live policy: only a gripper target at least stream_release_margin_rad more open than the hold, the end of the stream or an action releases it; low torque does not",
                "stream_release_margin_rad": crate::supervisor::GRIPPER_FORCE_RELEASE_MARGIN_RAD,
                "stream_contact_regulator": g.contact_torque_nm.zip(g.stream_contact_speed_rad_s).map(|(nm, speed)| json!({
                    "applies_to": ["servo_stream", "live_policy"],
                    "trigger": "|gripper torque| at or above the contact threshold (learned free-motion baseline + margin, contact_torque_nm until learned; or at or above the stall threshold with the jaw still) on fresh feedback while the stream closes",
                    "target_torque_nm": nm,
                    "contact_speed_rad_s": speed,
                    "effect": "closing command advances at most contact_speed_rad_s past the last command while |torque| is below target_torque_nm; above it the lead over the jaw is scaled by target_torque_nm / |torque|",
                    "ends": "jaw opens stream_release_margin_rad past the contact point, closes contact_lost_rad on with |torque| below half the target, or the stream ends",
                    "contact_lost_rad": crate::supervisor::GRIPPER_STREAM_CONTACT_LOST_RAD,
                    "not_bounded": "the first torque sample at contact (stiffness x POS_VEL lag); bounded only by max_lead_rad",
                    "calibrated": false,
                })),
                "invalidates_commands": false,
                "clear": ["open_gripper", "torque_below_release"],
            }));
        }

        if capabilities.collision_guard {
            let p = self.collision_policy();
            stops.push(json!({
                "id": "collision_protective_stop",
                "trigger": "residual torque above threshold_nm for persist_ticks consecutive ticks on any of J1..J6",
                "enabled": self.collision.enabled,
                "sensitivity": self.collision.sensitivity,
                "source": if self.collision.runtime { "runtime" } else { "profile" },
                "calibrated": false,
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

        if capabilities.torque_guard {
            if let Some(p) = c.torque_guard {
                stops.push(json!({
                    "id": "torque_limit_hold",
                    "trigger": "|measured torque| at or above ratio x the joint's TMAX for persist_ms on a watched joint (J1..J6, gripper excluded), torqued in position control",
                    "ratio": p.ratio,
                    "persist_ms": p.persist_ms,
                    "thresholds_nm": self.torque_guard_thresholds(),
                    "not_watched": ["gravity_comp", "unload_settle", "gripper"],
                    "lease_required": false,
                    "effect": "cancel_motion_hold_torque_on",
                    "action_code": "TORQUE_LIMIT",
                    "motion_limit_reason": "torque",
                    "while_held": "servo and jog targets that move the held joint further in the push direction (sign of the torque at the trip) are clamped to the hold position; targets that keep it or move it the other way pass; any motion that commands the held joint more than push_in_rad past the trip position trips at once; a motion that moves the held joint away from the load (park, home, watchdog park, jog, Cartesian) is not tripped on that joint for up to retreat.window_s while its torque does not rise and the joint follows",
                    "release_rad": crate::torque_guard::TORQUE_HOLD_RELEASE_RAD,
                    "push_in_rad": crate::torque_guard::TORQUE_PUSH_IN_RAD,
                    "retreat": {
                        "window_s": crate::torque_guard::TORQUE_RETREAT_WINDOW_S,
                        "torque_margin_ratio": crate::torque_guard::TORQUE_RETREAT_MARGIN_RATIO,
                        "lag_rad": crate::torque_guard::TORQUE_RETREAT_LAG_RAD,
                    },
                    "backoff_rad": crate::torque_guard::TORQUE_HOLD_BACKOFF_RAD,
                    "invalidates_commands": false,
                    "clear": ["move_joint_away_from_push_direction"],
                }));
            }
        }

        if capabilities.power_budget {
            if let Some(b) = c.power_budget.as_ref().filter(|b| b.enabled && b.governor.enabled) {
                let g = b.governor;
                stops.push(json!({
                    "id": "power_limit_hold",
                    "trigger": "estimated supply current (from motor torque and velocity, not measured) at or above hard x supply_rated_a for persist_ms while a motion runs and a heavy joint is commanded in the direction of its torque; between soft and hard the motion is slowed instead",
                    "estimate_only": true,
                    "supply_rated_a": b.supply_rated_a,
                    "supply_trip_a": b.supply_trip_a,
                    "soft": g.soft,
                    "hard": g.hard,
                    "floor_scale": g.floor_scale,
                    "persist_ms": g.persist_ms,
                    "recover_ms": g.recover_ms,
                    "not_applied": ["gravity_comp"],
                    "lease_required": false,
                    "effect": "cancel_motion_hold_torque_on",
                    "action_code": "POWER_LIMIT",
                    "motion_limit_reason": "power",
                    "while_held": "servo and jog targets that move a joint that was pushing further in its push direction are clamped to the hold position; targets that keep it or move it the other way pass",
                    "invalidates_commands": false,
                    "clear": ["estimate_below_soft_for_recover_ms", "torque_off", "stop"],
                }));
            }
        }

        let cap = c.speed_cap;
        let kinematics_configured = c.kinematics.is_some();
        let workspace_configured = c.workspace.is_some();
        let workspace_policy = self.workspace_policy();
        let workspace_active = workspace_policy.is_some();
        let geometry_coverage = match workspace_policy {
            Some((ws, _)) if !ws.link_capsules.is_empty() => "end_and_links",
            Some(_) => "end_only",
            None => "none",
        };
        let camera_mount_covered = workspace_active && c.camera_mount.offset_m.is_some();
        let self_collision_covered = workspace_policy.is_some_and(|(ws, _)| ws.self_collision_covered());
        let workspace = workspace_policy.map(|(ws, cam)| {
            let sc_covered = ws.self_collision_covered();
            let active_pairs: &[[String; 2]] = if sc_covered { &ws.self_collision_pairs } else { &[] };
            json!({
                // the effective floor: max(profile floor, valid calibrated table height - 3 mm)
                "floor_z_m": ws.floor_z_m,
                "profile_floor_z_m": self.profile_floor_z_m,
                "effective_floor_z_m": ws.floor_z_m,
                "floor_source": self.floor_source,
                "floor_rule": "effective floor = max(profile floor_z_m, table_z_m - 0.003) while a camera calibration is in effect and not stale; with table_z_source hover only while its acceptance is valid",
                "box_min_m": ws.box_min_m,
                "box_max_m": ws.box_max_m,
                "max_reach_m": ws.max_reach_m,
                "check_points_end_m": ws.check_points_end_m,
                "kinematics_configured": kinematics_configured,
                // the geometry checks run on the live arm model (kinematic correction included)
                "kinematics_corrected": self.config.arm_model.correction().is_some(),
                "workspace_configured": workspace_configured,
                "geometry_coverage": geometry_coverage,
                "camera_mount_covered": cam.offset_m.is_some(),
                "self_collision_covered": sc_covered,
                "self_collision_pairs": active_pairs,
                "self_collision_min_clearance_m": ws.self_collision_min_clearance_m,
            })
        });
        let keepout = self.keepout_state();
        let hw = c.hardware.clone().unwrap_or_default();
        let notes = hw.notes.clone().unwrap_or_else(|| {
            let mut n = Vec::new();
            if !hw.physical_estop { n.push("software stops only; no wired emergency stop".to_string()); }
            n.push("motor behaviour when CAN is physically lost depends on the motor's own timeout (unverified)".into());
            n.push("thermal trip and a completed watchdog park turn torque off; every other stop holds position with torque on".into());
            n
        });
        // gripper motor current cap: what the driver applies, else what the profile configures
        let driver_cap = self.driver.as_ref().and_then(|d| d.lock().ok().and_then(|drv| drv.motor_current_cap()));
        let gripper_current_cap = match (driver_cap, &c.damiao_gripper) {
            (Some(cap), _) => serde_json::to_value(cap).unwrap_or(Value::Null),
            (None, Some(g)) => json!({
                "joint": c.gripper_index(),
                "mode": g.mode,
                "current_raw": (g.mode == crate::model::DamiaoGripperMode::PosForce).then_some(g.current_ceiling_raw),
                "active": false,
                "calibrated": g.calibrated,
                "ceiling_torque_nm": g.ceiling_torque_nm,
                "torque_reference": g.torque_reference,
                "close_speed_rad_s": g.close_speed_rad_s,
                "note": if g.mode == crate::model::DamiaoGripperMode::PosVel { "no motor current cap (legacy POS_VEL gripper)" }
                    else { "configured; not applied by the attached driver" },
            }),
            (None, None) => Value::Null,
        };
        let hardware = json!({
            "physical_estop": hw.physical_estop,
            "torque_off_on_fault": false,
            "operator_auth_enforced": self.operator_auth_enforced,
            "gripper_current_cap": gripper_current_cap,
            "notes": notes,
        });

        let measure_multiview = json!({
                    "available": capabilities.measure_multiview,
                    "lease_required": true,
                    "operator_required": self.operator_auth_enforced,
                    "speed_scale": crate::measure_multiview::SPEED_SCALE,
                    "baseline_m": crate::measure_multiview::BASELINE_M,
                    "views": [crate::measure_multiview::MIN_VIEWS, crate::measure_multiview::MAX_VIEWS],
                    "motions": ["orbit", "translate"],
                    "default_motion": "orbit",
                    "axes": {"x": crate::measure_multiview::Axes::X.views().0, "xy": crate::measure_multiview::Axes::Xy.views().0},
                    "min_frame_margin_px": crate::measure_multiview::MIN_FRAME_MARGIN_PX,
                    "min_settle_s": crate::measure_multiview::MIN_SETTLE_S,
                    "settle_min_s": crate::measure_multiview::SETTLE_MIN_S,
                    // the longest settle wait a request can get (at the largest settle_min_s)
                    "max_settle_s": crate::measure_multiview::max_settle_s(crate::measure_multiview::SETTLE_MIN_S[1]),
                    "still": {"window": crate::measure_multiview::STILL_WINDOW, "median_vel_rad_s": crate::measure_multiview::STILL_VEL, "range_rad": crate::measure_multiview::STILL_RANGE_RAD},
                    "snapshot_bracket_rad": crate::measure_multiview::BRACKET_RAD,
                    "snapshot_tries": crate::measure_multiview::SNAPSHOT_TRIES,
                    "debug_dir": crate::measure_multiview::debug_dir().to_string_lossy(),
                    "motion": "joint-space quintic to each view pose, with the checks of every joint move (swept workspace, keep-out boxes, link capsules, power budget, torque and collision holds); a refused view (or one whose target would leave the image, OUT_OF_FRAME) is retried at the mirrored or a smaller offset, else the action fails with the refusal code",
                    "ends_on": ["stop", "e_stop", "fault", "collision", "torque_limit", "lease_loss", "watchdog"],
                    "return_to_start": "default true: joint-space move back to the start joints after the captures or a measurement failure; not after a stop, fault, collision, torque limit or lease loss",
                    "return_after": crate::measure_multiview::RETURN_AFTER,
                });
        let active_tool = self.schema().active_tool;
        json!({
            "describe_version": DESCRIBE_VERSION,
            "device": {
                "armd_version": env!("CARGO_PKG_VERSION"),
                "follower": self.follower,
                "profile_version": c.profile_version,
                "dof": c.dof,
                "joint_names": c.joint_names,
                "active_tool": active_tool,
                "dynamics_qualified": c.dynamics_qualified,
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
                "kinematics_configured": kinematics_configured,
                "workspace_configured": workspace_configured,
                "geometry_coverage": geometry_coverage,
                "camera_mount_covered": camera_mount_covered,
                "self_collision_covered": self_collision_covered,
                "keepout_boxes": keepout.boxes,
                "keepout_source": keepout.source,
                "keepout_rule": "boxes in force = profile boxes + runtime boxes (each tagged with its source); the runtime list cannot remove a profile box",
                "speed_cap": cap,
                "gripper": gripper.map(|(joint, g)| json!({
                    "joint": joint,
                    "max_close_torque_nm": g.max_close_torque_nm,
                    "contact_torque_nm": g.contact_torque_nm,
                    "max_lead_rad": g.max_lead_rad,
                    "squeeze_rad": g.squeeze_rad,
                    "contact_speed_rad_s": crate::motion::GRIPPER_CONTACT_SPEED,
                    "stream_contact_speed_rad_s": g.stream_contact_speed_rad_s,
                    // closing speed of grasp / place / sequence closes and calibration strokes
                    // (min of gripper_policy.close_speed_rad_s, joint max_velocity, and
                    // damiao_gripper.close_speed_rad_s when pos_force, wire-quantized in pos_force)
                    "close_speed_rad_s": self.effective_planned_close_speed(),
                    // grasp contact relative to the learned free-motion torque (crate::gripper_baseline)
                    "contact_baseline": {
                        "rule": "shared net-progress contact detector (actions, macros, streams; no instantaneous-velocity shortcut): once closing net progress stops over the window (<= 0.50 s, 0.002 rad watermark), contact triggers at baseline + max(margin_floor_nm, margin_spreads x spread) when calibrated or contact_torque_nm when uncalibrated for 2 ticks; while moving, contact requires |torque| >= max(threshold, contact_torque_nm, 0.82 N·m, 0.9 x ceiling_estimate) for 3 ticks; lag contact requires following error >= 0.30 rad with no net progress; the torque cap does not depend on it",
                        "margin_floor_nm": crate::gripper_baseline::MARGIN_FLOOR_NM,
                        "margin_spreads": crate::gripper_baseline::MARGIN_SPREADS,
                        "min_contact_nm": crate::gripper_baseline::MIN_CONTACT_NM,
                        "cap_fraction": crate::gripper_baseline::CONTACT_CAP_FRACTION,
                        "learned_from": "opening moves, and calibration strokes (gripper action with baseline calibrate/reset, nothing in the jaws, grasp false), that end at the target without a contact, stall or torque cap trip",
                        "stall_raise_max_nm": crate::supervisor::GRIPPER_STALL_BASELINE_RAISE_MAX_NM,
                        "stream_target_raise_max_nm": crate::supervisor::GRIPPER_STREAM_TARGET_RAISE_MAX_NM,
                        "observation": "gripper.baseline",
                        "calibrated": false,
                    },
                })),
                "action_speed_max": {
                    "linear_mps": cap.and_then(|s| s.linear_mps).map_or(MAX_LINEAR_MPS, |v| MAX_LINEAR_MPS.min(f64::from(v))),
                    "angular_rps": cap.and_then(|s| s.angular_rps).map_or(MAX_ANGULAR_RPS, |v| MAX_ANGULAR_RPS.min(f64::from(v))),
                },
                // action measure_multiview (src/measure_multiview.rs)
                "measure_multiview": measure_multiview,
            },
            "stops": stops,
            "faults": faults(),
            "hardware": hardware,
            "not_a_safety_function": true,
        })
    }
}
