//! Motion commands of the WS command path: direct commands, preset translation, gravity comp,
//! Cartesian plans and joint goals (family M).
//!
//! Renamed from `api/ws/motion_cmds.rs` and extended with preset handling from `api/ws.rs`
//! without behaviour changes (A1b PR 1).

#[allow(unused_imports)]
use super::*;
use super::dispatch::{
    AdmissionClass, AuthRequirement, DispatchEntry, DispatchHandler, DispatchKey, FrameKind,
    LeaseRequirement, OwnerDispatchCtx, ReplyPolicy,
};

pub(super) const ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::Preset),
        handler: DispatchHandler::Owner(dispatch_preset),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::PresetReply,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/jog"),
        handler: DispatchHandler::Owner(dispatch_motion_command),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/home"),
        handler: DispatchHandler::Owner(dispatch_motion_command),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/unload"),
        handler: DispatchHandler::Owner(dispatch_motion_command),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::CommandImmediate,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/mode/gravity_comp"),
        handler: DispatchHandler::Owner(dispatch_motion_command),
        admission: AdmissionClass::Ordinary,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::Operator(None),
        reply: ReplyPolicy::CommandImmediate,
    },
];

fn dispatch_preset(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    Some(handle_preset(ctx.inner, ctx.stamp, ctx.value, ctx.epoch))
}

fn dispatch_motion_command(ctx: &mut OwnerDispatchCtx<'_>) -> Option<Value> {
    let result = execute_command(ctx.inner, ctx.stamp, ctx.path, &ctx.value["body"], ctx.epoch);
    Some(match result {
        Ok(started) => command_ack(ctx.inner, &ctx.value["request_id"], started),
        Err(error) => command_reject(ctx.inner, &ctx.value["request_id"], error),
    })
}

// The caller holds the same write lock used to verify lease and epoch.
pub(super) fn execute_command(inner: &mut SupervisorInner, stamp: crate::supervisor::AdmissionStamp, path: &str, body: &Value, epoch: u64) -> Result<Started, ApiError> {
    validate_command_body_common(body).map_err(invalid_command)?;
    match path {
        "/api/v1/motion/jog" => {
            let speed_scale = crate::motion::num(body, "speed_scale").map_err(ArmError::from)?;
            let target_positions = positions(body.get("target_positions"))?;
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (_, id, duration) = SupervisorInner::jog_action_with_speed_scale(
                admitted,
                target_positions,
                body.get("duration_secs").and_then(Value::as_f64),
                Some(epoch),
                "joint_move",
                speed_scale,
            )?;
            return Ok(Some((id, duration)));
        }
        "/api/v1/motion/home" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_commissioning_for_ordinary_motion("home")?;
            let home = inner.config.home_positions.clone();
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (_, id, duration) = SupervisorInner::jog_action(admitted, home, body.get("duration_secs").and_then(Value::as_f64), Some(epoch), "home")?;
            return Ok(Some((id, duration)));
        }
        "/api/v1/motion/unload" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.gravity_comp_motion_busy().map_err(ArmError::from)?;
            inner.collision_hold_check().map_err(ArmError::from)?;
            if inner.handover_settling_since.is_some() {
                return Err(ArmError::Safety(SafetyError::HandoverSettling).into());
            }
            if !inner.supports_automatic_unload() {
                return Err(ArmError::Unsupported("Automatic unload requires a resting-pose profile and velocity, torque and enable-state feedback; no motion was started".into()).into());
            }
            if !inner.safety.state().is_operational() && !inner.holding_after_clear() {
                return Err(ArmError::Safety(SafetyError::NotArmed(inner.safety.state().clone())).into());
            }
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let id = SupervisorInner::unload_action(admitted)?;
            let duration = inner.active_trajectory.as_ref().map_or(0.0, |t| t.duration()) + 0.8;
            return Ok(id.map(|id| (id, duration)));
        }
        "/api/v1/mode/gravity_comp" => {
            let enable = body.get("enable").and_then(Value::as_bool)
                .ok_or_else(|| invalid_command("enable must be boolean"))?;
            if enable {
                let req = gravity_comp_request(body)?;
                let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
                SupervisorInner::enter_gravity_comp(admitted, req)?;
            } else {
                inner.exit_gravity_comp("REQUESTED", "Exit requested");
            }
        }
        _ => return Err(invalid_command("Unknown actuator command")),
    }
    Ok(None)
}

/// `scale` (number) and `damping_kd` (number for all six arm joints, or six numbers).
pub(super) fn gravity_comp_request(body: &Value) -> Result<crate::gravity_comp::GravityCompRequest, ApiError> {
    let scale = match body.get("scale") {
        None | Some(Value::Null) => None,
        Some(v) => Some(v.as_f64().filter(|x| x.is_finite()).ok_or_else(|| invalid_command("scale must be a number"))? as f32),
    };
    let damping_kd = match body.get("damping_kd") {
        None | Some(Value::Null) => None,
        Some(Value::Number(n)) => Some(vec![n.as_f64().unwrap_or(f64::NAN) as f32; 6]),
        Some(Value::Array(a)) => Some(a.iter().map(|v| v.as_f64().map_or(f32::NAN, |x| x as f32)).collect()),
        Some(_) => return Err(invalid_command("damping_kd must be a number or an array of 6 numbers")),
    };
    Ok(crate::gravity_comp::GravityCompRequest { scale, damping_kd })
}

/// Handle a WS `preset` frame under the supervisor write lock.
pub(super) fn handle_preset(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    value: &Value,
    epoch: u64,
) -> Value {
    if inner.calibration_busy {
        return json!({"type":"preset_reject", "name":value.get("name"), "status":409, "code":"MOTION_BUSY", "error":"MOTION_BUSY",
            "message":"a camera calibration apply or restore is running", "epoch_id":epoch});
    }
    let name = value.get("name").and_then(Value::as_str).unwrap_or("");
    let (path, body) = match name {
        "home" => ("/api/v1/motion/home", json!({})),
        "park" | "unload" => ("/api/v1/motion/unload", json!({})),
        _ => {
            let mut j = json!({"target_positions":value.get("positions")});
            if let Some(s) = value.get("speed_scale") {
                j["speed_scale"] = s.clone();
            }
            ("/api/v1/motion/jog", j)
        }
    };
    match execute_command(inner, stamp, path, &body, epoch) {
        Ok(started) => {
            let mut ack = json!({"type":"preset_ack", "name":name, "epoch_id":inner.safety.epoch_id()});
            if let Some((id, _)) = started {
                ack["action_id"] = json!(id);
            }
            ack
        }
        Err(error) => {
            let mut reply = rejection(error, "preset_reject", inner.safety.epoch_id());
            reply["name"] = json!(name);
            reply
        }
    }
}
