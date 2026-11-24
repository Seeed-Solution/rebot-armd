//! Motion commands of the WS command path: direct commands, preset translation, gravity comp,
//! Cartesian plans and joint goals (family M).
//!
//! Renamed from `api/ws/motion_cmds.rs` and extended with preset handling from `api/ws.rs`
//! without behaviour changes (A1b PR 1).

#[allow(unused_imports)]
use super::*;

// The caller holds the same write lock used to verify lease and epoch.
pub(super) fn execute_command(inner: &mut SupervisorInner, stamp: crate::supervisor::AdmissionStamp, path: &str, body: &Value, epoch: u64) -> Result<Started, ApiError> {
    validate_command_body_common(body).map_err(invalid_command)?;
    match path {
        "/api/v1/torque" | "/api/v1/arm" => {
            return lease::execute_torque_command(inner, stamp, body);
        }
        "/api/v1/safety/clear_fault" => {
            return lease::execute_clear_fault(inner, body, epoch);
        }
        "/api/v1/motion/servo" => {
            let targets = positions(body.get("targets"))?;
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            SupervisorInner::servo(admitted, targets, epoch).map_err(ArmError::from)?;
        }
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

/// A Cartesian action waiting for its plan (computed outside the lock).
pub(super) struct PendingPlan {
    pub(super) request_id: Value,
    pub(super) kind: &'static str,
    pub(super) input: PlanInput,
    pub(super) goal: crate::kinematics::Pose,
    pub(super) speed: crate::motion::Speed,
    pub(super) tag: Option<String>,
    pub(super) opts: crate::motion::PathOpts,
    /// Same-side approach (`precise: true` / `approach`, src/backlash.rs): settings or the
    /// reason for a direct motion (None = off).
    pub(super) approach: Result<crate::backlash::Plan, Option<&'static str>>,
    /// `path: "joint"`: the IK seeds ([`motion::goal_seeds`]), the `speed_scale` of the joint
    /// quintic and the same-side approach request (None = off). None: the straight line.
    pub(super) joint: Option<JointGoal>,
    /// IK seeds of the goal ([`motion::goal_seeds`]): the joint-space solve of `path: "joint"`,
    /// and for a straight line the check behind the `path: "joint"` hint of `IK_BRANCH_FLIP`.
    pub(super) seeds: Vec<[f64; 6]>,
}

/// `path: "joint"` part of a [`PendingPlan`].
pub(super) struct JointGoal {
    pub(super) speed_scale: f64,
    pub(super) same_side: Option<crate::backlash::Request>,
    /// Supervisor state the joint quintic is planned from without the lock.
    pub(super) snapshot: crate::supervisor::JointGoalSnapshot,
}

/// A macro action (crate::macro_action) checked under the lock, planned without it.
pub(super) enum PendingMacro { Grasp(crate::grasp::PendingGrasp), Place(crate::place::PendingPlace) }

#[allow(clippy::large_enum_variant)]
pub(super) enum PlannedMacro { Grasp(crate::grasp::PlannedGrasp), Place(crate::place::PlannedPlace) }

impl PendingMacro {
    pub(super) fn plan(self) -> Result<PlannedMacro, ArmError> {
        match self {
            Self::Grasp(g) => crate::grasp::plan(g).map(PlannedMacro::Grasp),
            Self::Place(p) => crate::place::plan(p).map(PlannedMacro::Place),
        }
    }
}

impl PlannedMacro {
    /// Begin the action and spawn its orchestrator. Returns (action id, expected duration).
    pub(super) fn start(self, admitted: crate::supervisor::Admitted<'_>, supervisor: &Arc<ArmSupervisor>, deps: &crate::grasp::GraspDeps) -> Result<(u64, f64), ArmError> {
        let stamp = admitted.stamp();
        match self {
            Self::Grasp(planned) => crate::grasp::begin(admitted, supervisor, deps.source.clone(), planned),
            Self::Place(planned) => {
                let (inner, proof) = admitted.consume()?;
                let id = inner.begin_macro_inner(&proof, "place", &planned.input, planned.request.tag.clone())?;
                if planned.orientation_levelled {
                    inner.update_action(id, |a| {
                        a.orientation = Some("levelled".into());
                    });
                }
                let duration = crate::place::expected_duration(&planned) + inner.approach_estimate_s("place_final", planned.request.approach);
                crate::place::spawn(supervisor.clone(), id, planned, stamp);
                Ok((id, duration))
            }
        }
    }
}

/// Aborts a tracked child task handle when dropped.
pub(super) struct AbortOnDrop(pub(super) tokio::task::AbortHandle);

impl Drop for AbortOnDrop {
    fn drop(&mut self) {
        self.0.abort();
    }
}

/// A macro action being planned and checked (grasp preflight: model switch, up to ~7 s) in a
/// tracked connection task, aborted automatically on drop so preflight never outlives the socket.
pub(super) struct MacroCheck {
    pub(super) request_id: Value,
    pub(super) generation: u64,
    pub(super) stamp: crate::supervisor::AdmissionStamp,
    pub(super) abort: AbortOnDrop,
    pub(super) rx: tokio::sync::oneshot::Receiver<Result<PlannedMacro, ArmError>>,
}
