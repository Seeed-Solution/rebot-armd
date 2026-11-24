//! Motion commands of the WS command path: direct commands, Cartesian plans and joint goals, macro planning and checks (tracks C and E).
//!
//! Moved from `api/ws.rs` without changes (armd 1.6 RT-core foundation); moved items, their
//! fields and inherent methods are `pub(super)` so the connection loop keeps using them.

#[allow(unused_imports)]
use super::*;

// The caller holds the same write lock used to verify lease and epoch.
pub(super) fn execute_command(inner: &mut SupervisorInner, stamp: crate::supervisor::AdmissionStamp, path: &str, body: &Value, epoch: u64) -> Result<Started, ApiError> {
    validate_command_body_common(body).map_err(invalid_command)?;
    match path {
        "/api/v1/torque" | "/api/v1/arm" => {
            let enabled = body.get("enable").and_then(Value::as_bool)
                .ok_or_else(|| invalid_command("enable must be boolean"))?;
            if enabled {
                let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
                inner.enable_torque(admitted).map_err(ArmError::from)?;
            } else {
                inner.disable_torque().map_err(ArmError::from)?;
            }
        }
        "/api/v1/safety/clear_fault" => {
            // {"mit_recovery": "torque_off_all"}: the operator accepts the MIT expiry recovery
            // (torque off on every joint); any other value is refused
            let mit_recovery = match body.get("mit_recovery") {
                None | Some(Value::Null) => false,
                Some(v) if v.as_str() == Some(crate::mit::RECOVERY_CONFIRM_VALUE) => true,
                Some(_) => return Err(invalid_command(&format!("mit_recovery must be \"{}\"", crate::mit::RECOVERY_CONFIRM_VALUE))),
            };
            inner.clear_fault_confirmed_for_epoch(mit_recovery, Some(epoch))?;
        }
        "/api/v1/motion/servo" => {
            let targets = positions(body.get("targets"))?;
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            inner.servo(admitted, targets, epoch).map_err(ArmError::from)?;
        }
        "/api/v1/motion/jog" => {
            let speed_scale = motion::num(body, "speed_scale").map_err(ArmError::from)?;
            let target_positions = positions(body.get("target_positions"))?;
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (_, id, duration) = inner.jog_action_with_speed_scale(
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
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            inner.check_commissioning_for_ordinary_motion("home")?;
            let home = inner.config.home_positions.clone();
            let (_, id, duration) = inner.jog_action(admitted, home, body.get("duration_secs").and_then(Value::as_f64), Some(epoch), "home")?;
            return Ok(Some((id, duration)));
        }
        "/api/v1/motion/unload" => {
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
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
            let id = inner.unload_action(admitted)?;
            let duration = inner.active_trajectory.as_ref().map_or(0.0, |t| t.duration()) + 0.8;
            return Ok(id.map(|id| (id, duration)));
        }
        "/api/v1/mode/gravity_comp" => {
            let enable = body.get("enable").and_then(Value::as_bool)
                .ok_or_else(|| invalid_command("enable must be boolean"))?;
            if enable {
                let req = gravity_comp_request(body)?;
                let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
                inner.enter_gravity_comp(admitted, req)?;
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
    pub(super) fn start(self, inner: &mut SupervisorInner, admitted: crate::supervisor::Admitted, supervisor: &Arc<ArmSupervisor>, deps: &crate::grasp::GraspDeps) -> Result<(u64, f64), ArmError> {
        let stamp = admitted.stamp();
        match self {
            Self::Grasp(planned) => crate::grasp::begin(inner, admitted, supervisor, deps.source.clone(), planned),
            Self::Place(planned) => {
                let id = inner.begin_macro(admitted, "place", &planned.input, planned.request.tag.clone())?;
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
