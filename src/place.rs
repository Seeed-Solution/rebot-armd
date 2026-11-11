//! Place action (`/api/v1/action/place`, docs/agent_action_api.md).
//!
//! Puts a held object down at a named pose's TCP or at a base-frame position. An async
//! orchestrator (crate::macro_action) runs the phases as sub-motions of one `place` action:
//! `lift` (straight up to max(current z, target z + approach)), `approach` (above the target
//! at target z + approach, in the place orientation), `descend` (straight down to the
//! target), `open` (only with `release`), `retreat` (straight up by the approach height,
//! only with `retreat`), `done`. All Cartesian phases are planned once up front from the
//! predicted joint states, so an unreachable target or a keep-out violation is rejected
//! before the action starts; each phase is planned again from the commanded state when it
//! runs.
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde_json::Value;

use crate::api::rest::ApiError;
use crate::kinematics::{self, Mat3, Pose, Vec3};
use crate::macro_action::{MacroCtx, Stop};
use crate::motion::{self, GoalSpec, GripperCommand, MotionError, PlanInput, Speed};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const DEFAULT_APPROACH_M: f64 = 0.08;
pub const APPROACH_RANGE_M: (f64, f64) = (0.02, 0.30);
pub const MIN_SPEED_SCALE: f64 = 0.1;
/// Lift is skipped when the TCP is already this close to (or above) the lift height.
const LIFT_EPS_M: f64 = 1e-3;
/// One sub-motion wait never exceeds this.
const PHASE_TIMEOUT_S: f64 = 60.0;

#[derive(Debug, Clone, PartialEq)]
pub struct PlaceRequest {
    /// Named pose the target came from.
    pub pose: Option<String>,
    /// Place point (end_link position, base frame).
    pub target: Vec3,
    /// Requested or pose orientation; None = keep the current one.
    pub orientation: Option<Mat3>,
    pub approach_m: f64,
    pub release: bool,
    pub retreat: bool,
    pub speed_scale: f64,
    pub tag: Option<String>,
}

impl PlaceRequest {
    /// Cartesian speed of every phase: the move_to defaults scaled by `speed_scale`
    /// (the profile speed cap still applies in the planner).
    pub fn speed(&self) -> Speed {
        Speed { linear: motion::DEFAULT_LINEAR_MPS * self.speed_scale, angular: motion::DEFAULT_ANGULAR_RPS * self.speed_scale }
    }
}

fn invalid(m: impl Into<String>) -> ArmError { MotionError::InvalidCommand(m.into()).into() }

fn boolean(body: &Value, key: &str, default: bool) -> Result<bool, ArmError> {
    match body.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v.as_bool().ok_or_else(|| invalid(format!("{key} must be a boolean"))),
    }
}

/// Body of `/api/v1/action/place` (named poses looked up in `inner`).
pub fn parse_place(inner: &SupervisorInner, body: &Value) -> Result<PlaceRequest, ArmError> {
    let has = |k: &str| body.get(k).is_some_and(|v| !v.is_null());
    if has("pose") == has("position") { return Err(invalid("exactly one of pose or position is required")); }
    let requested = motion::parse_orientation(body)?;
    let (pose, target, orientation) = if has("pose") {
        let name = body["pose"].as_str().ok_or_else(|| invalid("pose must be a string"))?;
        let stored = inner.poses.list.poses.iter().find(|p| p.name == name).ok_or_else(|| ArmError::Rejected {
            status: 404, code: "POSE_NOT_FOUND".into(), message: format!("No pose named '{name}'"),
        })?;
        // the stored TCP; poses saved without kinematics carry none, so use FK of the positions
        let tcp = match &stored.tcp {
            Some(t) => Pose::new(kinematics::rpy_to_r(t.rpy[0], t.rpy[1], t.rpy[2]), t.position),
            None if stored.positions.len() >= 6 => inner.config.arm_model.fk_f32(&stored.positions)
                .ok_or_else(|| invalid(format!("pose '{name}' positions are not a valid joint vector")))?,
            None => return Err(invalid(format!("pose '{name}' has no TCP"))),
        };
        (Some(name.to_string()), tcp.p, Some(requested.unwrap_or(tcp.r)))
    } else {
        (None, motion::parse_position(body)?, requested)
    };
    let approach_m = motion::num(body, "approach_height_m")?.unwrap_or(DEFAULT_APPROACH_M);
    if !(APPROACH_RANGE_M.0..=APPROACH_RANGE_M.1).contains(&approach_m) {
        return Err(invalid(format!("approach_height_m must be in [{}, {}]", APPROACH_RANGE_M.0, APPROACH_RANGE_M.1)));
    }
    let speed_scale = motion::num(body, "speed_scale")?.unwrap_or(1.0);
    if !(MIN_SPEED_SCALE..=1.0).contains(&speed_scale) { return Err(invalid(format!("speed_scale must be in [{MIN_SPEED_SCALE}, 1.0]"))); }
    Ok(PlaceRequest {
        pose, target, orientation, approach_m,
        release: boolean(body, "release", true)?,
        retreat: boolean(body, "retreat", true)?,
        speed_scale,
        tag: motion::tag(body)?,
    })
}

/// A request checked under the lock, ready to be planned outside it.
pub struct PendingPlace {
    pub input: PlanInput,
    pub request: PlaceRequest,
    pub open_rad: f32,
}

/// A place whose phases were found feasible from the start state.
pub struct PlannedPlace {
    pub input: PlanInput,
    pub request: PlaceRequest,
    /// Place orientation (requested, the pose's, or the start orientation).
    pub r_place: Mat3,
    /// Planned durations of lift, approach, descend, retreat (0 when skipped).
    pub durations: [f64; 4],
}

/// Under the lock: parse and check preconditions.
pub fn prepare(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<PendingPlace, ArmError> {
    if !body.is_object() { return Err(invalid("Command body must be an object")); }
    if !inner.config.six_plus_gripper() || inner.config.kinematics.is_none() {
        return Err(MotionError::Unsupported("Place needs a 7-DOF profile with kinematics and a gripper policy".into()).into());
    }
    let Some(policy) = inner.config.gripper_policy.as_ref() else {
        return Err(MotionError::Unsupported("Place needs a gripper policy".into()).into());
    };
    let open_rad = policy.open_rad;
    let request = parse_place(inner, body)?;
    if epoch != inner.safety.epoch_id() {
        return Err(crate::safety::SafetyError::EpochMismatch { expected: inner.safety.epoch_id(), received: epoch }.into());
    }
    let input = inner.plan_input()?;
    Ok(PendingPlace { input, request, open_rad })
}

/// Height the lift phase goes to from `z`.
fn lift_z(z: f64, req: &PlaceRequest) -> f64 { z.max(req.target[2] + req.approach_m) }

fn above(req: &PlaceRequest) -> Vec3 { [req.target[0], req.target[1], req.target[2] + req.approach_m] }

/// Outside the lock: plan lift → approach → descend → retreat chained from the start state.
pub fn plan(pending: PendingPlace) -> Result<PlannedPlace, ArmError> {
    let PendingPlace { input, request, open_rad } = pending;
    let speed = request.speed();
    let start = input.arm.fk(&input.q0);
    let r_place = request.orientation.unwrap_or(start.r);
    let mut cur = input.clone();
    let step = |cur: &mut PlanInput, goal: Pose| -> Result<f64, ArmError> {
        let path = motion::plan_cartesian(cur, goal, speed)?;
        let last = path.last();
        cur.q0 = std::array::from_fn(|i| f64::from(last[i]));
        Ok(path.duration)
    };
    let z = lift_z(start.p[2], &request);
    let lift = if z - start.p[2] > LIFT_EPS_M { step(&mut cur, Pose::new(start.r, [start.p[0], start.p[1], z]))? } else { 0.0 };
    let approach = step(&mut cur, Pose::new(r_place, above(&request)))?;
    let descend = step(&mut cur, Pose::new(r_place, request.target))?;
    if request.release { cur.j7 = open_rad; }
    // as the runtime phase: straight up by approach_m from where descend ended, same orientation
    let retreat = if request.retreat {
        let p = cur.arm.fk(&cur.q0).p;
        let goal = motion::resolve_goal(&cur, &GoalSpec::Absolute { p: [p[0], p[1], p[2] + request.approach_m], r: None });
        step(&mut cur, goal)?
    } else { 0.0 };
    Ok(PlannedPlace { input, request, r_place, durations: [lift, approach, descend, retreat] })
}

/// Rough total duration for the ack (paths + 0.5 s settle each, gripper open 2 s).
pub fn expected_duration(planned: &PlannedPlace) -> f64 {
    let moves: f64 = planned.durations.iter().filter(|d| **d > 0.0).map(|d| d + 0.5).sum();
    moves + if planned.request.release { 2.0 } else { 0.0 }
}

/// Start a place without a WS lease check (tests, in-process agents): prepare, plan, begin
/// and spawn the orchestrator. Returns (action id, expected duration).
pub async fn start(supervisor: &Arc<ArmSupervisor>, body: &Value) -> Result<(u64, f64), ApiError> {
    let pending = {
        let inner = supervisor.inner().read().await;
        prepare(&inner, body, inner.safety.epoch_id())?
    };
    let planned = tokio::task::spawn_blocking(move || plan(pending)).await
        .map_err(|e| ArmError::Internal(format!("planner task failed: {e}")))??;
    let id = supervisor.inner().write().await.begin_macro("place", &planned.input, planned.request.tag.clone())?;
    let duration = expected_duration(&planned);
    spawn(supervisor.clone(), id, planned);
    Ok((id, duration))
}

/// Spawn the orchestrator for an action begun with `SupervisorInner::begin_macro("place", ..)`.
pub fn spawn(supervisor: Arc<ArmSupervisor>, id: u64, planned: PlannedPlace) {
    tokio::spawn(run(supervisor, id, planned));
}

async fn phases(ctx: &MacroCtx, planned: &PlannedPlace) -> Result<(), Stop> {
    let req = &planned.request;
    let speed = req.speed();

    ctx.phase("lift", 0.05).await?;
    let p = ctx.pose().await.p;
    let z = lift_z(p[2], req);
    if z - p[2] > LIFT_EPS_M {
        ctx.move_arm("lift", GoalSpec::Absolute { p: [p[0], p[1], z], r: None }, speed).await?;
    }

    ctx.phase("approach", 0.2).await?;
    ctx.move_arm("approach", GoalSpec::Absolute { p: above(req), r: Some(planned.r_place) }, speed).await?;

    ctx.phase("descend", 0.55).await?;
    ctx.move_arm("descend", GoalSpec::Absolute { p: req.target, r: Some(planned.r_place) }, speed).await?;

    if req.release { ctx.gripper_phase("open", 0.75, GripperCommand::Open).await?; }

    if req.retreat {
        ctx.phase("retreat", 0.85).await?;
        let p = ctx.pose().await.p;
        ctx.move_arm("retreat", GoalSpec::Absolute { p: [p[0], p[1], p[2] + req.approach_m], r: None }, speed).await?;
    }
    Ok(())
}

async fn run(sup: Arc<ArmSupervisor>, id: u64, planned: PlannedPlace) {
    // generous whole-action budget: twice the planned time plus 20 s
    let budget = expected_duration(&planned) * 2.0 + 20.0;
    let ctx = MacroCtx {
        sup: sup.clone(), id,
        deadline: Instant::now() + Duration::from_secs_f64(budget),
        phase_timeout_s: PHASE_TIMEOUT_S, timeout_code: "PLACE_TIMEOUT",
        timeout_message: format!("Place did not finish within {budget:.0} s"),
    };
    let result = phases(&ctx, &planned).await;
    let mut inner = sup.inner().write().await;
    match result {
        // released: the jaws are open, nothing is held
        Ok(()) => inner.finish_macro(id, "done", None, None, planned.request.release.then_some(false), None),
        Err(Stop::Fail(code, message)) => {
            tracing::info!("Place {id} failed: {code} {message}");
            inner.finish_macro(id, "failed", Some(&code), Some(message), None, None);
        }
        Err(Stop::Cancelled) => inner.finish_macro(id, "cancelled", None, None, None, None),
    }
}
