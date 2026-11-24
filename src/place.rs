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
use crate::motion::{self, GoalFrame, GoalSpec, GripperCommand, MotionError, PlanInput, Speed};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const DEFAULT_APPROACH_M: f64 = 0.08;
pub const APPROACH_RANGE_M: (f64, f64) = (0.02, 0.30);
pub use crate::poses::MIN_SPEED_SCALE;
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
    pub auto_yaw: Option<bool>,
    /// `yaw: "outward"`: the place orientation's yaw is atan2(y, x) of the target and the
    /// approach and descent paths keep the yaw outward (`motion::PathOpts::yaw_outward`).
    pub yaw_outward: bool,
    pub speed_scale: f64,
    pub tag: Option<String>,
    /// Request field `approach` for the descent (src/backlash.rs, `place_final`).
    pub approach: crate::backlash::Request,
}

impl PlaceRequest {
    /// Cartesian speed of every phase: the move_to defaults scaled by `speed_scale`
    /// (the profile speed cap still applies in the planner).
    pub fn speed(&self) -> Speed {
        Speed { linear: motion::DEFAULT_LINEAR_MPS * self.speed_scale, angular: motion::DEFAULT_ANGULAR_RPS * self.speed_scale }
    }
}

fn invalid(m: impl Into<String>) -> ArmError { MotionError::InvalidCommand(m.into()).into() }

/// Body of `/api/v1/action/place` (named poses looked up in `inner`).
pub fn parse_place(inner: &SupervisorInner, body: &Value) -> Result<PlaceRequest, ArmError> {
    let has = |k: &str| body.get(k).is_some_and(|v| !v.is_null());
    if has("pose") == has("position") { return Err(invalid("exactly one of pose or position is required")); }
    let requested = motion::parse_orientation(body)?;
    let (pose, target, orientation) = if has("pose") {
        let name = body["pose"].as_str().ok_or_else(|| invalid("pose must be a string"))?;
        let stored = inner.poses.find(name)?;
        // FK of the stored joint positions (live TCP for the active tool); cached-only pose
        // (without valid joint vector) is accepted only for legacy stock-on-stock.
        let tcp = match (inner.config.tcp_pose_f32(&stored.positions), &stored.tcp) {
            (Some(pose), _) => pose,
            (None, Some(t)) => {
                let legacy_stock_only = inner.config.active_tool.is_stock
                    && stored.tool_id.is_none()
                    && stored.tool_config_digest.is_none();
                if !legacy_stock_only {
                    return Err(invalid(format!(
                        "pose '{name}' has no valid joint vector for active tool '{}'",
                        inner.config.active_tool.id
                    )));
                }
                Pose::new(kinematics::rpy_to_r(t.rpy[0], t.rpy[1], t.rpy[2]), t.position)
            }
            (None, None) if stored.positions.len() >= inner.config.arm_dof() => return Err(invalid(format!("pose '{name}' positions are not a valid joint vector"))),
            (None, None) => return Err(invalid(format!("pose '{name}' has no TCP"))),
        };
        (Some(name.to_string()), tcp.p, Some(requested.unwrap_or(tcp.r)))
    } else {
        (None, motion::parse_position(body)?, requested)
    };
    let approach_m = motion::num(body, "approach_height_m")?.unwrap_or(DEFAULT_APPROACH_M);
    if !(APPROACH_RANGE_M.0..=APPROACH_RANGE_M.1).contains(&approach_m) {
        return Err(invalid(format!("approach_height_m must be in [{}, {}]", APPROACH_RANGE_M.0, APPROACH_RANGE_M.1)));
    }
    let auto_yaw = match body.get("auto_yaw") {
        None | Some(Value::Null) => None,
        Some(Value::Bool(b)) => Some(*b),
        Some(_) => return Err(invalid("auto_yaw must be boolean")),
    };
    let speed_scale = crate::poses::speed_scale(body)?;
    Ok(PlaceRequest {
        pose, target, orientation, approach_m,
        release: motion::flag(body, "release", true)?,
        retreat: motion::flag(body, "retreat", true)?,
        auto_yaw,
        yaw_outward: motion::parse_yaw_mode(body)?,
        speed_scale,
        tag: motion::tag(body)?,
        approach: crate::backlash::parse_request(body)?,
    })
}

/// A request checked under the lock, ready to be planned outside it.
pub struct PendingPlace {
    pub input: PlanInput,
    pub request: PlaceRequest,
    pub open_rad: f32,
    pub held_spill_safe: bool,
    pub spill_max_tilt_deg: f64,
    pub table_clearance: Option<(Vec<Vec3>, f64)>,
}

/// A place whose phases were found feasible from the start state.
#[derive(Debug)]
pub struct PlannedPlace {
    pub input: PlanInput,
    pub request: PlaceRequest,
    /// Place orientation (requested, the pose's, or the start orientation).
    pub r_place: Mat3,
    /// Planned durations of lift, approach, descend, retreat (0 when skipped).
    pub durations: [f64; 4],
    pub orientation_levelled: bool,
    pub table_clearance: Option<(Vec<Vec3>, f64)>,
}

/// Under the lock: parse and check preconditions.
pub fn prepare(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<PendingPlace, ArmError> {
    if !body.is_object() { return Err(invalid("Command body must be an object")); }
    inner.check_commissioning_for_ordinary_motion("place")?;
    if !inner.config.has_cartesian_with_gripper() || inner.config.kinematics.is_none() {
        return Err(MotionError::Unsupported("Place needs an arm with kinematics and a gripper policy".into()).into());
    }
    let Some(policy) = inner.config.gripper_policy.as_ref() else {
        return Err(MotionError::Unsupported("Place needs a gripper policy".into()).into());
    };
    let open_rad = policy.open_rad;
    let request = parse_place(inner, body)?;
    if epoch != inner.safety.epoch_id() {
        return Err(crate::safety::SafetyError::EpochMismatch { expected: inner.safety.epoch_id(), received: epoch }.into());
    }
    let input = inner.plan_input_tcp()?;
    let held_spill_safe = inner.held.as_ref().is_some_and(|h| h.spill_safe);
    let spill_max_tilt_deg = inner.grasp_setup().map_or(8.0, |s| s.spill_max_tilt_deg);
    let table_clearance = match (inner.workspace_policy(), inner.grasp_setup()) {
        (Some((w, _)), gs) => {
            let rec = gs
                .as_ref()
                .map(|s| crate::grasp::CamModel::assumed(&inner.config.arm_model, s, &inner.config.camera_mount).rec)
                .unwrap_or_else(|| crate::kinematics::rpy_to_r(0.0, 0.0, f64::from(inner.config.camera_mount.rot_deg).to_radians()));
            let pts = crate::grasp::envelope_with_contact(
                Some(w),
                &inner.config.camera_mount,
                &rec,
                inner.config.effective_physical_contact_end_m(),
            );
            let bound = gs
                .map(|s| s.table_z_m + crate::grasp::TABLE_CLEARANCE_M)
                .unwrap_or_else(|| f64::from(w.floor_z_m));
            Some((pts, bound))
        }
        (None, Some(s)) => {
            let rec = crate::grasp::CamModel::assumed(&inner.config.arm_model, &s, &inner.config.camera_mount).rec;
            let pts = crate::grasp::envelope_with_contact(
                None,
                &inner.config.camera_mount,
                &rec,
                inner.config.effective_physical_contact_end_m(),
            );
            Some((pts, s.table_z_m + crate::grasp::TABLE_CLEARANCE_M))
        }
        (None, None) => None,
    };
    Ok(PendingPlace { input, request, open_rad, held_spill_safe, spill_max_tilt_deg, table_clearance })
}

/// Height the lift phase goes to from `z`.
fn lift_z(z: f64, req: &PlaceRequest) -> f64 { z.max(req.target[2] + req.approach_m) }

fn above(req: &PlaceRequest) -> Vec3 { [req.target[0], req.target[1], req.target[2] + req.approach_m] }

/// Outside the lock: plan lift → approach → descend → retreat chained from the start state.
pub fn plan(pending: PendingPlace) -> Result<PlannedPlace, ArmError> {
    let PendingPlace { input, request, open_rad, held_spill_safe, spill_max_tilt_deg, table_clearance } = pending;
    let speed = request.speed();
    let start = motion::start_pose(&input);
    let mut r_place = request.orientation.unwrap_or(start.r);
    let is_5dof = input.arm.arm_dof() == 5;
    let yaw_free = is_5dof && request.auto_yaw.unwrap_or(request.orientation.is_none());
    if yaw_free && request.orientation.is_none() && request.pose.is_none() {
        r_place = motion::apply_auto_yaw(&start.p, &request.target, r_place);
    } else if yaw_free && request.orientation.is_some() {
        r_place = motion::align_auto_yaw(&request.target, r_place);
    }
    if request.yaw_outward {
        r_place = motion::with_outward_yaw(&request.target, &r_place);
    }
    let mut orientation_levelled = false;
    if held_spill_safe {
        let tool_z = kinematics::column(&r_place, 0)[2];
        let tilt = (-tool_z).clamp(-1.0, 1.0).acos().to_degrees();
        if tilt > spill_max_tilt_deg {
            r_place = crate::grasp::look_down(&r_place);
            orientation_levelled = true;
        }
    }
    let mut cur = input.clone();
    let step = |cur: &mut PlanInput, goal: Pose, yaw_outward: bool| -> Result<motion::JointPath, ArmError> {
        let path = motion::plan_cartesian_opts(cur, goal, speed, motion::PathOpts { yaw_free, yaw_outward })?;
        let last = path.last();
        cur.q0 = std::array::from_fn(|i| f64::from(last[i]));
        Ok(path)
    };
    let check_table = |q: &[[f32; 7]]| -> Result<(), ArmError> {
        if let Some((points, bound)) = table_clearance.as_ref() {
            crate::backlash::check_table_clear(&input.arm, points, *bound, q.iter().map(|q| q.as_slice()))
                .map_err(|v| ArmError::from(v.into_motion_error()))?;
        }
        Ok(())
    };
    let z = lift_z(start.p[2], &request);
    let lift = if z - start.p[2] > LIFT_EPS_M {
        let lift_path = step(&mut cur, Pose::new(start.r, [start.p[0], start.p[1], z]), false)?;
        check_table(&lift_path.q)?;
        lift_path.duration
    } else {
        0.0
    };
    let approach_path = step(&mut cur, Pose::new(r_place, above(&request)), request.yaw_outward)?;
    check_table(&approach_path.q)?;
    let approach = approach_path.duration;
    let r_descend = if yaw_free && request.orientation.is_none() { motion::start_pose(&cur).r } else { r_place };
    let descend_path = step(&mut cur, Pose::new(r_descend, request.target), request.yaw_outward)?;
    check_table(&descend_path.q)?;
    let descend = descend_path.duration;
    if request.release { cur.j7 = open_rad; }
    // as the runtime phase: straight up by approach_m from where descend ended, same orientation
    let retreat = if request.retreat {
        let p = motion::start_pose(&cur).p;
        let goal = motion::resolve_goal(&cur, &GoalSpec::Absolute { p: [p[0], p[1], p[2] + request.approach_m], r: None });
        let retreat_path = step(&mut cur, goal, false)?;
        check_table(&retreat_path.q)?;
        retreat_path.duration
    } else { 0.0 };
    Ok(PlannedPlace { input, request, r_place, durations: [lift, approach, descend, retreat], orientation_levelled, table_clearance })
}

/// Rough total duration for the ack (paths + 0.5 s settle each, gripper open 2 s).
pub fn expected_duration(planned: &PlannedPlace) -> f64 {
    let moves: f64 = planned.durations.iter().filter(|d| **d > 0.0).map(|d| d + 0.5).sum();
    moves + if planned.request.release { 2.0 } else { 0.0 }
}

/// Start a place (tests, in-process agents): caller passes the `AdmissionStamp` captured at
/// admission (never re-sampled internally). Prepares, plans, validates `stamp` under the owner
/// lock, and spawns the orchestrator. Returns (action id, expected duration).
pub async fn start(supervisor: &Arc<ArmSupervisor>, stamp: crate::supervisor::AdmissionStamp, body: &Value) -> Result<(u64, f64), ApiError> {
    let pending = {
        let inner = supervisor.inner().read().await;
        prepare(&inner, body, stamp.estop_epoch)?
    };
    let planned = tokio::task::spawn_blocking(move || plan(pending)).await
        .map_err(|e| ArmError::Internal(format!("planner task failed: {e}")))??;
    let (id, duration) = {
        let mut inner = supervisor.inner().write().await;
        let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
        let id = inner.begin_macro(admitted, "place", &planned.input, planned.request.tag.clone())?;
        if planned.orientation_levelled {
            inner.update_action(id, |a| {
                a.orientation = Some("levelled".into());
            });
        }
        let duration = expected_duration(&planned) + inner.approach_estimate_s("place_final", planned.request.approach);
        (id, duration)
    };
    spawn(supervisor.clone(), id, planned, stamp);
    Ok((id, duration))
}

/// Spawn the orchestrator for an action begun with `SupervisorInner::begin_macro(stamp, "place", ..)`.
pub fn spawn(supervisor: Arc<ArmSupervisor>, id: u64, planned: PlannedPlace, stamp: crate::supervisor::AdmissionStamp) {
    tokio::spawn(run(supervisor, id, planned, stamp));
}

async fn phases(ctx: &MacroCtx, planned: &PlannedPlace) -> Result<(), Stop> {
    let req = &planned.request;
    let speed = req.speed();
    let is_5dof = planned.input.arm.arm_dof() == 5;
    let yaw_free = is_5dof && req.auto_yaw.unwrap_or(req.orientation.is_none());

    ctx.phase("lift", 0.05).await?;
    let p = ctx.tcp_pose().await.p;
    let z = lift_z(p[2], req);
    if z - p[2] > LIFT_EPS_M {
        ctx.move_arm_staged_in_with_table(
            GoalFrame::Tcp,
            "lift",
            GoalSpec::AbsoluteEx { p: [p[0], p[1], z], r: None, auto_yaw: yaw_free },
            speed,
            None,
            planned.table_clearance.clone(),
        )
        .await
        .map_err(|(s, _)| s)?;
    }

    ctx.phase("approach", 0.2).await?;
    let outward = |spec: GoalSpec| if req.yaw_outward { GoalSpec::YawOutward(Box::new(spec)) } else { spec };
    ctx.move_arm_staged_in_with_table(
        GoalFrame::Tcp,
        "approach",
        outward(GoalSpec::AbsoluteEx { p: above(req), r: Some(planned.r_place), auto_yaw: yaw_free }),
        speed,
        None,
        planned.table_clearance.clone(),
    )
    .await
    .map_err(|(s, _)| s)?;

    ctx.phase("descend", 0.55).await?;
    let r_descend = if yaw_free && req.orientation.is_none() { None } else { Some(planned.r_place) };
    // same-side approach (`place_final`, src/backlash.rs): the descent ends at the pre-target, the
    // short segment (phase final_approach) reaches the target, never rising to it from below
    let final_approach = crate::macro_action::FinalApproach {
        use_: "place_final",
        request: req.approach,
        rules: crate::backlash::DescentRules {
            table: planned.table_clearance.clone(),
            ..crate::backlash::DescentRules::tcp_at(planned.input.tcp_offset)
        },
    };
    ctx.move_arm_with_in(GoalFrame::Tcp, "descend", outward(GoalSpec::AbsoluteEx { p: req.target, r: r_descend, auto_yaw: yaw_free }), speed, Some(final_approach)).await?;

    if req.release { ctx.gripper_phase("open", 0.75, GripperCommand::Open).await?; }

    if req.retreat {
        ctx.phase("retreat", 0.85).await?;
        let p = ctx.tcp_pose().await.p;
        ctx.move_arm_staged_in_with_table(
            GoalFrame::Tcp,
            "retreat",
            GoalSpec::AbsoluteEx { p: [p[0], p[1], p[2] + req.approach_m], r: None, auto_yaw: yaw_free },
            speed,
            None,
            planned.table_clearance.clone(),
        )
        .await
        .map_err(|(s, _)| s)?;
    }
    Ok(())
}

async fn run(sup: Arc<ArmSupervisor>, id: u64, planned: PlannedPlace, stamp: crate::supervisor::AdmissionStamp) {
    // generous whole-action budget: twice the planned time plus 20 s
    let budget = expected_duration(&planned) * 2.0 + 20.0;
    let ctx = MacroCtx {
        sup: sup.clone(), id, stamp,
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
            inner.finish_macro(id, "failed", Some(&code), Some(message), None, None);
        }
        Err(Stop::Cancelled) => inner.finish_macro(id, "cancelled", None, None, None, None),
    }
    if let Some(a) = inner.last_action.as_ref().filter(|a| a.id == id) {
        let code = a.code.as_deref().unwrap_or(if a.state == "done" { "OK" } else { &a.state });
        let worst = a.motions.iter().max_by(|x, y| x.track_mm.total_cmp(&y.track_mm));
        tracing::info!("Place {id} {code}: phase_ms {:?}, worst motion {:?}", a.phase_ms, worst);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ArmConfig;
    use crate::poses::NamedPose;

    #[test]
    fn parse_place_rejects_cached_only_pose_under_non_stock_tool_and_accepts_legacy_stock() {
        let cached_pose = NamedPose {
            id: "legacy_bin".into(),
            name: "legacy_bin".into(),
            positions: vec![],
            tcp: Some(crate::poses::PoseTcp {
                position: [0.22, 0.0, 0.12],
                rpy: [0.0, std::f64::consts::FRAC_PI_2, 0.0],
            }),
            saved_at_ms: 1,
            profile_id: None,
            arm_dof: Some(6),
            gripper_joint: Some(6),
            joint_names: None,
            stale: None,
            tool_id: None,
            tool_config_digest: None,
        };

        let mut stock_sup = SupervisorInner::new(ArmConfig::rebot_dm());
        stock_sup.poses.list.poses.push(cached_pose.clone());
        let ok_req = parse_place(&stock_sup, &serde_json::json!({"pose": "legacy_bin"}))
            .expect("legacy stock-on-stock cached pose must remain accepted");
        assert_eq!(ok_req.target, [0.22, 0.0, 0.12]);

        let mut non_stock_cfg = ArmConfig::rebot_dm();
        non_stock_cfg.active_tool.is_stock = false;
        non_stock_cfg.active_tool.id = "soft_finray_gripper".into();
        non_stock_cfg.active_tool.config_digest = "v1:test".into();
        let mut tool_sup = SupervisorInner::new(non_stock_cfg);
        tool_sup.poses.list.poses.push(cached_pose);
        let err = parse_place(&tool_sup, &serde_json::json!({"pose": "legacy_bin"})).unwrap_err();
        assert!(
            matches!(err, ArmError::Motion(MotionError::InvalidCommand(ref m)) if m.contains("no valid joint vector")),
            "expected INVALID_COMMAND on cached-only pose with non-stock tool, got {err:?}"
        );
    }
}
