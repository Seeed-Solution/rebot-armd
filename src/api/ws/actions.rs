//! Prepared actions, Cartesian/joint/replay installs, macros, grasp dry-run, depth measurement,
//! and lease-free teach routing (family P).
//!
//! Moved from `api/ws.rs` and `api/ws/motion.rs` without behaviour changes (A1b PR 1).

#[allow(unused_imports)]
use super::*;

pub(super) enum ActionStart {
    Started(u64, f64),
    Plan(PendingPlan),
    Macro(PendingMacro),
    Replay(crate::replay::ReplayInput),
    DryRunGrasp(Value),
    Sequence(Box<crate::sequences::RunPlan>),
    CalibHover(Box<crate::calib_hover::HoverPlan>),
    /// `gripper_calibrate` (epoch, tag): begun with the supervisor handle at the call site.
    GripperCalibrate(u64, Option<String>),
    MeasureMultiview(Box<crate::measure_multiview::Plan>),
    #[cfg(feature = "policy-runtime")]
    Policy(Box<PendingPolicyStart>),
    #[cfg(feature = "policy-runtime")]
    PolicyStopped(bool),
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
    /// `path: "joint"`: the IK seeds ([`crate::motion::goal_seeds`]), the `speed_scale` of the joint
    /// quintic and the same-side approach request (None = off). None: the straight line.
    pub(super) joint: Option<JointGoal>,
    /// IK seeds of the goal ([`crate::motion::goal_seeds`]): the joint-space solve of `path: "joint"`,
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
pub(super) enum PendingMacro {
    Grasp(crate::grasp::PendingGrasp),
    Place(crate::place::PendingPlace),
}

#[allow(clippy::large_enum_variant)]
pub(super) enum PlannedMacro {
    Grasp(crate::grasp::PlannedGrasp),
    Place(crate::place::PlannedPlace),
}

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
    pub(super) fn start(
        self,
        admitted: crate::supervisor::Admitted<'_>,
        supervisor: &Arc<ArmSupervisor>,
        deps: &crate::grasp::GraspDeps,
    ) -> Result<(u64, f64), ArmError> {
        let stamp = admitted.stamp();
        match self {
            Self::Grasp(planned) => {
                crate::grasp::begin(admitted, supervisor, deps.source.clone(), planned)
            }
            Self::Place(planned) => {
                let (inner, proof) = admitted.consume()?;
                let id = inner.begin_macro_inner(
                    &proof,
                    "place",
                    &planned.input,
                    planned.request.tag.clone(),
                )?;
                if planned.orientation_levelled {
                    inner.update_action(id, |a| {
                        a.orientation = Some("levelled".into());
                    });
                }
                let duration = crate::place::expected_duration(&planned)
                    + inner.approach_estimate_s("place_final", planned.request.approach);
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

// Under the write lock, after lease / epoch / request_id checks.
#[allow(clippy::too_many_arguments)]
pub(super) fn prepare_action(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    path: &str,
    body: &Value,
    epoch: u64,
    request_id: &Value,
    auth: &crate::auth::OperatorAuth,
    ws_token: Option<&str>,
) -> Result<ActionStart, ApiError> {
    if !body.is_object() {
        return Err(invalid_command("Command body must be an object"));
    }
    let (kind, parsed) = match path {
        "/api/v1/action/move_to" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            ("move_to", crate::motion::parse_move_to(body))
        }
        "/api/v1/action/move_relative" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            ("move_relative", crate::motion::parse_move_relative(body))
        }
        "/api/v1/action/gripper" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            let mut request = crate::motion::parse_gripper(body).map_err(ArmError::from)?;
            if let Some(scale) = crate::motion::num(body, "speed_scale").map_err(ArmError::from)? {
                if !(0.05..=1.0).contains(&scale) {
                    return Err(ArmError::from(crate::motion::MotionError::InvalidCommand(
                        "speed_scale must be in [0.05, 1.0]".into(),
                    ))
                    .into());
                }
                if inner.config.active_tool.commissioning_required && scale > 0.2 + 1e-6 {
                    return Err(ArmError::Safety(SafetyError::CommissioningRequired(format!(
                        "gripper (speed_scale {scale:.2} > 0.2)"
                    )))
                    .into());
                }
                if body.get("speed").is_none_or(Value::is_null) {
                    request.speed = (2.0 * scale).clamp(0.01, 2.0);
                }
            }
            if body.get("baseline").is_some_and(|v| !v.is_null()) {
                return Err(invalid_command("baseline is not a gripper action field; calibrate with /api/v1/action/gripper_calibrate"));
            }
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (id, duration) = SupervisorInner::start_gripper(admitted, request, epoch)?;
            return Ok(ActionStart::Started(id, duration));
        }
        "/api/v1/action/gripper_zero_check" => {
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (id, duration) = SupervisorInner::start_gripper_zero_check_with_auth(
                admitted,
                body,
                epoch,
                Some((auth.clone(), ws_token.map(str::to_string))),
            )?;
            return Ok(ActionStart::Started(id, duration));
        }
        "/api/v1/action/grasp" => {
            if body.get("dry_run").and_then(Value::as_bool) == Some(true) {
                return Ok(ActionStart::DryRunGrasp(body.clone()));
            }
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::Macro(PendingMacro::Grasp(
                crate::grasp::prepare(inner, body, epoch)?,
            )));
        }
        "/api/v1/action/place" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::Macro(PendingMacro::Place(
                crate::place::prepare(inner, body, epoch)?,
            )));
        }
        "/api/v1/action/replay" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            let request = crate::replay::parse_replay(body).map_err(ArmError::from)?;
            return Ok(ActionStart::Replay(inner.replay_input(request, epoch)?));
        }
        "/api/v1/action/move_to_pose" => {
            let admitted = inner.check_admission(stamp).map_err(ArmError::from)?;
            let (id, duration) = crate::poses::start_action(admitted, body, epoch)?;
            return Ok(ActionStart::Started(id, duration));
        }
        "/api/v1/action/run_sequence" => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::Sequence(Box::new(
                crate::sequences::prepare_run(inner, body, epoch)?,
            )));
        }
        crate::calib_hover::PATH => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::CalibHover(Box::new(
                crate::calib_hover::prepare(inner, body, epoch)?,
            )));
        }
        crate::gripper_calib::CALIBRATE_PATH => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            let tag = crate::motion::tag(body).map_err(ArmError::from)?;
            return Ok(ActionStart::GripperCalibrate(epoch, tag));
        }
        crate::measure_multiview::PATH => {
            inner.preflight_admission(stamp).map_err(ArmError::from)?;
            inner.check_terminal_fifo_for_new_action().map_err(ArmError::from)?;
            return Ok(ActionStart::MeasureMultiview(Box::new(
                crate::measure_multiview::prepare(inner, body, epoch)?,
            )));
        }
        #[cfg(feature = "policy-runtime")]
        "/api/v1/action/policy/start" => {
            return Ok(ActionStart::Policy(Box::new(policy::prepare_policy_start(
                inner, stamp, body, epoch,
            )?)));
        }
        #[cfg(feature = "policy-runtime")]
        "/api/v1/action/policy/stop" => {
            return Ok(ActionStart::PolicyStopped(policy::prepare_policy_stop(inner)));
        }
        _ => return Err(invalid_command("Unknown action")),
    };
    let (mut spec, mut speed, tag) = parsed.map_err(ArmError::from)?;
    let _ = &mut spec;
    // move_to / move_relative keep their straight line to the goal; `precise: true` (profile
    // `applies_to` move_to_precise) or `approach: "same_side"` end it with the short segment
    let precise = crate::motion::flag(body, "precise", false).map_err(ArmError::from)?;
    let request = crate::backlash::parse_request(body).map_err(ArmError::from)?;
    let path_mode = crate::motion::parse_path_mode(body).map_err(ArmError::from)?;
    let same_side = precise || request != crate::backlash::Request::Profile;
    if path_mode == crate::motion::PathMode::Joint
        && body.get("speed").is_some_and(|v| !v.is_null())
    {
        return Err(invalid_command(
            "speed applies to path \"linear\"; path \"joint\" takes speed_scale (0.1..1.0)",
        ));
    }
    let linear_speed_scale = if path_mode == crate::motion::PathMode::Linear {
        if let Some(s) = crate::motion::num(body, "speed_scale").map_err(ArmError::from)? {
            if !(0.05..=1.0).contains(&s) {
                return Err(invalid_command("speed_scale must be in [0.05, 1.0]"));
            }
            if body.get("speed").is_none_or(Value::is_null) {
                speed.linear =
                    (crate::motion::MAX_LINEAR_MPS * s).min(crate::motion::MAX_LINEAR_MPS);
                speed.angular =
                    (crate::motion::MAX_ANGULAR_RPS * s).min(crate::motion::MAX_ANGULAR_RPS);
            }
            Some(s)
        } else {
            None
        }
    } else {
        None
    };
    let approach = if same_side && path_mode == crate::motion::PathMode::Linear {
        inner.approach_plan("move_to_precise", request)
    } else {
        Err(None)
    };
    let input = inner.plan_input_tcp()?;
    let (goal, opts) = crate::motion::resolve_goal_opts(&input, &spec);
    let joint = match path_mode {
        crate::motion::PathMode::Linear => {
            if inner.config.active_tool.commissioning_required {
                let scale_ok = linear_speed_scale.is_none_or(|s| s <= 0.2 + 1e-6);
                let speed_ok = speed.linear <= 0.2 * crate::motion::MAX_LINEAR_MPS + 1e-6
                    && speed.angular <= 0.2 * crate::motion::MAX_ANGULAR_RPS + 1e-6;
                if !scale_ok || !speed_ok {
                    return Err(ArmError::Safety(SafetyError::CommissioningRequired(format!(
                        "{kind} (requires speed_scale <= 0.2 or speed <= {:.3} m/s / {:.2} rad/s)",
                        0.2 * crate::motion::MAX_LINEAR_MPS,
                        0.2 * crate::motion::MAX_ANGULAR_RPS
                    )))
                    .into());
                }
            }
            None
        }
        crate::motion::PathMode::Joint => {
            let speed_scale = crate::poses::speed_scale(body)?;
            if inner.config.active_tool.commissioning_required && speed_scale > 0.2 + 1e-6 {
                return Err(ArmError::Safety(SafetyError::CommissioningRequired(format!(
                    "{kind} (speed_scale {speed_scale:.2} > 0.2)"
                )))
                .into());
            }
            Some(JointGoal {
                speed_scale,
                same_side: same_side.then_some(request),
                snapshot: inner.joint_goal_snapshot(speed_scale),
            })
        }
    };
    let seeds = crate::motion::goal_seeds(&input, &goal, &inner.goal_seed_poses());
    Ok(ActionStart::Plan(PendingPlan {
        request_id: request_id.clone(),
        kind,
        input,
        goal,
        speed,
        tag,
        opts,
        approach,
        joint,
        seeds,
    }))
}

/// Pending asynchronous action work produced by [`dispatch_action_command`].
pub(super) struct ActionPending {
    pub(super) plan: Option<(PendingPlan, u64)>,
    pub(super) macro_action: Option<(Value, PendingMacro, u64)>,
    pub(super) replay: Option<(Value, crate::replay::ReplayInput, u64)>,
    #[cfg(feature = "policy-runtime")]
    pub(super) policy: Option<(Value, PendingPolicyStart, u64, PolicyLoadingGuard)>,
    pub(super) dry_run_grasp: Option<(Value, Value)>,
}

/// Dispatch an `/api/v1/action/*` command under the supervisor write lock.
#[allow(clippy::too_many_arguments)]
pub(super) fn dispatch_action_command(
    inner: &mut SupervisorInner,
    stamp: crate::supervisor::AdmissionStamp,
    path: &str,
    value: &Value,
    epoch: u64,
    local_generation: u64,
    ws_auth: &WsAuth,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_deps: &crate::grasp::GraspDeps,
    recv_calib: &crate::calibration::CalibrationDeps,
    #[cfg(feature = "policy-runtime")] pending_policy_loads: &AtomicUsize,
    pending: &mut ActionPending,
) -> Option<Value> {
    let (auth_snap, token_snap) = ws_auth.auth_and_token();
    match prepare_action(
        inner,
        stamp,
        path,
        &value["body"],
        epoch,
        &value["request_id"],
        &auth_snap,
        token_snap.as_deref(),
    ) {
        Ok(ActionStart::Started(id, duration)) => {
            Some(command_ack(inner, &value["request_id"], Some((id, duration))))
        }
        Ok(ActionStart::Plan(plan)) => {
            pending.plan = Some((plan, local_generation));
            None
        }
        Ok(ActionStart::Macro(m)) => {
            pending.macro_action = Some((value["request_id"].clone(), m, local_generation));
            None
        }
        Ok(ActionStart::Replay(input)) => {
            pending.replay = Some((value["request_id"].clone(), input, local_generation));
            None
        }
        Ok(ActionStart::DryRunGrasp(body)) => {
            pending.dry_run_grasp = Some((value["request_id"].clone(), body));
            None
        }
        Ok(ActionStart::Sequence(plan)) => Some(
            match inner
                .check_admission(stamp)
                .map_err(ArmError::from)
                .and_then(|admitted| crate::sequences::begin(admitted, recv_supervisor, *plan))
            {
                Ok(started) => command_ack(inner, &value["request_id"], Some(started)),
                Err(error) => command_reject(inner, &value["request_id"], error.into()),
            },
        ),
        Ok(ActionStart::CalibHover(plan)) => Some(
            match inner
                .check_admission(stamp)
                .map_err(ArmError::from)
                .and_then(|admitted| {
                    crate::calib_hover::begin(
                        admitted,
                        recv_supervisor,
                        *plan,
                        recv_calib.upstream.clone(),
                        crate::calib_hover::jobs_dir(&recv_calib.path),
                    )
                }) {
                Ok((id, duration, job_id)) => {
                    let mut ack = command_ack(inner, &value["request_id"], Some((id, duration)));
                    ack["job_id"] = json!(job_id);
                    ack
                }
                Err(error) => command_reject(inner, &value["request_id"], error.into()),
            },
        ),
        Ok(ActionStart::GripperCalibrate(epoch, tag)) => Some(
            match inner
                .check_admission(stamp)
                .map_err(ArmError::from)
                .and_then(|admitted| {
                    crate::gripper_calib::begin_calibration(admitted, recv_supervisor, epoch, tag)
                }) {
                Ok(started) => command_ack(inner, &value["request_id"], Some(started)),
                Err(error) => command_reject(inner, &value["request_id"], error.into()),
            },
        ),
        Ok(ActionStart::MeasureMultiview(plan)) => Some(
            match inner
                .check_admission(stamp)
                .map_err(ArmError::from)
                .and_then(|admitted| {
                    crate::measure_multiview::begin(
                        admitted,
                        recv_supervisor,
                        *plan,
                        recv_deps.source.clone(),
                    )
                }) {
                Ok(started) => command_ack(inner, &value["request_id"], Some(started)),
                Err(error) => command_reject(inner, &value["request_id"], error.into()),
            },
        ),
        #[cfg(feature = "policy-runtime")]
        Ok(ActionStart::Policy(p)) => {
            inner.policy_loading = true;
            let loading_guard = PolicyLoadingGuard::new(recv_supervisor.clone());
            pending.policy = Some((value["request_id"].clone(), *p, local_generation, loading_guard));
            None
        }
        #[cfg(feature = "policy-runtime")]
        Ok(ActionStart::PolicyStopped(stopped)) => Some(policy::format_policy_stop_ack(
            inner,
            &value["request_id"],
            stopped,
            pending_policy_loads,
        )),
        Err(error) => Some(command_reject(inner, &value["request_id"], error)),
    }
}

/// Complete a joined `MacroCheck` (returns `None` when connection is closing).
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_macro_check(
    request_id: Value,
    generation: u64,
    stamp: crate::supervisor::AdmissionStamp,
    joined: Result<Result<PlannedMacro, ArmError>, tokio::sync::oneshot::error::RecvError>,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    recv_deps: &crate::grasp::GraspDeps,
    worker_closing: &AtomicBool,
) -> Option<Value> {
    let owner_transferred = || {
        let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while planning macro"});
        if let Some(s) = recv_supervisor.hub().latest() {
            r["epoch_id"] = json!(s.obs.epoch_id);
        }
        r
    };
    if worker_closing.load(Ordering::SeqCst) {
        return None;
    }
    if recv_supervisor.owner_mode() != crate::supervisor::OWNER_LEGACY {
        return Some(owner_transferred());
    }
    let mut inner = recv_supervisor.inner().write().await;
    if worker_closing.load(Ordering::SeqCst) {
        drop(inner);
        return None;
    }
    if inner.detached {
        drop(inner);
        return Some(owner_transferred());
    }
    Some(if let Err(err) = inner.preflight_admission(stamp) {
        command_reject(&inner, &request_id, ArmError::from(err).into())
    } else {
        match joined {
            Err(e) => command_reject(
                &inner,
                &request_id,
                ArmError::Internal(format!("planner task failed: {e}")).into(),
            ),
            Ok(Err(error)) => command_reject(&inner, &request_id, error.into()),
            Ok(Ok(planned)) => {
                if recv_generation.load(Ordering::SeqCst) != generation
                    || !inner.validate_servo_lease(recv_connection, generation)
                {
                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                } else {
                    match inner
                        .check_admission(stamp)
                        .map_err(ArmError::from)
                        .and_then(|admitted| planned.start(admitted, recv_supervisor, recv_deps))
                    {
                        Ok(started) => command_ack(&inner, &request_id, Some(started)),
                        Err(error) => command_reject(&inner, &request_id, error.into()),
                    }
                }
            }
        }
    })
}

/// Complete a Cartesian or `path: "joint"` plan outside the lock and install under the lock.
/// Returns `None` if `worker_closing` requests loop termination; `Some(resp)` otherwise.
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_pending_plan(
    plan: PendingPlan,
    generation: u64,
    stamp: crate::supervisor::AdmissionStamp,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    worker_closing: &Arc<AtomicBool>,
) -> Option<Option<Value>> {
    let PendingPlan {
        request_id,
        kind,
        input,
        goal,
        speed,
        tag,
        opts,
        approach,
        joint,
        seeds,
    } = plan;
    if let Some(joint) = joint {
        // path "joint": IK of the goal only (several seeds) without the lock,
        // then the move_to_pose quintic, checked under the lock
        // ... and the quintic's plan, power fit and swept-workspace check, which
        // the install reuses when the arm and the workspace did not change
        let JointGoal {
            speed_scale,
            same_side,
            snapshot,
        } = joint;
        let cancel = worker_closing.clone();
        let plan_conn = recv_connection.to_string();
        let solved = tokio::task::spawn_blocking(move || {
            maybe_pause_blocking_plan(&plan_conn);
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            let q = crate::motion::solve_goal_joints(&input, &goal, opts, &seeds);
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            let prepared = match (&q, same_side) {
                (Ok(q), None) => crate::supervisor::prepare_quintic(&input, &snapshot, q),
                _ => None,
            };
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            Some((input, q.map(|q| (q, prepared))))
        })
        .await;
        if worker_closing.load(Ordering::SeqCst) {
            return None;
        }
        let mut inner = recv_supervisor.inner().write().await;
        if worker_closing.load(Ordering::SeqCst) {
            return None;
        }
        if inner.detached {
            let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while planning"});
            if let Some(s) = recv_supervisor.hub().latest() {
                r["epoch_id"] = json!(s.obs.epoch_id);
            }
            Some(Some(r))
        } else {
            if let Some(hook) = inner.before_install_hook.take() {
                hook(&mut inner);
            }
            if let Err(err) = inner.preflight_admission(stamp) {
                Some(Some(command_reject(
                    &inner,
                    &request_id,
                    ArmError::from(err).into(),
                )))
            } else {
                Some(match solved {
                    Ok(None) => None,
                    Err(e) => Some(command_reject(
                        &inner,
                        &request_id,
                        ArmError::Internal(format!("planner task failed: {e}")).into(),
                    )),
                    Ok(Some((_, Err(error)))) => {
                        Some(command_reject(&inner, &request_id, ArmError::from(error).into()))
                    }
                    Ok(Some((input, Ok((q, prepared))))) => Some(
                        if recv_generation.load(Ordering::SeqCst) != generation
                            || !inner.validate_servo_lease(recv_connection, generation)
                        {
                            json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                        } else {
                            match inner
                                .check_admission(stamp)
                                .map_err(ArmError::from)
                                .and_then(|admitted| {
                                    SupervisorInner::install_joint_goal_prepared(
                                        admitted,
                                        &input,
                                        &q,
                                        speed_scale,
                                        kind,
                                        tag,
                                        same_side,
                                        prepared,
                                    )
                                }) {
                                Ok(started) => command_ack(&inner, &request_id, Some(started)),
                                Err(error) => command_reject(&inner, &request_id, error.into()),
                            }
                        },
                    ),
                })
            }
        }
    } else {
        let cancel = worker_closing.clone();
        let plan_conn = recv_connection.to_string();
        let planned = tokio::task::spawn_blocking(move || {
            maybe_pause_blocking_plan(&plan_conn);
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            let result = crate::motion::plan_cartesian_opts(&input, goal, speed, opts);
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            let pre = match (&approach, &result) {
                (Ok(plan), Ok(direct)) => {
                    crate::backlash::plan_cartesian_pre(&input, direct, plan, None, speed, opts)
                }
                (Err(Some(r)), _) => crate::backlash::Pre::Direct((*r).to_string()),
                _ => crate::backlash::Pre::Off,
            };
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            // a branch flip: would the joint-space move reach the goal (hint)?
            let joint_goal = match &result {
                Err(crate::motion::MotionError::IkBranchFlip { .. }) => {
                    crate::motion::solve_goal_joints(&input, &goal, opts, &seeds).ok()
                }
                _ => None,
            };
            if cancel.load(Ordering::Acquire) {
                return None;
            }
            Some((
                input,
                result
                    .map(|path| (path, pre, approach.ok()))
                    .map_err(|e| (e, joint_goal)),
            ))
        })
        .await;
        if worker_closing.load(Ordering::SeqCst) {
            return None;
        }
        let mut inner = recv_supervisor.inner().write().await;
        if worker_closing.load(Ordering::SeqCst) {
            return None;
        }
        if inner.detached {
            let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while planning"});
            if let Some(s) = recv_supervisor.hub().latest() {
                r["epoch_id"] = json!(s.obs.epoch_id);
            }
            Some(Some(r))
        } else {
            if let Some(hook) = inner.before_install_hook.take() {
                hook(&mut inner);
            }
            if let Err(err) = inner.preflight_admission(stamp) {
                Some(Some(command_reject(
                    &inner,
                    &request_id,
                    ArmError::from(err).into(),
                )))
            } else {
                Some(match planned {
                    Ok(None) => None,
                    Err(e) => Some(command_reject(
                        &inner,
                        &request_id,
                        ArmError::Internal(format!("planner task failed: {e}")).into(),
                    )),
                    Ok(Some((_, Err((error, joint_goal))))) => {
                        let mut api: ApiError = ArmError::from(error).into();
                        if joint_goal.is_some_and(|q| inner.joint_goal_feasible(&q)) {
                            api.message.push_str(crate::motion::JOINT_PATH_HINT);
                        }
                        Some(command_reject(&inner, &request_id, api))
                    }
                    Ok(Some((input, Ok((path, pre, approach))))) => Some(
                        if recv_generation.load(Ordering::SeqCst) != generation
                            || !inner.validate_servo_lease(recv_connection, generation)
                        {
                            json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while planning", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                        } else {
                            match inner
                                .check_admission(stamp)
                                .map_err(ArmError::from)
                                .and_then(|admitted| {
                                    SupervisorInner::install_path_same_side(
                                        admitted,
                                        &input,
                                        path,
                                        pre,
                                        approach.as_ref(),
                                        kind,
                                        tag,
                                    )
                                }) {
                                Ok(started) => command_ack(&inner, &request_id, Some(started)),
                                Err(error) => command_reject(&inner, &request_id, error.into()),
                            }
                        },
                    ),
                })
            }
        }
    }
}

/// Spawn a tracked macro planning/preflight task and return its [`MacroCheck`] handle.
#[allow(clippy::too_many_arguments)]
pub(super) fn spawn_macro_check(
    request_id: Value,
    pending: PendingMacro,
    generation: u64,
    stamp: crate::supervisor::AdmissionStamp,
    recv_deps: &crate::grasp::GraspDeps,
    worker_closing: &Arc<AtomicBool>,
    recv_connection: &str,
    worker_tasks: &ConnectionTasks,
) -> MacroCheck {
    let source = recv_deps.source.clone();
    let cancel = worker_closing.clone();
    let plan_conn = recv_connection.to_string();
    let (res_tx, res_rx) = tokio::sync::oneshot::channel();
    let abort = worker_tasks.spawn_abortable(async move {
        let planned_res = async {
            let cancel_blk = cancel.clone();
            let mut planned = tokio::task::spawn_blocking(move || {
                maybe_pause_blocking_plan(&plan_conn);
                if cancel_blk.load(Ordering::Acquire) {
                    return Err(ArmError::Safety(SafetyError::Stopped));
                }
                let out = pending.plan()?;
                if cancel_blk.load(Ordering::Acquire) {
                    return Err(ArmError::Safety(SafetyError::Stopped));
                }
                Ok(out)
            })
            .await
            .map_err(|e| ArmError::Internal(format!("planner task failed: {e}")))??;
            if cancel.load(Ordering::Acquire) {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            // grasp task requirements: checked without the lock, before the action begins
            if let PlannedMacro::Grasp(g) = &mut planned {
                crate::grasp::preflight(source.as_ref(), g).await?;
            }
            if cancel.load(Ordering::Acquire) {
                return Err(ArmError::Safety(SafetyError::Stopped));
            }
            Ok(planned)
        }
        .await;
        if !cancel.load(Ordering::Acquire) {
            let _ = res_tx.send(planned_res);
        }
    });
    MacroCheck {
        request_id,
        generation,
        stamp,
        abort: AbortOnDrop(abort),
        rx: res_rx,
    }
}

/// Complete a replay check outside the lock and install under the lock.
/// Returns `None` if `worker_closing` requests loop termination; `Some(resp)` otherwise.
#[allow(clippy::too_many_arguments)]
pub(super) async fn complete_pending_replay(
    request_id: Value,
    input: crate::replay::ReplayInput,
    generation: u64,
    stamp: crate::supervisor::AdmissionStamp,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_generation: &AtomicU64,
    worker_closing: &Arc<AtomicBool>,
) -> Option<Option<Value>> {
    let cancel = worker_closing.clone();
    let plan_conn = recv_connection.to_string();
    let planned = tokio::task::spawn_blocking(move || {
        maybe_pause_blocking_plan(&plan_conn);
        if cancel.load(Ordering::Acquire) {
            return None;
        }
        let out = crate::replay::plan(input);
        if cancel.load(Ordering::Acquire) {
            return None;
        }
        Some(out)
    })
    .await;
    if worker_closing.load(Ordering::SeqCst) {
        return None;
    }
    let mut inner = recv_supervisor.inner().write().await;
    if worker_closing.load(Ordering::SeqCst) {
        return None;
    }
    if inner.detached {
        let mut r = json!({"type":"command_reject", "request_id":request_id, "status":503, "code":"OWNER_TRANSFERRED", "error":"OWNER_TRANSFERRED", "message":"Ownership transferred to RT executor while checking replay"});
        if let Some(s) = recv_supervisor.hub().latest() {
            r["epoch_id"] = json!(s.obs.epoch_id);
        }
        Some(Some(r))
    } else if let Err(err) = inner.preflight_admission(stamp) {
        Some(Some(command_reject(
            &inner,
            &request_id,
            ArmError::from(err).into(),
        )))
    } else {
        Some(match planned {
            Ok(None) => None,
            Err(e) => Some(command_reject(
                &inner,
                &request_id,
                ArmError::Internal(format!("replay check task failed: {e}")).into(),
            )),
            Ok(Some(Err(error))) => Some(command_reject(&inner, &request_id, error.into())),
            Ok(Some(Ok(plan))) => Some(
                if recv_generation.load(Ordering::SeqCst) != generation
                    || !inner.validate_servo_lease(recv_connection, generation)
                {
                    json!({"type":"command_reject", "request_id":request_id, "status":409, "code":"LEASE_LOST", "message":"This connection lost the lease while the episode was checked", "error":"LEASE_LOST", "epoch_id":inner.safety.epoch_id()})
                } else {
                    match inner
                        .check_admission(stamp)
                        .map_err(ArmError::from)
                        .and_then(|admitted| SupervisorInner::install_replay(admitted, plan))
                    {
                        Ok(started) => command_ack(&inner, &request_id, Some(started)),
                        Err(error) => command_reject(&inner, &request_id, error.into()),
                    }
                },
            ),
        })
    }
}

/// Complete a dry-run grasp check outside the lock.
pub(super) async fn complete_dry_run_grasp(
    request_id: Value,
    body: Value,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_deps: &crate::grasp::GraspDeps,
) -> Value {
    let dry = crate::grasp::dry_run_task(recv_supervisor, recv_deps.source.as_ref(), &body).await;
    let mut ack = {
        let inner = recv_supervisor.inner().read().await;
        if !inner.detached {
            command_ack(&inner, &request_id, None)
        } else if let Some(s) = recv_supervisor.hub().latest() {
            let control_mode = if s.obs.control_mode == crate::model::ControlMode::GravityComp {
                "gravity_comp"
            } else {
                "position"
            };
            json!({"type":"command_ack", "request_id":request_id, "status":"accepted", "epoch_id":s.obs.epoch_id, "safety_state":s.obs.safety_state, "is_torqued":s.obs.is_torqued, "control_mode":control_mode})
        } else {
            command_ack(&inner, &request_id, None)
        }
    };
    for (k, v) in dry.fields() {
        ack[k.as_str()] = v;
    }
    ack
}

/// Handle lease-free `/api/v1/grasp/measure` (depth measurement).
/// Returns `false` if the outbound writer closed and the worker loop should break.
#[allow(clippy::too_many_arguments)]
pub(super) async fn handle_depth_measure(
    value: Value,
    last_ordinary_request: &mut Option<u64>,
    ws_auth: &WsAuth,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_deps: &crate::grasp::GraspDeps,
    ack_tx: &mpsc::Sender<Value>,
    worker_tasks: &ConnectionTasks,
    worker_writer: &WsWriterHandle,
) -> bool {
    let request_id = value.get("request_id").and_then(Value::as_u64);
    if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last) {
        let reply = json!({"type":"command_reject", "request_id":value.get("request_id"), "status":400, "code":"INVALID_COMMAND", "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"});
        return worker_writer.send_ordinary_control(reply).await;
    }
    *last_ordinary_request = request_id;
    if let Some(e) = lease_free_operator_check(ws_auth, crate::depth_measure::PATH, &value["body"]) {
        let mut reply = rejection(
            e,
            "command_reject",
            recv_supervisor.inner().read().await.safety.epoch_id(),
        );
        reply["request_id"] = value["request_id"].clone();
        return worker_writer.send_ordinary_control(reply).await;
    }
    let (sup, source, tx) = (
        recv_supervisor.clone(),
        recv_deps.source.clone(),
        ack_tx.clone(),
    );
    worker_tasks.spawn(async move {
        let result = crate::depth_measure::measure(&sup, source.as_ref(), &value["body"]).await;
        let _ = tx.send(crate::teach::reply(&value["request_id"], result)).await;
    });
    true
}

/// Handle lease-free teach commands (`crate::teach::is_lease_free`).
/// Returns `false` if the outbound writer closed and the worker loop should break.
#[allow(clippy::too_many_arguments)]
pub(super) async fn handle_teach_lease_free(
    value: Value,
    last_ordinary_request: &mut Option<u64>,
    ws_auth: &WsAuth,
    recv_supervisor: &Arc<ArmSupervisor>,
    recv_connection: &str,
    recv_client_label: &Mutex<Option<String>>,
    ack_tx: &mpsc::Sender<Value>,
    worker_tasks: &ConnectionTasks,
    worker_writer: &WsWriterHandle,
) -> bool {
    let request_id = value.get("request_id").and_then(Value::as_u64);
    if request_id.is_none() || last_ordinary_request.is_some_and(|last| request_id.unwrap() <= last) {
        let reply = json!({"type":"command_reject", "request_id":value.get("request_id"), "status":400, "code":"INVALID_COMMAND", "message":"request_id must be present and strictly increase", "error":"INVALID_COMMAND"});
        return worker_writer.send_ordinary_control(reply).await;
    }
    *last_ordinary_request = request_id;
    if let Some(e) = lease_free_operator_check(
        ws_auth,
        value["path"].as_str().unwrap_or(""),
        &value["body"],
    ) {
        let mut reply = rejection(
            e,
            "command_reject",
            recv_supervisor.inner().read().await.safety.epoch_id(),
        );
        reply["request_id"] = value["request_id"].clone();
        return worker_writer.send_ordinary_control(reply).await;
    }
    let caller_label = recv_client_label.lock().unwrap_or_else(|e| e.into_inner()).clone();
    let caller = crate::teach::Caller {
        connection: recv_connection.to_string(),
        client_id: value
            .get("client_id")
            .and_then(Value::as_str)
            .map(str::to_string)
            .or(caller_label),
    };
    let (sup, tx) = (recv_supervisor.clone(), ack_tx.clone());
    worker_tasks.spawn(async move {
        let path = value["path"].as_str().unwrap_or("").to_string();
        let result = crate::teach::lease_free(&sup, &caller, &path, &value["body"]).await;
        let _ = tx.send(crate::teach::reply(&value["request_id"], result)).await;
    });
    true
}
