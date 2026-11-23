//! Long actions of the supervisor: action lifecycle, macro (grasp/place/sequence) sub-motions, gripper actions and zero check, table-capture stillness, action reconciliation and stepping (track E).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

impl SupervisorInner {
    /// True on a real/physical driver (non-sim, non-mock) where missing jaw calibration
    /// curves must not fall back to `JawOpening::sim_default()`.
    pub fn requires_real_jaw_curve(&self) -> bool {
        self.driver.is_some()
            && matches!(self.follower.as_deref(), Some(f) if f != "sim" && f != "mock")
    }

    /// `max_w` (a grasp setup's `max_grasp_width_m`) against the gripper's jaw curve; None
    /// without a gripper policy.
    pub fn grasp_width_check(&self, max_w: f64) -> Option<crate::model::GraspWidthCheck> {
        let gp = self.config.gripper_policy.as_ref()?;
        Some(crate::model::check_grasp_width(gp, max_w, self.requires_real_jaw_curve()))
    }

    pub(crate) fn reset_table_stillness(&mut self) {
        self.table_still_first_rx_instants.clear();
        self.table_still_last_rx_instants.clear();
        self.table_still_anchor_positions.clear();
    }

    pub fn check_table_capture_stillness(&self) -> Result<(), ArmError> {
        if !self.feedback_numeric_valid {
            return Err(MotionError::InvalidCommand(
                "table_z_from_tcp requires valid joint feedback".into(),
            ).into());
        }
        if self.driver.is_some() && !self.hardware_feedback_fresh(Duration::from_millis(250)) {
            return Err(MotionError::InvalidCommand(
                "table_z_from_tcp requires fresh joint feedback".into(),
            ).into());
        }
        if self.active_trajectory.is_some()
            || self.servo_target.is_some()
            || self.commanded_velocities.iter().any(|v| v.abs() > 1e-3)
        {
            return Err(MotionError::InvalidCommand(
                "table_z_from_tcp requires the arm to be stationary".into(),
            ).into());
        }
        if self.measured_velocities.len() != self.config.dof
            || self.measured_velocities.iter().any(|v| !v.is_finite() || v.abs() > 0.02)
        {
            return Err(MotionError::InvalidCommand(
                "table_z_from_tcp requires measured joint velocities <= 0.02 rad/s".into(),
            ).into());
        }
        if self.driver.is_some() || self.still_window_ever_moved {
            let settled = if self.driver.is_some() {
                self.table_still_first_rx_instants.len() == self.config.dof
                    && self.table_still_last_rx_instants.len() == self.config.dof
                    && self.table_still_first_rx_instants
                        .iter()
                        .zip(&self.table_still_last_rx_instants)
                        .all(|(first, last)| {
                            last.checked_duration_since(*first)
                                .is_some_and(|d| d >= Duration::from_millis(200))
                        })
                    && self.table_still_anchor_positions.len() == self.measured_positions.len()
                    && self.measured_positions.iter().zip(&self.table_still_anchor_positions)
                        .all(|(m, a)| m.is_finite() && (m - a).abs() <= UNLOAD_STILL_POSITION_SPAN_RAD)
            } else {
                self.last_still_since_s.is_some_and(|t0| self.sim_time_s - t0 >= 0.20)
                    && self.still_anchor_positions.len() == self.measured_positions.len()
                    && self.measured_positions.iter().zip(&self.still_anchor_positions)
                        .all(|(m, a)| m.is_finite() && (m - a).abs() <= UNLOAD_STILL_POSITION_SPAN_RAD)
            };
            if !settled {
                return Err(MotionError::InvalidCommand(
                    "table_z_from_tcp requires the arm to remain still for at least 0.2 s with fresh distinct hardware samples".into(),
                ).into());
            }
        }
        Ok(())
    }

    /// Start a new action record; a running one is cancelled as SUPERSEDED.
    pub fn begin_action(&mut self, kind: &str, tag: Option<String>) -> u64 {
        let now = self.now_s();
        // any new action ends a running macro (its orchestrator sees the record cancelled)
        self.macro_run = None;
        #[cfg(feature = "policy-runtime")]
        if let Some(mut run) = self.policy_run.take() {
            run.stop_reason = Some("SUPERSEDED".to_string());
            if run.mode == crate::policy_runtime::PolicyMode::Live {
                self.servo_target = None;
                self.servo_settled_ticks.fill(0);
                self.servo_ws_min_depths = None;
            }
            self.last_policy_obs = Some(run.to_observation(false));
        }
        if let Some(mut old) = self.last_action.take() {
            if old.is_running() {
                old.state = "cancelled".into();
                old.code = Some("SUPERSEDED".into());
                old.message = Some(format!("Replaced by a new {kind} action"));
                old.finished_at = Some(now);
            }
            self.previous_action = Some((old, now + 2.0));
        }
        self.next_action_id += 1;
        let id = self.next_action_id;
        self.last_action = Some(ActionStatus {
            id, kind: kind.into(), tag, state: "running".into(), code: None, message: None, progress: 0.0,
            started_at: now, finished_at: None, holding: None, gripper_gap_rad: None, settled: None,
            task: None, phase: None, iteration: None, error_px: None, final_offset_px: None, acquire_height_m: None,
            episode: None, t_s: None, power_scale: None,
            sequence_id: None, step_index: None, step_type: None, repeat_index: None, repeats: None, failed_step: None, episodes: Vec::new(),
            align: None, align_turn_rad: None, angle_spread_deg: None, orientation: None, grasp_width_m: None, height_source: None, height_reason: None, object_height_m: None, close_height_m: None, probe_dz_m: None, depth_anchor: None, object_depth_m: None, object_length_m: None, jaw_clear: None, border: None, border_shift_px: None, probe_mode: None, lateral_baseline_m: None, lateral_baseline_err_mm: None, height_estimates: Vec::new(), yaw_estimates: Vec::new(), width_estimates: Vec::new(),
            recentre: None, recentre_iterations: None, descend_peak_residual_nm: None, descend_peak_tracking_rad: None,
            descend_contact: None, descend_contact_height_m: None, post_lift: None,
            pre_open: None, pre_open_m: None, held_width_m: None,
            stop_rad: None, offset_rad: None, pass: None,
            approach: None, approach_reason: None, approach_signs: None, approach_flipped: None,
            phase_ms: std::collections::BTreeMap::new(), motions: Vec::new(),
            constraint_id: None, geometry_id: None, violation_phase: None, reason: None,
            durability_confirmed: None, durability_warning: None, policy_stats: None, measurement: None,
        });
        self.action_lease_generation = self.active_lease.as_ref().map(|l| l.generation);
        self.quintic_settle = None;
        id
    }

    pub fn finish_action(&mut self, id: u64, state: &str, code: Option<&str>, message: Option<String>,
                     settled: Option<bool>, holding: Option<bool>, gap: Option<f32>) {
        let now = self.now_s();
        let mut store_offset_req: Option<(f32, Option<(crate::auth::OperatorAuth, Option<String>)>)> = None;
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.state = state.into();
            a.code = code.map(Into::into);
            a.message = message;
            a.finished_at = Some(now);
            if state != "failed" || code != Some("WORKSPACE_VIOLATION") {
                a.constraint_id = None;
                a.geometry_id = None;
                a.violation_phase = None;
                a.reason = None;
            }
            if state == "done" { a.progress = 1.0; }
            if settled.is_some() { a.settled = settled; }
            if holding.is_some() { a.holding = holding; }
            if gap.is_some() { a.gripper_gap_rad = gap; }
            if a.kind == "gripper_zero_check" {
                if let Some(params) = self.zero_check_params.take().filter(|p| p.action_id == id) {
                    if state == "done" {
                        let stop_rad = self.config.gripper_index().and_then(|idx| self.measured_positions.get(idx)).copied().unwrap_or(0.0);
                        let offset_rad = stop_rad - params.base_closed_rad;
                        let pass = offset_rad.abs() <= params.threshold_rad;
                        a.stop_rad = Some(stop_rad);
                        a.offset_rad = Some(offset_rad);
                        a.pass = Some(pass);
                        self.gripper_zero_checked = true;
                        self.last_zero_check_offset = Some(offset_rad);
                        if params.store_offset {
                            if offset_rad.abs() <= 0.5 {
                                store_offset_req = Some((offset_rad, params.operator_auth));
                            } else {
                                tracing::warn!("Gripper zero check offset {offset_rad:.4} exceeds 0.5 rad; storing refused");
                            }
                        }
                    }
                }
            }
            tracing::info!("Action {} ({}) {}{}", id, a.kind, state, code.map(|c| format!(" {c}")).unwrap_or_default());
        }
        if let Some((offset_rad, operator_auth)) = store_offset_req {
            let candidate_jaw = self.gripper_calib_jaw_opening.clone();
            match self.build_gripper_calibration_candidate(Some(offset_rad), candidate_jaw.clone()) {
                Err(e) => {
                    tracing::warn!("Failed to apply gripper calibration after zero_check: {e}");
                    if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                        a.state = "failed".into();
                        a.code = Some("INVALID_CALIBRATION".into());
                        a.message = Some(format!("Gripper calibration rejected: {e}"));
                    }
                }
                Ok((cand, _)) => {
                    let path = self.gripper_calib_path.clone().unwrap_or_else(crate::gripper_calib::path_from_env);
                    let file = crate::gripper_calib::GripperCalibFile {
                        offset_rad: Some(offset_rad),
                        jaw_opening: candidate_jaw.clone(),
                        ts: Some(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs())),
                        stale: self.gripper_calib_stale.clone(),
                        tool_id: Some(self.config.active_tool.id.clone()),
                        tool_config_digest: Some(self.config.active_tool.config_digest.clone()),
                    };
                    let text = serde_json::to_string_pretty(&file).unwrap_or_default();
                    if self.defer_zero_check_persist {
                        // Hold action as running until atomic persist-before-commit completes outside the lock.
                        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                            a.state = "running".into();
                            a.finished_at = None;
                        }
                        self.zero_check_persisting = Some(id);
                        self.pending_zero_check_persist = Some(PendingZeroCheckPersist {
                            action_id: id,
                            offset_rad,
                            jaw_opening: candidate_jaw,
                            base_revision: self.gripper_calib_revision,
                            lease_owner: self.active_lease.as_ref().map(|l| (l.client_id.clone(), l.generation)),
                            operator_auth,
                            write: crate::persist::PendingWrite::new(path, text),
                        });
                    } else {
                        let auth_ok = operator_auth.as_ref().is_none_or(|(a, tok)| !a.is_enforced() || a.is_operator(tok.as_deref()));
                        if !auth_ok {
                            if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                                a.state = "failed".into();
                                a.code = Some("OPERATOR_REQUIRED".into());
                                a.message = Some("Operator session expired or locked before gripper calibration persisted".into());
                            }
                        } else {
                            match crate::persist::StagedWrite::stage(path, text) {
                                Err(e) => {
                                    tracing::warn!("Failed to stage gripper calibration after zero_check: {e}");
                                    if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                                        a.state = "failed".into();
                                        a.code = Some("PERSIST_FAILED".into());
                                        a.message = Some(format!("Failed to persist gripper calibration: {e}"));
                                    }
                                }
                                Ok(staged) => match staged.rename_only() {
                                    Err(e) => {
                                        tracing::warn!("Failed to publish gripper calibration after zero_check: {e}");
                                        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                                            a.state = "failed".into();
                                            a.code = Some("PERSIST_FAILED".into());
                                            a.message = Some(format!("Failed to persist gripper calibration: {e}"));
                                        }
                                    }
                                    Ok(dir_sync) => {
                                        self.commit_gripper_calibration_candidate(cand);
                                        self.gripper_calib_revision += 1;
                                        let outcome = dir_sync.sync_parent_dir();
                                        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                                            match outcome {
                                                crate::persist::PublishOutcome::Published => {
                                                    a.durability_confirmed = Some(true);
                                                }
                                                crate::persist::PublishOutcome::PublishedDurabilityUnconfirmed(w) => {
                                                    a.durability_confirmed = Some(false);
                                                    a.durability_warning = Some(w);
                                                }
                                            }
                                        }
                                    }
                                },
                            }
                        }
                    }
                }
            }
        }
        if holding == Some(false) { self.held = None; }
        // a standalone close that detected an object counts as holding until a release completes
        if holding == Some(true) && self.held.is_none() {
            self.held = Some(HeldObject { label: String::new(), spill_safe: false });
        }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) { self.quintic_settle = None; }
        if self.macro_run.as_ref().is_some_and(|m| m.id == id) { self.macro_run = None; }
    }

    /// Start a park (NormalUnload) and record it as a `park` action. Returns its id when
    /// the park started.
    /// A park whose path enters a keep-out box is `WORKSPACE_VIOLATION`.
    /// After a fault or E-stop clear the motors may still hold the arm: safety Idle, torque on,
    /// re-arm pending (clear_fault keeps the hold rather than drop an arm that is not at rest).
    pub fn holding_after_clear(&self) -> bool {
        self.is_torqued && self.pending_rearm && !self.disable_pending
            && matches!(self.safety.state(), SafetyState::Idle)
            && !self.e_stop_latched.load(Ordering::SeqCst)
    }

    pub fn unload_action(&mut self) -> Result<Option<u64>, ArmError> {
        self.park_rejection = None;
        // A park from the hold after a clear re-arms through the verified enable path first (fresh
        // feedback, commanded = measured); without it NormalUnload keeps holding and parks nothing.
        if self.holding_after_clear() {
            self.set_torque(true)?;
        }
        self.execute_stop(StopPolicy::NormalUnload);
        if let Some(e) = self.park_rejection.take() { return Err(e.into()); }
        if self.mit.park_after_hand_back && !self.pending_normal_unload {
            // deferred behind the MIT brake and hand back: the action runs from now on
            let id = self.begin_action("park", None);
            self.mit.deferred_park_action = Some(id);
            return Ok(Some(id));
        }
        if !self.pending_normal_unload { return Ok(None); }
        let id = self.begin_action("park", None);
        if let Some(active) = self.active_trajectory.as_mut() { active.action_id = Some(id); }
        Ok(Some(id))
    }

    /// Preconditions shared by Cartesian and gripper actions.
    pub(crate) fn action_preconditions(&self) -> Result<(), ArmError> {
        if self.handover_settling_since.is_some() { return Err(SafetyError::HandoverSettling.into()); }
        if self.e_stop_latched.load(Ordering::SeqCst) { return Err(SafetyError::EStopActive.into()); }
        match self.safety.state() {
            SafetyState::EStop => return Err(SafetyError::EStopActive.into()),
            SafetyState::Fault(r) => return Err(SafetyError::FaultActive(r.clone()).into()),
            s if !s.is_operational() => return Err(SafetyError::NotArmed(s.clone()).into()),
            _ => {}
        }
        if self.pending_rearm || self.pending_normal_unload {
            return Err(SafetyError::NotArmed(self.safety.state().clone()).into());
        }
        self.collision_hold_check()?;
        if let Some(reason) = self.mit_motion_blocker() { return Err(MotionError::MotionBusy(reason).into()); }
        if let Some(reason) = self.busy_reason() { return Err(MotionError::MotionBusy(reason).into()); }
        Ok(())
    }

    /// Begin a macro action (`grasp`, `place`) after its first motion was planned from
    /// `input`. Its orchestrator drives it from now on. Returns its id.
    pub fn begin_macro(&mut self, kind: &str, input: &PlanInput, tag: Option<String>) -> Result<u64, ArmError> {
        self.check_plan_fresh(input)?;
        let id = self.begin_action(kind, tag);
        self.macro_run = Some(MacroRun { id, epoch: input.epoch, sub: SubState::Idle, stop_requested: false, phase: None, phase_start: None });
        self.set_action_phase(id, "start", 0.0, None, None);
        Ok(id)
    }

    /// Begin a sequence run (parent macro action of kind `sequence`, no Cartesian plan needed).
    pub fn begin_sequence_action(&mut self, tag: Option<String>, epoch: u64) -> Result<u64, ArmError> {
        self.begin_orchestrated_action("sequence", tag, epoch)
    }

    /// Begin a parent action of `kind` whose orchestrator runs outside the lock (sequence,
    /// calib_hover): epoch, preconditions, macro run record.
    pub fn begin_orchestrated_action(&mut self, kind: &str, tag: Option<String>, epoch: u64) -> Result<u64, ArmError> {
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        self.action_preconditions()?;
        self.gravity_comp_motion_busy()?;
        let id = self.begin_action(kind, tag);
        self.macro_run = Some(MacroRun { id, epoch, sub: SubState::Idle, stop_requested: false, phase: None, phase_start: None });
        self.set_action_phase(id, "start", 0.0, None, None);
        Ok(id)
    }

    /// The macro action `id` is still the running action.
    pub fn macro_alive(&self, id: u64) -> bool {
        self.macro_run.as_ref().is_some_and(|m| m.id == id && !m.stop_requested)
            && self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running())
    }

    pub fn macro_sub(&self, id: u64) -> Option<SubState> {
        self.macro_run.as_ref().filter(|m| m.id == id).map(|m| m.sub.clone())
    }

    /// Set `phase` / `progress` (and centring stats) of the running action `id`.
    pub fn set_action_phase(&mut self, id: u64, phase: &str, progress: f32, iteration: Option<u32>, error_px: Option<f32>) {
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) {
            let now = Instant::now();
            if m.phase.as_deref() != Some(phase) {
                if let (Some(prev_phase), Some(phase_start)) = (m.phase.take(), m.phase_start.take()) {
                    let elapsed_ms = now.duration_since(phase_start).as_millis() as u32;
                    if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                        *a.phase_ms.entry(prev_phase).or_insert(0) += elapsed_ms;
                    }
                }
                m.phase = Some(phase.to_string());
                m.phase_start = Some(now);
            }
        }
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.phase = Some(phase.into());
            a.progress = progress;
            // centring stats stay in the record after the phase (result of the closed loop)
            if iteration.is_some() { a.iteration = iteration; }
            if error_px.is_some() { a.error_px = error_px; }
        }
    }

    /// Grasp camera-down re-check result ([`ActionStatus::final_offset_px`]).
    pub fn set_action_final_offset(&mut self, id: u64, offset_px: [f32; 2]) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.final_offset_px = Some(offset_px);
        }
    }

    /// Grasp camera-down acquisition height ([`ActionStatus::acquire_height_m`]).
    pub fn set_action_acquire_height(&mut self, id: u64, height_m: f32) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.acquire_height_m = Some(height_m);
        }
    }

    /// Change diagnostic fields of the running action `id` (grasp status fields).
    pub fn update_action(&mut self, id: u64, f: impl FnOnce(&mut ActionStatus)) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) { f(a); }
    }

    /// Record structured workspace violation details on running action `id`.
    pub fn set_action_workspace_violation(&mut self, id: u64, detail: crate::kinematics::WorkspaceViolationDetail) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.constraint_id = detail.constraint_id;
            a.geometry_id = detail.geometry_id;
            a.violation_phase = detail.violation_phase;
            a.reason = detail.reason;
        }
    }

    /// Stop the running sub-motion of macro action `id` where the arm is (J1..J6 commanded to
    /// the measured positions, the gripper keeps its command) and report it done to the
    /// orchestrator. False when no sub-motion of `id` is running.
    pub fn end_macro_motion_at_measured(&mut self, id: u64) -> bool {
        if !self.macro_alive(id) || !self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) { return false; }
        self.stop_motion();
        let n = self.measured_positions.len().min(self.commanded_positions.len()).min(6);
        if self.measured_positions[..n].iter().all(|q| q.is_finite()) {
            for i in 0..n {
                let l = &self.config.limits[i];
                self.commanded_positions[i] = self.measured_positions[i].clamp(l.min_position, l.max_position);
            }
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.target_positions = self.commanded_positions.clone();
        self.safety.mark_motion_finished();
        if let Some(m) = self.macro_run.as_mut() {
            m.sub = SubState::Done(SubDone { settled: None, holding: None, gripper_gap_rad: None, at: Instant::now() });
        }
        true
    }

    pub(super) fn macro_sub_ready(&self, id: u64) -> Result<u64, ArmError> {
        let run = self.macro_run.as_ref().filter(|m| m.id == id && self.macro_alive(id))
            .ok_or_else(|| ArmError::Internal("the action is no longer running".into()))?;
        if run.sub == SubState::Running {
            return Err(MotionError::MotionBusy("a sub-motion of the action is running".into()).into());
        }
        Ok(run.epoch)
    }

    /// [`macro_sub_ready`](Self::macro_sub_ready) for sub-motions installed by other modules.
    pub(crate) fn macro_sub_ready_pub(&self, id: u64) -> Result<u64, ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        if epoch != self.safety.epoch_id() { return Err(MotionError::PlanStale("epoch changed during the action".into()).into()); }
        Ok(epoch)
    }

    pub(crate) fn set_macro_sub_running(&mut self, id: u64) {
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) { m.sub = SubState::Running; }
    }

    /// Install a planned sub-motion of macro action `id`. Returns its planned duration.
    pub fn install_macro_path(&mut self, id: u64, input: &PlanInput, path: JointPath) -> Result<f64, ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        if input.epoch != epoch { return Err(MotionError::PlanStale("epoch changed during the action".into()).into()); }
        self.check_plan_fresh(input)?;
        let duration = self.start_path_motion(input, &path)?;
        if input.power.is_some() { self.note_action_power_scale(id, path.power_scale); }
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Path(PathTrajectory::new(path))));
        if let Some(m) = self.macro_run.as_mut() { m.sub = SubState::Running; }
        Ok(duration + 0.5)
    }

    pub(super) fn check_gripper_close_torque_feedback(&self, request: &GripperRequest, policy: &crate::model::GripperPolicy, gripper_idx: usize) -> Result<(), ArmError> {
        if policy.max_close_torque_nm.is_none() {
            return Ok(());
        }
        let close_dir = if policy.closed_rad >= policy.open_rad { 1.0_f32 } else { -1.0_f32 };
        let cur = self.commanded_positions.get(gripper_idx).copied().unwrap_or(0.0);
        let target = match request.command {
            GripperCommand::Open => policy.open_rad,
            GripperCommand::Close => policy.closed_rad,
            GripperCommand::Move => policy.closed_rad + request.fraction.unwrap_or(0.0) as f32 * (policy.open_rad - policy.closed_rad),
            GripperCommand::Seek => policy.closed_rad + close_dir * 0.6,
        };
        let is_closing = matches!(request.command, GripperCommand::Close | GripperCommand::Seek)
            || (target - cur) * close_dir > 1e-4;
        if is_closing {
            if let Some(ref d) = self.driver {
                let has_torque_fb = d.lock().is_ok_and(|drv| drv.capabilities().torque_feedback);
                if !has_torque_fb {
                    return Err(MotionError::Unsupported(
                        "Gripper close requires torque_feedback when max_close_torque_nm is configured".into(),
                    ).into());
                }
            }
        }
        Ok(())
    }

    /// Effective closing speed limit enforced by the attached driver in position-force mode
    /// (`None` for legacy POS_VEL or drivers without a pos-force motor current cap).
    pub(crate) fn gripper_close_speed_limit(&self) -> Option<f32> {
        let gripper_idx = self.config.gripper_index()?;
        self.driver.as_ref()
            .and_then(|d| d.lock().ok())
            .and_then(|drv| drv.motor_current_cap())
            .filter(|cap| cap.mode == "pos_force" && cap.joint == gripper_idx)
            .and_then(|cap| cap.close_speed_rad_s)
            .filter(|v| v.is_finite() && *v > 0.0)
    }

    /// Start a gripper sub-motion of macro action `id`. Returns its expected duration.
    pub fn start_macro_gripper(&mut self, id: u64, request: GripperRequest) -> Result<f64, ArmError> {
        self.start_macro_gripper_within(id, request, None).map(|(dur, _)| dur)
    }

    /// Start a gripper sub-motion of macro action `id`, optionally verifying before installation
    /// that its planned duration fits within `max_budget_s`. Returns `(expected_duration, timeout_s)`.
    pub(crate) fn start_macro_gripper_within(
        &mut self,
        id: u64,
        request: GripperRequest,
        max_budget_s: Option<f64>,
    ) -> Result<(f64, f64), ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        let gripper_idx = self.config.gripper_index().ok_or_else(|| {
            MotionError::Unsupported("Gripper actions need a profile with a gripper joint".into())
        })?;
        let Some(policy) = self.config.gripper_policy.clone() else {
            return Err(MotionError::Unsupported("Gripper actions need a profile with a gripper policy".into()).into());
        };
        if epoch != self.safety.epoch_id() { return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into()); }
        self.action_preconditions()?;
        self.check_gripper_close_torque_feedback(&request, &policy, gripper_idx)?;
        let close_cap = self.gripper_close_speed_limit();
        let motion = GripperMotion::try_with_close_speed_limit(
            request, policy, self.config.limits[gripper_idx].clone(),
            self.commanded_positions[gripper_idx], gripper_idx, close_cap,
        )?;
        let duration = motion.expected_duration();
        let timeout_s = motion.timeout_s;
        if let Some(budget_s) = max_budget_s {
            if duration > budget_s {
                return Err(MotionError::InvalidCommand(format!(
                    "gripper motion needs {duration:.1} s (deadline {timeout_s:.1} s), exceeding remaining macro budget {budget_s:.1} s"
                )).into());
            }
        }
        self.releasing = matches!(motion.request.command, GripperCommand::Open);
        self.gripper_hold_vel_ceiling = Some(motion.limit.max_velocity);
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        let mut target = self.commanded_positions.clone();
        target[gripper_idx] = motion.target;
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Gripper(Box::new(motion))));
        if let Some(m) = self.macro_run.as_mut() { m.sub = SubState::Running; }
        Ok((duration, timeout_s))
    }

    /// Start a joint-space quintic sub-motion of macro action `id` (sequence pose step) to
    /// `target`, planned with `limits` from the commanded state; it counts as done once settled.
    /// Returns its planned duration.
    pub fn install_macro_quintic(&mut self, id: u64, target: Vec<f32>, limits: &[crate::model::JointLimit]) -> Result<f64, ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        if epoch != self.safety.epoch_id() { return Err(MotionError::PlanStale("epoch changed during the action".into()).into()); }
        self.action_preconditions()?;
        self.gravity_comp_motion_busy()?;
        let mut target = target;
        self.safety.validate_motion_command(Some(epoch), &target, &self.config.limits)?;
        self.torque_hold_filter(&mut target);
        self.power_hold_filter(&mut target);
        let traj = TrajectoryPlanner::plan(&self.commanded_positions, &self.commanded_velocities, &target,
            &vec![0.0; self.config.dof], None, limits)?;
        let (traj, power_scale) = self.power_fit_quintic(traj, &self.commanded_positions, &self.commanded_velocities, &target, limits, true)?;
        let samples = self.workspace_samples(&traj)?;
        self.check_workspace_path(&self.commanded_positions, samples.iter().map(Vec::as_slice))?;
        if self.config.gripper_index().is_some_and(|g| (target[g] - self.commanded_positions[g]).abs() > 1e-4) {
            self.gripper_hold_vel_ceiling = None;
        }
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        if let Some(s) = power_scale { self.note_action_power_scale(id, s); }
        let duration = traj.duration;
        self.quintic_settle = None;
        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, Some(id)).with_motion_limits(limits.to_vec()));
        if let Some(m) = self.macro_run.as_mut() { m.sub = SubState::Running; }
        Ok(duration + crate::motion::SETTLE_MAX_S)
    }

    /// End macro action `id` from its orchestrator (no-op when it already ended).
    pub fn finish_macro(&mut self, id: u64, state: &str, code: Option<&str>, message: Option<String>, holding: Option<bool>, gap: Option<f32>) {
        if !self.macro_alive(id) { return; }
        if self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) { self.stop_motion(); }
        if state == "done" { self.set_action_phase(id, "done", 1.0, None, None); }
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) {
            if let (Some(prev_phase), Some(phase_start)) = (m.phase.take(), m.phase_start.take()) {
                let ms = Instant::now().duration_since(phase_start).as_millis() as u32;
                if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                    *a.phase_ms.entry(prev_phase).or_insert(0) += ms;
                }
            }
        }
        self.finish_action(id, state, code, message, None, holding, gap);
    }

    /// `/motion/stop`: ends the running macro action now, under the caller's lock, so its
    /// orchestrator cannot install another sub-motion before the next tick.
    pub fn request_macro_stop(&mut self) {
        let Some(mut m) = self.macro_run.take() else { return; };
        m.stop_requested = true;
        let id = m.id;
        if self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) { self.stop_motion(); }
        if let (Some(prev_phase), Some(phase_start)) = (m.phase.take(), m.phase_start.take()) {
            let ms = Instant::now().duration_since(phase_start).as_millis() as u32;
            if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                *a.phase_ms.entry(prev_phase).or_insert(0) += ms;
            }
        }
        self.finish_action(id, "cancelled", Some("STOPPED"), Some("Motion stopped".into()), None, None, None);
    }

    /// Start a gripper action. Returns (action id, expected duration).
    pub fn start_gripper(&mut self, request: GripperRequest, epoch: u64) -> Result<(u64, f64), ArmError> {
        let gripper_idx = self.config.gripper_index().ok_or_else(|| {
            MotionError::Unsupported("Gripper actions need a profile with a gripper joint".into())
        })?;
        let Some(policy) = self.config.gripper_policy.clone() else {
            return Err(MotionError::Unsupported("Gripper actions need a profile with a gripper policy".into()).into());
        };
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        if self.config.active_tool.commissioning_required && request.speed > 0.4 + 1e-6 {
            return Err(SafetyError::CommissioningRequired(format!(
                "gripper (speed {:.2} rad/s > 0.40 rad/s)",
                request.speed
            )).into());
        }
        self.action_preconditions()?;
        self.check_gripper_close_torque_feedback(&request, &policy, gripper_idx)?;
        let close_cap = self.gripper_close_speed_limit();
        let motion = GripperMotion::try_with_close_speed_limit(
            request, policy, self.config.limits[gripper_idx].clone(),
            self.commanded_positions[gripper_idx], gripper_idx, close_cap,
        )?;
        self.releasing = matches!(motion.request.command, GripperCommand::Open);
        self.gripper_hold_vel_ceiling = Some(motion.limit.max_velocity);
        let duration = motion.expected_duration();
        let tag = motion.request.tag.clone();
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        let mut target = self.commanded_positions.clone();
        target[gripper_idx] = motion.target;
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        let id = self.begin_action("gripper", tag);
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Gripper(Box::new(motion))));
        Ok((id, duration))
    }

    /// Start a gripper zero check action. Returns (action id, expected duration).
    pub fn start_gripper_zero_check(&mut self, body: &serde_json::Value, epoch: u64) -> Result<(u64, f64), ArmError> {
        self.start_gripper_zero_check_with_auth(body, epoch, None)
    }

    pub fn start_gripper_zero_check_with_auth(
        &mut self,
        body: &serde_json::Value,
        epoch: u64,
        operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>,
    ) -> Result<(u64, f64), ArmError> {
        let gripper_idx = self.config.gripper_index().ok_or_else(|| {
            MotionError::Unsupported("Gripper zero check needs a profile with a gripper joint".into())
        })?;
        let Some(policy) = self.config.gripper_policy.clone() else {
            return Err(MotionError::Unsupported("Gripper zero check needs a profile with a gripper policy".into()).into());
        };
        if !policy.zero_check_supported {
            return Err(MotionError::Unsupported("Gripper zero check is not supported for this gripper policy".into()).into());
        }
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        self.action_preconditions()?;
        if !self.is_torqued {
            return Err(SafetyError::NotArmed(self.safety.state().clone()).into());
        }
        if self.held.is_some() {
            return Err(ArmError::Rejected {
                status: 409,
                code: "OBJECT_HELD".into(),
                message: "Gripper holds an object (or its release did not finish); open the gripper fully before the zero check".into(),
            });
        }
        let store_offset = body.get("store_offset").and_then(serde_json::Value::as_bool).unwrap_or(false);
        let threshold_rad = body.get("threshold_rad").and_then(serde_json::Value::as_f64).map(|v| v as f32).unwrap_or(0.10);
        if !threshold_rad.is_finite() || threshold_rad <= 0.0 {
            return Err(MotionError::InvalidCommand("threshold_rad must be positive and finite".into()).into());
        }

        let request = GripperRequest {
            command: GripperCommand::Seek,
            fraction: None,
            grasp: false,
            speed: 0.3,
            // derived from the travel at 0.3 rad/s (fully open -> closed takes about 20 s)
            timeout_s: None,
            tag: None,
        };
        self.check_gripper_close_torque_feedback(&request, &policy, gripper_idx)?;
        let close_cap = self.gripper_close_speed_limit();
        let motion = GripperMotion::try_with_close_speed_limit(
            request, policy, self.config.limits[gripper_idx].clone(),
            self.commanded_positions[gripper_idx], gripper_idx, close_cap,
        )?;
        let duration = motion.expected_duration();
        self.gripper_hold_vel_ceiling = Some(motion.limit.max_velocity);
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        let mut target = self.commanded_positions.clone();
        target[gripper_idx] = motion.target;
        self.target_positions = target;
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        let id = self.begin_action("gripper_zero_check", None);
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Gripper(Box::new(motion))));
        let base_closed_rad = self.base_gripper_policy.as_ref().map_or(0.0, |p| p.closed_rad);
        self.gripper_stalled = false;
        self.gripper_high_torque_ticks = 0;
        self.gripper_low_torque_ticks = 0;
        self.zero_check_params = Some(ZeroCheckParams {
            action_id: id,
            store_offset,
            threshold_rad,
            base_closed_rad,
            operator_auth,
        });
        Ok((id, duration))
    }

    pub fn gripper_zero_observation(&self) -> Option<crate::model::GripperZeroObservation> {
        let mut out = None;
        self.gripper_zero_observation_into(&mut out);
        out
    }

    /// [`Self::gripper_zero_observation`] into `out`, reusing its memory.
    pub fn gripper_zero_observation_into(&self, out: &mut Option<crate::model::GripperZeroObservation>) {
        let (Some(idx), Some(policy)) = (self.config.gripper_index(), self.config.gripper_policy.as_ref()) else {
            *out = None;
            return;
        };
        let j7 = self.measured_positions.get(idx).copied().unwrap_or(0.0);
        let offset_rad = self.gripper_calib_offset.or(self.last_zero_check_offset);
        let state = if self.gripper_calib_invalid {
            "calib_invalid"
        } else if self.gripper_zero_checked {
            "checked"
        } else {
            let min_j7 = policy.closed_rad.min(policy.open_rad) - 0.3;
            let max_j7 = policy.closed_rad.max(policy.open_rad) + 0.3;
            if j7 < min_j7 || j7 > max_j7 { "out_of_range" } else { "ok" }
        };
        match out {
            Some(o) => {
                crate::rt_core::scratch::set_str(&mut o.state, state);
                o.j7 = j7;
                o.offset_rad = offset_rad;
            }
            None => *out = Some(crate::model::GripperZeroObservation { state: state.to_string(), j7, offset_rad }),
        }
    }

    pub fn set_mock_gripper_stall_offset(&mut self, offset: Option<f32>) {
        self.mock_gripper_stall_offset = offset;
    }

    /// Single place that ends a running action whose reference went away (spec 2.3).
    pub(super) fn reconcile_action(&mut self, dt: f64, tick_output_accepted: bool) {
        let Some(action) = self.last_action.as_ref().filter(|a| a.is_running()) else { return; };
        let (id, is_park, is_replay) = (action.id, action.kind == "park", action.kind == "replay");
        let is_macro = self.macro_run.as_ref().is_some_and(|m| m.id == id);
        #[cfg(feature = "policy-runtime")]
        let is_policy = self.policy_run.as_ref().is_some_and(|p| p.action_id == id);
        #[cfg(not(feature = "policy-runtime"))]
        let is_policy = false;
        let carried = self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id));
        let lease_now = self.active_lease.as_ref().map(|l| l.generation);
        let cause: Option<(&str, &str, String)> = match self.safety.state() {
            SafetyState::EStop => Some(("failed", "ESTOP_ACTIVE", "E-stop latched".into())),
            SafetyState::Fault(FaultReason::WatchdogTimeout) => Some(("failed", "WATCHDOG", "Lease watchdog expired".into())),
            SafetyState::Fault(r) => Some(("failed", "FAULT", match r { FaultReason::Other(m) => m.clone(), r => format!("{r:?}") })),
            _ if !is_park && self.watchdog_park.is_some() => Some(("failed", "WATCHDOG", "Lease watchdog expired; parking".into())),
            _ if lease_now != self.action_lease_generation => Some(("cancelled", "LEASE_LOST", "Control lease lost or released".into())),
            _ => None,
        };
        if let Some((state, code, message)) = cause {
            // a macro sub-motion or a replay must not outlive its action
            if (is_macro || is_replay) && carried { self.stop_motion(); }
            if self.zero_check_persisting == Some(id) { self.zero_check_persisting = None; }
            #[cfg(feature = "policy-runtime")]
            if is_policy {
                let reason = match code {
                    "ESTOP_ACTIVE" => "ESTOP",
                    "WATCHDOG" => "WATCHDOG_TIMEOUT",
                    "LEASE_LOST" => "LEASE_LOST",
                    _ => "FAULT",
                };
                self.terminate_policy_run(reason, state, Some(code), Some(message));
                return;
            }
            self.finish_action(id, state, Some(code), Some(message), None, None, None);
            return;
        }
        if is_policy {
            return;
        }
        if let Some(run) = self.macro_run.as_ref().filter(|m| m.id == id) {
            // between sub-motions the orchestrator owns the action; it ends here only when
            // something else took the arm or a stop arrived
            if carried || self.zero_check_persisting == Some(id) { return; }
            let superseded = self.active_trajectory.is_some() || self.servo_target.is_some();
            // a joint-space sub-motion (sequence pose step) settles before it counts as done
            if !superseded && !run.stop_requested && run.sub == SubState::Running && self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) {
                if !tick_output_accepted { return; }
                let (cmd, meas, vel) = (&self.commanded_positions, &self.measured_positions, &self.measured_velocities);
                if let Some(settled) = self.quintic_settle.as_mut().and_then(|(_, settle)| settle.step(dt, cmd, meas, vel)) {
                    self.quintic_settle = None;
                    if let Some(m) = self.macro_run.as_mut() {
                        m.sub = SubState::Done(SubDone { settled: Some(settled), holding: None, gripper_gap_rad: None, at: Instant::now() });
                    }
                }
                return;
            }
            let ended: Option<(&str, &str, String)> = if superseded {
                Some(("cancelled", "SUPERSEDED", "Replaced by new motion".into()))
            } else if run.sub == SubState::Running || run.stop_requested {
                Some(("cancelled", "STOPPED", "Motion stopped".into()))
            } else { None };
            if let Some((state, code, message)) = ended { self.finish_action(id, state, Some(code), Some(message), None, None, None); }
            return;
        }
        let superseded = (self.active_trajectory.is_some() && !carried) || self.servo_target.is_some();
        if is_park {
            if self.mit.deferred_park_action == Some(id) { return; }
            if self.pending_normal_unload {
                let progress = self.active_trajectory.as_ref().and_then(|a| a.as_quintic()).map_or(1.0, |t| (t.elapsed / t.duration).min(1.0) as f32);
                if let Some(a) = self.last_action.as_mut() { a.progress = progress; }
            } else if !self.is_torqued && matches!(self.safety.state(), SafetyState::Idle) {
                self.finish_action(id, "done", None, None, None, None, None);
            } else if superseded {
                self.finish_action(id, "cancelled", Some("SUPERSEDED"), Some("Replaced by new motion".into()), None, None, None);
            } else {
                self.finish_action(id, "failed", Some("UNLOAD_FAILED"), Some("Park ended with torque still on".into()), None, None, None);
            }
            return;
        }
        if carried || self.zero_check_persisting == Some(id) { return; }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) {
            if superseded {
                self.finish_action(id, "cancelled", Some("SUPERSEDED"), Some("Replaced by new motion".into()), None, None, None);
                return;
            }
            if !tick_output_accepted { return; }
            let (cmd, meas, vel) = (&self.commanded_positions, &self.measured_positions, &self.measured_velocities);
            let result = self.quintic_settle.as_mut().and_then(|(_, settle)| settle.step(dt, cmd, meas, vel));
            if let Some(settled) = result {
                self.finish_action(id, "done", None, None, Some(settled), None, None);
            }
            return;
        }
        if superseded {
            self.finish_action(id, "cancelled", Some("SUPERSEDED"), Some("Replaced by new motion".into()), None, None, None);
        } else {
            self.finish_action(id, "cancelled", Some("STOPPED"), Some("Motion stopped".into()), None, None, None);
        }
    }

    /// Apply one executor step of a Path / Gripper / Replay reference.
    /// With tracking enabled, returns a deferred completion payload on `StepStatus::Done` so
    /// `"done"` is only published after that tick's output has been validated and written
    /// to the driver; with tracking off it publishes at once (e4f72b6) and returns None.
    pub(super) fn apply_action_step(&mut self, action_id: Option<u64>, mut out: StepOut) -> Option<DeferredActionDone> {
        // swap: the previous command's vectors go back to the scratch for the next step
        std::mem::swap(&mut self.commanded_positions, &mut out.positions);
        std::mem::swap(&mut self.commanded_velocities, &mut out.velocities);
        let status = std::mem::replace(&mut out.status, StepStatus::Running);
        let (progress, clear_gripper_stall) = (out.progress, out.clear_gripper_stall);
        self.scratch.step_out = out;
        if clear_gripper_stall {
            self.gripper_stalled = false;
            self.gripper_high_torque_ticks = 0;
            self.gripper_low_torque_ticks = 0;
        }
        let macro_sub = self.macro_run.as_ref().is_some_and(|m| Some(m.id) == action_id);
        if let (Some(id), Some(a)) = (action_id, self.last_action.as_mut()) {
            if a.id == id && a.is_running() && !macro_sub { a.progress = progress; }
        }
        match status {
            StepStatus::Running => None,
            StepStatus::Done { settled, holding, gripper_gap_rad } => {
                if !self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
                    // Tracking off: publish completion now, before this tick's driver write,
                    // exactly as before joint tracking (e4f72b6).
                    if self.releasing && matches!(self.active_trajectory.as_ref().map(|a| &a.reference), Some(Reference::Gripper(_))) {
                        self.releasing = false;
                        self.held = None;
                    }
                    self.active_trajectory = None;
                    self.commanded_velocities.fill(0.0);
                    self.target_positions = self.commanded_positions.clone();
                    if !self.pending_normal_unload { self.safety.mark_motion_finished(); }
                    if macro_sub {
                        if let Some(m) = self.macro_run.as_mut() {
                            m.sub = SubState::Done(SubDone { settled, holding, gripper_gap_rad, at: Instant::now() });
                        }
                    } else if let Some(id) = action_id { self.finish_action(id, "done", None, None, settled, holding, gripper_gap_rad); }
                    return None;
                }
                let clear_releasing = self.releasing
                    && matches!(self.active_trajectory.as_ref().map(|a| &a.reference), Some(Reference::Gripper(_)));
                self.active_trajectory = None;
                self.commanded_velocities.fill(0.0);
                self.target_positions = self.commanded_positions.clone();
                Some(DeferredActionDone {
                    action_id,
                    macro_sub,
                    settled,
                    holding,
                    gripper_gap_rad,
                    clear_releasing,
                })
            }
            StepStatus::Failed { code, message } => {
                if code == "TRACKING_STALL" {
                    self.stop_motion();
                    self.tracking.invalidate();
                    if self.measured_positions.iter().all(|q| q.is_finite()) {
                        self.commanded_positions = self.measured_positions.clone();
                    }
                    self.commanded_accelerations.fill(0.0);
                } else {
                    self.active_trajectory = None;
                }
                if action_id.is_none() && self.park_release == Some(ParkRelease::Opening) {
                    tracing::warn!("Park release opening failed ({code}: {message}); skipping re-close and keeping held state");
                    self.park_release = None;
                    self.park_gripper_contact = false;
                    self.releasing = false;
                }
                self.commanded_velocities.fill(0.0);
                self.target_positions = self.commanded_positions.clone();
                if !self.pending_normal_unload { self.safety.mark_motion_finished(); }
                if macro_sub {
                    if let Some(m) = self.macro_run.as_mut() { m.sub = SubState::Failed { code: code.into(), message }; }
                } else if let Some(id) = action_id { self.finish_action(id, "failed", Some(code), Some(message), None, None, None); }
                None
            }
        }
    }
}
