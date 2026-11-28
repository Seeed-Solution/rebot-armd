//! Macro (grasp/place/sequence) orchestration, sub-motion deadlines, contact stepping, and gripper close helpers.

#[allow(unused_imports)]
use super::super::*;
#[allow(unused_imports)]
use crate::macro_action::{ChildTicket, ContactGuard, ContactWatch};

impl SupervisorInner {
    /// Begin a macro action (`grasp`, `place`) after its first motion was planned from
    /// `input`. Its orchestrator drives it from now on. Returns its id.
    pub fn begin_macro(admitted: Admitted<'_>, kind: &str, input: &PlanInput, tag: Option<String>) -> Result<u64, ArmError> {
        let (this, proof) = admitted.consume()?;
        this.begin_macro_inner(&proof, kind, input, tag)
    }

    pub(crate) fn begin_macro_inner(&mut self, proof: &crate::supervisor::AdmittedProof, kind: &str, input: &PlanInput, tag: Option<String>) -> Result<u64, ArmError> {
        self.check_terminal_fifo_for_new_action()?;
        self.check_plan_fresh(input)?;
        if !self.can_admit_macro_run() {
            return Err(MotionError::MotionBusy(
                "RT macro reclamation queue is full; retry after drain".into(),
            )
            .into());
        }
        let id = self.begin_action(kind, tag);
        self.macro_run = Some(MacroRun::new(id, input.epoch, proof.stamp()));
        self.set_action_phase(id, "start", 0.0, None, None);
        Ok(id)
    }

    /// Begin a sequence run (parent macro action of kind `sequence`, no Cartesian plan needed).
    pub fn begin_sequence_action(admitted: Admitted<'_>, tag: Option<String>, epoch: u64) -> Result<u64, ArmError> {
        Self::begin_orchestrated_action(admitted, "sequence", tag, epoch)
    }

    /// Begin a parent action of `kind` whose orchestrator runs outside the lock (sequence,
    /// calib_hover): stamp, epoch, preconditions, macro run record.
    pub fn begin_orchestrated_action(admitted: Admitted<'_>, kind: &str, tag: Option<String>, epoch: u64) -> Result<u64, ArmError> {
        let (this, proof) = admitted.consume()?;
        this.begin_orchestrated_action_inner(&proof, kind, tag, epoch)
    }

    pub(crate) fn begin_orchestrated_action_inner(&mut self, proof: &crate::supervisor::AdmittedProof, kind: &str, tag: Option<String>, epoch: u64) -> Result<u64, ArmError> {
        self.check_terminal_fifo_for_new_action()?;
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        self.action_preconditions()?;
        self.gravity_comp_motion_busy()?;
        if !self.can_admit_macro_run() {
            return Err(MotionError::MotionBusy(
                "RT macro reclamation queue is full; retry after drain".into(),
            )
            .into());
        }
        let id = self.begin_action(kind, tag);
        self.macro_run = Some(MacroRun::new(id, epoch, proof.stamp()));
        self.set_action_phase(id, "start", 0.0, None, None);
        Ok(id)
    }

    /// The macro action `id` is still the running action (and not already in its terminal
    /// finishing transition awaiting bus acceptance).
    pub fn macro_alive(&self, id: u64) -> bool {
        self.macro_run
            .as_ref()
            .is_some_and(|m| m.id == id && !m.stop_requested && !m.finishing)
            && self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running())
    }

    pub fn macro_sub(&self, id: u64) -> Option<SubState> {
        self.macro_run.as_ref().filter(|m| m.id == id).map(|m| m.sub)
    }

    /// Set `phase` / `progress` (and centring stats) of the running action `id`.
    pub fn set_action_phase(&mut self, id: u64, phase: &str, progress: f32, iteration: Option<u32>, error_px: Option<f32>) {
        let rt_owned = self.rt_executor_owned;
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) {
            let now = Instant::now();
            if m.phase.as_deref() != Some(phase) {
                if let (Some(prev_phase), Some(phase_start)) = (m.phase.take(), m.phase_start.take()) {
                    if !rt_owned {
                        let elapsed_ms = now.duration_since(phase_start).as_millis() as u32;
                        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                            *a.phase_ms.entry(prev_phase.as_str().to_owned()).or_insert(0) += elapsed_ms;
                        }
                    }
                }
                m.phase = Some(crate::model::HolderLabel::new(phase));
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
        if !self.rt_executor_owned && !self.defer_terminal_capture {
            if let Some(a) = self.last_action.as_ref().filter(|a| a.id == id) {
                self.mirror_rich_action_off_rt(a);
            }
        }
    }

    /// Grasp camera-down re-check result ([`ActionStatus::final_offset_px`]).
    pub fn set_action_final_offset(&mut self, id: u64, offset_px: [f32; 2]) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.final_offset_px = Some(offset_px);
        }
        if !self.rt_executor_owned && !self.defer_terminal_capture {
            if let Some(a) = self.last_action.as_ref().filter(|a| a.id == id) {
                self.mirror_rich_action_off_rt(a);
            }
        }
    }

    /// Grasp camera-down acquisition height ([`ActionStatus::acquire_height_m`]).
    pub fn set_action_acquire_height(&mut self, id: u64, height_m: f32) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.acquire_height_m = Some(height_m);
        }
        if !self.rt_executor_owned && !self.defer_terminal_capture {
            if let Some(a) = self.last_action.as_ref().filter(|a| a.id == id) {
                self.mirror_rich_action_off_rt(a);
            }
        }
    }

    /// Change diagnostic fields of the running action `id` (grasp status fields).
    pub fn update_action(&mut self, id: u64, f: impl FnOnce(&mut ActionStatus)) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) { f(a); }
        if !self.rt_executor_owned && !self.defer_terminal_capture {
            if let Some(a) = self.last_action.as_ref().filter(|a| a.id == id) {
                self.mirror_rich_action_off_rt(a);
            }
        }
    }

    /// Record structured workspace violation details on running action `id`.
    pub fn set_action_workspace_violation(&mut self, id: u64, detail: crate::kinematics::WorkspaceViolationDetail) {
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id && a.is_running()) {
            a.constraint_id = detail.constraint_id;
            a.geometry_id = detail.geometry_id;
            a.violation_phase = detail.violation_phase;
            a.reason = detail.reason;
        }
        if !self.rt_executor_owned && !self.defer_terminal_capture {
            if let Some(a) = self.last_action.as_ref().filter(|a| a.id == id) {
                self.mirror_rich_action_off_rt(a);
            }
        }
    }

    /// True when macro action `id`'s current child owns either `active_trajectory` or a
    /// pending deferred completion awaiting bus TX acceptance.
    pub(in crate::supervisor) fn macro_child_active_or_pending(&self, id: u64) -> bool {
        if self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) {
            return true;
        }
        let Some(ticket) = self.macro_run.as_ref().filter(|m| m.id == id).and_then(|m| m.ticket()) else {
            return false;
        };
        self.pending_deferred_done.as_ref().is_some_and(|d| {
            d.target == DeferredCompletionTarget::MacroSub { action_id: id, ticket }
        })
    }

    /// Stop the running sub-motion of macro action `id` where the arm is (J1..J6 commanded to
    /// the measured positions, the gripper keeps its command) without publishing `SubState::Done`.
    /// False when no sub-motion of `id` is running or pending deferred TX acceptance.
    pub(in crate::supervisor) fn stop_macro_motion_at_measured(&mut self, id: u64) -> bool {
        if !self.macro_alive(id) || !self.macro_child_active_or_pending(id) { return false; }
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
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
        true
    }

    /// Stop the running sub-motion of macro action `id` where the arm is (J1..J6 commanded to
    /// the measured positions, the gripper keeps its command) and report it done to the
    /// orchestrator. False when no sub-motion of `id` is running.
    pub fn end_macro_motion_at_measured(&mut self, id: u64) -> bool {
        if !self.stop_macro_motion_at_measured(id) { return false; }
        self.safety.mark_motion_finished();
        if let Some(m) = self.macro_run.as_mut() {
            m.sub = SubState::Done(SubDone { settled: None, holding: None, gripper_gap_rad: None, at: Instant::now() });
        }
        true
    }

    pub(crate) fn macro_sub_ready(&self, id: u64) -> Result<u64, ArmError> {
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

    /// A child (sub-motion) of macro action `id` was installed: counts it and makes it the
    /// current child (state Running, the pending contact guard).
    pub(crate) fn set_macro_sub_running(&mut self, id: u64) {
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) { m.begin_child(); }
    }

    /// The state of child `t` while its parent runs; None when the parent ended or was
    /// replaced, or `t` is not its current child (a late result is never read as another's).
    pub fn macro_child_state(&self, t: ChildTicket) -> Option<SubState> {
        if !self.macro_alive(t.parent.0) { return None; }
        self.macro_run.as_ref().filter(|m| m.ticket() == Some(t)).map(|m| m.sub)
    }

    /// What the contact guard of child `t` saw (also after the parent ended, until the next child).
    pub fn macro_child_watch(&self, t: ChildTicket) -> Option<ContactWatch> {
        self.macro_run.as_ref().filter(|m| m.ticket() == Some(t)).map(|m| m.child.watch)
    }

    /// Children of macro action `id` installed so far (None when it is not the current run).
    pub fn macro_children(&self, id: u64) -> Option<u32> {
        self.macro_run.as_ref().filter(|m| m.id == id).map(|m| m.children)
    }

    /// Arms the RT deadline of the current child of macro `id` (the tick fails it with `code` /
    /// `message` when it still runs after `deadline`). Returns its ticket.
    pub fn arm_macro_child(&mut self, id: u64, deadline: Instant, code: &str, message: String) -> Option<ChildTicket> {
        self.arm_macro_child_bounded(
            id,
            deadline,
            crate::safety::BoundedText::new(code),
            crate::motion::StepErrorText::new(&message),
        )
    }

    pub(crate) fn arm_macro_child_bounded(
        &mut self,
        id: u64,
        deadline: Instant,
        code: crate::safety::BoundedText<48>,
        message: crate::motion::StepErrorText,
    ) -> Option<ChildTicket> {
        let m = self.macro_run.as_mut().filter(|m| m.id == id && m.children > 0)?;
        m.child.deadline = Some(deadline);
        m.child.timeout = Some((code, message));
        m.ticket()
    }

    /// The contact guard of the next child installed for macro `id` (None clears it).
    pub fn guard_next_macro_child(&mut self, id: u64, guard: Option<ContactGuard>) {
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) { m.next_guard = guard; }
    }

    /// Child `t` still runs `grace` after its deadline (no tick ended it).
    pub fn macro_child_overdue(&self, t: ChildTicket, now: Instant, grace: Duration) -> bool {
        self.macro_run.as_ref()
            .filter(|m| m.ticket() == Some(t) && m.sub == SubState::Running)
            .and_then(|m| m.child.deadline)
            .is_some_and(|d| now > d + grace)
    }

    /// Earliest live owner-clock action/macro deadline across the active macro parent
    /// (`m.deadline`) and active running macro child (`m.child.deadline`), or `None` when no
    /// live deadline is armed (or the macro is already stopping/finishing).
    ///
    /// Combined ONCE per tick with the control-release deadline (`effective_cutoff = min(...)`)
    /// and installed at the driver TX choke point.
    pub fn active_action_deadline(&self) -> Option<Instant> {
        let macro_dl = self.macro_run.as_ref().and_then(|m| {
            if m.stop_requested || m.finishing {
                return None;
            }
            let child_dl = (m.sub == SubState::Running).then_some(m.child.deadline).flatten();
            match (m.deadline, child_dl) {
                (Some(p), Some(c)) => Some(p.min(c)),
                (Some(p), None) => Some(p),
                (None, Some(c)) => Some(c),
                (None, None) => None,
            }
        });
        #[cfg(not(target_arch = "wasm32"))]
        let session_dl = self.sessions.earliest_output_deadline();
        #[cfg(target_arch = "wasm32")]
        let session_dl: Option<Instant> = None;
        match (macro_dl, session_dl) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        }
    }

    /// True when macro `action_id` has an elapsed parent deadline or elapsed running-child
    /// deadline at `now` (inclusive `now >= deadline`, matching the driver TX cutoff).
    pub(in crate::supervisor) fn macro_deadline_elapsed(&self, action_id: u64, now: Instant) -> bool {
        self.macro_run.as_ref().is_some_and(|m| {
            m.id == action_id
                && !m.stop_requested
                && !m.finishing
                && (m.deadline.is_some_and(|d| now >= d)
                    || (m.sub == SubState::Running && m.child.deadline.is_some_and(|d| now >= d)))
        })
    }

    /// **Single owner-side macro deadline enforcement**: checks BOTH the whole-macro parent
    /// deadline (`m.deadline`, highest priority) and the running child deadline
    /// (`m.child.deadline`) against `now` (inclusive `now >= deadline`, matching the driver TX
    /// cutoff). Used pre-step to avoid advancing the reference after expiry, and post-output
    /// in `reconcile_action` (including after a deadline-refused TX) to end the expired macro
    /// or child exactly once with its timeout terminal. Returns `true` iff a deadline expired.
    pub fn enforce_macro_deadlines(&mut self, now: Instant) -> bool {
        enum MacroExpiry {
            Parent {
                id: u64,
                timeout_code: crate::safety::BoundedText<48>,
            },
            Child {
                id: u64,
            },
        }
        let expiry = self.macro_run.as_ref().and_then(|m| {
            if m.stop_requested || m.finishing {
                return None;
            }
            if m.deadline.is_some_and(|d| now >= d) {
                return Some(MacroExpiry::Parent {
                    id: m.id,
                    timeout_code: m.timeout_code,
                });
            }
            if m.sub == SubState::Running && m.child.deadline.is_some_and(|d| now >= d) {
                return Some(MacroExpiry::Child { id: m.id });
            }
            None
        });
        let Some(expiry) = expiry else {
            return false;
        };
        match expiry {
            MacroExpiry::Parent { id, timeout_code } => self.finish_expired_macro_parent(id, timeout_code),
            MacroExpiry::Child { id } => self.finish_expired_macro_child(id),
        }
    }

    pub(in crate::supervisor) fn finish_expired_macro_parent(
        &mut self,
        id: u64,
        timeout_code: crate::safety::BoundedText<48>,
    ) -> bool {
        let code_str = if timeout_code.is_empty() { "TIMEOUT" } else { timeout_code.as_str() };
        self.enter_macro_finish_rt(
            id,
            "failed",
            Some(code_str),
            crate::motion::StepErrorText::empty(),
            None,
            None,
            None,
            false,
        )
        .is_ok()
    }

    pub(in crate::supervisor) fn finish_expired_macro_child(&mut self, id: u64) -> bool {
        let _ = self.quiesce_macro_children_rt(id);
        let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) else {
            return false;
        };
        let (code, message) = m.child.timeout.take().unwrap_or_else(|| {
            (
                crate::safety::BoundedText::new("TIMEOUT"),
                crate::motion::StepErrorText::new("Sub-motion did not finish in time"),
            )
        });
        m.child.deadline = None;
        m.sub = SubState::Failed { code, message };
        true
    }

    /// Ends the current macro when `m.deadline` has passed by entering
    /// [`Self::enter_macro_finish_rt`]. Prefer [`Self::enforce_macro_deadlines`] on tick paths.
    pub fn expire_macro_parent(&mut self, now: Instant) -> bool {
        let Some((id, timeout_code)) = self
            .macro_run
            .as_ref()
            .filter(|m| !m.stop_requested && !m.finishing)
            .and_then(|m| m.deadline.filter(|d| now >= *d).map(|_| (m.id, m.timeout_code)))
        else {
            return false;
        };
        self.finish_expired_macro_parent(id, timeout_code)
    }

    /// Ends the running child of the current macro when its deadline passed: the motion
    /// stops cleanly to a zero-velocity hold and the child fails with the macro's timeout code
    /// (the parent stays running, its worker ends it with that code). True when it did.
    pub fn expire_macro_child(&mut self, now: Instant) -> bool {
        let Some(m) = self.macro_run.as_ref() else { return false; };
        if m.stop_requested
            || m.finishing
            || m.sub != SubState::Running
            || m.deadline.is_some_and(|d| now >= d)
            || !m.child.deadline.is_some_and(|d| now >= d)
        {
            return false;
        }
        self.finish_expired_macro_child(m.id)
    }

    /// The contact guard of the current macro child, once per tick while its motion runs
    /// (including a final step awaiting deferred TX acceptance):
    /// peaks, and a residual above its joint's bound (0: unguarded) ends the child at the
    /// measured position, done (`stop_code` None, a descent meant to touch) or failed with
    /// `stop_code`.
    pub(in crate::supervisor) fn step_macro_contact(&mut self) {
        let Some((id, guard)) = self.macro_run.as_ref()
            .filter(|m| !m.stop_requested && m.sub == SubState::Running)
            .and_then(|m| m.child.guard.map(|g| (m.id, g))) else { return; };
        if !self.macro_child_active_or_pending(id) { return; }
        let n = self.config.arm_dof().min(self.commanded_positions.len()).min(self.measured_positions.len());
        let trk = (0..n).map(|i| (self.commanded_positions[i] - self.measured_positions[i]).abs()).fold(0.0f32, f32::max);
        let residual = self.residual_torques;
        let Some(m) = self.macro_run.as_mut() else { return; };
        let w = &mut m.child.watch;
        w.peak_tracking_rad = Some(w.peak_tracking_rad.unwrap_or(0.0).max(trk));
        let Some(r) = residual else { return; };
        let pk = w.peak_residual_nm.get_or_insert([0.0; 2]);
        pk[0] = pk[0].max(r[1].abs());
        pk[1] = pk[1].max(r[2].abs());
        let over = (0..r.len().min(guard.max_residual_nm.len()))
            .find(|&j| guard.max_residual_nm[j] > 0.0 && r[j].abs() > guard.max_residual_nm[j]);
        let Some(joint) = over.filter(|_| w.contact_joint.is_none()) else { return; };
        let ticket = self.macro_run.as_ref().and_then(|m| m.ticket());
        if !self.stop_macro_motion_at_measured(id) { return; }
        let z = self.config.tcp_pose_f32(&self.measured_positions).map(|p| p.p[2]);
        if let Some(m) = self.macro_run.as_mut() {
            m.child.watch.contact_joint = Some(joint);
            m.child.watch.contact_tcp_z_m = z;
            if let Some(code) = guard.stop_code {
                self.safety.mark_motion_finished();
                m.sub = SubState::Failed {
                    code: crate::safety::BoundedText::new(code),
                    message: crate::motion::format_step_error(format_args!("contact on J{}", joint + 1)),
                };
            } else if let Some(ticket) = ticket {
                // Success-on-contact: motion is stopped and locked to measured immediately,
                // while `SubState::Done` commits strictly through the shared post-output
                // accepted-output gate once the replacement hold completes on the bus.
                self.pending_deferred_done = Some(super::DeferredActionDone {
                    target: super::DeferredCompletionTarget::MacroSub { action_id: id, ticket },
                    epoch_id: self.safety.epoch_id(),
                    lease_generation: self.active_lease.as_ref().map(|l| l.generation),
                    settled: None,
                    holding: None,
                    gripper_gap_rad: None,
                    clear_releasing: false,
                    is_gripper: false,
                });
            }
        }
    }

    /// Installs one macro child primitive (`rt_core::ops::Primitive::{CartesianPath, Gripper,
    /// JointQuinticLimited, Replay}`) for the commit of its descriptor, after the adapter
    /// checked the stamp and that the ticket's parent is the stamp's parent generation.
    /// Refuses (nothing installed, active motion unchanged): `PARENT_REPLACED` when the parent
    /// is not the running macro, `PLAN_STALE` when the ticket is not its next child or the
    /// prepared `start` differs from the commanded position (gripper included, `>= 1e-4`),
    /// `MOTION_BUSY` while a child runs, and the install's own codes. Arms the child's RT
    /// deadline (`now + timeout_s`, `timeout_code`) and contact guard. Returns the parent.
    ///
    /// `CartesianPath` takes one waypoint (`n == 1`, an end-link goal; more: `UNSUPPORTED`).
    /// `JointQuinticLimited` plans from the commanded state with `max_velocity` (0: the
    /// profile's); `duration_s` is the prepared estimate and is not enforced.
    pub fn commit_macro_child(admitted: Admitted<'_>, primitive: &crate::rt_core::ops::Primitive, now: Instant) -> Result<crate::rt_core::broker::ActionId, &'static str> {
        let (this, proof) = admitted
            .consume()
            .map_err(|e| crate::rt_core::ops::static_code(e.into()))?;
        use crate::rt_core::ops::Primitive;
        let Some(spec) = primitive.child().copied() else { return Err("INVALID_COMMAND"); };
        let start = match primitive {
            Primitive::CartesianPath { dof, start, .. } | Primitive::Gripper { dof, start, .. }
            | Primitive::JointQuinticLimited { dof, start, .. } | Primitive::Replay { dof, start, .. } => &start[..usize::from(*dof).min(start.len())],
            _ => return Err("INVALID_COMMAND"),
        };
        if matches!(primitive, Primitive::CartesianPath { n, .. } if *n != 1) {
            this.commit_child_checks(&spec, start)?;
            return Err("UNSUPPORTED");
        }
        // Zero-allocation stale-J7 fast-fail before entering `commit_child_install` (preserves
        // exact refusal precedence while keeping `armd-control` strictly 0 alloc / 0 free).
        match primitive {
            Primitive::Gripper { .. } => {
                this.commit_child_checks(&spec, start)?;
                if this.config.gripper_index().is_none() || this.config.gripper_policy.is_none() {
                    return Err("UNSUPPORTED");
                }
                let epoch = this.macro_run.as_ref().map(|m| m.epoch).unwrap_or(0);
                if epoch != this.safety.epoch_id() {
                    return Err("EPOCH_MISMATCH");
                }
                this.action_preconditions_static()?;
                this.check_pos_force_gripper_fresh_static()?;
            }
            Primitive::JointQuinticLimited { dof, q, .. } => {
                this.commit_child_checks(&spec, start)?;
                let epoch = this.macro_run.as_ref().map(|m| m.epoch).unwrap_or(0);
                if epoch != this.safety.epoch_id() {
                    return Err("PLAN_STALE");
                }
                this.action_preconditions_static()?;
                if this.gravity_comp.is_some() {
                    return Err("MOTION_BUSY");
                }
                let n = usize::from(*dof).min(q.len());
                this.safety
                    .validate_motion_command(Some(epoch), &q[..n], &this.config.limits)
                    .map_err(|e| crate::rt_core::ops::static_code(e.into()))?;
                if this.config.gripper_index().is_some_and(|g| {
                    n > g
                        && this.commanded_positions.len() > g
                        && (q[g] - this.commanded_positions[g]).abs() > 1e-4
                }) {
                    this.check_pos_force_gripper_fresh_static()?;
                }
            }
            Primitive::Replay { gripper, .. } => {
                this.commit_child_checks(&spec, start)?;
                if *gripper && this.config.gripper_index().is_some() {
                    if this.config.active_tool.commissioning_required {
                        return Err("COMMISSIONING_REQUIRED");
                    }
                    this.replay_preconditions_static()?;
                    this.check_pos_force_gripper_fresh_static()?;
                }
            }
            _ => {}
        }
        let primitive = *primitive;
        this.commit_child_install(spec, start, now, move |this, id| match primitive {
            Primitive::JointQuinticLimited { dof, q, max_velocity, .. } => {
                let mut limits = this.planning_limits();
                for (i, l) in limits.iter_mut().enumerate().take(usize::from(dof)) {
                    if max_velocity[i] > 0.0 { l.max_velocity = l.max_velocity.min(max_velocity[i]); }
                }
                this.install_macro_quintic_inner(&proof, id, q[..usize::from(dof)].to_vec(), &limits, None).map(drop)
            }
            Primitive::Gripper { fraction, speed, grasp, .. } => {
                let command = if grasp && fraction <= 0.0 { GripperCommand::Close } else if !grasp && fraction >= 1.0 { GripperCommand::Open } else { GripperCommand::Move };
                let request = GripperRequest { command, fraction: (command == GripperCommand::Move).then_some(f64::from(fraction)), grasp,
                    speed: f64::from(speed), timeout_s: Some(spec.timeout_s), tag: None };
                this.start_macro_gripper_learning_within_inner(&proof, id, request, crate::motion::LearnMode::Opening, None, false).map(drop)
            }
            Primitive::CartesianPath { waypoints, speed_mps, .. } => {
                let w = waypoints[0];
                let goal = crate::kinematics::Pose::new(crate::kinematics::rpy_to_r(f64::from(w[3]), f64::from(w[4]), f64::from(w[5])),
                    [f64::from(w[0]), f64::from(w[1]), f64::from(w[2])]);
                this.plan_input().and_then(|input| {
                    let speed = crate::motion::Speed { linear: f64::from(speed_mps), angular: crate::motion::DEFAULT_ANGULAR_RPS };
                    let path = crate::motion::plan_cartesian(&input, goal, speed).map_err(ArmError::from)?;
                    this.install_macro_path_inner(&proof, id, &input, path).map(drop)
                })
            }
            Primitive::Replay { episode, speed, gripper, .. } => {
                let request = crate::replay::ReplayRequest { episode: episode.as_str().to_string(), speed: f64::from(speed), gripper, tag: None, allow_stale: false };
                let epoch = this.safety.epoch_id();
                this.replay_input(request, epoch)
                    .and_then(crate::replay::plan)
                    .and_then(|planned| this.install_macro_replay_inner(&proof, id, planned).map(drop))
            }
            _ => Err(MotionError::InvalidCommand("not a child primitive".into()).into()),
        })
    }

    /// The checks of a child commit before anything is installed: the ticket's parent is the
    /// running macro, the ticket is its next child, no child runs, and `start` is the
    /// commanded position (every joint, gripper included, `>= START_TOLERANCE`).
    pub(crate) fn commit_child_checks(&self, spec: &crate::rt_core::ops::ChildSpec, start: &[f32]) -> Result<u64, &'static str> {
        use crate::rt_core::ops::START_TOLERANCE;
        let id = spec.ticket.parent.0;
        let Some(run) = self.macro_run.as_ref().filter(|m| m.id == id && self.macro_alive(id)) else { return Err("PARENT_REPLACED"); };
        if spec.ticket.child_seq != run.child.seq.wrapping_add(1) { return Err("PLAN_STALE"); }
        if run.sub == SubState::Running { return Err("MOTION_BUSY"); }
        if start.len() != self.commanded_positions.len()
            || start.iter().zip(&self.commanded_positions).any(|(s, c)| !s.is_finite() || (s - c).abs() >= START_TOLERANCE) {
            return Err("PLAN_STALE");
        }
        Ok(id)
    }

    /// Commits one child of the running macro: [`Self::commit_child_checks`], the child's
    /// contact guard, `install(self, parent id)`, then its RT deadline. A refused install
    /// leaves no guard armed. Returns the parent.
    pub(crate) fn commit_child_install(&mut self, spec: crate::rt_core::ops::ChildSpec, start: &[f32], now: Instant,
        install: impl FnOnce(&mut Self, u64) -> Result<(), ArmError>) -> Result<crate::rt_core::broker::ActionId, &'static str> {
        let id = self.commit_child_checks(&spec, start)?;
        self.guard_next_macro_child(id, spec.contact);
        if let Err(e) = install(self, id) {
            self.guard_next_macro_child(id, None);
            return Err(crate::rt_core::ops::static_code(e));
        }
        let deadline = now + Duration::from_secs_f64(if spec.timeout_s.is_finite() { spec.timeout_s.clamp(0.0, 1e6) } else { 1e6 });
        self.arm_macro_child_bounded(
            id,
            deadline,
            crate::safety::BoundedText::new(spec.timeout_code),
            crate::motion::StepErrorText::new("Sub-motion did not finish in time"),
        );
        Ok(crate::rt_core::broker::ActionId(id))
    }

    /// Installs a prepared Cartesian path as a sub-motion of macro action `id` (the RT
    /// commit of a `Primitive::Prepared` child: no IK). Returns its planned duration.
    pub(crate) fn install_macro_prepared_path(admitted: Admitted<'_>, id: u64, path: JointPath, power_noted: bool) -> Result<f64, ArmError> {
        let (this, proof) = admitted.consume()?;
        let epoch = this.macro_sub_ready(id)?;
        this.check_prepared_fresh(epoch)?;
        let duration = this.start_path_motion_at_inner(&proof, epoch, &path)?;
        if power_noted { this.note_action_power_scale(id, path.power_scale); }
        this.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Path(PathTrajectory::new(path))));
        this.set_macro_sub_running(id);
        Ok(duration + 0.5)
    }

    /// Installs a prepared Cartesian path borrowing directly from an immutable pinned
    /// [`crate::rt_core::broker::PlanSlotRef`] as a sub-motion of macro action `id` (zero
    /// sample/param vector copies). Returns its planned duration.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn install_macro_prepared_path_slot(admitted: Admitted<'_>, id: u64, dof: usize, slot: crate::rt_core::broker::PlanSlotRef<crate::rt_core::ops::PlanSlot>, duration_s: f64, power_scale: Option<f64>) -> Result<f64, ArmError> {
        let (this, proof) = admitted.consume()?;
        let epoch = this.macro_sub_ready(id)?;
        this.validate_prepared_joint_path(dof, &slot.path, duration_s)
            .map_err(|_| ArmError::Motion(MotionError::InvalidCommand("invalid prepared Cartesian path".into())))?;
        this.check_prepared_fresh(epoch)?;
        let last = &slot.path.samples()[slot.path.len() - 1][..dof];
        let duration = this.start_path_motion_slice_inner(&proof, epoch, last, duration_s)?;
        let scale = power_scale.unwrap_or(1.0);
        if power_scale.is_some() { this.note_action_power_scale(id, scale); }
        let arm_dof = this.config.arm_dof();
        this.active_trajectory = Some(ActiveTrajectory::new(
            Some(id),
            Reference::Path(PathTrajectory::from_plan_slot(slot, dof, arm_dof, duration_s)),
        ));
        this.set_macro_sub_running(id);
        Ok(duration + 0.5)
    }

    /// Ends the running macro without its orchestrator (a guard, a fault, a newer action):
    /// records its terminal (`state`, `code`, child count) unless `finish_action` already did,
    /// so `CoreOps::action_outcome` / `action_children` answer for it.
    pub(crate) fn drop_macro_run(&mut self, state: &str, code: Option<&str>) {
        let Some(id) = self.macro_run.as_ref().map(|m| m.id) else { return; };
        if self.core.last_macro().is_none_or(|m| m.action.0 != id) { self.record_macro_terminal(id, state, code); }
        if let Some(run) = self.macro_run.take() {
            self.retire_macro_run(run);
        }
    }

    /// Keeps the terminal record of macro `id` (outcome with its code, child count) in the
    /// core, so it outlives the action record (`CoreOps::action_outcome`, `action_children`).
    pub(in crate::supervisor) fn record_macro_terminal(&mut self, id: u64, state: &str, code: Option<&str>) {
        let Some(children) = self.macro_run.as_ref().filter(|m| m.id == id).map(|m| m.children) else { return; };
        let outcome = crate::rt_core::ops::outcome_of(state, code).unwrap_or(crate::rt_core::broker::Outcome::Failed("FAILED"));
        self.core.record_macro_terminal(crate::rt_core::MacroTerminal { action: crate::rt_core::broker::ActionId(id), children, outcome });
    }

    /// Install a planned sub-motion of macro action `id`. Returns its planned duration.
    pub fn install_macro_path(admitted: Admitted<'_>, id: u64, input: &PlanInput, path: JointPath) -> Result<f64, ArmError> {
        let (this, proof) = admitted.consume()?;
        this.install_macro_path_inner(&proof, id, input, path)
    }

    pub(crate) fn install_macro_path_inner(&mut self, proof: &crate::supervisor::AdmittedProof, id: u64, input: &PlanInput, path: JointPath) -> Result<f64, ArmError> {
        self.install_macro_path_inner_with_j7_lock(proof, id, input, path, false)
    }

    pub(crate) fn install_macro_path_inner_with_j7_lock(&mut self, proof: &crate::supervisor::AdmittedProof, id: u64, input: &PlanInput, path: JointPath, lock_j7_hold: bool) -> Result<f64, ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        if input.epoch != epoch { return Err(MotionError::PlanStale("epoch changed during the action".into()).into()); }
        self.check_plan_fresh(input)?;
        let duration = self.start_path_motion_at_inner(proof, input.epoch, &path)?;
        if input.power.is_some() { self.note_action_power_scale(id, path.power_scale); }
        let hold_j7 = if lock_j7_hold { self.config.gripper_index() } else { None };
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Path(PathTrajectory::new(path).with_hold_j7(hold_j7))));
        self.set_macro_sub_running(id);
        Ok(duration + 0.5)
    }

    pub(crate) fn check_gripper_close_torque_feedback(&self, request: &GripperRequest, policy: &crate::model::GripperPolicy, gripper_idx: usize) -> Result<(), ArmError> {
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
                let has_torque_fb = d.with_ref(|drv| drv.capabilities().torque_feedback).unwrap_or(false);
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
        if self.rt_executor_owned {
            return self.driver_gripper_close_speed_limit;
        }
        let gripper_idx = self.config.gripper_index()?;
        self.driver.as_ref()
            .and_then(|d| d.with_ref(|drv| drv.pos_force().gripper_pos_force_info()).flatten())
            .filter(|info| info.joint == gripper_idx)
            .map(|info| info.close_speed_rad_s)
            .filter(|v| v.is_finite() && *v > 0.0)
    }

    /// Estimated motor ceiling torque (N·m) of the attached position-force gripper: calibrated
    /// `ceiling_torque_nm` when available, else the feedback-equivalent estimate from
    /// `current_raw * 0.0019 N·m/raw`. Allocation-free and lock-free on `armd-control`.
    pub(crate) fn gripper_ceiling_torque_estimate(&self) -> Option<f32> {
        if self.rt_executor_owned {
            return self.driver_gripper_ceiling_torque_estimate;
        }
        let gripper_idx = self.config.gripper_index()?;
        self.driver.as_ref()
            .and_then(|d| d.with_ref(|drv| drv.pos_force().gripper_pos_force_info()).flatten())
            .filter(|info| info.joint == gripper_idx)
            .and_then(|info| info.ceiling_torque_estimate_nm)
            .filter(|v| v.is_finite() && *v > 0.0)
    }

    /// Unquantized effective closing speed (rad/s) for armd-planned closes (`grasp`, `place`,
    /// `sequence`, `gripper_calibrate`): `min(gripper_policy.close_speed_rad_s, joint max_velocity,
    /// damiao_gripper.close_speed_rad_s when pos_force)`. Feed this into `GripperRequest` so
    /// [`crate::motion::GripperMotion::try_with_close_speed_limit`] is the single wire-speed
    /// quantization point (avoiding `raw 29 -> 0.29_f64 -> raw 28` double flooring).
    pub(crate) fn unquantized_planned_close_speed(&self) -> f64 {
        let mut v = self.config.gripper_policy.as_ref()
            .map_or(f64::from(crate::model::DEFAULT_CLOSE_SPEED_RAD_S), |p| p.close_speed());
        if let Some(max_v) = self.config.gripper_index()
            .and_then(|g| self.config.limits.get(g))
            .map(|l| f64::from(l.max_velocity))
            .filter(|v| v.is_finite() && *v > 0.0)
        {
            v = v.min(max_v);
        }
        if let Some(cap) = self.gripper_close_speed_limit() {
            v = v.min(f64::from(cap));
        }
        v
    }

    /// Effective closing speed (rad/s) for armd-planned closes (`grasp`, `place`, `sequence`,
    /// `gripper_calibrate`): `min(gripper_policy.close_speed_rad_s, joint max_velocity,
    /// damiao_gripper.close_speed_rad_s when pos_force)`, quantized once downward to `0.01 rad/s`
    /// when `pos_force` is active (for reporting and duration estimates; never feed back into
    /// `GripperRequest`).
    pub fn effective_planned_close_speed(&self) -> f64 {
        let v = self.unquantized_planned_close_speed();
        if self.gripper_close_speed_limit().is_some() {
            let raw = crate::model::DamiaoGripperDrive::quantize_vel_raw_f64(v);
            if raw > 0 {
                return f64::from(raw) / 100.0;
            }
        }
        v
    }

    /// Snapshot of the active tool's free-motion torque baseline for a new gripper motion. A
    /// baseline of another tool (id or config digest) or follower is replaced by an empty one.
    pub fn gripper_baseline_for_motion(&mut self) -> Option<crate::gripper_baseline::FreeTorqueBaseline> {
        let policy = self.config.gripper_policy.as_ref()?;
        let tool = &self.config.active_tool;
        let follower = self.follower.as_deref();
        let lo = policy.closed_rad.min(policy.open_rad);
        let hi = policy.closed_rad.max(policy.open_rad);
        // same tool and follower, and the same stroke (a gripper calibration offset moves it)
        let matches = self.gripper_baseline.as_ref().is_some_and(|b| b.belongs_to(&tool.id, &tool.config_digest, follower)
            && (b.lo_rad - lo).abs() < 1e-3 && (b.hi_rad - hi).abs() < 1e-3);
        if !matches {
            if self.gripper_baseline.is_some() {
                tracing::info!("Gripper baseline belongs to another tool; starting unlearned for {}", tool.id);
                self.gripper_baseline_dirty = true;
            }
            self.gripper_baseline = Some(crate::gripper_baseline::FreeTorqueBaseline::new(
                policy.closed_rad, policy.open_rad, Some(tool.id.clone()), Some(tool.config_digest.clone()), self.follower.clone()));
        }
        self.gripper_baseline.clone()
    }

    /// Forget the learned free-motion torque (`/api/v1/action/gripper` `"baseline": "reset"`);
    /// the next unloaded moves learn it again.
    pub fn reset_gripper_baseline(&mut self) {
        self.gripper_baseline = None;
        self.gripper_last_contact = None;
        self.gripper_baseline_pending.clear();
        self.gripper_baseline_generation += 1;
        self.gripper_baseline_for_motion();
        self.gripper_baseline_dirty = true;
        tracing::info!("Gripper baseline reset");
    }

    /// Contact threshold of the learned baseline at jaw `q` moving toward `closing` (the closing
    /// sign of the policy or its opposite) at |v| `speed`; None when not learned there.
    pub fn gripper_baseline_threshold(&self, q: f32, closing: bool, speed: f32) -> Option<f32> {
        let b = self.gripper_baseline.as_ref()?;
        let cap = self.config.gripper_policy.as_ref().and_then(|p| p.max_close_torque_nm);
        let dir = if closing { crate::gripper_baseline::Direction::Close } else { crate::gripper_baseline::Direction::Open };
        b.estimate(q, dir, speed).map(|e| e.threshold_nm(cap))
    }

    /// `observation.gripper.baseline` (zero-allocation inline [`crate::model::HolderLabel`]).
    pub fn gripper_baseline_observation(&self) -> crate::model::GripperBaselineObs {
        let tool = &self.config.active_tool;
        let b = self.gripper_baseline.as_ref().filter(|b| b.belongs_to(&tool.id, &tool.config_digest, self.follower.as_deref()));
        crate::model::GripperBaselineObs {
            state: crate::model::HolderLabel::new(b.map_or("unlearned", |b| b.state())),
            close_moves: b.map_or(0, |b| b.close_moves),
            open_moves: b.map_or(0, |b| b.open_moves),
            rejected_moves: b.map_or(0, |b| b.rejected_moves),
            last_contact: self.gripper_last_contact,
        }
    }

    /// Start a gripper sub-motion of macro action `id`. Returns its expected duration.
    pub fn start_macro_gripper(admitted: Admitted<'_>, id: u64, request: GripperRequest) -> Result<f64, ArmError> {
        Self::start_macro_gripper_learning_within(admitted, id, request, crate::motion::LearnMode::Opening, None, false)
            .map(|(dur, _, _)| dur)
    }

    /// [`Self::start_macro_gripper`] recording the move for the baseline as `learn` allows
    /// (`gripper_calibrate` strokes: `LearnMode::Calibration`).
    pub fn start_macro_gripper_learning(admitted: Admitted<'_>, id: u64, request: GripperRequest, learn: crate::motion::LearnMode) -> Result<f64, ArmError> {
        Self::start_macro_gripper_learning_within(admitted, id, request, learn, None, false)
            .map(|(dur, _, _)| dur)
    }

    /// Start a gripper sub-motion of macro action `id`, verifying before installation that
    /// `max_budget_s` is positive/finite and (for pos-force closes) covers the planned duration.
    /// Returns `(expected_duration, timeout_s, is_pos_force_close)`.
    #[doc(hidden)]
    pub fn start_macro_gripper_within(admitted: Admitted<'_>, id: u64, request: GripperRequest, max_budget_s: Option<f64>, extend_pos_force_close_timeout: bool) -> Result<(f64, f64, bool), ArmError> {
        Self::start_macro_gripper_learning_within(
            admitted,
            id,
            request,
            crate::motion::LearnMode::Opening,
            max_budget_s,
            extend_pos_force_close_timeout,
        )
    }

    /// [`Self::start_macro_gripper_within`] with an explicit baseline [`crate::motion::LearnMode`].
    #[doc(hidden)]
    pub fn start_macro_gripper_learning_within(admitted: Admitted<'_>, id: u64, request: GripperRequest, learn: crate::motion::LearnMode, max_budget_s: Option<f64>, extend_pos_force_close_timeout: bool) -> Result<(f64, f64, bool), ArmError> {
        let (this, proof) = admitted.consume()?;
        this.start_macro_gripper_learning_within_inner(&proof, id, request, learn, max_budget_s, extend_pos_force_close_timeout)
    }

    pub(crate) fn start_macro_gripper_learning_within_inner(&mut self, _proof: &crate::supervisor::AdmittedProof, id: u64, request: GripperRequest, learn: crate::motion::LearnMode, max_budget_s: Option<f64>, extend_pos_force_close_timeout: bool) -> Result<(f64, f64, bool), ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        let gripper_idx = self.config.gripper_index().ok_or_else(|| {
            MotionError::Unsupported("Gripper actions need a profile with a gripper joint".into())
        })?;
        let Some(policy) = self.config.gripper_policy.clone() else {
            return Err(MotionError::Unsupported("Gripper actions need a profile with a gripper policy".into()).into());
        };
        if epoch != self.safety.epoch_id() { return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into()); }
        self.action_preconditions()?;
        self.check_pos_force_gripper_fresh()?;
        self.check_gripper_close_torque_feedback(&request, &policy, gripper_idx)?;
        let close_cap = self.gripper_close_speed_limit();
        let ceiling_est = self.gripper_ceiling_torque_estimate();
        let mut motion = GripperMotion::try_with_close_speed_limit(
            request, policy, self.config.limits[gripper_idx].clone(),
            self.commanded_positions[gripper_idx], gripper_idx, close_cap,
        )?.with_ceiling_torque_estimate(ceiling_est);
        let is_pos_force_close = close_cap.is_some() && motion.is_closing();
        if is_pos_force_close && extend_pos_force_close_timeout && motion.request.timeout_s.is_some() {
            motion.use_default_timeout();
        }
        let duration = motion.expected_duration();
        let timeout_s = motion.timeout_s;
        if let Some(budget_s) = max_budget_s {
            if !(budget_s.is_finite() && budget_s > 0.0) {
                return Err(MotionError::InvalidCommand(format!(
                    "remaining macro budget exhausted ({budget_s:.3} s <= 0)"
                )).into());
            }
            if is_pos_force_close && duration > budget_s {
                return Err(MotionError::InvalidCommand(format!(
                    "gripper motion needs {duration:.1} s (deadline {timeout_s:.1} s), exceeding remaining macro budget {budget_s:.1} s"
                )).into());
            }
        }
        self.safety.mark_motion_started()?;
        let baseline = self.gripper_baseline_for_motion();
        let motion = motion.with_baseline(baseline, learn);
        self.releasing = matches!(motion.request.command, GripperCommand::Open);
        self.gripper_hold_vel_ceiling = Some(motion.wire_velocity_ceiling());
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.clear_servo_ws_min_depths_rt();
        let mut target = self.commanded_positions.clone();
        target[gripper_idx] = motion.target;
        self.target_positions = target;
        self.last_heartbeat = Instant::now();
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Gripper(Box::new(motion))));
        self.set_macro_sub_running(id);
        Ok((duration, timeout_s, is_pos_force_close))
    }

    /// Start a joint-space quintic sub-motion of macro action `id` (sequence pose step) to
    /// `target`, planned with `limits` from the commanded state; it counts as done once settled.
    /// Returns its planned duration.
    pub fn install_macro_quintic(admitted: Admitted<'_>, id: u64, target: Vec<f32>, limits: &[crate::model::JointLimit]) -> Result<f64, ArmError> {
        let (this, proof) = admitted.consume()?;
        this.install_macro_quintic_inner(&proof, id, target, limits, None)
    }

    /// [`Self::install_macro_quintic`] reusing an off-lock [`crate::supervisor::PreparedQuintic`]
    /// when start, target, limits, workspace revision, tool offset and power budget still match,
    /// or installing an already-certified exact trajectory on accepted drift.
    pub(crate) fn install_macro_quintic_prepared_scoped(
        admitted: Admitted<'_>,
        id: u64,
        target: Vec<f32>,
        limits: &[crate::model::JointLimit],
        prepared: Option<crate::supervisor::PreparedQuintic>,
        exact_traj: Option<(crate::trajectory::MultiJointQuinticTrajectory, Option<f64>)>,
        scope: Option<crate::model::PlanningScope>,
    ) -> Result<f64, ArmError> {
        let (this, proof) = admitted.consume()?;
        this.install_macro_quintic_inner_scoped(&proof, id, target, limits, prepared, exact_traj, scope)
    }

    pub(crate) fn install_macro_quintic_inner(
        &mut self,
        proof: &crate::supervisor::AdmittedProof,
        id: u64,
        target: Vec<f32>,
        limits: &[crate::model::JointLimit],
        prepared: Option<crate::supervisor::PreparedQuintic>,
    ) -> Result<f64, ArmError> {
        self.install_macro_quintic_inner_scoped(proof, id, target, limits, prepared, None, None)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn install_macro_quintic_inner_scoped(
        &mut self,
        _proof: &crate::supervisor::AdmittedProof,
        id: u64,
        target: Vec<f32>,
        limits: &[crate::model::JointLimit],
        prepared: Option<crate::supervisor::PreparedQuintic>,
        exact_traj: Option<(crate::trajectory::MultiJointQuinticTrajectory, Option<f64>)>,
        scope: Option<crate::model::PlanningScope>,
    ) -> Result<f64, ArmError> {
        let epoch = self.macro_sub_ready(id)?;
        if epoch != self.safety.epoch_id() { return Err(MotionError::PlanStale("epoch changed during the action".into()).into()); }
        self.action_preconditions()?;
        self.gravity_comp_motion_busy()?;
        let mut target = target;
        self.safety.validate_motion_command(Some(epoch), &target, &self.config.limits)?;
        let moves_gripper = self.config.gripper_index().is_some_and(|g| (target[g] - self.commanded_positions[g]).abs() > 1e-4);
        if moves_gripper {
            self.check_pos_force_gripper_fresh()?;
        }
        self.torque_hold_filter(&mut target);
        self.power_hold_filter(&mut target);
        let reuse = prepared.filter(|p| {
            p.start == self.commanded_positions
                && p.v0 == self.commanded_velocities
                && p.target == target
                && p.limits.as_slice() == limits
                && p.workspace_revision == self.workspace_revision
                && p.tcp_offset == self.config.tcp_offset
                && p.power.as_ref() == self.power_budget()
                && p.checked_workspace == self.workspace_policy().is_some()
                && (!self.power.is_static_restricted() || p.static_restricted)
        });
        let (traj, power_scale) = if let Some(exact) = exact_traj {
            exact
        } else if let Some(p) = reuse {
            self.prepared_quintics_used += 1;
            (p.traj, p.power_scale)
        } else {
            let traj = TrajectoryPlanner::plan(&self.commanded_positions, &self.commanded_velocities, &target,
                &vec![0.0; self.config.dof], None, limits)?;
            let (traj, power_scale) = self.power_fit_quintic(traj, &self.commanded_positions, &self.commanded_velocities, &target, limits, true)?;
            if let Some(sc) = scope {
                crate::motion::check_quintic_planning_limits(&traj, &self.config.limits, sc)?;
            }
            let samples = self.workspace_samples(&traj)?;
            self.check_workspace_path(&self.commanded_positions, samples.iter().map(Vec::as_slice))?;
            (traj, power_scale)
        };
        if let Some(sc) = scope {
            crate::motion::check_quintic_planning_limits(&traj, &self.config.limits, sc)?;
        }
        if moves_gripper {
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
        self.set_macro_sub_running(id);
        Ok(duration + crate::motion::SETTLE_MAX_S)
    }

    /// End macro action `id` from its orchestrator (no-op when it already ended or is finishing).
    pub fn finish_macro(&mut self, id: u64, state: &str, code: Option<&str>, message: Option<String>, holding: Option<bool>, gap: Option<f32>) {
        if !self.macro_alive(id) { return; }
        self.finish_macro_inner(id, state, code, message, holding, gap);
    }

    /// Commits a macro terminal transition for `id` (shared by immediate finish and post-TX
    /// accepted `DeferredCompletionTarget::MacroFinish`, even when `m.finishing == true`).
    pub(crate) fn finish_macro_inner(&mut self, id: u64, state: &str, code: Option<&str>, message: Option<String>, holding: Option<bool>, gap: Option<f32>) {
        let still_running = self.macro_run.as_ref().is_some_and(|m| m.id == id)
            && self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running());
        if !still_running { return; }
        if self.rt_executor_owned {
            let _ = self.quiesce_macro_children_rt(id);
        } else if self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) {
            self.stop_motion();
        }
        if state == "done" { self.set_action_phase(id, "done", 1.0, None, None); }
        let rt_owned = self.rt_executor_owned;
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) {
            if let (Some(prev_phase), Some(phase_start)) = (m.phase.take(), m.phase_start.take()) {
                if !rt_owned {
                    let ms = Instant::now().duration_since(phase_start).as_millis() as u32;
                    if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                        *a.phase_ms.entry(prev_phase.as_str().to_owned()).or_insert(0) += ms;
                    }
                }
            }
        }
        self.finish_action(id, state, code, message, None, holding, gap);
    }

    /// **Single owner-side macro finishing state machine** shared across explicit `FinishMacro`
    /// and parent `deadline` expiry:
    /// - Enforces `m.finishing` idempotency (`PARENT_REPLACED` on duplicate finishes while bus hold
    ///   acceptance is pending).
    /// - Quiesces active child trajectory/settle/deferred `MacroSub` (preserving gripper measured hold).
    /// - Defers final terminal commit through `DeferredCompletionTarget::MacroFinish` whenever a
    ///   driver-backed RT owner quiesced active motion OR has an unaccepted replacement hold.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn enter_macro_finish_rt(
        &mut self,
        id: u64,
        state_str: &str,
        code: Option<&str>,
        msg_bounded: crate::motion::StepErrorText,
        legacy_msg: Option<String>,
        holding: Option<bool>,
        gripper_gap_rad: Option<f32>,
        clear_held: bool,
    ) -> Result<(), &'static str> {
        let Some(run) = self.macro_run.as_ref().filter(|m| m.id == id) else {
            return Err("PARENT_REPLACED");
        };
        if run.stop_requested || run.finishing
            || !self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running())
        {
            return Err("PARENT_REPLACED");
        }
        let prev_hold_pending = run.hold_pending_bus_accept;
        let child_active = self.macro_child_running(id);
        let quiesced_any = if child_active {
            self.quiesce_macro_children_rt(id)
        } else {
            false
        };
        let needs_bus_accept =
            self.is_rt_owner() && self.driver.is_some() && (quiesced_any || prev_hold_pending);
        if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) {
            m.finishing = true;
            if needs_bus_accept {
                m.hold_pending_bus_accept = true;
            }
        }
        if needs_bus_accept {
            self.pending_deferred_done = Some(super::DeferredActionDone {
                target: DeferredCompletionTarget::MacroFinish {
                    action_id: id,
                    state: crate::model::HolderLabel::new(state_str),
                    code: code.map(crate::model::HolderLabel::new),
                    message: msg_bounded,
                    clear_held,
                },
                epoch_id: self.safety.epoch_id(),
                lease_generation: self.active_lease.as_ref().map(|l| l.generation),
                settled: None,
                holding,
                gripper_gap_rad,
                clear_releasing: false,
                is_gripper: false,
            });
            return Ok(());
        }
        if clear_held {
            self.clear_held_rt();
        }
        let msg = if self.rt_executor_owned {
            if !msg_bounded.is_empty() {
                let mut buf = self.take_action_msg_buf(id, msg_bounded.as_str().len().max(160));
                buf.clear();
                buf.push_str(msg_bounded.as_str());
                Some(buf)
            } else {
                None
            }
        } else {
            legacy_msg.or_else(|| (!msg_bounded.is_empty()).then(|| msg_bounded.as_str().to_owned()))
        };
        self.finish_macro_inner(id, state_str, code, msg, holding, gripper_gap_rad);
        Ok(())
    }

    /// `/motion/stop`: ends the running macro action now, under the caller's lock, so its
    /// orchestrator cannot install another sub-motion before the next tick.
    pub fn request_macro_stop(&mut self) {
        let Some(id) = self.macro_run.as_ref().map(|m| m.id) else { return; };
        self.record_macro_terminal(id, "cancelled", Some("STOPPED"));
        let Some(mut m) = self.macro_run.take() else { return; };
        m.stop_requested = true;
        if self.rt_executor_owned {
            self.quiesce_macro_children_rt(id);
        } else if self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id)) {
            self.stop_motion();
        }
        let rt_owned = self.rt_executor_owned;
        if let (Some(prev_phase), Some(phase_start)) = (m.phase.take(), m.phase_start.take()) {
            if !rt_owned {
                let ms = Instant::now().duration_since(phase_start).as_millis() as u32;
                if let Some(a) = self.last_action.as_mut().filter(|a| a.id == id) {
                    *a.phase_ms.entry(prev_phase.as_str().to_owned()).or_insert(0) += ms;
                }
            }
        }
        self.retire_macro_run(m);
        let msg = if rt_owned {
            let mut buf = self.take_action_msg_buf(id, 32);
            buf.clear();
            buf.push_str("Motion stopped");
            Some(buf)
        } else {
            Some("Motion stopped".into())
        };
        self.finish_action(id, "cancelled", Some("STOPPED"), msg, None, None, None);
    }
}
