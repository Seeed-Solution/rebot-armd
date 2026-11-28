//! Observation publication, snapshot synthesis, schema caching, and read accessors.

#[allow(unused_imports)]
use super::*;

/// How often a reader that waits for the next sample looks again.
pub(super) const OBS_POLL: Duration = Duration::from_millis(1);

/// How long [`ArmSupervisor::observation`] waits for the read lock when no sample answers.
pub const OBS_LOCK_WAIT: Duration = Duration::from_millis(500);

/// 503 `OBSERVATION_UNAVAILABLE`: neither a fresh sample nor the read lock in time.
pub(super) fn observation_unavailable() -> ArmError {
    ArmError::Rejected { status: 503, code: "OBSERVATION_UNAVAILABLE".into(),
        message: "No current observation: the control tick has not published and the supervisor lock stayed busy".into() }
}

pub(super) fn owner_transferred() -> ArmError {
    ArmError::Rejected {
        status: 503,
        code: "OWNER_TRANSFERRED".into(),
        message: "ArmCore is owned by the synchronous RT executor; legacy lock path is disabled".into(),
    }
}

pub(super) fn executor_unavailable() -> ArmError {
    ArmError::Rejected {
        status: 503,
        code: "EXECUTOR_UNAVAILABLE".into(),
        message: "RT executor ownership is transitioning or unavailable".into(),
    }
}

/// A published sample, or the read guard when no sample answers (no control loop running).
enum Fresh<'a> {
    Sample(Arc<crate::rt_core::publish::Snapshot>),
    Locked(crate::supervisor_lock::SupReadGuard<'a, SupervisorInner>),
}


impl SupervisorInner {
    pub(super) fn gravity_output_enabled(&self) -> bool {
        self.gravity_compensated
            && self.config.dynamics_qualified
            && self.config.gravity_model.is_some()
            && self.config.dof >= self.config.arm_dof()
            && (self.driver.is_none()
                || self
                    .read_driver_for_tick()
                    .is_some_and(|drv| drv.capabilities().torque_feedforward))
    }

    /// Adds the time since the previous mark to phase `i` of the running tick (no-op
    /// outside `ArmSupervisor::tick`).
    pub(super) fn tick_phase(&mut self, i: usize) {
        if let Some(mark) = self.tick_phase_mark {
            let now = Instant::now();
            self.tick_phases_us[i] = self.tick_phases_us[i].saturating_add(crate::diagnostics::tick::us(now - mark));
            self.tick_phase_mark = Some(now);
        }
    }

    /// When `obs` (computed by [`Self::current_observation`]) was sampled and the absolute
    /// local deadline until which it stays correct with no write: the first clock-driven
    /// flip of the lease holder (`acquired_at + lease_timeout`), the previous-action window
    /// (`start_time + until`) or the 250 ms feedback freshness behind `feedback_valid` /
    /// `enable_state`, capped at `max_age` after the sample. Both come from the clocks the
    /// observation was computed from, so a later publication does not extend them.
    pub fn observation_valid_until(&self, obs: &ArmObservation, max_age: Duration) -> (Instant, Instant) {
        const FEEDBACK_FRESH: Duration = Duration::from_millis(250);
        let sampled = self.start_time + Duration::from_secs_f64(obs.timestamp.max(0.0));
        let mut until = sampled + max_age;
        if let Some(lease) = self.active_lease.as_ref() {
            let expires = lease.acquired_at + self.lease_timeout;
            if expires > sampled {
                until = until.min(expires);
            }
        }
        if let Some((_, end)) = self.previous_action.as_ref() {
            if obs.timestamp < *end { until = until.min(self.start_time + Duration::from_secs_f64(*end)); }
        }
        // ages are whole ms, truncated and read after `timestamp`: one more ms of margin
        if let Some(oldest) = obs.feedback_age_ms.iter().flatten().max() {
            let age = Duration::from_millis(oldest.saturating_add(1));
            if age <= FEEDBACK_FRESH { until = until.min(sampled + (FEEDBACK_FRESH - age)); }
        }
        (sampled, until.max(sampled))
    }

    pub(crate) fn begin_tick_phases(&mut self, start: Instant) {
        self.tick_phases_us = [0; 5];
        self.tick_phase_mark = Some(start);
    }

    pub(crate) fn take_tick_phases(&mut self) -> ([u32; 5], Option<Instant>) {
        (self.tick_phases_us, self.tick_phase_mark.take())
    }

    /// Updates `schema_revision` and publishes the matching `ObservationSchema` into `StateHub`'s
    /// off-thread cache. Called strictly off the RT control thread whenever schema inputs change.
    pub(crate) fn refresh_schema_cache(&mut self) {
        self.schema_revision = self.schema_revision.saturating_add(1).max(1);
        let schema = Arc::new(self.schema());
        self.core.hub().set_schema_cache(self.schema_revision, schema);
    }

    /// Zero-allocation, lock-free `Copy` snapshot of owner authority, lease, torque, busy, and
    /// planning inputs published alongside every observation on `StateHub`.
    pub(crate) fn owner_state_snapshot(&self, now: Instant) -> crate::rt_core::publish::OwnerStateSnapshot {
        let fenced_gen = self.effective_fenced_lease_generation();
        let active_conn_closed = self.active_lease_conn_id != 0
            && self.is_connection_closed(self.active_lease_conn_id);
        let active = if active_conn_closed {
            None
        } else {
            self.active_lease.as_ref().filter(|a| {
                a.generation > fenced_gen
                    && now.saturating_duration_since(a.acquired_at) < self.lease_timeout
            })
        };
        let (lease_generation, lease_owner, lease_owner_conn_id, lease_label, lease_expires_at) =
            match active {
                Some(a) => (
                    a.generation,
                    Some(crate::model::HolderLabel::new(&a.client_id)),
                    self.active_lease_conn_id,
                    Some(crate::model::HolderLabel::new(&a.label)),
                    Some(a.acquired_at + self.lease_timeout),
                ),
                None => (0, None, 0, None, None),
            };
        let (last_evicted_owner, last_evicted_generation) = match self.last_evicted.as_ref() {
            Some((conn, gen)) => (Some(crate::model::HolderLabel::new(conn)), *gen),
            None => (None, 0),
        };
        let mut commanded_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        for (dst, &src) in commanded_positions.iter_mut().zip(self.commanded_positions.iter()).take(dof) {
            *dst = src;
        }
        crate::rt_core::publish::OwnerStateSnapshot {
            arm: self.core.id(),
            boot: self.boot_id,
            control_frequency_hz: self.config.control_frequency_hz.max(1),
            lease_enabled: self.lease_enabled,
            lease_generation,
            lease_owner,
            lease_owner_conn_id,
            lease_label,
            lease_expires_at,
            last_evicted_owner,
            last_evicted_generation,
            fenced_lease_generation: fenced_gen,
            last_preemption: self.last_preemption,
            epoch: self.safety.epoch_id(),
            stop_generation: self.current_stop_gen(),
            config_revision: self.workspace_revision,
            parent_generation: self
                .last_action
                .as_ref()
                .filter(|a| a.is_running())
                .map_or(0, |a| a.id),
            schema_revision: self.schema_revision,
            is_torqued: self.is_torqued,
            disable_pending: self.disable_pending,
            torque_transition_pending: self.torque_transition.is_some() || self.core.transition_pending(),
            motion_active: self.active_trajectory.is_some(),
            handover_settling: self.handover_settling_since.is_some(),
            calibration_busy: self.calibration_busy || self.motor_zero_busy,
            config_inhibited: self.core.motion_inhibited_by_config(),
            e_stop_latched: self.e_stop_latched.load(Ordering::SeqCst),
            operator_auth_enforced: self.operator_auth_enforced,
            gravity_comp_active: self.gravity_comp.is_some(),
            safety_stopped: self.safety.state().is_stopped() || self.safety.has_latched_fault(),
            safety_recoverable: self.safety.recoverable(),
            dof: dof as u8,
            commanded_positions,
            residual_torques: self.residual_torques,
            has_zero_reference: self.config.zero_reference.is_some(),
            zero_convention: self.zero_convention,
        }
    }

    /// Synthesizes a coherent [`Snapshot`](crate::rt_core::publish::Snapshot) under a legacy read
    /// guard when no control loop is ticking (never called in executor mode).
    pub(crate) fn synthesize_legacy_snapshot(&self, generation: u64) -> crate::rt_core::publish::Snapshot {
        let obs = self.current_observation();
        let control_hz = self.config.control_frequency_hz.max(1);
        let (sampled_at, valid_until) = self.observation_valid_until(
            &obs,
            Duration::from_secs_f64(2.0 / f64::from(control_hz)),
        );
        let owner = self.owner_state_snapshot(sampled_at);
        let clock_now = self.last_clock_now.unwrap_or(sampled_at);
        let (wd_gen, wd_attempt) = match self.watchdog_park_pending {
            Some(p) if !p.waiting_mit_handback => (p.generation, p.attempt),
            _ => (0, 0),
        };
        let rt = crate::rt_core::publish::RtSample {
            positions: self.measured_positions.clone(),
            velocities: self.measured_velocities.clone(),
            commanded_positions: self.commanded_positions.clone(),
            commanded_velocities: self.commanded_velocities.clone(),
            feedback_instants: self.core.feedback().instants().to_vec(),
            feedback_seqs: self.core.feedback().seqs().to_vec(),
            feedback_queue_bounds: self.joint_feedback_queue_bounds.clone(),
            owner: self.published_owner_meta(),
            boot_id: self.boot_id,
            clock_now: Some(clock_now),
            workspace_revision: self.workspace_revision,
            motion_config_revision: self.motion_config_revision,
            stop_generation: self.stop_requests.as_ref().map_or_else(|| self.current_stop_gen(), |s| s.applied()),
            lease_generation: self.lease_valid_at(clock_now).map_or(0, |l| l.generation),
            config_inhibited: self.core.motion_inhibited_by_config(),
            calibration_busy: self.calibration_busy,
            holding_after_clear: self.holding_after_clear(),
            watchdog_park_pending_gen: wd_gen,
            watchdog_park_pending_attempt: wd_attempt,
            torque_backoff: self.torque_guard.backoff_history,
            power_diag: self.power.diagnostics_snapshot(),
        };
        crate::rt_core::publish::Snapshot {
            meta: crate::rt_core::publish::SampleMeta {
                seq: self.core.published_seq(),
                generation,
                sampled_at,
                valid_until,
            },
            obs,
            owner,
            rt,
            policy_diag_seq: 0,
            action_diag_seq: 0,
        }
    }

    /// Publishes an observation filled in place at `timestamp` with lock/release generation
    /// `generation` without re-running `tick_control`.
    pub fn publish_at(&mut self, timestamp: f64, generation: u64, keep: bool) -> TickPublished {
        self.refresh_gravity_comp_workspace_rt();
        let mut obs = self.core.take_observation_buffer();
        {
            let (obs_ref, spares) = obs.split_mut();
            self.fill_observation_with_spare(timestamp, obs_ref, Some(spares));
        }
        self.tick_phase(4);
        let body_end = self.tick_phase_mark.take();
        let control_hz = self.config.control_frequency_hz;
        let (sampled_at, valid_until) = self.observation_valid_until(&obs, Duration::from_secs_f64(2.0 / f64::from(control_hz.max(1))));
        let owner_state = self.owner_state_snapshot(sampled_at);
        let kept = keep.then(|| (*obs).clone());
        if let Some(ref hook) = self.before_publish_hook {
            hook();
        }
        let clock_now = self.last_clock_now.unwrap_or(sampled_at);
        let (wd_gen, wd_attempt) = match self.watchdog_park_pending {
            Some(p) if !p.waiting_mit_handback => (p.generation, p.attempt),
            _ => (0, 0),
        };
        let meta = crate::rt_core::publish::RtMotionMeta {
            boot_id: self.boot_id,
            clock_now: Some(clock_now),
            workspace_revision: self.workspace_revision,
            motion_config_revision: self.motion_config_revision,
            stop_generation: self.stop_requests.as_ref().map_or_else(|| self.current_stop_gen(), |s| s.applied()),
            lease_generation: self.lease_valid_at(clock_now).map_or(0, |l| l.generation),
            config_inhibited: self.core.motion_inhibited_by_config(),
            calibration_busy: self.calibration_busy,
            holding_after_clear: self.holding_after_clear(),
            watchdog_park_pending_gen: wd_gen,
            watchdog_park_pending_attempt: wd_attempt,
        };
        self.sync_motion_generation_for_publish();
        let owner = self.published_owner_meta();
        let next_pub_seq = self.core.next_publish_seq();
        let mid_csw = self
            .first_terminal_csw_mark
            .take()
            .unwrap_or_else(crate::diagnostics::tick::CswMark::now);
        self.last_mid_csw_start = Some(mid_csw);
        let mid_csw_start = Some(mid_csw);
        let (phase_terminal_us, captured_any) =
            self.capture_edge_terminals(timestamp, Some(next_pub_seq));
        let publish_start = Instant::now();
        let SupervisorInner {
            core,
            measured_positions,
            measured_velocities,
            commanded_positions,
            commanded_velocities,
            joint_feedback_queue_bounds,
            torque_guard,
            power,
            ..
        } = self;
        let power_diag = power.diagnostics_snapshot();
        core.publish_with_rt_meta(
            obs,
            generation,
            sampled_at,
            valid_until,
            owner_state,
            measured_positions,
            measured_velocities,
            commanded_positions,
            commanded_velocities,
            joint_feedback_queue_bounds,
            owner,
            meta,
            Some(&torque_guard.backoff_history),
            power_diag,
        );
        #[cfg(not(target_arch = "wasm32"))]
        let _ = self.flush_published_session_terminals();
        let publish_only_us = crate::diagnostics::tick::us(publish_start.elapsed());
        self.last_publish_only_us = publish_only_us;
        TickPublished {
            kept,
            body_end,
            phase_terminal_us,
            publish_only_us,
            captured_any,
            mid_csw_start,
        }
    }

    /// The control loop's tick under the write guard of lock generation `generation`:
    /// [`Self::tick_control`], the observation filled in place into the publication's back
    /// slot ([`Self::fill_observation`]), and its publication (before the guard drops: no
    /// other write can come in between). No allocation in a steady-state tick; `keep` copies
    /// the observation for the caller (tests).
    pub fn tick_publish(&mut self, dt: f64, generation: u64, keep: bool) -> TickPublished {
        self.tick_publish_with_deadline(dt, generation, keep, None)
    }

    pub(crate) fn tick_publish_with_deadline(
        &mut self,
        dt: f64,
        generation: u64,
        keep: bool,
        release_deadline: Option<Instant>,
    ) -> TickPublished {
        let mut sys_now = Instant::now;
        let timestamp = self.tick_control_inner(dt, release_deadline, &mut sys_now);
        // Keep `nonblocking_driver_release` active through `publish_at` whenever a legacy torque
        // transition ran or remains pending so observation filling never blocks on a contended
        // driver mutex (R4 Major 3).
        let keep_nb =
            self.has_non_broker_torque_transition() || self.last_tick_torque_ws_direction.is_some();
        self.nonblocking_driver_release = keep_nb;
        let published = self.publish_at(timestamp, generation, keep);
        self.nonblocking_driver_release = false;
        published
    }

    pub fn current_observation(&self) -> ArmObservation {
        let mut obs = ArmObservation::blank();
        self.fill_observation(self.start_time.elapsed().as_secs_f64(), &mut obs);
        obs
    }

    /// Pre-warms ONLY private `core`-owned RT scratch/history/spare buffers and constructs
    /// privately staged pre-sized observation slots `(template, front, back, mid)` WITHOUT
    /// touching shared `hub` state (so a timed-out startup loser never mutates `hub`).
    pub(crate) fn prewarm_rt_observation_spares(
        &mut self,
    ) -> (
        Box<ArmObservation>,
        ArmObservation,
        ArmObservation,
        ArmObservation,
    ) {
        let _ = Self::default_rebot_dm_resting_pose();
        let dof = self.config.dof;
        let arm_dof = self.config.arm_dof();
        self.still_anchor_positions.reserve(dof);
        self.table_still_anchor_positions.reserve(dof);
        self.table_still_first_rx_instants.reserve(dof);
        self.table_still_last_rx_instants.reserve(dof);
        self.joint_feedback_queue_bounds.reserve(dof);
        self.motor_errors.reserve(dof);
        self.gravity_torques.reserve(arm_dof);
        self.mit.last_ff.reserve(dof.max(crate::rt_core::mailbox::MAX_DOF));
        self.closed_connections.reserve_rt();
        if let Some(ref mut a) = self.active_lease {
            if a.client_id.capacity() < 64 {
                a.client_id.reserve(64 - a.client_id.len());
            }
            if a.label.capacity() < 128 {
                a.label.reserve(128 - a.label.len());
            }
            self.lease_client_spare = String::new();
            self.lease_label_spare = String::new();
        } else {
            if self.lease_client_spare.capacity() < 64 {
                self.lease_client_spare
                    .reserve(64 - self.lease_client_spare.len());
            }
            if self.lease_label_spare.capacity() < 128 {
                self.lease_label_spare
                    .reserve(128 - self.lease_label_spare.len());
            }
        }
        if let Some((ref mut c, _)) = self.last_evicted {
            if c.capacity() < 64 {
                c.reserve(64 - c.len());
            }
            self.evicted_conn_spare = String::new();
        } else if self.evicted_conn_spare.capacity() < 64 {
            self.evicted_conn_spare
                .reserve(64 - self.evicted_conn_spare.len());
        }
        self.driver_has_torque_feedback = self
            .driver
            .as_ref()
            .is_none_or(|d| d.with_ref(|drv| drv.capabilities().torque_feedback).unwrap_or(false));
        let pf_info = self
            .config
            .gripper_index()
            .and_then(|g| {
                self.driver
                    .as_ref()
                    .and_then(|d| d.with_ref(|drv| drv.pos_force().gripper_pos_force_info()).flatten())
                    .filter(|info| info.joint == g)
            });
        self.driver_gripper_close_speed_limit = pf_info
            .as_ref()
            .map(|info| info.close_speed_rad_s)
            .filter(|v| v.is_finite() && *v > 0.0);
        self.driver_gripper_ceiling_torque_estimate = pf_info
            .as_ref()
            .and_then(|info| info.ceiling_torque_estimate_nm)
            .filter(|v| v.is_finite() && *v > 0.0);
        if let Some(h) = self.held.as_mut() {
            h.label.reserve(128);
            self.held_label_spare = String::new();
        } else {
            self.held_label_spare.reserve(128);
        }
        self.prewarm_rt_gripper_spares();
        let tmpl = self.current_observation();
        let make_one = || {
            let mut obs = tmpl.clone();
            obs.ensure_rt_safety_text_spare();
            obs.gravity_torques.reserve(arm_dof);
            obs.motor_errors.reserve(dof);
            if let Some(gz) = obs.gripper_zero.as_mut() {
                // Pre-size for longest state ("calib_invalid", 13 bytes) so state transitions
                // ("ok" -> "checked" / "out_of_range" / "calib_invalid") never realloc on RT.
                if gz.state.capacity() < 16 {
                    gz.state.reserve(16 - gz.state.len());
                }
            }
            obs
        };
        (Box::new(make_one()), make_one(), make_one(), make_one())
    }

    /// Pre-warms RT-owned scratch/history buffers, static fallback policies, and empty
    /// triple-buffer observation slots for the RT executor before the first release runs
    /// (nonblocking under reader contention; leaves `published_seq` unchanged).
    #[doc(hidden)]
    pub fn prewarm_observation_slots(&mut self) {
        let (tmpl, front, back, mid) = self.prewarm_rt_observation_spares();
        self.core.hub().install_prewarm_template_and_front(tmpl, front);
        let _ = self.core.install_prewarmed_producer_slots(back, mid);
    }

    /// [`Self::current_observation`] at `timestamp` written into `out`, reusing its memory: the
    /// control tick fills its published sample this way, without allocating while no value
    /// changes its shape (armd 1.6 RT core). Every field is overwritten.
    pub fn fill_observation(&self, timestamp: f64, out: &mut ArmObservation) {
        self.fill_observation_with_spare(timestamp, out, None);
    }

    pub(super) fn fill_observation_with_spare(
        &self,
        timestamp: f64,
        out: &mut ArmObservation,
        mut spares: Option<&mut crate::rt_core::publish::ObservationSpares>,
    ) {
        use crate::rt_core::scratch::{fill_n, set_opt, set_opt_eq};
        let tcp = if self.config.kinematics.is_some() {
            self.config.tcp_pose_f32(&self.measured_positions).map(|pose| {
                let rpy = crate::kinematics::r_to_rpy(&pose.r);
                let quat = crate::kinematics::r_to_quat(&pose.r);
                crate::model::TcpState {
                    xyz_m: [pose.p[0] as f32, pose.p[1] as f32, pose.p[2] as f32],
                    rpy_rad: [rpy[0] as f32, rpy[1] as f32, rpy[2] as f32],
                    quat_wxyz: [quat[0] as f32, quat[1] as f32, quat[2] as f32, quat[3] as f32],
                }
            })
        } else {
            None
        };
        let gripper = if let (Some(idx), Some(policy)) = (self.config.gripper_index(), self.config.gripper_policy.as_ref()) {
            if let Some(&rad) = self.measured_positions.get(idx) {
                let span = policy.open_rad - policy.closed_rad;
                let fraction = if span.abs() < 1e-6 { 0.0 } else { ((rad - policy.closed_rad) / span).clamp(0.0, 1.0) };
                let opening_m = policy.jaw_opening.as_ref().map(|jo| jo.opening_m(rad));
                Some(crate::model::GripperObs {
                    joint: idx,
                    rad,
                    fraction,
                    opening_m,
                    stalled: self.gripper_stalled,
                    force_limited: self.gripper_stalled && self.gripper_force_capped,
                    force_limit_trips: self.gripper_force_trips,
                    baseline: Some(self.gripper_baseline_observation()),
                })
            } else {
                None
            }
        } else {
            None
        };
        let driver_capabilities = self.read_driver_for_tick().map(|drv| drv.capabilities()).unwrap_or_default();
        out.timestamp = timestamp;
        out.positions.clone_from(&self.measured_positions);
        out.velocities.clone_from(&self.measured_velocities);
        out.torques.clone_from(&self.torques);
        out.temperatures.clone_from(&self.temperatures);
        out.is_torqued = self.is_torqued;
        // `fault` reflects the most severe recorded kind and worst-of recoverability
        // (`FaultInfo` is stack-allocated `Copy`, 0 heap ops).
        let state = self.safety.state();
        if state.is_stopped() || self.safety.has_latched_fault() || out.fault.is_some() {
            out.fault = self.safety.fault();
            if let Some(ref mut fi) = out.fault {
                if fi.code == FaultCode::EnableFailed
                    && self.enable_lifecycle == EnableLifecycle::FailedHold
                {
                    self.measured_hold.write_fault_message(&mut fi.message);
                }
            }
        }
        out.set_safety_state(state);
        out.epoch_id = self.safety.epoch_id();
        out.gravity_compensated = self.gravity_output_enabled();
        out.gravity_torques.clone_from(&self.gravity_torques);
        out.driver_capabilities = driver_capabilities;
        out.enable_state = if self.driver.is_none() {
            if self.is_torqued { EnableState::Enabled } else { EnableState::Disabled }
        } else {
            self.read_driver_for_tick().map_or(EnableState::Unknown, |drv| drv.enable_state())
        };
        match self.read_driver_for_tick() {
            Some(drv) => drv.feedback_age_ms_into(&mut out.feedback_age_ms),
            None => fill_n(&mut out.feedback_age_ms, Some(0), self.config.dof),
        }
        out.disable_pending = self.disable_pending;
        out.feedback_valid = self.hardware_feedback_fresh(Duration::from_millis(250));
        out.commanded_positions.clone_from(&self.commanded_positions);
        let (action_spare, prev_action_spare) = match spares.as_mut() {
            Some(s) => (
                Some((&mut s.action_msg, &mut s.action_durability_warning)),
                Some((&mut s.prev_action_msg, &mut s.prev_action_durability_warning)),
            ),
            None => (None, None),
        };
        sync_action_obs(&mut out.action, self.last_action.as_ref(), action_spare);
        let prev_src = self
            .previous_action
            .as_ref()
            .filter(|(_, until)| timestamp < *until)
            .map(|(a, _)| a);
        sync_action_obs(&mut out.previous_action, prev_src, prev_action_spare);
        out.control_mode = if self.gravity_comp.is_some() { crate::model::ControlMode::GravityComp } else { crate::model::ControlMode::Position };
        self.gravity_comp_observation_into(&mut out.gravity_comp);
        match self.read_driver_for_tick() {
            Some(drv) => drv.joint_mode().with_range_check(&mut |rc| set_opt_eq(&mut out.driver_range_check, rc)),
            None => out.driver_range_check = None,
        }
        // inline label: setting or clearing it never allocates or frees on the tick
        let holder = self.lease_holder_str();
        if out.lease_holder.as_deref() != holder { out.lease_holder = holder.map(crate::model::HolderLabel::new); }
        out.motion_limit.clone_from(&self.motion_limit);
        #[cfg(not(target_arch = "wasm32"))]
        out.set_workspace_violation_rt(self.workspace_violation.as_ref());
        #[cfg(target_arch = "wasm32")]
        set_opt_eq(&mut out.workspace_violation, self.workspace_violation.as_ref());
        if out.motor_errors != self.motor_errors { out.motor_errors.clone_from(&self.motor_errors); }
        out.residual_torques = self.residual_torques;
        out.collision = self.collision_observation();
        self.power_observation_into(&mut out.power);
        self.gripper_zero_observation_into(&mut out.gripper_zero);
        out.tcp = tcp;
        out.gripper = gripper;
        if driver_capabilities.load_feedback && self.loads.len() == self.config.dof {
            set_opt(&mut out.loads, Some(&self.loads));
        } else {
            out.loads = None;
        }
        #[cfg(feature = "policy-runtime")]
        self.policy_observation_into_with_spares(&mut out.policy, spares);
        #[cfg(not(feature = "policy-runtime"))]
        { out.policy = None; }
        out.watchdog_park_blocked_reason = self.watchdog_park_blocked_reason;
        out.safety_log_dropped = self.safety.log_dropped().saturating_add(self.safety_log_ring_dropped);
        out.effective_owner = if self.rt_executor_owned {
            crate::model::EffectiveOwner::Executor
        } else {
            crate::model::EffectiveOwner::Legacy
        };
        out.owner_reason = self.owner_reason;
    }
}

impl ArmSupervisor {
    #[doc(hidden)]
    pub fn inner(&self) -> &Arc<SupervisorLock<SupervisorInner>> {
        &self.inner
    }

    #[doc(hidden)]
    pub fn hub(&self) -> &Arc<crate::rt_core::publish::StateHub> {
        &self.hub
    }

    /// Persistent lock-free [`PolicyAuthorization`] shared with `SupervisorInner` on `OWNER_EXECUTOR`.
    #[cfg(feature = "policy-runtime")]
    pub fn rt_policy_authorization(&self) -> crate::policy_runtime::PolicyAuthorization {
        self.rt_policy_auth.clone()
    }

    /// Exclusive-lifetime lock-free cancellation slots for executor policy start/run.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    pub fn policy_cancel_slots(&self) -> &Arc<crate::rt_core::requests::policy::PolicyCancelSlots> {
        &self.policy_cancel_slots
    }

    /// Off-thread coordinator for immutable RT policy terminal records and qualification fences.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    pub fn policy_qualification(
        &self,
    ) -> &Arc<crate::rt_core::requests::policy::PolicyQualificationCoordinator> {
        &self.policy_qualification
    }

    /// Bounded permit pool capping concurrent blocking policy loads/warmups and inferences.
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    pub fn policy_backend_permits(&self) -> &Arc<tokio::sync::Semaphore> {
        &self.policy_backend_permits
    }

    /// Lock-free `policy_live_allowed` query on `OWNER_EXECUTOR` (reads `PublishedConfig`),
    /// preserving the legacy read-lock path on `OWNER_LEGACY`.
    #[cfg(feature = "policy-runtime")]
    pub async fn policy_live_allowed_off_lock(&self) -> bool {
        if self.owner_mode() == OWNER_EXECUTOR {
            self.published_config
                .latest()
                .is_some_and(|c| c.policy_live_allowed)
        } else {
            self.inner.read().await.policy_live_allowed()
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[doc(hidden)]
    pub fn executor_recorder(&self) -> &Arc<crate::teach::ExecutorRecorder> {
        &self.executor_recorder
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn executor_base_gen(&self) -> u64 {
        self.executor_base_gen.load(Ordering::Acquire)
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn terminal_capture_channel_arc(&self) -> Arc<terminal_capture::TerminalCaptureChannel> {
        Arc::clone(&self.terminal_capture_channel)
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn update_sequence_rich_status_off_rt(
        &self,
        action_id: u64,
        f: impl FnOnce(&mut crate::motion::ActionStatus),
    ) {
        self.terminal_capture_channel
            .update_rich_action_off_rt(action_id, "sequence", f);
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn apply_sequence_overlay_off_rt(&self, obs: &mut ArmObservation) {
        if let Some(a) = obs.action.as_mut() {
            self.terminal_capture_channel.apply_sequence_overlay_off_rt(a);
        }
        if let Some(a) = obs.previous_action.as_mut() {
            self.terminal_capture_channel.apply_sequence_overlay_off_rt(a);
        }
    }

    /// The current observation, without the supervisor lock while the control loop runs
    /// (`GET /api/v1/observation`).
    pub async fn observation(&self) -> Result<ArmObservation, ArmError> {
        let mut obs = match self.fresh(true).await? {
            Fresh::Sample(s) => s.obs.clone(),
            Fresh::Locked(inner) => inner.current_observation(),
        };
        self.apply_sequence_overlay_off_rt(&mut obs);
        Ok(obs)
    }

    /// [`Self::observation`] for in-process callers and tests: when it is unavailable in Legacy
    /// mode, waits for the read lock without a bound; in Executor mode, routes through the RT owner
    /// (never reading the detached shell).
    pub async fn get_observation(&self) -> ArmObservation {
        match self.observation().await {
            Ok(obs) => obs,
            Err(_) => {
                if self.owner_mode() == OWNER_EXECUTOR {
                    if let Some(h) = self.executor_handle() {
                        if let Ok(mut obs) = h.observation_async().await {
                            self.hub.attach_action_measurement(&mut obs);
                            self.apply_sequence_overlay_off_rt(&mut obs);
                            self.hub.attach_action_diag(&mut obs);
                            return obs;
                        }
                    }
                    if let Some(s) = self.hub.latest() {
                        let mut obs = s.obs.clone();
                        self.apply_sequence_overlay_off_rt(&mut obs);
                        return obs;
                    }
                    return ArmObservation::blank();
                }
                let inner = self.inner.read().await;
                if self.check_legacy_under_guard(&inner).is_ok() {
                    return inner.current_observation();
                }
                drop(inner);
                if let Some(h) = self.executor_handle() {
                    if let Ok(mut obs) = h.observation_async().await {
                        self.hub.attach_action_measurement(&mut obs);
                        self.apply_sequence_overlay_off_rt(&mut obs);
                        self.hub.attach_action_diag(&mut obs);
                        return obs;
                    }
                }
                let mut obs = self.hub.latest().map_or_else(ArmObservation::blank, |s| s.obs.clone());
                self.apply_sequence_overlay_off_rt(&mut obs);
                obs
            }
        }
    }

    pub(super) fn wait_bound(&self) -> Duration { Duration::from_micros(2 * self.control_period_us.load(Ordering::Relaxed)) }

    pub(super) fn answers(s: &crate::rt_core::publish::Snapshot, want: u64, now: Instant) -> bool {
        s.meta.generation >= want && now <= s.meta.valid_until
    }

    async fn fresh(&self, count: bool) -> Result<Fresh<'_>, ArmError> {
        self.fresh_min_gen(count, None).await
    }

    async fn fresh_min_gen(&self, count: bool, min_gen: Option<u64>) -> Result<Fresh<'_>, ArmError> {
        use crate::rt_core::publish::Poll;
        let mode = self.owner_mode();
        if mode == OWNER_TRANSFERRING || mode == OWNER_FAILED {
            if count { self.obs_unavailable.fetch_add(1, Ordering::Relaxed); }
            return Err(observation_unavailable());
        }
        if mode == OWNER_EXECUTOR && self.executor_handle().is_none_or(|h| h.is_exited()) {
            if count { self.obs_unavailable.fetch_add(1, Ordering::Relaxed); }
            return Err(observation_unavailable());
        }
        let base_want = if mode == OWNER_EXECUTOR {
            self.executor_base_gen.load(Ordering::Acquire).saturating_add(1)
        } else {
            self.inner.generation()
        };
        let want = base_want.max(min_gen.unwrap_or(0));
        let started = Instant::now();
        let bound = self.wait_bound();
        let mut waited = false;
        loop {
            if mode == OWNER_EXECUTOR && self.executor_handle().is_none_or(|h| h.is_exited()) {
                if count { self.obs_unavailable.fetch_add(1, Ordering::Relaxed); }
                return Err(observation_unavailable());
            }
            let poll = self.hub.try_latest();
            let now = Instant::now();
            match poll {
                Poll::Ready(None) if mode != OWNER_EXECUTOR => break,
                Poll::Ready(None) => {}
                Poll::Ready(Some(s)) => {
                    if Self::answers(&s, want, now) {
                        if count {
                            self.obs_cache_hits.fetch_add(1, Ordering::Relaxed);
                            if waited { self.obs_waits.fetch_add(1, Ordering::Relaxed); }
                        }
                        return Ok(Fresh::Sample(s));
                    }
                    if mode != OWNER_EXECUTOR && now.saturating_duration_since(s.meta.sampled_at) > bound {
                        break;
                    }
                }
                Poll::Busy => {}
            }
            if now.duration_since(started) >= bound { break; }
            waited = true;
            tokio::time::sleep(OBS_POLL).await;
        }
        // Executor mode NEVER falls back to reading the detached shell under `self.inner`.
        if self.owner_mode() != OWNER_LEGACY {
            if self.owner_mode() == OWNER_EXECUTOR && self.executor_handle().is_some_and(|h| !h.is_exited()) {
                if let Poll::Ready(Some(s)) = self.hub.try_latest() {
                    if Self::answers(&s, want, Instant::now()) {
                        if count { self.obs_cache_hits.fetch_add(1, Ordering::Relaxed); }
                        return Ok(Fresh::Sample(s));
                    }
                }
            }
            if count { self.obs_unavailable.fetch_add(1, Ordering::Relaxed); }
            return Err(observation_unavailable());
        }
        let seen_seq = self.hub.published_seq();
        let lock_deadline = tokio::time::Instant::now() + OBS_LOCK_WAIT;
        let unavailable = || {
            if let Poll::Ready(Some(s)) = self.hub.try_latest() {
                if Self::answers(&s, want, Instant::now()) {
                    if count { self.obs_cache_hits.fetch_add(1, Ordering::Relaxed); }
                    return Ok(Fresh::Sample(s));
                }
            }
            if count { self.obs_unavailable.fetch_add(1, Ordering::Relaxed); }
            Err(observation_unavailable())
        };
        let Ok(inner) = tokio::time::timeout_at(lock_deadline, self.inner.read()).await else { return unavailable(); };
        if self.check_legacy_under_guard(&inner).is_err() {
            drop(inner);
            return unavailable();
        }
        if self.hub.published_seq() != seen_seq {
            drop(inner);
            if let Poll::Ready(Some(s)) = self.hub.try_latest() {
                if Self::answers(&s, want, Instant::now()) {
                    if count { self.obs_cache_hits.fetch_add(1, Ordering::Relaxed); }
                    return Ok(Fresh::Sample(s));
                }
            }
            let Ok(inner) = tokio::time::timeout_at(lock_deadline, self.inner.read()).await else { return unavailable(); };
            if self.check_legacy_under_guard(&inner).is_err() {
                drop(inner);
                return unavailable();
            }
            if count { self.obs_cache_misses.fetch_add(1, Ordering::Relaxed); }
            return Ok(Fresh::Locked(inner));
        }
        if count { self.obs_cache_misses.fetch_add(1, Ordering::Relaxed); }
        Ok(Fresh::Locked(inner))
    }

    /// Returns a coherent published [`Snapshot`](crate::rt_core::publish::Snapshot) at least as
    /// fresh as `min_gen` (in executor mode strictly from `StateHub` with zero `SupervisorLock`
    /// reads, returning `503 OBSERVATION_UNAVAILABLE` if unavailable/stale/exited; in legacy mode
    /// from `StateHub` or synthesized under a short read guard when no control loop runs).
    pub async fn fresh_snapshot(
        &self,
        min_gen: Option<u64>,
    ) -> Result<Arc<crate::rt_core::publish::Snapshot>, ArmError> {
        match self.fresh_min_gen(false, min_gen).await? {
            Fresh::Sample(s) => Ok(s),
            Fresh::Locked(inner) => {
                let gen = self.inner.generation();
                Ok(Arc::new(inner.synthesize_legacy_snapshot(gen)))
            }
        }
    }

    /// Reads a coherent [`ServiceSnapshot`](crate::rt_core::publish::ServiceSnapshot) respecting
    /// the active owner boundary:
    /// - `OWNER_EXECUTOR`: reads strictly from [`StateHub`] (zero `SupervisorLock`), rejecting
    ///   exited executors and pre-handoff cached samples (`min_generation >= executor_base_gen + 1`).
    /// - `OWNER_LEGACY`: serves from [`StateHub`] when a live control loop is publishing, or
    ///   synthesizes a single-guard coherent snapshot under a short read lock (rechecking
    ///   detachment) when no ticker runs.
    /// - `OWNER_TRANSFERRING` / `OWNER_FAILED`: fails closed with `503 EXECUTOR_UNAVAILABLE`.
    pub async fn read_services(
        &self,
        mut req: crate::rt_core::publish::ReadRequirements,
    ) -> Result<Arc<crate::rt_core::publish::ServiceSnapshot>, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                if self.executor_handle().is_none_or(|h| h.is_exited()) {
                    return Err(observation_unavailable());
                }
                let base_want = self.executor_base_gen.load(Ordering::Acquire).saturating_add(1);
                req.min_generation = Some(req.min_generation.map_or(base_want, |g| g.max(base_want)));
                self.hub.read_services(req).map_err(ArmError::from)
            }
            OWNER_LEGACY => {
                let legacy_gen = self.inner.generation();
                let mut cached_req = req;
                cached_req.min_generation = Some(
                    cached_req
                        .min_generation
                        .map_or(legacy_gen, |g| g.max(legacy_gen)),
                );
                if let Ok(svc) = self.hub.read_services(cached_req) {
                    if self.owner_mode() == OWNER_LEGACY {
                        return Ok(svc);
                    }
                }
                let lock_deadline = tokio::time::Instant::now() + OBS_LOCK_WAIT;
                let inner = tokio::time::timeout_at(lock_deadline, self.inner.read())
                    .await
                    .map_err(|_| observation_unavailable())?;
                self.check_legacy_under_guard(&inner)?;
                let gen = self.inner.generation();
                let snap = Arc::new(inner.synthesize_legacy_snapshot(gen));
                let identity = Arc::new(crate::rt_core::publish::ServiceIdentity::from_inner(&inner));
                let prep_env = req.require_prep_env.then(|| {
                    Arc::new(crate::rt_core::requests::actions::ActionPrepEnv::from_inner(&inner))
                });
                drop(inner);
                crate::rt_core::publish::StateHub::build_service_snapshot_from(
                    snap,
                    identity,
                    prep_env,
                    req,
                    Instant::now(),
                )
                .map_err(ArmError::from)
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Coherent control frequency for WS upgrade (`?obs_hz=...`).
    /// In executor mode, requires a fresh `StateHub` snapshot (`503 OBSERVATION_UNAVAILABLE` if
    /// unavailable); in legacy mode, serves from `StateHub` or the immutable configured value.
    pub async fn control_frequency_hz_checked(&self) -> Result<u32, ArmError> {
        if self.owner_mode() == OWNER_LEGACY {
            if let Some(s) = self.running_sample() {
                return Ok(s.owner.control_frequency_hz.max(1));
            }
            return Ok(self.control_frequency_hz.max(1));
        }
        let snap = self.fresh_snapshot(None).await?;
        Ok(snap.owner.control_frequency_hz.max(1))
    }

    /// Coherent observation schema served from `StateHub`'s off-thread cache paired by
    /// `OwnerStateSnapshot::schema_revision` (returns explicit `503 OBSERVATION_UNAVAILABLE` in
    /// executor mode when no fresh snapshot or matching cached schema is available, and overlays
    /// live `zero_convention` from `OwnerStateSnapshot` without RT allocations).
    pub async fn schema(&self) -> Result<ObservationSchema, ArmError> {
        if self.owner_mode() == OWNER_LEGACY {
            let inner = self.inner.read().await;
            if self.check_legacy_under_guard(&inner).is_ok() {
                return Ok(inner.schema());
            }
        }
        match self.fresh_min_gen(false, None).await? {
            Fresh::Sample(s) => {
                if let Some(cached) = self.hub.cached_schema(s.owner.schema_revision) {
                    let mut schema = (*cached).clone();
                    schema.zero_convention = if s.owner.has_zero_reference {
                        serde_json::to_value(s.owner.zero_convention).ok()
                    } else {
                        None
                    };
                    return Ok(schema);
                }
                Err(observation_unavailable())
            }
            Fresh::Locked(inner) => Ok(inner.schema()),
        }
    }

    /// Coherent lease activity check for REST routes (`LEASE_ACTIVE` vs `LEASE_REQUIRED`),
    /// returning explicit `Err(503 OBSERVATION_UNAVAILABLE)` when no fresh snapshot is available.
    pub async fn lease_active_checked(&self) -> Result<bool, ArmError> {
        match self.fresh_min_gen(false, None).await? {
            Fresh::Sample(s) => Ok(s.owner.is_lease_active_at(Instant::now())),
            Fresh::Locked(inner) => Ok(inner.is_lease_active()),
        }
    }

    pub(super) fn running_sample(&self) -> Option<Arc<crate::rt_core::publish::Snapshot>> {
        self.hub.latest().filter(|s| s.meta.sampled_at.elapsed() <= self.wait_bound())
    }

    pub async fn arm_feedback_snapshot(&self, arm: &[usize]) -> (Vec<f32>, Vec<f32>, Vec<Option<Instant>>, Vec<u64>) {
        if let Some(s) = self.running_sample() {
            return s.rt.arm_feedback(arm);
        }
        if self.owner_mode() != OWNER_LEGACY {
            return self
                .hub
                .latest()
                .map_or_else(|| (Vec::new(), Vec::new(), Vec::new(), Vec::new()), |s| s.rt.arm_feedback(arm));
        }
        let inner = self.inner.read().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            return inner.arm_feedback_snapshot();
        }
        drop(inner);
        self.hub
            .latest()
            .map_or_else(|| (Vec::new(), Vec::new(), Vec::new(), Vec::new()), |s| s.rt.arm_feedback(arm))
    }

    pub async fn measured_positions(&self) -> Vec<f32> {
        if let Some(s) = self.running_sample() {
            return s.rt.positions.clone();
        }
        if self.owner_mode() != OWNER_LEGACY {
            return self.hub.latest().map_or_else(Vec::new, |s| s.rt.positions.clone());
        }
        let inner = self.inner.read().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            return inner.measured_positions.clone();
        }
        drop(inner);
        self.hub.latest().map_or_else(Vec::new, |s| s.rt.positions.clone())
    }

    /// Scheduling of the control thread, reported by it after its real-time request.
    pub fn set_control_thread_sched(&self, sched: crate::diagnostics::tick::ControlThreadSched) {
        *self.control_sched.lock().unwrap_or_else(|e| e.into_inner()) = Some(sched);
    }

    pub async fn get_schema(&self) -> ObservationSchema {
        if let Ok(schema) = self.schema().await {
            return schema;
        }
        if self.owner_mode() != OWNER_LEGACY {
            if let Some(cached) = self.hub.latest_cached_schema() {
                return (*cached).clone();
            }
        }
        self.inner.read().await.schema()
    }

    /// Records the `--follower` model name for `GET /api/v1/safety/describe`.
    pub async fn set_follower(&self, name: &str) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.follower = Some(name.to_string());
            self.hub.set_service_identity(Arc::new(
                crate::rt_core::publish::ServiceIdentity::from_inner(&inner),
            ));
        }
    }

    /// Attaches (or clears) the startup-resolved multi-arm cell context before control-loop
    /// ownership transfers to the RT executor.
    pub async fn set_cell(
        &self,
        cell: Option<Arc<crate::cell::CellContext>>,
    ) -> Result<(), ArmError> {
        let mut inner = self.inner.write().await;
        self.check_legacy_under_guard(&inner)?;
        inner.cell = cell.clone();
        inner.refresh_schema_cache();
        *self.cell.write().unwrap_or_else(|e| e.into_inner()) = cell;
        let snap = crate::rt_core::requests::config::PublishedConfig::from_inner(&inner);
        let gen = self.inner.generation();
        self.published_config.publish(snap, gen);
        Ok(())
    }

    /// Returns the active startup-resolved cell context, or `None` in single-arm mode.
    pub fn cell_context(&self) -> Option<Arc<crate::cell::CellContext>> {
        self.cell.read().unwrap_or_else(|e| e.into_inner()).clone()
    }

    /// Read-only status snapshot for `GET /api/v1/cell/status` (off-RT, zero supervisor lock).
    pub fn cell_status(&self) -> crate::cell::CellStatusResponse {
        match self.cell_context() {
            Some(ctx) => ctx.status_response(),
            None => crate::cell::CellStatusResponse::disabled(),
        }
    }

    pub async fn get_config(&self) -> ArmConfig {
        let inner = self.inner.read().await;
        inner.config.clone()
    }

    pub async fn try_set_driver(&self, mut driver: Box<dyn ArmDriver>) -> Result<(), ArmError> {
        // Acquire the legacy shell lock and verify authoritative writer/disarm state BEFORE
        // performing ANY I/O on the incoming driver (DRV-F1 B1/B3).
        let mut inner = self.inner.write().await;
        self.check_legacy_under_guard(&inner)?;
        #[cfg(not(target_arch = "wasm32"))]
        if inner.driver.is_some() && !crate::rt_core::executor::core_is_confirmed_disarmed(&inner) {
            return Err(ArmError::Rejected {
                status: 409,
                code: "DRIVER_ENERGIZED".into(),
                message: "Cannot replace an energized or unconfirmed hardware driver".into(),
            });
        }
        driver.set_estop_state(self.estop_state.clone());
        driver.set_estop_gate(inner.estop_gate());
        driver.set_gripper_index(inner.config.gripper_index());
        let mut drv: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &inner.joint_zero_offsets));
        drv.set_stopped_disable_gate(inner.stopped_disable_gate());
        let caps = drv.capabilities();
        if caps.load_feedback {
            inner.loads = vec![f32::NAN; inner.config.dof];
        }
        if let Ok(states) = drv.read_joint_states() {
            let loads = drv.joint_loads_snapshot();
            if !states.is_empty() && drv.is_feedback_valid() && SupervisorInner::states_valid(&states, caps, inner.config.dof, loads.as_deref()) {
                inner.sync_driver_feedback(&states, loads.as_deref());
                for (i, s) in states.iter().enumerate().take(inner.config.dof) {
                    if s.position.is_finite() {
                        inner.commanded_positions[i] = s.position;
                        inner.target_positions[i] = s.position;
                    }
                }
            }
        }
        inner.driver_caps = caps;
        inner.apply_driver_limits(drv.as_ref());
        inner.install_driver(drv);
        inner.bump_motion_config_revision();
        inner.refresh_schema_cache();
        let motion_cfg = Arc::new(crate::rt_core::requests::motion::MotionConfigSnapshot::from_supervisor(&inner));
        drop(inner);
        self.hub.set_legacy_motion_config_if_newer(motion_cfg);
        Ok(())
    }

    pub async fn set_driver(&self, driver: Box<dyn ArmDriver>) {
        let _ = self.try_set_driver(driver).await;
    }

    pub fn with_driver(self, mut driver: Box<dyn ArmDriver>) -> Self {
        let mut motion_cfg = None;
        if let Ok(mut inner) = self.inner.try_write() {
            #[cfg(not(target_arch = "wasm32"))]
            let replaceable = inner.driver.is_none()
                || crate::rt_core::executor::core_is_confirmed_disarmed(&inner);
            #[cfg(target_arch = "wasm32")]
            let replaceable = inner.driver.is_none();
            if self.check_legacy_under_guard(&inner).is_ok() && replaceable {
                driver.set_estop_state(self.estop_state.clone());
                driver.set_estop_gate(inner.estop_gate());
                driver.set_gripper_index(inner.config.gripper_index());
                driver.joint_mode_mut().set_strict_mode_handover(inner.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled));
                let caps = driver.capabilities();
                if caps.load_feedback {
                    inner.loads = vec![f32::NAN; inner.config.dof];
                }
                let driver: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &inner.joint_zero_offsets));
                inner.driver_caps = driver.capabilities();
                inner.apply_driver_limits(driver.as_ref());
                inner.install_driver(driver);
                inner.bump_motion_config_revision();
                inner.refresh_schema_cache();
                motion_cfg = Some(Arc::new(
                    crate::rt_core::requests::motion::MotionConfigSnapshot::from_supervisor(&inner),
                ));
            }
        }
        if let Some(mc) = motion_cfg {
            self.hub.set_legacy_motion_config_if_newer(mc);
        }
        self
    }

    pub async fn enable_torque(&self, stamp: AdmissionStamp) -> Result<SafetyState, ArmError> {
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        let mut inner = self.inner.clone().write_owned().await;
        self.check_legacy_under_guard(&inner)?;
        // Post-lock double-check: verify atomic e-stop latch again after acquiring write lock
        if inner.estop_pending() {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        tokio::task::spawn_blocking(move || {
            let admitted = inner.check_admission(stamp)?;
            SupervisorInner::enable_torque(admitted)
        })
        .await
        .map_err(|e| ArmError::Internal(format!("enable_torque blocking task failed: {e}")))?
        .map_err(ArmError::Safety)
    }

    /// Exempt from `Admitted` (Invariant 1c): torque disable is a stop-direction action that
    /// removes actuator output and must never be blocked by a Stop or admission check.
    pub async fn disable_torque(&self) -> Result<SafetyState, ArmError> {
        let mut inner = self.inner.clone().write_owned().await;
        self.check_legacy_under_guard(&inner)?;
        tokio::task::spawn_blocking(move || inner.disable_torque())
            .await
            .map_err(|e| ArmError::Internal(format!("disable_torque blocking task failed: {e}")))?
            .map_err(ArmError::Safety)
    }

    pub async fn heartbeat(&self, client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return Err(SafetyError::OwnerTransferred);
        }
        inner.heartbeat(client_id, epoch_id)
    }

    pub async fn acquire_lease(&self, client_id: &str) -> Result<u64, (String, u64)> {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return Err(("OWNER_TRANSFERRED".into(), 0));
        }
        inner.acquire_lease(client_id)
    }

    pub async fn preempt_lease(&self, conn: &str, label: &str) -> Result<(u64, Option<(String, u64)>), PreemptError> {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return Err(PreemptError::OwnerTransferred);
        }
        inner.preempt_lease(conn, label)
    }

    pub async fn is_handover_settling(&self) -> bool {
        let inner = self.inner.read().await;
        self.check_legacy_under_guard(&inner).is_ok() && inner.handover_settling_since.is_some()
    }

    pub async fn release_lease(&self, client_id: &str, generation: u64) -> bool {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_err() {
            return false;
        }
        inner.release_lease(client_id, generation)
    }
}
