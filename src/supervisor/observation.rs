//! Observation publication, snapshot synthesis, schema caching, and read accessors.

#[allow(unused_imports)]
use super::*;

// scaffold-extra-0
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

// scaffold-extra-1
impl ArmSupervisor {
}
