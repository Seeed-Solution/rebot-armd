//! Admission stamps, single-use proof tokens, connection registries, and stop/gate helpers.

#[allow(unused_imports)]
use super::*;

// scaffold-impl-0
impl AdmissionStamp {
}

// scaffold-impl-1
impl std::fmt::Debug for Admitted<'_> {
}

// scaffold-impl-2
impl AdmittedProof {
}

// scaffold-impl-3
impl<'g> Admitted<'g> {
}

// scaffold-impl-4
impl MotionStartKind {
}

// scaffold-impl-5
impl ArmError {
}

// scaffold-impl-6
impl From<crate::recording::StoreError> for ArmError {
}

// scaffold-impl-7
impl Default for LiveConnections {
}

// scaffold-impl-8
impl LiveConnections {
}

// scaffold-impl-9
impl Default for ClosedConnections {
}

// scaffold-impl-10
impl ClosedConnections {
}

// scaffold-impl-11
impl<'a> LiveDisconnectFenceView<'a> {
}

// scaffold-impl-12
impl ConsumedStamps {
}

// scaffold-extra-0
impl SupervisorInner {
    /// Advance the stop generation under the owner lock whenever a Stop applies.
    pub fn advance_stop_generation(&mut self) -> u64 {
        self.reset_hold_tx_joint_progress();
        self.j7_admitted_stamp = None;
        self.stop_gen.fetch_add(1, Ordering::SeqCst) + 1
    }

    /// Monotonically synchronize `stop_gen` with an RT executor's `CommitCtx::stop_generation`.
    pub(crate) fn sync_stop_generation(&mut self, gen: u64) {
        let _ = self.stop_gen.fetch_max(gen, Ordering::SeqCst);
    }

    /// Keep `lease_gen_mirror` synchronized with `active_lease` under the owner lock.
    pub(crate) fn sync_lease_gen_mirror(&self) {
        self.lease_gen_mirror.store(self.current_active_lease_gen(), Ordering::SeqCst);
    }

    pub(super) fn verify_admission_stamp(&self, stamp: AdmissionStamp) -> Result<(), SafetyError> {
        if stamp.connection_id != 0 && self.is_connection_closed(stamp.connection_id) {
            return Err(SafetyError::LeaseLost);
        }
        if stamp.stop_gen != self.current_stop_gen() {
            return Err(SafetyError::Stopped);
        }
        let active_gen = self.current_active_lease_gen();
        if stamp.lease_gen != active_gen {
            return Err(SafetyError::LeaseLost);
        }
        let current_epoch = self.safety.epoch_id();
        if stamp.estop_epoch != current_epoch {
            if self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop)
            {
                return Err(SafetyError::EStopActive);
            }
            return Err(SafetyError::EpochMismatch {
                expected: current_epoch,
                received: stamp.estop_epoch,
            });
        }
        if self.internal_fault_lane.is_fenced() {
            return Err(SafetyError::EStopActive);
        }
        if self.e_stop_latched.load(Ordering::SeqCst)
            || matches!(self.safety.state(), SafetyState::EStop)
        {
            return Err(SafetyError::EStopActive);
        }
        Ok(())
    }

    pub(crate) fn is_stamp_consumed(&self, request_id: u64) -> bool {
        if request_id == 0 {
            return true;
        }
        if (request_id & BROKER_REQUEST_ID_BIT) != 0 {
            self.consumed_broker_stamps
                .contains(request_id & !BROKER_REQUEST_ID_BIT)
        } else if (request_id & RT_INTERNAL_REQUEST_ID_BIT) != 0 {
            self.consumed_internal_stamps
                .contains(request_id & !RT_INTERNAL_REQUEST_ID_BIT)
        } else {
            self.consumed_stamps.contains(request_id)
        }
    }

    pub(crate) fn record_consumed_stamp(&mut self, request_id: u64) {
        if request_id == 0 {
            return;
        }
        if (request_id & BROKER_REQUEST_ID_BIT) != 0 {
            self.consumed_broker_stamps
                .record(request_id & !BROKER_REQUEST_ID_BIT);
        } else if (request_id & RT_INTERNAL_REQUEST_ID_BIT) != 0 {
            self.consumed_internal_stamps
                .record(request_id & !RT_INTERNAL_REQUEST_ID_BIT);
        } else {
            self.consumed_stamps.record(request_id);
        }
    }

    /// Read-only preflight admission check used before off-lock planning or non-installing
    /// validation: verifies authority and confirms `stamp.request_id` is non-zero and not yet
    /// consumed, WITHOUT consuming it.
    pub fn preflight_admission(&self, stamp: AdmissionStamp) -> Result<(), SafetyError> {
        if self.is_stamp_consumed(stamp.request_id) {
            return Err(SafetyError::StampAlreadyConsumed);
        }
        self.verify_admission_stamp(stamp)
    }

    /// Single owner-lock motion/output admission gate: the SOLE constructor of [`Admitted`].
    /// Compares the caller's admission-captured [`AdmissionStamp`] against current owner state
    /// without re-sampling, consumes its single-use [`AdmissionStamp::request_id`] under the
    /// owner lock, and returns an [`Admitted<'g>`] token borrowing the owner guard.
    pub fn check_admission<'g>(
        &'g mut self,
        stamp: AdmissionStamp,
    ) -> Result<Admitted<'g>, SafetyError> {
        if self.is_stamp_consumed(stamp.request_id) {
            return Err(SafetyError::StampAlreadyConsumed);
        }
        self.verify_admission_stamp(stamp)?;
        self.record_consumed_stamp(stamp.request_id);
        Ok(Admitted {
            inner: self,
            stamp,
            _not_send_sync: std::marker::PhantomData,
        })
    }

    /// Unified motion-start wrapper: requires an [`Admitted`] token by value under the owner lock.
    pub fn start_motion<Op: MotionStartOp>(admitted: Admitted<'_>, op: Op) -> Result<Op::Output, ArmError> {
        let verify_res = admitted.verify_live();
        #[cfg(not(target_arch = "wasm32"))]
        {
            let inner = &mut *admitted.inner;
            let dof = inner.config.dof.min(crate::torque::MAX_CHANNELS) as u8;
            let live_refusal = match &verify_res {
                Ok(_) => None,
                Err(SafetyError::EStopActive) => Some(crate::torque::shadow::LiveRefusalKind::EStopActive),
                Err(SafetyError::FaultActive(_)) => Some(crate::torque::shadow::LiveRefusalKind::FaultLatched),
                Err(_) => Some(crate::torque::shadow::LiveRefusalKind::Other),
            };
            inner
                .torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::MotionCommand {
                    channels: crate::torque::ChannelMask::from_count(dof),
                    live_allowed: verify_res.is_ok(),
                    live_refusal,
                    live_intent: inner.shadow_live_intent(),
                    live_is_torqued: inner.is_torqued,
                    live_stopped: inner.safety.state().is_stopped(),
                });
        }
        verify_res?;
        op.execute(admitted)
    }

    /// Observation schema: profile capabilities plus the parts that depend on the driver.
    pub fn schema(&self) -> ObservationSchema {
        let mut schema = ObservationSchema::from_config(&self.config);
        schema.capabilities.gravity_comp = self.gravity_comp_support().is_ok();
        let torque_feedback = self.driver.is_some() && self.driver_has_torque_feedback;
        schema.capabilities.collision_guard &= torque_feedback;
        schema.capabilities.torque_guard &= torque_feedback;
        schema.capabilities.power_budget &= torque_feedback;
        schema.capabilities.auto_yaw = self.config.arm_dof() == 5;
        #[cfg(feature = "policy-runtime")]
        if let Some(ref mut pr) = schema.capabilities.policy_runtime {
            pr.modes = if self.policy_live_allowed() {
                vec!["shadow".to_string(), "live_sim".to_string()]
            } else {
                vec!["shadow".to_string(), "live_gated".to_string()]
            };
        }
        if schema.feetech_joints.is_none() {
            schema.feetech_joints = self.config.feetech.as_ref().map(|f| f.joints.clone());
        }
        schema.capabilities.calibration_tools = Some(self.calibration_tools_with(&schema.capabilities));
        if let Some(t) = schema.capabilities.teach.as_mut() { t.episodes = self.episodes.enabled(); }
        schema.capabilities.cell = self.cell.as_ref().map(|c| c.capabilities());
        schema.joint_zero = self.calibration.as_ref().and_then(|c| c.joint_zero.as_ref()).and_then(|z| serde_json::to_value(z).ok());
        schema.zero_convention = self.config.zero_reference.as_ref().and_then(|_| serde_json::to_value(self.zero_convention).ok());
        schema
    }

    /// The E-stop and lease-fence gate of this arm's bus driver: a request not yet applied by the
    /// tick (plus authorized-stop generations when attached to an RT executor, and the lock-free
    /// fence-first disconnect lease atomics).
    pub fn estop_gate(&self) -> crate::rt_core::urgent::EstopGate {
        let gate = crate::rt_core::urgent::EstopGate::from_state(
            self.estop_state.clone(),
            self.estop_applied.clone(),
        )
        .with_owner_stop_gen(self.stop_gen.clone())
        .with_lease_gate(crate::rt_core::urgent::LeaseGate::new(
            self.lease_gen_mirror.clone(),
            self.atomic_fenced_lease_gen.clone(),
            self.pending_disconnect_hold.clone(),
        ));
        match self.stop_requests.as_ref() {
            Some(stops) => gate.with_stops(stops.clone()),
            None => gate,
        }
    }

    /// Current time on the attached driver's clock (or `Instant::now()` when driverless).
    pub(crate) fn driver_clock_now(&self) -> Instant {
        if self.rt_executor_owned {
            return self.driver_clock_now_or(Instant::now());
        }
        self.driver
            .as_ref()
            .and_then(|d| d.with_ref(|drv| drv.clock_now()))
            .unwrap_or_else(Instant::now)
    }

    /// Active disable intent carried by an in-flight or pending background disable, if any.
    pub fn active_disable_kind(&self) -> Option<crate::driver::DisableKind> {
        self.active_disable_kind
    }

    /// Driver-side Rule A′ gate injected into every attached hardware/sim driver so the
    /// transmit choke point enforces the stopped rest-pose rule at send time.
    pub fn stopped_disable_gate(&self) -> crate::driver::StoppedDisableGate {
        let mut arm_joints = [0usize; crate::rt_core::mailbox::MAX_DOF];
        let mut arm_joints_len = 0usize;
        for &j in self.config.arm_joints().iter() {
            if arm_joints_len < arm_joints.len() {
                arm_joints[arm_joints_len] = j;
                arm_joints_len += 1;
            }
        }
        let mut rest_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let rest = self.config.resting_positions();
        for (i, &v) in rest.iter().enumerate().take(rest_positions.len()) {
            rest_positions[i] = v;
        }
        crate::driver::StoppedDisableGate {
            safety_stopped: self.safety.stopped_flag(),
            has_resting_pose: self.config.resting_pose.is_some(),
            arm_joints,
            arm_joints_len,
            rest_positions,
            rest_pose_tolerance_rad: self.config.rest_pose_tolerance_rad,
        }
    }

    /// Attaches the executor's authorized-stop generation tracker and updates the bus
    /// driver's `EstopGate` so mid-batch writes also stop on authorized stop requests.
    pub fn attach_stop_requests(&mut self, stops: crate::rt_core::urgent::StopRequests) {
        self.stop_requests = Some(stops);
        let gate = self.estop_gate();
        if let Some(d) = self.driver.as_mut() {
            let _ = d.with_mut(|drv| drv.set_estop_gate(gate));
        }
    }

    /// An E-stop request the tick has not applied yet (not the latch: hold output while
    /// latched still reaches the motors).
    pub(crate) fn estop_request_unapplied(&self) -> bool {
        self.estop_state.requested_count() != self.applied_estop_count
    }

    /// True when either an E-stop request or an attached authorized-stop request has not
    /// yet been applied by the control tick / release engine.
    pub(crate) fn urgent_stop_unapplied(&self) -> bool {
        self.estop_request_unapplied()
            || self.stop_requests.as_ref().is_some_and(crate::rt_core::urgent::StopRequests::pending)
    }

    pub(crate) fn estop_pending(&self) -> bool {
        let (req, latched) = self.estop_state.snapshot();
        latched || req != self.applied_estop_count
    }

    pub(super) fn reset_gripper_stall(&mut self) {
        self.gripper_stalled = false;
        self.gripper_stall_sign = 0.0;
        self.gripper_prev_hw_overload = false;
        self.gripper_high_torque_ticks = 0;
        self.gripper_low_torque_ticks = 0;
        self.gripper_stall_window.reset();
        self.gripper_force_capped = false;
        self.gripper_force_ticks = 0;
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
        self.gripper_stream_contact_detector.reset();
        self.gripper_stream_lease = None;
        self.gripper_cap_ramp = None;
        self.gripper_over_cap_prev = false;
    }

    /// End the gripper state of a servo stream or live policy (target, contact regulator) on
    /// stop, E-stop, fault and lease loss: nothing may advance J7 for that stream afterwards.
    /// While that state was active and torque is on, J7 holds the measured jaw.
    pub fn end_gripper_stream(&mut self) {
        let active = self.gripper_stream_target.is_some() || self.gripper_stream_contact.is_some();
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
        self.gripper_stream_contact_detector.reset();
        self.gripper_stream_lease = None;
        if !active || !self.is_torqued { return; }
        let Some(g) = self.config.gripper_index() else { return };
        let Some(&m) = self.measured_positions.get(g) else { return };
        if !m.is_finite() { return; }
        let m = self.config.limits.get(g).map_or(m, |l| m.clamp(l.min_position, l.max_position));
        if g < self.commanded_positions.len() { self.commanded_positions[g] = m; }
        if g < self.commanded_velocities.len() { self.commanded_velocities[g] = 0.0; }
        if g < self.target_positions.len() { self.target_positions[g] = m; }
        if let Some(st) = self.servo_target.as_mut() { if st.len() > g { st[g] = m; } }
        if let Some(h) = self.gripper_motion_hold.as_mut() { h.0 = m; }
    }

    /// A gripper action (or macro gripper step) ends without finishing (stop, E-stop, fault,
    /// lease loss, cancel): J7 command and target go to the measured jaw (clamped to the limits),
    /// so a command that led the jaw does not keep compressing after the stop.
    pub fn hold_gripper_at_measured(&mut self) {
        if !self.is_torqued { return; }
        let Some(g) = self.config.gripper_index() else { return };
        let Some(&m) = self.measured_positions.get(g) else { return };
        if !m.is_finite() { return; }
        let m = self.config.limits.get(g).map_or(m, |l| m.clamp(l.min_position, l.max_position));
        if g < self.commanded_positions.len() { self.commanded_positions[g] = m; }
        if g < self.commanded_velocities.len() { self.commanded_velocities[g] = 0.0; }
        if g < self.target_positions.len() { self.target_positions[g] = m; }
        if let Some(st) = self.servo_target.as_mut() { if st.len() > g { st[g] = m; } }
        if let Some(h) = self.gripper_motion_hold.as_mut() { h.0 = m; }
    }

    pub(super) fn gripper_reference_active(&self) -> bool {
        matches!(self.active_trajectory.as_ref().map(|a| &a.reference), Some(Reference::Gripper(_)))
    }

    /// Zero-allocation stack version of measured hold pose writing into `dst` (up to `dof`).
    pub(crate) fn measured_hold_pose_into(&self, dst: &mut [f32]) -> usize {
        let dof = self.commanded_positions.len().min(dst.len());
        dst[..dof].copy_from_slice(&self.commanded_positions[..dof]);
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            for (i, (c, m)) in dst[..dof].iter_mut().zip(&self.measured_positions).enumerate() {
                let l = &self.config.limits[i];
                *c = m.clamp(l.min_position, l.max_position);
            }
        }
        if let Some(g) = self.config.gripper_index() {
            if let (Some(c), Some(&cmd)) = (dst[..dof].get_mut(g), self.commanded_positions.get(g)) {
                *c = cmd;
            }
        }
        dof
    }

    /// End every motion and hold the measured pose (clamped to the limits) with torque on;
    /// the gripper keeps its command so that a held object stays held.
    pub(crate) fn hold_measured_pose(&mut self) {
        self.hold_measured_pose_inner(false);
    }

    /// [`Self::hold_measured_pose`] with optional tracking-history preservation (zero heap ops,
    /// zero moves/clones of `self.tracking`; used by `torque_trip` so active tracking lead ramps
    /// smoothly in the relieving direction).
    pub(crate) fn hold_measured_pose_inner(&mut self, preserve_tracking: bool) {
        self.stop_motion();
        self.servo_stream_active = false;
        if !preserve_tracking {
            self.tracking.invalidate();
        }
        let gripper_idx = self.config.gripper_index();
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            for (i, (c, m)) in self
                .commanded_positions
                .iter_mut()
                .zip(&self.measured_positions)
                .enumerate()
            {
                if Some(i) == gripper_idx {
                    continue;
                }
                let l = &self.config.limits[i];
                *c = m.clamp(l.min_position, l.max_position);
            }
        }
        copy_into(&mut self.target_positions, &self.commanded_positions);
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.servo_lead_stall.fill(None);
        self.safety.mark_motion_finished();
    }
}
