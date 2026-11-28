//! Driver attachment, feedback sync, limit probing/application, and feedback freshness.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

impl SupervisorInner {
    pub fn with_driver(mut self, mut driver: Box<dyn ArmDriver>) -> Self {
        driver.set_estop_state(self.estop_state.clone());
        driver.set_estop_gate(self.estop_gate());
        driver.set_gripper_index(self.config.gripper_index());
        driver.joint_mode_mut().set_strict_mode_handover(self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled));
        if driver.capabilities().load_feedback {
            self.loads = vec![f32::NAN; self.config.dof];
        }
        let driver: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &self.joint_zero_offsets));
        self.driver_caps = driver.capabilities();
        self.apply_driver_limits(driver.as_ref());
        self.install_driver(driver);
        self.bump_motion_config_revision();
        self.refresh_schema_cache();
        self
    }

    pub fn sync_driver_feedback(
        &mut self,
        states: &[crate::model::JointState],
        loads: Option<&[f32]>,
    ) {
        Self::apply_feedback(
            &mut self.measured_positions,
            &mut self.measured_velocities,
            &mut self.torques,
            &mut self.temperatures,
            &mut self.loads,
            self.config.dof,
            states,
            loads,
        );
    }

    pub(crate) fn apply_feedback(
        measured_positions: &mut [f32],
        measured_velocities: &mut [f32],
        torques: &mut [f32],
        temperatures: &mut [f32],
        loads_vec: &mut Vec<f32>,
        dof: usize,
        states: &[crate::model::JointState],
        loads: Option<&[f32]>,
    ) {
        for (i, state) in states.iter().enumerate().take(dof) {
            measured_positions[i] = state.position;
            measured_velocities[i] = state.velocity;
            torques[i] = state.torque;
            temperatures[i] = state.temperature;
        }
        if let Some(loads) = loads {
            if loads.len() == dof && loads.iter().all(|x| x.is_finite()) {
                copy_into(loads_vec, loads);
            }
        }
    }

    pub fn apply_driver_limits(&mut self, driver: &dyn ArmDriver) {
        let vel: Vec<_> = (0..self.config.dof).map(|i| driver.max_command_velocity(i)).collect();
        let pos: Vec<_> = (0..self.config.dof).map(|i| driver.position_limits(i)).collect();
        self.apply_extracted_driver_limits(&vel, &pos);
    }

    pub fn apply_extracted_driver_limits(
        &mut self,
        vel_limits: &[Option<f32>],
        pos_limits: &[Option<(f32, f32)>],
    ) {
        self.apply_extracted_driver_limits_inner(vel_limits, pos_limits, true);
    }

    pub fn apply_extracted_driver_limits_inner(
        &mut self,
        vel_limits: &[Option<f32>],
        pos_limits: &[Option<(f32, f32)>],
        trigger_fault: bool,
    ) {
        self.driver_config_error = None;
        for (i, slot) in self.cached_driver_pos_limits.iter_mut().enumerate() {
            *slot = pos_limits.get(i).copied().flatten();
        }
        let mut candidate = self.config.clone();

        for (i, limit) in candidate.limits.iter_mut().enumerate() {
            if let Some(Some(max)) = vel_limits.get(i) {
                if max.is_finite() && *max > 0.0 {
                    limit.max_velocity = limit.max_velocity.min(*max);
                }
            }
            if let Some(Some((d_min, d_max))) = pos_limits.get(i).copied() {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    let reason = format!(
                        "Driver position limits [{:.4}, {:.4}] for joint '{}' are invalid",
                        d_min, d_max, limit.name
                    );
                    self.driver_config_error = Some(reason.clone());
                    if trigger_fault {
                        self.safety.latch(
                            crate::safety::FaultKind::JointLimit,
                            crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                                buf.push_str(&reason);
                            }),
                        );
                    }
                    return;
                }
                let new_min = limit.min_position.max(d_min);
                let new_max = limit.max_position.min(d_max);
                if !new_min.is_finite() || !new_max.is_finite() || new_max - new_min < 1e-2 {
                    let reason = format!(
                        "Driver position limits [{:.4}, {:.4}] for joint '{}' conflict with profile limits [{:.4}, {:.4}] (intersected range < 0.01)",
                        d_min, d_max, limit.name, limit.min_position, limit.max_position
                    );
                    self.driver_config_error = Some(reason.clone());
                    if trigger_fault {
                        self.safety.latch(
                            crate::safety::FaultKind::JointLimit,
                            crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                                buf.push_str(&reason);
                            }),
                        );
                    }
                    return;
                }
                if let Some(pos) = candidate.resting_pose.as_ref().and_then(|r| r.positions.as_ref()).and_then(|p| p.get(i).copied()) {
                    if pos < new_min || pos > new_max {
                        let reason = format!(
                            "Driver position limits [{:.4}, {:.4}] for joint '{}' conflict with resting pose position {:.4}",
                            d_min, d_max, limit.name, pos
                        );
                        self.driver_config_error = Some(reason.clone());
                        if trigger_fault {
                            self.safety.latch(
                                crate::safety::FaultKind::JointLimit,
                                crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                                    buf.push_str(&reason);
                                }),
                            );
                        }
                        return;
                    }
                }
                if (new_min - limit.min_position).abs() > 1e-4 || (new_max - limit.max_position).abs() > 1e-4 {
                    tracing::info!(
                        "Intersected joint '{}' position limits with driver range: profile=[{:.4}, {:.4}], hw=[{:.4}, {:.4}] -> [{:.4}, {:.4}]",
                        limit.name, limit.min_position, limit.max_position, d_min, d_max, new_min, new_max
                    );
                }
                limit.min_position = new_min;
                limit.max_position = new_max;
            }
        }

        if candidate.validate_runtime_narrowed().is_err()
            && self.base_gripper_policy.is_some()
            && (self.gripper_calib_offset.is_some() || self.gripper_calib_jaw_opening.is_some())
        {
            let mut base_test = candidate.clone();
            base_test.gripper_policy = self.base_gripper_policy.clone();
            if base_test.validate_runtime_narrowed().is_ok() {
                tracing::warn!("Driver limits invalidated outdated calibrated gripper policy; reverting to base uncalibrated policy");
                self.gripper_calib_invalid = true;
                self.gripper_calib_offset = None;
                self.gripper_calib_jaw_opening = None;
                self.config.gripper_policy = self.base_gripper_policy.clone();
                candidate = base_test;
            }
        }

        if let Err(err) = candidate.validate_runtime_narrowed() {
            let reason = format!("Driver limits violated config invariants: {err}");
            self.driver_config_error = Some(reason.clone());
            if trigger_fault {
                self.safety.latch(
                    crate::safety::FaultKind::JointLimit,
                    crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                        buf.push_str(&reason);
                    }),
                );
            }
            return;
        }

        self.config = candidate;
        self.bump_motion_config_revision();
        if !self.rt_executor_owned {
            self.refresh_schema_cache();
        }
    }

    pub fn reapply_driver_limits(&mut self) -> bool {
        const MAX_DOF: usize = crate::rt_core::mailbox::MAX_DOF;
        let dof = self.config.dof;
        if dof <= MAX_DOF {
            let mut vel = [None::<f32>; MAX_DOF];
            let mut pos = [None::<(f32, f32)>; MAX_DOF];
            let locked = self.driver.as_ref().is_some_and(|d| {
                d.with_ref(|drv| {
                    for i in 0..dof {
                        vel[i] = drv.max_command_velocity(i);
                        pos[i] = drv.position_limits(i);
                    }
                })
                .is_some()
            });
            if locked {
                self.apply_extracted_driver_limits_inner(&vel[..dof], &pos[..dof], false);
                self.driver_config_error.is_none()
            } else {
                false
            }
        } else {
            let limits = self.driver.as_ref().and_then(|d| {
                d.with_ref(|drv| {
                    let vel: Vec<_> = (0..dof).map(|i| drv.max_command_velocity(i)).collect();
                    let pos: Vec<_> = (0..dof).map(|i| drv.position_limits(i)).collect();
                    (vel, pos)
                })
            });
            if let Some((vel_limits, pos_limits)) = limits {
                self.apply_extracted_driver_limits_inner(&vel_limits, &pos_limits, false);
                self.driver_config_error.is_none()
            } else {
                false
            }
        }
    }

    /// Zero-allocation check whether a candidate `GripperPolicy` satisfies both profile and
    /// driver hardware position limits without mutating `self`.
    #[allow(clippy::result_large_err)]
    pub(crate) fn validate_gripper_policy_against_driver_bounded(
        &self,
        gp: &crate::model::GripperPolicy,
    ) -> Result<(), crate::safety::BoundedText<256>> {
        use crate::safety::BoundedText;
        let Some(g_idx) = self.config.gripper_index() else {
            return Err(BoundedText::new("Gripper calibration requires a gripper joint"));
        };
        let Some(lim) = self.config.limits.get(g_idx) else {
            return Err(BoundedText::new("Gripper calibration requires gripper joint limits"));
        };
        let mut lo = lim.min_position;
        let mut hi = lim.max_position;
        if self.rt_executor_owned {
            if let Some((d_min, d_max)) = self.cached_driver_pos_limits.get(g_idx).copied().flatten() {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    return Err(BoundedText::new("Driver position limits for gripper joint are invalid"));
                }
                lo = lo.max(d_min);
                hi = hi.min(d_max);
            }
        } else if let Some(d) = self.driver.as_ref() {
            let pos_lim = d
                .with_ref(|drv| drv.position_limits(g_idx))
                .ok_or_else(|| BoundedText::new("driver unavailable while validating gripper calibration"))?;
            if let Some((d_min, d_max)) = pos_lim {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    return Err(BoundedText::new("Driver position limits for gripper joint are invalid"));
                }
                lo = lo.max(d_min);
                hi = hi.min(d_max);
            }
        }
        if self.driver_config_error.is_some() && self.driver.is_none() {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("Driver limit configuration error prevents gripper calibration")));
        }
        let in_lim = |v: f32| v.is_finite() && v >= lo && v <= hi;
        if !(in_lim(gp.closed_rad)
            && in_lim(gp.open_rad)
            && gp.jaw_opening.as_ref().is_none_or(|jo| jo.points.iter().all(|&[q, _]| in_lim(q))))
        {
            return Err(BoundedText::new(
                "Driver limits violated config invariants: Invalid gripper policy (open/closed inside the gripper joint limits, 0 <= squeeze_rad <= 0.5)",
            ));
        }
        Ok(())
    }

    /// Pure read-only validation of driver position/velocity limits into bounded stack storage
    /// (zero allocations, zero mutations to `self`).
    #[allow(clippy::result_large_err)] // inline bounded text avoids RT heap allocation
    pub(crate) fn probe_driver_limits_bounded(
        &self,
    ) -> Result<CandidateDriverLimits, crate::safety::BoundedText<256>> {
        self.probe_driver_limits_with_offset_shift_bounded(None)
    }

    /// Probes driver limits for candidate `new_offsets` BEFORE mutating `ZeroOffsetDriver`
    /// (zero heap allocations).
    #[allow(clippy::result_large_err)]
    pub(crate) fn probe_driver_limits_for_offsets_bounded(
        &self,
        new_offsets: &[f32],
    ) -> Result<CandidateDriverLimits, crate::safety::BoundedText<256>> {
        self.probe_driver_limits_with_offset_shift_bounded(Some(new_offsets))
    }

    #[allow(clippy::result_large_err)]
    pub(in crate::supervisor) fn probe_driver_limits_with_offset_shift_bounded(
        &self,
        candidate_offsets: Option<&[f32]>,
    ) -> Result<CandidateDriverLimits, crate::safety::BoundedText<256>> {
        use crate::safety::BoundedText;
        const MAX_DOF: usize = crate::rt_core::mailbox::MAX_DOF;
        let dof = self.config.dof;
        if dof > MAX_DOF || self.config.limits.len() != dof || self.config.home_positions.len() != dof {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver limit retry exceeds bounded RT DOF")));
        }
        let mut vel_limits = [None::<f32>; MAX_DOF];
        let mut pos_limits = [None::<(f32, f32)>; MAX_DOF];
        let locked = self.driver.as_ref().is_some_and(|d| {
            d.with_ref(|drv| {
                for i in 0..dof {
                    vel_limits[i] = drv.max_command_velocity(i);
                    pos_limits[i] = drv.position_limits(i).map(|(lo, hi)| {
                        if let Some(new_off) = candidate_offsets {
                            let old = self.joint_zero_offsets.get(i).copied().unwrap_or(0.0);
                            let next = new_off.get(i).copied().unwrap_or(old);
                            let shift = old - next;
                            (lo + shift, hi + shift)
                        } else {
                            (lo, hi)
                        }
                    });
                }
            })
            .is_some()
        });
        if !locked {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver unavailable while revalidating limits")));
        }

        let mut cand_min = [0.0f32; MAX_DOF];
        let mut cand_max = [0.0f32; MAX_DOF];
        let mut cand_vel = [0.0f32; MAX_DOF];
        for i in 0..dof {
            let limit = &self.config.limits[i];
            let mut new_vel = limit.max_velocity;
            if let Some(max) = vel_limits[i] {
                if max.is_finite() && max > 0.0 {
                    new_vel = new_vel.min(max);
                }
            }
            if !new_vel.is_finite() || new_vel <= 0.0 {
                let mut msg = BoundedText::<256>::empty();
                let _ = core::fmt::write(&mut msg, format_args!("Driver limits violated config invariants: Joint {i} max_velocity must be positive finite"));
                return Err(msg);
            }
            let mut new_min = limit.min_position;
            let mut new_max = limit.max_position;
            if let Some((d_min, d_max)) = pos_limits[i] {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    let mut msg = BoundedText::<256>::empty();
                    let _ = core::fmt::write(
                        &mut msg,
                        format_args!("Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' are invalid", limit.name),
                    );
                    return Err(msg);
                }
                new_min = new_min.max(d_min);
                new_max = new_max.min(d_max);
                if !new_min.is_finite() || !new_max.is_finite() || new_max - new_min < 1e-2 {
                    let mut msg = BoundedText::<256>::empty();
                    let _ = core::fmt::write(
                        &mut msg,
                        format_args!(
                            "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' conflict with profile limits [{:.4}, {:.4}] (intersected range < 0.01)",
                            limit.name, limit.min_position, limit.max_position
                        ),
                    );
                    return Err(msg);
                }
                if let Some(pos) = self.config.resting_pose.as_ref().and_then(|r| r.positions.as_ref()).and_then(|p| p.get(i).copied()) {
                    if !pos.is_finite() || pos < new_min || pos > new_max {
                        let mut msg = BoundedText::<256>::empty();
                        let _ = core::fmt::write(
                            &mut msg,
                            format_args!(
                                "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' conflict with resting pose position {pos:.4}",
                                limit.name
                            ),
                        );
                        return Err(msg);
                    }
                }
            }
            let home = self.config.home_positions[i];
            if !home.is_finite() || home < new_min || home > new_max {
                return Err(BoundedText::new(
                    "Driver limits violated config invariants: Home positions must be finite and within joint limits",
                ));
            }
            cand_min[i] = new_min;
            cand_max[i] = new_max;
            cand_vel[i] = new_vel;
        }
        if let (Some(g), Some(dg)) = (self.config.gripper_index(), &self.config.damiao_gripper) {
            if dg.mode == crate::model::DamiaoGripperMode::PosForce
                && crate::model::DamiaoGripperDrive::quantize_vel_raw(cand_vel[g]) == 0
            {
                let mut msg = BoundedText::<256>::empty();
                let _ = core::fmt::write(
                    &mut msg,
                    format_args!("Driver limits violated config invariants: Joint {g} max_velocity encodes to 0 on the pos-force wire (must quantize to >= 0.01 rad/s)"),
                );
                return Err(msg);
            }
        }
        if let Some(gp) = &self.config.gripper_policy {
            let ok = self.config.gripper_index().is_some_and(|g_idx| {
                let (lo, hi) = (cand_min[g_idx], cand_max[g_idx]);
                let in_lim = |v: f32| v.is_finite() && v >= lo && v <= hi;
                in_lim(gp.closed_rad)
                    && in_lim(gp.open_rad)
                    && gp.jaw_opening.as_ref().is_none_or(|jo| jo.points.iter().all(|&[q, _]| in_lim(q)))
            });
            if !ok {
                return Err(BoundedText::new(
                    "Driver limits violated config invariants: Invalid gripper policy (open/closed inside the gripper joint limits, 0 <= squeeze_rad <= 0.5)",
                ));
            }
        }
        if self.driver_config_error.is_some() && self.retired_driver_config_error.is_some() {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver config error reclamation pending")));
        }

        Ok(CandidateDriverLimits {
            dof,
            min: cand_min,
            max: cand_max,
            vel: cand_vel,
            raw_pos: pos_limits,
        })
    }

    /// Commits validated candidate driver limits in place and retires any previous error string
    /// off-thread (zero heap ops; invoked ONLY after `clear_check` passes).
    pub(crate) fn commit_driver_limits_bounded(&mut self, cand: &CandidateDriverLimits) {
        for i in 0..cand.dof {
            self.config.limits[i].min_position = cand.min[i];
            self.config.limits[i].max_position = cand.max[i];
            self.config.limits[i].max_velocity = cand.vel[i];
            self.cached_driver_pos_limits[i] = cand.raw_pos[i];
        }
        self.bump_motion_config_revision();
        if let Some(prev) = self.driver_config_error.take() {
            if self.retired_driver_config_error.is_none() {
                self.retired_driver_config_error = Some(prev);
            } else {
                self.driver_config_error = Some(prev);
            }
        }
    }

    /// Pure read-only check of `positions` against either validated candidate limits (if
    /// `driver_config_error` just resolved) or current `self.config.limits` (zero heap allocations).
    pub(crate) fn joint_limit_clear_violation_for_positions(
        &self,
        positions: &[f32],
        cand: Option<&CandidateDriverLimits>,
    ) -> Option<crate::safety::JointLimitClearViolation> {
        use crate::safety::{BoundedText, JointLimitClearViolation as V};
        let dof = cand.map_or(self.config.dof, |c| c.dof);
        if positions.len() != dof || self.config.limits.len() != dof {
            return Some(V::FeedbackIncomplete {
                measured_len: positions.len(),
                dof,
            });
        }
        for i in 0..dof {
            let q = positions[i];
            let joint_name = BoundedText::new(&self.config.limits[i].name);
            if !q.is_finite() {
                return Some(V::NonFinite {
                    joint_idx: i as u16,
                    joint_name,
                    value: q,
                });
            }
            let (min, max) = match cand {
                Some(c) => (c.min[i], c.max[i]),
                None => (self.config.limits[i].min_position, self.config.limits[i].max_position),
            };
            if q < min - JOINT_LIMIT_CLEAR_MARGIN_RAD || q > max + JOINT_LIMIT_CLEAR_MARGIN_RAD {
                return Some(V::OutOfBounds {
                    joint_idx: i as u16,
                    joint_name,
                    value: q,
                    min,
                    max,
                    margin: JOINT_LIMIT_CLEAR_MARGIN_RAD,
                });
            }
        }
        None
    }

    /// Pure read-only check of measured positions against either validated candidate limits (if
    /// `driver_config_error` just resolved) or current `self.config.limits` (zero heap allocations).
    pub(crate) fn joint_limit_clear_violation_with_candidates(
        &self,
        cand: Option<&CandidateDriverLimits>,
    ) -> Option<crate::safety::JointLimitClearViolation> {
        self.joint_limit_clear_violation_for_positions(&self.measured_positions, cand)
    }

    /// Bounded, zero-allocation retry of driver limits for the RT executor `clear_fault` path.
    #[allow(dead_code)]
    pub(crate) fn reapply_driver_limits_bounded(&mut self) -> Option<crate::safety::BoundedText<256>> {
        match self.probe_driver_limits_bounded() {
            Ok(cand) => {
                self.commit_driver_limits_bounded(&cand);
                None
            }
            Err(err) => Some(err),
        }
    }

    /// Store the motor error codes, logging each per-joint change once.
    pub(in crate::supervisor) fn update_motor_errors(&mut self, errors: Vec<Option<crate::model::MotorError>>) {
        if errors == self.motor_errors { return; }
        for (i, e) in errors.iter().enumerate() {
            let prev = self.motor_errors.get(i).cloned().flatten();
            match (prev, e) {
                (p, Some(e)) if p.as_ref() != Some(e) =>
                    tracing::warn!("Joint {} motor reports error code 0x{:X} ({})", i + 1, e.code, e.name),
                (Some(p), None) => tracing::info!("Joint {} motor error 0x{:X} ({}) cleared", i + 1, p.code, p.name),
                _ => {}
            }
        }
        self.motor_errors = errors;
    }

    pub(crate) fn states_valid(
        states: &[crate::model::JointState],
        caps: DriverCapabilities,
        dof: usize,
        loads: Option<&[f32]>,
    ) -> bool {
        states.len() == dof
            && states.iter().all(|s| crate::driver::joint_state_valid(s, caps))
            && (!caps.load_feedback
                || loads.is_some_and(|l| l.len() == dof && l.iter().all(|x| x.is_finite())))
    }

    /// Per-joint subset numeric validity (e.g., J1..J6 arm joints excluding the gripper).
    #[allow(dead_code)]
    pub(crate) fn states_valid_joints(
        states: &[crate::model::JointState],
        caps: DriverCapabilities,
        joints: &[usize],
        loads: Option<&[f32]>,
    ) -> bool {
        !joints.is_empty()
            && joints.iter().all(|&i| {
                states.get(i).is_some_and(|s| crate::driver::joint_state_valid(s, caps))
                    && (!caps.load_feedback
                        || loads.and_then(|l| l.get(i)).is_some_and(|x| x.is_finite()))
            })
    }

    pub fn hardware_feedback_fresh(&self, max_age: Duration) -> bool {
        if !self.feedback_numeric_valid { return false; }
        // Ages alone decide: drivers' is_feedback_valid() hard-codes 250 ms,
        // which would silently cap callers asking for a longer bound.
        if self.driver.is_none() {
            return true;
        }
        self.read_driver_for_tick()
            .is_some_and(|drv| drv.feedback_all_within(self.config.dof, max_age))
    }

    /// True when every arm joint (excluding a `PosForce` Damiao gripper) has valid numeric
    /// feedback within `max_age`. Used by gravity compensation so transient J7 pos-force
    /// staleness does not prematurely abort or wedge arm-only handback before
    /// `feedback_stale_fault_ms` latches `J7_FEEDBACK_STALE`.
    pub fn hardware_arm_feedback_fresh(&self, max_age: Duration) -> bool {
        if !self.feedback_numeric_valid { return false; }
        if !self.config.damiao_gripper.as_ref().is_some_and(|g| g.mode == crate::model::DamiaoGripperMode::PosForce) {
            return self.hardware_feedback_fresh(max_age);
        }
        let arm = self.config.arm_joints();
        if self.driver.is_none() {
            return true;
        }
        self.read_driver_for_tick()
            .is_some_and(|drv| drv.feedback_joints_within(&arm, max_age))
    }

    /// Per arm joint, an instant before which its last sample cannot have been enqueued: the
    /// driver's receive-queue bound, or the receive instant for drivers without a receive queue.
    pub fn arm_feedback_queue_bounds(&self) -> Vec<Option<Instant>> {
        self.config.arm_joint_indices().iter().map(|&i| match self.joint_feedback_queue_bounds.get(i) {
            Some(b) => *b,
            None => self.core.feedback().instant(i),
        }).collect()
    }

    /// Snapshot of `(measured_positions, arm_velocities, arm_feedback_instants, arm_feedback_seqs)`
    /// read coherently under the supervisor lock.
    pub fn arm_feedback_snapshot(&self) -> (Vec<f32>, Vec<f32>, Vec<Option<Instant>>, Vec<u64>) {
        let arm = self.config.arm_joint_indices();
        let vels = arm.iter().map(|&i| self.measured_velocities.get(i).copied().unwrap_or(0.0)).collect();
        let instants = arm
            .iter()
            .map(|&i| self.core.feedback().instant(i))
            .collect();
        let seqs = arm
            .iter()
            .map(|&i| self.core.feedback().seq(i))
            .collect();
        (self.measured_positions.clone(), vels, instants, seqs)
    }
}
