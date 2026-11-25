//! Configuration of the supervisor: commissioning, calibration and joint/motor zeros, camera mount, kinematics correction, grasp setup and gripper calibration (track B).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

impl SupervisorInner {
    /// Reject ordinary motion/action dispatch while a newly installed non-stock tool awaits commissioning.
    pub fn check_commissioning_for_ordinary_motion(&self, op: &str) -> Result<(), ArmError> {
        if self.config.active_tool.commissioning_required {
            return Err(SafetyError::CommissioningRequired(op.to_string()).into());
        }
        Ok(())
    }

    pub fn tool_commissioning_response(&self) -> serde_json::Value {
        serde_json::json!({
            "active_tool": self.config.active_tool,
            "commissioning_required": self.config.active_tool.commissioning_required,
            "confirmed_at": self.tool_commissioning_confirmed_at,
            "revision": self.tool_commissioning_revision,
        })
    }

    /// `capabilities.calibration_tools`: the profile's camera / kinematics / gripper support
    /// and the driver's motor zero support.
    pub fn calibration_tools(&self) -> crate::model::CalibrationTools {
        self.calibration_tools_with(&crate::model::Capabilities::from_config(&self.config))
    }

    pub(super) fn calibration_tools_with(&self, caps: &crate::model::Capabilities) -> crate::model::CalibrationTools {
        use crate::model::{CalibrationToolInfo, GripperCalibrationTool, MotorZeroTool};
        let no_grasp = "this profile has no grasp setup (the camera calibration needs one)";
        let kin_reason = if self.config.arm_model.chain().is_some() {
            "this profile uses a URDF chain; the kinematic correction applies to the built-in model only"
        } else { no_grasp };
        let mut support = self.motor_zero_support();
        let (motor, software) = (support.supported, self.config.zero_reference.is_some());
        let methods: Vec<String> = [(software, "software"), (motor, "motor")].into_iter().filter(|(ok, _)| *ok).map(|(_, m)| m.to_string()).collect();
        if software && !motor {
            support = crate::model::MotorZeroSupport { supported: true, notes: support.reason.take().map(|r| vec![format!("motor zero: {r}")]).unwrap_or_default(), ..Default::default() };
        }
        let any = motor || software;
        let motor_zero = MotorZeroTool {
            joints: if any { (1..=self.config.dof).collect() } else { Vec::new() },
            requires: if any { ["lease", "torque_off", "no_action", "no_fault"].map(String::from).to_vec() } else { Vec::new() },
            max_offset_deg: crate::model::MOTOR_ZERO_MAX_OFFSET_DEG,
            verify_mrad: crate::model::MOTOR_ZERO_VERIFY_MRAD,
            invalidates: if any { ["kinematics", "camera", "poses", "gripper_calibration"].map(String::from).to_vec() } else { Vec::new() },
            zero_pose: self.config.zero_pose.clone(),
            motor, software, methods,
            support,
        };
        let policy = self.config.gripper_policy.as_ref().filter(|_| self.config.gripper_index().is_some());
        let gripper = GripperCalibrationTool {
            supported: policy.is_some(),
            zero_check: policy.is_some_and(|p| p.zero_check_supported),
            calibration: policy.is_some(),
            reason: policy.is_none().then(|| "this profile has no gripper joint with a gripper policy".to_string()),
        };
        crate::model::CalibrationTools {
            camera: CalibrationToolInfo::new(caps.calibration_apply, Vec::new(), no_grasp),
            kinematics: CalibrationToolInfo::new(caps.calibration_kinematics, vec![2, 3, 4, 5], kin_reason),
            motor_zero,
            gripper,
        }
    }

    /// The driver's motor zero support; the mock follower (no driver) zeroes its readings.
    pub fn motor_zero_support(&self) -> crate::model::MotorZeroSupport {
        match self.driver.as_ref() {
            Some(d) => d.with_ref(|drv| drv.motor_zero_support()).unwrap_or_else(|| crate::model::MotorZeroSupport::unsupported("driver unavailable")),
            None => crate::model::MotorZeroSupport::set_current("process", true,
                vec!["mock follower: the readings of the zeroed joints become 0 (the simulated arm has no encoder offset)".into()]),
        }
    }

    /// Install joint zero offsets (motor - joint, rad, one per joint) at the driver boundary.
    /// The current readings, commanded and target positions move by old - new, so nothing jumps;
    /// the mock follower (no driver) moves its simulated readings the same way. Needs torque
    /// off; the caller checks the rest.
    pub fn set_joint_zero_offsets(&mut self, offsets: &[f32]) -> Result<(), String> {
        if self.is_torqued || self.disable_pending { return Err("torque is on".into()); }
        if offsets.len() != self.config.dof || !offsets.iter().all(|o| o.is_finite()) {
            return Err(format!("joint zero needs {} finite offsets", self.config.dof));
        }
        let old = std::mem::replace(&mut self.joint_zero_offsets, offsets.to_vec());
        let changed = old != offsets;
        self.tracking.invalidate();
        if changed { self.calibration_revision += 1; self.refresh_effective_floor(); }
        if let Some(d) = self.driver.as_mut() {
            let dof = self.config.dof;
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let has_pos_limits = d
                .with_mut(|drv| {
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    drv.set_joint_zero_offsets(offsets);
                    (0..dof).any(|i| drv.position_limits(i).is_some())
                })
                .ok_or_else(|| "driver unavailable".to_string())?;
            if has_pos_limits { self.reapply_driver_limits(); }
        }
        let mut gripper_zero_moved = false;
        for (i, (o, n)) in old.iter().zip(offsets).enumerate() {
            let d = o - n;
            if d == 0.0 { continue; }
            if Some(i) == self.config.gripper_index() { gripper_zero_moved = true; }
            for v in [&mut self.measured_positions, &mut self.commanded_positions, &mut self.target_positions] {
                if let Some(p) = v.get_mut(i) { *p += d; }
            }
        }
        // the free-motion baseline is indexed by jaw position: a new gripper zero invalidates it
        if gripper_zero_moved && self.gripper_baseline.is_some() { self.reset_gripper_baseline(); }
        if old.len() > 5 && offsets.len() > 5 && old[5] != offsets[5] {
            self.zero_convention = Default::default();
        }
        self.zero_check_pending = true;
        #[cfg(not(target_arch = "wasm32"))]
        if changed {
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::JointZeroRecalibrated);
            if !self.rt_executor_owned {
                self.flush_torque_shadow();
            }
        }
        Ok(())
    }

    /// The reference pose in joint angles (`home_positions` derived from `zero_reference`).
    pub fn zero_reference_joints(&self) -> Option<Vec<f32>> {
        self.config.zero_reference.as_ref().map(|_| self.config.home_positions.clone())
    }

    /// The zero check (`zero_convention`) at startup and each time torque goes off: with torque
    /// off and fresh feedback, an arm in the rest pose with J6 about +-90 deg from the reference is
    /// reported as `legacy_j6`. Never moves the arm. A `legacy_j6` verdict stays until a zero of J6.
    pub(super) fn step_zero_convention(&mut self) {
        const REBOT_DM_TOL: [f32; 6] = [0.15, 0.08, 0.08, 0.08, 0.08, 0.08];
        const DEFAULT_TOL: [f32; 6] = [0.08; 6];
        if self.zero_check_prev_torqued && !self.is_torqued { self.zero_check_pending = true; }
        self.zero_check_prev_torqued = self.is_torqued;
        if !self.zero_check_pending || self.is_torqued || self.calibration_busy { return; }
        if self.config.zero_reference.is_none() { self.zero_check_pending = false; return; }
        if let Some(d) = self.driver.as_ref() {
            if !d.with_ref(|drv| drv.is_feedback_valid()).unwrap_or(false) { return; }
        }
        let tol: &[f32] = match &self.config.resting_pose {
            Some(p) => &p.position_tolerances,
            None if self.driver.is_none() => &REBOT_DM_TOL,
            None => &DEFAULT_TOL,
        };
        self.zero_check_pending = false;
        let Some(v) = crate::joint_zero::check_convention(&self.measured_positions, &self.config.home_positions, tol) else { return; };
        if self.zero_convention.status == "legacy_j6" && v.status != "legacy_j6" { return; }
        if v.status == "legacy_j6" && self.zero_convention.status != "legacy_j6" {
            if self.rt_executor_owned {
                self.pending_legacy_j6_log = Some(v);
            } else if let Some(msg) = v.message() {
                tracing::warn!("Joint zero check: {msg}");
            }
        }
        self.zero_convention = v;
    }

    /// Make the current position of `joint` (0-based) its zero: the driver's
    /// [`ArmDriver::set_zero_position`], or for the mock follower the joint's reading (and
    /// commanded / target value) becomes 0. Needs torque off; the caller checks the rest.
    pub fn set_motor_zero(&mut self, joint: usize) -> Result<(), String> {
        if self.is_torqued || self.disable_pending { return Err("torque is on".into()); }
        if joint >= self.config.dof { return Err(format!("joint {} does not exist", joint + 1)); }
        // a new J6 zero ends a legacy_j6 verdict; the next check decides again
        if joint == 5 { self.zero_convention = Default::default(); }
        self.zero_check_pending = true;
        match self.driver.as_mut() {
            Some(d) => d.with_mut(|drv| drv.set_zero_position(joint)).ok_or_else(|| "driver unavailable".to_string())?.map_err(|e| e.to_string()),
            None => {
                // the simulated motor reads 0: the joint reads 0 - its joint zero offset
                let r = -self.joint_zero_offsets.get(joint).copied().unwrap_or(0.0);
                for v in [&mut self.measured_positions, &mut self.commanded_positions, &mut self.target_positions] {
                    if let Some(p) = v.get_mut(joint) { *p = r; }
                }
                Ok(())
            }
        }
    }

    /// Runtime grasp setup (`config.grasp`, with the camera tilt taken from `camera_mount`).
    /// None when the profile cannot grasp (no kinematics, gripper policy or grasp section).
    pub fn grasp_setup(&self) -> Option<crate::grasp::GraspSetup> {
        if !self.config.has_cartesian_with_gripper() || self.config.kinematics.is_none() || self.config.gripper_policy.is_none() { return None; }
        let mut s = self.config.grasp.clone()?;
        s.camera_tilt_deg = f64::from(self.config.camera_mount.tilt_deg);
        Some(s)
    }

    /// The camera mount is calibrated (`calibration.json` with `q_end_cam` in effect).
    pub fn camera_calibrated(&self) -> bool {
        self.calibration.as_ref().is_some_and(|c| c.camera_mount.q_end_cam.is_some())
    }

    /// Install the calibration in effect (load / apply / revert / restore / zero invalidation)
    /// and recompute the effective floor.
    pub fn set_calibration(&mut self, c: Option<crate::calibration::CalibrationFile>) {
        self.calibration = c;
        #[cfg(feature = "policy-runtime")]
        { self.calibration_digest = Self::calibration_digest_of(self.calibration.as_ref()); }
        self.refresh_effective_floor();
        if !self.rt_executor_owned {
            self.refresh_schema_cache();
        }
    }

    /// Recompute `config.workspace.floor_z_m` = max(profile floor, valid calibrated table
    /// height - 3 mm) (src/calibration.rs `effective_floor`). Every floor check (end points,
    /// link capsules, swept paths, servo steps) reads that field. A change bumps the workspace
    /// revision (plans made with the old floor fail PLAN_STALE).
    pub fn refresh_effective_floor(&mut self) {
        let Some(profile) = self.profile_floor_z_m else { return; };
        let (_, fp) = crate::calibration::live_fingerprints(self);
        let legacy_fp = crate::calibration::legacy_file_fingerprint(
            self.calibration.as_ref(),
            &self.config.camera_mount,
            &self.joint_zero_offsets,
        );
        let (floor, source, _) = crate::calibration::effective_floor_for_with_legacy(
            profile,
            self.calibration.as_ref(),
            &fp,
            Some(&legacy_fp),
            &self.config.active_tool,
        );
        self.floor_source = source;
        let Some(ws) = self.config.workspace.as_mut() else { return; };
        if ws.floor_z_m != floor {
            tracing::info!("Effective floor {:.4} m (was {:.4} m, profile {profile:.4} m, source {source})", floor, ws.floor_z_m);
            ws.floor_z_m = floor;
            self.workspace_revision += 1;
            self.bump_motion_config_revision();
            self.clear_servo_ws_min_depths_rt();
        }
    }

    /// Install a camera mount (calibration apply / restore / startup): bumps the workspace
    /// revision when it changes and keeps the grasp setup's tilt in sync with the mount's.
    pub fn set_camera_mount(&mut self, mount: crate::model::CameraMount) {
        if self.config.camera_mount != mount {
            self.workspace_revision += 1;
            self.bump_motion_config_revision();
            self.clear_servo_ws_min_depths_rt();
        }
        self.config.camera_mount = mount;
        let tilt = f64::from(self.config.camera_mount.tilt_deg);
        if let Some(g) = self.config.grasp.as_mut() { g.camera_tilt_deg = tilt; }
        self.refresh_effective_floor();
        self.prepare_workspace_eval_bufs();
        if !self.rt_executor_owned {
            self.refresh_schema_cache();
        }
    }

    /// Install the kinematic correction of the built-in model (calibration apply / restore /
    /// startup; None = uncorrected): the live swap of `config.arm_model`, which every FK / IK /
    /// workspace / keep-out / grasp path reads under this lock. Bumps the workspace revision when
    /// it changes (plans made with the old model fail PLAN_STALE) and recomputes the stored
    /// poses' TCPs. Err (nothing changed): a URDF chain profile or values outside the bounds.
    pub fn set_kinematics_correction(&mut self, corr: Option<&crate::kinematics::Correction>) -> Result<(), String> {
        let model = self.config.arm_model.with_correction(corr)?;
        if model.correction() == self.config.arm_model.correction() { return Ok(()); }
        self.config.arm_model = model;
        // the servo escape baseline was measured with the old model
        self.workspace_revision += 1;
        self.bump_motion_config_revision();
        self.clear_servo_ws_min_depths_rt();
        self.calibration_revision += 1;
        self.refresh_pose_tcps();
        self.refresh_effective_floor();
        if !self.rt_executor_owned {
            self.refresh_schema_cache();
        }
        Ok(())
    }

    /// Replace the runtime setup; `camera_tilt_deg` also sets `camera_mount.tilt_deg`.
    pub fn set_grasp_setup(&mut self, setup: crate::grasp::GraspSetup) {
        if self.config.camera_mount.tilt_deg != setup.camera_tilt_deg as f32 {
            self.workspace_revision += 1;
            self.bump_motion_config_revision();
            self.clear_servo_ws_min_depths_rt();
        }
        self.config.camera_mount.tilt_deg = setup.camera_tilt_deg as f32;
        self.config.grasp = Some(setup);
    }

    /// Apply gripper calibration: offset shifts closed_rad, open_rad and jaw points.
    pub fn apply_gripper_calibration(
        &mut self,
        offset_rad: Option<f32>,
        jaw_opening: Option<crate::model::JawOpening>,
    ) -> Result<(), String> {
        if self.base_gripper_policy.is_none() {
            self.base_gripper_policy = self.config.gripper_policy.clone();
        }
        let Some(base) = self.base_gripper_policy.clone() else {
            return Err("No base gripper policy available".into());
        };
        let mut policy = base.clone();
        let offset = offset_rad.unwrap_or(0.0);
        if !offset.is_finite() {
            return Err("offset_rad must be finite".into());
        }
        let closed_rad = base.closed_rad + offset;
        let mut open_rad = base.open_rad + offset;
        if !closed_rad.is_finite() || !open_rad.is_finite() {
            return Err("Computed closed_rad or open_rad is not finite".into());
        }
        let lim = self.config.gripper_index().and_then(|idx| self.config.limits.get(idx));
        // The measured stop must lie inside the limits; the open end may be shifted past one
        // (the base open_rad sits close to it), so it is held at the limit.
        if let Some(lim) = lim {
            if closed_rad < lim.min_position || closed_rad > lim.max_position {
                return Err(format!(
                    "Calibrated closed_rad {closed_rad:.4} exceeds joint limits [{}, {}]",
                    lim.min_position, lim.max_position
                ));
            }
            open_rad = open_rad.clamp(lim.min_position, lim.max_position);
        }
        if (closed_rad - open_rad).abs() < 0.1 {
            return Err(format!(
                "Base gripper policy span |closed_rad - open_rad| < 0.1 (closed: {closed_rad}, open: {open_rad})"
            ));
        }
        policy.closed_rad = closed_rad;
        policy.open_rad = open_rad;

        let candidate_jo = jaw_opening.clone()
            .or_else(|| self.gripper_calib_jaw_opening.clone())
            .or_else(|| base.jaw_opening.clone());
        // The table is checked against the limits before the offset shift: the shift can move
        // its open end past a limit, like open_rad, and those points are never reached.
        let mut jo = match candidate_jo {
            Some(jo) => jo,
            None if self.config.active_tool.is_stock => crate::model::JawOpening::sim_default(),
            None => return Err("Non-stock tool requires an explicit jaw_opening calibration curve".into()),
        };
        jo.validate(lim).map_err(|e| format!("Invalid jaw_opening calibration against joint limits: {e}"))?;
        for pt in &mut jo.points {
            pt[0] += offset;
        }
        jo.validate(None).map_err(|e| format!("Invalid jaw_opening calibration: {e}"))?;
        policy.jaw_opening = Some(jo);
        #[cfg(not(target_arch = "wasm32"))]
        let max_w = self.config.grasp.as_ref().map(|g| g.max_grasp_width_m);
        #[cfg(target_arch = "wasm32")]
        let max_w: Option<f64> = None;
        // the jaw curve is not checked against max_grasp_width_m: a narrower curve keeps the
        // calibration and clamps the grasp width (`grasp_width_check`)
        crate::model::validate_effective_gripper_policy_ex(&policy, lim, None, false)?;
        if let Some(max_w) = max_w {
            if let crate::model::GraspWidthCheck::Clamped { curve_max_m } =
                crate::model::check_grasp_width(&policy, max_w, false)
            {
                tracing::warn!(
                    "Gripper jaw curve opens {curve_max_m:.4} m, less than max_grasp_width_m {max_w:.4} m; grasps use {curve_max_m:.4} m"
                );
            }
        }

        self.config.gripper_policy = Some(policy);
        self.gripper_calib_offset = offset_rad;
        if jaw_opening.is_some() {
            self.gripper_calib_jaw_opening = jaw_opening;
        }
        self.gripper_calib_invalid = false;
        Ok(())
    }

    /// Validate a candidate gripper calibration (including hardware driver limits) without
    /// leaving `self` mutated. Returns the resulting effective GripperPolicy on success.
    /// Build and validate a complete gripper calibration candidate (including driver limit
    /// re-application) without leaving `SupervisorInner` mutated.
    pub fn build_gripper_calibration_candidate(
        &mut self,
        offset_rad: Option<f32>,
        jaw_opening: Option<crate::model::JawOpening>,
    ) -> Result<(GripperCalibrationCandidate, crate::model::GripperPolicy), String> {
        let prev_config = self.config.clone();
        let prev_base = self.base_gripper_policy.clone();
        let prev_offset = self.gripper_calib_offset;
        let prev_jaw = self.gripper_calib_jaw_opening.clone();
        let prev_invalid = self.gripper_calib_invalid;
        let prev_err = self.driver_config_error.clone();

        let res = (|| {
            self.apply_gripper_calibration(offset_rad, jaw_opening)?;
            if self.driver.is_some() || self.driver_config_error.is_some() {
                let ok = self.reapply_driver_limits();
                if !ok || self.gripper_calib_invalid || self.driver_config_error.is_some() {
                    return Err(self.driver_config_error.clone().unwrap_or_else(|| {
                        "Gripper calibration rejected by driver hardware limits".to_string()
                    }));
                }
            }
            let policy = self.config.gripper_policy.clone().ok_or_else(|| {
                "Gripper calibration requires a gripper policy".to_string()
            })?;
            let cand = GripperCalibrationCandidate {
                config: self.config.clone(),
                base_gripper_policy: self.base_gripper_policy.clone(),
                gripper_calib_offset: self.gripper_calib_offset,
                gripper_calib_jaw_opening: self.gripper_calib_jaw_opening.clone(),
                gripper_calib_invalid: self.gripper_calib_invalid,
                driver_config_error: self.driver_config_error.clone(),
            };
            Ok((cand, policy))
        })();

        self.config = prev_config;
        self.base_gripper_policy = prev_base;
        self.gripper_calib_offset = prev_offset;
        self.gripper_calib_jaw_opening = prev_jaw;
        self.gripper_calib_invalid = prev_invalid;
        self.driver_config_error = prev_err;

        res
    }

    /// Standalone off-thread validation and construction of a gripper calibration candidate
    /// (`armd-persist` runs this off `armd-control` so policy cloning and error formatting never
    /// touch the RT allocator).
    pub(crate) fn validate_gripper_calibration_off_thread(
        base_policy: Option<&crate::model::GripperPolicy>,
        current_calib_jaw: Option<&crate::model::JawOpening>,
        gripper_limit: Option<&crate::model::JointLimit>,
        is_stock_tool: bool,
        offset_rad: Option<f32>,
        jaw_opening: Option<&crate::model::JawOpening>,
    ) -> Result<crate::model::GripperPolicy, String> {
        let base = base_policy.ok_or_else(|| "No base gripper policy available".to_string())?;
        let offset = offset_rad.unwrap_or(0.0);
        if !offset.is_finite() {
            return Err("offset_rad must be finite".into());
        }
        let closed_rad = base.closed_rad + offset;
        let mut open_rad = base.open_rad + offset;
        if !closed_rad.is_finite() || !open_rad.is_finite() {
            return Err("Computed closed_rad or open_rad is not finite".into());
        }
        if let Some(lim) = gripper_limit {
            if closed_rad < lim.min_position || closed_rad > lim.max_position {
                return Err(format!(
                    "Calibrated closed_rad {closed_rad:.4} exceeds joint limits [{}, {}]",
                    lim.min_position, lim.max_position
                ));
            }
            open_rad = open_rad.clamp(lim.min_position, lim.max_position);
        }
        if (closed_rad - open_rad).abs() < 0.1 {
            return Err(format!(
                "Base gripper policy span |closed_rad - open_rad| < 0.1 (closed: {closed_rad}, open: {open_rad})"
            ));
        }
        let candidate_jo = jaw_opening
            .cloned()
            .or_else(|| current_calib_jaw.cloned())
            .or_else(|| base.jaw_opening.clone());
        let mut jo = match candidate_jo {
            Some(jo) => jo,
            None if is_stock_tool => crate::model::JawOpening::sim_default(),
            None => return Err("Non-stock tool requires an explicit jaw_opening calibration curve".into()),
        };
        let mut policy = base.clone();
        policy.closed_rad = closed_rad;
        policy.open_rad = open_rad;
        if let Err(e) = jo.validate(gripper_limit) {
            return Err(format!("Invalid jaw_opening calibration against joint limits: {e}"));
        }
        for pt in &mut jo.points {
            pt[0] += offset;
        }
        if let Err(e) = jo.validate(None) {
            return Err(format!("Invalid jaw_opening calibration: {e}"));
        }
        policy.jaw_opening = Some(jo);
        crate::model::validate_effective_gripper_policy_ex(&policy, gripper_limit, None, false)?;
        Ok(policy)
    }

    /// In-place swap of validated gripper calibration fields on the RT executor thread,
    /// returning replaced heap values so the caller drops them off the RT thread.
    pub(crate) fn swap_gripper_calibration_in_place(
        &mut self,
        offset_rad: f32,
        new_policy: crate::model::GripperPolicy,
        new_jaw: Option<crate::model::JawOpening>,
    ) -> (Option<crate::model::GripperPolicy>, Option<crate::model::JawOpening>) {
        let old_policy = self.config.gripper_policy.replace(new_policy);
        let returned_policy = if self.base_gripper_policy.is_none() {
            self.base_gripper_policy = old_policy;
            None
        } else {
            old_policy
        };
        let old_jaw = match new_jaw {
            Some(j) => self.gripper_calib_jaw_opening.replace(j),
            None => None,
        };
        self.gripper_calib_offset = Some(offset_rad);
        self.gripper_calib_invalid = false;
        self.gripper_calib_revision += 1;
        (returned_policy, old_jaw)
    }

    /// In-place terminal update for a failed or cancelled zero-check action on `armd-control`:
    /// writes inline [`HolderLabel`] `state`/`code` without allocation and reuses preallocated
    /// `msg_buf` (or worker-provided strings), returning any replaced/unused `(code_buf, old_msg)`
    /// for off-thread reclamation.
    pub(crate) fn finish_zero_check_action_in_place(
        &mut self,
        action_id: u64,
        terminal_state: &str,
        code: &str,
        message: &str,
        code_buf: Option<String>,
        msg_buf: Option<String>,
    ) -> (Option<String>, Option<String>) {
        let now = self.now_s();
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == action_id && a.is_running()) {
            a.state = crate::model::HolderLabel::new(terminal_state);
            a.code = Some(crate::model::HolderLabel::new(code));
            let old_msg = match msg_buf {
                Some(mut buf) => {
                    buf.clear();
                    buf.push_str(message);
                    a.message.replace(buf)
                }
                None => {
                    if let Some(existing) = a.message.as_mut() {
                        existing.clear();
                        existing.push_str(message);
                        None
                    } else {
                        a.message = Some(message.into());
                        None
                    }
                }
            };
            a.finished_at = Some(now);
            self.record_terminal_transition_by_id(action_id);
            return (code_buf, old_msg);
        }
        (code_buf, msg_buf)
    }

    /// In-place terminal update writing inline [`HolderLabel`] `state`/`code` and moving
    /// worker-allocated `message` into `ActionStatus` without allocating or freeing on
    /// `armd-control`, returning `(Some(code), old_msg)` for off-thread reclamation.
    pub(crate) fn fail_zero_check_action_owned(
        &mut self,
        action_id: u64,
        code: String,
        message: String,
    ) -> (Option<String>, Option<String>) {
        let now = self.now_s();
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == action_id && a.is_running()) {
            a.state = crate::model::HolderLabel::new("failed");
            a.code = Some(crate::model::HolderLabel::new(&code));
            let old_msg = a.message.replace(message);
            a.finished_at = Some(now);
            self.record_terminal_transition_by_id(action_id);
            return (Some(code), old_msg);
        }
        (Some(code), Some(message))
    }

    pub(crate) fn complete_zero_check_action(
        &mut self,
        action_id: u64,
        durability: crate::rt_core::config_iface::Durability,
        warning: Option<String>,
    ) -> Option<String> {
        let now = self.now_s();
        let mut transitioned_terminal = false;
        let ret = if let Some(a) = self.last_action.as_mut().filter(|a| a.id == action_id) {
            if a.is_running() {
                a.state = crate::model::HolderLabel::new("done");
                a.progress = 1.0;
                a.finished_at = Some(now);
                transitioned_terminal = true;
            }
            match durability {
                crate::rt_core::config_iface::Durability::Durable => {
                    a.durability_confirmed = Some(true);
                    let prev = a.durability_warning.take();
                    warning.or(prev)
                }
                crate::rt_core::config_iface::Durability::DurableUnconfirmed => {
                    a.durability_confirmed = Some(false);
                    match warning {
                        Some(w) => a.durability_warning.replace(w),
                        None => a.durability_warning.take(),
                    }
                }
                crate::rt_core::config_iface::Durability::Volatile => {
                    a.durability_confirmed = Some(false);
                    warning
                }
            }
        } else {
            return warning;
        };
        if transitioned_terminal {
            self.record_terminal_transition_by_id(action_id);
        }
        ret
    }

    /// Zero-allocation terminal update when parent-directory fsync fails after `rename_only()`
    /// for `gripper_zero_check(store_offset=true)`: marks action failed (`PERSIST_FAILED`),
    /// records `durability_confirmed = Some(false)` and the worker-allocated `warning`, and
    /// returns any displaced buffers for off-thread reclamation.
    pub(crate) fn fail_zero_check_dir_sync_in_place(
        &mut self,
        action_id: u64,
        warning: String,
        mut msg_buf: Option<String>,
    ) -> (Option<String>, Option<String>) {
        let now = self.now_s();
        let mut transitioned_terminal = false;
        if let Some(a) = self.last_action.as_mut().filter(|a| a.id == action_id) {
            if a.is_running() {
                a.state = crate::model::HolderLabel::new("failed");
                a.code = Some(crate::model::HolderLabel::new("PERSIST_FAILED"));
                a.finished_at = Some(now);
                transitioned_terminal = true;
            }
            a.durability_confirmed = Some(false);
            let old_warn = a.durability_warning.replace(warning);
            let old_msg = match msg_buf.take() {
                Some(mut b) => {
                    b.clear();
                    const MSG: &str = "Parent directory fsync failed after rename; reconciliation required";
                    if b.capacity() >= MSG.len() {
                        b.push_str(MSG);
                        a.message.replace(b)
                    } else {
                        Some(b)
                    }
                }
                None => None,
            };
            if transitioned_terminal {
                self.record_terminal_transition_by_id(action_id);
            }
            return (old_msg, old_warn);
        }
        (msg_buf, Some(warning))
    }

    /// Infallibly commit a pre-validated [`GripperCalibrationCandidate`] after disk rename.
    pub fn commit_gripper_calibration_candidate(&mut self, cand: GripperCalibrationCandidate) {
        self.config = cand.config;
        self.base_gripper_policy = cand.base_gripper_policy;
        self.gripper_calib_offset = cand.gripper_calib_offset;
        self.gripper_calib_jaw_opening = cand.gripper_calib_jaw_opening;
        self.gripper_calib_invalid = cand.gripper_calib_invalid;
        self.driver_config_error = cand.driver_config_error;
        self.prepare_park_release_spare();
        self.bump_motion_config_revision();
        if !self.rt_executor_owned {
            self.refresh_schema_cache();
        }
    }

    pub fn validate_gripper_calibration_with_driver(
        &mut self,
        offset_rad: Option<f32>,
        jaw_opening: Option<crate::model::JawOpening>,
    ) -> Result<crate::model::GripperPolicy, String> {
        self.build_gripper_calibration_candidate(offset_rad, jaw_opening).map(|(_, p)| p)
    }

    pub fn gripper_calibration_response(&self) -> crate::gripper_calib::GripperCalibrationResponse {
        let policy = self.config.gripper_policy.clone();
        let calibrated = policy.as_ref().is_some_and(|p| p.is_calibrated());
        let jaw_opening = policy.as_ref().and_then(|p| p.jaw_opening.clone());
        crate::gripper_calib::GripperCalibrationResponse {
            offset_rad: self.gripper_calib_offset,
            jaw_opening,
            calibrated,
            policy,
            ts: None,
            stale: self.gripper_calib_stale.clone(),
            revision: self.gripper_calib_revision,
        }
    }

    /// Deterministic SHA-256 digest of the active camera calibration state, or
    /// `EMPTY_CALIBRATION_DIGEST` when uncalibrated.
    #[cfg(feature = "policy-runtime")]
    pub fn active_calibration_digest(&self) -> String { self.calibration_digest.clone() }

    /// [`Self::active_calibration_digest`] without a copy (live policy steps).
    #[cfg(feature = "policy-runtime")]
    pub fn active_calibration_digest_str(&self) -> &str { &self.calibration_digest }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn calibration_digest_of(c: Option<&crate::calibration::CalibrationFile>) -> String {
        match c {
            None => crate::policy_runtime::EMPTY_CALIBRATION_DIGEST.to_string(),
            Some(cal) => {
                let bytes = serde_json::to_vec(cal).unwrap_or_default();
                crate::gravity::compute_sha256(&bytes)
            }
        }
    }
}

/// Candidate ids of configuration changes (increasing within a process run).
static NEXT_CONFIG_CANDIDATE: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

/// How long a configuration proposal stamped at a safe point stays valid. Reservation and
/// activation happen under the same supervisor write guard today, so it only bounds a
/// proposal that is not reserved at once.
pub const CONFIG_PROPOSAL_BUDGET: std::time::Duration = std::time::Duration::from_secs(5);

/// Versioned configuration at the existing commit boundary (track B, [`crate::rt_core::config_iface`]).
///
/// The worker holds the candidate payload (keyed by the reservation's `CandidateId`) and the
/// [`Reservation`](crate::rt_core::config_iface::Reservation) token; it passes the token by
/// value to exactly one of activate or release.
impl SupervisorInner {
    /// Commit context at `now` threading the real non-zero process boot identity
    /// (`BootId::process()`). Live `stop_generation` requires the executor's `StopRequests`
    /// handle ([`Self::config_ctx_with`]); this legacy overload uses `0` until step 1 moves
    /// executor-owned `BootId` and `StopRequests` onto `ArmCore`.
    pub fn config_ctx(&self, now: Instant) -> crate::rt_core::ops::CommitCtx {
        crate::rt_core::ops::CommitCtx {
            arm: self.core.id(),
            boot: crate::rt_core::broker::BootId::process(),
            now,
            stop_generation: 0,
        }
    }

    /// Commit context at `now` threading the executor's real `boot` identity and live urgent
    /// stop generation (`stops.current()`).
    pub fn config_ctx_with(
        &self,
        boot: crate::rt_core::broker::BootId,
        stops: &crate::rt_core::urgent::StopRequests,
        now: Instant,
    ) -> crate::rt_core::ops::CommitCtx {
        crate::rt_core::ops::CommitCtx {
            arm: self.core.id(),
            boot,
            now,
            stop_generation: stops.current(),
        }
    }

    /// A proposal for `domain` built from `expected_revision`, stamped at `ctx` with the
    /// current epoch and `lease_generation` (0: the change needs no lease, e.g. REST
    /// commissioning), valid for [`CONFIG_PROPOSAL_BUDGET`] on the same monotonic clock.
    pub fn config_proposal(&self, domain: crate::rt_core::config_iface::ConfigDomain, expected_revision: u64, lease_generation: u64, ctx: &crate::rt_core::ops::CommitCtx) -> crate::rt_core::config_iface::ConfigProposal {
        use crate::rt_core::broker::{Authority, ClientId, RequestId, Stamp};
        let candidate = NEXT_CONFIG_CANDIDATE.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let authority = Authority { lease_generation, epoch: self.safety.epoch_id(), stop_generation: ctx.stop_generation, config_revision: self.workspace_revision, ..Authority::default() };
        crate::rt_core::config_iface::ConfigProposal {
            domain,
            expected_revision,
            candidate: crate::rt_core::config_iface::CandidateId(candidate),
            stamp: Stamp {
                arm: ctx.arm,
                boot: ctx.boot,
                request: RequestId(candidate),
                client: ClientId(0),
                client_seq: candidate,
                authority,
                deadline: ctx.now + CONFIG_PROPOSAL_BUDGET,
            },
        }
    }

    /// Reserves a pre-stamped `proposal` against a live `ctx` (real boot identity, live stop
    /// generation, monotonic clock), translating refusals to `ArmError`.
    pub fn reserve_config_proposal(
        &mut self,
        proposal: &crate::rt_core::config_iface::ConfigProposal,
        ctx: &crate::rt_core::ops::CommitCtx,
    ) -> Result<crate::rt_core::config_iface::Reservation, ArmError> {
        use crate::rt_core::config_iface::{ConfigAuthority, Reserved};
        match ConfigAuthority::reserve(self, proposal, ctx) {
            Reserved::Granted(r) => Ok(r),
            Reserved::Refused { code: "REVISION_MISMATCH", current } => {
                Err(ArmError::PolicyRevisionMismatch { expected: proposal.expected_revision, current })
            }
            Reserved::Refused { code, .. } => Err(config_refusal(code)),
        }
    }

    /// Reserves `domain` at `expected_revision` now ([`crate::rt_core::config_iface::ConfigAuthority::reserve`]),
    /// with the refusal as the error the existing commit path reports
    /// (`REVISION_MISMATCH` -> `POLICY_REVISION_MISMATCH`, others 409 with the code).
    pub fn reserve_config(&mut self, domain: crate::rt_core::config_iface::ConfigDomain, expected_revision: u64, lease_generation: u64) -> Result<crate::rt_core::config_iface::Reservation, ArmError> {
        let ctx = self.config_ctx(Instant::now());
        let proposal = self.config_proposal(domain, expected_revision, lease_generation, &ctx);
        self.reserve_config_proposal(&proposal, &ctx)
    }

    /// Reserves a pre-stamped `proposal` against live `ctx`, applies the candidate and
    /// activates it (`Volatile`).
    pub fn commit_config_proposal<T>(
        &mut self,
        proposal: &crate::rt_core::config_iface::ConfigProposal,
        ctx: &crate::rt_core::ops::CommitCtx,
        apply: impl FnOnce(&mut Self) -> Result<T, ArmError>,
    ) -> Result<(T, u64), ArmError> {
        let r = self.reserve_config_proposal(proposal, ctx)?;
        self.activate_config_candidate(r, apply)
    }

    /// Reports the reservation's candidate renamed into place
    /// ([`crate::rt_core::config_iface::ConfigAuthority::persisted`]). On refusal the
    /// reservation is released and the error returned.
    pub fn config_persisted(&mut self, r: crate::rt_core::config_iface::Reservation, durability: crate::rt_core::config_iface::Durability) -> Result<crate::rt_core::config_iface::Reservation, ArmError> {
        use crate::rt_core::config_iface::ConfigAuthority;
        match ConfigAuthority::persisted(self, &r, durability) {
            Ok(()) => Ok(r),
            Err(code) => {
                let domain = r.domain();
                let _ = ConfigAuthority::release(self, r);
                Err(ArmError::Internal(format!("{domain:?} configuration persisted outside its reservation: {code}")))
            }
        }
    }

    /// Applies the candidate of the held reservation `r` with `apply` and activates it
    /// ([`crate::rt_core::config_iface::ConfigAuthority::activate`]): the domain moves from
    /// `r.from()` to `r.from() + 1` and the hold ends. When `apply` fails the reservation is
    /// released: the hold ends before persistence and becomes `Reconciling` after it (the
    /// file on disk is not the active state; motion stays inhibited until a restart loads it).
    pub fn activate_config_candidate<T>(
        &mut self,
        r: crate::rt_core::config_iface::Reservation,
        apply: impl FnOnce(&mut Self) -> Result<T, ArmError>,
    ) -> Result<(T, u64), ArmError> {
        use crate::rt_core::config_iface::{Activated, ConfigAuthority};
        let (domain, from) = (r.domain(), r.from());
        if self.core.config_held(domain).is_none_or(|h| h.grant != Some(r.grant())) {
            let _ = ConfigAuthority::release(self, r);
            return Err(ArmError::Internal(format!("{domain:?} configuration activated without its reservation")));
        }
        let value = match apply(self) {
            Ok(v) => v,
            Err(e) => {
                if ConfigAuthority::revision(self, domain) == from {
                    let _ = ConfigAuthority::release(self, r);
                } else {
                    // partly applied: activation ends the hold (or reconciles) at the revision reached
                    if let Activated::Refused { reservation, .. } = ConfigAuthority::activate(self, r) { let _ = ConfigAuthority::release(self, reservation); }
                }
                return Err(e);
            }
        };
        match ConfigAuthority::activate(self, r) {
            Activated::Active { revision, .. } => Ok((value, revision)),
            Activated::Refused { code, reservation } => {
                let _ = ConfigAuthority::release(self, reservation);
                Err(ArmError::Internal(format!("{domain:?} configuration not activated: {code}")))
            }
            Activated::Reconciling { code } => {
                tracing::error!("{domain:?} configuration persisted but not activated ({code}); reconciling");
                Err(ArmError::Internal(format!("{domain:?} configuration persisted but not activated: {code}")))
            }
        }
    }

    /// Reserves `domain` at `expected_revision`, applies the candidate and activates it in
    /// one step (a change with nothing to persist: `Volatile`).
    pub fn commit_config<T>(
        &mut self,
        domain: crate::rt_core::config_iface::ConfigDomain,
        expected_revision: u64,
        lease_generation: u64,
        apply: impl FnOnce(&mut Self) -> Result<T, ArmError>,
    ) -> Result<(T, u64), ArmError> {
        let r = self.reserve_config(domain, expected_revision, lease_generation)?;
        self.activate_config_candidate(r, apply)
    }
}

/// A reservation refusal as the 409 the commit path reports.
fn config_refusal(code: &'static str) -> ArmError {
    ArmError::Rejected {
        status: 409,
        code: code.into(),
        message: match code {
            "CONFIG_BUSY" => "another configuration change is being activated or reconciled".into(),
            "EPOCH_MISMATCH" => "Command epoch is no longer current".into(),
            "LEASE_LOST" => "This connection lost the lease while persisting configuration".into(),
            "STOPPED" => "configuration change overtaken by an authorized stop".into(),
            "WRONG_TARGET" => "configuration proposal targets another arm or stale boot identity".into(),
            "DEADLINE_EXPIRED" => "configuration proposal deadline expired before reservation".into(),
            _ => format!("configuration change refused: {code}"),
        },
    }
}
