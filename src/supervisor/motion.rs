//! Motion commands, stop and leases of the supervisor: jog/quintic/path/joint-goal installs, plan freshness, stop policy, lease acquire/preempt/release, heartbeat and watchdog park (track C).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

impl SupervisorInner {
    /// Report that `reason` shapes the motion now. The same reason keeps its start time;
    /// a fresh limit of higher priority is not replaced by a lower one.
    pub fn note_motion_limit(&mut self, reason: MotionLimitReason, joint: Option<usize>) {
        let now = self.now_s();
        if let Some(cur) = self.motion_limit.as_mut() {
            if cur.reason == reason {
                cur.joint = joint;
                self.motion_limit_seen_s = now;
                return;
            }
            if now - self.motion_limit_seen_s < MOTION_LIMIT_CLEAR_S && cur.reason.priority() > reason.priority() {
                return;
            }
        }
        if reason != MotionLimitReason::Workspace {
            self.clear_workspace_violation_rt();
        }
        self.motion_limit = Some(MotionLimit { reason, joint, since_ms: (now * 1000.0) as u64 });
        self.motion_limit_seen_s = now;
    }

    /// End of tick: keep `blocked` while the hold lasts, clear a limit not seen for
    /// [`MOTION_LIMIT_CLEAR_S`].
    pub(super) fn expire_motion_limit(&mut self) {
        if let Some(h) = self.torque_guard.hold { self.note_motion_limit(MotionLimitReason::Torque, Some(h.joint)); }
        if self.gripper_stalled && self.gripper_force_capped {
            self.note_motion_limit(MotionLimitReason::GripperForce, self.config.gripper_index());
        }
        if self.power.gov.hold.is_some() {
            let j = self.motion_limit.as_ref().filter(|l| l.reason == MotionLimitReason::Power).and_then(|l| l.joint);
            self.note_motion_limit(MotionLimitReason::Power, j);
        }
        if self.servo_blocked_hold {
            #[cfg(feature = "policy-runtime")]
            let policy_active = self.policy_run.is_some();
            #[cfg(not(feature = "policy-runtime"))]
            let policy_active = false;
            let resumed = (self.servo_target.is_some() && !policy_active)
                || self.active_trajectory.is_some()
                || self.macro_run.is_some()
                || self.gravity_comp.is_some()
                || !self.is_torqued
                || self.safety.state().is_stopped();
            if resumed {
                self.clear_blocked_hold();
            } else {
                self.motion_limit_seen_s = self.now_s();
            }
        }
        if self.motion_limit.is_some() && self.now_s() - self.motion_limit_seen_s > MOTION_LIMIT_CLEAR_S {
            self.motion_limit = None;
            self.clear_workspace_violation_rt();
        }
    }

    pub(crate) fn retire_servo_ws_min_depths(&mut self) {
        self.clear_servo_ws_min_depths_rt();
    }

    pub(crate) fn retire_motion_payloads(&mut self) {
        self.release_servo_target();
        self.clear_servo_ws_min_depths_rt();
        let servo_ws_min_depths = None;
        let trajectory = match self.active_trajectory.take() {
            Some(traj) => {
                if self.rt_executor_owned
                    && traj.motion_limits.is_none()
                    && traj.then.is_none()
                {
                    match traj.reference {
                        Reference::Gripper(mut b) => {
                            if self.move_samples_spares.len() < self.move_samples_spares.capacity() {
                                if let Some(samples) = b.take_samples_spare() {
                                    self.recycle_move_samples_rt(samples);
                                }
                            }
                            if traj.action_id.is_none()
                                && b.is_park_release_clean()
                                && self.park_release_spare.is_none()
                            {
                                self.park_release_spare = Some(b);
                                None
                            } else if !b.has_inner_heap_storage() && self.gripper_motion_spare.is_none() {
                                self.gripper_motion_spare = Some(b);
                                None
                            } else {
                                let rebuilt = ActiveTrajectory {
                                    action_id: traj.action_id,
                                    reference: Reference::Gripper(b),
                                    then: None,
                                    span: traj.span,
                                    motion_limits: None,
                                    motion_vel_caps: traj.motion_vel_caps,
                                    motion_max_velocity: traj.motion_max_velocity,
                                };
                                rebuilt.has_heap_storage().then_some(rebuilt)
                            }
                        }
                        reference => {
                            let rebuilt = ActiveTrajectory {
                                action_id: traj.action_id,
                                reference,
                                then: None,
                                span: traj.span,
                                motion_limits: None,
                                motion_vel_caps: traj.motion_vel_caps,
                                motion_max_velocity: traj.motion_max_velocity,
                            };
                            rebuilt.has_heap_storage().then_some(rebuilt)
                        }
                    }
                } else {
                    self.recycle_or_keep_heap_trajectory(traj)
                }
            }
            None => None,
        };
        self.clear_unload_settle();
        let unload_settle = self.unload_settle.take();
        if trajectory.is_some() || unload_settle.is_some() || servo_ws_min_depths.is_some() {
            self.push_retired_stop(crate::rt_core::urgent::RetiredStopPayloads {
                trajectory,
                unload_settle,
                servo_ws_min_depths,
            });
        }
    }

    pub(crate) fn push_retired_stop(&mut self, retired: crate::rt_core::urgent::RetiredStopPayloads) {
        if self.rt_executor_owned
            && self.pending_retired_stops.len() < self.pending_retired_stops.capacity()
        {
            self.pending_retired_stops.push_back(retired);
        } else {
            drop(retired);
        }
    }

    /// Pure read-only preview of torque-guard and power-governor hold clamping without
    /// mutating `self.motion_limit`. Returns `(torque_clamped_joint, power_clamped_joint)`.
    pub(crate) fn preview_hold_filters(&self, targets: &mut [f32]) -> (Option<usize>, Option<usize>) {
        let mut torque_clamped = None;
        if let Some(h) = self.torque_guard.hold {
            if let Some(t) = targets.get_mut(h.joint) {
                if (*t - h.hold_q) * h.sign > 0.0 {
                    *t = h.hold_q;
                    torque_clamped = Some(h.joint);
                }
            }
        }
        let mut power_clamped = None;
        if let Some(h) = self.power.gov.hold {
            for (j, t) in targets.iter_mut().enumerate().take(crate::power::MAX_JOINTS) {
                if h.push[j] != 0.0 && (*t - h.hold_q[j]) * h.push[j] > 0.0 {
                    *t = h.hold_q[j];
                    power_clamped.get_or_insert(j);
                }
            }
        }
        (torque_clamped, power_clamped)
    }

    pub(crate) fn commit_hold_filter_notes(&mut self, torque_clamped: Option<usize>, power_clamped: Option<usize>) {
        if let Some(j) = torque_clamped {
            self.note_motion_limit(MotionLimitReason::Torque, Some(j));
        }
        if let Some(j) = power_clamped {
            self.note_motion_limit(MotionLimitReason::Power, Some(j));
        }
    }

    /// Explicit user release uses the same bounded deceleration as a stream timeout.
    /// In RT executor mode, heap-owning motion payloads are retired off-thread (zero RT frees).
    pub fn stop_motion(&mut self) {
        // a STOP cancels a deferred park continuation and any in-flight ParkRearm before disable
        let cancelled_continuation = self.park_continuation.take();
        if let Some(cont) = cancelled_continuation {
            self.abort_park_rearm_transition();
            self.notify_park_continuation();
            if !self.park_disable_in_flight()
                && self.last_action.as_ref().is_some_and(|a| a.id == cont.action_id && a.is_running())
            {
                self.finish_action(cont.action_id, "cancelled", Some("STOPPED"), None, None, None, None);
            }
        }
        // a STOP cancels a park deferred behind the MIT hand back
        self.mit.deferred_park_off_owner = false;
        if std::mem::take(&mut self.mit.park_after_hand_back) {
            if let Some(id) = self.mit.deferred_park_action.take() {
                let mut msg = self.take_action_msg_buf(id, 32);
                msg.clear();
                msg.push_str("Motion stopped");
                self.finish_action(id, "cancelled", Some("STOPPED"), Some(msg), None, None, None);
            }
        }
        // (the message is built only when a run ends: a stop without one does not allocate)
        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
        if self.policy_load_reservation.take().is_some() {
            self.policy_loading = false;
            self.policy_cancel_slots.set_active_load_token(0);
            self.reconcile_target_stream_authority(Instant::now());
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            self.terminate_policy_run_static("STOPPED", "cancelled", Some("STOPPED"), Some("Motion stopped"));
        }
        if self.gripper_reference_active() { self.hold_gripper_at_measured(); }
        self.servo_settled_ticks.fill(0);
        self.retire_motion_payloads();
        self.pending_deferred_done = None;
        self.pending_normal_unload = false;
        self.zero_check_persisting = None;
        self.end_gripper_stream();
        self.j7_admitted_stamp = None;
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
    }

    pub fn execute_stop(&mut self, policy: StopPolicy) {
        self.apply_stop_policy(policy);
    }

    pub(crate) fn execute_stop_in_tick(&mut self, policy: StopPolicy) {
        self.apply_stop_policy_in_tick(policy, true);
    }

    /// Dispatches or arms `DisableProtective { Thermal }` without re-latching `ThermalTrip` or
    /// churning `epoch_id` (R12 B4: a thermal latch must never suppress its own protective cut).
    pub(crate) fn redispatch_thermal_protective_disable(&mut self) {
        // Cancel any in-flight enable through the reducer before protective thermal cut
        // takes over, preserving the thermal latch (highest severity) and protective cut.
        self.interrupt_enable_for_thermal();
        let already_protective = matches!(
            self.active_disable_kind,
            Some(crate::driver::DisableKind::DisableProtective { .. })
        );
        let thermal_disable = crate::driver::DisableKind::DisableProtective {
            reason: crate::driver::DisableProtectiveReason::Thermal,
        };
        self.active_disable_kind = Some(thermal_disable);
        let rt_needs_redispatch = self.rt_executor_owned
            && self.background_disable_retry.is_none()
            && (self.is_torqued || self.torque_intent == TorqueIntent::On)
            && !self
                .driver
                .as_ref()
                .is_some_and(|d| d.try_with_ref(|drv| drv.torque_transition_pending()).unwrap_or(false));
        // T2 / R12 B4: Upgrading from a user `DisableRequested` to `DisableProtective { Thermal }`
        // (or redispatching after a thermal cut whose retry is not yet armed) MUST bypass
        // the 250 ms throttle so the driver immediately dispatches/arms protective disable.
        if !self.disable_pending
            || !already_protective
            || rt_needs_redispatch
            || (!self.rt_executor_owned
                && self.last_disable_attempt.is_none_or(|t| t.elapsed() >= Duration::from_millis(250)))
        {
            if let Err(e) = self.request_disable_bounded(thermal_disable) {
                if !self.safety.deferred_logging() {
                    tracing::error!("Thermal shutdown not confirmed: {}", e.message.as_str());
                }
            }
        }
    }

    /// Execute the `StopPolicy::EStopLatch` transition without ever blocking on `driver`.
    /// Separated from `self.driver` so tests can verify real multi-thread mutex contention
    /// against the exact production latch path in safe Rust.
    #[doc(hidden)]
    pub fn apply_estop_latch_with_driver(&mut self, driver: Option<&std::sync::Mutex<Box<dyn ArmDriver>>>) {
        self.apply_estop_latch_impl(driver);
    }

    fn apply_estop_latch_impl(&mut self, ext_driver: Option<&std::sync::Mutex<Box<dyn ArmDriver>>>) {
        self.estop_state.latch();
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow
            .enqueue(crate::torque::shadow::ShadowInputKind::Stop {
                kind: crate::torque::StopKind::EStop,
                live_intent: self.shadow_live_intent(),
                live_is_torqued: self.is_torqued,
                live_stopped: true,
            });
        // Immediate rigid lock at current position, latch E-Stop, advance epoch.
        // Maintain active restorative torque hold: do NOT cut power or set is_torqued=false.
        self.pending_normal_unload = false;
        self.watchdog_park_pending = None;
        self.retire_motion_payloads();
        self.pending_deferred_done = None;
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
        self.gripper_stream_contact_detector.reset();
        self.gripper_stream_lease = None;

        // Clear any prior captured stopped-hold targets and recapture fresh (`<= 100 ms`) measured
        // positions through the single owner-snapshot capture kernel (never blocking on a contended
        // driver mutex and never falling back to old cached `commanded_positions`). Also update
        // `commanded_positions` for observation telemetry when the coherent feedback snapshot is
        // fresh and finite. In `DriverSlot::Exclusive`, borrows directly with zero lock; in
        // `DriverSlot::Legacy` / external test mutex, uses non-blocking `try_lock`.
        self.measured_hold.clear();
        const ESTOP_FEEDBACK_MAX_AGE: Duration = Duration::from_millis(250);
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let has_driver = ext_driver.is_some() || self.driver.is_some();
        let (driver_contended, snapshot_fresh) = {
            let mut fb_bounds = std::mem::take(&mut self.scratch.fb.bounds);
            let mut ages_buf = std::mem::take(&mut self.scratch.ages);
            let mut snapshot_out: Option<crate::supervisor::driver_transitions::OwnerDriverSnapshot> = None;
            let mut check_and_capture = |drv: &dyn ArmDriver,
                                         bounds: &mut Vec<Option<Instant>>,
                                         ages: &mut Vec<Option<Duration>>|
             -> bool {
                let now = drv.clock_now();
                let fb = self.core.feedback();
                drv.feedback_queue_bounds_into(bounds);
                let has_bounds = !bounds.is_empty();
                let fresh = feedback_numeric_valid
                    && (0..dof).all(|i| {
                        fb.instant(i)
                            .is_some_and(|t| now.saturating_duration_since(t) <= ESTOP_FEEDBACK_MAX_AGE)
                    })
                    && (!has_bounds
                        || (self.joint_feedback_queue_bounds.len() == dof
                            && self.joint_feedback_queue_bounds.iter().all(|b| {
                                b.is_some_and(|t| now.saturating_duration_since(t) <= ESTOP_FEEDBACK_MAX_AGE)
                            })));
                snapshot_out = Some(crate::supervisor::driver_transitions::OwnerDriverSnapshot::capture_from_driver(
                    drv,
                    dof,
                    feedback_numeric_valid,
                    &self.measured_positions,
                    ages,
                    bounds,
                ));
                fresh
            };
            let res = if let Some(raw) = ext_driver {
                match raw.try_lock() {
                    Err(_) => (true, false),
                    Ok(drv) => (false, check_and_capture(drv.as_ref(), &mut fb_bounds, &mut ages_buf)),
                }
            } else if let Some(slot) = self.driver.as_ref() {
                match slot.try_with_ref(|drv| check_and_capture(drv, &mut fb_bounds, &mut ages_buf)) {
                    Err(_) => (true, false),
                    Ok(fresh) => (false, fresh),
                }
            } else {
                (false, feedback_numeric_valid)
            };
            self.scratch.fb.bounds = fb_bounds;
            self.scratch.ages = ages_buf;
            if let Some(snap) = snapshot_out {
                self.owner_driver_snapshot = snap;
                let _ = self.capture_measured_hold_from_owner();
            } else if res.0 {
                self.owner_driver_snapshot.invalidate();
            }
            res
        };
        if self.measured_positions.iter().all(|p| p.is_finite())
            && self.measured_positions.len() == self.config.dof
            && snapshot_fresh
        {
            crate::rt_core::scratch::copy_into(&mut self.commanded_positions, &self.measured_positions);
        }
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
        if let (Some(g), Some(h)) = (self.config.gripper_index(), self.gripper_motion_hold.as_mut()) {
            if g < self.commanded_positions.len() {
                h.0 = self.commanded_positions[g];
            }
        }

        // Transition floating gravity compensation to Exiting using the freshness-gated
        // hold position (never overwriting pre-stop commands with stale readings and never taking
        // a blocking `d.lock()` in `gravity_comp_write_now`).
        let mut exited_gravity_comp = false;
        if let Some(run) = self.gravity_comp.as_mut() {
            if run.phase.floating() {
                let mut hold = [0.0f32; 6];
                for (dst, &src) in hold.iter_mut().zip(&self.commanded_positions) { *dst = src; }
                run.phase = crate::gravity_comp::GravityCompPhase::Exiting { hold, elapsed_s: 0.0 };
                self.gravity_comp_handback_ready = false;
                run.position = hold;
                for (dst, &src) in run.kp.iter_mut().zip(&run.policy.hold_kp) { *dst = src; }
                for (dst, &src) in run.kd.iter_mut().zip(&run.policy.hold_kd) { *dst = src; }
                self.tracking.invalidate();
                let at = self.start_time.elapsed().as_secs_f64();
                self.safety.log_gravity_comp_estop_exit();
                self.gravity_comp_last_exit = Some(crate::model::GravityCompExit {
                    code: crate::model::HolderLabel::new("ESTOP"),
                    message: crate::model::HolderLabel::new("E-stop"),
                    at,
                });
                exited_gravity_comp = true;
            }
        }

        self.commanded_velocities.fill(0.0);
        if !has_driver {
            self.measured_velocities.fill(0.0);
        }
        // Send the stiff MIT exit hold immediately when uncontended; never block on a held mutex.
        let hold_out = if exited_gravity_comp && !driver_contended && self.is_torqued && !self.disable_pending {
            self.write_gravity_comp_immediate_inner(ext_driver, true, true)
        } else {
            crate::supervisor::TickOutput::None
        };
        self.safety.latch(crate::safety::FaultKind::Estop, crate::safety::FaultDetail::Estop);
        // If an urgent E-stop request is still pending (requested != applied), `pending_stop_check`
        // intentionally returns `EstopInhibited` and defers the hold to the first eligible stopped
        // release; do not log or classify that as a hold write failure.
        let suppress_pending_estop = matches!(
            &hold_out,
            crate::supervisor::TickOutput::EstopInhibited { .. }
        );
        if !suppress_pending_estop {
            if let crate::supervisor::TickOutput::Error { ref message, .. } = hold_out {
                let mut msg = crate::safety::BoundedText::<160>::empty();
                msg.push_str(message.as_str());
                self.safety.log_gravity_comp_hold_write_failed(msg);
            }
            let _ = self.classify_tick_output(hold_out, false, false);
        }
    }

    pub fn apply_stop_policy(&mut self, policy: StopPolicy) {
        self.apply_stop_policy_in_tick(policy, false);
    }

    pub(crate) fn apply_stop_policy_in_tick(&mut self, policy: StopPolicy, in_tick: bool) {
        self.bump_motion_generation();
        #[cfg(not(target_arch = "wasm32"))]
        {
            let reason = match policy {
                StopPolicy::EStopLatch => "ESTOP_ACTIVE",
                _ => "STOPPED",
            };
            self.revoke_sessions_on_stop(reason);
        }
        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
        if self.policy_load_reservation.take().is_some() {
            self.policy_loading = false;
            self.policy_cancel_slots.set_active_load_token(0);
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            match policy {
                StopPolicy::NormalUnload => {
                    self.terminate_policy_run_static("STOPPED", "cancelled", Some("STOPPED"), Some("Arm unloading"));
                }
                StopPolicy::EStopLatch => {
                    self.terminate_policy_run_static("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched"));
                }
                StopPolicy::ThermalTrip => {
                    self.terminate_policy_run_static("THERMAL_TRIP", "failed", Some("THERMAL_TRIP"), Some("Thermal trip"));
                }
                StopPolicy::BusLossFallback => {
                    self.terminate_policy_run_static("BUS_LOSS", "failed", Some("BUS_LOSS"), Some("CAN feedback lost"));
                }
            }
        }
        if policy == StopPolicy::NormalUnload && !self.supports_automatic_unload() {
            tracing::warn!("NormalUnload unsupported: resting pose or confirmed velocity/torque/enable feedback unavailable; no unload motion started");
            return;
        }
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.retire_servo_ws_min_depths();
        self.servo_stream_active = false;
        // M1: keep output history across NormalUnload so park ramps any residual lead out smoothly
        // instead of dropping active lead in a single tick.
        if policy != StopPolicy::NormalUnload {
            self.tracking.invalidate();
        }
        // Hold J7 at measured on non-park stops, except EStopLatch which applies its own
        // non-blocking 250 ms feedback freshness gate before snapping commanded_positions.
        if policy != StopPolicy::NormalUnload && policy != StopPolicy::EStopLatch && self.gripper_reference_active() {
            self.hold_gripper_at_measured();
        }
        match policy {
            StopPolicy::NormalUnload => {
                // Exempt from `Admitted` (Invariant 1a): system-initiated stop-direction park
                // motion (triggered by stop policy / lease watchdog / already-admitted unload)
                // returns the arm to the resting pose and disables torque, rather than starting
                // new client motion.
                if self.handover_settling_since.is_some() {
                    tracing::warn!("NormalUnload rejected while handover settling; maintaining holding torque");
                    return;
                }
                // If in EStop or Fault: strictly forbid planning motion, maintain holding torque if torqued!
                if self.safety.state().is_stopped() || self.e_stop_latched.load(Ordering::SeqCst) {
                    tracing::warn!("NormalUnload rejected in EStop or Fault state; maintaining holding torque");
                    if self.is_torqued {
                        self.pending_normal_unload = false;
                        self.unload_settle = None;
                        self.replace_active_trajectory(None);
                        self.servo_target = None;
                    }
                    return;
                }

                // MIT tracking hands back to confirmed stationary position control before the
                // park. Moving: brake to rest first (the reference stops; the tick ramps the
                // velocity down), hand back at rest, then this park is requested again.
                if self.mit_active() && self.mit_hand_back("park").is_err() {
                    if self.mit_active() && !self.safety.state().is_stopped() {
                        tracing::warn!("Park requested while MIT tracking moves the arm: braking to rest and handing back first");
                        self.replace_active_trajectory(None);
                        self.mit.park_after_hand_back = true;
                    }
                    return;
                }
                // Return to home position cleanly, then disable torque.
                // Allow unload from Operational or Idle.
                let can_unload = self.is_torqued && !self.pending_rearm && (
                    self.safety.state().is_operational()
                    || matches!(self.safety.state(), SafetyState::Idle)
                );
                if can_unload {
                    let home = self.config.resting_positions().to_vec();
                    let clamped_start: Vec<f32> = self
                        .commanded_positions
                        .iter()
                        .enumerate()
                        .map(|(i, &p)| {
                            if i < self.config.limits.len() {
                                let lim = &self.config.limits[i];
                                p.clamp(lim.min_position, lim.max_position)
                            } else {
                                p
                            }
                        })
                        .collect();
                    if let Ok(traj) = TrajectoryPlanner::plan(
                        &clamped_start,
                        &self.commanded_velocities,
                        &home,
                        &vec![0.0; self.config.dof],
                        Some(1.5),
                        &self.planning_limits(),
                    ) {
                        // power budget: slowed to fit, never refused
                        let traj = match self.power_fit_quintic(traj.clone(), &clamped_start, &self.commanded_velocities, &home, &self.planning_limits(), false) {
                            Ok((t, _)) => t,
                            Err(e) => { tracing::warn!("Park power planning failed ({e}); parking as planned"); traj }
                        };
                        // Park checks all enabled workspace constraints: destination resting pose
                        // must be valid, and path must not violate new constraints or worsen
                        // existing violations (monotonic escape with 1 mm jitter tolerance).
                        if let Some((ws, cam)) = self.workspace_policy() {
                            let rest_v = self.config.arm_model.workspace_violation_with_tcp(&home, ws, cam, self.config.tcp_offset);
                            if rest_v.total > crate::kinematics::WORKSPACE_EPS {
                                let e = rest_v.worst.map_or_else(
                                    || SafetyError::from_workspace(crate::chain::WorkspaceFailure::invalid("park", "resting pose violates workspace")),
                                    |w| {
                                        let mut detail = crate::kinematics::WorkspaceViolationDetail {
                                            constraint_id: Some(w.label(ws)),
                                            geometry_id: Some(if w.point == 0 { "end_origin".into() } else { format!("end_point:{}", w.point) }),
                                            violation_phase: Some("park".into()),
                                            reason: Some(format!("resting pose violates workspace ({})", w.label(ws))),
                                        };
                                        if let Some(bi) = w.keepout {
                                            if let Some(kb) = ws.keepout_boxes.get(bi) {
                                                detail.constraint_id = Some(format!("keepout:{}", kb.name));
                                            }
                                        }
                                        SafetyError::WorkspaceViolation {
                                            limit: w.label(ws),
                                            value: w.value as f32,
                                            bound: w.bound as f32,
                                            point: w.point,
                                            detail: Box::new(detail),
                                        }
                                    },
                                );
                                tracing::warn!("Park resting pose violates workspace ({e}); maintaining safe hold");
                                self.park_rejection = Some(e);
                                self.pending_normal_unload = false;
                                self.unload_settle = None;
                                self.replace_active_trajectory(None);
                                return;
                            }
                        }
                        let samples = match self.workspace_samples_with_tol(&traj, crate::kinematics::WORKSPACE_PARK_ESCAPE_EPS) {
                            Ok(s) => s,
                            Err(e) => {
                                let e = e.with_workspace_phase("park");
                                tracing::warn!("Park path cannot be verified against the workspace ({e}); maintaining safe hold");
                                self.park_rejection = Some(e);
                                self.pending_normal_unload = false;
                                self.unload_settle = None;
                                self.replace_active_trajectory(None);
                                return;
                            }
                        };
                        if let Err(e) = self.check_workspace_path_with_tol(&clamped_start, samples.iter().map(Vec::as_slice), crate::kinematics::WORKSPACE_PARK_ESCAPE_EPS) {
                            let e = e.with_workspace_phase("park");
                            tracing::warn!("Park path violates workspace ({e}); maintaining safe hold");
                            self.park_rejection = Some(e);
                            self.pending_normal_unload = false;
                            self.unload_settle = None;
                            self.replace_active_trajectory(None);
                            return;
                        }
                        let _ = self.safety.set_torque(true);
                        let _ = self.safety.mark_motion_started();
                        self.unload_planned_duration = Some(traj.duration);
                        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, None));
                        self.target_positions = home;
                        self.pending_normal_unload = true;
                        self.unload_settle = None;
                        self.park_gripper_contact = false;
                        self.park_release = None;
                        let now = Instant::now();
                        self.last_heartbeat = now;
                        self.last_motion_frame = now;
                        return;
                    }
                }
                // If the arm is torqued or torque_intent is On, maintain holding torque if motion
                // cannot be planned or arm is not operational! NEVER cut power or bypass the rest
                // gate under gravity when motion planning fails or arm is not operational.
                if self.is_torqued || self.torque_intent == crate::supervisor::TorqueIntent::On {
                    tracing::warn!("NormalUnload cannot plan motion or arm not operational; maintaining holding torque to prevent dropping arm");
                    self.pending_normal_unload = false;
                    self.unload_settle = None;
                    self.replace_active_trajectory(None);
                    self.servo_target = None;
                    return;
                }
                // Already cleanly disarmed (`!is_torqued && torque_intent == Off`): clean up motion state
                // without issuing an ungated hardware disable.
                self.pending_normal_unload = false;
                self.unload_settle = None;
                self.is_torqued = false;
                self.tracking.invalidate();
                self.replace_active_trajectory(None);
                self.reset_gripper_stall();
                self.commanded_velocities.fill(0.0);
                if self.driver.is_none() {
                    self.measured_velocities.fill(0.0);
                }
                let _ = self.safety.set_torque(false);
                self.mit_torque_off();
            }
            StopPolicy::EStopLatch => {
                self.apply_estop_latch_impl(None);
            }
            StopPolicy::ThermalTrip => {
                // Protective-cut exception (decision b, 2026-11-24): thermal trip is NOT a user
                // torque-off request; it intentionally bypasses `may_disable_torque_while_stopped`
                // and disables the motors in any pose (the arm may drop).
                self.stop_motion();
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                self.thermal_cooldown.reset();
                let (joint_idx, temperature) = self.temperatures.iter().copied().enumerate()
                    .filter(|(_, t)| t.is_finite()).max_by(|a,b| a.1.total_cmp(&b.1)).unwrap_or((0, 0.0));
                self.safety.latch(
                    crate::safety::FaultKind::ThermalTrip,
                    crate::safety::FaultDetail::ThermalTrip {
                        joint_idx,
                        temperature,
                        limit: self.config.max_temperature,
                    },
                );
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow
                    .enqueue(crate::torque::shadow::ShadowInputKind::Thermal {
                        channel: (joint_idx.min(crate::torque::MAX_CHANNELS - 1)) as u8,
                        temperature_c: temperature,
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: true,
                    });
                self.redispatch_thermal_protective_disable();
            }
            StopPolicy::BusLossFallback => {
                self.exit_gravity_comp_inner("BUS_LOSS", "CAN feedback lost", !in_tick);
                // Bus loss / disconnect: hold position locked, latch fault, advance epoch
                // Maintain holding torque: DO NOT disable hardware torque to prevent gravitational collapse!
                self.pending_normal_unload = false;
                self.retire_motion_payloads();
                self.pending_deferred_done = None;
                self.gripper_stream_target = None;
                self.gripper_stream_contact = None;
                self.gripper_stream_contact_detector.reset();
                self.gripper_stream_lease = None;
                if self.measured_positions.iter().all(|p| p.is_finite())
                    && self.measured_positions.len() == self.config.dof
                {
                    crate::rt_core::scratch::copy_into(&mut self.commanded_positions, &self.measured_positions);
                }
                crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
                self.commanded_velocities.fill(0.0);
                if self.driver.is_none() {
                    self.measured_velocities.fill(0.0);
                }
                if self.bus_loss_instant.is_none() {
                    self.bus_loss_instant = Some(Instant::now());
                }
                self.safety.latch(crate::safety::FaultKind::BusLoss, crate::safety::FaultDetail::BusLoss);
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let multi_device_gap = self
                        .torque_shadow
                        .state()
                        .is_some_and(|s| s.config.device_count > 1);
                    self.torque_shadow
                        .enqueue(crate::torque::shadow::ShadowInputKind::BusLoss {
                            device: 0,
                            multi_device_gap,
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: true,
                        });
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        if !self.rt_executor_owned {
            self.flush_torque_shadow();
        }
    }

    pub fn preempt_lease(&mut self, conn: &str, label: &str) -> Result<(u64, Option<(String, u64)>), PreemptError> {
        if self.detached {
            return Err(PreemptError::OwnerTransferred);
        }
        let now = Instant::now();
        if let Some(ref active) = self.active_lease {
            let is_expired = active.generation <= self.fenced_lease_generation
                || active.acquired_at.elapsed() >= self.lease_timeout;
            if active.client_id == conn && !is_expired {
                let gen = active.generation;
                self.authorize_servo_stream_for_committed_lease(gen);
                return Ok((gen, None));
            }

            if active.client_id != conn {
                if let Some(last) = self.last_preemption {
                    let elapsed = last.elapsed();
                    if elapsed < PREEMPT_COOLDOWN {
                        let remaining_ms = (PREEMPT_COOLDOWN.saturating_sub(elapsed).as_millis() as u64).max(1);
                        return Err(PreemptError::Cooldown(remaining_ms));
                    }
                }
            }
        }

        self.cancel_watchdog_park_for_new_lease(conn);

        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            self.terminate_policy_run_static("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease preempted"));
        }

        // Decision-based hardware disabled check (never inferred from feedback telemetry).
        let confirmed_hardware_disabled = !self.is_torqued
            && !self.disable_pending
            && self.safety.state() == &SafetyState::Idle
            && (self.driver.is_none() || self.torque_intent == crate::supervisor::TorqueIntent::Off);

        let old_gen = self.active_lease.as_ref().map_or(0, |a| a.generation);
        let needs_stop = !(confirmed_hardware_disabled
            || self.safety.state().is_stopped()
            || self.is_fully_settled_armed_hold());
        if needs_stop {
            self.handover_settling_since = Some(now);
        } else {
            self.handover_settling_since = None;
        }
        if old_gen != 0 {
            self.fence_displaced_lease_generation(old_gen, needs_stop);
        } else if needs_stop {
            self.stop_motion();
        }

        let evicted = self.active_lease.as_ref().filter(|a| a.client_id != conn).map(|a| (a.client_id.clone(), a.generation));

        self.lease_generation_counter += 1;
        let gen = self.lease_generation_counter;
        self.set_active_lease_in_place(0, conn, label, gen, now);
        self.authorize_servo_stream_for_committed_lease(gen);
        self.last_heartbeat = now;
        if let Some((ref ev_conn, ev_gen)) = evicted {
            self.last_preemption = Some(now);
            self.set_last_evicted_in_place(ev_conn, ev_gen);
        }

        if !self.rt_executor_owned {
            tracing::info!("Preempted lease for connection {} (label {}), generation {}, evicted: {:?}", conn, label, gen, evicted);
        }
        Ok((gen, evicted))
    }

    pub fn acquire_lease(&mut self, client_id: &str) -> Result<u64, (String, u64)> {
        if self.detached {
            return Err(("OWNER_TRANSFERRED".into(), 0));
        }
        let now = Instant::now();
        if let Some(ref active) = self.active_lease {
            let is_expired = active.generation <= self.fenced_lease_generation
                || active.acquired_at.elapsed() >= self.lease_timeout;
            if !is_expired {
                if active.client_id == client_id {
                    let gen = active.generation;
                    self.authorize_servo_stream_for_committed_lease(gen);
                    return Ok(gen);
                }
                return Err((active.client_id.clone(), active.generation));
            }
        }
        // A new controller never inherits an expired controller's queued motion,
        // nor a watchdog park started after the previous lease was released.
        self.cancel_watchdog_park_for_new_lease(client_id);
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            self.terminate_policy_run_static("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease acquired by new controller"));
        }
        if let Some(old_gen) = self.active_lease.as_ref().map(|a| a.generation) {
            self.fence_displaced_lease_generation(old_gen, true);
        }
        self.lease_generation_counter += 1;
        let gen = self.lease_generation_counter;
        self.set_active_lease_in_place(0, client_id, "", gen, now);
        self.authorize_servo_stream_for_committed_lease(gen);
        Ok(gen)
    }

    /// A new lease owner takes over from a running watchdog park: the park's
    /// motion is cancelled like any other queued motion and the arm brakes to a hold.
    pub(crate) fn cancel_watchdog_park_for_new_lease(&mut self, client_id: &str) {
        let had_pending = self.watchdog_park_pending.take().is_some();
        if self.watchdog_park.take().is_some() && (self.pending_normal_unload || had_pending) {
            if !self.rt_executor_owned {
                tracing::info!("Watchdog park cancelled: new lease acquired by {client_id}; holding position");
            }
            self.stop_motion();
        }
    }

    /// Budget for a watchdog park, in control-loop time: deceleration of the physical arm
    /// from its measured velocity (max over joints of |v| / planning acceleration), the
    /// planned park duration (which already starts from the commanded velocity), the
    /// settle hold and [`WATCHDOG_PARK_MARGIN`]; never less than [`WATCHDOG_PARK_TIMEOUT`].
    /// Zero heap allocations (uses inline planning limits when within inline DOF bound).
    pub fn watchdog_park_timeout(&self) -> Duration {
        let decel = if let Some((limits, n)) = self.planning_limits_inline(None) {
            self.measured_velocities.iter().zip(&limits[..n])
                .filter(|(v, l)| v.is_finite() && l.max_acceleration > 1e-6)
                .map(|(v, l)| f64::from(v.abs() / l.max_acceleration))
                .fold(0.0, f64::max)
        } else {
            self.measured_velocities.iter().zip(&self.config.limits)
                .filter(|(v, l)| v.is_finite() && l.max_acceleration > 1e-6)
                .map(|(v, l)| f64::from(v.abs() / l.max_acceleration))
                .fold(0.0, f64::max)
        };
        let planned = self.unload_planned_duration
            .filter(|d| d.is_finite() && *d >= 0.0)
            .map_or(Duration::ZERO, |d| Duration::from_secs_f64((decel + d).min(3600.0)) + UNLOAD_SETTLE_HOLD + WATCHDOG_PARK_MARGIN);
        planned.max(WATCHDOG_PARK_TIMEOUT)
    }

    /// Advance a running watchdog park by `dt`. When the budget runs out with the arm at the
    /// resting pose and only the stationary check pending, the budget is extended once by
    /// [`WATCHDOG_PARK_SETTLE_GRACE`]; otherwise torque is held and WatchdogTimeout latched.
    pub(super) fn supervise_watchdog_park(&mut self, dt: f64) {
        let Some(mut park) = self.watchdog_park.take() else { return; };
        if let Some(held) = self.core.config().held() {
            let msg = if held.state == crate::rt_core::config_iface::HoldState::Reconciling {
                "configuration reconciliation active"
            } else {
                "configuration hold active"
            };
            let reason = crate::safety::BoundedText::new(msg);
            self.watchdog_park_pending = None;
            self.watchdog_park_blocked_reason = Some(crate::model::HolderLabel::new(msg));
            self.safety.log_watchdog_park_blocked(reason);
            self.stop_motion();
            if self.is_torqued {
                self.latch_watchdog_fault();
            }
            return;
        }
        if self.calibration_busy {
            let msg = "calibration active";
            let reason = crate::safety::BoundedText::new(msg);
            self.watchdog_park_pending = None;
            self.watchdog_park_blocked_reason = Some(crate::model::HolderLabel::new(msg));
            self.safety.log_watchdog_park_blocked(reason);
            self.stop_motion();
            if self.is_torqued {
                self.latch_watchdog_fault();
            }
            return;
        }
        if !self.pending_normal_unload && self.watchdog_park_pending.is_none() {
            if !self.rt_executor_owned {
                tracing::warn!("Watchdog park interrupted (safety state {:?})", self.safety.state());
            }
            if self.is_torqued {
                self.latch_watchdog_fault();
            }
            return;
        }
        if self.watchdog_park_pending.is_some()
            && (!self.is_torqued || self.safety.state().is_stopped() || self.e_stop_latched.load(Ordering::SeqCst))
        {
            self.watchdog_park_pending = None;
            return;
        }
        // budget in reference time: the power governor may slow the park
        if dt.is_finite() && dt > 0.0 { park.elapsed_s += dt * self.motion_time_scale(); }
        if park.elapsed_s < park.budget_s {
            self.watchdog_park = Some(park);
            return;
        }
        if self.watchdog_park_pending.take().is_some() {
            self.mit.park_after_hand_back = false;
            self.mit.deferred_park_off_owner = false;
            if !self.rt_executor_owned {
                tracing::warn!("Watchdog park off-thread plan timed out after {:.2} s; holding torque and latching WatchdogTimeout", park.budget_s);
            }
            self.safety.log_watchdog_park_blocked(crate::safety::BoundedText::new("watchdog park plan timed out"));
            self.latch_watchdog_fault();
            return;
        }
        if self.rt_executor_owned {
            let at_pose = self.active_trajectory.is_none() && self.is_resting_pose_static_ok();
            if at_pose && !park.grace_granted {
                park.budget_s += WATCHDOG_PARK_SETTLE_GRACE.as_secs_f64();
                park.grace_granted = true;
                self.watchdog_park = Some(park);
                return;
            }
            let reason = self.unload_settle_blocker_static();
            self.safety.log_watchdog_park_blocked(crate::safety::BoundedText::new(reason));
            self.latch_watchdog_fault();
            return;
        }
        let blocker = self.unload_settle_blocker().unwrap_or_else(|| "settle pending".into());
        let at_pose = self.active_trajectory.is_none() && self.resting_pose_static_violation().is_none();
        if at_pose && !park.grace_granted {
            tracing::warn!("Watchdog park at the resting pose after {:.2} s but not settled ({blocker}); waiting up to {:?} more",
                park.elapsed_s, WATCHDOG_PARK_SETTLE_GRACE);
            park.budget_s += WATCHDOG_PARK_SETTLE_GRACE.as_secs_f64();
            park.grace_granted = true;
            self.watchdog_park = Some(park);
            return;
        }
        tracing::warn!("Watchdog park did not finish within {:.2} s ({blocker}); holding torque and latching WatchdogTimeout", park.budget_s);
        self.latch_watchdog_fault();
    }

    /// Allocation-free static reason code for a pending unload settle on `armd-control`.
    fn unload_settle_blocker_static(&mut self) -> &'static str {
        if self.active_trajectory.is_some() {
            "park trajectory still running"
        } else if !self.is_resting_pose_static_ok() {
            "resting pose static check failed"
        } else {
            "settle stationary check pending"
        }
    }

    /// Zero-allocation bounded check of conditions for parking automatically when the lease
    /// watchdog expires. Returns the reason the park is not allowed, or `None` when it is.
    pub(super) fn watchdog_park_blocker_bounded(&self) -> Option<crate::safety::BoundedText<160>> {
        use crate::safety::BoundedText;
        if let Some(held) = self.core.config().held() {
            let msg = if held.state == crate::rt_core::config_iface::HoldState::Reconciling {
                "configuration reconciliation active"
            } else {
                "configuration hold active"
            };
            return Some(BoundedText::new(msg));
        }
        if self.calibration_busy {
            return Some(BoundedText::new("calibration active"));
        }
        let Some(policy) = &self.config.resting_pose else {
            return Some(BoundedText::new("no resting pose configured"));
        };
        if !self.supports_automatic_unload() {
            return Some(BoundedText::new("driver lacks velocity/torque/enable feedback"));
        }
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Some(BoundedText::new("E-stop latched"));
        }
        if self.collision.hold.is_some() {
            return Some(BoundedText::new("collision hold"));
        }
        if let Some(h) = self.torque_guard.hold {
            let rest = self.config.resting_positions().get(h.joint).copied().unwrap_or(h.hold_q);
            if (rest - h.anchor_q) * h.sign > crate::torque_guard::TORQUE_PUSH_IN_RAD {
                let mut msg = BoundedText::<160>::empty();
                let _ = core::fmt::write(
                    &mut msg,
                    format_args!("torque hold on joint {}: the resting pose lies in the push direction", h.joint + 1),
                );
                return Some(msg);
            }
        }
        if !self.safety.state().is_operational() {
            let mut msg = BoundedText::<160>::empty();
            let bstate = crate::safety::BoundedSafetyState::from_state(self.safety.state());
            let _ = core::fmt::write(&mut msg, format_args!("safety state {bstate:?}"));
            return Some(msg);
        }
        if self.pending_rearm || self.disable_pending || self.output_suspended || self.handover_settling_since.is_some() {
            return Some(BoundedText::new("re-arm, disable, output suspension or handover pending"));
        }
        if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) {
            let mut msg = BoundedText::<160>::empty();
            let _ = core::fmt::write(&mut msg, format_args!("feedback older than {} ms", policy.feedback_max_age_ms));
            return Some(msg);
        }
        None
    }

    /// Conditions for parking automatically when the lease watchdog expires.
    /// Returns the reason the park is not allowed, or `None` when it is.
    #[allow(dead_code)]
    pub(super) fn watchdog_park_blocker(&self) -> Option<String> {
        self.watchdog_park_blocker_bounded().map(|b| b.to_string())
    }

    /// Previous watchdog behaviour: rigid hold at the measured (or commanded) position
    /// and latch Fault(WatchdogTimeout). Torque stays on. Performs zero heap allocations.
    pub(crate) fn latch_watchdog_fault(&mut self) {
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            self.terminate_policy_run_static("WATCHDOG_TIMEOUT", "failed", Some("WATCHDOG_TIMEOUT"), Some("Lease watchdog expired"));
        }
        if !self.safety.state().is_stopped() {
            // Preserve a torque-guard backed-off command before updating commanded_positions in place.
            let torque_hold = self.torque_guard.hold.and_then(|h| {
                self.commanded_positions.get(h.joint).copied().map(|c| (h.joint, c))
            });
            if self.measured_positions.iter().all(|p| p.is_finite())
                && self.measured_positions.len() == self.config.dof
            {
                crate::rt_core::scratch::copy_into(&mut self.commanded_positions, &self.measured_positions);
            }
            if let Some((j, c)) = torque_hold {
                if let Some(slot) = self.commanded_positions.get_mut(j) {
                    *slot = c;
                }
            }
            self.watchdog_park = None;
            self.watchdog_park_pending = None;
            self.pending_normal_unload = false;
            self.retire_motion_payloads();
            self.pending_deferred_done = None;
            self.gripper_stream_target = None;
            self.gripper_stream_contact = None;
            self.gripper_stream_contact_detector.reset();
            self.gripper_stream_lease = None;
            self.servo_settled_ticks.fill(0);
            self.clear_servo_ws_min_depths_rt();
            self.servo_stream_active = false;
            self.tracking.invalidate();
            crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
            self.commanded_velocities.fill(0.0);
            self.commanded_accelerations.fill(0.0);
        }
        self.safety.latch(
            crate::safety::FaultKind::WatchdogTimeout,
            crate::safety::FaultDetail::WatchdogTimeout,
        );
    }

    /// Drops the lease if `client_id`/`generation` own it. Returns true only when the
    /// lease was still valid: callers stop motion on true. An expired lease is dropped
    /// without that, because its expiry may already have started a watchdog park that a
    /// client without a valid lease must not cancel.
    pub fn release_lease(&mut self, client_id: &str, generation: u64) -> bool {
        if self.detached {
            return false;
        }
        if let Some(ref active) = self.active_lease {
            if active.client_id == client_id && active.generation == generation {
                let valid = active.generation > self.fenced_lease_generation
                    && active.acquired_at.elapsed() < self.lease_timeout;
                let displaced = self.clear_active_lease_in_place().unwrap_or(generation);
                self.fence_displaced_lease_generation(displaced, false);
                #[cfg(feature = "policy-runtime")]
                if self.policy_run.is_some() {
                    self.terminate_policy_run_static("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease released"));
                }
                return valid;
            }
        }
        false
    }

    /// Releases whichever lease `client_id` currently holds under the owner lock, regardless of
    /// generation (used by WS teardown so a concurrent acquire cannot outlive connection close).
    pub fn release_lease_for_connection(&mut self, client_id: &str) -> bool {
        if self.detached {
            return false;
        }
        let Some(gen) = self
            .active_lease
            .as_ref()
            .filter(|active| active.client_id == client_id)
            .map(|active| active.generation)
        else {
            return false;
        };
        self.release_lease(client_id, gen)
    }

    /// Stops active motion FIRST if `client_id`/`generation` currently owns a valid lease, then
    /// releases that lease (matching normal WS `release_lease` semantics). An expired lease is
    /// dropped without cancelling a running watchdog park.
    pub fn release_lease_and_stop(&mut self, client_id: &str, generation: u64) -> bool {
        if self.detached {
            return false;
        }
        let is_valid_owner = self.active_lease.as_ref().is_some_and(|a| {
            a.client_id == client_id
                && a.generation == generation
                && a.acquired_at.elapsed() < self.lease_timeout
        });
        if is_valid_owner {
            self.stop_motion();
        }
        self.release_lease(client_id, generation)
    }

    /// Stops active motion FIRST if `client_id` currently owns a valid lease (any generation),
    /// then releases that lease under the owner lock.
    pub fn release_lease_for_connection_and_stop(&mut self, client_id: &str) -> bool {
        if self.detached {
            return false;
        }
        let Some(gen) = self
            .active_lease
            .as_ref()
            .filter(|active| active.client_id == client_id)
            .map(|active| active.generation)
        else {
            return false;
        };
        self.release_lease_and_stop(client_id, gen)
    }

    /// Records the label a lease holder sent with `acquire_lease` (shown to other clients).
    pub fn set_lease_label(&mut self, client_id: &str, generation: u64, label: &str) {
        if self.detached {
            return;
        }
        if let Some(active) = self.active_lease.as_mut().filter(|a| a.client_id == client_id && a.generation == generation) {
            active.label = label.to_string();
        }
    }

    /// Label of the valid lease holder (empty string when it sent none); None when no lease is held.
    pub fn lease_holder(&self) -> Option<String> {
        self.lease_holder_str().map(str::to_owned)
    }

    /// [`Self::lease_holder`] without the copy.
    pub fn lease_holder_str(&self) -> Option<&str> {
        self.active_lease
            .as_ref()
            .filter(|a| {
                a.generation > self.fenced_lease_generation
                    && a.acquired_at.elapsed() < self.lease_timeout
            })
            .map(|a| a.label.as_str())
    }

    /// Why `client_id` no longer holds `generation`: "preempted" when a forced acquire
    /// evicted exactly that lease, otherwise "expired" (heartbeats stopped arriving).
    pub fn lease_loss_reason(&self, client_id: &str, generation: u64) -> &'static str {
        match &self.last_evicted {
            Some((conn, gen)) if conn == client_id && *gen == generation => "preempted",
            _ => "expired",
        }
    }

    pub fn is_lease_active(&self) -> bool {
        if let Some(ref active) = self.active_lease {
            active.generation > self.fenced_lease_generation
                && active.acquired_at.elapsed() < self.lease_timeout
        } else {
            false
        }
    }

    pub fn heartbeat(&mut self, client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        if self.detached {
            return Err(SafetyError::OwnerTransferred);
        }
        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            });
        }
        if let Some(ref mut active) = self.active_lease {
            if active.client_id == client_id
                && active.generation > self.fenced_lease_generation
                && active.acquired_at.elapsed() < self.lease_timeout
            {
                active.acquired_at = Instant::now();
                self.last_heartbeat = Instant::now();
            } else if !self.rt_executor_owned {
                tracing::debug!(
                    "Heartbeat ignored from non-lease owner '{}' (lease owner is '{}')",
                    client_id, active.client_id
                );
            }
        }
        // NOTE: When there is no active lease, do NOT refresh last_heartbeat.
        // Watchdog can only be refreshed by the authenticated active lease owner.
        Ok(())
    }

    /// Planned-motion limits in fixed stack storage (no `Vec` or joint-name `String`
    /// allocations). Returns `None` if `config.limits.len() > MAX_MOTION_DOF`.
    pub(crate) fn planning_limits_inline(&self, speed_scale: Option<f64>) -> Option<([crate::model::JointLimit; crate::model::MAX_MOTION_DOF], usize)> {
        let dof = self.config.limits.len();
        if dof > crate::model::MAX_MOTION_DOF { return None; }
        let scale = self.config.speed_cap.and_then(|c| c.joint_velocity_scale).unwrap_or(1.0);
        let gripper_idx = self.config.gripper_index();
        let commissioning = self.config.active_tool.commissioning_required;
        let mut out: [crate::model::JointLimit; crate::model::MAX_MOTION_DOF] = std::array::from_fn(|_| crate::model::JointLimit {
            name: String::new(),
            min_position: 0.0,
            max_position: 0.0,
            max_velocity: 0.0,
            max_acceleration: 0.0,
        });
        for (i, src) in self.config.limits.iter().enumerate() {
            let mut max_velocity = src.max_velocity;
            if Some(i) != gripper_idx {
                if let Some(cap) = self.config.planned_max_velocity {
                    max_velocity = max_velocity.min(cap);
                }
                max_velocity *= scale;
            }
            if let Some(s) = speed_scale {
                if Some(i) != gripper_idx || commissioning {
                    max_velocity *= s as f32;
                }
            }
            out[i] = crate::model::JointLimit {
                name: String::new(),
                min_position: src.min_position,
                max_position: src.max_position,
                max_velocity,
                max_acceleration: src.max_acceleration,
            };
        }
        Some((out, dof))
    }

    /// Zero-allocation workspace sweep check of a joint-space quintic trajectory using
    /// reused evaluator buffers (`ws_eval_bufs`) and stack-only adaptive subdivision.
    /// Falls back to [`Self::workspace_samples`] + [`Self::check_workspace_path`] only when
    /// `dof > INLINE_POLYS` or `arm_dof > MAX_CHAIN_JOINTS`.
    pub(crate) fn check_workspace_quintic_inline(&mut self, traj: &MultiJointQuinticTrajectory) -> Result<(), SafetyError> {
        if self.workspace_policy().is_none() {
            return Ok(());
        }
        if traj.polynomials.len() > crate::trajectory::INLINE_POLYS
            || self.config.arm_model.arm_dof() > crate::chain::MAX_CHAIN_JOINTS
        {
            let samples = self.workspace_samples(traj)?;
            return self.check_workspace_path(&self.commanded_positions, samples.iter().map(Vec::as_slice));
        }
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let result = (|| {
            let Some((ws, cam)) = self.workspace_policy() else { return Ok(()); };
            let eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
                &self.config.arm_model,
                ws,
                cam,
                self.config.tcp_offset,
                &mut bufs,
            )
            .with_escape_tol(crate::kinematics::WORKSPACE_EPS);
            let checked = eval
                .check_quintic_inline(traj, MAX_KEEPOUT_EVALS_PER_PLAN, "sweep")
                .map_err(SafetyError::from_workspace);
            eval.recycle(&mut bufs);
            checked
        })();
        self.ws_eval_bufs = bufs;
        result
    }

    pub fn jog(admitted: Admitted<'_>, target_positions: Vec<f32>, duration_secs: Option<f64>, epoch_id: Option<u64>) -> Result<u64, ArmError> {
        Self::jog_action(admitted, target_positions, duration_secs, epoch_id, "joint_move").map(|(epoch, _, _)| epoch)
    }

    /// Jog that records an action of `kind` (`joint_move` or `home`).
    /// Returns (epoch, action id, planned duration).
    pub fn jog_action(admitted: Admitted<'_>, target_positions: Vec<f32>, duration_secs: Option<f64>, epoch_id: Option<u64>, kind: &str) -> Result<(u64, u64, f64), ArmError> {
        Self::jog_action_with_speed_scale(admitted, target_positions, duration_secs, epoch_id, kind, None)
    }

    pub fn jog_action_with_speed_scale(admitted: Admitted<'_>, target_positions: Vec<f32>, duration_secs: Option<f64>, epoch_id: Option<u64>, kind: &str, speed_scale: Option<f64>) -> Result<(u64, u64, f64), ArmError> {
        let (this, proof) = admitted.consume()?;
        this.jog_action_with_speed_scale_inner(&proof, target_positions, duration_secs, epoch_id, kind, speed_scale)
    }

    fn jog_action_with_speed_scale_inner(&mut self, proof: &crate::supervisor::AdmittedProof, target_positions: Vec<f32>, duration_secs: Option<f64>, epoch_id: Option<u64>, kind: &str, speed_scale: Option<f64>) -> Result<(u64, u64, f64), ArmError> {
        if let Some(s) = speed_scale {
            if !s.is_finite() || !(0.05..=1.0).contains(&s) {
                return Err(MotionError::InvalidCommand("speed_scale must be in [0.05, 1.0]".into()).into());
            }
        }
        if self.config.active_tool.commissioning_required {
            match speed_scale {
                Some(s) if s <= 0.2 + 1e-6 && kind != "home" => {}
                _ => return Err(SafetyError::CommissioningRequired(format!("{kind} (requires speed_scale <= 0.2)")).into()),
            }
        }
        let mut limits = self.planning_limits();
        if let Some(s) = speed_scale {
            let gripper_idx = self.config.gripper_index();
            let commissioning = self.config.active_tool.commissioning_required;
            for (i, l) in limits.iter_mut().enumerate() {
                if Some(i) != gripper_idx || commissioning {
                    l.max_velocity *= s as f32;
                }
            }
        }
        if self.config.active_tool.commissioning_required {
            if let (Some(g_idx), Some(policy)) = (self.config.gripper_index(), self.config.gripper_policy.as_ref()) {
                if let (Some(&target_g), Some(&cur_g)) = (target_positions.get(g_idx), self.commanded_positions.get(g_idx)) {
                    let close_dir = if policy.closed_rad >= policy.open_rad { 1.0_f32 } else { -1.0_f32 };
                    if (target_g - cur_g) * close_dir > 1e-4 && policy.max_close_torque_nm.is_some() {
                        if let Some(ref d) = self.driver {
                            let has_torque_fb = d.with_ref(|drv| drv.capabilities().torque_feedback).unwrap_or(false);
                            if !has_torque_fb {
                                return Err(MotionError::Unsupported(
                                    "Gripper close during commissioning jog requires torque_feedback when max_close_torque_nm is configured".into(),
                                ).into());
                            }
                        }
                    }
                }
            }
        }
        self.start_quintic(proof, target_positions, duration_secs, epoch_id, kind, None, &limits)
    }

    /// Joint-space quintic to `target_positions` planned with `limits`, recorded as an
    /// action of `kind`. Returns (epoch, action id, planned duration).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn start_quintic(&mut self, proof: &crate::supervisor::AdmittedProof, target_positions: Vec<f32>, duration_secs: Option<f64>, epoch_id: Option<u64>, kind: &str, tag: Option<String>, limits: &[crate::model::JointLimit]) -> Result<(u64, u64, f64), ArmError> {
        self.start_quintic_prepared(proof, target_positions, duration_secs, epoch_id, kind, tag, limits, None)
    }

    /// [`start_quintic`](Self::start_quintic) that installs `prepared` (planned, power-fitted
    /// and workspace-checked without the lock) when it was made from the same start, target,
    /// limits, workspace revision, tool offset and power budget; else plans and checks here.
    /// Every other check runs here either way.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn start_quintic_prepared(&mut self, _proof: &crate::supervisor::AdmittedProof, target_positions: Vec<f32>, duration_secs: Option<f64>, epoch_id: Option<u64>, kind: &str, tag: Option<String>, limits: &[crate::model::JointLimit], prepared: Option<PreparedQuintic>) -> Result<(u64, u64, f64), ArmError> {
        self.check_terminal_fifo_for_new_action()?;
        let mut target_positions = target_positions;
        self.config_motion_check()?;
        self.gravity_comp_motion_busy()?;
        // jog, home, move_to_pose, backlash approach: MIT must own its joints when requested
        if let Some(reason) = self.mit_motion_blocker() { return Err(MotionError::MotionBusy(reason).into()); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading {
            return Err(MotionError::MotionBusy("a policy is loading".into()).into());
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            return Err(MotionError::MotionBusy("a policy run is active".into()).into());
        }
        self.collision_hold_check()?;
        if self.handover_settling_since.is_some() {
            return Err(ArmError::Safety(SafetyError::HandoverSettling));
        }

        if self.shutdown_committed || self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }

        if self.pending_rearm || self.pending_normal_unload {
            return Err(ArmError::Safety(SafetyError::NotArmed(self.safety.state().clone())));
        }
        if self.disable_pending || self.torque_transition.is_some() || self.park_continuation.is_some() {
            return Err(MotionError::MotionBusy("a torque transition or park is pending".into()).into());
        }
        if self.macro_run.as_ref().is_some_and(|m| m.finishing) {
            return Err(MotionError::MotionBusy("macro finishing hold pending bus acceptance".into()).into());
        }
        self.pending_normal_unload = false;

        // 1. Validate command against single controller authority, safety state, and soft limits
        self.safety.validate_motion_command(
            epoch_id,
            &target_positions,
            &self.config.limits,
        )?;
        if self.config.gripper_index().is_some_and(|g| {
            target_positions.len() > g
                && self.commanded_positions.len() > g
                && (target_positions[g] - self.commanded_positions[g]).abs() > 1e-4
        }) {
            self.check_pos_force_gripper_fresh()?;
        }
        self.torque_hold_filter(&mut target_positions);
        self.power_hold_filter(&mut target_positions);

        // Gripper stall release check on reverse command
        if let Some(g_idx) = self.config.gripper_index() {
            if self.gripper_stalled && target_positions.len() > g_idx && self.commanded_positions.len() > g_idx {
                let delta = target_positions[g_idx] - self.commanded_positions[g_idx];
                let tau_g = self.torques.get(g_idx).copied().unwrap_or(0.0);
                if (delta * tau_g < -1e-4) || (delta * self.gripper_stall_sign < -1e-4) {
                    self.gripper_stalled = false;
                    self.gripper_high_torque_ticks = 0;
                    self.gripper_low_torque_ticks = 0;
                    self.gripper_stall_window.reset();
                }
            }
        }

        let reuse = prepared.filter(|p| {
            duration_secs.is_none()
                && p.start == self.commanded_positions
                && p.v0 == self.commanded_velocities
                && p.target == target_positions
                && p.limits.as_slice() == limits
                && p.workspace_revision == self.workspace_revision
                && p.tcp_offset == self.config.tcp_offset
                && p.power.as_ref() == self.power_budget()
                && p.checked_workspace == self.workspace_policy().is_some()
        });
        let (traj, power_scale) = if let Some(p) = reuse {
            self.prepared_quintics_used += 1;
            (p.traj, p.power_scale)
        } else {
            // 2. Plan quintic polynomial trajectory from commanded state to target
            let zero_buf = [0.0f32; crate::model::MAX_MOTION_DOF];
            let zero_vec;
            let zeros: &[f32] = if self.config.dof <= zero_buf.len() {
                &zero_buf[..self.config.dof]
            } else {
                zero_vec = vec![0.0; self.config.dof];
                &zero_vec
            };
            let traj = TrajectoryPlanner::plan(
                &self.commanded_positions,
                &self.commanded_velocities,
                &target_positions,
                zeros,
                duration_secs,
                limits,
            )?;
            // 2.1 Power budget: slow the move to fit (or refuse with POWER_BUDGET); the workspace
            // check below runs on the trajectory that will be executed
            let (traj, power_scale) = self.power_fit_quintic(traj, &self.commanded_positions, &self.commanded_velocities, &target_positions, limits, true)?;
            let samples = self.workspace_samples(&traj)?;
            self.check_workspace_path(&self.commanded_positions, samples.iter().map(Vec::as_slice))?;
            (traj, power_scale)
        };

        // 3. Only renew lease AFTER command validation and planning succeed
        self.last_heartbeat = Instant::now();

        if self.config.gripper_index().is_some_and(|g| (target_positions[g] - self.commanded_positions[g]).abs() > 1e-4) {
            self.gripper_hold_vel_ceiling = None;
        }
        self.release_servo_target();
        self.clear_servo_ws_min_depths_rt();
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &target_positions);
        self.safety.mark_motion_started()?;
        let duration = traj.duration;
        let id = self.begin_action(kind, tag);
        if let Some(s) = power_scale { self.note_action_power_scale(id, s); }
        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, Some(id)).with_motion_limits(limits.to_vec()));

        Ok((self.safety.epoch_id(), id, duration))
    }

    /// Why the workspace rule may not change now: a motion, macro, stream or park is
    /// running (each was checked against the current rule). Zero-allocation static variant.
    pub(crate) fn workspace_change_blocker_static(&self) -> Option<&'static str> {
        if self.macro_run.is_some() { return Some("a macro action is running"); }
        if self.pending_normal_unload || self.park_continuation.is_some() || self.park_disable_in_flight() {
            return Some("a park is running");
        }
        self.busy_reason_static()
    }

    /// Same as [`Self::workspace_change_blocker_static`], excluding `self.calibration_busy`
    /// for checks made by the active calibration transaction that already owns `calibration_busy`.
    pub(crate) fn workspace_change_blocker_for_calibration_static(&self) -> Option<&'static str> {
        if self.macro_run.is_some() { return Some("a macro action is running"); }
        if self.pending_normal_unload || self.park_continuation.is_some() || self.park_disable_in_flight() {
            return Some("a park is running");
        }
        if self.macro_run.as_ref().is_some_and(|m| m.finishing) {
            return Some("macro finishing hold pending bus acceptance");
        }
        if self.zero_check_persisting.is_some() { return Some("gripper calibration is being persisted"); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading { return Some("a policy is loading"); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() { return Some("a policy run is active"); }
        if self.active_trajectory.is_some() { return Some("a trajectory is running"); }
        if self.servo_target.is_some() { return Some("servo streaming is active"); }
        if self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) { return Some("the arm is still moving"); }
        None
    }

    pub(crate) fn busy_reason_static(&self) -> Option<&'static str> {
        if self.macro_run.as_ref().is_some_and(|m| m.finishing) {
            return Some("macro finishing hold pending bus acceptance");
        }
        if self.calibration_busy { return Some("a calibration apply is running"); }
        if self.zero_check_persisting.is_some() { return Some("gripper calibration is being persisted"); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading { return Some("a policy is loading"); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() { return Some("a policy run is active"); }
        if self.active_trajectory.is_some() { return Some("a trajectory is running"); }
        if self.servo_target.is_some() { return Some("servo streaming is active"); }
        if self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) { return Some("the arm is still moving"); }
        None
    }

    /// Why the workspace rule may not change now: a motion, macro, stream or park is
    /// running (each was checked against the current rule).
    pub fn workspace_change_blocker(&self) -> Option<String> {
        self.workspace_change_blocker_static().map(String::from)
    }

    /// Zero-allocation predicate equivalent to `self.busy_reason().is_some()`.
    pub(crate) fn has_busy_reason(&self) -> bool {
        if self.calibration_busy || self.zero_check_persisting.is_some() {
            return true;
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading || self.policy_run.is_some() {
            return true;
        }
        self.active_trajectory.is_some()
            || self.servo_target.is_some()
            || self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3)
    }

    pub(super) fn busy_reason(&self) -> Option<String> {
        self.busy_reason_static().map(String::from)
    }

    /// Snapshot for planning a Cartesian action outside the lock (default [`crate::motion::GoalFrame::EndLink`]).
    pub fn plan_input(&self) -> Result<PlanInput, ArmError> {
        self.plan_input_in(crate::motion::GoalFrame::EndLink)
    }

    /// Snapshot for planning a public TCP Cartesian action (`move_to`, `move_relative`, `place`).
    pub fn plan_input_tcp(&self) -> Result<PlanInput, ArmError> {
        self.plan_input_in(crate::motion::GoalFrame::Tcp)
    }

    /// Snapshot for planning a Cartesian action in `goal_frame`.
    pub fn plan_input_in(&self, goal_frame: crate::motion::GoalFrame) -> Result<PlanInput, ArmError> {
        if self.config.kinematics.is_none() || !self.config.has_cartesian_with_gripper() {
            return Err(MotionError::Unsupported("Cartesian actions need a kinematics model and an arm with gripper".into()).into());
        }
        self.action_preconditions()?;
        self.gravity_comp_motion_busy()?;
        let arm_dof = self.config.arm_dof();
        let gripper_idx = self.config.gripper_index().unwrap_or(arm_dof);
        Ok(PlanInput {
            q0: crate::kinematics::q6(&self.commanded_positions[..arm_dof.min(self.commanded_positions.len())]),
            j7: self.commanded_positions.get(gripper_idx).copied().unwrap_or(0.0),
            limits: self.planning_limits(),
            speed_cap: self.config.speed_cap,
            ws: self.workspace_policy().map(|(ws, _)| ws.clone()),
            cam: self.config.camera_mount.clone(),
            arm: self.config.arm_model.clone(),
            epoch: self.safety.epoch_id(),
            workspace_revision: self.workspace_revision,
            power: self.power_budget().cloned(),
            tcp_offset: self.config.tcp_offset,
            goal_frame,
        })
    }

    /// Install a planned path if nothing changed since `input` was taken.
    /// Returns (action id, expected duration).
    pub fn install_path(admitted: Admitted<'_>, input: &PlanInput, path: JointPath, kind: &str, tag: Option<String>) -> Result<(u64, f64), ArmError> {
        let (this, proof) = admitted.consume()?;
        this.install_path_inner(&proof, input, path, kind, tag)
    }

    pub(crate) fn install_path_inner(&mut self, proof: &crate::supervisor::AdmittedProof, input: &PlanInput, path: JointPath, kind: &str, tag: Option<String>) -> Result<(u64, f64), ArmError> {
        self.check_terminal_fifo_for_new_action()?;
        self.check_plan_fresh(input)?;
        let duration = self.start_path_motion_at_inner(proof, input.epoch, &path)?;
        let id = self.begin_action(kind, tag);
        if input.power.is_some() { self.note_action_power_scale(id, path.power_scale); }
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Path(PathTrajectory::new(path))));
        Ok((id, duration + 0.5))
    }

    /// The checks `check_plan_fresh` makes of a plan input, for a prepared path whose start
    /// and workspace revision the commit already checked: the epoch, and an idle arm.
    pub(crate) fn check_prepared_fresh(&self, epoch: u64) -> Result<(), ArmError> {
        if self.safety.epoch_id() != epoch {
            return Err(MotionError::PlanStale("epoch changed while planning".into()).into());
        }
        match self.action_preconditions() {
            Err(ArmError::Motion(MotionError::MotionBusy(reason))) => Err(MotionError::PlanStale(format!("arm no longer idle ({reason})")).into()),
            other => other,
        }
    }

    /// Zero-allocation structural validation of a prepared Cartesian path in a plan slot.
    pub(crate) fn validate_prepared_joint_path(&self, dof: usize, path: &crate::rt_core::ops::PreparedPath, duration_s: f64)
        -> Result<(), &'static str> {
        let s = path.params();
        let ok = path.len() >= 2 && dof <= 7 && dof == self.config.dof && duration_s.is_finite() && duration_s > 0.0
            && s.first() == Some(&0.0) && s.last() == Some(&1.0) && s.windows(2).all(|w| w[1] >= w[0]);
        if !ok { return Err("INVALID_COMMAND"); }
        Ok(())
    }

    /// Zero-allocation equivalent of [`Self::check_prepared_fresh`] preserving identical
    /// refusal precedence and static codes.
    pub(crate) fn check_prepared_fresh_static(&self, epoch: u64) -> Result<(), &'static str> {
        if self.safety.epoch_id() != epoch {
            return Err("PLAN_STALE");
        }
        if self.handover_settling_since.is_some() { return Err("REJECTED"); }
        if self.e_stop_latched.load(Ordering::SeqCst) { return Err("ESTOP_ACTIVE"); }
        match self.safety.state() {
            SafetyState::EStop => return Err("ESTOP_ACTIVE"),
            SafetyState::Fault(_) => return Err("FAULT_ACTIVE"),
            s if !s.is_operational() => return Err("NOT_ARMED"),
            _ => {}
        }
        if self.pending_rearm || self.pending_normal_unload {
            return Err("NOT_ARMED");
        }
        if self.collision.hold.is_some() { return Err("COLLISION_HOLD"); }
        if self.core.motion_inhibited_by_config() { return Err("CONFIG_BUSY"); }
        if self.macro_run.as_ref().is_some_and(|m| m.finishing) { return Err("MOTION_BUSY"); }
        if self.mit_motion_blocked() { return Err("PLAN_STALE"); }
        if self.calibration_busy
            || self.zero_check_persisting.is_some()
            || self.active_trajectory.is_some()
            || self.servo_target.is_some()
            || self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3)
        {
            return Err("PLAN_STALE");
        }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading || self.policy_run.is_some() {
            return Err("PLAN_STALE");
        }
        Ok(())
    }

    /// Installs a prepared Cartesian path borrowing directly from an immutable pinned
    /// [`crate::rt_core::broker::PlanSlotRef`] as a `move_to` action (zero allocations and
    /// zero frees on both accepted and refused commits). Returns `(action id, expected duration)`.
    pub(crate) fn install_prepared_path_slot(admitted: Admitted<'_>, epoch: u64, dof: usize, slot: crate::rt_core::broker::PlanSlotRef<crate::rt_core::ops::PlanSlot>, duration_s: f64, power_scale: Option<f64>) -> Result<(u64, f64), &'static str> {
        let (this, proof) = admitted
            .consume()
            .map_err(|e| crate::rt_core::ops::static_code(e.into()))?;
        this.check_terminal_fifo_for_new_action()
            .map_err(|e| crate::rt_core::ops::static_code(e.into()))?;
        this.validate_prepared_joint_path(dof, &slot.path, duration_s)?;
        this.check_prepared_fresh_static(epoch)?;
        let last = &slot.path.samples()[slot.path.len() - 1][..dof];
        let duration = this
            .start_path_motion_slice_inner(&proof, epoch, last, duration_s)
            .map_err(crate::rt_core::ops::static_code)?;
        let id = this.begin_action_rt("move_to");
        let scale = power_scale.unwrap_or(1.0);
        if power_scale.is_some() { this.note_action_power_scale(id, scale); }
        let arm_dof = this.config.arm_dof();
        this.active_trajectory = Some(ActiveTrajectory::new(
            Some(id),
            Reference::Path(PathTrajectory::from_plan_slot(slot, dof, arm_dof, duration_s)),
        ));
        Ok((id, duration + 0.5))
    }

    /// The [`JointPath`] of a prepared Cartesian path (`dof` joints, samples at the slot's
    /// path parameters): `INVALID_COMMAND` for fewer than two samples, more joints than a
    /// path sample holds, or parameters that do not run from 0 to 1 without going back.
    /// Fallback used by `CoreOps::commit_prepared(&PreparedPath)` when no pinned slot is passed.
    pub(crate) fn prepared_joint_path(&self, dof: usize, path: &crate::rt_core::ops::PreparedPath, duration_s: f64, power_scale: Option<f64>)
        -> Result<JointPath, &'static str> {
        self.validate_prepared_joint_path(dof, path, duration_s)?;
        let s = path.params();
        let q = path.samples().iter().map(|x| { let mut a = [0.0f32; 7]; a[..dof].copy_from_slice(&x[..dof]); a }).collect();
        Ok(JointPath { dof, arm_dof: self.config.arm_dof(), q, s: s.to_vec(), duration: duration_s, power_scale: power_scale.unwrap_or(1.0) })
    }

    /// Installs a prepared Cartesian path as a `move_to` action (the RT commit: no IK).
    /// Returns (action id, expected duration).
    pub(crate) fn install_prepared_path(admitted: Admitted<'_>, epoch: u64, path: JointPath, power_noted: bool) -> Result<(u64, f64), ArmError> {
        let (this, proof) = admitted.consume()?;
        this.check_terminal_fifo_for_new_action()?;
        this.check_prepared_fresh(epoch)?;
        let duration = this.start_path_motion_at_inner(&proof, epoch, &path)?;
        let id = this.begin_action("move_to", None);
        if power_noted { this.note_action_power_scale(id, path.power_scale); }
        this.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Path(PathTrajectory::new(path))));
        Ok((id, duration + 0.5))
    }

    /// Joint-space goal seeds besides the current joints ([`crate::motion::goal_seeds`]): the
    /// profile home and the resting pose (when it differs), arm joints only.
    pub fn goal_seed_poses(&self) -> Vec<[f64; 6]> {
        let arm_dof = self.config.arm_dof();
        let q6 = |q: &[f32]| crate::kinematics::q6(&q[..arm_dof.min(q.len())]);
        let mut seeds = vec![q6(&self.config.home_positions)];
        let rest = q6(self.config.resting_positions());
        if rest != seeds[0] { seeds.push(rest); }
        seeds
    }

    /// `move_to` / `move_relative` with `path: "joint"`: the joint-space quintic of
    /// `move_to_pose` from the commanded joints to the arm joints `q` (gripper kept), with the
    /// arm joints' planning velocity times `speed_scale`. The quintic is checked like every joint
    /// move (swept workspace, keep-out boxes, link capsules, power budget, torque and collision
    /// holds). `same_side`: end with the same-side approach of `move_to_precise`.
    /// Returns (action id, expected duration).
    #[allow(clippy::too_many_arguments)]
    pub fn install_joint_goal(admitted: Admitted<'_>, input: &PlanInput, q: &[f64; 6], speed_scale: f64, kind: &str, tag: Option<String>,
        same_side: Option<crate::backlash::Request>) -> Result<(u64, f64), ArmError> {
        Self::install_joint_goal_prepared(admitted, input, q, speed_scale, kind, tag, same_side, None)
    }

    /// [`install_joint_goal`](Self::install_joint_goal) with the quintic prepared without the
    /// lock ([`prepare_quintic`]); used when its inputs still match.
    #[allow(clippy::too_many_arguments)]
    pub fn install_joint_goal_prepared(admitted: Admitted<'_>, input: &PlanInput, q: &[f64; 6], speed_scale: f64, kind: &str, tag: Option<String>,
        same_side: Option<crate::backlash::Request>, prepared: Option<PreparedQuintic>) -> Result<(u64, f64), ArmError> {
        let (this, proof) = admitted.consume()?;
        this.check_plan_fresh(input)?;
        let target = this.joint_goal_target(q);
        let limits = this.joint_goal_limits(speed_scale);
        let (_, id, duration) = match same_side {
            Some(request) => this.start_quintic_same_side(&proof, target, Some(input.epoch), kind, tag, &limits, "move_to_precise", request)?,
            None => this.start_quintic_prepared(&proof, target, None, Some(input.epoch), kind, tag, &limits, prepared)?,
        };
        Ok((id, duration))
    }

    /// What [`prepare_quintic`] needs from the supervisor for a `path: "joint"` move: the
    /// commanded joints and velocities and the quintic's limits (taken under the lock).
    pub fn joint_goal_snapshot(&self, speed_scale: f64) -> JointGoalSnapshot {
        JointGoalSnapshot {
            start: self.commanded_positions.clone(),
            v0: self.commanded_velocities.clone(),
            limits: self.joint_goal_limits(speed_scale),
            arm_dof: self.config.arm_dof(),
            gripper_index: self.config.gripper_index(),
        }
    }

    /// Whether the joint-space quintic to the arm joints `q` would pass the checks of
    /// [`start_quintic`](Self::start_quintic) that depend on the path (soft limits, swept
    /// workspace); nothing starts. Used for the `path: "joint"` hint of a failed straight line.
    pub fn joint_goal_feasible(&self, q: &[f64; 6]) -> bool {
        let target = self.joint_goal_target(q);
        let limits = self.planning_limits();
        if target.iter().zip(&self.config.limits).any(|(t, l)| !t.is_finite() || *t < l.min_position || *t > l.max_position) {
            return false;
        }
        let Ok(traj) = crate::trajectory::TrajectoryPlanner::plan(
            &self.commanded_positions, &vec![0.0; self.config.dof], &target, &vec![0.0; self.config.dof], None, &limits) else { return false; };
        match self.workspace_samples(&traj) {
            Ok(samples) => self.check_workspace_path(&self.commanded_positions, samples.iter().map(Vec::as_slice)).is_ok(),
            Err(_) => false,
        }
    }

    /// The commanded joints with the arm joints replaced by `q` (gripper and extra joints kept).
    pub(crate) fn joint_goal_target(&self, q: &[f64; 6]) -> Vec<f32> {
        joint_goal_target_from(&self.commanded_positions, q, self.config.arm_dof(), self.config.gripper_index())
    }

    /// `planning_limits()` with the non-gripper velocities times `speed_scale` (as `move_to_pose`).
    pub(crate) fn joint_goal_limits(&self, speed_scale: f64) -> Vec<crate::model::JointLimit> {
        let gripper_idx = self.config.gripper_index();
        let mut limits = self.planning_limits();
        for (i, l) in limits.iter_mut().enumerate() {
            if Some(i) != gripper_idx { l.max_velocity *= speed_scale as f32; }
        }
        limits
    }

    /// PLAN_STALE checks shared by every installed plan.
    pub(crate) fn check_plan_fresh(&self, input: &PlanInput) -> Result<(), ArmError> {
        if self.safety.epoch_id() != input.epoch {
            return Err(MotionError::PlanStale("epoch changed while planning".into()).into());
        }
        if self.workspace_revision != input.workspace_revision {
            return Err(MotionError::PlanStale("workspace or keep-out boxes changed while planning".into()).into());
        }
        match self.action_preconditions() {
            Err(ArmError::Motion(MotionError::MotionBusy(reason))) => {
                return Err(MotionError::PlanStale(format!("arm no longer idle ({reason})")).into());
            }
            other => other?,
        }
        let arm_dof = input.arm.arm_dof();
        let gripper_idx = self.config.gripper_index().unwrap_or(arm_dof);
        if crate::motion::plan_start_moved(&self.commanded_positions, &input.q0[..arm_dof.min(6)], gripper_idx, input.j7) {
            return Err(MotionError::PlanStale("commanded positions changed while planning".into()).into());
        }
        Ok(())
    }

    // -- RT commit (armd 1.6 RT core, track C) ------------------------------------------

    /// Feedback a commit may plan from: numerically valid, and fresh per the driver (no driver: simulated).
    /// On RT executor ownership, uses the Step-2 cached poll state with zero extra driver locks.
    pub fn commit_feedback_valid(&self) -> bool {
        if !self.feedback_numeric_valid {
            return false;
        }
        if self.driver.is_none() {
            return true;
        }
        if self.rt_executor_owned {
            return self.last_poll_feedback_valid;
        }
        self.read_driver_for_tick()
            .is_some_and(|drv| drv.is_feedback_valid())
    }

    /// The refusal code when a primitive admitted under `admitted` may not be installed,
    /// beyond the stamp and authority checks of the commit (identity, deadline, stop, epoch,
    /// a lease valid at the commit instant, configuration revision, parent, prepared start):
    /// the configuration authority's active workspace revision (`PLAN_STALE`), no
    /// configuration reservation inhibiting motion (`CONFIG_BUSY`) and valid feedback
    /// (`FEEDBACK_STALE`). None: the install's own checks decide. Changes nothing.
    pub fn motion_commit_blocker(&self, admitted: &crate::rt_core::broker::Authority) -> Option<&'static str> {
        use crate::rt_core::config_iface::{ConfigAuthority, ConfigDomain};
        if ConfigAuthority::revision(self, ConfigDomain::Workspace) != admitted.config_revision { return Some("PLAN_STALE"); }
        if self.core.motion_inhibited_by_config() { return Some("CONFIG_BUSY"); }
        if !self.commit_feedback_valid() { return Some("FEEDBACK_STALE"); }
        None
    }

    /// Legacy motion admission (`action_preconditions`, `start_quintic_prepared`): a
    /// configuration reservation or reconciliation inhibits motion (`CONFIG_BUSY`), as the RT
    /// commit and the planning snapshot's `config_inhibited` do.
    pub(crate) fn config_motion_check(&self) -> Result<(), ArmError> {
        if !self.core.motion_inhibited_by_config() { return Ok(()); }
        Err(ArmError::Rejected { status: 409, code: "CONFIG_BUSY".into(),
            message: "a configuration change is being activated or reconciled; motion is refused until it ends".into() })
    }

    /// An authorized stop (the RT's urgent-stop generation advanced): ends client motion
    /// like [`Self::stop_motion`] (plus gravity-comp exit and macro cancellation), except a
    /// running watchdog park. The park belongs to the lease watchdog, not to the client that
    /// stopped: only a new lease ([`Self::acquire_lease`], [`Self::preempt_lease`]) takes it
    /// over. Returns false when the park was kept.
    ///
    /// RT-safe: when keeping a watchdog park, records fields into [`crate::rt_core::ops::stop_diag`]
    /// without tracing or allocating; a normal-priority thread formats off-thread.
    pub fn authorized_stop(&mut self) -> bool {
        if let Some(park) = self.watchdog_park.as_ref().filter(|_| self.pending_normal_unload || self.watchdog_park_pending.is_some()) {
            crate::rt_core::ops::stop_diag().record_park_ignored(self.core.id(), park.elapsed_s, park.budget_s);
            return false;
        }
        if self.is_watchdog_park_disable_in_flight() {
            crate::rt_core::ops::stop_diag().record_park_ignored(self.core.id(), 0.0, 0.0);
            return false;
        }
        if self.gravity_comp.is_some() {
            self.exit_gravity_comp("STOPPED", "Stop command");
        }
        let disable_in_flight = self.park_disable_in_flight();
        let running_park_id = self
            .last_action
            .as_ref()
            .filter(|a| a.is_running() && a.kind == "park")
            .map(|a| a.id);
        self.stop_motion();
        if !disable_in_flight {
            if let Some(id) = running_park_id {
                if self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running()) {
                    self.finish_action(id, "cancelled", Some("STOPPED"), None, None, None, None);
                }
            }
        }
        if self.macro_run.is_some() {
            self.request_macro_stop();
        }
        self.advance_stop_generation();
        self.abort_legacy_enable_on_authorized_stop();
        true
    }

    /// Validates `last` under safety epoch `epoch` and activates path motion without allocating
    /// or freeing (`target_positions` is updated in place via `copy_into`). Consumes a single-use
    /// [`Admitted`] proof by value.
    #[doc(hidden)]
    pub fn start_path_motion_slice(admitted: Admitted<'_>, epoch: u64, last: &[f32], duration: f64) -> Result<f64, ArmError> {
        let (this, proof) = admitted.consume()?;
        this.start_path_motion_slice_inner(&proof, epoch, last, duration)
    }

    pub(crate) fn start_path_motion_slice_inner(&mut self, _proof: &crate::supervisor::AdmittedProof, epoch: u64, last: &[f32], duration: f64) -> Result<f64, ArmError> {
        self.safety.validate_motion_command(Some(epoch), last, &self.config.limits)?;
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.clear_servo_ws_min_depths_rt();
        crate::rt_core::scratch::copy_into(&mut self.target_positions, last);
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        Ok(duration)
    }

    /// [`Self::start_path_motion_slice_inner`] for a path planned under safety epoch `epoch`.
    pub(crate) fn start_path_motion_at_inner(&mut self, proof: &crate::supervisor::AdmittedProof, epoch: u64, path: &JointPath) -> Result<f64, ArmError> {
        self.start_path_motion_slice_inner(proof, epoch, path.last_slice(), path.duration)
    }
}
