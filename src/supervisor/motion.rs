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
            self.workspace_violation = None;
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
            self.workspace_violation = None;
        }
    }

    /// Explicit user release uses the same bounded deceleration as a stream timeout.
    pub fn stop_motion(&mut self) {
        // a STOP cancels a park deferred behind the MIT hand back
        if std::mem::take(&mut self.mit.park_after_hand_back) {
            if let Some(id) = self.mit.deferred_park_action.take() {
                self.finish_action(id, "cancelled", Some("STOPPED"), Some("Motion stopped".into()), None, None, None);
            }
        }
        // (the message is built only when a run ends: a stop without one does not allocate)
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() {
            self.terminate_policy_run("STOPPED", "cancelled", Some("STOPPED"), Some("Motion stopped".into()));
        }
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.active_trajectory = None;
        self.pending_normal_unload = false;
        self.unload_settle = None;
        self.zero_check_persisting = None;
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
    }

    pub fn execute_stop(&mut self, policy: StopPolicy) {
        self.apply_stop_policy(policy);
    }

    /// Execute the `StopPolicy::EStopLatch` transition without ever blocking on `driver`.
    /// Separated from `self.driver` so tests can verify real multi-thread mutex contention
    /// against the exact production latch path in safe Rust.
    #[doc(hidden)]
    pub fn apply_estop_latch_with_driver(&mut self, driver: Option<&std::sync::Mutex<Box<dyn ArmDriver>>>) {
        self.e_stop_latched.store(true, Ordering::SeqCst);
        // Immediate rigid lock at current position, latch E-Stop, advance epoch.
        // Maintain active restorative torque hold: do NOT cut power or set is_torqued=false.
        self.pending_normal_unload = false;
        self.unload_settle = None;
        self.active_trajectory = None;
        self.servo_target = None;

        // Lock onto measured_positions only when finite and the supervisor's own coherent
        // feedback snapshot (`core.feedback()` + `joint_feedback_queue_bounds`, captured
        // alongside `measured_positions`) is provably within the 250 ms freshness bound
        // against the driver's clock. Use non-blocking `try_lock` so E-stop never waits on
        // a contended driver mutex; if contended, poisoned, stale, or unbounded, hold
        // `commanded_positions`.
        const ESTOP_FEEDBACK_MAX_AGE: Duration = Duration::from_millis(250);
        let (driver_contended, snapshot_fresh) = match driver {
            None => (false, self.feedback_numeric_valid),
            Some(d) => match d.try_lock() {
                Err(_) => (true, false),
                Ok(drv) => {
                    let now = drv.clock_now();
                    let dof = self.config.dof;
                    let fb = self.core.feedback();
                    let fresh = self.feedback_numeric_valid
                        && (0..dof).all(|i| {
                            fb.instant(i).is_some_and(|t| now.saturating_duration_since(t) <= ESTOP_FEEDBACK_MAX_AGE)
                        })
                        && {
                            drv.feedback_queue_bounds_into(&mut self.scratch.fb.bounds);
                            if self.scratch.fb.bounds.is_empty() {
                                true
                            } else {
                                self.joint_feedback_queue_bounds.len() == dof
                                    && self.joint_feedback_queue_bounds.iter().all(|b| {
                                        b.is_some_and(|t| now.saturating_duration_since(t) <= ESTOP_FEEDBACK_MAX_AGE)
                                    })
                            }
                        };
                    (false, fresh)
                }
            },
        };
        let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
            && self.measured_positions.len() == self.config.dof
            && snapshot_fresh
        {
            self.measured_positions.clone()
        } else {
            self.commanded_positions.clone()
        };

        // Transition floating gravity compensation to Exiting using the freshness-gated
        // `hold_pos` (never overwriting pre-stop commands with stale readings and never taking
        // a blocking `d.lock()` in `gravity_comp_write_now`).
        let mut exited_gravity_comp = false;
        if let Some(run) = self.gravity_comp.as_mut() {
            if run.phase.floating() {
                let mut hold = [0.0f32; 6];
                for (dst, &src) in hold.iter_mut().zip(&hold_pos) { *dst = src; }
                run.phase = crate::gravity_comp::GravityCompPhase::Exiting { hold, elapsed_s: 0.0 };
                run.position = hold;
                for (dst, &src) in run.kp.iter_mut().zip(&run.policy.hold_kp) { *dst = src; }
                for (dst, &src) in run.kd.iter_mut().zip(&run.policy.hold_kd) { *dst = src; }
                self.tracking.invalidate();
                let at = self.start_time.elapsed().as_secs_f64();
                tracing::warn!("Gravity compensation exit: ESTOP E-stop");
                self.gravity_comp_last_exit = Some(crate::model::GravityCompExit {
                    code: "ESTOP".into(),
                    message: "E-stop".into(),
                    at,
                });
                exited_gravity_comp = true;
            }
        }

        self.target_positions = hold_pos.clone();
        self.commanded_positions = hold_pos;
        self.commanded_velocities = vec![0.0; self.config.dof];
        if driver.is_none() {
            self.measured_velocities = vec![0.0; self.config.dof];
        }
        // Send the stiff MIT exit hold immediately when uncontended; never block on a held mutex.
        if exited_gravity_comp && !driver_contended && self.is_torqued && !self.disable_pending {
            if let (Some(targets), Some(d)) = (self.gravity_comp_targets(), driver) {
                if let Ok(mut drv) = d.try_lock() {
                    if let Err(e) = drv.write_joint_targets(&targets) {
                        tracing::error!("Gravity comp hold write failed: {e}");
                    }
                }
            }
        }
        self.safety.trigger_e_stop();
    }

    pub fn apply_stop_policy(&mut self, policy: StopPolicy) {
        #[cfg(feature = "policy-runtime")]
        match policy {
            StopPolicy::NormalUnload => {
                self.terminate_policy_run("STOPPED", "cancelled", Some("STOPPED"), Some("Arm unloading".into()));
            }
            StopPolicy::EStopLatch => {
                self.terminate_policy_run("ESTOP", "failed", Some("ESTOP_ACTIVE"), Some("E-stop latched".into()));
            }
            StopPolicy::ThermalTrip => {
                self.terminate_policy_run("THERMAL_TRIP", "failed", Some("THERMAL_TRIP"), Some("Thermal trip".into()));
            }
            StopPolicy::BusLossFallback => {
                self.terminate_policy_run("BUS_LOSS", "failed", Some("BUS_LOSS"), Some("CAN feedback lost".into()));
            }
        }
        if policy == StopPolicy::NormalUnload && !self.supports_automatic_unload() {
            tracing::warn!("NormalUnload unsupported: resting pose or confirmed velocity/torque/enable feedback unavailable; no unload motion started");
            return;
        }
        self.servo_target = None;
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.servo_stream_active = false;
        // M1: keep output history across NormalUnload so park ramps any residual lead out smoothly
        // instead of dropping active lead in a single tick.
        if policy != StopPolicy::NormalUnload {
            self.tracking.invalidate();
        }
        match policy {
            StopPolicy::NormalUnload => {
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
                        self.active_trajectory = None;
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
                        self.active_trajectory = None;
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
                                self.active_trajectory = None;
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
                                self.active_trajectory = None;
                                return;
                            }
                        };
                        if let Err(e) = self.check_workspace_path_with_tol(&clamped_start, samples.iter().map(Vec::as_slice), crate::kinematics::WORKSPACE_PARK_ESCAPE_EPS) {
                            let e = e.with_workspace_phase("park");
                            tracing::warn!("Park path violates workspace ({e}); maintaining safe hold");
                            self.park_rejection = Some(e);
                            self.pending_normal_unload = false;
                            self.unload_settle = None;
                            self.active_trajectory = None;
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
                // If the arm is torqued, maintain holding torque if motion cannot be planned or arm is not operational!
                // NEVER cut power under gravity when motion planning fails or arm is in stopped/idle state!
                if self.is_torqued {
                    tracing::warn!("NormalUnload cannot plan motion or arm not operational; maintaining holding torque to prevent dropping arm");
                    self.pending_normal_unload = false;
                    self.unload_settle = None;
                    self.active_trajectory = None;
                    self.servo_target = None;
                    return;
                }
                // If not torqued, ensure state is clean
                self.pending_normal_unload = false;
                self.unload_settle = None;
                self.is_torqued = false;
                self.tracking.invalidate();
                self.active_trajectory = None;
                self.reset_gripper_stall();
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                let _ = self.safety.set_torque(false);
                self.mit_torque_off();
                if let Some(ref d) = self.driver {
                    if let Ok(mut driver) = d.lock() {
                        let _ = driver.set_torque(false);
                    }
                }
            }
            StopPolicy::EStopLatch => {
                let driver = self.driver.take();
                self.apply_estop_latch_with_driver(driver.as_ref());
                self.driver = driver;
            }
            StopPolicy::ThermalTrip => {
                self.stop_motion();
                self.commanded_velocities.fill(0.0);
                self.commanded_accelerations.fill(0.0);
                let (joint_idx, temperature) = self.temperatures.iter().copied().enumerate()
                    .filter(|(_, t)| t.is_finite()).max_by(|a,b| a.1.total_cmp(&b.1)).unwrap_or((0, 0.0));
                if !matches!(self.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { .. })) {
                    self.safety.trigger_fault(FaultReason::ThermalTrip { joint_idx, temperature, limit: self.config.max_temperature });
                }
                if let Err(e) = self.request_disable() { tracing::error!("Thermal shutdown not confirmed: {e}"); }
            }
            StopPolicy::BusLossFallback => {
                self.exit_gravity_comp("BUS_LOSS", "CAN feedback lost");
                // Bus loss / disconnect: hold position locked, latch fault, advance epoch
                // Maintain holding torque: DO NOT disable hardware torque to prevent gravitational collapse!
                self.pending_normal_unload = false;
                self.unload_settle = None;
                self.active_trajectory = None;
                self.servo_target = None;
                let hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
                    && self.measured_positions.len() == self.config.dof
                {
                    self.measured_positions.clone()
                } else {
                    self.commanded_positions.clone()
                };
                self.target_positions = hold_pos.clone();
                self.commanded_positions = hold_pos;
                self.commanded_velocities = vec![0.0; self.config.dof];
                if self.driver.is_none() {
                    self.measured_velocities = vec![0.0; self.config.dof];
                }
                self.bus_loss_instant = Some(Instant::now());
                self.safety.trigger_fault(FaultReason::BusLoss);
            }
        }
    }

    pub fn preempt_lease(&mut self, conn: &str, label: &str) -> Result<(u64, Option<(String, u64)>), PreemptError> {
        let now = Instant::now();
        if let Some(ref active) = self.active_lease {
            let is_expired = active.acquired_at.elapsed() >= self.lease_timeout;
            if active.client_id == conn && !is_expired {
                return Ok((active.generation, None));
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
        self.terminate_policy_run("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease preempted".into()));

        // Four-category state categorization (strictly fail-closed, no unwrap/panic, driver lock poison returns false)
        let confirmed_hardware_disabled = match &self.driver {
            None => !self.is_torqued && !self.disable_pending && self.safety.state() == &SafetyState::Idle,
            Some(driver) => !self.is_torqued
                && !self.disable_pending
                && self.safety.state() == &SafetyState::Idle
                && driver.lock().is_ok_and(|drv| {
                    if !drv.capabilities().enable_feedback {
                        true
                    } else {
                        drv.enable_state() == EnableState::Disabled
                    }
                }),
        };

        if confirmed_hardware_disabled
            || self.safety.state().is_stopped()
            || self.is_fully_settled_armed_hold()
        {
            self.handover_settling_since = None;
        } else {
            self.stop_motion();
            self.handover_settling_since = Some(now);
        }

        let evicted = self.active_lease.as_ref().filter(|a| a.client_id != conn).map(|a| (a.client_id.clone(), a.generation));

        self.lease_generation_counter += 1;
        let gen = self.lease_generation_counter;
        self.active_lease = Some(ActiveLease {
            client_id: conn.to_string(),
            generation: gen,
            label: label.to_string(),
            acquired_at: now,
            last_valid_motion_instant: None,
        });
        self.last_heartbeat = now;
        if evicted.is_some() {
            self.last_preemption = Some(now);
            self.last_evicted = evicted.clone();
        }

        tracing::info!("Preempted lease for connection {} (label {}), generation {}, evicted: {:?}", conn, label, gen, evicted);
        Ok((gen, evicted))
    }

    pub fn acquire_lease(&mut self, client_id: &str) -> Result<u64, (String, u64)> {
        let now = Instant::now();
        if let Some(ref active) = self.active_lease {
            let is_expired = active.acquired_at.elapsed() >= self.lease_timeout;
            if !is_expired {
                if active.client_id == client_id { return Ok(active.generation); }
                return Err((active.client_id.clone(), active.generation));
            }
        }
        // A new controller never inherits an expired controller's queued motion,
        // nor a watchdog park started after the previous lease was released.
        self.cancel_watchdog_park_for_new_lease(client_id);
        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease acquired by new controller".into()));
        if self.active_lease.is_some() { self.stop_motion(); }
        self.lease_generation_counter += 1;
        let gen = self.lease_generation_counter;
        self.active_lease = Some(ActiveLease {
            client_id: client_id.to_string(),
            generation: gen,
            label: String::new(),
            acquired_at: now,
            last_valid_motion_instant: None,
        });
        Ok(gen)
    }

    /// A new lease owner takes over from a running watchdog park: the park's
    /// motion is cancelled like any other queued motion and the arm brakes to a hold.
    pub(super) fn cancel_watchdog_park_for_new_lease(&mut self, client_id: &str) {
        if self.watchdog_park.take().is_some() && self.pending_normal_unload {
            tracing::info!("Watchdog park cancelled: new lease acquired by {client_id}; holding position");
            self.stop_motion();
        }
    }

    /// Budget for a watchdog park, in control-loop time: deceleration of the physical arm
    /// from its measured velocity (max over joints of |v| / planning acceleration), the
    /// planned park duration (which already starts from the commanded velocity), the
    /// settle hold and [`WATCHDOG_PARK_MARGIN`]; never less than [`WATCHDOG_PARK_TIMEOUT`].
    pub fn watchdog_park_timeout(&self) -> Duration {
        let decel = self.measured_velocities.iter().zip(self.planning_limits())
            .filter(|(v, l)| v.is_finite() && l.max_acceleration > 1e-6)
            .map(|(v, l)| f64::from(v.abs() / l.max_acceleration))
            .fold(0.0, f64::max);
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
        if !self.pending_normal_unload {
            tracing::warn!("Watchdog park interrupted (safety state {:?})", self.safety.state());
            if self.is_torqued && !self.safety.state().is_stopped() {
                self.latch_watchdog_fault();
            }
            return;
        }
        // budget in reference time: the power governor may slow the park
        if dt.is_finite() && dt > 0.0 { park.elapsed_s += dt * self.motion_time_scale(); }
        if park.elapsed_s < park.budget_s {
            self.watchdog_park = Some(park);
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

    /// Conditions for parking automatically when the lease watchdog expires.
    /// Returns the reason the park is not allowed, or `None` when it is.
    pub(super) fn watchdog_park_blocker(&self) -> Option<String> {
        let Some(policy) = &self.config.resting_pose else { return Some("no resting pose configured".into()); };
        if !self.supports_automatic_unload() { return Some("driver lacks velocity/torque/enable feedback".into()); }
        if self.e_stop_latched.load(Ordering::SeqCst) { return Some("E-stop latched".into()); }
        if self.collision.hold.is_some() { return Some("collision hold".into()); }
        if let Some(h) = self.torque_guard.hold {
            let rest = self.config.resting_positions().get(h.joint).copied().unwrap_or(h.hold_q);
            if (rest - h.anchor_q) * h.sign > crate::torque_guard::TORQUE_PUSH_IN_RAD {
                return Some(format!("torque hold on joint {}: the resting pose lies in the push direction", h.joint + 1));
            }
        }
        if !self.safety.state().is_operational() { return Some(format!("safety state {:?}", self.safety.state())); }
        if self.pending_rearm || self.disable_pending || self.output_suspended || self.handover_settling_since.is_some() {
            return Some("re-arm, disable, output suspension or handover pending".into());
        }
        if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) {
            return Some(format!("feedback older than {} ms", policy.feedback_max_age_ms));
        }
        None
    }

    /// Previous watchdog behaviour: rigid hold at the measured (or commanded) position
    /// and latch Fault(WatchdogTimeout). Torque stays on.
    pub(crate) fn latch_watchdog_fault(&mut self) {
        #[cfg(feature = "policy-runtime")]
        self.terminate_policy_run("WATCHDOG_TIMEOUT", "failed", Some("WATCHDOG_TIMEOUT"), Some("Lease watchdog expired".into()));
        let mut hold_pos = if self.measured_positions.iter().all(|p| p.is_finite())
            && self.measured_positions.len() == self.config.dof
        {
            self.measured_positions.clone()
        } else {
            self.commanded_positions.clone()
        };
        // a torque hold keeps its backed-off command (src/torque_guard.rs)
        if let Some(h) = self.torque_guard.hold {
            if let (Some(p), Some(c)) = (hold_pos.get_mut(h.joint), self.commanded_positions.get(h.joint)) { *p = *c; }
        }
        self.watchdog_park = None;
        self.pending_normal_unload = false;
        self.unload_settle = None;
        self.active_trajectory = None;
        self.servo_target = None; self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.target_positions = hold_pos.clone();
        self.commanded_positions = hold_pos;
        self.commanded_velocities = vec![0.0; self.config.dof];
        self.safety.trigger_fault(FaultReason::WatchdogTimeout);
    }

    /// Drops the lease if `client_id`/`generation` own it. Returns true only when the
    /// lease was still valid: callers stop motion on true. An expired lease is dropped
    /// without that, because its expiry may already have started a watchdog park that a
    /// client without a valid lease must not cancel.
    pub fn release_lease(&mut self, client_id: &str, generation: u64) -> bool {
        if let Some(ref active) = self.active_lease {
            if active.client_id == client_id && active.generation == generation {
                let valid = active.acquired_at.elapsed() < self.lease_timeout;
                self.active_lease = None;
                #[cfg(feature = "policy-runtime")]
                self.terminate_policy_run("LEASE_LOST", "cancelled", Some("LEASE_LOST"), Some("Control lease released".into()));
                return valid;
            }
        }
        false
    }

    /// Records the label a lease holder sent with `acquire_lease` (shown to other clients).
    pub fn set_lease_label(&mut self, client_id: &str, generation: u64, label: &str) {
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
        self.active_lease.as_ref().filter(|a| a.acquired_at.elapsed() < self.lease_timeout).map(|a| a.label.as_str())
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
            active.acquired_at.elapsed() < self.lease_timeout
        } else {
            false
        }
    }

    pub fn heartbeat(&mut self, client_id: &str, epoch_id: u64) -> Result<(), SafetyError> {
        if epoch_id != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch {
                expected: self.safety.epoch_id(),
                received: epoch_id,
            });
        }
        if let Some(ref mut active) = self.active_lease {
            if active.client_id == client_id && active.acquired_at.elapsed() < self.lease_timeout {
                active.acquired_at = Instant::now();
                self.last_heartbeat = Instant::now();
            } else {
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
    /// allocations). Returns `None` if `config.limits.len() > MAX_DOF`.
    pub(crate) fn planning_limits_inline(&self, speed_scale: Option<f64>) -> Option<([crate::model::JointLimit; crate::rt_core::mailbox::MAX_DOF], usize)> {
        let dof = self.config.limits.len();
        if dof > crate::rt_core::mailbox::MAX_DOF { return None; }
        let scale = self.config.speed_cap.and_then(|c| c.joint_velocity_scale).unwrap_or(1.0);
        let gripper_idx = self.config.gripper_index();
        let commissioning = self.config.active_tool.commissioning_required;
        let mut out: [crate::model::JointLimit; crate::rt_core::mailbox::MAX_DOF] = std::array::from_fn(|_| crate::model::JointLimit {
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

    /// Zero-allocation static refusal checks for RT jog commits (`commit_primitive` /
    /// `Primitive::JointQuintic`), preserving the exact refusal precedence of `jog_action_slice`.
    pub(crate) fn jog_precheck_static(&self) -> Result<(), &'static str> {
        if self.config.active_tool.commissioning_required { return Err("COMMISSIONING_REQUIRED"); }
        if self.core.motion_inhibited_by_config() { return Err("CONFIG_BUSY"); }
        if self.gravity_comp.is_some() { return Err("MOTION_BUSY"); }
        // When MIT tracking is configured on a torqued arm, defer remaining checks to
        // `jog_action_slice` so `mit_motion_blocker()` keeps its exact precedence.
        if self.is_torqued && self.config.joint_tracking.is_some() { return Ok(()); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading || self.policy_run.is_some() { return Err("MOTION_BUSY"); }
        if self.collision.hold.is_some() { return Err("COLLISION_HOLD"); }
        if self.handover_settling_since.is_some() { return Err("REJECTED"); }
        if self.e_stop_latched.load(Ordering::SeqCst) { return Err("ESTOP_ACTIVE"); }
        if self.pending_rearm || self.pending_normal_unload { return Err("NOT_ARMED"); }
        Ok(())
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

    /// Jog from a fixed slice (RT commit path: no `to_vec()`, no `planning_limits()`
    /// allocation, no `workspace_samples` `Vec`, no `target_positions` replacement free).
    pub(crate) fn jog_action_slice(
        &mut self,
        target_positions: &[f32],
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
        kind: &str,
    ) -> Result<(u64, u64, f64), ArmError> {
        use crate::rt_core::mailbox::MAX_DOF;
        if self.config.active_tool.commissioning_required {
            return Err(SafetyError::CommissioningRequired(format!("{kind} (requires speed_scale <= 0.2)")).into());
        }
        let n = target_positions.len();
        if n > MAX_DOF {
            return self.jog_action(target_positions.to_vec(), duration_secs, epoch_id, kind);
        }
        let Some((limits_buf, lim_n)) = self.planning_limits_inline(None) else {
            return self.jog_action(target_positions.to_vec(), duration_secs, epoch_id, kind);
        };
        let limits = &limits_buf[..lim_n];
        let mut target = [0.0f32; MAX_DOF];
        target[..n].copy_from_slice(target_positions);
        let target = &mut target[..n];

        self.config_motion_check()?;
        self.gravity_comp_motion_busy()?;
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
        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }
        if self.pending_rearm || self.pending_normal_unload {
            return Err(ArmError::Safety(SafetyError::NotArmed(self.safety.state().clone())));
        }
        self.pending_normal_unload = false;

        self.safety.validate_motion_command(epoch_id, target, &self.config.limits)?;
        self.torque_hold_filter(target);
        self.power_hold_filter(target);

        if let Some(g_idx) = self.config.gripper_index() {
            if self.gripper_stalled && target.len() > g_idx && self.commanded_positions.len() > g_idx {
                let delta = target[g_idx] - self.commanded_positions[g_idx];
                let tau_g = self.torques.get(g_idx).copied().unwrap_or(0.0);
                if (delta * tau_g < -1e-4) || (delta * self.gripper_stall_sign < -1e-4) {
                    self.gripper_stalled = false;
                    self.gripper_high_torque_ticks = 0;
                    self.gripper_low_torque_ticks = 0;
                }
            }
        }

        let zeros = [0.0f32; MAX_DOF];
        let traj = TrajectoryPlanner::plan(
            &self.commanded_positions,
            &self.commanded_velocities,
            target,
            &zeros[..self.config.dof],
            duration_secs,
            limits,
        )?;
        let (traj, power_scale) = self.power_fit_quintic(traj, &self.commanded_positions, &self.commanded_velocities, target, limits, true)?;
        self.check_workspace_quintic_inline(&traj)?;

        self.last_heartbeat = Instant::now();
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        crate::rt_core::scratch::copy_into(&mut self.target_positions, target);
        self.safety.mark_motion_started()?;
        let duration = traj.duration;
        let id = self.begin_action_rt(kind);
        if let Some(s) = power_scale { self.note_action_power_scale(id, s); }
        // No `speed_scale` here, so `active_tracking_limits` already uses `planning_limits()`
        // when `motion_limits` is None (avoids cloning `Vec<JointLimit>` and joint name strings).
        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, Some(id)));
        Ok((self.safety.epoch_id(), id, duration))
    }

    pub fn jog(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
    ) -> Result<u64, ArmError> {
        self.jog_action(target_positions, duration_secs, epoch_id, "joint_move").map(|(epoch, _, _)| epoch)
    }

    /// Jog that records an action of `kind` (`joint_move` or `home`).
    /// Returns (epoch, action id, planned duration).
    pub fn jog_action(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
        kind: &str,
    ) -> Result<(u64, u64, f64), ArmError> {
        self.jog_action_with_speed_scale(target_positions, duration_secs, epoch_id, kind, None)
    }

    pub fn jog_action_with_speed_scale(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
        kind: &str,
        speed_scale: Option<f64>,
    ) -> Result<(u64, u64, f64), ArmError> {
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
                            let has_torque_fb = d.lock().is_ok_and(|drv| drv.capabilities().torque_feedback);
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
        self.start_quintic(target_positions, duration_secs, epoch_id, kind, None, &limits)
    }

    /// Joint-space quintic to `target_positions` planned with `limits`, recorded as an
    /// action of `kind`. Returns (epoch, action id, planned duration).
    pub(crate) fn start_quintic(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
        kind: &str,
        tag: Option<String>,
        limits: &[crate::model::JointLimit],
    ) -> Result<(u64, u64, f64), ArmError> {
        self.start_quintic_prepared(target_positions, duration_secs, epoch_id, kind, tag, limits, None)
    }

    /// [`start_quintic`](Self::start_quintic) that installs `prepared` (planned, power-fitted
    /// and workspace-checked without the lock) when it was made from the same start, target,
    /// limits, workspace revision, tool offset and power budget; else plans and checks here.
    /// Every other check runs here either way.
    pub(crate) fn start_quintic_prepared(
        &mut self,
        target_positions: Vec<f32>,
        duration_secs: Option<f64>,
        epoch_id: Option<u64>,
        kind: &str,
        tag: Option<String>,
        limits: &[crate::model::JointLimit],
        prepared: Option<PreparedQuintic>,
    ) -> Result<(u64, u64, f64), ArmError> {
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

        if self.e_stop_latched.load(Ordering::SeqCst) {
            return Err(ArmError::Safety(SafetyError::EStopActive));
        }

        if self.pending_rearm || self.pending_normal_unload {
            return Err(ArmError::Safety(SafetyError::NotArmed(self.safety.state().clone())));
        }
        self.pending_normal_unload = false;

        // 1. Validate command against single controller authority, safety state, and soft limits
        self.safety.validate_motion_command(
            epoch_id,
            &target_positions,
            &self.config.limits,
        )?;
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
            let zero_buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
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
        self.servo_ws_min_depths = None;
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &target_positions);
        self.safety.mark_motion_started()?;
        let duration = traj.duration;
        let id = self.begin_action(kind, tag);
        if let Some(s) = power_scale { self.note_action_power_scale(id, s); }
        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, Some(id)).with_motion_limits(limits.to_vec()));

        Ok((self.safety.epoch_id(), id, duration))
    }

    /// Why the workspace rule may not change now: a motion, macro, stream or park is
    /// running (each was checked against the current rule).
    pub fn workspace_change_blocker(&self) -> Option<String> {
        if self.macro_run.is_some() { return Some("a macro action is running".into()); }
        if self.pending_normal_unload { return Some("a park is running".into()); }
        self.busy_reason()
    }

    pub(super) fn busy_reason(&self) -> Option<String> {
        if self.calibration_busy { return Some("a calibration apply is running".into()); }
        if self.zero_check_persisting.is_some() { return Some("gripper calibration is being persisted".into()); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_loading { return Some("a policy is loading".into()); }
        #[cfg(feature = "policy-runtime")]
        if self.policy_run.is_some() { return Some("a policy run is active".into()); }
        if self.active_trajectory.is_some() { return Some("a trajectory is running".into()); }
        if self.servo_target.is_some() { return Some("servo streaming is active".into()); }
        if self.commanded_velocities.iter().any(|v| v.abs() >= 1e-3) { return Some("the arm is still moving".into()); }
        None
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
    pub fn install_path(&mut self, input: &PlanInput, path: JointPath, kind: &str, tag: Option<String>) -> Result<(u64, f64), ArmError> {
        self.check_plan_fresh(input)?;
        let duration = self.start_path_motion(input, &path)?;
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
        if self.mit_motion_blocker().is_some() { return Err("PLAN_STALE"); }
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
    pub(crate) fn install_prepared_path_slot(
        &mut self,
        epoch: u64,
        dof: usize,
        slot: crate::rt_core::broker::PlanSlotRef<crate::rt_core::ops::PlanSlot>,
        duration_s: f64,
        power_scale: Option<f64>,
    ) -> Result<(u64, f64), &'static str> {
        self.validate_prepared_joint_path(dof, &slot.path, duration_s)?;
        self.check_prepared_fresh_static(epoch)?;
        let last = &slot.path.samples()[slot.path.len() - 1][..dof];
        let duration = self
            .start_path_motion_slice(epoch, last, duration_s)
            .map_err(crate::rt_core::ops::static_code)?;
        let id = self.begin_action_rt("move_to");
        let scale = power_scale.unwrap_or(1.0);
        if power_scale.is_some() { self.note_action_power_scale(id, scale); }
        let arm_dof = self.config.arm_dof();
        self.active_trajectory = Some(ActiveTrajectory::new(
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
    pub(crate) fn install_prepared_path(&mut self, epoch: u64, path: JointPath, power_noted: bool) -> Result<(u64, f64), ArmError> {
        self.check_prepared_fresh(epoch)?;
        let duration = self.start_path_motion_at(epoch, &path)?;
        let id = self.begin_action("move_to", None);
        if power_noted { self.note_action_power_scale(id, path.power_scale); }
        self.active_trajectory = Some(ActiveTrajectory::new(Some(id), Reference::Path(PathTrajectory::new(path))));
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
    pub fn install_joint_goal(&mut self, input: &PlanInput, q: &[f64; 6], speed_scale: f64, kind: &str, tag: Option<String>,
        same_side: Option<crate::backlash::Request>) -> Result<(u64, f64), ArmError> {
        self.install_joint_goal_prepared(input, q, speed_scale, kind, tag, same_side, None)
    }

    /// [`install_joint_goal`](Self::install_joint_goal) with the quintic prepared without the
    /// lock ([`prepare_quintic`]); used when its inputs still match.
    pub fn install_joint_goal_prepared(&mut self, input: &PlanInput, q: &[f64; 6], speed_scale: f64, kind: &str, tag: Option<String>,
        same_side: Option<crate::backlash::Request>, prepared: Option<PreparedQuintic>) -> Result<(u64, f64), ArmError> {
        self.check_plan_fresh(input)?;
        let target = self.joint_goal_target(q);
        let limits = self.joint_goal_limits(speed_scale);
        let (_, id, duration) = match same_side {
            Some(request) => self.start_quintic_same_side(target, Some(input.epoch), kind, tag, &limits, "move_to_precise", request)?,
            None => self.start_quintic_prepared(target, None, Some(input.epoch), kind, tag, &limits, prepared)?,
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
    pub fn commit_feedback_valid(&self) -> bool {
        self.feedback_numeric_valid && self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv| drv.is_feedback_valid()))
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
    /// like [`Self::stop_motion`], except a running watchdog park. The park belongs to the
    /// lease watchdog, not to the client that stopped: only a new lease ([`Self::acquire_lease`],
    /// [`Self::preempt_lease`]) takes it over. Returns false when the park was kept.
    ///
    /// RT-safe: when keeping a watchdog park, records fields into [`crate::rt_core::ops::stop_diag`]
    /// without tracing or allocating; a normal-priority thread formats off-thread.
    pub fn authorized_stop(&mut self) -> bool {
        if let Some(park) = self.watchdog_park.as_ref().filter(|_| self.pending_normal_unload) {
            crate::rt_core::ops::stop_diag().record_park_ignored(self.core.id(), park.elapsed_s, park.budget_s);
            return false;
        }
        self.stop_motion();
        true
    }

    pub(super) fn start_path_motion(&mut self, input: &PlanInput, path: &JointPath) -> Result<f64, ArmError> {
        self.start_path_motion_at(input.epoch, path)
    }

    /// Validates `last` under safety epoch `epoch` and activates path motion without allocating
    /// or freeing (`target_positions` is updated in place via `copy_into`).
    pub fn start_path_motion_slice(&mut self, epoch: u64, last: &[f32], duration: f64) -> Result<f64, ArmError> {
        self.safety.validate_motion_command(Some(epoch), last, &self.config.limits)?;
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        crate::rt_core::scratch::copy_into(&mut self.target_positions, last);
        self.safety.mark_motion_started()?;
        self.last_heartbeat = Instant::now();
        Ok(duration)
    }

    /// [`Self::start_path_motion`] for a path planned under safety epoch `epoch`.
    pub(crate) fn start_path_motion_at(&mut self, epoch: u64, path: &JointPath) -> Result<f64, ArmError> {
        self.start_path_motion_slice(epoch, path.last_slice(), path.duration)
    }
}
