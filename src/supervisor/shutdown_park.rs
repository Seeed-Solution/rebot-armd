//! Shutdown sequencing, handback quiescing, internal fault lane, and park-release helpers.

#[allow(unused_imports)]
use super::*;

// scaffold-extra-0
impl SupervisorInner {
    /// Returns `true` when no background disable requires a periodic 250 ms retry (either no
    /// background disable is pending, a broker torque transition owns it, or the RT executor is
    /// actively polling an in-flight driver disable transition).
    pub(crate) fn confirm_background_disable_if_ready(&mut self) -> bool {
        if !self.disable_pending || self.torque_transition.is_some() {
            return true;
        }
        if self.driver.is_none() {
            self.disable_accepted();
            return true;
        }
        self.rt_executor_owned
            && self
                .driver
                .as_ref()
                .is_some_and(|d| d.try_with_ref(|drv| drv.torque_transition_pending()).unwrap_or(false))
    }

    pub fn shutdown_ready(&self) -> bool {
        if self.disable_pending || self.gravity_comp.is_some() || self.torque_transition_pending() {
            return false;
        }
        match &self.driver {
            None => !self.is_torqued,
            Some(_) => !self.is_torqued && self.torque_intent == TorqueIntent::Off,
        }
    }
    /// Quiesces active motion/targets, refreshes live measured-position hold provenance directly
    /// from the attached driver at a release boundary (zero allocs, zero locks on `Exclusive`),
    /// aligns `commanded_positions`/`target_positions` to fresh measured positions, and returns
    /// `true` ONLY when every arm joint (J1–J6) has fresh, finite, mode-verified hold coverage
    /// right now (never accepting stale retained `captured_positions` or unverified modes).
    pub(crate) fn quiesce_motion_and_refresh_hold_for_handback(&mut self) -> bool {
        #[cfg(not(target_arch = "wasm32"))]
        self.revoke_sessions_on_stop("HANDBACK");
        self.active_trajectory = None;
        self.release_servo_target();
        self.servo_stream_active = false;
        self.macro_run = None;
        self.gravity_comp = None;
        self.pending_normal_unload = false;
        self.unload_settle = None;
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.refresh_measured_hold_for_handback();
        let snap = self.owner_driver_snapshot;
        if !snap.valid {
            return false;
        }
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        let gripper_idx = self.config.gripper_index();
        let arm_dof = self.config.arm_dof().min(dof);
        let mut all_arm_covered = true;
        for j in 0..dof {
            let is_arm_joint = j < arm_dof && gripper_idx != Some(j);
            let fresh_pos = snap.fresh_samples[j]
                .map(|(p, _)| p)
                .filter(|p| p.is_finite());
            let mode_verified = snap.hold_mode_verified[j];
            let status_fresh = matches!(
                self.measured_hold.joint_status[j],
                crate::driver::JointHoldStatus::Held | crate::driver::JointHoldStatus::Queued
            );
            if let (Some(live_pos), true, true) = (fresh_pos, mode_verified, status_fresh) {
                let hold_pos = self.measured_hold.captured_positions[j]
                    .filter(|p| p.is_finite())
                    .unwrap_or(live_pos);
                if j < self.commanded_positions.len() {
                    self.commanded_positions[j] = hold_pos;
                }
                if j < self.target_positions.len() {
                    self.target_positions[j] = hold_pos;
                }
            } else if is_arm_joint {
                all_arm_covered = false;
            }
        }
        all_arm_covered
    }

    /// Refreshes live measured-position hold provenance directly from the attached driver at a
    /// release boundary (zero allocations, zero locks on `DriverSlot::Exclusive`) immediately
    /// before energized handback to Legacy ownership.
    pub(crate) fn refresh_measured_hold_for_handback(&mut self) {
        self.owner_driver_snapshot.invalidate();
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        if let Some(d) = self.driver.as_ref() {
            let mut ages = std::mem::take(&mut self.scratch.ages);
            let mut bounds = std::mem::take(&mut self.scratch.fb.bounds);
            let feedback_numeric_valid = self.feedback_numeric_valid;
            let measured = &self.measured_positions;
            if let Some(snap) = d.with_ref(|drv| {
                driver_transitions::OwnerDriverSnapshot::capture_from_driver(
                    drv,
                    dof,
                    feedback_numeric_valid,
                    measured,
                    &mut ages,
                    &mut bounds,
                )
            }) {
                self.owner_driver_snapshot = snap;
            }
            self.scratch.ages = ages;
            self.scratch.fb.bounds = bounds;
        }
        let _ = self.capture_measured_hold_from_owner();
    }

    /// Latches a non-recoverable fault naming the panicking non-control thread (DRV-F1 / FU-PANIC).
    pub(super) fn latch_internal_fault_non_recoverable(&mut self) {
        let mut name_buf = [0u8; 64];
        let thread_name = self.internal_fault_lane.copy_thread_name(&mut name_buf);
        self.safety.latch(
            crate::safety::FaultKind::Other,
            crate::safety::FaultDetail::formatted(crate::safety::FaultCode::Other, |buf| {
                let _ = core::fmt::write(
                    buf,
                    format_args!("non-control thread panic ({thread_name}); commands fenced"),
                );
            }),
        );
        self.internal_fault_lane.mark_fault_latched();
    }

    /// Owner-side lock-free service of [`crate::panic_coord::InternalFaultLane`] invoked on EVERY
    /// control tick/release across BOTH Legacy and RT Executor owners (zero queue dependency).
    ///
    /// On a non-control thread panic while `armd-control` is healthy:
    /// 1. Revokes prior motion, servo targets, macro/policy runs, and any pending enable transition.
    /// 2. Attempts normal stop-direction park to the resting pose (`StopPolicy::NormalUnload`).
    /// 3. At confirmed rest, normal rest-gated disable (`Rule A'`) removes torque and latches a
    ///    non-recoverable fault naming the panicking thread.
    /// 4. If park cannot start or fails/aborts mid-way, keeps holding and latches the
    ///    non-recoverable fault (never disables torque away from rest).
    #[inline(never)]
    pub(crate) fn service_internal_fault_lane(&mut self) {
        use crate::panic_coord::InternalFaultParkOutcome;
        if !self.internal_fault_lane.is_fenced() {
            return;
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.revoke_sessions_on_stop("INTERNAL_FAULT");
        if self.internal_fault_lane.take_park_request() {
            // Revoke all prior client motion, servo targets, macro/policy runs, and any in-flight
            // enable transition without setting E-stop (so `NormalUnload` remains eligible).
            if let Some(driver_transitions::TorqueTransition::Enable(_)) =
                self.torque_transition.take()
            {
                #[cfg(not(target_arch = "wasm32"))]
                let shadow_rt = &mut self.torque_shadow;
                if let Some(d) = self.driver.as_mut() {
                    let _ = d.try_with_mut(|drv| {
                        drv.abort_enable();
                        #[cfg(not(target_arch = "wasm32"))]
                        shadow_rt.drain_from_driver(drv);
                    });
                }
                self.enable_lifecycle = driver_transitions::EnableLifecycle::FailedHold;
                self.pending_rearm = true;
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow.enqueue(
                    crate::torque::shadow::ShadowInputKind::HandshakeResult {
                        live_op_id: self.shadow_enable_op_id,
                        confirmed_mask: crate::torque::ChannelMask::EMPTY,
                        outcome: crate::torque::HandshakeOutcome::Failed,
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    },
                );
            }
            self.stop_motion();
            self.servo_target = None;
            self.servo_stream_active = false;
            self.drop_macro_run("failed", Some("INTERNAL_FAULT"));
            #[cfg(feature = "policy-runtime")]
            if self.policy_run.is_some() {
                self.policy_run = None;
            }
            if !self.is_torqued && self.torque_intent == TorqueIntent::Off {
                self.latch_internal_fault_non_recoverable();
                self.internal_fault_lane
                    .set_outcome(InternalFaultParkOutcome::ParkedAndDisabled);
                return;
            }
            self.apply_stop_policy(StopPolicy::NormalUnload);
            if self.pending_normal_unload {
                self.internal_fault_park_active = true;
                self.internal_fault_lane
                    .set_outcome(InternalFaultParkOutcome::Parking);
            } else {
                // Park could not start (workspace violation, blocked, no rest pose, already
                // stopped, or failed hold): keep holding and latch non-recoverable fault.
                self.latch_internal_fault_non_recoverable();
                self.internal_fault_lane
                    .set_outcome(InternalFaultParkOutcome::HoldingFallback);
            }
            return;
        }

        if self.internal_fault_park_active {
            if !self.is_torqued
                && self.torque_intent == TorqueIntent::Off
                && !self.disable_pending
                && !self.torque_transition_pending()
            {
                self.internal_fault_park_active = false;
                self.latch_internal_fault_non_recoverable();
                self.internal_fault_lane
                    .set_outcome(InternalFaultParkOutcome::ParkedAndDisabled);
            } else if !self.pending_normal_unload
                && !self.disable_pending
                && !self.torque_transition_pending()
                && self.active_trajectory.is_none()
            {
                // Park ended without disabling (e.g. rest-gate refused or trajectory aborted):
                // retain holding torque and latch non-recoverable fault.
                self.internal_fault_park_active = false;
                self.latch_internal_fault_non_recoverable();
                self.internal_fault_lane
                    .set_outcome(InternalFaultParkOutcome::HoldingFallback);
            }
        }
    }

    /// True when this supervisor is idle/disarmed with no active torque, gravity-comp, or
    /// disable transition (never enabled or cleanly disarmed), making a bounded read-only
    /// hardware status poll (`0xCC`) safe to confirm physical disarm on shutdown/teardown.
    #[inline]
    pub(crate) fn eligible_for_readonly_disarm_proof(&self) -> bool {
        self.driver.is_some()
            && !self.is_torqued
            && !self.disable_pending
            && self.gravity_comp.is_none()
            && !self.torque_transition_pending()
            && self.enable_lifecycle == driver_transitions::EnableLifecycle::Disarmed
    }

    /// Actively probe causal hardware disarm proof within `deadline` using ONLY read-only
    /// driver status polls (`0xCC` on Damiao; never enables or disables motors).
    pub(crate) fn probe_driver_disarmed_until(&mut self, deadline: Instant) -> bool {
        if self.driver.is_none() {
            return true;
        }
        if !self.eligible_for_readonly_disarm_proof() {
            return false;
        }
        let Some(slot) = self.driver.as_mut() else {
            return true;
        };
        matches!(
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                slot.try_with_mut(|drv| drv.probe_hardware_disarmed_until(deadline))
            })),
            Ok(Ok(true))
        )
    }

    /// Record a causally proven idle/cold-boot physical disarm (`torque_intent = Off`) without
    /// altering safety state or emitting any disable frames.
    pub(crate) fn record_cold_boot_disarm_proof(&mut self) {
        self.torque_intent = TorqueIntent::Off;
        self.measured_hold.clear();
        self.last_poll_enable_state = EnableState::Disabled;
        self.last_poll_partial_enable = false;
    }

    /// Authoritative shutdown gate shared by legacy lock and RT executor ownership paths.
    /// Returns `true` when safe to stop the control loop and exit; returns `false` after
    /// latching stop when hardware disarm is not confirmed. When idle/never-enabled on boot,
    /// actively obtains causal disarm proof via bounded read-only status polls (`0xCC`).
    /// When shutdown was refused while not armed (`!is_torqued || disable_pending`), a repeated
    /// shutdown signal issues `DisableProtective { ProcessExit }`.
    pub fn request_shutdown(&mut self) -> bool {
        self.request_shutdown_until(Instant::now() + crate::driver::SHUTDOWN_DISARM_PROOF_WINDOW)
    }

    /// Bounded variant of [`Self::request_shutdown`] carrying the caller's absolute `deadline`.
    pub fn request_shutdown_until(&mut self, deadline: Instant) -> bool {
        if self.driver.is_none() || self.shutdown_ready() {
            self.shutdown_refused_unknown = false;
            tracing::info!("shutdown: latching EStop so the control loop stops before exit (in memory only, not persisted; the next start is Idle)");
            self.estop_state.latch();
            return true;
        }
        // Cold-boot / never-enabled idle arm: `set_driver` initializes `torque_intent = On`
        // fail-closed, so `shutdown_ready()` is false until physical disarm is causally proven.
        // Actively poll every axis read-only (`0xCC`, bounded <= 300 ms under `deadline`); accept
        // ONLY when every axis replies disabled with fresh causal evidence. Never send enable
        // frames, and keep repeated refused-Unknown shutdowns on the ProcessExit disable path.
        if self.eligible_for_readonly_disarm_proof()
            && !self.shutdown_refused_unknown
            && self.probe_driver_disarmed_until(deadline)
        {
            self.record_cold_boot_disarm_proof();
            self.shutdown_refused_unknown = false;
            tracing::info!(
                "shutdown: hardware disarm causally confirmed on all axes; latching EStop in memory before exit (not persisted; the next start is Idle)"
            );
            self.estop_state.latch();
            return true;
        }
        // Signal-path shutdown vs protective-cut exception:
        // - A normal SIGTERM / Ctrl-C with an energized/holding arm latches E-Stop and refuses
        //   shutdown (returns `false`), keeping the process running and in control (motors hold).
        // - Repeated shutdown when `!is_torqued || disable_pending` (e.g. startup before any
        //   enable or an in-flight disable) issues `DisableProtective { ProcessExit }` so process
        //   exit disables motors in any pose.
        if !self.safety.state().is_stopped() {
            self.execute_stop(StopPolicy::EStopLatch);
        }
        let hw_known_enabled = self.driver.as_ref().is_some_and(|d| {
            d.try_with_ref(|drv| {
                drv.enable_state() == EnableState::Enabled
                    || drv.has_any_joint_enabled()
                    || drv.has_known_partial_enable()
            })
            .unwrap_or(true)
        });
        let unaccepted_idle =
            self.torque_intent == TorqueIntent::On && (!self.is_torqued || self.disable_pending);
        if unaccepted_idle && self.shutdown_refused_unknown {
            let exit_kind = crate::driver::DisableKind::DisableProtective {
                reason: crate::driver::DisableProtectiveReason::ProcessExit,
            };
            if self.rt_executor_owned {
                if self.torque_transition.is_none() {
                    let _ = self.request_disable_bounded(exit_kind);
                }
            } else {
                let outcome = self.attempt_shutdown_protective_disable(PANIC_DISABLE_TIMEOUT);
                outcome.log("Repeated shutdown protective disable", PANIC_DISABLE_TIMEOUT);
            }
            if self.shutdown_ready() {
                self.shutdown_refused_unknown = false;
                tracing::info!("shutdown: latching EStop so the control loop stops before exit (in memory only, not persisted; the next start is Idle)");
                self.estop_state.latch();
                return true;
            }
        }
        // Only arm repeated-shutdown Unknown fallback when hardware is NOT known enabled (DRV-F1 B7).
        self.shutdown_refused_unknown = unaccepted_idle && !hw_known_enabled;
        false
    }

    /// Final shutdown admission executed under the owner's exclusive authority.
    /// Returns `true` and irreversibly sets `shutdown_committed` + `e_stop_latched` when
    /// shutdown is safe so no concurrent `clear_fault`, torque enable, or motion command can
    /// re-energize the arm before process exit; hardware disable paths remain permitted.
    pub fn commit_shutdown(&mut self) -> bool {
        self.commit_shutdown_until(Instant::now() + crate::driver::SHUTDOWN_DISARM_PROOF_WINDOW)
    }

    /// Bounded variant of [`Self::commit_shutdown`] that actively refreshes missing/stale
    /// read-only disarm proof (`0xCC`) within `deadline` when software state is already disarmed.
    pub fn commit_shutdown_until(&mut self, deadline: Instant) -> bool {
        if self.driver.is_some()
            && self.shutdown_ready()
            && self.eligible_for_readonly_disarm_proof()
            && !crate::rt_core::executor::core_is_confirmed_disarmed(self)
            && !self.probe_driver_disarmed_until(deadline)
        {
            return false;
        }
        let ready = self.driver.is_none()
            || (self.shutdown_ready()
                && !self.is_torqued
                && !self.disable_pending
                && !self.torque_transition_pending()
                && crate::rt_core::executor::core_is_confirmed_disarmed(self));
        if !ready {
            return false;
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.revoke_sessions_on_stop("shutdown");
        self.shutdown_committed = true;
        self.estop_state.latch();
        self.stop_motion();
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::ProcessExit {
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                });
            if !self.rt_executor_owned {
                self.flush_torque_shadow();
            }
        }
        true
    }

    pub fn is_fully_settled_hold(&self) -> bool {
        if self.gravity_comp.is_some() { return false; }
        // Safe condition 1: Confirmed disabled (torque off, Idle, not disable pending)
        if !self.is_torqued && matches!(self.safety.state(), SafetyState::Idle) && !self.disable_pending {
            return true;
        }

        let (has_velocity_capability, is_valid) = if self.driver.is_none() {
            (true, true)
        } else {
            self.read_driver_for_tick()
                .map_or((false, false), |drv| (drv.capabilities().velocity_feedback, drv.is_feedback_valid()))
        };

        if !is_valid {
            return false;
        }

        let velocity_settled = if has_velocity_capability {
            self.measured_velocities.len() == self.config.dof
                && self.measured_velocities.iter().all(|v| v.is_finite() && v.abs() <= 0.05)
        } else {
            true
        };

        let arm_joints = self.config.arm_joints();
        let tracking_settled = !self.config.joint_tracking.as_ref().is_some_and(|jt| {
            jt.enabled
                && self
                    .tracking
                    .has_residual_lead(&self.commanded_positions, &arm_joints)
        });

        (self.safety.state() == &SafetyState::Armed || self.safety.state() == &SafetyState::Idle)
            && self.active_trajectory.is_none()
            && self.servo_target.is_none()
            && self.commanded_velocities.len() == self.config.dof
            && self.commanded_velocities.iter().all(|v| *v == 0.0)
            && self.hardware_feedback_fresh(Duration::from_millis(100))
            && velocity_settled
            && tracking_settled
    }

    pub fn is_fully_settled_armed_hold(&self) -> bool {
        self.is_fully_settled_hold()
    }

    // ---- actions ----

    pub(crate) fn now_s(&self) -> f64 { self.start_time.elapsed().as_secs_f64() }

    // ---- grasp ----

    // ---- macro actions (crate::macro_action) ----

    /// Gripper lead cap (`gripper_policy.max_lead_rad`): with torque on and fresh feedback the
    /// gripper command stays within the bound ahead of the measured position in the closing
    /// direction, whatever wrote it (macro
    /// gripper moves, servo streams, whole-arm references, gravity compensation), so a blocked
    /// jaw cannot build a large position error.
    pub(super) fn gripper_lead_cap(&mut self) {
        let (Some(g), Some(lead)) = (self.config.gripper_index(), self.config.gripper_policy.as_ref().and_then(|p| p.max_lead_rad)) else { return; };
        if self.driver.is_none() || !self.is_torqued || !self.hardware_feedback_fresh(Duration::from_millis(100)) { return; }
        let (Some(&meas), Some(&cmd)) = (self.measured_positions.get(g), self.commanded_positions.get(g)) else { return; };
        if !meas.is_finite() || !cmd.is_finite() { return; }
        // closing direction only: an open ends on the open stop, nothing to crush there
        let dir = self.config.gripper_policy.as_ref().map_or(1.0, |p| if p.closed_rad >= p.open_rad { 1.0 } else { -1.0 });
        // stream contact regulator: the closing command stops at its limit
        let cmd = match self.gripper_stream_contact.filter(|c| self.gripper_stream_target.is_some() && c.limit_rad.is_finite()) {
            Some(c) if (cmd - c.limit_rad) * dir > 1e-6 => {
                let l = &self.config.limits[g];
                self.commanded_positions[g] = c.limit_rad.clamp(l.min_position, l.max_position);
                // the stepper restarts from rest at the limit (an open frame reverses at once)
                if let Some(v) = self.commanded_velocities.get_mut(g) { *v = 0.0; }
                self.note_motion_limit(MotionLimitReason::GripperForce, Some(g));
                self.commanded_positions[g]
            }
            _ => cmd,
        };
        if (cmd - meas) * dir > lead {
            let capped = meas + dir * lead;
            let l = &self.config.limits[g];
            self.commanded_positions[g] = capped.clamp(l.min_position, l.max_position);
        }
    }

    /// J7 stall freeze for whole-arm references. A joint quintic (park, home, presets), path or
    /// replay writes all seven joints each tick; without this a J7 target past an object in the
    /// jaw is driven at full motor torque (the stall latch alone is overwritten next tick, and
    /// re-arms after the torque drops, squeezing in steps). Once J7 is stalled, or an object is
    /// held, J7 stays at the hold point while the reference pushes further in the stall
    /// direction; a reference moving J7 the other way ends the freeze.
    pub(super) fn gripper_hold_filter(&mut self, positions: &mut [f32], velocities: &mut [f32]) {
        let Some(g_idx) = self.config.gripper_index() else { return; };
        if positions.len() <= g_idx || self.commanded_positions.len() <= g_idx { return; }
        if self.gripper_motion_hold.is_none() {
            let close_sign = self.config.gripper_policy.as_ref()
                .map_or(1.0, |p| if p.closed_rad >= p.open_rad { 1.0 } else { -1.0 });
            if self.gripper_stalled {
                self.gripper_motion_hold = Some((self.commanded_positions[g_idx], self.gripper_stall_sign));
            } else if self.held.is_some() {
                self.gripper_motion_hold = Some((self.commanded_positions[g_idx], close_sign));
            }
        }
        let Some((hold, sign)) = self.gripper_motion_hold else { return; };
        let push = (positions[g_idx] - hold) * sign;
        if push > 0.0 {
            positions[g_idx] = hold;
            if velocities.len() > g_idx { velocities[g_idx] = 0.0; }
        } else if push < -1e-3 {
            self.gripper_motion_hold = None;
            return;
        }
        if self.pending_normal_unload {
            // a hold at the jaw's own closed stop is not an object
            let gap = self.config.gripper_policy.as_ref().map_or(f32::INFINITY, |p| (hold - p.closed_rad).abs() - p.holding_gap_rad);
            if gap >= 0.0 && !self.park_gripper_contact {
                if !self.rt_executor_owned {
                    tracing::info!("Park: gripper held at {hold:.3} rad (object in the jaw); releasing it at the resting pose");
                }
                self.park_gripper_contact = true;
            }
        }
    }

    /// Release step of a park that carried an object, run once the park trajectory ends:
    /// open [`PARK_RELEASE_OPEN_RAD`] past the contact, then close (stall-protected gripper
    /// moves), then the unload settle. Returns true while the release owns the tick.
    ///
    /// Exempt from `Admitted` (Invariant 1a): system-initiated stop-direction motion
    /// completing a `NormalUnload` park so the gripper releases safely before torque-off.
    pub(super) fn step_park_release(&mut self) -> bool {
        if self.is_pos_force_j7_stale() {
            return false;
        }
        let next = match self.park_release {
            None if self.park_gripper_contact => ParkRelease::Opening,
            Some(ParkRelease::Opening) => ParkRelease::Closing,
            Some(ParkRelease::Closing) => {
                self.park_release = None;
                self.park_gripper_contact = false;
                return false;
            }
            None => return false,
        };
        let Some(gripper_idx) = self.config.gripper_index() else {
            self.park_gripper_contact = false;
            return false;
        };
        let Some(policy) = self.config.gripper_policy.as_ref() else {
            self.park_gripper_contact = false;
            return false;
        };
        let (command, fraction) = match next {
            ParkRelease::Opening => {
                let open_sign = if policy.open_rad >= policy.closed_rad { 1.0 } else { -1.0 };
                let target = self.commanded_positions[gripper_idx] + open_sign * PARK_RELEASE_OPEN_RAD;
                let span = policy.open_rad - policy.closed_rad;
                let f = if span.abs() > 1e-6 { ((target - policy.closed_rad) / span).clamp(0.0, 1.0) } else { 1.0 };
                (GripperCommand::Move, Some(f64::from(f)))
            }
            ParkRelease::Closing => (GripperCommand::Close, None),
        };
        let close_cap = self.gripper_close_speed_limit();
        let ceiling_est = self.gripper_ceiling_torque_estimate();
        let mut boxed = if let Some(spare) = self.park_release_spare.take() {
            spare
        } else if !self.rt_executor_owned {
            let req = GripperRequest { command, fraction, grasp: false, speed: PARK_RELEASE_SPEED, timeout_s: None, tag: None };
            match GripperMotion::try_with_close_speed_limit(
                req, policy.clone(), self.config.limits[gripper_idx].clone(),
                self.commanded_positions[gripper_idx], gripper_idx, close_cap,
            ) {
                Ok(m) => Box::new(m.with_ceiling_torque_estimate(ceiling_est)),
                Err(e) => {
                    tracing::error!("Park release skipped: {e}");
                    return false;
                }
            }
        } else {
            return false;
        };
        if boxed.reinit_park_release_in_place(
            command, fraction, PARK_RELEASE_SPEED, &self.config.limits[gripper_idx],
            self.commanded_positions[gripper_idx], gripper_idx, close_cap, ceiling_est,
        ).is_err() {
            self.park_release_spare = Some(boxed);
            if !self.rt_executor_owned { tracing::error!("Park release skipped: invalid pos-force close speed"); }
            return false;
        }
        // the stall latch of the carry would block the opening move's first ticks otherwise
        self.gripper_stalled = false;
        self.gripper_high_torque_ticks = 0;
        self.gripper_low_torque_ticks = 0;
        self.gripper_stall_window.reset();
        self.gripper_motion_hold = None;
        if next == ParkRelease::Opening { self.releasing = true; }
        let duration = boxed.expected_duration();
        let target_rad = boxed.target;
        let timeout_s = boxed.timeout_s;
        let wire_ceil = boxed.wire_velocity_ceiling();
        if !self.rt_executor_owned {
            tracing::info!("Park release: {} joint {} {:.3} -> {:.3} rad", if next == ParkRelease::Opening { "opening" } else { "closing" },
                gripper_idx + 1, self.commanded_positions[gripper_idx], target_rad);
        }
        if let Some(park) = self.watchdog_park.as_mut() { park.budget_s += timeout_s.max(duration); }
        self.gripper_hold_vel_ceiling = Some(wire_ceil);
        self.target_positions[gripper_idx] = target_rad;
        if self.watchdog_park.is_some() {
            // Lease-watchdog park owns gripper release independent of client stops; stamp from
            // local owner state (never `max(local, requested)`).
            self.j7_admitted_stamp = Some(crate::driver::PosForceIntentStamp {
                stop_gen: self.stop_gen.load(Ordering::SeqCst),
                estop_epoch: self.safety.epoch_id(),
                lease_gen: 0,
            });
        }
        self.replace_active_trajectory(Some(ActiveTrajectory::new(None, Reference::Gripper(boxed))));
        self.park_release = Some(next);
        true
    }

    /// Replaces `self.active_trajectory`, retiring any previous trajectory without RT frees.
    pub(crate) fn replace_active_trajectory(&mut self, next: Option<ActiveTrajectory>) {
        if let Some(old) = std::mem::replace(&mut self.active_trajectory, next) {
            self.retire_active_trajectory(old);
        }
    }

    /// Queue a replaced `ActiveTrajectory` for off-thread drop when owned by the RT executor
    /// (prevents dropping heap-backed `motion_limits` / path data on `armd-control`).
    /// Zero-heap trajectories (e.g., pinned `PlanSlotRef` paths/replays or inline quintics)
    /// drop immediately so `PlanSlotRef` pins return to 0 without consuming `retired_trajectories`;
    /// clean `Box<GripperMotion>` shells are recycled into `park_release_spare` or `gripper_motion_spare` (0 dealloc).
    pub(crate) fn retire_active_trajectory(&mut self, traj: ActiveTrajectory) {
        if !self.rt_executor_owned {
            drop(traj);
            return;
        }
        if traj.motion_limits.is_none() && traj.then.is_none() {
            if let Reference::Gripper(mut b) = traj.reference {
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
                    return;
                }
                if !b.has_inner_heap_storage() && self.gripper_motion_spare.is_none() {
                    self.gripper_motion_spare = Some(b);
                    return;
                }
                let traj = ActiveTrajectory {
                    action_id: traj.action_id,
                    reference: Reference::Gripper(b),
                    then: None,
                    span: traj.span,
                    motion_limits: None,
                    motion_vel_caps: traj.motion_vel_caps,
                    motion_max_velocity: traj.motion_max_velocity,
                };
                if self.retired_trajectories.len() < self.retired_trajectories.capacity() {
                    self.retired_trajectories.push_back(traj);
                }
                return;
            }
        }
        if !traj.has_heap_storage() {
            drop(traj);
            return;
        }
        if self.retired_trajectories.len() < self.retired_trajectories.capacity() {
            self.retired_trajectories.push_back(traj);
        }
    }

    /// Boxes a `GripperMotion` reusing `gripper_motion_spare` when available (0 alloc on RT).
    pub(crate) fn box_gripper_motion_rt(&mut self, motion: GripperMotion) -> Box<GripperMotion> {
        if let Some(mut spare) = self.gripper_motion_spare.take() {
            *spare = motion;
            spare
        } else {
            Box::new(motion)
        }
    }

    /// Zero-alloc `Copy` snapshot of the active tool's free-motion gripper torque baseline
    /// (`None` if unlearned or belonging to another tool/follower/stroke).
    pub(crate) fn gripper_baseline_data_rt(&self) -> Option<crate::gripper_baseline::BaselineData> {
        let policy = self.config.gripper_policy.as_ref()?;
        let tool = &self.config.active_tool;
        let follower = self.follower.as_deref();
        let lo = policy.closed_rad.min(policy.open_rad);
        let hi = policy.closed_rad.max(policy.open_rad);
        self.gripper_baseline
            .as_ref()
            .filter(|b| {
                b.belongs_to(&tool.id, &tool.config_digest, follower)
                    && (b.lo_rad - lo).abs() < 1e-3
                    && (b.hi_rad - hi).abs() < 1e-3
            })
            .map(crate::gripper_baseline::FreeTorqueBaseline::data_snapshot)
    }

    /// Zero-alloc RT check for close-torque feedback capability using the cached scalar
    /// `driver_has_torque_feedback` (no mutex lock and no formatted error string allocation).
    pub(crate) fn check_gripper_close_torque_feedback_rt(
        &self,
        command: crate::motion::GripperCommand,
        fraction: Option<f64>,
        policy: &crate::model::GripperPolicy,
        gripper_idx: usize,
    ) -> Result<(), &'static str> {
        if policy.max_close_torque_nm.is_none() {
            return Ok(());
        }
        let close_dir = if policy.closed_rad >= policy.open_rad { 1.0_f32 } else { -1.0_f32 };
        let cur = self.commanded_positions.get(gripper_idx).copied().unwrap_or(0.0);
        let target = match command {
            crate::motion::GripperCommand::Open => policy.open_rad,
            crate::motion::GripperCommand::Close => policy.closed_rad,
            crate::motion::GripperCommand::Move => {
                policy.closed_rad + fraction.unwrap_or(0.0) as f32 * (policy.open_rad - policy.closed_rad)
            }
            crate::motion::GripperCommand::Seek => policy.closed_rad + close_dir * 0.6,
        };
        let is_closing = matches!(command, crate::motion::GripperCommand::Close | crate::motion::GripperCommand::Seek)
            || (target - cur) * close_dir > 1e-4;
        if is_closing && self.driver.is_some() && !self.driver_has_torque_feedback {
            return Err("UNSUPPORTED");
        }
        Ok(())
    }

    /// Pops one preallocated `MoveSamples` buffer for zero-alloc RT `LearnMode::Opening` recording.
    pub(crate) fn take_move_samples_spare_rt(&mut self) -> Option<crate::gripper_baseline::MoveSamples> {
        self.move_samples_spares.pop()
    }

    /// Recycles a `MoveSamples` buffer back into `move_samples_spares` without RT deallocation.
    pub(crate) fn recycle_move_samples_rt(&mut self, mut samples: crate::gripper_baseline::MoveSamples) {
        samples.reset();
        if self.move_samples_spares.len() < self.move_samples_spares.capacity() {
            self.move_samples_spares.push(samples);
        }
    }

    /// Single shared RT point for attaching baseline data and (only when `motion` will record
    /// learning) one preallocated `MoveSamples` spare from `move_samples_spares`.
    /// Must be called ONLY after final admission and start checks succeed; any unused spare is
    /// returned to the pool without dropping on `armd-control`.
    pub(crate) fn attach_gripper_learning_rt(
        &mut self,
        motion: crate::motion::GripperMotion,
        baseline: Option<crate::gripper_baseline::BaselineData>,
        learn: crate::motion::LearnMode,
    ) -> crate::motion::GripperMotion {
        let spare = if motion.will_record_learning(learn) {
            self.take_move_samples_spare_rt()
        } else {
            None
        };
        let (motion, unused) = motion.with_baseline_data_and_samples(baseline, learn, spare);
        if let Some(buf) = unused {
            self.recycle_move_samples_rt(buf);
        }
        motion
    }

    /// Test-only accessor for the current RT learning-buffer spare pool depth.
    #[doc(hidden)]
    pub fn test_move_samples_spare_count(&self) -> usize {
        self.move_samples_spares.len()
    }

    /// Computes the commanded joint vector that will hold immediately after quiescing an active
    /// macro gripper child (without mutating `self` so refused replacements leave state untouched).
    /// Returns `(dof, will_quiesce_macro_gripper)`.
    pub(crate) fn post_quiesce_commanded_positions(
        &self,
        out: &mut [f32; crate::rt_core::mailbox::MAX_DOF],
    ) -> (usize, bool) {
        let dof = self.commanded_positions.len().min(crate::rt_core::mailbox::MAX_DOF);
        out[..dof].copy_from_slice(&self.commanded_positions[..dof]);
        let active_macro_gripper = self.macro_run.as_ref().is_some_and(|m| {
            self.active_trajectory.as_ref().is_some_and(|a| {
                a.action_id == Some(m.id) && matches!(a.reference, Reference::Gripper(_))
            })
        });
        if active_macro_gripper {
            if let Some(g) = self.config.gripper_index().filter(|&g| g < dof) {
                if let Some(&meas) = self.measured_positions.get(g).filter(|v| v.is_finite()) {
                    out[g] = meas;
                }
            }
        }
        (dof, active_macro_gripper)
    }

    /// Updates `self.held` on RT without allocating or freeing heap memory.
    pub(crate) fn set_held_rt(&mut self, label: crate::model::HolderLabel, spill_safe: bool) {
        let src = label.as_str();
        if let Some(existing) = self.held.as_mut() {
            existing.spill_safe = spill_safe;
            if existing.label.capacity() < src.len()
                && self.held_label_spare.capacity() >= src.len()
            {
                let old = std::mem::replace(
                    &mut existing.label,
                    std::mem::take(&mut self.held_label_spare),
                );
                if old.capacity() > 0 && self.held_label_spare.capacity() == 0 {
                    // Only reachable if prewarm wasn't called; recycle instead of dropping on RT.
                    let mut recycled = old;
                    recycled.clear();
                    self.held_label_spare = recycled;
                }
            }
            existing.label.clear();
            if existing.label.capacity() >= src.len() {
                existing.label.push_str(src);
            }
        } else {
            let mut buf = if self.held_label_spare.capacity() >= src.len() {
                std::mem::take(&mut self.held_label_spare)
            } else {
                String::new()
            };
            buf.clear();
            if buf.capacity() >= src.len() {
                buf.push_str(src);
            }
            self.held = Some(HeldObject {
                label: buf,
                spill_safe,
            });
        }
    }

    /// Clears `self.held`, recycling its heap `label` buffer into `held_label_spare` on RT
    /// so clearing a held object never deallocates on `armd-control`.
    pub(crate) fn clear_held_rt(&mut self) {
        if let Some(old) = self.held.take() {
            if self.rt_executor_owned && old.label.capacity() > self.held_label_spare.capacity() {
                let mut recycled = old.label;
                recycled.clear();
                self.held_label_spare = recycled;
            }
        }
    }

    /// True while macro parent `parent_id` has an active child trajectory, quintic settle,
    /// deferred TX completion (`MacroSub`), or `SubState::Running`.
    pub(crate) fn macro_child_running(&self, parent_id: u64) -> bool {
        let Some(run) = self.macro_run.as_ref().filter(|m| m.id == parent_id) else {
            return false;
        };
        if run.sub == crate::macro_action::SubState::Running || run.hold_pending_bus_accept {
            return true;
        }
        if self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(parent_id)) {
            return true;
        }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == parent_id) {
            return true;
        }
        self.pending_deferred_done.as_ref().is_some_and(|d| {
            matches!(d.target, DeferredCompletionTarget::MacroSub { action_id, .. } if action_id == parent_id)
        })
    }

    /// Allocation-free quiesce of any active child motion/settle/deferred completion belonging
    /// to macro `id` (preserving gripper measured-position hold safety semantics!).
    /// Returns true if any active trajectory, settle, or deferred `MacroSub` was quiesced.
    pub(crate) fn quiesce_macro_children_rt(&mut self, id: u64) -> bool {
        let mut quiesced = false;
        let carried = self.active_trajectory.as_ref().is_some_and(|a| a.action_id == Some(id));
        if carried {
            quiesced = true;
            let is_gripper = self
                .active_trajectory
                .as_ref()
                .is_some_and(|a| matches!(a.reference, Reference::Gripper(_)));
            if is_gripper {
                self.hold_gripper_at_measured();
                self.releasing = false;
            }
            self.replace_active_trajectory(None);
            self.tracking.invalidate();
            self.commanded_velocities.fill(0.0);
            self.commanded_accelerations.fill(0.0);
            crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
            self.safety.mark_motion_finished();
        }
        if self.quintic_settle.as_ref().is_some_and(|(sid, _)| *sid == id) {
            quiesced = true;
            self.quintic_settle = None;
        }
        if self.pending_deferred_done.as_ref().is_some_and(|d| {
            matches!(d.target, DeferredCompletionTarget::MacroSub { action_id, .. } if action_id == id)
        }) {
            quiesced = true;
            self.pending_deferred_done = None;
        }
        if quiesced && self.is_rt_owner() && self.driver.is_some() {
            if let Some(m) = self.macro_run.as_mut().filter(|m| m.id == id) {
                m.hold_pending_bus_accept = true;
            }
        }
        quiesced
    }

    /// Zero-allocation `Copy` owner metadata published on every control tick into `RtSample`.
    pub(crate) fn published_owner_meta(&self) -> crate::rt_core::publish::PublishedOwnerMeta {
        use crate::rt_core::publish::{PublishedHeld, PublishedMacroState, PublishedOwnerMeta};
        let authority = crate::rt_core::broker::Authority {
            lease_generation: self.current_active_lease_gen(),
            epoch: self.safety.epoch_id(),
            stop_generation: self.current_stop_gen(),
            config_revision: self.workspace_revision,
            parent_generation: self
                .last_action
                .as_ref()
                .filter(|a| a.is_running())
                .map_or(0, |a| a.id),
        };
        let held = self.held.as_ref().map(|h| PublishedHeld {
            label: crate::model::HolderLabel::new(&h.label),
            spill_safe: h.spill_safe,
        });
        let macro_state = self
            .macro_run
            .as_ref()
            .map(|m| PublishedMacroState {
                active_id: m.id,
                alive: !m.stop_requested
                    && !m.finishing
                    && self
                        .last_action
                        .as_ref()
                        .is_some_and(|a| a.id == m.id && a.is_running()),
                children: m.children,
                child_seq: m.child.seq,
                child_running: self.macro_child_running(m.id),
                child_sub: m.sub,
                child_watch: m.child.watch,
            })
            .unwrap_or_default();
        #[cfg(feature = "policy-runtime")]
        let policy_busy = self.policy_loading || self.policy_run.is_some();
        #[cfg(not(feature = "policy-runtime"))]
        let policy_busy = false;
        #[cfg(not(target_arch = "wasm32"))]
        let (mode_switch, prev_mode_switch_terminal) = if self.mode_switch.is_idle() {
            (None, None)
        } else if self.mode_switch.active_session.is_some() {
            (
                self.current_mode_switch_snapshot()
                    .map(crate::rt_core::publish::PublishedModeSwitch::from),
                self.mode_switch
                    .latest_terminal()
                    .map(crate::rt_core::publish::PublishedModeSwitch::from),
            )
        } else {
            (
                self.mode_switch
                    .latest_terminal()
                    .map(crate::rt_core::publish::PublishedModeSwitch::from),
                self.mode_switch
                    .previous_terminal()
                    .map(crate::rt_core::publish::PublishedModeSwitch::from),
            )
        };
        PublishedOwnerMeta {
            arm: self.core.id(),
            boot: self.boot_id,
            authority,
            control_release_seq: self.control_release_seq,
            motion_generation: self.motion_generation,
            calibration_revision: self.calibration_revision,
            collision_revision: self.collision.revision,
            poses_revision: self.poses.revision,
            sequences_revision: self.sequences.revision,
            grasp_setup_revision: self.grasp_setup_revision,
            gripper_calib_revision: self.gripper_calib_revision,
            power_budget_revision: self.power_budget_revision,
            tool_commissioning_revision: self.tool_commissioning_revision,
            tool_digest_hash: crate::rt_core::ops::tool_digest_hash(&self.config.active_tool.config_digest),
            motion_active: self.active_trajectory.is_some() || self.servo_target.is_some(),
            config_inhibited: self.core.motion_inhibited_by_config(),
            calibration_busy: self.calibration_busy,
            feedback_valid: self.commit_feedback_valid(),
            is_torqued: self.is_torqued,
            pending_rearm: self.pending_rearm,
            pending_normal_unload: self.pending_normal_unload,
            collision_hold: self.collision.hold.is_some(),
            handover_settling: self.handover_settling_since.is_some(),
            gravity_comp_active: self.gravity_comp.is_some(),
            policy_busy,
            zero_check_persisting: self.zero_check_persisting.is_some(),
            torque_or_power_hold: self.torque_guard.hold.is_some() || self.power.gov.hold.is_some(),
            static_restricted: self.power.is_static_restricted(),
            watchdog_park_active: self.watchdog_park.is_some(),
            active_lease_present: self.active_lease.is_some(),
            held,
            macro_state,
            #[cfg(not(target_arch = "wasm32"))]
            mode_switch,
            #[cfg(not(target_arch = "wasm32"))]
            prev_mode_switch_terminal,
        }
    }
}
