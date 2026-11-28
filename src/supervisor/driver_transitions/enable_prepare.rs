//! Torque enable preparation, failed-hold transitions, and enable reducer startup/cancel.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

impl SupervisorInner {
    pub fn enable_torque(admitted: Admitted<'_>) -> Result<SafetyState, SafetyError> {
        let (this, proof) = admitted.consume()?;
        this.enable_torque_inner(&proof)
    }

    pub(crate) fn enable_torque_inner(&mut self, _proof: &crate::supervisor::AdmittedProof) -> Result<SafetyState, SafetyError> {
        if self.torque_transition.is_some() {
            return Err(SafetyError::ClearFaultRejected("A torque transition is pending".into()));
        }
        // Torque changes and re-arming are refused in gravity compensation: exit first.
        self.gravity_comp_servo_busy()?;
        let (was_torqued, prev_gripper_cmd) = self.enable_prepare()?;
        let mit_qualified = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
        let mut outcome = self.start_enable_reducer(
            EnableOrigin::BrokerTransition,
            was_torqued,
            prev_gripper_cmd,
            None,
            false,
            mit_qualified,
        );
        let mut first = true;
        let res = loop {
            match outcome {
                EnableStepOutcome::Armed(state) => break Ok(state),
                EnableStepOutcome::Failed {
                    message,
                    feedback_unavailable,
                    estop_interrupted,
                    ..
                } => {
                    break Err(if feedback_unavailable {
                        SafetyError::FeedbackUnavailable
                    } else if estop_interrupted {
                        SafetyError::EStopActive
                    } else {
                        SafetyError::ClearFaultRejected(message.as_str().to_string())
                    });
                }
                EnableStepOutcome::Pending { .. } => {
                    if !first {
                        if let Some(ref d) = self.driver {
                            let _ = d.with_ref(|drv| drv.clock_pause(Duration::from_micros(500)));
                        }
                    }
                    first = false;
                    let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() else {
                        break Err(SafetyError::ClearFaultRejected(
                            "Enable transition reducer state lost".into(),
                        ));
                    };
                    let (next, _) = self.poll_enable_reducer(reducer, None);
                    #[cfg(not(target_arch = "wasm32"))]
                    if !self.rt_executor_owned {
                        self.flush_torque_shadow();
                    }
                    outcome = next;
                }
            }
        };
        #[cfg(not(target_arch = "wasm32"))]
        if !self.rt_executor_owned {
            self.flush_torque_shadow();
        }
        res
    }

    /// Exempt from `Admitted` (Invariant 1c): torque disable is a stop-direction action that
    /// removes actuator output and must never be blocked by a Stop or admission check (still
    /// subject to resting-pose and torque-off safety checks).
    pub fn disable_torque(&mut self) -> Result<SafetyState, SafetyError> {
        let res = (|| {
            if self.torque_transition.is_some() {
                return Err(SafetyError::ClearFaultRejected("A torque transition is pending".into()));
            }
            // Torque changes and re-arming are refused in gravity compensation: exit first.
            self.gravity_comp_servo_busy()?;
            self.torque_off(false)
        })();
        #[cfg(not(target_arch = "wasm32"))]
        if !self.rt_executor_owned {
            self.flush_torque_shadow();
        }
        res
    }

    /// Enable, before the driver's handshake: every refusal (gravity compensation, driver
    /// limits, handover, pending disable, E-stop, fault, MIT gate), then the feedback read
    /// that seeds the commanded state. Returns `(was_torqued, prev_gripper_cmd)`.
    pub(in crate::supervisor) fn enable_prepare(&mut self) -> Result<(bool, Option<f32>), SafetyError> {
        self.enable_prepare_inner(true, true, false)
    }

    pub(in crate::supervisor) fn enable_prepare_inner(
        &mut self,
        allow_bus_read: bool,
        allow_sync_mit: bool,
        nonblocking_lock: bool,
    ) -> Result<(bool, Option<f32>), SafetyError> {
        if self.internal_fault_lane.is_fenced() || self.shutdown_committed {
            return Err(SafetyError::EStopActive);
        }
        if self.core.motion_inhibited_by_config() {
            return Err(SafetyError::ClearFaultRejected(
                "CONFIG_BUSY: configuration or hardware commissioning reconciliation is active; rearm is blocked".into(),
            ));
        }
        if let Some(ref err) = self.driver_config_error {
            return Err(SafetyError::MotionForbiddenInFaultState(format!("Driver limit configuration error: {err}")));
        }
        if self.handover_settling_since.is_some() {
            return Err(SafetyError::HandoverSettling);
        }
        if self.disable_pending { return Err(SafetyError::ClearFaultRejected("Torque disable is still pending".into())); }
        // Atomic e-stop check
        if self.estop_pending() {
            return Err(SafetyError::EStopActive);
        }

        // Strict pre-check: If stopped (E-Stop or Fault), reject IMMEDIATELY before touching driver!
        if self.safety.state().is_stopped() {
            if let SafetyState::Fault(ref reason) = self.safety.state() {
                return Err(SafetyError::FaultActive(reason.clone()));
            }
            return Err(SafetyError::EStopActive);
        }
        if let Some(d) = self.driver.as_ref() {
            if let Some(driver) = d.read() {
                if let Err((max_frames, required)) = driver.tx().check_watchdog_tx_budget_for_enable() {
                    return Err(SafetyError::ClearFaultRejected(format!(
                        "WATCHDOG_BUDGET_INSUFFICIENT: tx_budget.max_frames ({max_frames}) < required ({required}) for energized watchdog continuity"
                    )));
                }
            }
        }

        // RT executor step 1 does not support synchronous MIT register qualification/handover or
        // gravity-compensation mode switches; fail closed before touching any sync driver helpers.
        if (!allow_sync_mit || self.rt_executor_owned)
            && (self.gravity_comp.is_some()
                || self.mit_active()
                || self.mit.revoke_pending
                || crate::mit::mit_joint_mask(&self.config) != 0)
        {
            return Err(SafetyError::ClearFaultRejected(
                "UNSUPPORTED: MIT mode qualification/handover and gravity compensation are not supported under synchronous RT executor step 1".into(),
            ));
        }

        // MIT tracking (Phase 2): every arm and rearm rechecks the qualification against a fresh
        // register readback (nonzero TIMEOUT, identity, ranges); unqualified hardware refuses
        // MIT before any motion. MIT not requested: no readback, no change.
        // A rearm while MIT owns joints hands them back first (at rest), so neither the
        // register readback below nor the driver's enable runs under an active MIT owner
        // (no dynamic output during the reads, no silent mode restore).
        if self.mit_active() {
            #[cfg(not(target_arch = "wasm32"))]
            if self.driver.is_some() && self.driver_supports_nonblocking_mode_switch() {
                self.mit.armed_ok = false;
                self.active_trajectory = None;
                self.servo_target = None;
                self.mit.revoke_pending = true;
                return Err(SafetyError::ClearFaultRejected(
                    "MIT_HAND_BACK_REQUIRED: MIT tracking must hand back via PosVelHandback before rearm".into(),
                ));
            }
            self.mit.armed_ok = false;
            if let Err(e) = self.mit_hand_back("rearm") {
                // not at rest: the authorization is gone, so the motion brakes to rest and MIT
                // hands back there (the tick); no motion starts meanwhile
                if self.mit_active() {
                    self.active_trajectory = None;
                    self.servo_target = None;
                    self.mit.revoke_pending = true;
                }
                return Err(SafetyError::ClearFaultRejected(e));
            }
        }
        if let Err(e) = self.mit_gate() {
            // revoked before returning: no handover until a later arm passes the gate
            self.mit.armed_ok = false;
            let msg = format!("MIT_UNQUALIFIED: MIT tracking refused: {e}");
            tracing::error!("{msg}");
            self.mit_gate_error = Some(msg.clone());
            return Err(SafetyError::ClearFaultRejected(msg));
        }
        self.mit_gate_error = None;

        // Enable hardware torque so bus devices wake up and publish state, then verify physical feedback.
        if let Some(mut d) = self.driver.take() {
            let prep_res: Result<(), SafetyError> = (|| {
                // Secondary double-check latch right before hardware actuation
                if self.estop_pending() {
                    return Err(SafetyError::EStopActive);
                }
                let run_prep = |driver: &mut dyn crate::driver::ArmDriver| -> Result<(), SafetyError> {
                    if self.estop_pending() {
                        return Err(SafetyError::EStopActive);
                    }
                    if allow_bus_read && !driver.is_feedback_valid() {
                        if let Ok(states) = driver.read_joint_states() {
                            let loads = driver.joint_loads_snapshot();
                            if !states.is_empty()
                                && driver.is_feedback_valid()
                                && SupervisorInner::states_valid(
                                    &states,
                                    driver.capabilities(),
                                    self.config.dof,
                                    loads.as_deref(),
                                )
                            {
                                Self::apply_feedback(
                                    &mut self.measured_positions,
                                    &mut self.measured_velocities,
                                    &mut self.torques,
                                    &mut self.temperatures,
                                    &mut self.loads,
                                    self.config.dof,
                                    &states,
                                    loads.as_deref(),
                                );
                                for (i, s) in states.iter().enumerate().take(self.config.dof) {
                                    if s.position.is_finite() {
                                        self.commanded_positions[i] = s.position;
                                        self.target_positions[i] = s.position;
                                    }
                                }
                            }
                        }
                    }
                    self.refresh_owner_driver_snapshot_from_ref(&*driver);
                    Ok(())
                };
                if nonblocking_lock {
                    match d.try_with_mut(run_prep) {
                        Ok(res) => res,
                        Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                            Err(SafetyError::ClearFaultRejected("DRIVER_BUSY".into()))
                        }
                        Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                            self.owner_driver_snapshot.invalidate();
                            Err(SafetyError::FeedbackUnavailable)
                        }
                    }
                } else {
                    match d.with_mut(run_prep) {
                        Some(res) => res,
                        None => {
                            self.owner_driver_snapshot.invalidate();
                            Err(SafetyError::FeedbackUnavailable)
                        }
                    }
                }
            })();
            self.driver = Some(d);
            prep_res?;
        }
        let was_torqued = self.is_torqued;
        let prev_gripper_cmd = if was_torqued {
            self.config.gripper_index().and_then(|i| self.commanded_positions.get(i).copied())
        } else {
            None
        };
        Ok((was_torqued, prev_gripper_cmd))
    }

    /// Enforces thermal protection inside the enable reducer (`Enabling` and post-enable
    /// `Validate` before arming): any finite joint temperature above `max_temperature` aborts
    /// the enable through `enter_failed_hold`, latches `FaultKind::ThermalTrip`, and starts
    /// `DisableProtective { Thermal }` without ever arming.
    pub(in crate::supervisor) fn check_enable_thermal_trip(
        &mut self,
        was_torqued: bool,
        temperature_available: bool,
    ) -> Option<EnableStepOutcome> {
        if !temperature_available
            || !self
                .temperatures
                .iter()
                .any(|t| t.is_finite() && *t > self.config.max_temperature)
        {
            return None;
        }
        // Run `enter_failed_hold` BEFORE `StopPolicy::ThermalTrip` because `enter_failed_hold`
        // clears `disable_pending` / `active_disable_kind`; `ThermalTrip` then latches
        // highest-severity `FaultKind::ThermalTrip` and starts `DisableProtective { Thermal }`.
        let message = self.enter_failed_hold(was_torqued, |buf| {
            buf.push_str("thermal trip during enable");
        });
        self.apply_stop_policy(StopPolicy::ThermalTrip);
        Some(EnableStepOutcome::Failed {
            rt_code: "THERMAL_TRIP",
            message,
            feedback_unavailable: false,
            estop_interrupted: false,
        })
    }

    /// Reducer-owned cancellation of an in-flight enable when `StopPolicy::ThermalTrip` takes
    /// over: cancels queued enable work, records the `ENABLE_FAILED` diagnostic with per-joint
    /// status (while `ThermalTrip` retains highest display precedence), and ends any pending RT
    /// `TorqueOn` command with `THERMAL_TRIP` before thermal protection starts `DisableProtective`.
    pub(crate) fn interrupt_enable_for_thermal(&mut self) {
        if !matches!(self.torque_transition, Some(TorqueTransition::Enable(_))) {
            return;
        }
        let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() else {
            return;
        };
        match reducer.origin {
            EnableOrigin::BrokerTransition => {
                self.core.begin_transition(Err("THERMAL_TRIP"));
            }
            EnableOrigin::ParkRearm { action_id, .. } => {
                self.finish_park_rearm_failed(action_id, "THERMAL_TRIP");
            }
            EnableOrigin::LegacyWs { token, slot, .. } => {
                let _ = self.enter_failed_hold(reducer.was_torqued, |buf| {
                    buf.push_str("interrupted by thermal trip");
                });
                self.settle_legacy_ws_failed(
                    token,
                    slot,
                    LegacyTorqueFailure::StaticCode("THERMAL_TRIP"),
                );
                return;
            }
            EnableOrigin::DetachedCancel => {}
        }
        let _ = self.enter_failed_hold(reducer.was_torqued, |buf| {
            buf.push_str("interrupted by thermal trip");
        });
    }

    /// Zero-allocation RT preflight for torque enable (all refusal paths return static codes).
    pub(in crate::supervisor) fn enable_prepare_rt_precheck(&mut self, authority: Option<EnableAuthority>) -> Result<(), &'static str> {
        #[cfg(not(target_arch = "wasm32"))]
        if (self.calibration_busy && self.motor_zero_busy)
            || self.commissioning.is_active()
            || self.sessions.blocks_ordinary_output(None)
        {
            return Err("MOTION_BUSY");
        }
        #[cfg(target_arch = "wasm32")]
        if (self.calibration_busy && self.motor_zero_busy) || self.commissioning.is_active() {
            return Err("MOTION_BUSY");
        }
        if self.core.motion_inhibited_by_config() {
            return Err("CONFIG_BUSY");
        }
        if self.torque_transition.is_some() {
            return Err("TRANSITION_BUSY");
        }
        if self.gravity_comp.is_some() {
            return Err("MOTION_BUSY");
        }
        if self.internal_fault_lane.is_fenced() {
            return Err("INTERNAL_FAULT");
        }
        if self.shutdown_committed {
            return Err("ESTOP_ACTIVE");
        }
        if self.driver_config_error.is_some() {
            return Err("MOTION_FORBIDDEN_IN_FAULT_STATE");
        }
        if self.handover_settling_since.is_some() {
            return Err("HANDOVER_SETTLING");
        }
        if self.disable_pending {
            return Err("DISABLE_PENDING");
        }
        if self.estop_pending() {
            return Err("ESTOP_ACTIVE");
        }
        if self.safety.state().is_stopped() {
            return Err(if matches!(self.safety.state(), SafetyState::Fault(_)) {
                "FAULT_ACTIVE"
            } else {
                "ESTOP_ACTIVE"
            });
        }
        if self.mit_active() || self.mit.revoke_pending {
            return Err("MOTION_BUSY");
        }
        if crate::mit::mit_joint_mask(&self.config) != 0 {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if crate::rt_core::sched::mit_refusal().is_some() {
                    return Err("MIT_UNQUALIFIED");
                }
                let Some(auth) = authority else {
                    return Err("MIT_UNQUALIFIED");
                };
                let Some(proof) = self.mit_gate_proof.take() else {
                    return Err("MIT_UNQUALIFIED");
                };
                let now = Instant::now();
                let matches_admission = proof.admission_request_id == auth.admission_request_id
                    && proof.connection_id == auth.owner_conn_id
                    && proof.lease_generation == auth.lease_generation
                    && proof.estop_epoch == auth.admitted_epoch
                    && proof.estop_epoch == self.safety.epoch_id()
                    && proof.stop_gen == auth.stop_gen
                    && proof.stop_gen == self.current_stop_gen()
                    && proof.calibration_revision == self.calibration_revision
                    && proof.motion_config_revision == self.motion_config_revision
                    && proof.zero_offsets_hash
                        == crate::mit::zero_offsets_hash(&self.joint_zero_offsets)
                    && now.saturating_duration_since(proof.verified_at) <= Duration::from_secs(5);
                if !matches_admission {
                    return Err("MIT_UNQUALIFIED");
                }
            }
            #[cfg(target_arch = "wasm32")]
            {
                return Err("UNSUPPORTED");
            }
        }
        Ok(())
    }

    /// Zero-allocation RT enable prepare (runs after [`Self::enable_prepare_rt_precheck`]).
    /// Never infers `was_torqued` from feedback-reported enable state (R10 decision-owned torque).
    /// Returns `(was_torqued, prev_gripper_cmd, mit_qualified)`.
    pub(in crate::supervisor) fn enable_prepare_rt(
        &mut self,
        authority: Option<EnableAuthority>,
    ) -> Result<(bool, Option<f32>, bool), &'static str> {
        self.enable_prepare_rt_precheck(authority)?;
        let mit_qualified = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
        let was_torqued = self.is_torqued;
        let prev_gripper_cmd = if was_torqued {
            self.config
                .gripper_index()
                .and_then(|i| self.commanded_positions.get(i).copied())
        } else {
            None
        };
        if let Some(d) = self.driver.take() {
            if self.estop_pending() {
                self.driver = Some(d);
                return Err("ESTOP_ACTIVE");
            }
            let Some(driver) = d.read() else {
                self.owner_driver_snapshot.invalidate();
                self.driver = Some(d);
                return Err("FEEDBACK_UNAVAILABLE");
            };
            if self.estop_pending() {
                drop(driver);
                self.driver = Some(d);
                return Err("ESTOP_ACTIVE");
            }
            if driver.tx().check_watchdog_tx_budget_for_enable().is_err() {
                drop(driver);
                self.driver = Some(d);
                return Err("WATCHDOG_BUDGET_INSUFFICIENT");
            }
            self.refresh_owner_driver_snapshot_from_ref(driver.as_ref());
            drop(driver);
            self.driver = Some(d);
        }
        Ok((was_torqued, prev_gripper_cmd, mit_qualified))
    }

    /// Starts the single budgeted enable reducer (`Enable -> Validate -> Armed | FailedHold`).
    pub(in crate::supervisor) fn start_enable_reducer(
        &mut self,
        origin: EnableOrigin,
        was_torqued: bool,
        prev_gripper_cmd: Option<f32>,
        authority: Option<EnableAuthority>,
        budgeted: bool,
        mit_qualified: bool,
    ) -> EnableStepOutcome {
        self.start_enable_reducer_inner(
            origin,
            was_torqued,
            prev_gripper_cmd,
            authority,
            budgeted,
            false,
            mit_qualified,
        )
    }

    /// Computes the owner-clock authority ceiling (`min(lease_expiry, request_deadline)`) so
    /// Damiao/Feetech enforce authority immediately before every `CMD_ENABLE` / torque-on frame.
    pub(in crate::supervisor) fn compute_enable_owner_deadline(
        &self,
        owner_now: Instant,
        auth: Option<EnableAuthority>,
        req_deadline: Option<Instant>,
    ) -> Option<Instant> {
        let lease_exp = auth.and_then(|a| {
            if self.lease_enabled && a.lease_generation != 0 {
                Some(
                    self.active_lease
                        .as_ref()
                        .filter(|l| l.generation == a.lease_generation)
                        .map_or(owner_now, |l| l.acquired_at + self.lease_timeout),
                )
            } else {
                None
            }
        });
        match (lease_exp, req_deadline) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        }
    }

    pub(in crate::supervisor) fn start_enable_reducer_inner(
        &mut self,
        origin: EnableOrigin,
        was_torqued: bool,
        prev_gripper_cmd: Option<f32>,
        authority: Option<EnableAuthority>,
        budgeted: bool,
        retry_unverified_ranges: bool,
        mit_qualified: bool,
    ) -> EnableStepOutcome {
        use crate::driver::{TorqueOutcome, TorqueProgress as Drv};
        let reducer = EnableReducer {
            origin,
            phase: EnableReducerPhase::Enable,
            was_torqued,
            prev_gripper_cmd,
            authority,
            budgeted,
            mit_qualified,
        };
        self.torque_intent = TorqueIntent::On;
        self.enable_lifecycle = EnableLifecycle::Enabling;
        #[cfg(not(target_arch = "wasm32"))]
        {
            let live_op_id = self.torque_shadow.alloc_live_op_id();
            self.shadow_enable_op_id = live_op_id;
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::EnableStart {
                    live_op_id,
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                });
        }
        if self.driver.is_none() {
            return self.step_enable_validate(
                EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                None,
                budgeted,
            );
        }
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let owner_now = Instant::now();
        let req_dl = match origin {
            EnableOrigin::LegacyWs { deadline, .. } => Some(deadline),
            _ => None,
        };
        let owner_dl = self.compute_enable_owner_deadline(owner_now, authority, req_dl);
        let expected_lease_gen = authority
            .and_then(|a| (a.lease_generation != 0).then_some(a.lease_generation));
        let measured_positions = &self.measured_positions;
        let pending_bus_loss_notify = &mut self.pending_driver_bus_loss_notify;
        let res = {
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let d = self.driver.as_mut().expect("checked is_some");
            let run_start = |drv: &mut dyn crate::driver::ArmDriver| {
                if *pending_bus_loss_notify {
                    drv.bus_recovery_mut().notify_bus_loss();
                    *pending_bus_loss_notify = false;
                }
                let enable_fb = drv.capabilities().enable_feedback;
                let drv_now = drv.clock_now();
                let tx_dl = owner_dl.map(|dl| drv_now + dl.saturating_duration_since(owner_now));
                drv.tx_mut().set_enable_authority(tx_dl, expected_lease_gen);
                let started = if budgeted && retry_unverified_ranges {
                    drv.begin_torque_budgeted_with_range_retry(true, 0)
                } else if budgeted {
                    drv.begin_torque_budgeted(true, 0)
                } else {
                    drv.begin_torque(true)
                };
                let snap = OwnerDriverSnapshot::capture_from_driver(
                    &*drv,
                    dof,
                    feedback_numeric_valid,
                    measured_positions,
                    &mut ages_buf,
                    &mut bounds_buf,
                );
                #[cfg(not(target_arch = "wasm32"))]
                shadow_rt.drain_from_driver(drv);
                (started, enable_fb, drv_now, snap)
            };
            if budgeted {
                d.try_with_mut(run_start).ok()
            } else {
                d.with_mut(run_start)
            }
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
        let Some((started, enable_fb, drv_now, snap)) = res else {
            self.owner_driver_snapshot.invalidate();
            let message = self.enter_failed_hold(was_torqued, |buf| {
                buf.push_str("driver mutex unavailable during enable");
            });
            return EnableStepOutcome::Failed {
                rt_code: "FEEDBACK_UNAVAILABLE",
                message,
                feedback_unavailable: true,
                estop_interrupted: false,
            };
        };
        self.owner_driver_snapshot = snap;
        match started {
            Ok(Drv::Pending { wake }) => {
                self.torque_transition = Some(TorqueTransition::Enable(reducer));
                EnableStepOutcome::Pending {
                    wake: if budgeted { drv_now } else { wake },
                }
            }
            Ok(Drv::Done(TorqueOutcome::Confirmed)) => self.step_enable_validate(
                EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                None,
                budgeted,
            ),
            Ok(Drv::Done(TorqueOutcome::Unconfirmed)) if !enable_fb => self.step_enable_validate(
                EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                None,
                budgeted,
            ),
            Ok(Drv::Done(TorqueOutcome::Unconfirmed)) => {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable not confirmed");
                });
                EnableStepOutcome::Failed {
                    rt_code: "FEEDBACK_UNCONFIRMED",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: false,
                }
            }
            Err(e) => {
                let rt_code = enable_driver_err_code(&e);
                let estop_interrupted = crate::driver::is_estop_inhibited(&e);
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    e.write_bounded(buf);
                });
                EnableStepOutcome::Failed {
                    rt_code,
                    message,
                    feedback_unavailable: false,
                    estop_interrupted,
                }
            }
        }
    }

    /// Drains an in-flight enable cancellation (`EnableReducerPhase::Cancelling`) with zero enable
    /// TX (zero-lock on `DriverSlot::Exclusive`, nonblocking `try_with_mut` on `DriverSlot::Legacy`)
    /// until `drv.abort_enable()` clears the driver's `torque_hs` so stopped-hold writes can never
    /// stay blocked by `TorqueTransitionPending`.
    pub(in crate::supervisor) fn step_enable_cancel(
        &mut self,
        reducer: EnableReducer,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
    ) -> EnableStepOutcome {
        let mut drv_any_enabled = false;
        let mut contended = false;
        if let Some(mut d) = self.driver.take() {
            match d.try_with_mut(|drv| {
                let enable_dispatched = drv.enable_frames_dispatched();
                drv.abort_enable();
                self.refresh_owner_driver_snapshot_from_ref(&*drv);
                let any = enable_dispatched
                    || drv.enable_state() != EnableState::Disabled
                    || drv.has_any_joint_enabled()
                    || drv.has_known_partial_enable();
                if reducer.was_torqued || any {
                    let (hold_mask, _) = self.capture_measured_hold_from_owner();
                    drv.tx_mut().drop_pending_holds_outside_mask(hold_mask);
                }
                any
            }) {
                Ok(any) => {
                    drv_any_enabled = any;
                }
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                    contended = true;
                }
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                    self.owner_driver_snapshot.invalidate();
                    drv_any_enabled = true;
                    let _ = self.capture_measured_hold_from_owner();
                }
            }
            self.driver = Some(d);
        }
        if contended {
            self.torque_transition = Some(TorqueTransition::Enable(reducer));
            let wake = budget.map_or_else(Instant::now, |b| b.now);
            return EnableStepOutcome::Pending { wake };
        }
        let restore_intent = self.detached_cancel_restore_intent.take();
        if reducer.was_torqued || drv_any_enabled {
            let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
            for j in 0..dof {
                if let Some(pos) = self.measured_hold.captured_positions[j] {
                    if j < self.measured_positions.len() {
                        self.measured_positions[j] = pos;
                    }
                    if j < self.commanded_positions.len() {
                        self.commanded_positions[j] = pos;
                    }
                    if j < self.target_positions.len() {
                        self.target_positions[j] = pos;
                    }
                }
            }
            self.commanded_velocities.fill(0.0);
            self.commanded_accelerations.fill(0.0);
            self.enable_lifecycle = EnableLifecycle::FailedHold;
            self.is_torqued = true;
            self.torque_intent = TorqueIntent::On;
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
        } else if let Some(prev_intent) = restore_intent {
            // Reconcile provisional FailedHold from contended clean Stop once driver confirms
            // zero enable frames were dispatched and zero joints are energized (R5 M5).
            if !self.safety.state().is_stopped() && !self.estop_pending() {
                self.enable_lifecycle = EnableLifecycle::Disarmed;
                self.is_torqued = false;
                self.torque_intent = prev_intent;
                self.pending_rearm = false;
                self.measured_hold.clear();
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow.enqueue(
                    crate::torque::shadow::ShadowInputKind::EnableCancelledPreTx {
                        live_op_id: self.shadow_enable_op_id,
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    },
                );
            }
        }
        EnableStepOutcome::Failed {
            rt_code: "CANCELLED",
            message: crate::safety::BoundedText::new("enable cancellation drained"),
            feedback_unavailable: false,
            estop_interrupted: false,
        }
    }

    /// True when a protective (thermal or process-exit) hardware disable is currently pending.
    #[inline]
    pub(crate) fn is_protective_disable_active(&self) -> bool {
        self.disable_pending
            && matches!(
                self.active_disable_kind,
                Some(crate::driver::DisableKind::DisableProtective { .. })
            )
    }

    pub(crate) fn sync_detached_torque_flag(&self) {
        self.detached_torque_pending_atomic
            .store(self.has_detached_torque_transition(), Ordering::Release);
    }

    /// Retains `EnableOrigin::DetachedCancel` (`Cancelling` phase) when a driver mutex was
    /// contended during an enable abort, provided no newer transition or protective disable owns
    /// the slot. A later release drains `drv.abort_enable()` with zero enable TX.
    pub(crate) fn retain_detached_enable_cancel_if_clear(&mut self, was_torqued: bool) {
        if self.torque_transition.is_none() && !self.is_protective_disable_active() {
            self.torque_transition = Some(TorqueTransition::Enable(EnableReducer {
                origin: EnableOrigin::DetachedCancel,
                phase: EnableReducerPhase::Cancelling,
                was_torqued,
                prev_gripper_cmd: None,
                authority: None,
                budgeted: true,
                mit_qualified: false,
            }));
        }
        self.sync_detached_torque_flag();
    }

    /// Single non-blocking enable-abort helper shared across Stop, ParkRearm abort, timeout, and
    /// E-stop paths. Returns `(drv_any_enabled, contended)`. When `retain_on_contention` is true
    /// and `driver.try_lock()` returns `WouldBlock`, installs `DetachedCancel` so a later release
    /// drains `drv.abort_enable()` with zero enable TX.
    pub(crate) fn abort_enable_driver_nonblocking(
        &mut self,
        was_torqued: bool,
        retain_on_contention: bool,
    ) -> (bool, bool) {
        let mut drv_any_enabled = false;
        let mut contended = false;
        if let Some(mut d) = self.driver.take() {
            match d.try_with_mut(|drv| {
                let enable_dispatched = drv.enable_frames_dispatched();
                drv.abort_enable();
                self.refresh_owner_driver_snapshot_from_ref(&*drv);
                let any = enable_dispatched
                    || drv.enable_state() != EnableState::Disabled
                    || drv.has_any_joint_enabled()
                    || drv.has_known_partial_enable();
                if was_torqued || any {
                    let (hold_mask, _) = self.capture_measured_hold_from_owner();
                    drv.tx_mut().drop_pending_holds_outside_mask(hold_mask);
                }
                any
            }) {
                Ok(any) => {
                    drv_any_enabled = any;
                }
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                    contended = true;
                    drv_any_enabled = true;
                    self.owner_driver_snapshot.invalidate();
                    let _ = self.capture_measured_hold_from_owner();
                }
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                    drv_any_enabled = true;
                    self.owner_driver_snapshot.invalidate();
                    let _ = self.capture_measured_hold_from_owner();
                }
            }
            self.driver = Some(d);
        }
        if contended && retain_on_contention {
            self.retain_detached_enable_cancel_if_clear(was_torqued);
        }
        (drv_any_enabled, contended)
    }

    /// Cancels an in-flight `EnableOrigin::ParkRearm` cleanly on client Stop without latching
    /// `EnableFailed`, preserving holding torque on any energized joints (non-blocking `try_lock`,
    /// retaining `DetachedCancel` on contention).
    pub(in crate::supervisor) fn cancel_park_rearm_on_stop(&mut self, action_id: u64, was_torqued: bool) {
        let (drv_any_enabled, _) = self.abort_enable_driver_nonblocking(was_torqued, true);
        self.is_torqued = was_torqued || drv_any_enabled;
        self.pending_rearm = true;
        if self.park_continuation.is_some_and(|c| c.action_id == action_id) {
            self.park_continuation = None;
            self.notify_park_continuation();
        }
        if self
            .last_action
            .as_ref()
            .is_some_and(|a| a.id == action_id && a.is_running())
        {
            self.finish_action(action_id, "cancelled", Some("STOPPED"), None, None, None, None);
        }
    }
}
