//! Enable reducer polling and post-enable validation.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

impl SupervisorInner {
    /// Advances the single budgeted enable reducer by one poll step (`Enable -> Validate -> Armed | FailedHold`).
    /// Re-verifies live [`EnableAuthority`] BEFORE any subsequent enable retry TX so a displaced
    /// lease, disconnect, E-stop, fault, or epoch change transitions immediately to `FailedHold`
    /// without sending rollback disable frames.
    pub(in crate::supervisor) fn poll_enable_reducer(
        &mut self,
        reducer: EnableReducer,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
    ) -> (EnableStepOutcome, Option<(u16, bool)>) {
        let entry_wall = Instant::now();
        let base = budget.map_or(entry_wall, |b| b.now);
        let mut live_now = || base + Instant::now().saturating_duration_since(entry_wall);
        self.poll_enable_reducer_with_clock(reducer, budget, &mut live_now)
    }

    pub(in crate::supervisor) fn poll_enable_reducer_with_clock(
        &mut self,
        reducer: EnableReducer,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> (EnableStepOutcome, Option<(u16, bool)>) {
        use crate::driver::{TorqueOutcome, TorqueProgress as Drv};
        if matches!(reducer.phase, EnableReducerPhase::Cancelling) {
            return (self.step_enable_cancel(reducer, budget), None);
        }
        if let Some(auth) = reducer.authority {
            let check_now = match reducer.origin {
                EnableOrigin::LegacyWs { .. } => Instant::now(),
                _ => budget.map_or_else(Instant::now, |b| b.now),
            };
            if let Some(code) = self.enable_authority_revoked(auth, check_now) {
                let estop = code == "ESTOP_ACTIVE";
                let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                    buf.push_str("enable authority revoked: ");
                    buf.push_str(code);
                });
                return (
                    EnableStepOutcome::Failed {
                        rt_code: code,
                        message,
                        feedback_unavailable: false,
                        estop_interrupted: estop,
                    },
                    None,
                );
            }
        }
        if let EnableOrigin::LegacyWs {
            token,
            slot,
            stop_gen,
            prev_torque_intent,
            ..
        } = reducer.origin
        {
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
                let estop = self.cancel_legacy_enable_on_stop(
                    token,
                    slot,
                    reducer.was_torqued,
                    prev_torque_intent,
                );
                return (
                    EnableStepOutcome::Failed {
                        rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                        message: crate::safety::BoundedText::new(if estop {
                            "interrupted by E-stop"
                        } else {
                            "enable cancelled by stop"
                        }),
                        feedback_unavailable: false,
                        estop_interrupted: estop,
                    },
                    None,
                );
            }
        }
        if let EnableOrigin::ParkRearm { action_id, stop_gen } = reducer.origin {
            if self.urgent_stop_unapplied()
                || self.current_stop_gen() != stop_gen
                || !self
                    .park_continuation
                    .is_some_and(|c| c.action_id == action_id && c.stop_gen == stop_gen)
            {
                self.cancel_park_rearm_on_stop(action_id, reducer.was_torqued);
                return (
                    EnableStepOutcome::Failed {
                        rt_code: "STOPPED",
                        message: crate::safety::BoundedText::new("park rearm cancelled by stop"),
                        feedback_unavailable: false,
                        estop_interrupted: false,
                    },
                    None,
                );
            }
        }
        if self.estop_pending() || self.safety.state().is_stopped() {
            let estop = self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop);
            let rt_code = if estop { "ESTOP_ACTIVE" } else { "FAULT_ACTIVE" };
            let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                buf.push_str(if estop {
                    "interrupted by E-stop"
                } else {
                    "interrupted by active fault"
                });
            });
            return (
                EnableStepOutcome::Failed {
                    rt_code,
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                },
                None,
            );
        }
        match reducer.phase {
            EnableReducerPhase::Cancelling => (self.step_enable_cancel(reducer, budget), None),
            EnableReducerPhase::Validate => (
                self.step_enable_validate(reducer, budget, budget.is_some()),
                None,
            ),
            EnableReducerPhase::Enable => {
                if self.driver.is_none() {
                    let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                        buf.push_str("driver missing during enable poll");
                    });
                    return (
                        EnableStepOutcome::Failed {
                            rt_code: "INTERNAL",
                            message,
                            feedback_unavailable: false,
                            estop_interrupted: false,
                        },
                        None,
                    );
                }
                let mut scratch_states = std::mem::take(&mut self.scratch.states);
                let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
                let mut scratch_bounds = std::mem::take(&mut self.scratch.fb.bounds);
                let dof = self.config.dof;
                let feedback_numeric_valid = self.feedback_numeric_valid;
                let control_release_seq = self.control_release_seq;
                let owner_now = match reducer.origin {
                    EnableOrigin::LegacyWs { .. } => Instant::now(),
                    _ => budget.map_or_else(Instant::now, |b| b.now),
                };
                let req_dl = match reducer.origin {
                    EnableOrigin::LegacyWs { deadline, .. } => Some(deadline),
                    _ => None,
                };
                let owner_dl = self.compute_enable_owner_deadline(owner_now, reducer.authority, req_dl);
                let expected_lease_gen = reducer
                    .authority
                    .and_then(|a| (a.lease_generation != 0).then_some(a.lease_generation));
                let measured_positions = &self.measured_positions;
                let conservative_cap = self.pending_conservative_tx_cap;
                let mut cons_rx_delta = 0u16;
                let mut cons_tx_delta = 0u16;
                let lock_res = {
                    #[cfg(not(target_arch = "wasm32"))]
                    let shadow_rt = &mut self.torque_shadow;
                    let d = self.driver.as_mut().expect("checked is_some");
                    let run_poll = |drv: &mut dyn crate::driver::ArmDriver| {
                        if control_release_seq > 0 {
                            drv.begin_control_release(control_release_seq);
                        }
                        let caps = drv.capabilities();
                        let enable_fb = caps.enable_feedback;
                        let temp_fb = caps.temperature_feedback;
                        let (drv_now, r, rx, expired_defer) = if let Some(b) = budget {
                            match crate::supervisor::translate_driver_cutoff_conservative(
                                drv,
                                b.deadline,
                                now_fn,
                            ) {
                                Ok((drv_now, live_owner_now, drv_deadline)) => {
                                    let tx_dl = owner_dl.map(|dl| {
                                        drv_now + dl.saturating_duration_since(live_owner_now)
                                    });
                                    drv.tx_mut().set_enable_authority(tx_dl, expected_lease_gen);
                                    let before_cons = if conservative_cap {
                                        let bc = drv.tx().release_budget_consumed();
                                        drv.tx_mut().sync_release_budget_limits(0, 0);
                                        Some(bc)
                                    } else {
                                        None
                                    };
                                    let r = if reducer.budgeted {
                                        drv.poll_torque_budgeted(drv_deadline, b.max_rx_frames)
                                    } else {
                                        drv.poll_torque(drv_deadline)
                                    };
                                    if let Some(bc) = before_cons {
                                        let after = drv.tx().release_budget_consumed();
                                        cons_rx_delta = cons_rx_delta.saturating_add(after.0.saturating_sub(bc.0));
                                        cons_tx_delta = cons_tx_delta.saturating_add(after.1.saturating_sub(bc.1));
                                    }
                                    let rx = drv.take_last_poll_rx();
                                    (drv_now, r, rx, None)
                                }
                                Err((drv_now, live_owner_now)) => (
                                    drv_now,
                                    Ok(Drv::Pending { wake: drv_now }),
                                    Some((0, true)),
                                    Some(live_owner_now),
                                ),
                            }
                        } else {
                            let drv_now = drv.clock_now();
                            let tx_dl = owner_dl.map(|dl| drv_now + dl.saturating_duration_since(owner_now));
                            drv.tx_mut().set_enable_authority(tx_dl, expected_lease_gen);
                            let r = drv.poll_torque(drv_now + crate::driver::DAMIAO_RX_PASS);
                            (drv_now, r, None, None)
                        };
                        let mut cached_temps_read = false;
                        if expired_defer.is_none()
                            && temp_fb
                            && drv.read_cached_joint_states_into(&mut scratch_states).is_some()
                        {
                            drv.feedback_ages_into(&mut scratch_ages);
                            cached_temps_read = true;
                        }
                        let snap = OwnerDriverSnapshot::capture_from_driver(
                            &*drv,
                            dof,
                            feedback_numeric_valid,
                            measured_positions,
                            &mut scratch_ages,
                            &mut scratch_bounds,
                        );
                        #[cfg(not(target_arch = "wasm32"))]
                        shadow_rt.drain_from_driver(drv);
                        (r, rx, enable_fb, temp_fb, drv_now, cached_temps_read, snap, expired_defer)
                    };
                    if budget.is_some() || reducer.budgeted {
                        match d.try_with_mut(run_poll) {
                            Ok(v) => Ok(v),
                            Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                            Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
                        }
                    } else {
                        d.with_mut(run_poll).ok_or(false)
                    }
                };
                if conservative_cap {
                    self.conservative_step_rx_delta =
                        self.conservative_step_rx_delta.saturating_add(cons_rx_delta);
                    self.conservative_step_tx_delta =
                        self.conservative_step_tx_delta.saturating_add(cons_tx_delta);
                }
                let (poll_res, rx, enable_fb, temp_fb, drv_now, cached_temps_read, snap, expired_defer) = match lock_res {
                    Ok(v) => v,
                    Err(true) => {
                        self.scratch.states = scratch_states;
                        self.scratch.ages = scratch_ages;
                        self.scratch.fb.bounds = scratch_bounds;
                        self.torque_transition = Some(TorqueTransition::Enable(reducer));
                        let wake = budget.map_or_else(Instant::now, |b| b.now);
                        return (EnableStepOutcome::Pending { wake }, None);
                    }
                    Err(false) => {
                        self.scratch.states = scratch_states;
                        self.scratch.ages = scratch_ages;
                        self.scratch.fb.bounds = scratch_bounds;
                        self.owner_driver_snapshot.invalidate();
                        let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                            buf.push_str("driver mutex poisoned during enable poll");
                        });
                        return (
                            EnableStepOutcome::Failed {
                                rt_code: "FEEDBACK_UNAVAILABLE",
                                message,
                                feedback_unavailable: true,
                                estop_interrupted: false,
                            },
                            None,
                        );
                    }
                };
                self.owner_driver_snapshot = snap;
                if let Some(wake) = expired_defer {
                    self.scratch.states = scratch_states;
                    self.scratch.ages = scratch_ages;
                    self.scratch.fb.bounds = scratch_bounds;
                    self.torque_transition = Some(TorqueTransition::Enable(reducer));
                    return (EnableStepOutcome::Pending { wake }, rx);
                }
                if cached_temps_read {
                    let n = self.config.dof.min(self.temperatures.len()).min(scratch_states.len());
                    for i in 0..n {
                        if scratch_ages.get(i).copied().flatten().is_some()
                            && scratch_states[i].temperature.is_finite()
                        {
                            self.temperatures[i] = scratch_states[i].temperature;
                        }
                    }
                }
                self.scratch.states = scratch_states;
                self.scratch.ages = scratch_ages;
                self.scratch.fb.bounds = scratch_bounds;
                if let Some(failed) = self.check_enable_thermal_trip(reducer.was_torqued, temp_fb) {
                    return (failed, rx);
                }
                match poll_res {
                    Ok(Drv::Pending { wake }) => {
                        self.torque_transition = Some(TorqueTransition::Enable(reducer));
                        let mapped_wake = match budget {
                            Some(b) => b.now + wake.saturating_duration_since(drv_now),
                            None => wake,
                        };
                        (EnableStepOutcome::Pending { wake: mapped_wake }, rx)
                    }
                    Ok(Drv::Done(TorqueOutcome::Confirmed)) => (
                        self.step_enable_validate(
                            EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                            budget,
                            budget.is_some(),
                        ),
                        rx,
                    ),
                    Ok(Drv::Done(TorqueOutcome::Unconfirmed)) if !enable_fb => (
                        self.step_enable_validate(
                            EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                            budget,
                            budget.is_some(),
                        ),
                        rx,
                    ),
                    Ok(Drv::Done(TorqueOutcome::Unconfirmed)) => {
                        let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                            buf.push_str("enable not confirmed");
                        });
                        (
                            EnableStepOutcome::Failed {
                                rt_code: "FEEDBACK_UNCONFIRMED",
                                message,
                                feedback_unavailable: false,
                                estop_interrupted: false,
                            },
                            rx,
                        )
                    }
                    Err(e) => {
                        // Major 1: If the driver stopped an enable poll due to a stop-stamp change
                        // (`EstopInhibited`) on `EnableOrigin::LegacyWs`, resolve live E-stop vs
                        // ordinary client Stop so an ordinary Stop aborts cleanly without latching
                        // `EnableFailed` or falsely reporting `ESTOP_ACTIVE`.
                        if crate::driver::is_estop_inhibited(&e) {
                            if let EnableOrigin::LegacyWs {
                                token,
                                slot,
                                stop_gen,
                                prev_torque_intent,
                                ..
                            } = reducer.origin
                            {
                                let is_estop = self.estop_pending()
                                    || self.e_stop_latched.load(Ordering::SeqCst)
                                    || matches!(self.safety.state(), SafetyState::EStop);
                                if !is_estop
                                    && (self.urgent_stop_unapplied()
                                        || self.current_stop_gen() != stop_gen)
                                {
                                    let estop = self.cancel_legacy_enable_on_stop(
                                        token,
                                        slot,
                                        reducer.was_torqued,
                                        prev_torque_intent,
                                    );
                                    return (
                                        EnableStepOutcome::Failed {
                                            rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                                            message: crate::safety::BoundedText::new(if estop {
                                                "interrupted by E-stop"
                                            } else {
                                                "enable cancelled by stop"
                                            }),
                                            feedback_unavailable: false,
                                            estop_interrupted: estop,
                                        },
                                        rx,
                                    );
                                }
                            }
                        }
                        let mut rt_code = enable_driver_err_code(&e);
                        if matches!(e, crate::driver::ArmDriverError::EnableAuthorityExpired) {
                            if let EnableOrigin::LegacyWs { deadline, .. } = reducer.origin {
                                let now = Instant::now();
                                let auth_revoked = reducer
                                    .authority
                                    .and_then(|a| self.enable_authority_revoked(a, now));
                                if let Some(code) = auth_revoked {
                                    rt_code = code;
                                } else if now >= deadline {
                                    rt_code = "DEADLINE_EXPIRED";
                                }
                            }
                        }
                        let live_estop = self.estop_pending()
                            || self.e_stop_latched.load(Ordering::SeqCst)
                            || matches!(self.safety.state(), SafetyState::EStop);
                        let estop_interrupted =
                            crate::driver::is_estop_inhibited(&e) || live_estop;
                        if live_estop {
                            rt_code = "ESTOP_ACTIVE";
                        }
                        let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                            e.write_bounded(buf);
                        });
                        (
                            EnableStepOutcome::Failed {
                                rt_code,
                                message,
                                feedback_unavailable: false,
                                estop_interrupted,
                            },
                            rx,
                        )
                    }
                }
            }
        }
    }

    /// Post-enable validation phase of the single enable reducer (`Validate -> Armed | FailedHold`).
    /// Any validation failure or interruption routes through [`Self::enter_failed_hold`] (zero
    /// automatic rollback at any pose).
    pub(in crate::supervisor) fn step_enable_validate(
        &mut self,
        reducer: EnableReducer,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
        use_cached: bool,
    ) -> EnableStepOutcome {
        let was_torqued = reducer.was_torqued;
        let prev_gripper_cmd = reducer.prev_gripper_cmd;
        let check_now = match reducer.origin {
            EnableOrigin::LegacyWs { .. } => Instant::now(),
            _ => budget.map_or_else(Instant::now, |b| b.now),
        };
        if let Some(auth) = reducer.authority {
            if let Some(code) = self.enable_authority_revoked(auth, check_now) {
                let estop = code == "ESTOP_ACTIVE";
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable authority revoked before validation: ");
                    buf.push_str(code);
                });
                return EnableStepOutcome::Failed {
                    rt_code: code,
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if let EnableOrigin::LegacyWs {
            token,
            slot,
            stop_gen,
            prev_torque_intent,
            deadline,
        } = reducer.origin
        {
            if self.estop_pending()
                || self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop)
            {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("interrupted by E-stop before validation");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "ESTOP_ACTIVE",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: true,
                };
            }
            if Instant::now() >= deadline {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable request deadline expired before validation");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "DEADLINE_EXPIRED",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: false,
                };
            }
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
                let estop = self.cancel_legacy_enable_on_stop(
                    token,
                    slot,
                    was_torqued,
                    prev_torque_intent,
                );
                return EnableStepOutcome::Failed {
                    rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                    message: crate::safety::BoundedText::new(if estop {
                        "interrupted by E-stop before arming"
                    } else {
                        "enable cancelled by stop before arming"
                    }),
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if self.estop_pending() || self.safety.state().is_stopped() {
            let estop = self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop);
            let rt_code = if estop { "ESTOP_ACTIVE" } else { "FAULT_ACTIVE" };
            let message = self.enter_failed_hold(was_torqued, |buf| {
                buf.push_str(if estop {
                    "interrupted by E-stop before validation"
                } else {
                    "interrupted by active fault before validation"
                });
            });
            return EnableStepOutcome::Failed {
                rt_code,
                message,
                feedback_unavailable: false,
                estop_interrupted: estop,
            };
        }

        if let Some(mut d) = self.driver.take() {
            let mut scratch_states = std::mem::take(&mut self.scratch.states);
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_loads = std::mem::take(&mut self.scratch.loads);
            let run_validate = |driver: &mut dyn crate::driver::ArmDriver| {
                let temp_fb = driver.capabilities().temperature_feedback;
                let read_res: Result<(), ()> = if use_cached {
                    if driver.read_cached_joint_states_into(&mut scratch_states).is_some() {
                        Ok(())
                    } else if let Some(b) = budget {
                        let entry_wall = Instant::now();
                        let mut live_now =
                            || b.now + Instant::now().saturating_duration_since(entry_wall);
                        match crate::supervisor::translate_driver_cutoff_conservative(
                            driver,
                            b.deadline,
                            &mut live_now,
                        ) {
                            Ok((_, _, drv_deadline)) => driver
                                .read_joint_states_budgeted(
                                    &mut scratch_states,
                                    drv_deadline,
                                    b.max_rx_frames,
                                )
                                .map(|_| ())
                                .map_err(|_| ()),
                            Err(_) => Err(()),
                        }
                    } else {
                        Err(())
                    }
                } else {
                    driver.read_joint_states_into(&mut scratch_states).map_err(|_| ())
                };
                if read_res.is_err() {
                    (Err("post-enable feedback unavailable"), temp_fb)
                } else {
                    let has_loads = driver.joint_loads_into(&mut scratch_loads);
                    let loads_slice = has_loads.then_some(scratch_loads.as_slice());
                    if !driver.is_feedback_valid()
                        || !Self::states_valid(&scratch_states, driver.capabilities(), self.config.dof, loads_slice)
                        || (driver.capabilities().enable_feedback && driver.enable_state() != EnableState::Enabled)
                    {
                        // Preserve any finite received temperatures for thermal enforcement
                        // even when another joint failed post-enable validation.
                        if temp_fb {
                            driver.feedback_ages_into(&mut scratch_ages);
                            let n = self.config.dof.min(self.temperatures.len()).min(scratch_states.len());
                            for i in 0..n {
                                if scratch_ages.get(i).copied().flatten().is_some()
                                    && scratch_states[i].temperature.is_finite()
                                {
                                    self.temperatures[i] = scratch_states[i].temperature;
                                }
                            }
                        }
                        #[cfg(not(target_arch = "wasm32"))]
                        self.torque_shadow.drain_from_driver(driver);
                        (Err("post-enable feedback invalid or enable unconfirmed"), temp_fb)
                    } else {
                        driver.feedback_ages_into(&mut scratch_ages);
                        let oldest_age = if scratch_ages.iter().all(Option::is_some) {
                            scratch_ages.iter().flatten().copied().max()
                        } else {
                            None
                        };
                        self.driver_caps = driver.capabilities();
                        self.feedback_numeric_valid = true;
                        self.last_can_feedback_instant = oldest_age.and_then(|age| Instant::now().checked_sub(age));
                        self.last_poll_enable_state = driver.enable_state();
                        self.last_poll_partial_enable = driver.has_known_partial_enable();
                        self.last_poll_feedback_valid = true;
                        self.last_poll_oldest_age = oldest_age;
                        self.core.poll_feedback(&mut self.joint_feedback_queue_bounds, self.config.dof, &*driver, &mut self.scratch.fb);
                        Self::apply_feedback(
                            &mut self.measured_positions,
                            &mut self.measured_velocities,
                            &mut self.torques,
                            &mut self.temperatures,
                            &mut self.loads,
                            self.config.dof,
                            &scratch_states,
                            loads_slice,
                        );
                        self.owner_driver_snapshot = OwnerDriverSnapshot::capture_from_driver(
                            &*driver,
                            self.config.dof,
                            self.feedback_numeric_valid,
                            &self.measured_positions,
                            &mut scratch_ages,
                            &mut self.scratch.fb.bounds,
                        );
                        #[cfg(not(target_arch = "wasm32"))]
                        self.torque_shadow.drain_from_driver(driver);
                        (Ok(()), temp_fb)
                    }
                }
            };
            let lock_out = if budget.is_some() || reducer.budgeted {
                match d.try_with_mut(run_validate) {
                    Ok(v) => Ok(v),
                    Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                    Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
                }
            } else {
                d.with_mut(run_validate).ok_or(false)
            };
            self.driver = Some(d);
            let (validate_res, temp_fb) = match lock_out {
                Ok(v) => v,
                Err(true) => {
                    self.scratch.states = scratch_states;
                    self.scratch.ages = scratch_ages;
                    self.scratch.loads = scratch_loads;
                    self.torque_transition = Some(TorqueTransition::Enable(reducer));
                    return EnableStepOutcome::Pending { wake: check_now };
                }
                Err(false) => (Err("driver mutex poisoned during validation"), false),
            };
            self.scratch.states = scratch_states;
            self.scratch.ages = scratch_ages;
            self.scratch.loads = scratch_loads;
            if let Some(failed) = self.check_enable_thermal_trip(was_torqued, temp_fb) {
                return failed;
            }
            if let Err(cause) = validate_res {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str(cause);
                });
                return EnableStepOutcome::Failed {
                    rt_code: "FEEDBACK_UNAVAILABLE",
                    message,
                    feedback_unavailable: true,
                    estop_interrupted: false,
                };
            }
        } else if let Some(failed) = self.check_enable_thermal_trip(was_torqued, true) {
            return failed;
        }

        if let EnableOrigin::LegacyWs {
            token,
            slot,
            stop_gen,
            prev_torque_intent,
            deadline,
        } = reducer.origin
        {
            if self.estop_pending()
                || self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop)
            {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("interrupted by E-stop before arming");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "ESTOP_ACTIVE",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: true,
                };
            }
            if Instant::now() >= deadline {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable request deadline expired before arming");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "DEADLINE_EXPIRED",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: false,
                };
            }
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
                let estop = self.cancel_legacy_enable_on_stop(
                    token,
                    slot,
                    was_torqued,
                    prev_torque_intent,
                );
                return EnableStepOutcome::Failed {
                    rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                    message: crate::safety::BoundedText::new(if estop {
                        "interrupted by E-stop"
                    } else {
                        "enable cancelled by stop"
                    }),
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if let EnableOrigin::ParkRearm { action_id, stop_gen } = reducer.origin {
            let cont_lost = budget.is_some()
                && !self
                    .park_continuation
                    .is_some_and(|c| c.action_id == action_id && c.stop_gen == stop_gen);
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen || cont_lost {
                self.cancel_park_rearm_on_stop(action_id, was_torqued);
                return EnableStepOutcome::Failed {
                    rt_code: "STOPPED",
                    message: crate::safety::BoundedText::new("park rearm cancelled by stop"),
                    feedback_unavailable: false,
                    estop_interrupted: false,
                };
            }
        }

        // Initialize commanded_positions to verified measured_positions to prevent sudden jumps
        copy_into(&mut self.commanded_positions, &self.measured_positions);
        copy_into(&mut self.target_positions, &self.measured_positions);
        // Preserve gripper holding position when re-arming an already torqued arm so grasped objects are not released
        if let Some(g_cmd) = prev_gripper_cmd {
            if let Some(grip_idx) = self.config.gripper_index() {
                self.commanded_positions[grip_idx] = g_cmd;
                self.target_positions[grip_idx] = g_cmd;
            }
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
            let arm_indices = self.config.arm_joints();
            let has_vel_fb = self.driver.is_none() || self.driver_caps.velocity_feedback;
            // Positive evidence only: without velocity feedback a single sample cannot show
            // rest; the history stays Unknown until two samples do (plain output meanwhile).
            let arm_stationary = has_vel_fb
                && arm_indices.iter().all(|&j| {
                    self.measured_velocities
                        .get(j)
                        .is_some_and(|v| v.is_finite() && v.abs() <= crate::tracking::STATIONARY_VELOCITY_RAD_S)
                });
            let aligned_with_issued = !self
                .tracking
                .has_residual_lead(&self.commanded_positions, &arm_indices);
            if was_torqued && self.tracking.last_issued_positions.is_some() {
                if arm_stationary && aligned_with_issued {
                    self.tracking.seed(&self.commanded_positions);
                } else {
                    // Already torqued and moving or holding active lead: preserve real
                    // issued history so the next tick ramps smoothly instead of jumping.
                    self.tracking.ws_min_depths = None;
                }
            } else if arm_stationary {
                self.tracking.seed(&self.commanded_positions);
            } else {
                self.tracking.invalidate();
            }
        } else {
            self.tracking.invalidate();
        }
        if self.driver.is_none() { self.measured_velocities.fill(0.0); }
        self.replace_active_trajectory(None);
        self.pending_normal_unload = false;
        self.clear_unload_settle();
        self.watchdog_park = None;
        self.watchdog_park_pending = None;
        self.servo_target = None;
        self.servo_stream_active = false;
        self.clear_servo_ws_min_depths_rt();
        self.gripper_hold_vel_ceiling = None;

        if let Some(auth) = reducer.authority {
            let post_now = match reducer.origin {
                EnableOrigin::LegacyWs { .. } => Instant::now(),
                _ => budget.map_or_else(Instant::now, |b| b.now),
            };
            if let Some(code) = self.enable_authority_revoked(auth, post_now) {
                let estop = code == "ESTOP_ACTIVE";
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable authority revoked before arming: ");
                    buf.push_str(code);
                });
                return EnableStepOutcome::Failed {
                    rt_code: code,
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if let EnableOrigin::LegacyWs {
            token,
            slot,
            stop_gen,
            prev_torque_intent,
            deadline,
        } = reducer.origin
        {
            if self.estop_pending()
                || self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop)
            {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("interrupted by E-stop before validation");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "ESTOP_ACTIVE",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: true,
                };
            }
            if Instant::now() >= deadline {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable request deadline expired before validation");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "DEADLINE_EXPIRED",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: false,
                };
            }
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
                let estop = self.cancel_legacy_enable_on_stop(
                    token,
                    slot,
                    was_torqued,
                    prev_torque_intent,
                );
                return EnableStepOutcome::Failed {
                    rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                    message: crate::safety::BoundedText::new(if estop {
                        "interrupted by E-stop before arming"
                    } else {
                        "enable cancelled by stop before arming"
                    }),
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if self.estop_pending() || self.safety.state().is_stopped() {
            let estop = self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop);
            let rt_code = if estop { "ESTOP_ACTIVE" } else { "FAULT_ACTIVE" };
            let message = self.enter_failed_hold(was_torqued, |buf| {
                buf.push_str(if estop {
                    "interrupted by E-stop before arming"
                } else {
                    "interrupted by active fault before arming"
                });
            });
            return EnableStepOutcome::Failed {
                rt_code,
                message,
                feedback_unavailable: false,
                estop_interrupted: estop,
            };
        }
        let armed_now = budget.map_or_else(Instant::now, |b| b.now);
        if use_cached {
            match self.safety.set_torque_rt(true) {
                Ok(()) => {
                    self.enable_lifecycle = EnableLifecycle::Armed;
                    self.measured_hold.clear();
                    self.is_torqued = true;
                    self.pending_rearm = false;
                    self.disable_pending = false;
                    self.background_disable_retry = None;
                    self.mit.armed_ok = reducer.mit_qualified;
                    self.mit.handback_suppress_until = None;
                    self.mit.handover_error_rt = None;
                    if !self.rt_executor_owned {
                        self.mit.handover_error = None;
                    }
                    self.last_heartbeat = armed_now;
                    self.last_motion_frame = armed_now;
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        let dof = self.config.dof.min(crate::torque::MAX_CHANNELS) as u8;
                        self.torque_shadow
                            .enqueue(crate::torque::shadow::ShadowInputKind::HandshakeResult {
                                live_op_id: self.shadow_enable_op_id,
                                confirmed_mask: crate::torque::ChannelMask::from_count(dof),
                                outcome: crate::torque::HandshakeOutcome::Confirmed,
                                live_intent: self.shadow_live_intent(),
                                live_is_torqued: self.is_torqued,
                                live_stopped: self.safety.state().is_stopped(),
                            });
                    }
                    EnableStepOutcome::Armed(SafetyState::Armed)
                }
                Err(rt_code) => {
                    let estop_interrupted = rt_code == "ESTOP_ACTIVE";
                    let message = self.enter_failed_hold(was_torqued, |buf| {
                        buf.push_str(rt_code);
                    });
                    EnableStepOutcome::Failed {
                        rt_code,
                        message,
                        feedback_unavailable: false,
                        estop_interrupted,
                    }
                }
            }
        } else {
            match self.safety.set_torque(true) {
                Ok(state) => {
                    self.enable_lifecycle = EnableLifecycle::Armed;
                    self.measured_hold.clear();
                    self.is_torqued = true;
                    self.pending_rearm = false;
                    self.disable_pending = false;
                    self.background_disable_retry = None;
                    self.mit.armed_ok = reducer.mit_qualified;
                    self.mit.handback_suppress_until = None;
                    self.mit.handover_error_rt = None;
                    if !self.rt_executor_owned {
                        self.mit.handover_error = None;
                    }
                    self.last_heartbeat = armed_now;
                    self.last_motion_frame = armed_now;
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        let dof = self.config.dof.min(crate::torque::MAX_CHANNELS) as u8;
                        self.torque_shadow
                            .enqueue(crate::torque::shadow::ShadowInputKind::HandshakeResult {
                                live_op_id: self.shadow_enable_op_id,
                                confirmed_mask: crate::torque::ChannelMask::from_count(dof),
                                outcome: crate::torque::HandshakeOutcome::Confirmed,
                                live_intent: self.shadow_live_intent(),
                                live_is_torqued: self.is_torqued,
                                live_stopped: self.safety.state().is_stopped(),
                            });
                    }
                    EnableStepOutcome::Armed(state)
                }
                Err(e) => {
                    let rt_code = torque_error_code(&e);
                    let estop_interrupted = matches!(e, SafetyError::EStopActive);
                    let message = self.enter_failed_hold(was_torqued, |buf| {
                        buf.push_str(rt_code);
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
    }
}
