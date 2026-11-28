//! Process-exit/shutdown panic disable and post-panic torque shadow recording.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

impl SupervisorInner {
    pub(in crate::supervisor) fn mark_panic_disable_requested(&mut self) {
        self.estop_state.latch();
        self.active_disable_kind = Some(crate::driver::DisableKind::DisableProtective {
            reason: crate::driver::DisableProtectiveReason::ProcessExit,
        });
        self.disable_requested();
    }

    /// Explicit test-only ProcessExit authorization on `SupervisorInner` (DRV-F1 B7).
    #[doc(hidden)]
    pub fn authorize_process_exit_for_test(&mut self) {
        self.process_exit_authorized = true;
    }

    /// Bounded last-chance disable reserved for explicit `ProcessExit` paths owned by this
    /// `SupervisorInner`. Refuses protective cuts unless explicit owner-bound ProcessExit
    /// authorization is set (`process_exit_authorized` or `shutdown_committed`, DRV-F1 B7).
    #[doc(hidden)]
    pub fn attempt_panic_disable(&mut self, timeout: Duration) -> PanicDisableOutcome {
        if !self.process_exit_authorized && !self.shutdown_committed {
            return PanicDisableOutcome::Failed(
                "protective disable requires explicit ProcessExit authorization".into(),
            );
        }
        self.attempt_bounded_teardown_disable(timeout, true)
    }

    /// Bounded protective disable on repeated shutdown (drains driver observations without
    /// emitting a premature `ProcessExit` before `commit_shutdown`).
    pub(crate) fn attempt_shutdown_protective_disable(&mut self, timeout: Duration) -> PanicDisableOutcome {
        self.mark_panic_disable_requested();
        let origin = Instant::now();
        let (coop_deadline, hard_deadline) = panic_disable_deadlines(origin, timeout);
        let coop_budget = coop_deadline.saturating_duration_since(origin);
        let outcome = run_with_panic_deadline_observer_until(
            hard_deadline,
            || self.attempt_panic_disable_until(coop_deadline, hard_deadline, coop_budget),
            |_| true,
            move || {
                PanicDisableOutcome::TimedOut.log("Bounded panic disable", timeout);
            },
        )
        .unwrap_or(PanicDisableOutcome::TimedOut);
        #[cfg(not(target_arch = "wasm32"))]
        if self.torque_shadow.is_enabled() {
            let deferred_mit = self.torque_shadow.take_deferred_mit_off();
            if !deferred_mit.is_empty() {
                self.torque_shadow.enqueue(
                    crate::torque::shadow::ShadowInputKind::MitOwnershipChange {
                        channels: deferred_mit,
                        target_mode: crate::motor_mode::ControlMode::PositionVelocity,
                        device_wide: false,
                    },
                );
            }
            match &outcome {
                PanicDisableOutcome::Confirmed
                | PanicDisableOutcome::Unconfirmed
                | PanicDisableOutcome::NoDriver => {
                    if let Some(ref mut d) = self.driver {
                        let shadow_rt = &mut self.torque_shadow;
                        match d {
                            crate::supervisor_lock::DriverSlot::Exclusive(drv) => {
                                shadow_rt.drain_from_driver(drv.get_mut().as_mut());
                            }
                            crate::supervisor_lock::DriverSlot::Legacy(_)
                                if crate::rt_core::executor::on_rt_control_thread() =>
                            {
                                shadow_rt.record_external_staging_loss(1);
                            }
                            crate::supervisor_lock::DriverSlot::Legacy(_) => {
                                if d.try_with_mut(|drv| shadow_rt.drain_from_driver(drv)).is_err() {
                                    shadow_rt.record_external_staging_loss(1);
                                }
                            }
                        }
                    }
                }
                PanicDisableOutcome::TimedOut | PanicDisableOutcome::Failed(_) => {
                    self.torque_shadow.record_external_staging_loss(1);
                }
            }
            self.flush_torque_shadow();
        }
        outcome
    }

    pub(in crate::supervisor) fn attempt_bounded_teardown_disable(
        &mut self,
        timeout: Duration,
        is_caught_panic: bool,
    ) -> PanicDisableOutcome {
        #[cfg(not(target_arch = "wasm32"))]
        let (pre_intent, pre_torqued, pre_stopped, pre_ts) = (
            self.shadow_live_intent(),
            self.is_torqued,
            self.safety.state().is_stopped(),
            self.cached_driver_clock_now(),
        );
        self.mark_panic_disable_requested();
        let origin = Instant::now();
        let (coop_deadline, hard_deadline) = panic_disable_deadlines(origin, timeout);
        let coop_budget = coop_deadline.saturating_duration_since(origin);
        let outcome = run_with_panic_deadline_observer_until(
            hard_deadline,
            || self.attempt_panic_disable_until(coop_deadline, hard_deadline, coop_budget),
            |_| true,
            move || {
                PanicDisableOutcome::TimedOut.log("Bounded panic disable", timeout);
            },
        )
        .unwrap_or(PanicDisableOutcome::TimedOut);
        #[cfg(not(target_arch = "wasm32"))]
        self.record_post_panic_disable_shadow_at(
            is_caught_panic,
            &outcome,
            pre_intent,
            pre_torqued,
            pre_stopped,
            pre_ts,
        );
        outcome
    }

    /// Record panic or forced-teardown shadow state AFTER the live deadline observer has sealed
    /// the live `PanicDisableOutcome`. Signals incomplete staging loss if the driver timed out
    /// or panicked so `interval_unverifiable` latches cleanly. No-op (zero locks) when disabled.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn record_post_panic_disable_shadow(
        &mut self,
        is_caught_panic: bool,
        outcome: &PanicDisableOutcome,
    ) {
        let pre_intent = self.shadow_live_intent();
        let pre_torqued = self.is_torqued;
        let pre_stopped = self.safety.state().is_stopped();
        let pre_ts = self.cached_driver_clock_now();
        self.record_post_panic_disable_shadow_at(
            is_caught_panic,
            outcome,
            pre_intent,
            pre_torqued,
            pre_stopped,
            pre_ts,
        );
    }

    /// Record panic or forced-teardown shadow state using the pre-disable live state and causal
    /// timestamp captured BEFORE `attempt_panic_disable_until` ran.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn record_post_panic_disable_shadow_at(
        &mut self,
        is_caught_panic: bool,
        outcome: &PanicDisableOutcome,
        pre_intent: crate::torque::TorqueIntent,
        pre_torqued: bool,
        pre_stopped: bool,
        pre_ts: Instant,
    ) {
        if !self.torque_shadow.is_enabled() {
            return;
        }
        // Emit any MIT ownership release deferred during `attempt_panic_disable_until` strictly
        // AFTER the live panic deadline observer has sealed `outcome`.
        let deferred_mit = self.torque_shadow.take_deferred_mit_off();
        if !deferred_mit.is_empty() {
            self.torque_shadow.enqueue_at(
                pre_ts,
                crate::torque::shadow::ShadowInputKind::MitOwnershipChange {
                    channels: deferred_mit,
                    target_mode: crate::motor_mode::ControlMode::PositionVelocity,
                    device_wide: false,
                },
            );
        }
        if is_caught_panic {
            self.torque_shadow.enqueue_at(
                pre_ts,
                crate::torque::shadow::ShadowInputKind::Panic {
                    tier: crate::torque::PanicTier::CaughtControlPanic,
                    live_intent: pre_intent,
                    live_is_torqued: pre_torqued,
                    live_stopped: pre_stopped,
                },
            );
        } else {
            self.torque_shadow.enqueue_at(
                pre_ts,
                crate::torque::shadow::ShadowInputKind::ProcessExit {
                    live_intent: pre_intent,
                    live_is_torqued: pre_torqued,
                    live_stopped: pre_stopped,
                },
            );
        }
        match outcome {
            PanicDisableOutcome::Confirmed
            | PanicDisableOutcome::Unconfirmed
            | PanicDisableOutcome::NoDriver => {
                if let Some(ref mut d) = self.driver {
                    // Hard-RT invariant: NEVER attempt a driver mutex lock from `armd-control`
                    // (even `try_lock`) on emergency teardown; exclusive RT driver drains
                    // lock-free, while legacy slots on `armd-control` latch loss lock-free.
                    let shadow_rt = &mut self.torque_shadow;
                    match d {
                        crate::supervisor_lock::DriverSlot::Exclusive(drv) => {
                            shadow_rt.drain_from_driver(drv.get_mut().as_mut());
                        }
                        crate::supervisor_lock::DriverSlot::Legacy(_)
                            if crate::rt_core::executor::on_rt_control_thread() =>
                        {
                            shadow_rt.record_external_staging_loss(1);
                        }
                        crate::supervisor_lock::DriverSlot::Legacy(_) => {
                            if d.try_with_mut(|drv| shadow_rt.drain_from_driver(drv)).is_err() {
                                shadow_rt.record_external_staging_loss(1);
                            }
                        }
                    }
                }
            }
            PanicDisableOutcome::TimedOut | PanicDisableOutcome::Failed(_) => {
                self.torque_shadow.record_external_staging_loss(1);
            }
        }
        self.flush_torque_shadow();
    }

    pub(crate) fn attempt_panic_disable_until(
        &mut self,
        coop_deadline: Instant,
        hard_deadline: Instant,
        coop_budget: Duration,
    ) -> PanicDisableOutcome {
        self.mark_panic_disable_requested();
        let Some(ref mut d) = self.driver else {
            self.disable_accepted_inner(false);
            return PanicDisableOutcome::NoDriver;
        };
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| match d {
            crate::supervisor_lock::DriverSlot::Exclusive(drv) => {
                if Instant::now() >= hard_deadline {
                    return PanicDisableOutcome::TimedOut;
                }
                bounded_panic_disable_driver_until(drv.get_mut().as_mut(), coop_deadline, hard_deadline, coop_budget)
            }
            crate::supervisor_lock::DriverSlot::Legacy(lock) => {
                let mut drv = loop {
                    match lock.try_lock() {
                        Ok(g) => break g,
                        Err(std::sync::TryLockError::Poisoned(p)) => break p.into_inner(),
                        Err(std::sync::TryLockError::WouldBlock) => {
                            let rem = hard_deadline.saturating_duration_since(Instant::now());
                            if rem.is_zero() {
                                return PanicDisableOutcome::TimedOut;
                            }
                            std::thread::sleep(rem.min(Duration::from_millis(1)));
                        }
                    }
                };
                if Instant::now() >= hard_deadline {
                    return PanicDisableOutcome::TimedOut;
                }
                bounded_panic_disable_driver_until(&mut **drv, coop_deadline, hard_deadline, coop_budget)
            }
        }))
        .unwrap_or_else(|_| PanicDisableOutcome::Failed("panic during driver panic-disable".into()));
        if outcome == PanicDisableOutcome::Confirmed {
            self.disable_accepted_inner(false);
        }
        outcome
    }

    pub fn driver_feedback_confirmed(&self) -> bool {
        self.read_driver_for_tick()
            .is_some_and(|drv| drv.is_feedback_valid())
    }
}
