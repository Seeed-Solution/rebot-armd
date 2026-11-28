//! Background disable and active torque transition release servicing.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

impl SupervisorInner {
    /// Serves any background (non-broker) disable handshake and bounded [`BackgroundDisableRetry`]
    /// started by `request_disable_bounded` or a transport-failed broker disable under RT executor
    /// ownership, sharing Step 2's single RX budget.
    ///
    /// Every retry routes strictly through the driver's single
    /// `begin_disable_torque_budgeted(kind, 0)` (`send(DisableRequested | DisableProtective, ...)`)
    /// choke point with the ORIGINAL `DisableKind` (`started_at`) and rest-pose gate within the
    /// single request lifetime. Exhausting retries or the window latches
    /// `StaticFault::DisableUnconfirmed` with zero heap allocations; choke-point refusals
    /// (`TorqueOffWouldDrop` / `DisableRequestExpired`) terminate cleanly without retrying.
    pub(crate) fn service_background_disable_with_clock(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Option<(u16, bool)> {
        if !self.disable_pending || self.torque_transition.is_some() {
            return None;
        }
        self.driver.as_ref()?;
        if matches!(
            self.background_disable_retry,
            Some(BackgroundDisableRetry::Exhausted)
        ) {
            return None;
        }
        let control_release_seq = self.control_release_seq;
        // Probe / poll phase under non-blocking try_lock (M4).
        enum BgPollStep {
            Contended,
            Poisoned,
            BudgetExpired,
            Polled {
                poll_res: Result<crate::driver::TorqueProgress, crate::driver::ArmDriverError>,
                rx: Option<(u16, bool)>,
                post_now: Instant,
                still_pending: bool,
                frames_dispatched: bool,
                dispatched_mask: u64,
            },
            Idle {
                drv_now: Instant,
            },
        }
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let measured_positions = &self.measured_positions;
        let conservative_cap = self.pending_conservative_tx_cap;
        let mut cons_rx_delta = 0u16;
        let mut cons_tx_delta = 0u16;
        let mut new_snap = None;
        let step = {
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let d = self.driver.as_mut().expect("checked is_some");
            match d.try_with_mut(|drv| {
                if control_release_seq > 0 {
                    drv.begin_control_release(control_release_seq);
                }
                if drv.torque_transition_pending() {
                    let Ok((_, _, drv_deadline)) =
                        crate::supervisor::translate_driver_cutoff_conservative(
                            drv,
                            budget.deadline,
                            now_fn,
                        )
                    else {
                        return BgPollStep::BudgetExpired;
                    };
                    let before_cons = if conservative_cap {
                        let bc = drv.tx().release_budget_consumed();
                        drv.tx_mut().sync_release_budget_limits(0, 0);
                        Some(bc)
                    } else {
                        None
                    };
                    let poll_res = drv.poll_torque_budgeted(drv_deadline, budget.max_rx_frames);
                    if let Some(bc) = before_cons {
                        let after = drv.tx().release_budget_consumed();
                        cons_rx_delta = cons_rx_delta.saturating_add(after.0.saturating_sub(bc.0));
                        cons_tx_delta = cons_tx_delta.saturating_add(after.1.saturating_sub(bc.1));
                    }
                    let rx = drv.take_last_poll_rx();
                    let post_now = drv.clock_now();
                    new_snap = Some(OwnerDriverSnapshot::capture_from_driver(
                        &*drv,
                        dof,
                        feedback_numeric_valid,
                        measured_positions,
                        &mut ages_buf,
                        &mut bounds_buf,
                    ));
                    let still_pending = drv.torque_transition_pending();
                    let frames_dispatched = drv.disable_frames_dispatched();
                    let dispatched_mask = drv.disable_dispatched_mask();
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    BgPollStep::Polled {
                        poll_res,
                        rx,
                        post_now,
                        still_pending,
                        frames_dispatched,
                        dispatched_mask,
                    }
                } else {
                    BgPollStep::Idle { drv_now: drv.clock_now() }
                }
            }) {
                Ok(v) => v,
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => BgPollStep::Contended,
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => BgPollStep::Poisoned,
            }
        };
        if conservative_cap {
            self.conservative_step_rx_delta =
                self.conservative_step_rx_delta.saturating_add(cons_rx_delta);
            self.conservative_step_tx_delta =
                self.conservative_step_tx_delta.saturating_add(cons_tx_delta);
        }
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
        if let Some(snap) = new_snap {
            self.owner_driver_snapshot = snap;
        }
        let drv_now = match step {
            BgPollStep::Contended => {
                self.record_disable_lock_contention();
                return None;
            }
            BgPollStep::Poisoned => return None,
            BgPollStep::BudgetExpired => return Some((0, true)),
            BgPollStep::Polled {
                poll_res,
                rx,
                post_now,
                still_pending,
                frames_dispatched,
                dispatched_mask,
            } => {
                self.reset_disable_lock_contention();
                let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                match poll_res {
                    Ok(crate::driver::TorqueProgress::Done(_)) => {
                        self.disable_accepted();
                        let _ = self.torque_off_finish_rt();
                    }
                    Err(
                        crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired,
                    ) => {
                        if frames_dispatched {
                            self.disable_first_frame_accepted();
                        }
                        if partial_remaining {
                            match self.background_disable_retry {
                                None => {
                                    let base_kind = self.active_disable_kind.unwrap_or(
                                        crate::driver::DisableKind::DisableRequested {
                                            started_at: post_now,
                                        },
                                    );
                                    if let Some(rk) = self.commit_partial_disable_retry_kind(
                                        base_kind,
                                        post_now,
                                        true,
                                        true,
                                    ) {
                                        let _ = self.arm_background_disable_retry_immediate_with_kind(
                                            rk,
                                            post_now,
                                        );
                                    }
                                }
                                Some(BackgroundDisableRetry::Active {
                                    attempts,
                                    kind,
                                    started_at,
                                    client_park,
                                    ..
                                }) => {
                                    if let Some(expected_stop_gen) = client_park {
                                        if self.client_park_stop_invalidated(expected_stop_gen) {
                                            self.clear_active_disable_retry_state();
                                            return rx;
                                        }
                                    }
                                    let window = self.background_disable_window_for(kind);
                                    if post_now.saturating_duration_since(started_at) >= window
                                        || attempts >= BACKGROUND_DISABLE_MAX_RETRIES
                                    {
                                        self.exhaust_background_disable_retry();
                                    } else {
                                        self.background_disable_retry =
                                            Some(BackgroundDisableRetry::Active {
                                                attempts,
                                                kind,
                                                started_at,
                                                next_retry_at: post_now,
                                                client_park,
                                            });
                                    }
                                }
                                Some(BackgroundDisableRetry::Exhausted) => {}
                            }
                        } else {
                            self.disable_pending = false;
                            self.active_disable_kind = None;
                            self.last_disable_attempt = None;
                            self.background_disable_retry = None;
                        }
                    }
                    Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                        if frames_dispatched {
                            self.disable_first_frame_accepted();
                        }
                        if let Some(BackgroundDisableRetry::Active {
                            kind,
                            started_at,
                            ..
                        }) = self.background_disable_retry
                        {
                            let window = self.background_disable_window_for(kind);
                            if post_now.saturating_duration_since(started_at) >= window {
                                if matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
                                    && self.torque_off_stopped()
                                    && !frames_dispatched
                                {
                                    self.disable_pending = false;
                                    self.active_disable_kind = None;
                                    self.last_disable_attempt = None;
                                    self.background_disable_retry = None;
                                } else {
                                    self.exhaust_background_disable_retry();
                                }
                            }
                        }
                    }
                    Err(_) => {
                        if frames_dispatched {
                            self.disable_first_frame_accepted();
                        }
                        match self.background_disable_retry {
                            Some(BackgroundDisableRetry::Active {
                                attempts,
                                kind,
                                started_at,
                                client_park,
                                ..
                            }) => {
                                if let Some(expected_stop_gen) = client_park {
                                    if self.client_park_stop_invalidated(expected_stop_gen) {
                                        self.clear_active_disable_retry_state();
                                        return rx;
                                    }
                                }
                                let window = self.background_disable_window_for(kind);
                                let window_expired =
                                    post_now.saturating_duration_since(started_at) >= window;
                                if window_expired
                                    && matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
                                    && self.torque_off_stopped()
                                    && !frames_dispatched
                                {
                                    self.clear_active_disable_retry_state();
                                } else if window_expired
                                    || (!still_pending && attempts >= BACKGROUND_DISABLE_MAX_RETRIES)
                                {
                                    self.exhaust_background_disable_retry();
                                } else if !still_pending {
                                    let is_protective = matches!(
                                        kind,
                                        crate::driver::DisableKind::DisableProtective { .. }
                                    );
                                    self.background_disable_retry =
                                        Some(BackgroundDisableRetry::Active {
                                            attempts,
                                            kind,
                                            started_at,
                                            next_retry_at: if partial_remaining || is_protective {
                                                post_now
                                            } else {
                                                post_now + BACKGROUND_DISABLE_RETRY_INTERVAL
                                            },
                                            client_park,
                                        });
                                }
                            }
                            Some(BackgroundDisableRetry::Exhausted) => {}
                            None => {
                                let kind = self.active_disable_kind.unwrap_or(
                                    crate::driver::DisableKind::DisableRequested {
                                        started_at: post_now,
                                    },
                                );
                                let is_protective = matches!(
                                    kind,
                                    crate::driver::DisableKind::DisableProtective { .. }
                                );
                                if let Some(rk) = self.commit_partial_disable_retry_kind(
                                    kind,
                                    post_now,
                                    frames_dispatched,
                                    partial_remaining,
                                ) {
                                    let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                                } else if is_protective {
                                    let _ = self.arm_background_disable_retry_immediate_with_kind(kind, post_now);
                                } else {
                                    let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                                }
                            }
                        }
                    }
                }
                return rx;
            }
            BgPollStep::Idle { drv_now } => {
                self.reset_disable_lock_contention();
                drv_now
            }
        };

        let Some(BackgroundDisableRetry::Active {
            attempts,
            kind,
            started_at,
            next_retry_at,
            client_park,
        }) = self.background_disable_retry
        else {
            return None;
        };
        if let Some(expected_stop_gen) = client_park {
            if self.client_park_stop_invalidated(expected_stop_gen) {
                self.clear_active_disable_retry_state();
                return None;
            }
        }
        let window = self.background_disable_window_for(kind);
        let window_expired = drv_now.saturating_duration_since(started_at) >= window;
        if window_expired
            && matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
            && self.torque_off_stopped()
            && self.torque_intent != TorqueIntent::Off
        {
            self.clear_active_disable_retry_state();
            return None;
        }
        if attempts >= BACKGROUND_DISABLE_MAX_RETRIES || window_expired {
            self.exhaust_background_disable_retry();
            return None;
        }
        if drv_now < next_retry_at {
            return None;
        }

        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let measured_positions = &self.measured_positions;
        let retry_out = {
            let d = self.driver.as_mut().expect("checked is_some");
            match d.try_with_mut(|drv| {
                let res = if let Some(expected_stop_gen) = client_park {
                    drv.begin_client_park_disable_with_kind(kind, Some(0), expected_stop_gen)
                } else {
                    drv.begin_disable_torque_budgeted(kind, 0)
                };
                let handshake_pending = drv.torque_transition_pending();
                let frames_dispatched = drv.disable_frames_dispatched();
                let dispatched_mask = drv.disable_dispatched_mask();
                let post_now = drv.clock_now();
                let snap = OwnerDriverSnapshot::capture_from_driver(
                    &*drv,
                    dof,
                    feedback_numeric_valid,
                    measured_positions,
                    &mut ages_buf,
                    &mut bounds_buf,
                );
                (res, handshake_pending, frames_dispatched, dispatched_mask, post_now, snap)
            }) {
                Ok(v) => Ok(v),
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
            }
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
        let (res, handshake_pending, frames_dispatched, dispatched_mask, post_now, snap) = match retry_out {
            Ok(v) => {
                self.reset_disable_lock_contention();
                v
            }
            Err(true) => {
                self.record_disable_lock_contention();
                return None;
            }
            Err(false) => return None,
        };
        self.owner_driver_snapshot = snap;
        let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);

        match res {
            Ok(crate::driver::TorqueProgress::Done(_)) => {
                self.disable_accepted();
                let _ = self.torque_off_finish_rt();
                return None;
            }
            Err(
                crate::driver::ArmDriverError::TorqueOffWouldDrop
                | crate::driver::ArmDriverError::DisableRequestExpired,
            ) => {
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                }
                if !partial_remaining {
                    self.disable_pending = false;
                    self.active_disable_kind = None;
                    self.last_disable_attempt = None;
                    self.background_disable_retry = None;
                    return None;
                }
            }
            Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                }
            }
            Err(e) => {
                if let Some(expected_stop_gen) = client_park {
                    if crate::driver::is_estop_inhibited(&e)
                        || self.client_park_stop_invalidated(expected_stop_gen)
                    {
                        self.clear_active_disable_retry_state();
                        return None;
                    }
                }
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                }
            }
        }

        let next_attempts = attempts.saturating_add(1);
        let window_expired = post_now.saturating_duration_since(started_at) >= window;
        if window_expired
            && matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
            && self.torque_off_stopped()
            && !frames_dispatched
        {
            self.clear_active_disable_retry_state();
        } else if window_expired || (!handshake_pending && next_attempts >= BACKGROUND_DISABLE_MAX_RETRIES)
        {
            self.exhaust_background_disable_retry();
        } else {
            let is_protective = matches!(
                kind,
                crate::driver::DisableKind::DisableProtective { .. }
            );
            self.background_disable_retry = Some(BackgroundDisableRetry::Active {
                attempts: next_attempts,
                kind,
                started_at,
                next_retry_at: if partial_remaining || is_protective {
                    post_now
                } else {
                    post_now + BACKGROUND_DISABLE_RETRY_INTERVAL
                },
                client_park,
            });
        }
        None
    }

    /// Serves the transition in flight once, within `budget` (the executor's clock; the
    /// driver's receive pass gets the same span on the driver's clock). None: none in flight.
    /// `Pending { wake }` is on the executor's clock.
    pub(crate) fn service_torque_transition(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
    ) -> Option<TransitionPoll> {
        let mut live_now = Instant::now;
        self.service_torque_transition_with_clock(budget, &mut live_now)
    }

    pub(crate) fn service_torque_transition_with_clock(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Option<TransitionPoll> {
        use crate::rt_core::ops::TorqueProgress as Rt;
        let t = self.torque_transition.take()?;
        let ws_origin_dir = match t {
            TorqueTransition::Enable(EnableReducer {
                origin: EnableOrigin::LegacyWs { .. } | EnableOrigin::DetachedCancel,
                ..
            }) => Some("on"),
            TorqueTransition::Disable {
                origin: DisableOrigin::LegacyWs { .. } | DisableOrigin::Detached { .. },
                ..
            } => Some("off"),
            _ => None,
        };
        self.last_tick_torque_ws_direction = ws_origin_dir;
        // Enforce authoritative request deadline BEFORE dispatch, retry, or arming (Major 1 / B4).
        match t {
            TorqueTransition::Enable(EnableReducer {
                origin: EnableOrigin::LegacyWs { token, slot, deadline, .. },
                ..
            }) if Instant::now() >= deadline => {
                self.torque_transition = Some(t);
                let _ = self.expire_legacy_torque_transition_inner(token, slot, true, true);
                return Some(TransitionPoll {
                    progress: Err("DEADLINE_EXPIRED"),
                    rx: None,
                    broker_owned: false,
                });
            }
            TorqueTransition::Disable {
                origin: DisableOrigin::LegacyWs { token, slot, deadline },
                ..
            } if Instant::now() >= deadline => {
                self.torque_transition = Some(t);
                let _ = self.expire_legacy_torque_transition_inner(token, slot, false, true);
                return Some(TransitionPoll {
                    progress: Err("DEADLINE_EXPIRED"),
                    rx: None,
                    broker_owned: false,
                });
            }
            TorqueTransition::Disable {
                origin: DisableOrigin::Detached { deadline: Some(dl) },
                kind,
            } if Instant::now() >= dl => {
                match self.try_abort_expired_undispatched_disable(kind, Some(dl)) {
                    DisableExpiryDecision::Aborted => {
                        return Some(TransitionPoll {
                            progress: Err("DEADLINE_EXPIRED"),
                            rx: None,
                            broker_owned: false,
                        });
                    }
                    DisableExpiryDecision::ContendedRetry
                    | DisableExpiryDecision::KeepDetachedWithDeadline => {
                        return Some(TransitionPoll {
                            progress: Ok(Rt::Pending { wake: budget.now }),
                            rx: None,
                            broker_owned: false,
                        });
                    }
                    DisableExpiryDecision::KeepDetachedDispatched => {
                        // Already dispatched (or protective): let the next line poll to completion
                        // with `deadline: None`.
                        let next = self.torque_transition.take()?;
                        return self.service_torque_transition_post_deadline(next, budget, now_fn);
                    }
                }
            }
            _ => {}
        }
        let out = self.service_torque_transition_post_deadline(t, budget, now_fn);
        self.sync_detached_torque_flag();
        out
    }

    pub(in crate::supervisor) fn service_torque_transition_post_deadline(
        &mut self,
        t: TorqueTransition,
        budget: &crate::rt_core::ops::ServiceBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Option<TransitionPoll> {
        use crate::driver::TorqueProgress as Drv;
        use crate::rt_core::ops::TorqueProgress as Rt;
        let broker_owned = matches!(
            t,
            TorqueTransition::Enable(EnableReducer { origin: EnableOrigin::BrokerTransition, .. })
                | TorqueTransition::Disable { origin: DisableOrigin::BrokerTransition, .. }
        );
        match t {
            TorqueTransition::Enable(reducer) => {
                let origin = reducer.origin;
                let (outcome, rx) = self.poll_enable_reducer_with_clock(reducer, Some(budget), now_fn);
                let progress = match outcome {
                    EnableStepOutcome::Armed(_) => {
                        match origin {
                            EnableOrigin::ParkRearm { action_id, stop_gen } => {
                                if let Some(c) = self
                                    .park_continuation
                                    .as_mut()
                                    .filter(|c| c.action_id == action_id && c.stop_gen == stop_gen)
                                {
                                    c.phase = super::ParkContinuationPhase::ReadyForPlan;
                                }
                                self.notify_park_continuation();
                            }
                            EnableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_accepted(token, slot);
                            }
                            EnableOrigin::BrokerTransition | EnableOrigin::DetachedCancel => {}
                        }
                        Ok(Rt::Done)
                    }
                    EnableStepOutcome::Pending { wake } => Ok(Rt::Pending { wake }),
                    EnableStepOutcome::Failed {
                        rt_code,
                        message,
                        feedback_unavailable,
                        estop_interrupted,
                    } => {
                        match origin {
                            EnableOrigin::ParkRearm { action_id, .. } => {
                                if rt_code != "STOPPED" {
                                    self.finish_park_rearm_failed(action_id, rt_code);
                                }
                            }
                            EnableOrigin::LegacyWs { token, slot, .. } => {
                                let failure = Self::classify_legacy_enable_failure(
                                    rt_code,
                                    message,
                                    feedback_unavailable,
                                    estop_interrupted,
                                );
                                self.settle_legacy_ws_failed(token, slot, failure);
                            }
                            EnableOrigin::BrokerTransition | EnableOrigin::DetachedCancel => {}
                        }
                        Err(rt_code)
                    }
                };
                Some(TransitionPoll { progress, rx, broker_owned })
            }
            TorqueTransition::Disable { origin, kind } => {
                if self.driver.is_none() {
                    match origin {
                        DisableOrigin::Park { watchdog, action_id, .. } => {
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                "INTERNAL",
                                "Driver missing during park disable",
                            );
                        }
                        DisableOrigin::LegacyWs { token, slot, .. } => {
                            self.settle_legacy_ws_failed(
                                token,
                                slot,
                                LegacyTorqueFailure::StaticCode("INTERNAL"),
                            );
                        }
                        DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                    }
                    return Some(TransitionPoll { progress: Err("INTERNAL"), rx: None, broker_owned });
                }
                let req_deadline = match origin {
                    DisableOrigin::LegacyWs { deadline, .. } => Some(deadline),
                    DisableOrigin::Detached { deadline } => deadline,
                    _ => None,
                };
                let is_protective =
                    matches!(kind, crate::driver::DisableKind::DisableProtective { .. });
                let control_release_seq = self.control_release_seq;
                let rt_executor_owned = self.rt_executor_owned;
                let nonblocking_lock = rt_executor_owned
                    || self.nonblocking_driver_release
                    || broker_owned
                    || matches!(origin, DisableOrigin::LegacyWs { .. } | DisableOrigin::Detached { .. });
                let dof = self.config.dof;
                let feedback_numeric_valid = self.feedback_numeric_valid;
                let measured_positions = &self.measured_positions;
                let conservative_cap = self.pending_conservative_tx_cap;
                let mut cons_rx_delta = 0u16;
                let mut cons_tx_delta = 0u16;
                let mut ages_buf = std::mem::take(&mut self.scratch.ages);
                let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
                #[allow(clippy::large_enum_variant)] // RT stack-only enum; avoids heap Box
                enum DisablePollOutcome {
                    Ran(
                        Instant,
                        Result<Drv, crate::driver::ArmDriverError>,
                        bool,
                        u64,
                        Option<(u16, bool)>,
                        Instant,
                        OwnerDriverSnapshot,
                    ),
                    BudgetExpired(Instant),
                    Contended,
                    Poisoned,
                }
                let poll_out = {
                    #[cfg(not(target_arch = "wasm32"))]
                    let shadow_rt = &mut self.torque_shadow;
                    let d = self.driver.as_mut().expect("checked is_some");
                    let run_poll = |drv: &mut dyn crate::driver::ArmDriver| {
                        if control_release_seq > 0 {
                            drv.begin_control_release(control_release_seq);
                        }
                        let (drv_now, live_owner_now, mut drv_deadline) =
                            match crate::supervisor::translate_driver_cutoff_conservative(
                                drv,
                                budget.deadline,
                                now_fn,
                            ) {
                                Ok(translated) => translated,
                                Err((_, owner_now)) => {
                                    return DisablePollOutcome::BudgetExpired(owner_now);
                                }
                            };
                        // Bound any undispatched non-protective WS/Detached disable by its
                        // remaining request deadline so first dispatch never crosses expiry (R5 B2/M2).
                        let mut applied_ws_dl = false;
                        if let Some(dl) = req_deadline {
                            if !is_protective && !drv.disable_frames_dispatched() {
                                let req_rem = dl.saturating_duration_since(live_owner_now);
                                drv_deadline = drv_deadline.min(drv_now + req_rem);
                                drv.set_tx_release_deadline(Some(drv_deadline));
                                applied_ws_dl = true;
                            }
                        }
                        let use_budgeted = broker_owned
                            || rt_executor_owned
                            || drv.supports_rt_executor().is_ok();
                        let before_cons = if conservative_cap {
                            let bc = drv.tx().release_budget_consumed();
                            drv.tx_mut().sync_release_budget_limits(0, 0);
                            Some(bc)
                        } else {
                            None
                        };
                        let r = if use_budgeted {
                            drv.poll_torque_budgeted(drv_deadline, budget.max_rx_frames)
                        } else {
                            drv.poll_torque(drv_deadline)
                        };
                        if let Some(bc) = before_cons {
                            let after = drv.tx().release_budget_consumed();
                            cons_rx_delta = cons_rx_delta.saturating_add(after.0.saturating_sub(bc.0));
                            cons_tx_delta = cons_tx_delta.saturating_add(after.1.saturating_sub(bc.1));
                        }
                        if applied_ws_dl {
                            drv.set_tx_release_deadline(None);
                        }
                        let frames_dispatched = drv.disable_frames_dispatched();
                        let dispatched_mask = drv.disable_dispatched_mask();
                        let rx = drv.take_last_poll_rx();
                        let post_now = drv.clock_now();
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
                        DisablePollOutcome::Ran(
                            drv_now,
                            r,
                            frames_dispatched,
                            dispatched_mask,
                            rx,
                            post_now,
                            snap,
                        )
                    };
                    if nonblocking_lock {
                        match d.try_with_mut(run_poll) {
                            Ok(v) => v,
                            Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                                DisablePollOutcome::Contended
                            }
                            Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                                DisablePollOutcome::Poisoned
                            }
                        }
                    } else {
                        d.with_mut(run_poll).unwrap_or(DisablePollOutcome::Poisoned)
                    }
                };
                if conservative_cap {
                    self.conservative_step_rx_delta =
                        self.conservative_step_rx_delta.saturating_add(cons_rx_delta);
                    self.conservative_step_tx_delta =
                        self.conservative_step_tx_delta.saturating_add(cons_tx_delta);
                }
                self.scratch.ages = ages_buf;
                self.scratch.fb.bounds = bounds_buf;
                let (drv_now, r, frames_dispatched, dispatched_mask, rx, post_now, snap) = match poll_out {
                    DisablePollOutcome::Ran(
                        drv_now,
                        r,
                        frames_dispatched,
                        dispatched_mask,
                        rx,
                        post_now,
                        snap,
                    ) => {
                        self.reset_disable_lock_contention();
                        (drv_now, r, frames_dispatched, dispatched_mask, rx, post_now, snap)
                    }
                    DisablePollOutcome::BudgetExpired(owner_now) => {
                        self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                        return Some(TransitionPoll {
                            progress: Ok(Rt::Pending { wake: owner_now }),
                            rx: Some((0, true)),
                            broker_owned,
                        });
                    }
                    DisablePollOutcome::Contended => {
                        self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                        self.record_disable_lock_contention();
                        return Some(TransitionPoll {
                            progress: Ok(Rt::Pending { wake: budget.now }),
                            rx: None,
                            broker_owned,
                        });
                    }
                    DisablePollOutcome::Poisoned => {
                        match origin {
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_failed(
                                    watchdog,
                                    action_id,
                                    "FEEDBACK_UNAVAILABLE",
                                    "Driver unavailable during park disable",
                                );
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_failed(
                                    token,
                                    slot,
                                    LegacyTorqueFailure::FeedbackUnavailable,
                                );
                            }
                            DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                        }
                        return Some(TransitionPoll {
                            progress: Err("FEEDBACK_UNAVAILABLE"),
                            rx: None,
                            broker_owned,
                        });
                    }
                };
                self.owner_driver_snapshot = snap;
                let to_exec = |wake: Instant| budget.now + wake.saturating_duration_since(drv_now);
                match r {
                    Ok(Drv::Pending { wake }) => {
                        if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                            self.disable_first_frame_accepted();
                        }
                        self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                        Some(TransitionPoll {
                            progress: Ok(Rt::Pending { wake: to_exec(wake) }),
                            rx,
                            broker_owned,
                        })
                    }
                    Ok(Drv::Done(_)) => match origin {
                        DisableOrigin::BrokerTransition => {
                            self.disable_accepted();
                            let progress = self.torque_off_finish_rt().map(|()| Rt::Done);
                            Some(TransitionPoll { progress, rx, broker_owned })
                        }
                        DisableOrigin::Park { watchdog, action_id, .. } => {
                            self.complete_park_disable_accepted(watchdog, action_id);
                            Some(TransitionPoll { progress: Ok(Rt::Done), rx, broker_owned })
                        }
                        DisableOrigin::LegacyWs { token, slot, .. } => {
                            self.disable_accepted();
                            let progress = match self.torque_off_finish_rt() {
                                Ok(()) => {
                                    self.settle_legacy_ws_accepted(token, slot);
                                    Ok(Rt::Done)
                                }
                                Err(code) => {
                                    self.settle_legacy_ws_disable_failed(token, slot, code, None);
                                    Err(code)
                                }
                            };
                            Some(TransitionPoll { progress, rx, broker_owned })
                        }
                        DisableOrigin::Detached { .. } => {
                            self.disable_accepted();
                            let progress = self.torque_off_finish_rt().map(|()| Rt::Done);
                            Some(TransitionPoll { progress, rx, broker_owned })
                        }
                    },
                    Err(e @ crate::driver::ArmDriverError::PartialDisable { .. }) => {
                        let (err_detail, code) = self.handle_partial_disable_abort(e);
                        match origin {
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_failed(
                                    watchdog,
                                    action_id,
                                    code,
                                    err_detail.as_str(),
                                );
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_failed(
                                    token,
                                    slot,
                                    LegacyTorqueFailure::DisableFailed { detail: err_detail },
                                );
                            }
                            DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                        }
                        Some(TransitionPoll {
                            progress: Err(code),
                            rx,
                            broker_owned,
                        })
                    }
                    Err(crate::driver::ArmDriverError::LeaseFenced) => {
                        self.clear_active_disable_retry_state();
                        match origin {
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_failed(
                                    watchdog,
                                    action_id,
                                    "LEASE_LOST",
                                    "Lease lost before park disable dispatched",
                                );
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_failed(
                                    token,
                                    slot,
                                    LegacyTorqueFailure::LeaseLost,
                                );
                            }
                            DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                        }
                        Some(TransitionPoll {
                            progress: Err("LEASE_LOST"),
                            rx,
                            broker_owned,
                        })
                    }
                    Err(e) => {
                        if let DisableOrigin::Park {
                            watchdog: false,
                            action_id,
                            ..
                        } = origin
                        {
                            if !frames_dispatched
                                && crate::driver::is_estop_inhibited(&e)
                                && !self.e_stop_latched.load(Ordering::SeqCst)
                                && !self.estop_pending()
                                && !self.safety.state().is_stopped()
                            {
                                self.clear_active_disable_retry_state();
                                self.cancel_park_before_disable(action_id);
                                return Some(TransitionPoll {
                                    progress: Err("STOPPED"),
                                    rx,
                                    broker_owned,
                                });
                            }
                        }
                        if crate::driver::is_estop_inhibited(&e) {
                            match origin {
                                DisableOrigin::Park { watchdog, action_id, .. } => {
                                    self.finish_park_disable_failed(
                                        watchdog,
                                        action_id,
                                        "ESTOP_ACTIVE",
                                        "E-stop inhibited park disable",
                                    );
                                }
                                DisableOrigin::LegacyWs { token, slot, .. } => {
                                    self.settle_legacy_ws_failed(
                                        token,
                                        slot,
                                        LegacyTorqueFailure::EStopActive,
                                    );
                                }
                                DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                            }
                            return Some(TransitionPoll {
                                progress: Err("ESTOP_ACTIVE"),
                                rx,
                                broker_owned,
                            });
                        }
                        let refused = matches!(
                            e,
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                                | crate::driver::ArmDriverError::DisableRequestExpired
                        );
                        let client_park_stopped = matches!(
                            origin,
                            DisableOrigin::Park {
                                watchdog: false,
                                expected_stop_gen,
                                ..
                            } if self.client_park_stop_invalidated(expected_stop_gen)
                        );
                        let mut err_detail =
                            crate::safety::BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::empty();
                        err_detail.push_str("Torque disable transmission failed: ");
                        e.write_bounded(&mut err_detail);
                        let (code, text) = if client_park_stopped {
                            self.clear_active_disable_retry_state();
                            let code = self.disable_step_failed_inner(e, false);
                            (code, "Client park disable stopped mid-batch")
                        } else {
                            if let DisableOrigin::Park {
                                watchdog: false,
                                expected_stop_gen,
                                ..
                            } = origin
                            {
                                self.active_disable_client_park = Some(expected_stop_gen);
                            }
                            let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                            let partial_retry = self.commit_partial_disable_retry_kind(
                                kind,
                                post_now,
                                frames_dispatched,
                                partial_remaining,
                            );
                            let allow_undispatched_retry = !matches!(
                                origin,
                                DisableOrigin::LegacyWs { .. }
                                    | DisableOrigin::Detached { deadline: Some(_), .. }
                            );
                            if let Some(rk) = partial_retry {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                            } else if !refused
                                && (frames_dispatched || is_protective || allow_undispatched_retry)
                            {
                                let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                            } else if !frames_dispatched && !is_protective && !allow_undispatched_retry {
                                self.clear_active_disable_retry_state();
                            }
                            let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                            let text = if refused && !frames_dispatched {
                                "Park torque-off refused by rest-pose gate"
                            } else {
                                "Torque disable transmission failed"
                            };
                            (code, text)
                        };
                        match origin {
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_failed(watchdog, action_id, code, text);
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_disable_failed(
                                    token,
                                    slot,
                                    code,
                                    Some(err_detail),
                                );
                            }
                            DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                        }
                        Some(TransitionPoll {
                            progress: Err(code),
                            rx,
                            broker_owned,
                        })
                    }
                }
            }
        }
    }
}
