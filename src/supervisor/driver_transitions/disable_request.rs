//! Bounded and unbounded hardware disable request sequencing.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

impl SupervisorInner {
    /// Bounded, zero-allocation error from [`Self::request_disable_bounded`].
    #[allow(clippy::result_large_err)] // inline bounded error keeps RT disable retry zero-alloc
    pub(crate) fn request_disable_bounded(
        &mut self,
        kind: crate::driver::DisableKind,
    ) -> Result<(), RequestDisableError> {
        use crate::rt_core::ops::TorqueProgress as Rt;
        // T2: Never silently discard a pending broker/park `Disable` transition when upgrading to
        // protective disable (e.g. thermal trip). Non-disable transitions (such as an uncancelled
        // `Enable`) end `SUPERSEDED` / `UNLOAD_FAILED`; a pending `Disable` stays attached to the
        // upgraded driver disable handshake (or completes/fails here) so its request receives a
        // terminal delivered exactly once.
        let pending_disable_origin = match self.torque_transition {
            Some(TorqueTransition::Disable { origin, .. }) => Some(origin),
            _ => None,
        };
        let pending_disable = pending_disable_origin.is_some();
        let is_protective = matches!(kind, crate::driver::DisableKind::DisableProtective { .. });
        let prev_protective = matches!(
            self.active_disable_kind,
            Some(crate::driver::DisableKind::DisableProtective { .. })
        );
        if is_protective && !prev_protective {
            // Upgrading from a non-protective disable to thermal/exit protective disable clears
            // any prior non-protective retry exhaustion so the protective cut always runs.
            self.background_disable_retry = None;
        }
        if !pending_disable {
            if let Some(t) = self.torque_transition.take() {
                match t {
                    TorqueTransition::Enable(EnableReducer {
                        origin: EnableOrigin::BrokerTransition,
                        ..
                    }) => {
                        self.core.begin_transition(Err("SUPERSEDED"));
                    }
                    TorqueTransition::Enable(EnableReducer {
                        origin: EnableOrigin::ParkRearm { action_id, .. },
                        ..
                    }) => {
                        self.finish_park_rearm_failed(action_id, "UNLOAD_FAILED");
                    }
                    TorqueTransition::Enable(EnableReducer {
                        origin: EnableOrigin::LegacyWs { token, slot, .. },
                        ..
                    }) => {
                        self.settle_legacy_ws_failed(token, slot, LegacyTorqueFailure::StaticCode("SUPERSEDED"));
                    }
                    TorqueTransition::Enable(EnableReducer {
                        origin: EnableOrigin::DetachedCancel,
                        ..
                    }) => {}
                    TorqueTransition::Disable { .. } => {}
                }
            }
        }
        if self.rt_executor_owned
            && !pending_disable
            && matches!(self.background_disable_retry, Some(BackgroundDisableRetry::Exhausted))
        {
            return Err(RequestDisableError {
                code: "DISABLE_UNCONFIRMED",
                message: crate::safety::BoundedText::new("Background torque disable retries exhausted"),
                refused_by_chokepoint: false,
            });
        }
        if is_protective {
            self.mit_torque_off();
            self.active_disable_client_park = None;
        }
        self.active_disable_kind = Some(kind);
        self.disable_requested();
        #[cfg(not(target_arch = "wasm32"))]
        if let crate::driver::DisableKind::DisableRequested { started_at } = kind {
            let live_op_id = self.torque_shadow.alloc_live_op_id();
            self.torque_shadow.enqueue_at(
                started_at,
                crate::torque::shadow::ShadowInputKind::DisableRequest {
                    live_op_id,
                    explicit_release_authorized: false,
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        }
        if self.driver.is_some() {
            if self.rt_executor_owned {
                let dof = self.config.dof;
                let feedback_numeric_valid = self.feedback_numeric_valid;
                let mut ages_buf = std::mem::take(&mut self.scratch.ages);
                let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
                let outcome = {
                    #[cfg(not(target_arch = "wasm32"))]
                    let shadow_rt = &mut self.torque_shadow;
                    let d = self.driver.as_mut().expect("checked is_some");
                    match d.try_with_mut(|drv| {
                        let drv_now = drv.clock_now();
                        let res = drv.begin_disable_torque_budgeted(kind, 0);
                        let post_now = drv.clock_now();
                        let frames_dispatched = drv.disable_frames_dispatched();
                        let dispatched_mask = drv.disable_dispatched_mask();
                        let snap = OwnerDriverSnapshot::capture_from_driver(
                            &*drv,
                            dof,
                            feedback_numeric_valid,
                            &self.measured_positions,
                            &mut ages_buf,
                            &mut bounds_buf,
                        );
                        #[cfg(not(target_arch = "wasm32"))]
                        shadow_rt.drain_from_driver(drv);
                        (res, drv_now, post_now, frames_dispatched, dispatched_mask, snap)
                    }) {
                        Ok(v) => Ok(v),
                        Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                        Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
                    }
                };
                self.scratch.ages = ages_buf;
                self.scratch.fb.bounds = bounds_buf;
                let (res, drv_now, post_now, frames_dispatched, dispatched_mask, snap) = match outcome {
                    Ok(v) => {
                        self.reset_disable_lock_contention();
                        v
                    }
                    Err(true) => {
                        self.record_disable_lock_contention();
                        if let Some(origin) = pending_disable_origin {
                            self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                        } else if is_protective {
                            // R12 B4: A contended first protective (thermal / exit) cut MUST arm
                            // a bounded per-release retry (`next_retry_at: now`) on the driver's
                            // lock-free clock so `service_background_disable` retries every
                            // release until dispatched.
                            let retry_now = self.driver_clock_now_or(Instant::now());
                            let _ = self.arm_background_disable_retry_immediate_with_kind(
                                kind,
                                retry_now,
                            );
                        }
                        return Err(RequestDisableError {
                            code: "DRIVER_BUSY",
                            message: crate::safety::BoundedText::new("Driver mutex contended on RT disable"),
                            refused_by_chokepoint: false,
                        });
                    }
                    Err(false) => {
                        if let Some(origin) = pending_disable_origin {
                            self.torque_transition = None;
                            match origin {
                                DisableOrigin::BrokerTransition => {
                                    self.core.begin_transition(Err("FEEDBACK_UNAVAILABLE"));
                                }
                                DisableOrigin::Park { watchdog, action_id, .. } => {
                                    self.finish_park_disable_failed(
                                        watchdog,
                                        action_id,
                                        "FEEDBACK_UNAVAILABLE",
                                        "Feedback unavailable",
                                    );
                                }
                                DisableOrigin::LegacyWs { token, slot, .. } => {
                                    self.settle_legacy_ws_failed(
                                        token,
                                        slot,
                                        LegacyTorqueFailure::FeedbackUnavailable,
                                    );
                                }
                                DisableOrigin::Detached { .. } => {}
                            }
                        }
                        return Err(RequestDisableError {
                            code: "FEEDBACK_UNAVAILABLE",
                            message: crate::safety::BoundedText::new("Feedback unavailable"),
                            refused_by_chokepoint: false,
                        });
                    }
                };
                let res = match res {
                    Ok(p) => p,
                    Err(e) => {
                        let refused = matches!(
                            e,
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                                | crate::driver::ArmDriverError::DisableRequestExpired
                        );
                        let mut message = crate::safety::BoundedText::empty();
                        message.push_str("Torque disable transmission failed: ");
                        e.write_bounded(&mut message);
                        let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                        let partial_retry = self.commit_partial_disable_retry_kind(
                            kind,
                            post_now,
                            frames_dispatched,
                            partial_remaining,
                        );
                        let code = if let Some(origin) = pending_disable_origin {
                            self.torque_transition = None;
                            if let Some(rk) = partial_retry {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                            } else if is_protective {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(kind, post_now);
                            } else if !refused {
                                let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                            }
                            let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                            match origin {
                                DisableOrigin::BrokerTransition => {
                                    self.core.begin_transition(Err(code));
                                }
                                DisableOrigin::Park { watchdog, action_id, .. } => {
                                    self.finish_park_disable_failed(
                                        watchdog,
                                        action_id,
                                        code,
                                        "Torque disable transmission failed",
                                    );
                                }
                                DisableOrigin::LegacyWs { token, slot, .. } => {
                                    self.settle_legacy_ws_disable_failed(
                                        token,
                                        slot,
                                        code,
                                        Some(message),
                                    );
                                }
                                DisableOrigin::Detached { .. } => {}
                            }
                            code
                        } else {
                            if let Some(rk) = partial_retry {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                            } else if is_protective {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(kind, post_now);
                            } else if refused {
                                self.disable_pending = false;
                                self.active_disable_kind = None;
                                self.last_disable_attempt = None;
                                self.background_disable_retry = None;
                            } else {
                                let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                            }
                            crate::driver::handshake::rt_progress(false, Err(e))
                                .err()
                                .unwrap_or("HARDWARE_FAULT")
                        };
                        return Err(RequestDisableError {
                            code,
                            message,
                            refused_by_chokepoint: refused && !frames_dispatched,
                        });
                    }
                };
                self.owner_driver_snapshot = snap;
                let done_now = matches!(res, crate::driver::TorqueProgress::Done(_));
                if done_now {
                    self.disable_accepted();
                    if let Some(origin) = pending_disable_origin {
                        self.torque_transition = None;
                        match origin {
                            DisableOrigin::BrokerTransition => {
                                let terminal = self.torque_off_finish_rt().map(|()| Rt::Done);
                                self.core.begin_transition(terminal);
                            }
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_after_accepted(watchdog, action_id);
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                match self.torque_off_finish_rt() {
                                    Ok(()) => self.settle_legacy_ws_accepted(token, slot),
                                    Err(code) => {
                                        self.settle_legacy_ws_disable_failed(token, slot, code, None)
                                    }
                                }
                            }
                            DisableOrigin::Detached { .. } => {
                                let _ = self.torque_off_finish_rt();
                            }
                        }
                    }
                } else {
                    if frames_dispatched {
                        self.disable_first_frame_accepted();
                    }
                    if let Some(origin) = pending_disable_origin {
                        self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                    } else {
                        let _ = self.arm_background_disable_retry_with_kind(kind, drv_now);
                    }
                }
                return Ok(());
            }
            if self
                .driver
                .as_ref()
                .expect("checked is_some")
                .as_legacy_lock()
                .is_some_and(|l| l.is_poisoned())
            {
                if let Some(origin) = pending_disable_origin {
                    self.torque_transition = None;
                    match origin {
                        DisableOrigin::BrokerTransition => {
                            self.core.begin_transition(Err("FEEDBACK_UNAVAILABLE"));
                        }
                        DisableOrigin::Park { watchdog, action_id, .. } => {
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                "FEEDBACK_UNAVAILABLE",
                                "Feedback unavailable",
                            );
                        }
                        DisableOrigin::LegacyWs { token, slot, .. } => {
                            self.settle_legacy_ws_failed(
                                token,
                                slot,
                                LegacyTorqueFailure::FeedbackUnavailable,
                            );
                        }
                        DisableOrigin::Detached { .. } => {}
                    }
                }
                return Err(RequestDisableError {
                    code: "FEEDBACK_UNAVAILABLE",
                    message: crate::safety::BoundedText::new("Feedback unavailable"),
                    refused_by_chokepoint: false,
                });
            }
            let d = self.driver.as_mut().expect("checked is_some");
            let mut drv = match d.write() {
                Some(g) => g,
                None => {
                    return Err(RequestDisableError {
                        code: "FEEDBACK_UNAVAILABLE",
                        message: crate::safety::BoundedText::new("Feedback unavailable"),
                        refused_by_chokepoint: false,
                    });
                }
            };
            // R12 B1: Never hold the driver mutex while waiting/sleeping for K=3 rest-pose
            // evidence on legacy owners. Drivers supporting deferred per-release rest-gate wait
            // (`FeetechArmDriver`, `DamiaoArmDriver`) begin non-blockingly: if awaiting K=3
            // (`Pending` with `!frames_dispatched`), release the mutex immediately so subsequent
            // legacy releases advance one bounded poll step while holds keep running between
            // releases.
            if drv.supports_budgeted_disable_await() {
                match drv.begin_disable_torque(kind) {
                    Ok(crate::driver::TorqueProgress::Done(_)) => {}
                    Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                        let frames_dispatched = drv.disable_frames_dispatched();
                        if !frames_dispatched {
                            let mut ages_buf = std::mem::take(&mut self.scratch.ages);
                            let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
                            self.owner_driver_snapshot = OwnerDriverSnapshot::capture_from_driver(
                                drv.as_ref(),
                                self.config.dof,
                                self.feedback_numeric_valid,
                                &self.measured_positions,
                                &mut ages_buf,
                                &mut bounds_buf,
                            );
                            self.scratch.ages = ages_buf;
                            self.scratch.fb.bounds = bounds_buf;
                            #[cfg(not(target_arch = "wasm32"))]
                            self.torque_shadow.drain_from_driver(&mut **drv);
                            drop(drv);
                            if let Some(origin) = pending_disable_origin {
                                self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                            }
                            return Ok(());
                        }
                        // Frame 1 already dispatched (`torque_intent = Off`): complete remaining
                        // bounded disable repeats without re-sending repeat 1 or starving holds.
                        if let Err(e) = drv.bus_recovery_mut().finish_dispatched_disable() {
                            let refused = matches!(
                                e,
                                crate::driver::ArmDriverError::TorqueOffWouldDrop
                                    | crate::driver::ArmDriverError::DisableRequestExpired
                            );
                            let post_now = drv.clock_now();
                            let frames_dispatched = drv.disable_frames_dispatched() || true;
                            let dispatched_mask = drv.disable_dispatched_mask();
                            let mut message = crate::safety::BoundedText::empty();
                            message.push_str("Torque disable transmission failed: ");
                            e.write_bounded(&mut message);
                            #[cfg(not(target_arch = "wasm32"))]
                            self.torque_shadow.drain_from_driver(&mut **drv);
                            drop(drv);
                            let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                            let partial_retry = self.commit_partial_disable_retry_kind(
                                kind,
                                post_now,
                                frames_dispatched,
                                partial_remaining,
                            );
                            let code = if let Some(origin) = pending_disable_origin {
                                self.torque_transition = None;
                                let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                                match origin {
                                    DisableOrigin::BrokerTransition => {
                                        self.core.begin_transition(Err(code));
                                    }
                                    DisableOrigin::Park { watchdog, action_id, .. } => {
                                        self.finish_park_disable_failed(
                                            watchdog,
                                            action_id,
                                            code,
                                            "Torque disable transmission failed",
                                        );
                                    }
                                    DisableOrigin::LegacyWs { token, slot, .. } => {
                                        self.settle_legacy_ws_disable_failed(
                                            token,
                                            slot,
                                            code,
                                            Some(message),
                                        );
                                    }
                                    DisableOrigin::Detached { .. } => {}
                                }
                                code
                            } else {
                                if refused && partial_retry.is_none() {
                                    self.disable_pending = false;
                                    self.active_disable_kind = None;
                                    self.last_disable_attempt = None;
                                    self.background_disable_retry = None;
                                }
                                crate::driver::handshake::rt_progress(false, Err(e))
                                    .err()
                                    .unwrap_or("HARDWARE_FAULT")
                            };
                            return Err(RequestDisableError {
                                code,
                                message,
                                refused_by_chokepoint: refused && !frames_dispatched,
                            });
                        }
                    }
                    Err(e) => {
                        let refused = matches!(
                            e,
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                                | crate::driver::ArmDriverError::DisableRequestExpired
                        );
                        let post_now = drv.clock_now();
                        let frames_dispatched = drv.disable_frames_dispatched();
                        let dispatched_mask = drv.disable_dispatched_mask();
                        let mut message = crate::safety::BoundedText::empty();
                        message.push_str("Torque disable transmission failed: ");
                        e.write_bounded(&mut message);
                        #[cfg(not(target_arch = "wasm32"))]
                        self.torque_shadow.drain_from_driver(&mut **drv);
                        drop(drv);
                        let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                        let partial_retry = self.commit_partial_disable_retry_kind(
                            kind,
                            post_now,
                            frames_dispatched,
                            partial_remaining,
                        );
                        let code = if let Some(origin) = pending_disable_origin {
                            self.torque_transition = None;
                            let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                            match origin {
                                DisableOrigin::BrokerTransition => {
                                    self.core.begin_transition(Err(code));
                                }
                                DisableOrigin::Park { watchdog, action_id, .. } => {
                                    self.finish_park_disable_failed(
                                        watchdog,
                                        action_id,
                                        code,
                                        "Torque disable transmission failed",
                                    );
                                }
                                DisableOrigin::LegacyWs { token, slot, .. } => {
                                    self.settle_legacy_ws_disable_failed(
                                        token,
                                        slot,
                                        code,
                                        Some(message),
                                    );
                                }
                                DisableOrigin::Detached { .. } => {}
                            }
                            code
                        } else {
                            if refused && partial_retry.is_none() {
                                self.disable_pending = false;
                                self.active_disable_kind = None;
                                self.last_disable_attempt = None;
                                self.background_disable_retry = None;
                            }
                            crate::driver::handshake::rt_progress(false, Err(e))
                                .err()
                                .unwrap_or("HARDWARE_FAULT")
                        };
                        return Err(RequestDisableError {
                            code,
                            message,
                            refused_by_chokepoint: refused && !frames_dispatched,
                        });
                    }
                }
            } else if let Err(e) = drv.disable_torque(kind) {
                let refused = matches!(
                    e,
                    crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired
                );
                let post_now = drv.clock_now();
                let frames_dispatched = drv.disable_frames_dispatched();
                let dispatched_mask = drv.disable_dispatched_mask();
                let mut message = crate::safety::BoundedText::empty();
                message.push_str("Torque disable transmission failed: ");
                e.write_bounded(&mut message);
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow.drain_from_driver(&mut **drv);
                drop(drv);
                let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                let partial_retry = self.commit_partial_disable_retry_kind(
                    kind,
                    post_now,
                    frames_dispatched,
                    partial_remaining,
                );
                let code = if let Some(origin) = pending_disable_origin {
                    self.torque_transition = None;
                    let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                    match origin {
                        DisableOrigin::BrokerTransition => {
                            self.core.begin_transition(Err(code));
                        }
                        DisableOrigin::Park { watchdog, action_id, .. } => {
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                code,
                                "Torque disable transmission failed",
                            );
                        }
                        DisableOrigin::LegacyWs { token, slot, .. } => {
                            self.settle_legacy_ws_disable_failed(
                                token,
                                slot,
                                code,
                                Some(message),
                            );
                        }
                        DisableOrigin::Detached { .. } => {}
                    }
                    code
                } else {
                    if refused && partial_retry.is_none() {
                        self.disable_pending = false;
                        self.active_disable_kind = None;
                        self.last_disable_attempt = None;
                        self.background_disable_retry = None;
                    }
                    crate::driver::handshake::rt_progress(false, Err(e))
                        .err()
                        .unwrap_or("HARDWARE_FAULT")
                };
                return Err(RequestDisableError {
                    code,
                    message,
                    refused_by_chokepoint: refused && !frames_dispatched,
                });
            }
            let mut ages_buf = std::mem::take(&mut self.scratch.ages);
            let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
            self.owner_driver_snapshot = OwnerDriverSnapshot::capture_from_driver(
                drv.as_ref(),
                self.config.dof,
                self.feedback_numeric_valid,
                &self.measured_positions,
                &mut ages_buf,
                &mut bounds_buf,
            );
            self.scratch.ages = ages_buf;
            self.scratch.fb.bounds = bounds_buf;
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.drain_from_driver(&mut **drv);
            drop(drv);
        }
        self.disable_accepted();
        if let Some(origin) = pending_disable_origin {
            self.torque_transition = None;
            match origin {
                DisableOrigin::BrokerTransition => {
                    let terminal = self
                        .torque_off_finish_in_place()
                        .map(|()| Rt::Done)
                        .map_err(|e| torque_error_code(&e));
                    self.core.begin_transition(terminal);
                }
                DisableOrigin::Park { watchdog, action_id, .. } => {
                    self.finish_park_disable_after_accepted(watchdog, action_id);
                }
                DisableOrigin::LegacyWs { token, slot, .. } => {
                    match self.torque_off_finish_rt() {
                        Ok(()) => self.settle_legacy_ws_accepted(token, slot),
                        Err(code) => {
                            self.settle_legacy_ws_disable_failed(token, slot, code, None)
                        }
                    }
                }
                DisableOrigin::Detached { .. } => {
                    let _ = self.torque_off_finish_rt();
                }
            }
        }
        Ok(())
    }

    /// A successful transport write is not confirmation that torque was removed.
    pub(in crate::supervisor) fn request_disable(&mut self, kind: crate::driver::DisableKind) -> Result<(), SafetyError> {
        self.request_disable_bounded(kind).map_err(|e| {
            if e.code == "FEEDBACK_UNAVAILABLE" {
                SafetyError::FeedbackUnavailable
            } else if e.code == "TORQUE_OFF_WOULD_DROP" {
                let r = self
                    .may_disable_torque_while_stopped()
                    .err()
                    .unwrap_or(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback);
                self.last_stopped_torque_off_refusal = Some(r);
                r.into_safety_error(&self.config.joint_names)
            } else {
                SafetyError::ClearFaultRejected(e.message.as_str().to_string())
            }
        })
    }

    /// State when a disable request is in flight: `disable_pending` blocks new motion and
    /// enable/clear-fault transitions, while holds and MIT hold context stay active until the
    /// rest gate passes and the first disable frame is actually dispatched.
    pub(in crate::supervisor) fn disable_requested(&mut self) {
        self.disable_requested_at(Instant::now());
    }

    pub(in crate::supervisor) fn disable_requested_at(&mut self, now: Instant) {
        self.disable_pending = true;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        self.last_disable_attempt = Some(now);
    }

    /// State once the rest-pose gate has passed and the first disable frame has been dispatched
    /// (while bounded repeat frames are still finishing): records `torque_intent = Off` and stops
    /// holds while keeping `disable_pending = true` until the handshake completes.
    pub(crate) fn disable_first_frame_accepted(&mut self) {
        self.mit_torque_off();
        self.torque_intent = TorqueIntent::Off;
        self.measured_hold.clear();
    }

    pub(crate) fn reset_reported_torque_loss(&mut self) {
        self.reported_torque_loss_count = 0;
        self.reported_torque_loss_last_release = 0;
        self.reported_torque_loss_last_sample_seq = None;
        self.reported_torque_loss_from_motion = false;
    }

    /// State once an explicit disable (passing the rest-pose gate) or protective disable has been
    /// accepted and transmitted: records `torque_intent = Off` by decision and stops holding.
    pub(crate) fn disable_accepted(&mut self) {
        self.disable_accepted_inner(true);
    }

    pub(crate) fn disable_accepted_inner(&mut self, emit_mit_shadow: bool) {
        self.mit_torque_off_inner(emit_mit_shadow);
        self.torque_intent = TorqueIntent::Off;
        self.is_torqued = false;
        self.pending_rearm = false;
        self.disable_pending = false;
        self.active_disable_kind = None;
        self.active_disable_client_park = None;
        self.background_disable_retry = None;
        self.gripper_hold_vel_ceiling = None;
        self.enable_lifecycle = EnableLifecycle::Disarmed;
        self.measured_hold.clear();
        self.reset_reported_torque_loss();
        self.last_poll_enable_state = EnableState::Disabled;
        self.last_poll_partial_enable = false;
    }
}
