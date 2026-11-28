//! Legacy WS torque slot tracking, cancellation, and expiry.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

impl SupervisorInner {
    pub(in crate::supervisor) fn reserve_legacy_torque_slot(
        &mut self,
    ) -> Option<(u64, u8, std::sync::Arc<LegacyTorqueSlot>)> {
        let token = self.legacy_torque_seq.wrapping_add(1).max(1);
        self.legacy_torque_seq = token;
        for (idx, slot) in self.legacy_torque_slots.iter().enumerate() {
            if slot.try_reserve(token) {
                return Some((token, idx as u8, std::sync::Arc::clone(slot)));
            }
        }
        None
    }

    pub(in crate::supervisor) fn settle_legacy_ws_accepted(&mut self, token: u64, slot: u8) {
        let epoch_id = self.safety.epoch_id();
        let safety_state = self.safety.state_arc();
        let is_torqued = self.is_torqued;
        let control_mode_gravity_comp = self.gravity_comp_active();
        if let Some(s) = self.legacy_torque_slots.get(usize::from(slot)) {
            if s.settle_with(token, || LegacyTorqueOutcome::Accepted {
                epoch_id,
                safety_state,
                is_torqued,
                control_mode_gravity_comp,
            }) {
                self.legacy_torque_settled_this_tick = true;
            }
        }
    }

    pub(crate) fn settle_legacy_ws_failed(&mut self, token: u64, slot: u8, failure: LegacyTorqueFailure) {
        let epoch_id = self.safety.epoch_id();
        let safety_state = self.safety.state_arc();
        if let Some(s) = self.legacy_torque_slots.get(usize::from(slot)) {
            if s.settle_with(token, || LegacyTorqueOutcome::Failed {
                failure,
                epoch_id,
                safety_state,
            }) {
                self.legacy_torque_settled_this_tick = true;
            }
        }
    }

    /// Cancels an in-flight `EnableOrigin::LegacyWs` on client Stop without latching a
    /// non-recoverable `EnableFailed` fault (unless E-stop is also pending/latched, in which case
    /// E-stop takes precedence). Returns `true` when E-stop won.
    pub(in crate::supervisor) fn cancel_legacy_enable_on_stop(
        &mut self,
        token: u64,
        slot: u8,
        was_torqued: bool,
        prev_torque_intent: TorqueIntent,
    ) -> bool {
        if self.estop_pending()
            || self.e_stop_latched.load(Ordering::SeqCst)
            || matches!(self.safety.state(), SafetyState::EStop)
        {
            let _ = self.enter_failed_hold(was_torqued, |buf| {
                buf.push_str("interrupted by E-stop");
            });
            self.settle_legacy_ws_failed(token, slot, LegacyTorqueFailure::EStopActive);
            return true;
        }
        let (drv_any_enabled, contended) = self.abort_enable_driver_nonblocking(was_torqued, true);
        if contended && !was_torqued {
            self.detached_cancel_restore_intent = Some(prev_torque_intent);
        } else {
            self.detached_cancel_restore_intent = None;
        }
        let any_energized = was_torqued || drv_any_enabled;
        if !any_energized {
            self.enable_lifecycle = EnableLifecycle::Disarmed;
            self.is_torqued = false;
            self.torque_intent = prev_torque_intent;
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
        } else {
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
            // Use FailedHold (without latching EnableFailed) so tick_control always holds
            // partially enabled / interrupted-rearm motors via service_stopped_measured_hold
            // rather than ordinary dynamic output, while allowing immediate operator recovery.
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
        }
        self.settle_legacy_ws_failed(token, slot, LegacyTorqueFailure::Stopped);
        false
    }

    /// Immediately aborts an in-flight `EnableOrigin::LegacyWs` when `authorized_stop` runs.
    pub(crate) fn abort_legacy_enable_on_authorized_stop(&mut self) {
        let is_legacy_enable = matches!(
            self.torque_transition,
            Some(TorqueTransition::Enable(EnableReducer {
                origin: EnableOrigin::LegacyWs { .. },
                ..
            }))
        );
        if !is_legacy_enable {
            return;
        }
        let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() else {
            return;
        };
        if let EnableOrigin::LegacyWs {
            token,
            slot,
            prev_torque_intent,
            ..
        } = reducer.origin
        {
            let _ = self.cancel_legacy_enable_on_stop(
                token,
                slot,
                reducer.was_torqued,
                prev_torque_intent,
            );
        }
    }

    /// Starts a bounded, ticked legacy WS torque-enable transition under a short write guard
    /// without blocking or sleeping on CAN/serial confirmation (consumes A1's [`Admitted`] by value).
    pub(crate) fn begin_enable_torque_transition_legacy(
        admitted: Admitted<'_>,
        deadline: Instant,
    ) -> Result<LegacyTorqueStart, SafetyError> {
        let (this, proof) = admitted.consume()?;
        if this.calibration_busy && this.motor_zero_busy {
            return Err(SafetyError::MotionBusy);
        }
        if this.torque_transition.is_some() {
            return Err(SafetyError::ClearFaultRejected(
                "A torque transition is pending".into(),
            ));
        }
        this.gravity_comp_servo_busy()?;
        let (was_torqued, prev_gripper_cmd) = this.enable_prepare_inner(false, true, true)?;
        let mit_qualified = this.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
        let Some((token, slot_idx, slot)) = this.reserve_legacy_torque_slot() else {
            return Err(SafetyError::ClearFaultRejected(
                "A torque transition is pending".into(),
            ));
        };
        let s = proof.stamp();
        let authority = EnableAuthority {
            lease_generation: s.lease_gen,
            owner_conn_id: s.connection_id,
            admitted_epoch: s.estop_epoch,
            admission_request_id: s.request_id,
            stop_gen: s.stop_gen,
        };
        let prev_torque_intent = this.torque_intent;
        let budgeted = this.driver.is_none() || this.driver_supports_rt_executor_nonblocking();
        let origin = EnableOrigin::LegacyWs {
            token,
            slot: slot_idx,
            stop_gen: s.stop_gen,
            prev_torque_intent,
            deadline,
        };
        match this.start_enable_reducer_inner(
            origin,
            was_torqued,
            prev_gripper_cmd,
            Some(authority),
            budgeted,
            budgeted,
            mit_qualified,
        ) {
            EnableStepOutcome::Armed(state) => {
                slot.release_unstarted(token);
                Ok(LegacyTorqueStart::ImmediateAccepted {
                    epoch_id: this.safety.epoch_id(),
                    safety_state: state,
                    is_torqued: this.is_torqued,
                    control_mode_gravity_comp: this.gravity_comp_active(),
                })
            }
            EnableStepOutcome::Pending { .. } => Ok(LegacyTorqueStart::Pending {
                token,
                slot_idx,
                slot,
            }),
            EnableStepOutcome::Failed {
                rt_code,
                message,
                feedback_unavailable,
                estop_interrupted,
            } => {
                slot.release_unstarted(token);
                Ok(LegacyTorqueStart::ImmediateRejected {
                    failure: Self::classify_legacy_enable_failure(
                        rt_code,
                        message,
                        feedback_unavailable,
                        estop_interrupted,
                    ),
                    epoch_id: this.safety.epoch_id(),
                    safety_state: this.safety.state().clone(),
                })
            }
        }
    }

    /// Non-blocking torque-off preflight for legacy WS admission (never blocks on `driver.lock()`).
    pub(in crate::supervisor) fn torque_off_check_legacy_nonblocking(&mut self) -> Result<Result<(), SafetyError>, ()> {
        if !self.torque_off_stopped() {
            if let Some(reason) = self.resting_pose_violation() {
                tracing::warn!("Torque off refused, not at resting pose: {reason}");
                return Ok(Err(SafetyError::ArmNotAtRestingPose {
                    current: self.measured_positions.clone(),
                    expected: self.config.resting_positions().to_vec(),
                    reason: Some(reason),
                }));
            }
        }
        let stopped_res = match self.may_disable_torque_while_stopped_nonblocking() {
            Ok(r) => r,
            Err(()) => return Err(()),
        };
        match stopped_res {
            Ok(()) => {
                self.last_stopped_torque_off_refusal = None;
                Ok(Ok(()))
            }
            Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback) => {
                match self.can_defer_stopped_disable_to_driver_nonblocking() {
                    Ok(true) => {
                        self.last_stopped_torque_off_refusal = None;
                        Ok(Ok(()))
                    }
                    Ok(false) => {
                        let r = crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback;
                        self.last_stopped_torque_off_refusal = Some(r);
                        Ok(Err(r.into_safety_error(&self.config.joint_names)))
                    }
                    Err(()) => Err(()),
                }
            }
            Err(r) => {
                self.last_stopped_torque_off_refusal = Some(r);
                Ok(Err(r.into_safety_error(&self.config.joint_names)))
            }
        }
    }

    /// Starts a bounded, ticked legacy WS torque-disable transition under a short write guard
    /// without blocking on rest-gate probes or disable repeat sleeps. Exempt from `Admitted`.
    pub(crate) fn begin_disable_torque_transition_legacy(
        &mut self,
        deadline: Instant,
    ) -> Result<LegacyTorqueStart, SafetyError> {
        if Instant::now() >= deadline {
            return Ok(LegacyTorqueStart::ImmediateRejected {
                failure: LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state().clone(),
            });
        }
        if self.calibration_busy && self.motor_zero_busy {
            return Err(SafetyError::MotionBusy);
        }
        if self.torque_transition.is_some()
            || matches!(
                self.active_disable_kind,
                Some(crate::driver::DisableKind::DisableProtective { .. })
            )
            || matches!(
                self.background_disable_retry,
                Some(BackgroundDisableRetry::Active { .. })
            )
        {
            return Err(SafetyError::ClearFaultRejected(
                "A torque transition is pending".into(),
            ));
        }
        self.gravity_comp_servo_busy()?;
        match self.torque_off_check_legacy_nonblocking() {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let live_op_id = self.torque_shadow.alloc_live_op_id();
                    let live_refusal = match &e {
                        SafetyError::TorqueOffWouldDrop { .. }
                        | SafetyError::ArmNotAtRestingPose { .. } => {
                            crate::torque::shadow::LiveRefusalKind::TorqueOffWouldDrop
                        }
                        SafetyError::MotionBusy | SafetyError::ClearFaultRejected(_) => {
                            crate::torque::shadow::LiveRefusalKind::BusyOrTransition
                        }
                        _ => crate::torque::shadow::LiveRefusalKind::Other,
                    };
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: Some(live_refusal),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        },
                    );
                    if !self.rt_executor_owned {
                        self.flush_torque_shadow();
                    }
                }
                return Err(e);
            }
            Err(()) => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let live_op_id = self.torque_shadow.alloc_live_op_id();
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: Some(crate::torque::shadow::LiveRefusalKind::DriverBusy),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        },
                    );
                    if !self.rt_executor_owned {
                        self.flush_torque_shadow();
                    }
                }
                self.record_disable_lock_contention();
                return Ok(LegacyTorqueStart::ImmediateRejected {
                    failure: LegacyTorqueFailure::StaticCode("DRIVER_BUSY"),
                    epoch_id: self.safety.epoch_id(),
                    safety_state: self.safety.state().clone(),
                });
            }
        }

        #[cfg(not(target_arch = "wasm32"))]
        let live_op_id = self.torque_shadow.alloc_live_op_id();
        if self.driver.is_none() {
            #[cfg(not(target_arch = "wasm32"))]
            {
                self.torque_shadow
                    .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                        live_op_id,
                        explicit_release_authorized: false,
                        live_refusal: None,
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    });
                if !self.rt_executor_owned {
                    self.flush_torque_shadow();
                }
            }
            self.stop_motion();
            self.commanded_velocities.fill(0.0);
            self.background_disable_retry = None;
            self.disable_requested();
            self.disable_accepted();
            self.torque_off_finish_in_place()?;
            return Ok(LegacyTorqueStart::ImmediateAccepted {
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state().clone(),
                is_torqued: self.is_torqued,
                control_mode_gravity_comp: self.gravity_comp_active(),
            });
        }

        let Some((token, slot_idx, slot)) = self.reserve_legacy_torque_slot() else {
            return Err(SafetyError::ClearFaultRejected(
                "A torque transition is pending".into(),
            ));
        };
        let now_wall = Instant::now();
        if now_wall >= deadline {
            slot.release_unstarted(token);
            return Ok(LegacyTorqueStart::ImmediateRejected {
                failure: LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state().clone(),
            });
        }

        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let measured_positions = &self.measured_positions;
        let period = Duration::from_secs_f64(
            1.0 / f64::from(self.config.control_frequency_hz.max(1)),
        );
        let per_tick_tx_budget = period.saturating_sub(self.tx_release_publish_margin());
        let active_lease_gen = (self.lease_enabled && !self.safety.state().is_stopped())
            .then(|| self.current_active_lease_gen())
            .filter(|&g| g > 0);
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let locked = {
            #[cfg(not(target_arch = "wasm32"))]
            let live_intent = self.shadow_live_intent();
            #[cfg(not(target_arch = "wasm32"))]
            let live_is_torqued = self.is_torqued;
            #[cfg(not(target_arch = "wasm32"))]
            let live_stopped = self.safety.state().is_stopped();
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let d = self.driver.as_mut().expect("checked is_some");
            match d.try_with_mut(|drv| {
                if drv.disable_frames_dispatched() || drv.torque_transition_pending() {
                    Err(2u8)
                } else if Instant::now() >= deadline {
                    Err(3u8)
                } else {
                    let requested_at = drv.clock_now();
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.enqueue_at(
                        requested_at,
                        crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: None,
                            live_intent,
                            live_is_torqued,
                            live_stopped,
                        },
                    );
                    let disable_kind = crate::driver::DisableKind::DisableRequested {
                        started_at: requested_at,
                    };
                    let req_rem = deadline.saturating_duration_since(Instant::now());
                    let step_budget = req_rem.min(per_tick_tx_budget);
                    let use_budgeted = drv.supports_rt_executor().is_ok();
                    drv.tx_mut().set_disable_authority(Some(requested_at + req_rem), active_lease_gen);
                    drv.set_tx_release_deadline(Some(requested_at + step_budget));
                    let res = if use_budgeted {
                        drv.begin_disable_torque_budgeted(disable_kind, 0)
                    } else {
                        drv.begin_disable_torque(disable_kind)
                    };
                    drv.set_tx_release_deadline(None);
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
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    Ok((
                        res,
                        requested_at,
                        post_now,
                        disable_kind,
                        frames_dispatched,
                        dispatched_mask,
                        snap,
                    ))
                }
            }) {
                Ok(inner_res) => inner_res,
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(1u8),
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(0u8),
            }
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;

        let (res, requested_at, post_now, disable_kind, frames_dispatched, dispatched_mask, snap) =
            match locked {
                Ok(v) => {
                    self.reset_disable_lock_contention();
                    #[cfg(not(target_arch = "wasm32"))]
                    if !self.rt_executor_owned {
                        self.flush_torque_shadow();
                    }
                    v
                }
                Err(3u8) => {
                    slot.release_unstarted(token);
                    return Ok(LegacyTorqueStart::ImmediateRejected {
                        failure: LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                        epoch_id: self.safety.epoch_id(),
                        safety_state: self.safety.state().clone(),
                    });
                }
                Err(2u8) => {
                    slot.release_unstarted(token);
                    return Err(SafetyError::ClearFaultRejected(
                        "A torque transition is pending".into(),
                    ));
                }
                Err(1u8) => {
                    slot.release_unstarted(token);
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        self.torque_shadow.enqueue(
                            crate::torque::shadow::ShadowInputKind::DisableRequest {
                                live_op_id,
                                explicit_release_authorized: false,
                                live_refusal: Some(
                                    crate::torque::shadow::LiveRefusalKind::DriverBusy,
                                ),
                                live_intent: self.shadow_live_intent(),
                                live_is_torqued: self.is_torqued,
                                live_stopped: self.safety.state().is_stopped(),
                            },
                        );
                        if !self.rt_executor_owned {
                            self.flush_torque_shadow();
                        }
                    }
                    self.record_disable_lock_contention();
                    return Ok(LegacyTorqueStart::ImmediateRejected {
                        failure: LegacyTorqueFailure::StaticCode("DRIVER_BUSY"),
                        epoch_id: self.safety.epoch_id(),
                        safety_state: self.safety.state().clone(),
                    });
                }
                Err(_) => {
                    slot.release_unstarted(token);
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        self.torque_shadow.enqueue(
                            crate::torque::shadow::ShadowInputKind::DisableRequest {
                                live_op_id,
                                explicit_release_authorized: false,
                                live_refusal: Some(crate::torque::shadow::LiveRefusalKind::Other),
                                live_intent: self.shadow_live_intent(),
                                live_is_torqued: self.is_torqued,
                                live_stopped: self.safety.state().is_stopped(),
                            },
                        );
                        if !self.rt_executor_owned {
                            self.flush_torque_shadow();
                        }
                    }
                    return Err(SafetyError::FeedbackUnavailable);
                }
            };

        self.owner_driver_snapshot = snap;
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        self.background_disable_retry = None;
        self.disable_requested_at(requested_at);
        self.active_disable_kind = Some(disable_kind);
        self.active_disable_client_park = None;

        match res {
            Ok(crate::driver::TorqueProgress::Done(_)) => {
                slot.release_unstarted(token);
                self.disable_accepted();
                self.torque_off_finish_in_place()?;
                Ok(LegacyTorqueStart::ImmediateAccepted {
                    epoch_id: self.safety.epoch_id(),
                    safety_state: self.safety.state().clone(),
                    is_torqued: self.is_torqued,
                    control_mode_gravity_comp: self.gravity_comp_active(),
                })
            }
            Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                    self.disable_first_frame_accepted();
                }
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::LegacyWs {
                        token,
                        slot: slot_idx,
                        deadline,
                    },
                    kind: disable_kind,
                });
                Ok(LegacyTorqueStart::Pending {
                    token,
                    slot_idx,
                    slot,
                })
            }
            Err(e @ crate::driver::ArmDriverError::PartialDisable { .. }) => {
                // Never return ImmediateRejected once any disable frame was dispatched:
                // settle the slot with a single terminal fault detailing applied/uncertain/not attempted.
                let (err_detail, _) = self.handle_partial_disable_abort(e);
                self.settle_legacy_ws_failed(
                    token,
                    slot_idx,
                    LegacyTorqueFailure::DisableFailed { detail: err_detail },
                );
                Ok(LegacyTorqueStart::Pending {
                    token,
                    slot_idx,
                    slot,
                })
            }
            Err(e) => {
                let mut err_detail =
                    crate::safety::BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::empty();
                err_detail.push_str("Torque disable transmission failed: ");
                e.write_bounded(&mut err_detail);
                if frames_dispatched {
                    let refused = matches!(
                        e,
                        crate::driver::ArmDriverError::TorqueOffWouldDrop
                            | crate::driver::ArmDriverError::DisableRequestExpired
                    );
                    let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                    let partial_retry = self.commit_partial_disable_retry_kind(
                        disable_kind,
                        post_now,
                        frames_dispatched,
                        partial_remaining,
                    );
                    if let Some(rk) = partial_retry {
                        let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                    } else if !refused {
                        let _ = self.arm_background_disable_retry_with_kind(disable_kind, post_now);
                    }
                    let _ = self.disable_step_failed_inner(e, partial_retry.is_some());
                    self.settle_legacy_ws_failed(
                        token,
                        slot_idx,
                        LegacyTorqueFailure::DisableFailed { detail: err_detail },
                    );
                    return Ok(LegacyTorqueStart::Pending {
                        token,
                        slot_idx,
                        slot,
                    });
                }
                slot.release_unstarted(token);
                self.clear_active_disable_retry_state();
                let code = self.disable_step_failed_inner(e, false);
                let failure = match code {
                    "TORQUE_OFF_WOULD_DROP" => {
                        let refusal = self
                            .may_disable_torque_while_stopped_nonblocking()
                            .ok()
                            .and_then(Result::err)
                            .or(self.last_stopped_torque_off_refusal)
                            .unwrap_or(
                                crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback,
                            );
                        self.last_stopped_torque_off_refusal = Some(refusal);
                        LegacyTorqueFailure::TorqueOffWouldDrop(refusal)
                    }
                    "ESTOP_ACTIVE" => LegacyTorqueFailure::EStopActive,
                    "LEASE_LOST" => LegacyTorqueFailure::LeaseLost,
                    "FEEDBACK_UNAVAILABLE" => LegacyTorqueFailure::FeedbackUnavailable,
                    _ => LegacyTorqueFailure::DisableFailed { detail: err_detail },
                };
                Ok(LegacyTorqueStart::ImmediateRejected {
                    failure,
                    epoch_id: self.safety.epoch_id(),
                    safety_state: self.safety.state().clone(),
                })
            }
        }
    }

    /// Single locked decision for an expired non-protective disable (`LegacyWs` or `Detached`):
    /// - Protective disable -> never aborted (`KeepDetachedDispatched`).
    /// - Missing driver or poisoned lock -> clears transition + retry state (`Aborted`).
    /// - Contended `try_with_mut` -> keeps `Detached { deadline: Some(dl) }` pending without clearing
    ///   retry state (`ContendedRetry`).
    /// - Acquired single driver borrow -> clears `deadline` ONLY when `disable_frames_dispatched()`
    ///   proves at least one disable frame was already sent; otherwise attempts
    ///   `abort_pending_disable()` and keeps `deadline` intact if abort is refused.
    pub(super) fn try_abort_expired_undispatched_disable(
        &mut self,
        kind: crate::driver::DisableKind,
        deadline: Option<Instant>,
    ) -> DisableExpiryDecision {
        let is_protective =
            matches!(kind, crate::driver::DisableKind::DisableProtective { .. });
        if is_protective {
            self.torque_transition = Some(TorqueTransition::Disable {
                origin: DisableOrigin::Detached { deadline: None },
                kind,
            });
            return DisableExpiryDecision::KeepDetachedDispatched;
        }
        let Some(ref mut d) = self.driver else {
            self.clear_active_disable_retry_state();
            return DisableExpiryDecision::Aborted;
        };
        let decision = match d.try_with_mut(|drv| {
            if drv.disable_frames_dispatched() {
                DisableExpiryDecision::KeepDetachedDispatched
            } else if drv.abort_pending_disable() {
                DisableExpiryDecision::Aborted
            } else {
                // Driver refused abort without proving dispatch; keep deadline intact so no first
                // disable frame can ever transmit after request expiry.
                DisableExpiryDecision::KeepDetachedWithDeadline
            }
        }) {
            Ok(dec) => dec,
            Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                DisableExpiryDecision::ContendedRetry
            }
            Err(crate::supervisor_lock::DriverTryError::Poisoned) => DisableExpiryDecision::Aborted,
        };
        match decision {
            DisableExpiryDecision::Aborted => self.clear_active_disable_retry_state(),
            DisableExpiryDecision::ContendedRetry
            | DisableExpiryDecision::KeepDetachedWithDeadline => {
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::Detached { deadline },
                    kind,
                });
            }
            DisableExpiryDecision::KeepDetachedDispatched => {
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::Detached { deadline: None },
                    kind,
                });
            }
        }
        self.sync_detached_torque_flag();
        decision
    }

    /// Resolves an expired legacy WS torque transition under the owner write guard.
    /// - When `settle_slot` is `false` (waiter-driven timeout): checks for an already-settled
    ///   outcome first, releases the slot, cleans up/detaches the transition, and returns the
    ///   terminal outcome directly to the waiter.
    /// - When `settle_slot` is `true` (control-tick authoritative deadline fence): cleans up/detaches
    ///   the transition and publishes the terminal outcome into the waiter slot so the waiter wakes.
    pub(in crate::supervisor) fn expire_legacy_torque_transition_inner(
        &mut self,
        token: u64,
        slot_idx: u8,
        enabled: bool,
        settle_slot: bool,
    ) -> LegacyTorqueOutcome {
        if !settle_slot {
            if let Some(settled) = self
                .legacy_torque_slots
                .get(usize::from(slot_idx))
                .and_then(|s| s.take_settled(token))
            {
                return settled;
            }
            if let Some(s) = self.legacy_torque_slots.get(usize::from(slot_idx)) {
                s.release_unstarted(token);
            }
        }
        if enabled {
            let matches_ours = matches!(
                self.torque_transition,
                Some(TorqueTransition::Enable(EnableReducer {
                    origin: EnableOrigin::LegacyWs { token: t, .. },
                    ..
                })) if t == token
            );
            if matches_ours {
                if let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() {
                    let prev_torque_intent = match reducer.origin {
                        EnableOrigin::LegacyWs { prev_torque_intent, .. } => prev_torque_intent,
                        _ => TorqueIntent::Off,
                    };
                    let (drv_any_enabled, contended) =
                        self.abort_enable_driver_nonblocking(reducer.was_torqued, false);
                    if !reducer.was_torqued && !drv_any_enabled {
                        self.enable_lifecycle = EnableLifecycle::Disarmed;
                        self.is_torqued = false;
                        self.torque_intent = prev_torque_intent;
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
                    } else {
                        let _ = self.enter_failed_hold(reducer.was_torqued, |buf| {
                            buf.push_str("enable transition timed out");
                        });
                    }
                    if contended {
                        self.retain_detached_enable_cancel_if_clear(reducer.was_torqued);
                    }
                }
            }
            let is_estop = self.estop_pending()
                || self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop);
            let failure = if is_estop {
                LegacyTorqueFailure::EStopActive
            } else {
                LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED")
            };
            if settle_slot {
                self.settle_legacy_ws_failed(token, slot_idx, failure);
            }
            LegacyTorqueOutcome::Failed {
                failure,
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state_arc(),
            }
        } else {
            let matches_ours = matches!(
                self.torque_transition,
                Some(TorqueTransition::Disable {
                    origin: DisableOrigin::LegacyWs { token: t, .. },
                    ..
                }) if t == token
            );
            let mut aborted_before_dispatch = false;
            if matches_ours {
                if let Some(TorqueTransition::Disable {
                    origin: DisableOrigin::LegacyWs { deadline, .. },
                    kind,
                }) = self.torque_transition.take()
                {
                    if matches!(
                        self.try_abort_expired_undispatched_disable(kind, Some(deadline)),
                        DisableExpiryDecision::Aborted
                    ) {
                        aborted_before_dispatch = true;
                    }
                }
            }
            let failure = if aborted_before_dispatch {
                LegacyTorqueFailure::TorqueOffWouldDrop(
                    crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback,
                )
            } else {
                LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED")
            };
            if settle_slot {
                self.settle_legacy_ws_failed(token, slot_idx, failure);
            }
            LegacyTorqueOutcome::Failed {
                failure,
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state_arc(),
            }
        }
    }

    /// Resolves a waiter deadline atomically under the owner write guard against any concurrent
    /// completion. Never cancels a protective disable or an already-dispatched disable.
    pub(crate) fn timeout_legacy_torque_transition(
        &mut self,
        token: u64,
        slot_idx: u8,
        enabled: bool,
    ) -> LegacyTorqueOutcome {
        self.expire_legacy_torque_transition_inner(token, slot_idx, enabled, false)
    }
}
