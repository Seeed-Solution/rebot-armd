//! Park-disable transition initiation, completion, and partial-disable handling.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

// scaffold-extra-0
impl SupervisorInner {
    /// Single park terminal torque-off function routed strictly through the torque-off rest-pose
    /// gate (`may_disable_torque_while_stopped_nonblocking` / `eval_stopped_torque_off` + driver
    /// `DisableRequested` K=3 rest-pose gate) and the single `TorqueTransition::Disable` authority
    /// (zero-alloc on RT). Client Stop is linearized against every motor's disable transmit at the
    /// driver boundary; watchdog park is exempt from client Stop while still rest-pose gated.
    pub(crate) fn begin_park_disable_transition(&mut self, watchdog: bool, action_id: Option<u64>) {
        if !watchdog && self.urgent_stop_unapplied() {
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                    live_op_id: action_id.unwrap_or(0),
                    admitted: false,
                    settled_ok: Some(false),
                });
            self.cancel_park_before_disable(action_id);
            return;
        }
        if self.torque_transition.is_some()
            || self.safety.state().is_stopped()
            || self.e_stop_latched.load(Ordering::SeqCst)
            || self.estop_pending()
        {
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                    live_op_id: action_id.unwrap_or(0),
                    admitted: false,
                    settled_ok: Some(false),
                });
            self.finish_park_disable_failed(
                watchdog,
                action_id,
                "UNLOAD_FAILED",
                "Park torque-off refused at resting pose",
            );
            return;
        }
        // Route park disable through the torque-off rest-pose gate (both RT non-blocking and legacy).
        if self.rt_executor_owned {
            match self.may_disable_torque_while_stopped_nonblocking() {
                Err(()) => {
                    self.record_disable_lock_contention();
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "DRIVER_BUSY",
                        "Driver busy during park torque-off",
                    );
                    return;
                }
                Ok(Ok(())) => {
                    self.last_stopped_torque_off_refusal = None;
                }
                Ok(Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)) => {
                    match self.can_defer_stopped_disable_to_driver_nonblocking() {
                        Err(()) => {
                            self.record_disable_lock_contention();
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                "DRIVER_BUSY",
                                "Driver busy during park torque-off",
                            );
                            return;
                        }
                        Ok(true) => {
                            self.last_stopped_torque_off_refusal = None;
                        }
                        Ok(false) => {
                            self.last_stopped_torque_off_refusal = Some(
                                crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback,
                            );
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                "TORQUE_OFF_WOULD_DROP",
                                "Park torque-off refused by rest-pose gate",
                            );
                            return;
                        }
                    }
                }
                Ok(Err(r)) => {
                    self.last_stopped_torque_off_refusal = Some(r);
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "TORQUE_OFF_WOULD_DROP",
                        "Park torque-off refused by rest-pose gate",
                    );
                    return;
                }
            }
            match self.check_resting_pose_rt(true) {
                super::RtRestingCheck::AtRest => {}
                super::RtRestingCheck::OffRest => {
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "ARM_NOT_AT_RESTING_POSE",
                        "Park torque-off refused at resting pose",
                    );
                    return;
                }
                super::RtRestingCheck::LockContended => {
                    self.record_disable_lock_contention();
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "DRIVER_BUSY",
                        "Driver busy during park torque-off",
                    );
                    return;
                }
            }
        } else {
            match self.eval_stopped_torque_off() {
                Ok(()) => {}
                Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)
                    if self.can_defer_stopped_disable_to_driver() =>
                {
                    self.last_stopped_torque_off_refusal = None;
                }
                Err(_) => {
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "TORQUE_OFF_WOULD_DROP",
                        "Park torque-off refused by rest-pose gate",
                    );
                    return;
                }
            }
            if !self.is_resting_pose_static_ok() {
                self.finish_park_disable_failed(
                    watchdog,
                    action_id,
                    "ARM_NOT_AT_RESTING_POSE",
                    "Park torque-off refused at resting pose",
                );
                return;
            }
        }
        if self.driver.is_none() {
            if !watchdog && self.urgent_stop_unapplied() {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                            live_op_id: action_id.unwrap_or(0),
                            admitted: false,
                            settled_ok: Some(false),
                        },
                    );
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::Stop {
                            kind: crate::torque::StopKind::Stop,
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: true,
                        },
                    );
                }
                self.cancel_park_before_disable(action_id);
                return;
            }
            #[cfg(not(target_arch = "wasm32"))]
            {
                let live_op_id = action_id
                    .unwrap_or_else(|| self.torque_shadow.alloc_live_op_id());
                self.torque_shadow.enqueue(
                    crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                        live_op_id,
                        admitted: true,
                        settled_ok: None,
                    },
                );
                self.torque_shadow.enqueue(
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
            self.retire_motion_payloads();
            self.pending_deferred_done = None;
            self.end_gripper_stream();
            crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
            self.commanded_velocities.fill(0.0);
            self.disable_requested();
            self.complete_park_disable_accepted(watchdog, action_id);
            return;
        }
        #[allow(clippy::large_enum_variant)] // inline zero-alloc RT stack enum
        enum ParkDisableStart {
            CancelBeforeDisable,
            Contended,
            Poisoned,
            Outcome {
                res: Result<crate::driver::TorqueProgress, crate::driver::ArmDriverError>,
                requested_at: Instant,
                post_now: Instant,
                disable_kind: crate::driver::DisableKind,
                frames_dispatched: bool,
                dispatched_mask: u64,
                snap: OwnerDriverSnapshot,
            },
        }

        let stop_before_write = self.e_stop_latched.clone();
        let expected_stop_gen = self.stop_requests.as_ref().map(|s| s.applied());
        let estop_state = &self.estop_state;
        let applied_estop_count = self.applied_estop_count;
        let stop_gen_atomic = &self.stop_gen;
        let stop_requests = self.stop_requests.as_ref();
        let safety_stopped = self.safety.state().is_stopped();
        let rt_executor_owned = self.rt_executor_owned;
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let measured_positions = &self.measured_positions;
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let estop_pending_now = || {
            let (req, latched) = estop_state.snapshot();
            latched || req != applied_estop_count
        };
        let urgent_stop_unapplied_now = || {
            estop_state.requested_count() != applied_estop_count
                || stop_requests.is_some_and(crate::rt_core::urgent::StopRequests::pending)
        };
        let current_stop_gen_now = || {
            Self::compute_current_stop_gen(stop_gen_atomic, stop_requests)
        };
        #[cfg(not(target_arch = "wasm32"))]
        let pre_live_intent = self.shadow_live_intent();
        #[cfg(not(target_arch = "wasm32"))]
        let pre_live_torqued = self.is_torqued;
        let start = {
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let d = self.driver.as_mut().expect("checked is_some");
            let run_start = |drv: &mut dyn crate::driver::ArmDriver| -> ParkDisableStart {
                // Linearize client Stop and hardware E-stop against EVERY motor's disable transmit
                // both inside the driver slot AND inside the driver's per-motor gate
                // (`DamiaoArmDriver::send(TxKind::ClientParkDisable)` /
                // `FeetechArmDriver::write_torque_off_checked_budgeted`).
                if !watchdog && urgent_stop_unapplied_now() {
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        shadow_rt.drain_from_driver(drv);
                        let now = drv.clock_now();
                        shadow_rt.enqueue_at(
                            now,
                            crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                                live_op_id: action_id.unwrap_or(0),
                                admitted: false,
                                settled_ok: Some(false),
                            },
                        );
                        shadow_rt.enqueue_at(
                            now,
                            crate::torque::shadow::ShadowInputKind::Stop {
                                kind: crate::torque::StopKind::Stop,
                                live_intent: pre_live_intent,
                                live_is_torqued: pre_live_torqued,
                                live_stopped: true,
                            },
                        );
                    }
                    ParkDisableStart::CancelBeforeDisable
                } else if stop_before_write.load(Ordering::SeqCst) || estop_pending_now() {
                    let now = drv.clock_now();
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    let disable_kind =
                        crate::driver::DisableKind::DisableRequested { started_at: now };
                    let snap = OwnerDriverSnapshot::capture_from_driver(
                        &*drv,
                        dof,
                        feedback_numeric_valid,
                        measured_positions,
                        &mut ages_buf,
                        &mut bounds_buf,
                    );
                    ParkDisableStart::Outcome {
                        res: Err(crate::driver::ArmDriverError::EstopInhibited),
                        requested_at: now,
                        post_now: now,
                        disable_kind,
                        frames_dispatched: false,
                        dispatched_mask: 0,
                        snap,
                    }
                } else {
                    let requested_at = drv.clock_now();
                    let disable_kind = crate::driver::DisableKind::DisableRequested {
                        started_at: requested_at,
                    };
                    // Run the live disable I/O BEFORE any shadow enqueue/drain so shadow work can
                    // never shift release-cutoff TX deadlines (Major #1).
                    let res = if !watchdog {
                        let max_rx = if rt_executor_owned || drv.supports_rt_executor().is_ok() {
                            Some(0)
                        } else {
                            None
                        };
                        drv.begin_client_park_disable(max_rx, expected_stop_gen)
                    } else if rt_executor_owned || drv.supports_rt_executor().is_ok() {
                        drv.begin_disable_torque_budgeted(disable_kind, 0)
                    } else {
                        drv.begin_disable_torque(disable_kind)
                    };
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        let live_op_id =
                            action_id.unwrap_or_else(|| shadow_rt.alloc_live_op_id());
                        shadow_rt.enqueue_at(
                            requested_at,
                            crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                                live_op_id,
                                admitted: true,
                                settled_ok: None,
                            },
                        );
                        shadow_rt.enqueue_at(
                            requested_at,
                            crate::torque::shadow::ShadowInputKind::DisableRequest {
                                live_op_id,
                                explicit_release_authorized: false,
                                live_refusal: None,
                                live_intent: pre_live_intent,
                                live_is_torqued: pre_live_torqued,
                                live_stopped: safety_stopped,
                            },
                        );
                        shadow_rt.drain_from_driver(drv);
                    }
                    let frames_dispatched = drv.disable_frames_dispatched();
                    let dispatched_mask = drv.disable_dispatched_mask();
                    let post_now = drv.clock_now();
                    if !watchdog
                        && !frames_dispatched
                        && matches!(&res, Err(e) if crate::driver::is_estop_inhibited(e))
                        && !stop_before_write.load(Ordering::SeqCst)
                        && !estop_pending_now()
                        && !safety_stopped
                        && (urgent_stop_unapplied_now()
                            || expected_stop_gen.is_some_and(|g| current_stop_gen_now() != g))
                    {
                        #[cfg(not(target_arch = "wasm32"))]
                        {
                            shadow_rt.enqueue_at(
                                post_now,
                                crate::torque::shadow::ShadowInputKind::Stop {
                                    kind: crate::torque::StopKind::Stop,
                                    live_intent: pre_live_intent,
                                    live_is_torqued: pre_live_torqued,
                                    live_stopped: true,
                                },
                            );
                            shadow_rt.enqueue_at(
                                post_now,
                                crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                                    live_op_id: action_id.unwrap_or(0),
                                    admitted: false,
                                    settled_ok: Some(false),
                                },
                            );
                        }
                        // Driver stop-gated client park before any disable frame reached the wire:
                        // zero disable frames were transmitted, so cancel cleanly and hold torque.
                        ParkDisableStart::CancelBeforeDisable
                    } else {
                        let snap = OwnerDriverSnapshot::capture_from_driver(
                            &*drv,
                            dof,
                            feedback_numeric_valid,
                            measured_positions,
                            &mut ages_buf,
                            &mut bounds_buf,
                        );
                        ParkDisableStart::Outcome {
                            res,
                            requested_at,
                            post_now,
                            disable_kind,
                            frames_dispatched,
                            dispatched_mask,
                            snap,
                        }
                    }
                }
            };
            if rt_executor_owned || self.nonblocking_driver_release {
                match d.try_with_mut(run_start) {
                    Ok(v) => v,
                    Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                        ParkDisableStart::Contended
                    }
                    Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                        ParkDisableStart::Poisoned
                    }
                }
            } else {
                d.with_mut(run_start).unwrap_or(ParkDisableStart::Poisoned)
            }
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;

        let (res, requested_at, post_now, disable_kind, frames_dispatched, dispatched_mask, snap) =
            match start {
                ParkDisableStart::CancelBeforeDisable => {
                    self.cancel_park_before_disable(action_id);
                    return;
                }
                ParkDisableStart::Contended => {
                    self.record_disable_lock_contention();
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "DRIVER_BUSY",
                        "Driver busy during park disable",
                    );
                    return;
                }
                ParkDisableStart::Poisoned => {
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "FEEDBACK_UNAVAILABLE",
                        "Driver unavailable during park disable",
                    );
                    return;
                }
                ParkDisableStart::Outcome {
                    res,
                    requested_at,
                    post_now,
                    disable_kind,
                    frames_dispatched,
                    dispatched_mask,
                    snap,
                } => {
                    self.reset_disable_lock_contention();
                    (
                        res,
                        requested_at,
                        post_now,
                        disable_kind,
                        frames_dispatched,
                        dispatched_mask,
                        snap,
                    )
                }
            };

        self.owner_driver_snapshot = snap;
        self.retire_motion_payloads();
        self.pending_deferred_done = None;
        self.end_gripper_stream();
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
        self.commanded_velocities.fill(0.0);
        self.background_disable_retry = None;
        self.active_disable_kind = Some(disable_kind);
        self.active_disable_client_park = (!watchdog).then_some(expected_stop_gen);
        self.disable_requested_at(requested_at);

        match res {
            Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                    self.disable_first_frame_accepted();
                }
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::Park {
                        watchdog,
                        action_id,
                        expected_stop_gen,
                    },
                    kind: disable_kind,
                });
            }
            Ok(crate::driver::TorqueProgress::Done(_)) => {
                self.complete_park_disable_accepted(watchdog, action_id);
            }
            Err(e @ crate::driver::ArmDriverError::PartialDisable { .. }) => {
                let (err_detail, code) = self.handle_partial_disable_abort(e);
                self.finish_park_disable_failed(watchdog, action_id, code, err_detail.as_str());
            }
            Err(ref e) if crate::driver::is_estop_inhibited(e) => {
                // Keep `disable_pending` set when E-stop intervened and do not latch `DriverWriteFailed`.
                if let Some(id) = action_id {
                    if self
                        .last_action
                        .as_ref()
                        .is_some_and(|a| a.id == id && a.is_running())
                    {
                        let mut msg = self.take_action_msg_buf(id, 64);
                        msg.clear();
                        msg.push_str("E-stop latched during park disable");
                        self.finish_action(
                            id,
                            "failed",
                            Some("ESTOP_ACTIVE"),
                            Some(msg),
                            None,
                            None,
                            None,
                        );
                    }
                }
            }
            Err(e) => {
                let refused = matches!(
                    e,
                    crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired
                );
                let client_park_stopped =
                    !watchdog && self.client_park_stop_invalidated(expected_stop_gen);
                let (code, text) = if client_park_stopped {
                    // Stop arrived mid-batch after partial motor disable: NEVER hand the remaining
                    // undispatched motors to an unstamped retry; hold remaining energized motors
                    // and latch `DriverWriteFailed`.
                    self.clear_active_disable_retry_state();
                    let code = self.disable_step_failed_inner(e, false);
                    (code, "Client park disable stopped mid-batch")
                } else {
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
                    let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                    let text = if refused && !frames_dispatched {
                        "Park torque-off refused by rest-pose gate"
                    } else {
                        "Torque disable transmission failed"
                    };
                    (code, text)
                };
                self.finish_park_disable_failed(watchdog, action_id, code, text);
            }
        }
    }

    pub(in crate::supervisor) fn cancel_park_before_disable(&mut self, action_id: Option<u64>) {
        self.safety.mark_motion_finished();
        if let Some(id) = action_id {
            if self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running()) {
                self.finish_action(id, "cancelled", Some("STOPPED"), None, None, None, None);
            }
        }
    }

    /// Zero-allocation completion when park torque-off is accepted through the rest-pose gate:
    /// records `torque_intent = Off` and preserves any latched E-stop/Fault safety state.
    pub(in crate::supervisor) fn complete_park_disable_accepted(&mut self, watchdog: bool, action_id: Option<u64>) {
        self.disable_accepted();
        self.finish_park_disable_after_accepted(watchdog, action_id);
    }

    pub(in crate::supervisor) fn finish_park_disable_after_accepted(&mut self, watchdog: bool, action_id: Option<u64>) {
        if !self.safety.state().is_stopped() {
            let _ = self.torque_off_finish_rt();
        } else {
            self.reset_gripper_stall();
            self.gripper_hold_vel_ceiling = None;
            if self.driver.is_none() { self.measured_velocities.fill(0.0); }
        }
        self.finish_park_disable_ok(watchdog, action_id);
    }

    pub(in crate::supervisor) fn finish_park_rearm_failed(&mut self, action_id: u64, code: &'static str) {
        if self.park_continuation.is_some_and(|c| c.action_id == action_id) { self.park_continuation = None; }
        let mut msg = self.take_action_msg_buf(action_id, 64);
        msg.clear(); msg.push_str("Park rearm failed");
        self.finish_action(action_id, "failed", Some(code), Some(msg), None, None, None);
        self.notify_park_continuation();
    }

    pub(in crate::supervisor) fn finish_park_disable_ok(&mut self, watchdog: bool, action_id: Option<u64>) {
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow
            .enqueue(crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                live_op_id: action_id.unwrap_or(0),
                admitted: false,
                settled_ok: Some(true),
            });
        if watchdog && !self.rt_executor_owned { tracing::info!("Watchdog park complete: torque off at resting pose"); }
        if let Some(id) = action_id { if !self.safety.state().is_stopped() { self.finish_action(id, "done", None, None, None, None, None); } }
    }

    pub(in crate::supervisor) fn finish_park_disable_failed(&mut self, watchdog: bool, action_id: Option<u64>, code: &'static str, text: &str) {
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow
            .enqueue(crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                live_op_id: action_id.unwrap_or(0),
                admitted: false,
                settled_ok: Some(false),
            });
        self.safety.mark_motion_finished();
        if watchdog && (self.is_torqued || self.torque_intent == TorqueIntent::On) && !self.safety.state().is_stopped() {
            self.latch_watchdog_fault();
        }
        if let Some(id) = action_id {
            if self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running()) {
                let mut msg = self.take_action_msg_buf(id, 96);
                msg.clear(); msg.push_str(text);
                self.finish_action(id, "failed", Some(code), Some(msg), None, None, None);
            }
        }
    }

    #[inline]
    pub(in crate::supervisor) fn all_joints_dispatched_for_disable(
        &self,
        frames_dispatched: bool,
        dispatched_mask: u64,
    ) -> bool {
        let all = self.all_joints_mask();
        if dispatched_mask != 0 {
            (dispatched_mask & all) == all
        } else {
            frames_dispatched
        }
    }
}
