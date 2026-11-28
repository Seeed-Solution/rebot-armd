//! Stopped measured-hold servicing, owner driver snapshots, and background disable retry.

#[allow(unused_imports)]
use super::*;
#[allow(unused_imports)]
use super::super::*;

// scaffold-extra-0
impl SupervisorInner {
    /// Persistent enable lifecycle owned by the transition reducer.
    pub fn enable_lifecycle(&self) -> EnableLifecycle {
        self.enable_lifecycle
    }

    /// Owner-owned torque decision state (`Off` only after an accepted disable or protective cut).
    pub fn torque_intent(&self) -> TorqueIntent {
        self.torque_intent
    }

    /// Latest per-joint hold status observed by the stopped/failed-hold tracker.
    pub fn joint_hold_status(&self) -> &[crate::driver::JointHoldStatus] {
        &self.measured_hold.joint_status[..self.measured_hold.dof]
    }

    /// Single shared clear-fault enable context for BOTH legacy and RT clear owners:
    /// reported enable state from feedback is telemetry only (never blocks recovery).
    #[inline]
    pub(in crate::supervisor) fn populate_enable_clear_context(&self, ctx: &mut crate::safety::ClearContext, enable_fb: bool) {
        ctx.enable_state_known = true;
        ctx.failed_hold_recovery_allowed = self.enable_lifecycle == EnableLifecycle::FailedHold
            && enable_fb
            && self.safety.has_latched_fault_code(FaultCode::EnableFailed)
            && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::BusLoss)
            && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::ThermalTrip);
    }

    /// Stopped-hold eligibility derived strictly from the owner's decision state (`torque_intent`):
    /// holds are the default after any stop, fault, E-stop, failed/interrupted enable, lease loss,
    /// or watchdog event, and stop ONLY after an accepted disable records `torque_intent = Off`.
    #[inline]
    pub(crate) fn stopped_hold_eligible(&self) -> bool {
        self.torque_intent == TorqueIntent::On
            && !matches!(
                self.active_disable_kind,
                Some(crate::driver::DisableKind::DisableProtective { .. })
            )
    }

    /// Refreshes [`Self::owner_driver_snapshot`] from an already-locked driver reference without
    /// acquiring `self.driver` and without heap allocations (Rule R1).
    pub(crate) fn refresh_owner_driver_snapshot_from_ref(&mut self, drv: &dyn crate::driver::ArmDriver) {
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        self.owner_driver_snapshot = OwnerDriverSnapshot::capture_from_driver(
            drv,
            self.config.dof,
            self.feedback_numeric_valid,
            &self.measured_positions,
            &mut ages_buf,
            &mut bounds_buf,
        );
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
    }

    /// L2: For every joint while torqued or holding while stopped (`torqued_or_stopped`) whose
    /// control mode is unverified, request a `RID_CTRL_MODE` readback every tick while already
    /// holding the driver lock in `poll_hardware_rx`. If still unverified after 200 ms, latch a
    /// non-recoverable fault naming the joint(s) and report the joint unheld
    /// (`UnheldNoVerifiedMode`). Zero heap allocations.
    pub(crate) fn enforce_unverified_mode_readback_parts(
        measured_hold: &mut MeasuredHoldTracker,
        safety: &mut crate::safety::SafetyManager,
        snap: &OwnerDriverSnapshot,
        transition_pending: bool,
        torqued_or_stopped: bool,
        config_dof: usize,
        driver: &mut dyn crate::driver::ArmDriver,
    ) {
        use crate::driver::JointHoldStatus;
        if !snap.valid || transition_pending {
            return;
        }
        let drv_now = snap.captured_at.unwrap_or_else(|| driver.clock_now());
        let dof = config_dof.min(crate::rt_core::mailbox::MAX_DOF);
        measured_hold.dof = dof;
        let mut unverified_mask = 0u64;
        let mut timed_out_mask = 0u64;
        for j in 0..dof {
            if !torqued_or_stopped || snap.hold_mode_verified[j] {
                measured_hold.unverified_mode_since[j] = None;
                measured_hold.unverified_mode_faulted_mask &= !(1u64 << j);
                continue;
            }
            unverified_mask |= 1u64 << j;
            measured_hold.joint_status[j] = JointHoldStatus::UnheldNoVerifiedMode;
            let since = *measured_hold.unverified_mode_since[j].get_or_insert(drv_now);
            if drv_now.saturating_duration_since(since) >= UNVERIFIED_MODE_READBACK_TIMEOUT
                && (measured_hold.unverified_mode_faulted_mask & (1u64 << j)) == 0
            {
                timed_out_mask |= 1u64 << j;
            }
        }
        if unverified_mask != 0 {
            let _ = driver.joint_mode_mut().request_mode_readback_budgeted(unverified_mask);
        }
        if timed_out_mask != 0 {
            measured_hold.unverified_mode_faulted_mask |= timed_out_mask;
            let detail = crate::safety::FaultDetail::formatted(FaultCode::HardwareFault, |buf| {
                let mut first = true;
                for j in 0..dof {
                    if (timed_out_mask & (1u64 << j)) != 0 {
                        if !first {
                            buf.push_str(", ");
                        }
                        first = false;
                        let _ = core::fmt::write(buf, format_args!("J{}", j + 1));
                    }
                }
                buf.push_str(" control mode unverified after 200 ms readback timeout; joint unheld");
            });
            safety.latch(crate::safety::FaultKind::Other, detail);
        }
    }

    /// Single shared stopped-hold capture kernel reading strictly from the owner's inline
    /// [`Self::owner_driver_snapshot`] with ZERO driver mutex acquisition and ZERO heap
    /// allocations (Rule R1). Commands holds to EVERY joint with fresh (`<= 100 ms`) finite
    /// measured position and verified hold mode, regardless of reported motor enable telemetry.
    /// Returns `(hold_mask, invalidated_mask)`.
    pub(crate) fn capture_measured_hold_from_owner(&mut self) -> (u64, u64) {
        use crate::driver::{JointHoldStatus, STOPPED_DISABLE_FEEDBACK_MAX_AGE};
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        self.measured_hold.dof = dof;
        if !self.owner_driver_snapshot.valid {
            let mut hold_mask = 0u64;
            let mut invalidated_mask = 0u64;
            for j in 0..dof {
                if self.measured_hold.captured_positions[j].is_some() {
                    if self.owner_driver_snapshot.hold_mode_verified[j] {
                        self.measured_hold.joint_status[j] = JointHoldStatus::HeldStaleTarget;
                        hold_mask |= 1u64 << j;
                    } else {
                        self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoVerifiedMode;
                    }
                } else {
                    self.measured_hold.captured_continuity_epoch[j] = None;
                    self.measured_hold.last_seen_fresh_at[j] = None;
                    self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoFreshFeedback;
                    invalidated_mask |= 1u64 << j;
                }
            }
            return (hold_mask, invalidated_mask);
        }
        let snap = self.owner_driver_snapshot;
        let drv_now = snap.captured_at.unwrap_or_else(Instant::now);
        let mut hold_mask = 0u64;
        let mut invalidated_mask = 0u64;
        for j in 0..dof {
            let Some((pos, epoch)) = snap.fresh_samples[j] else {
                // B2: Never drop an already-captured hold while torque_intent == On.
                // Keep the last captured target and report `HeldStaleTarget` ("hold target stale")
                // once mode is verified; if mode is unverified, follow L2 (200 ms readback -> fault)
                // while preserving the captured hold target.
                if self.measured_hold.captured_positions[j].is_some() {
                    if snap.hold_mode_verified[j] {
                        self.measured_hold.joint_status[j] = JointHoldStatus::HeldStaleTarget;
                        hold_mask |= 1u64 << j;
                    } else {
                        self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoVerifiedMode;
                    }
                } else {
                    self.measured_hold.captured_continuity_epoch[j] = None;
                    self.measured_hold.last_seen_fresh_at[j] = None;
                    self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoFreshFeedback;
                    invalidated_mask |= 1u64 << j;
                }
                continue;
            };
            if !snap.hold_mode_verified[j] {
                // L2: 200 ms readback -> else fault; preserve any already-captured hold target.
                self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoVerifiedMode;
                invalidated_mask |= 1u64 << j;
                continue;
            }
            let continuity_broken = self.measured_hold.captured_positions[j].is_none()
                || self.measured_hold.captured_continuity_epoch[j] != Some(epoch)
                || self.measured_hold.last_seen_fresh_at[j].is_some_and(|prev| {
                    drv_now.saturating_duration_since(prev) > STOPPED_DISABLE_FEEDBACK_MAX_AGE
                });
            if continuity_broken {
                self.measured_hold.captured_positions[j] = Some(pos);
                self.measured_hold.captured_continuity_epoch[j] = Some(epoch);
                self.measured_hold.joint_status[j] = JointHoldStatus::Queued;
                invalidated_mask |= 1u64 << j;
            } else if !matches!(
                self.measured_hold.joint_status[j],
                JointHoldStatus::Held | JointHoldStatus::Queued
            ) {
                self.measured_hold.joint_status[j] = JointHoldStatus::Queued;
            }
            self.measured_hold.last_seen_fresh_at[j] = Some(drv_now);
            hold_mask |= 1u64 << j;
        }
        (hold_mask, invalidated_mask)
    }

    /// Evaluates each joint independently from fresh (`<= 100 ms`) finite measured position with
    /// transport provenance and continuity tracking, and verified hold mode without acquiring the
    /// driver mutex (Rule R1; zero heap allocations).
    pub fn snapshot_measured_hold_with_invalidated(&mut self) -> (u64, u64) {
        use crate::driver::JointHoldStatus;
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        self.measured_hold.dof = dof;
        if self.driver.is_none() {
            for j in 0..dof {
                if self.is_torqued || self.torque_intent == TorqueIntent::On {
                    if let Some(p) = self.measured_positions.get(j).copied().filter(|p| p.is_finite()) {
                        let _ = self.measured_hold.captured_positions[j].get_or_insert(p);
                        self.measured_hold.joint_status[j] = JointHoldStatus::Held;
                    } else {
                        self.measured_hold.captured_positions[j] = None;
                        self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoFreshFeedback;
                    }
                } else {
                    self.measured_hold.captured_positions[j] = None;
                    self.measured_hold.joint_status[j] = JointHoldStatus::SkippedOff;
                }
            }
            return (0, 0);
        }
        self.capture_measured_hold_from_owner()
    }

    pub(crate) fn snapshot_measured_hold(&mut self) -> u64 {
        self.snapshot_measured_hold_with_invalidated().0
    }

    /// Test-only wrapper around [`Self::service_stopped_measured_hold`] returning `true` when the
    /// stopped-hold pass completed without exposing private `TickOutput`.
    #[doc(hidden)]
    pub fn service_stopped_measured_hold_complete(
        &mut self,
        dt: f64,
        gravity_output_enabled: bool,
    ) -> bool {
        matches!(
            self.service_stopped_measured_hold(dt, gravity_output_enabled),
            crate::supervisor::TickOutput::Complete { is_hold: true, .. }
        )
    }

    #[doc(hidden)]
    pub fn owner_hold_mask(&self) -> u64 {
        self.measured_hold.owner_hold_mask
    }

    #[doc(hidden)]
    pub fn owner_pending_hold_mask(&self) -> u64 {
        self.measured_hold.owner_pending_hold_mask
    }

    #[doc(hidden)]
    pub fn invalidate_owner_driver_snapshot_for_test(&mut self) {
        self.owner_driver_snapshot.invalidate();
    }

    /// Test-only helper (L3): holds `self.driver`'s mutex on a separate OS thread while running
    /// `f(self)` (which must not replace or drop `self.driver`), then releases and joins the
    /// holder thread before returning.
    #[doc(hidden)]
    pub fn with_contended_driver_for_test<R>(&mut self, f: impl FnOnce(&mut Self) -> R) -> R {
        let Some(driver_lock) = self
            .driver
            .as_ref()
            .and_then(crate::supervisor_lock::DriverSlot::as_legacy_lock)
        else {
            return f(self);
        };
        let addr =
            (driver_lock.raw_mutex() as *const std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>)
                as usize;
        let (locked_tx, locked_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let (release_tx, release_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let holder = std::thread::spawn(move || {
            // SAFETY: `self.driver` stays in place for the duration of `f(self)` and this thread
            // is strictly joined before `with_contended_driver_for_test` returns (even on panic).
            let mutex = unsafe { &*(addr as *const std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>) };
            let _guard = mutex.lock().unwrap();
            let _ = locked_tx.send(());
            let _ = release_rx.recv_timeout(Duration::from_secs(5));
        });
        locked_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("holder acquired driver mutex");
        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(self)));
        let _ = release_tx.send(());
        let _ = holder.join();
        match res {
            Ok(v) => v,
            Err(payload) => std::panic::resume_unwind(payload),
        }
    }

    /// Services one masked measured-position hold pass while stopped or in `FailedHold`
    /// (zero heap allocation on RT; hold capture and MIT preparation read owner-held data without
    /// locking the driver mutex; never falls back to cached `commanded_positions`).
    pub(crate) fn service_stopped_measured_hold(
        &mut self,
        dt: f64,
        gravity_output_enabled: bool,
    ) -> crate::supervisor::TickOutput {
        use crate::driver::{JointHoldStatus, TxBatchOutcome};
        use crate::supervisor::TickOutput;
        if self.urgent_stop_unapplied() {
            return TickOutput::None;
        }
        if self.driver.is_none() {
            return TickOutput::None;
        }
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        let (mut hold_mask, invalidated_mask) = self.snapshot_measured_hold_with_invalidated();

        let mut targets = std::mem::take(&mut self.scratch.targets);
        targets.clear();
        for i in 0..self.config.dof {
            let pos = if i < dof {
                self.measured_hold.captured_positions[i].unwrap_or(0.0)
            } else {
                0.0
            };
            targets.push(crate::driver::JointTarget {
                position: pos,
                velocity: Some(0.0),
                torque: if gravity_output_enabled {
                    self.gravity_torques_ramped.get(i).copied()
                } else {
                    Some(0.0)
                },
                kp: None,
                kd: None,
            });
        }

        // Per-joint masked MIT hold preparation (Rule R1: zero alloc, no feedback-age Vec,
        // no `last_ff.clone()`, and no driver mutex lock).
        let mut ff_buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let mit_prepared = if self.mit.owner == crate::mit::MitOwner::Active && hold_mask != 0 {
            let before_mask = hold_mask;
            let (filtered_mask, any_mit) =
                self.mit_prepare_masked_into(hold_mask, &mut targets, dt, &mut ff_buf);
            let rejected = before_mask & !filtered_mask;
            if rejected != 0 {
                for j in 0..dof {
                    if (rejected & (1u64 << j)) != 0 {
                        self.measured_hold.joint_status[j] = if self.measured_hold.captured_positions[j].is_some() {
                            JointHoldStatus::HeldStaleTarget
                        } else {
                            JointHoldStatus::UnheldNoFreshFeedback
                        };
                    }
                }
            }
            hold_mask = filtered_mask;
            any_mit
        } else {
            false
        };

        // L3: The stopped-hold queue and mask are owner state. Record them inline before TX.
        self.measured_hold.owner_hold_mask = hold_mask;
        for (slot, target) in self.measured_hold.owner_hold_targets[..dof]
            .iter_mut()
            .zip(targets.iter())
        {
            *slot = target.position;
        }

        // L3: An empty mask with no owner-tracked queued holds takes NO driver lock at all.
        let keep_queued_mask = hold_mask & !invalidated_mask;
        let needs_prune = (self.measured_hold.owner_pending_hold_mask & !keep_queued_mask) != 0;
        if hold_mask == 0 && !needs_prune {
            self.measured_hold.consecutive_lock_contended = 0;
            self.measured_hold.lock_contended_escalated = false;
            self.scratch.targets = targets;
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.enqueue_at(
                self.cached_driver_clock_now(),
                crate::torque::shadow::ShadowInputKind::LiveHoldCheckpoint {
                    hold_mask: crate::torque::ChannelMask::EMPTY,
                    stale_held_mask: crate::torque::ChannelMask::EMPTY,
                    contended: false,
                    escalated: false,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
            return TickOutput::None;
        }

        // L3: Non-blocking `try_lock` for stopped-hold TX: on contention, skip this tick's send
        // and increment the diagnostics counter; escalate after `STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS`
        // (25 consecutive skips, 0.5 s at 50 Hz) with a counted warning and a non-recoverable fault
        // (zero heap allocation on RT).
        let mit_lost = self.mit.owner == crate::mit::MitOwner::Lost;
        let mut lock_contended = false;
        #[cfg(not(target_arch = "wasm32"))]
        let shadow_rt = &mut self.torque_shadow;
        let write_result = match self.driver.as_mut() {
            Some(d) => match d.try_with_mut(|drv| {
                drv.mit_mut().set_mit_ownership_lost(mit_lost);
                let dispatched_mask = drv.disable_dispatched_mask();
                let effective_hold_mask = hold_mask & !dispatched_mask;
                let effective_keep_mask = keep_queued_mask & !dispatched_mask;
                drv.tx_mut().drop_pending_holds_outside_mask(effective_keep_mask);
                drv.pos_force_mut().set_pending_pos_force_hold_hint(true);
                let r = drv.write_measured_hold_masked(effective_hold_mask, &targets);
                let now = drv.clock_now();
                let max_age = drv.tx().tx_frame_max_age();
                let oldest_pending = drv.tx().oldest_pending_tx_age();
                let last_expired = drv.tx().last_tx_expired_age();
                let sent_any_fallback = drv.tx().last_hold_tx_sent_any();
                let due_mask = drv.tx().last_hold_tx_due_mask();
                let sent_mask = drv.tx().last_hold_tx_sent_mask();
                let pf_status = drv.pos_force_mut().take_last_pos_force_tx_status();
                let has_pending_holds = drv.tx().pending_tx_len_total() > 0;
                #[cfg(not(target_arch = "wasm32"))]
                shadow_rt.drain_from_driver(drv);
                (
                    r,
                    now,
                    max_age,
                    oldest_pending,
                    last_expired,
                    sent_any_fallback,
                    due_mask,
                    sent_mask,
                    pf_status,
                    has_pending_holds,
                    effective_hold_mask,
                )
            }) {
                Ok(v) => Some(v),
                Err(_) => {
                    lock_contended = true;
                    None
                }
            },
            None => None,
        };
        self.scratch.targets = targets;

        if lock_contended {
            crate::diagnostics::tick::record_stopped_hold_lock_contended();
            self.measured_hold.consecutive_lock_contended =
                self.measured_hold.consecutive_lock_contended.saturating_add(1);
            if self.measured_hold.consecutive_lock_contended >= STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS
                && !self.measured_hold.lock_contended_escalated
            {
                self.measured_hold.lock_contended_escalated = true;
                crate::diagnostics::tick::record_stopped_hold_contention_escalation();
                let skips = self.measured_hold.consecutive_lock_contended;
                let detail = crate::safety::FaultDetail::formatted(FaultCode::HardwareFault, |buf| {
                    let _ = core::fmt::write(
                        buf,
                        format_args!(
                            "Stopped-hold TX skipped {} consecutive ticks due to driver mutex contention",
                            skips
                        ),
                    );
                });
                self.safety.latch(crate::safety::FaultKind::Other, detail);
            }
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::LiveHoldCheckpoint {
                    hold_mask: crate::torque::ChannelMask::EMPTY,
                    stale_held_mask: crate::torque::ChannelMask::EMPTY,
                    contended: true,
                    escalated: self.measured_hold.lock_contended_escalated,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
            return TickOutput::None;
        }
        self.measured_hold.consecutive_lock_contended = 0;
        self.measured_hold.lock_contended_escalated = false;

        let Some((
            write_res,
            now,
            max_age,
            oldest_pending,
            last_expired,
            sent_any_fallback,
            due_mask,
            sent_mask,
            pf_status,
            has_pending_holds,
            effective_hold_mask,
        )) = write_result
        else {
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.enqueue_at(
                self.cached_driver_clock_now(),
                crate::torque::shadow::ShadowInputKind::LiveHoldCheckpoint {
                    hold_mask: crate::torque::ChannelMask::EMPTY,
                    stale_held_mask: crate::torque::ChannelMask::EMPTY,
                    contended: false,
                    escalated: false,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
            return TickOutput::None;
        };

        #[cfg(not(target_arch = "wasm32"))]
        let mut shadow_hm = crate::torque::ChannelMask::EMPTY;
        #[cfg(not(target_arch = "wasm32"))]
        let mut shadow_stale_hm = crate::torque::ChannelMask::EMPTY;

        let out = match write_res {
            Ok(report) => {
                let n = report.dof.min(dof);
                let mut next_pending_mask = 0u64;
                for j in 0..n {
                    self.measured_hold.joint_status[j] = report.joint_status[j];
                    #[cfg(not(target_arch = "wasm32"))]
                    if j < crate::torque::MAX_CHANNELS {
                        match report.joint_status[j] {
                            JointHoldStatus::Held | JointHoldStatus::Queued => {
                                shadow_hm.insert(j as u8);
                            }
                            JointHoldStatus::HeldStaleTarget => {
                                shadow_hm.insert(j as u8);
                                shadow_stale_hm.insert(j as u8);
                            }
                            _ => {}
                        }
                    }
                    if report.joint_status[j] == JointHoldStatus::Queued
                        || (report.joint_status[j] == JointHoldStatus::HeldStaleTarget
                            && has_pending_holds)
                    {
                        next_pending_mask |= 1u64 << j;
                    }
                    // B2: Preserve captured hold targets while torque_intent == On; only clear
                    // when explicitly SkippedOff (UnheldNoVerifiedMode preserves captured_positions).
                    if report.joint_status[j] == JointHoldStatus::SkippedOff {
                        self.measured_hold.captured_positions[j] = None;
                        self.measured_hold.captured_continuity_epoch[j] = None;
                        self.measured_hold.last_seen_fresh_at[j] = None;
                    }
                }
                #[cfg(not(target_arch = "wasm32"))]
                if shadow_hm.is_empty() && report.sent_any {
                    for j in 0..n.min(crate::torque::MAX_CHANNELS) {
                        if (effective_hold_mask & (1u64 << j)) != 0 {
                            shadow_hm.insert(j as u8);
                        }
                    }
                }
                if has_pending_holds && next_pending_mask == 0 {
                    next_pending_mask = hold_mask;
                }
                self.measured_hold.owner_pending_hold_mask = next_pending_mask;
                if hold_mask == 0 && !report.sent_any && matches!(report.outcome, TxBatchOutcome::Complete) {
                    TickOutput::None
                } else {
                    match report.outcome {
                        TxBatchOutcome::Complete => TickOutput::Complete {
                            is_hold: true,
                            now,
                            max_age,
                            due_mask,
                            sent_mask,
                            pf_status,
                        },
                        TxBatchOutcome::Deferred { pending } => TickOutput::Deferred {
                            pending,
                            now,
                            max_age,
                            oldest_age: oldest_pending,
                            due_mask,
                            is_hold: true,
                            sent_mask,
                            pf_status,
                        },
                        TxBatchOutcome::Expired { dropped, pending } => TickOutput::Expired {
                            dropped,
                            pending,
                            now,
                            max_age,
                            oldest_age: last_expired,
                            due_mask,
                            sent_mask,
                            is_hold: true,
                            pf_status,
                        },
                    }
                }
            }
            Err(crate::driver::ArmDriverError::EstopInhibited) => {
                TickOutput::EstopInhibited { is_hold: true }
            }
            Err(e) => {
                let mut buf = crate::mit::OutputErrorText::empty();
                e.write_bounded(&mut buf);
                TickOutput::Error {
                    message: buf,
                    now,
                    max_age,
                    is_hold: true,
                    sent_any: sent_any_fallback,
                    due_mask,
                    sent_mask,
                    pf_status,
                }
            }
        };

        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow.enqueue_at(
            now,
            crate::torque::shadow::ShadowInputKind::LiveHoldCheckpoint {
                hold_mask: shadow_hm,
                stale_held_mask: shadow_stale_hm,
                contended: false,
                escalated: false,
                live_intent: self.shadow_live_intent(),
                live_is_torqued: self.is_torqued,
                live_stopped: self.safety.state().is_stopped(),
            },
        );

        if mit_prepared && matches!(out, TickOutput::Complete { .. }) {
            let ff_n = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
            if self.mit.last_ff.len() == ff_n {
                self.mit.last_ff.copy_from_slice(&ff_buf[..ff_n]);
            } else {
                self.mit.last_ff.clear();
                self.mit.last_ff.extend_from_slice(&ff_buf[..ff_n]);
            }
            self.mit.last_output = Some(now);
        }
        out
    }

    #[inline]
    pub(in crate::supervisor) fn all_joints_mask(&self) -> u64 {
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        if dof >= 64 {
            u64::MAX
        } else {
            (1u64 << dof) - 1
        }
    }

    #[inline]
    pub(in crate::supervisor) fn is_partial_disable_remaining(&self, dispatched_mask: u64) -> bool {
        let all = self.all_joints_mask();
        dispatched_mask != 0 && (dispatched_mask & all) != all
    }

    /// R12 B3: Once ANY motor has received a disable frame (`frames_dispatched == true`), the
    /// disable is committed (`torque_intent = Off`, holds stopped). When undispatched motors
    /// remain (`partial_remaining`), preserve `disable_pending = true` and return the retry
    /// `DisableKind` so the retry path finishes the remaining motors (keeping original `started_at`
    /// while within 1 s, or starting a bounded continuation pass if expiry fired mid-batch after
    /// at least one motor was already disabled).
    pub(in crate::supervisor) fn commit_partial_disable_retry_kind(
        &mut self,
        kind: crate::driver::DisableKind,
        post_now: Instant,
        frames_dispatched: bool,
        partial_remaining: bool,
    ) -> Option<crate::driver::DisableKind> {
        if !frames_dispatched {
            return None;
        }
        self.disable_first_frame_accepted();
        if !partial_remaining {
            return None;
        }
        let retry_kind = match kind {
            crate::driver::DisableKind::DisableRequested { started_at }
                if post_now.saturating_duration_since(started_at)
                    >= crate::driver::STOPPED_DISABLE_REQUEST_LIFETIME =>
            {
                crate::driver::DisableKind::DisableRequested { started_at: post_now }
            }
            other => other,
        };
        self.disable_pending = true;
        self.active_disable_kind = Some(retry_kind);
        Some(retry_kind)
    }

    #[inline]
    pub(crate) fn client_park_stop_invalidated(&self, expected_stop_gen: Option<u64>) -> bool {
        self.e_stop_latched.load(Ordering::SeqCst)
            || self.estop_pending()
            || self.urgent_stop_unapplied()
            || expected_stop_gen.is_some_and(|g| self.current_stop_gen() != g)
    }

    #[inline]
    pub(crate) fn clear_active_disable_retry_state(&mut self) {
        self.disable_pending = false;
        self.active_disable_kind = None;
        self.active_disable_client_park = None;
        self.last_disable_attempt = None;
        self.background_disable_retry = None;
    }

    /// Arms bounded [`BackgroundDisableRetry`] retrying immediately on the next release
    /// (`next_retry_at: drv_now`), used for committed partial disables (R12 B3) and contended
    /// protective thermal cuts (R12 B4).
    pub(in crate::supervisor) fn arm_background_disable_retry_immediate_with_kind(
        &mut self,
        kind: crate::driver::DisableKind,
        drv_now: Instant,
    ) -> bool {
        if matches!(
            self.background_disable_retry,
            Some(BackgroundDisableRetry::Exhausted)
        ) {
            return false;
        }
        let client_park = if matches!(kind, crate::driver::DisableKind::DisableProtective { .. }) {
            None
        } else {
            self.active_disable_client_park
        };
        if let Some(expected_stop_gen) = client_park {
            if self.client_park_stop_invalidated(expected_stop_gen) {
                return false;
            }
        }
        let origin = match kind {
            crate::driver::DisableKind::DisableRequested { started_at, .. } => started_at,
            crate::driver::DisableKind::DisableProtective { .. } => drv_now,
        };
        let window = self.background_disable_window_for(kind);
        if drv_now.saturating_duration_since(origin) >= window {
            self.exhaust_background_disable_retry();
            return false;
        }
        if self.background_disable_retry.is_none() {
            self.background_disable_retry = Some(BackgroundDisableRetry::Active {
                attempts: 0,
                kind,
                started_at: origin,
                next_retry_at: drv_now,
                client_park,
            });
        }
        true
    }

    /// Arms bounded [`BackgroundDisableRetry`] preserving the original [`crate::driver::DisableKind`]
    /// (`started_at` on the driver clock for `DisableRequested`) so every retry passes through the
    /// driver's single `begin_disable_torque_budgeted(kind, 0)` choke point and rest-pose gate
    /// within the original request's lifetime.
    pub(in crate::supervisor) fn arm_background_disable_retry_with_kind(
        &mut self,
        kind: crate::driver::DisableKind,
        drv_now: Instant,
    ) -> bool {
        if matches!(
            self.background_disable_retry,
            Some(BackgroundDisableRetry::Exhausted)
        ) {
            return false;
        }
        let client_park = if matches!(kind, crate::driver::DisableKind::DisableProtective { .. }) {
            None
        } else {
            self.active_disable_client_park
        };
        if let Some(expected_stop_gen) = client_park {
            if self.client_park_stop_invalidated(expected_stop_gen) {
                return false;
            }
        }
        let origin = match kind {
            crate::driver::DisableKind::DisableRequested { started_at, .. } => started_at,
            crate::driver::DisableKind::DisableProtective { .. } => drv_now,
        };
        let window = self.background_disable_window_for(kind);
        if drv_now.saturating_duration_since(origin) >= window {
            self.exhaust_background_disable_retry();
            return false;
        }
        if self.background_disable_retry.is_none() {
            self.background_disable_retry = Some(BackgroundDisableRetry::Active {
                attempts: 0,
                kind,
                started_at: origin,
                next_retry_at: drv_now + BACKGROUND_DISABLE_RETRY_INTERVAL,
                client_park,
            });
        }
        true
    }

    #[inline]
    pub(in crate::supervisor) fn background_disable_window_for(&self, kind: crate::driver::DisableKind) -> Duration {
        match kind {
            crate::driver::DisableKind::DisableRequested { .. } => {
                crate::driver::STOPPED_DISABLE_REQUEST_LIFETIME
            }
            crate::driver::DisableKind::DisableProtective { .. } => BACKGROUND_DISABLE_MAX_WINDOW,
        }
    }

    pub(in crate::supervisor) fn exhaust_background_disable_retry(&mut self) {
        self.background_disable_retry = Some(BackgroundDisableRetry::Exhausted);
        self.safety.latch(
            crate::safety::FaultKind::Other,
            crate::safety::FaultDetail::Static(crate::safety::StaticFault::DisableUnconfirmed),
        );
    }

    /// R12 B1: Advances an in-flight deferred legacy disable (`FeetechArmDriver::pending_disable`
    /// or `DamiaoArmDriver::AwaitRestGate` / `Disable`) by ONE bounded poll step per control
    /// release, releasing the driver mutex immediately afterward so `service_stopped_measured_hold`
    /// keeps servicing holds between releases.
    ///
    /// Returns `true` when the driver had a pending disable transition that was serviced this
    /// release (suppressing the 250 ms fallback restart on the same tick).
    pub(crate) fn service_legacy_pending_disable(&mut self) -> bool {
        if self.rt_executor_owned || !self.disable_pending || self.torque_transition.is_some() {
            return false;
        }
        if self.driver.is_none() {
            return false;
        }
        let control_release_seq = self.control_release_seq;
        let retry_kind = self.active_disable_kind;
        let client_park = self.active_disable_client_park;
        if let Some(expected_stop_gen) = client_park {
            if self.client_park_stop_invalidated(expected_stop_gen) {
                self.clear_active_disable_retry_state();
                return true;
            }
        }
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let period = Duration::from_secs_f64(
            1.0 / f64::from(self.config.control_frequency_hz.max(1)),
        );
        let per_tick_budget = period.saturating_sub(self.tx_release_publish_margin());
        let step = {
            let d = self.driver.as_mut().expect("checked is_some");
            d.with_mut(|drv| {
                let pending_hs = drv.torque_transition_pending();
                let partial_dispatched = drv.disable_dispatched_mask() != 0;
                if !pending_hs && !partial_dispatched {
                    return None;
                }
                if control_release_seq > 0 {
                    drv.begin_control_release(control_release_seq);
                }
                let drv_now = drv.clock_now();
                let poll_res = if pending_hs {
                    drv.poll_torque(drv_now + per_tick_budget)
                } else {
                    let k = retry_kind.unwrap_or(crate::driver::DisableKind::DisableRequested {
                        started_at: drv_now,
                    });
                    if let Some(expected_stop_gen) = client_park {
                        drv.begin_client_park_disable_with_kind(k, None, expected_stop_gen)
                    } else {
                        drv.begin_disable_torque(k)
                    }
                };
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
                Some((poll_res, post_now, frames_dispatched, dispatched_mask, snap))
            })
            .flatten()
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
        let Some(step) = step else {
            return false;
        };
        let (poll_res, post_now, frames_dispatched, dispatched_mask, snap) = step;
        self.owner_driver_snapshot = snap;
        let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
        let active_kind = self.active_disable_kind.unwrap_or(
            crate::driver::DisableKind::DisableRequested { started_at: post_now },
        );
        match poll_res {
            Ok(crate::driver::TorqueProgress::Done(_)) => {
                self.disable_accepted();
                let _ = self.torque_off_finish_in_place();
            }
            Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                    self.disable_first_frame_accepted();
                }
            }
            Err(
                crate::driver::ArmDriverError::TorqueOffWouldDrop
                | crate::driver::ArmDriverError::DisableRequestExpired,
            ) => {
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                    if !partial_remaining {
                        self.disable_pending = false;
                        self.active_disable_kind = None;
                        self.last_disable_attempt = None;
                    } else if let crate::driver::DisableKind::DisableRequested { started_at } = active_kind {
                        if post_now.saturating_duration_since(started_at)
                            >= crate::driver::STOPPED_DISABLE_REQUEST_LIFETIME
                        {
                            if self.background_disable_retry.is_none() {
                                let rk = crate::driver::DisableKind::DisableRequested {
                                    started_at: post_now,
                                };
                                self.active_disable_kind = Some(rk);
                                let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                            } else {
                                self.exhaust_background_disable_retry();
                                self.disable_pending = false;
                                self.active_disable_kind = None;
                            }
                        }
                    }
                } else {
                    self.disable_pending = false;
                    self.active_disable_kind = None;
                    self.last_disable_attempt = None;
                    self.background_disable_retry = None;
                }
            }
            Err(e) => {
                if let Some(expected_stop_gen) = client_park {
                    if crate::driver::is_estop_inhibited(&e)
                        || self.client_park_stop_invalidated(expected_stop_gen)
                    {
                        self.clear_active_disable_retry_state();
                        return true;
                    }
                }
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                }
                if !self.safety.deferred_logging() {
                    tracing::warn!("Legacy deferred torque disable step failed: {e}");
                }
            }
        }
        true
    }
}
