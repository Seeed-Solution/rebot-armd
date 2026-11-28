//! Driver installation, unload settle, retired queue drains, and torque-shadow helpers.

#[allow(unused_imports)]
use super::*;

// scaffold-extra-0
impl SupervisorInner {
    /// Zero-allocation monotonic bump of [`Self::motion_generation`] on motion lifecycle changes.
    #[inline]
    pub(crate) fn bump_motion_generation(&mut self) {
        self.motion_generation = self.motion_generation.wrapping_add(1).max(1);
    }

    #[inline]
    pub(super) fn motion_state_fingerprint(&self) -> u64 {
        let mut h = 0xcbf29ce484222325u64;
        let mut mix = |v: u64| {
            h ^= v;
            h = h.wrapping_mul(0x100000001b3);
        };
        let traj_tag = self
            .active_trajectory
            .as_ref()
            .map_or(0u64, |t| t.action_id.unwrap_or(u64::MAX));
        mix(traj_tag);
        mix(u64::from(self.servo_target.is_some()));
        mix(u64::from(self.servo_stream_active));
        let gc_tag = match self.gravity_comp.as_ref().map(|g| &g.phase) {
            None => 0u64,
            Some(crate::gravity_comp::GravityCompPhase::RampIn) => 1,
            Some(crate::gravity_comp::GravityCompPhase::Active) => 2,
            Some(crate::gravity_comp::GravityCompPhase::Exiting { .. }) => 3,
            Some(crate::gravity_comp::GravityCompPhase::ExitFailed { .. }) => 4,
        };
        mix(gc_tag);
        #[cfg(feature = "policy-runtime")]
        let policy_tag = self.policy_run.as_ref().map_or(0u64, |p| p.action_id.max(1));
        #[cfg(not(feature = "policy-runtime"))]
        let policy_tag = 0u64;
        mix(policy_tag);
        mix(self.stop_requests.as_ref().map_or_else(|| self.current_stop_gen(), |s| s.applied()));
        mix(self.safety.epoch_id());
        h.max(1)
    }

    #[inline]
    pub(super) fn sync_motion_generation_for_publish(&mut self) {
        let fp = self.motion_state_fingerprint();
        if self.last_published_motion_fp != 0 && self.last_published_motion_fp != fp {
            self.bump_motion_generation();
        }
        self.last_published_motion_fp = fp;
    }

    /// Pop one retired `ActiveTrajectory` for off-thread drop (0 alloc, 0 drop).
    pub fn pop_retired_trajectory(&mut self) -> Option<ActiveTrajectory> {
        self.retired_trajectories.pop_front()
    }

    /// True when the supervisor has guaranteed off-thread retirement capacity for a new `MacroRun`
    /// so retiring it on `armd-control` can never fall back to freeing heap strings on the RT
    /// thread. Always `true` in Legacy mode (off-RT).
    pub fn can_admit_macro_run(&self) -> bool {
        !self.rt_executor_owned
            || self.retired_macro_runs.len() + usize::from(self.macro_run.is_some())
                < self.retired_macro_runs.capacity()
    }

    /// Queue a replaced `MacroRun` for off-thread drop when owned by the RT executor
    /// (prevents dropping heap-backed `phase` / `SubState::Failed` strings on `armd-control`).
    pub(crate) fn retire_macro_run(&mut self, run: crate::macro_action::MacroRun) {
        if self.rt_executor_owned
            && self.retired_macro_runs.len() < self.retired_macro_runs.capacity()
        {
            self.retired_macro_runs.push_back(run);
        }
    }

    /// Number of retired `MacroRun` instances currently queued for off-thread drop.
    pub fn retired_macro_runs_len(&self) -> usize {
        self.retired_macro_runs.len()
    }

    /// Pop one retired `MacroRun` for off-thread drop (0 alloc, 0 drop).
    pub fn pop_retired_macro_run(&mut self) -> Option<crate::macro_action::MacroRun> {
        self.retired_macro_runs.pop_front()
    }

    /// True when the supervisor has guaranteed off-thread retirement capacity (or unconditional
    /// RT spare recycling) for a new `PolicyRun` so ending it on `armd-control` never frees heap
    /// fields on the RT thread.
    #[cfg(feature = "policy-runtime")]
    pub fn can_admit_policy_run(&self) -> bool {
        if self.rt_executor_owned {
            self.policy_run.is_none() && self.policy_run_spare.is_some()
        } else {
            true
        }
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn record_pending_policy_close(&mut self, action_id: u64) {
        if action_id > self.pending_policy_close_max {
            self.pending_policy_close_max = action_id;
        }
    }

    #[cfg(feature = "policy-runtime")]
    pub(crate) fn has_pending_policy_close(&self) -> bool {
        self.pending_policy_close_max != 0
    }

    /// Flushes any pending post-publication policy stream close into the durable atomic watermark
    /// (zero allocations, zero drop risk). Returns the flushed high-water mark (0 if none).
    #[cfg(feature = "policy-runtime")]
    pub(crate) fn flush_pending_policy_close_watermark(&mut self) -> u64 {
        let max_id = std::mem::replace(&mut self.pending_policy_close_max, 0);
        if max_id != 0 {
            self.atomic_closed_policy_action_id
                .fetch_max(max_id, Ordering::Release);
        }
        max_id
    }

    /// Test-only helper: initializes RT executor mode, preallocated `PolicyRun` spare, and
    /// triple-buffer observation prewarm on a standalone `SupervisorInner` for counting-allocator
    /// tests (`tests/test_target_consume_alloc.rs`).
    #[doc(hidden)]
    #[cfg(feature = "policy-runtime")]
    pub fn init_rt_policy_spare_for_test(
        &mut self,
        boot: crate::rt_core::broker::BootId,
    ) -> crate::policy_runtime::PolicyAuthorization {
        self.rt_executor_owned = true;
        self.boot_id = boot;
        self.policy_run_spare = Some(crate::policy_runtime::PolicyRun::preallocated_rt_spare(
            self.rt_policy_auth.clone(),
        ));
        self.prewarm_rt_buffers();
        self.rt_policy_auth.clone()
    }

    /// Queue a finished `PolicyRun` for off-thread drop when owned by the RT executor
    /// (prevents dropping heap strings/vectors on `armd-control`).
    #[cfg(feature = "policy-runtime")]
    pub(crate) fn retire_policy_run(&mut self, run: crate::policy_runtime::PolicyRun) {
        if self.rt_executor_owned
            && self.retired_policy_runs.len() < self.retired_policy_runs.capacity()
        {
            self.retired_policy_runs.push_back(run);
        }
    }

    /// Number of retired `PolicyRun` instances currently queued for off-thread drop.
    #[cfg(feature = "policy-runtime")]
    pub fn retired_policy_runs_len(&self) -> usize {
        self.retired_policy_runs.len()
    }

    /// Pop one retired `PolicyRun` for off-thread drop (0 alloc, 0 drop).
    #[cfg(feature = "policy-runtime")]
    pub fn pop_retired_policy_run(&mut self) -> Option<crate::policy_runtime::PolicyRun> {
        self.retired_policy_runs.pop_front()
    }

    /// Current exact count of suppressed `"Stopped hold output failed"` log events pending the
    /// next 1/s log window (or deferred-ring retry).
    pub fn hold_tx_error_suppressed(&self) -> u64 {
        self.hold_tx_error_suppressed
    }

    /// Per-`(joint, slot)` (`index = joint * 2 + slot`) consecutive stopped-hold ticks without
    /// reaching the transport.
    pub fn hold_tx_no_progress_ticks(&self) -> &[u32] {
        &self.hold_tx_no_progress_ticks
    }

    /// Return an `UnloadSettle` buffer to `unload_settle_spare` (resetting its logical state
    /// without freeing its preallocated sample ring; any overflow is retired off-thread on RT).
    pub(crate) fn clear_unload_settle(&mut self) {
        if let Some(mut settle) = self.unload_settle.take() {
            settle.pending_s = 0.0;
            settle.held_s = 0.0;
            settle.held_since_s = None;
            settle.samples.clear();
            settle.last_report_s = None;
            if self.unload_settle_spare.is_none() {
                self.unload_settle_spare = Some(settle);
            } else {
                self.push_retired_stop(crate::rt_core::urgent::RetiredStopPayloads {
                    unload_settle: Some(settle),
                    ..Default::default()
                });
            }
        }
    }

    /// One control tick of the unload settle: the static resting-pose conditions must hold
    /// continuously for [`UNLOAD_SETTLE_HOLD`] and the arm must be stationary over that
    /// window ([`Self::unload_still_violation`]); then torque is cut. A settle pending past
    /// the hold logs which condition blocks it, at most once per second.
    pub(super) fn step_unload_settle(&mut self, dt: f64) {
        let cap = ((UNLOAD_SETTLE_HOLD.as_secs_f64() * f64::from(self.config.control_frequency_hz.max(1)))
            .ceil() as usize
            + 16)
            .max(64);
        let settle_opt = self
            .unload_settle
            .take()
            .or_else(|| self.unload_settle_spare.take())
            .or_else(|| (!self.rt_executor_owned).then(|| UnloadSettle::with_capacity(cap)));
        let Some(mut settle) = settle_opt else {
            // Fail-closed on RT without allocating if spare was ever exhausted: hold resting pose.
            let g_idx = self.config.gripper_index();
            let saved_gripper = g_idx.and_then(|g| self.commanded_positions.get(g).copied());
            copy_into(&mut self.commanded_positions, self.config.resting_positions());
            if let (Some(g), Some(pos)) = (g_idx, saved_gripper) {
                if let Some(slot) = self.commanded_positions.get_mut(g) { *slot = pos; }
            }
            copy_into(&mut self.target_positions, &self.commanded_positions);
            self.commanded_velocities.fill(0.0);
            return;
        };
        settle.pending_s += dt;
        let rt_mode = self.rt_executor_owned;
        let static_ok = if rt_mode {
            match self.check_resting_pose_rt(true) {
                RtRestingCheck::AtRest => true,
                RtRestingCheck::OffRest => false,
                RtRestingCheck::LockContended => {
                    self.record_disable_lock_contention();
                    false
                }
            }
        } else {
            self.is_resting_pose_static_ok()
        };
        if static_ok {
            let since = *settle.held_since_s.get_or_insert(settle.pending_s);
            settle.held_s = settle.pending_s - since;
            let oldest = settle.pending_s - UNLOAD_SETTLE_HOLD.as_secs_f64() - 1e-6;
            while settle.samples.front().is_some_and(|s| s.t < oldest) { settle.samples.pop_front(); }
            if settle.samples.capacity() > 0 && settle.samples.len() == settle.samples.capacity() {
                settle.samples.pop_front();
            }
            settle.samples.push_back(UnloadSettleSample::from_slices(
                settle.pending_s,
                &self.measured_positions,
                &self.measured_velocities,
            ));
        } else {
            settle.held_s = 0.0;
            settle.held_since_s = None;
            settle.samples.clear();
        }
        let settled_ok = static_ok
            && if rt_mode {
                self.unload_still_ok_rt(&settle)
            } else {
                self.is_unload_still_ok(&settle)
            };
        if settled_ok {
            let pending_s = settle.pending_s;
            // Always recycle `settle` into `unload_settle_spare` (zero drop on RT).
            self.unload_settle = Some(settle);
            self.clear_unload_settle();
            self.pending_normal_unload = false;
            if !rt_mode {
                tracing::info!(
                    "Arm settled cleanly at home position for {} ms after {:.2} s; safely cutting torque to Standby/Idle.",
                    UNLOAD_SETTLE_HOLD.as_millis(), pending_s
                );
            }
            let watchdog_park = self.watchdog_park.take().is_some();
            let park_action_id = self
                .last_action
                .as_ref()
                .filter(|a| a.is_running() && a.kind == "park")
                .map(|a| a.id);
            self.begin_park_disable_transition(watchdog_park, park_action_id);
            return;
        }
        if !rt_mode {
            let report_after = UNLOAD_SETTLE_HOLD.as_secs_f64() + UNLOAD_SETTLE_REPORT_AFTER_S;
            if settle.pending_s >= report_after && settle.last_report_s.is_none_or(|t| settle.pending_s - t >= 1.0) {
                let blocker = self
                    .resting_pose_static_violation()
                    .or_else(|| self.unload_still_violation(&settle))
                    .unwrap_or_else(|| "settle pending".into());
                tracing::warn!("Unload settle pending for {:.1} s: {blocker}", settle.pending_s);
                settle.last_report_s = Some(settle.pending_s);
            }
        }
        // Not yet stably at rest: hold the home target in place (0 alloc). Gripper keeps its
        // command: driving it to the home value would squeeze anything still in the jaw.
        let g_idx = self.config.gripper_index();
        let saved_gripper = g_idx.and_then(|g| self.commanded_positions.get(g).copied());
        copy_into(&mut self.commanded_positions, self.config.resting_positions());
        if let (Some(g), Some(pos)) = (g_idx, saved_gripper) {
            if let Some(slot) = self.commanded_positions.get_mut(g) {
                *slot = pos;
            }
        }
        copy_into(&mut self.target_positions, &self.commanded_positions);
        self.commanded_velocities.fill(0.0);
        self.unload_settle = Some(settle);
    }

    /// Blocker of a pending unload settle, for watchdog reporting.
    pub(super) fn unload_settle_blocker(&self) -> Option<String> {
        if self.active_trajectory.is_some() { return Some("park trajectory still running".into()); }
        self.resting_pose_static_violation()
            .or_else(|| self.unload_settle.as_ref().and_then(|s| self.unload_still_violation(s)))
    }

    /// Whether stopped-hold TX stalling is allowed to install `CAN_TX_STALLED`.
    /// Never overwrites `EStop`, `ThermalTrip`, `MitOwner::Lost`, an already-latched
    /// `CanTxStalled`, or a saturated off-thread reclamation queue.
    pub(super) fn can_escalate_to_can_tx_stalled(&self) -> bool {
        if self.hold_tx_stalled_latched || self.e_stop_latched.load(Ordering::SeqCst) {
            return false;
        }
        if self.safety.has_latched_fault_kind(crate::safety::FaultKind::Estop)
            || self.safety.has_latched_fault_kind(crate::safety::FaultKind::ThermalTrip)
            || self.safety.has_latched_fault_kind(crate::safety::FaultKind::CanTxStalled)
        {
            return false;
        }
        if matches!(
            self.safety.state(),
            SafetyState::EStop
                | SafetyState::Fault(FaultReason::ThermalTrip { .. } | FaultReason::CanTxStalled)
        ) {
            return false;
        }
        if self.mit.owner == crate::mit::MitOwner::Lost {
            return false;
        }
        self.safety.can_accept_fault_transition()
    }

    /// Attach a driver, bind tick-epilogue ownership of the single J7 `PosForce` emitter, wire
    /// the stopped-disable gate, cache its lock-free clock, gripper capabilities, and
    /// terminal-capture-only initial snapshot (without mutating hardware park/torque-off
    /// interlock `last_poll_*` fields before the first poll), capture the owner driver snapshot,
    /// and initialize owner torque intent to `On` (R10 hold-by-default).
    pub(crate) fn install_driver(&mut self, mut driver: Box<dyn ArmDriver>) {
        driver.set_estop_gate(self.estop_gate());
        driver.pos_force_mut().set_external_pos_force_emitter(true);
        driver.set_stopped_disable_gate(self.stopped_disable_gate());
        self.driver_clock = driver.driver_clock();
        self.last_valid_motor_rx = [None; crate::model::MAX_MOTION_DOF];
        self.pending_driver_bus_loss_notify = false;
        let caps = driver.capabilities();
        self.driver_has_torque_feedback = caps.torque_feedback;
        self.driver_has_load_feedback = caps.load_feedback;
        if let Some(pf) = self
            .config
            .gripper_index()
            .and_then(|g| driver.pos_force().gripper_pos_force_info().filter(|info| info.joint == g))
        {
            self.driver_gripper_close_speed_limit =
                (pf.close_speed_rad_s.is_finite() && pf.close_speed_rad_s > 0.0)
                    .then_some(pf.close_speed_rad_s);
            self.driver_gripper_ceiling_torque_estimate = pf
                .ceiling_torque_estimate_nm
                .filter(|v| v.is_finite() && *v > 0.0);
        }
        let driver_valid = driver.is_feedback_valid();
        self.driver_feedback_confirmed_cached = driver_valid;
        let poll_before = self.cached_driver_clock_now();
        let mut ages = Vec::with_capacity(self.config.dof);
        driver.feedback_ages_into(&mut ages);
        let mut cache = terminal_capture::CaptureDriverCache {
            enable_state: EnableState::Unknown,
            driver_caps: caps,
            feedback_valid: driver_valid,
            ..terminal_capture::CaptureDriverCache::default()
        };
        cache.copy_ages_from(&ages, self.config.dof, poll_before);
        self.capture_driver_cache = cache;
        self.refresh_owner_driver_snapshot_from_ref(driver.as_ref());
        let dof = self.config.dof;
        if self.motor_errors.capacity() < dof {
            self.motor_errors.reserve(dof - self.motor_errors.len());
        }
        if driver.has_hardware_fault() {
            let mut status = Vec::with_capacity(dof);
            if driver.motor_status_into(&mut status) {
                self.motor_errors.resize_with(status.len(), || None);
                for (slot, &code) in self.motor_errors.iter_mut().zip(status.iter()) {
                    *slot = driver
                        .decode_motor_error_static(code)
                        .map(std::borrow::Cow::Borrowed)
                        .or_else(|| driver.decode_motor_error(code).map(std::borrow::Cow::Owned))
                        .map(|name| crate::model::MotorError { code, name });
                }
            }
        }
        self.torque_intent = crate::supervisor::TorqueIntent::On;
        #[cfg(not(target_arch = "wasm32"))]
        {
            driver.shadow_mut().set_shadow_enabled(self.torque_shadow.is_enabled());
            let boot_sync = crate::torque::shadow::LiveBootSync {
                live_is_torqued: self.is_torqued,
                live_stopped: self.safety.state().is_stopped(),
                estop_latched: self.e_stop_latched.load(Ordering::SeqCst)
                    || matches!(self.safety.state(), SafetyState::EStop),
                has_fault: matches!(self.safety.state(), SafetyState::Fault(_)),
                failed_hold: self.enable_lifecycle
                    == driver_transitions::EnableLifecycle::FailedHold,
            };
            self.torque_shadow.reinit_for_driver_synced(
                &self.config,
                driver.shadow().shadow_vendor_kind(),
                driver.driver_clock(),
                boot_sync,
            );
            self.torque_shadow.drain_from_driver(&mut *driver);
            self.torque_shadow.flush_deferred_steps();
        }
        self.driver = Some(crate::supervisor_lock::DriverSlot::legacy_with_counter(
            driver,
            Arc::clone(&self.driver_lock_attempts),
        ));
    }

    /// True when any joint currently reports a decoded motor hardware error in `self.motor_errors`.
    /// Lock-free and zero-allocation; safe to call on RT authority checks and under driver locks.
    #[inline]
    pub(crate) fn has_reported_motor_errors(&self) -> bool {
        self.motor_errors.iter().any(Option::is_some)
    }

    /// Flush all pending shadow reducer steps and emit compact SPSC records (outer post-write boundary only).
    #[inline]
    pub(crate) fn flush_torque_shadow(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow.flush_deferred_steps();
    }

    /// Flush deferred torque shadow steps immediately when `urgent`, when the staging FIFO is
    /// nearing half capacity, or when at least `margin` remains before `release_deadline`.
    /// Otherwise defers the non-urgent flush to the next release so it cannot push this release
    /// past its deadline. Returns `true` if a flush ran.
    #[inline]
    #[doc(hidden)]
    pub fn flush_torque_shadow_if_budget(
        &mut self,
        urgent: bool,
        now: Instant,
        release_deadline: Instant,
        margin: Duration,
    ) -> bool {
        #[cfg(not(target_arch = "wasm32"))]
        {
            if !self.torque_shadow.has_pending_steps() {
                return false;
            }
            if urgent
                || self.torque_shadow.should_flush_urgently()
                || release_deadline.saturating_duration_since(now) >= margin
            {
                self.torque_shadow.flush_deferred_steps();
                return true;
            }
            false
        }
        #[cfg(target_arch = "wasm32")]
        {
            let _ = (urgent, now, release_deadline, margin);
            false
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[inline]
    pub(crate) fn shadow_live_intent(&self) -> crate::torque::TorqueIntent {
        match self.torque_intent {
            crate::supervisor::TorqueIntent::On => crate::torque::TorqueIntent::On,
            crate::supervisor::TorqueIntent::Off => crate::torque::TorqueIntent::Off,
        }
    }

    /// Test-only shadow fault injection setter on `SupervisorInner` (never exposed via HTTP/WS/CLI/env/JSON).
    #[cfg(not(target_arch = "wasm32"))]
    #[doc(hidden)]
    pub fn test_set_torque_shadow_fault(&mut self, fault: crate::torque::shadow::ShadowFaultInjection) {
        self.torque_shadow.set_injected_fault(fault);
    }

    /// Test-only synchronous flush + drain on `SupervisorInner`.
    #[cfg(not(target_arch = "wasm32"))]
    #[doc(hidden)]
    pub fn test_flush_and_drain_torque_shadow(&mut self) {
        self.torque_shadow.flush_and_drain_sync();
    }

    /// Test-only flush of pending shadow steps without blocking on the off-thread consumer.
    #[cfg(not(target_arch = "wasm32"))]
    #[doc(hidden)]
    pub fn torque_shadow_flush_for_test(&mut self) {
        self.flush_torque_shadow();
    }

    /// Test-only entrypoint invoking `record_post_panic_disable_shadow` directly (for hard-RT
    /// alloc/lock proofs on `armd-control`).
    #[cfg(not(target_arch = "wasm32"))]
    #[doc(hidden)]
    pub fn test_record_post_panic_disable_shadow(
        &mut self,
        is_caught_panic: bool,
        outcome: &crate::supervisor::PanicDisableOutcome,
    ) {
        self.record_post_panic_disable_shadow(is_caught_panic, outcome);
    }

    /// Test-only conversion of `SupervisorInner`'s mounted `DriverSlot::Legacy` into
    /// `DriverSlot::Exclusive` (for hard-RT teardown proofs without spawning the executor thread).
    #[doc(hidden)]
    pub fn test_into_exclusive_driver(&mut self) -> Result<(), &'static str> {
        if let Some(slot) = self.driver.take() {
            match slot.into_exclusive() {
                Ok(ex) => {
                    self.driver = Some(ex);
                    Ok(())
                }
                Err((orig, reason)) => {
                    self.driver = Some(orig);
                    Err(reason)
                }
            }
        } else {
            Ok(())
        }
    }

    /// Test-only read of the `DriverSlot` mutex lock-attempt counter on `SupervisorInner`.
    #[doc(hidden)]
    pub fn test_driver_lock_attempts(&self) -> u64 {
        self.driver_lock_attempts.load(Ordering::Acquire)
    }

    /// Test-only setter for torque shadow owner attribution (mirrors `handoff_to_executor`).
    #[cfg(not(target_arch = "wasm32"))]
    #[doc(hidden)]
    pub fn test_set_torque_shadow_owner(&mut self, owner: crate::torque::shadow::ShadowOwner) {
        self.flush_torque_shadow();
        self.torque_shadow.set_owner(owner);
    }

    /// Read current torque shadow diagnostics from `SupervisorInner`'s paired hub.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn torque_shadow_diagnostics(&self) -> crate::torque::shadow::TorqueShadowDiagnostics {
        self.torque_shadow.hub().map_or_else(
            || crate::torque::shadow::TorqueShadowDiagnostics::disabled(self.torque_shadow.owner()),
            |h| h.snapshot(),
        )
    }

    /// Lock-free Step-2 cached hardware feedback confirmation (`false` when driverless, lock failed,
    /// or last Step-2 poll failed — fail-closed without taking the driver mutex in Step 3/4).
    #[cfg(feature = "policy-runtime")]
    #[inline]
    pub(crate) fn rt_driver_feedback_confirmed(&self) -> bool {
        self.driver_feedback_confirmed_cached
    }

    /// Pushes an immutable owner terminal outcome for an executor-owned policy run into the
    /// lock-free `rtrb` ring consumed off-thread by `PolicyQualificationCoordinator` (0 RT allocs).
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    #[inline]
    pub(crate) fn push_rt_policy_terminal(
        &mut self,
        action_id: u64,
        steps: u64,
        stop_reason: &str,
        action_state: &str,
        stats: crate::motion::PolicyStats,
    ) {
        if let Some(tx) = self.policy_terminal_tx.as_mut() {
            tx.push(crate::rt_core::requests::policy::PolicyTerminalOutcome {
                action_id,
                steps,
                stop_reason: crate::model::HolderLabel::new(stop_reason),
                action_state: crate::model::HolderLabel::new(action_state),
                stats,
            });
        }
    }

    /// Lock-free read of the driver clock for RT off-thread forwarding timestamps, falling back
    /// to a non-blocking driver try-lock or `fallback` when no lock-free driver clock is attached.
    pub(crate) fn driver_clock_now_or(&self, fallback: Instant) -> Instant {
        self.driver_clock
            .as_ref()
            .map(|c| c.now())
            .or_else(|| {
                self.driver
                    .as_ref()
                    .and_then(|d| d.try_with_ref(|drv| drv.clock_now()).ok())
            })
            .unwrap_or(fallback)
    }

    /// Consumes any pending `"Stopped hold output failed"` aggregate retained due to a full
    /// `safety_log_tx` ring so RT executor teardown can deliver it off-thread without losing count.
    pub(crate) fn take_pending_hold_tx_error_for_shutdown(
        &mut self,
    ) -> Option<crate::safety::SafetyLogEvent> {
        if self.hold_tx_error_suppressed == 0 {
            return None;
        }
        let (partial, message, logged_at) = self.pending_hold_tx_error_retry.take()?;
        let suppressed = self.hold_tx_error_suppressed - 1;
        self.hold_tx_error_suppressed = 0;
        Some(crate::safety::SafetyLogEvent::StoppedHoldOutputFailed {
            partial,
            suppressed,
            message,
            logged_at,
        })
    }

    /// Rate-limit stopped-hold backlog stall diagnostics to at most once per second on the driver
    /// clock (independent of `"Stopped hold output failed"` throttling).
    pub(super) fn should_log_hold_tx_stall(&mut self, now: Instant) -> bool {
        let should = self
            .last_hold_tx_stall_log
            .is_none_or(|last| now.saturating_duration_since(last) >= Duration::from_secs(1));
        if should {
            self.last_hold_tx_stall_log = Some(now);
        }
        should
    }

    /// Rate-limit `"Stopped hold output failed"` logs to at most 1/s on the driver clock across
    /// both Legacy (direct tracing) and RT Executor (zero-alloc deferred safety log) paths,
    /// maintaining an exact suppressed-event counter even under inline or downstream ring
    /// saturation (rolling back the throttle window if downstream `safety_log_tx` rejects).
    pub(crate) fn log_stopped_hold_output_failed_throttled(&mut self, now: Instant, partial: bool, message: &str) {
        if self
            .last_hold_tx_error_log
            .is_some_and(|last| now.saturating_duration_since(last) < Duration::from_secs(1))
        {
            self.hold_tx_error_suppressed = self.hold_tx_error_suppressed.saturating_add(1);
            return;
        }
        let mut msg = crate::safety::BoundedText::<160>::empty();
        msg.push_str(message);
        if self
            .safety
            .log_stopped_hold_output_failed(partial, self.hold_tx_error_suppressed, msg, now)
        {
            self.hold_tx_error_suppressed = 0;
            self.pending_hold_tx_error_retry = None;
            self.last_hold_tx_error_log = Some(now);
            if !self.safety.deferred_logging() {
                self.last_hold_tx_error_delivered = Some(now);
            }
        } else {
            // Deferred safety-log ring is full of unrelated events: retain this occurrence so no
            // count is lost, and leave `last_hold_tx_error_log` unchanged for immediate retry.
            self.hold_tx_error_suppressed = self.hold_tx_error_suppressed.saturating_add(1);
            self.pending_hold_tx_error_retry = Some((partial, msg, now));
        }
    }

    /// Reset all per-joint non-stopped TX lack-of-progress counters and timestamps.
    pub(crate) fn reset_tx_joint_progress(&mut self) {
        self.tx_joint_no_progress_ticks.fill(0);
        self.tx_joint_no_progress_since.fill(None);
    }

    /// Reset all per-`(joint, slot)` stopped-hold TX lack-of-progress counters and timestamps.
    pub(crate) fn reset_hold_tx_joint_progress(&mut self) {
        self.hold_tx_no_progress_ticks.fill(0);
        self.hold_tx_no_progress_since.fill(None);
    }

    /// True when an active position-force gripper (`damiao_gripper.mode == PosForce`) currently
    /// has stale (`> 100 ms`) or missing feedback. Uses non-blocking `try_lock()` so callers on
    /// `armd-control` never block on a mutex (falling back to `self.j7_stale_since.is_some()` on
    /// contention).
    pub(crate) fn is_pos_force_j7_stale(&self) -> bool {
        if !self.config.damiao_gripper.as_ref().is_some_and(|g| g.mode == crate::model::DamiaoGripperMode::PosForce) {
            return false;
        }
        let Some(d) = self.driver.as_ref() else {
            return false;
        };
        d.try_with_ref_recover(|drv| !drv.pos_force().pos_force_feedback_fresh())
            .unwrap_or_else(|| self.j7_stale_since.is_some())
    }

    /// Last provably fresh (`<= 100 ms`) J7 position reported by the driver (in user coordinates),
    /// falling back to finite `measured_positions[g]` or `commanded_positions[g]`. Zero-lock in
    /// `DriverSlot::Exclusive`, non-blocking `try_lock` in `DriverSlot::Legacy`.
    pub(crate) fn pos_force_j7_freeze_position(&self) -> Option<f32> {
        let g = self.config.gripper_index()?;
        let raw = self
            .driver
            .as_ref()
            .and_then(|d| d.try_with_ref_recover(|drv| drv.pos_force().pos_force_last_fresh_position()).flatten())
            .or_else(|| self.measured_positions.get(g).copied().filter(|p| p.is_finite()))
            .or_else(|| self.commanded_positions.get(g).copied().filter(|p| p.is_finite()))?;
        Some(self.config.limits.get(g).map_or(raw, |l| raw.clamp(l.min_position, l.max_position)))
    }

    /// True when the active trajectory drives the gripper (`Reference::Gripper`, or
    /// `Reference::Replay` with `gripper == true` and a configured gripper joint).
    pub(crate) fn active_reference_drives_gripper(&self) -> bool {
        match self.active_trajectory.as_ref().map(|a| &a.reference) {
            Some(Reference::Gripper(_)) => true,
            Some(Reference::Replay(r)) => r.gripper && r.gripper_idx.is_some(),
            _ => false,
        }
    }

    /// Zero-allocation admission check refusing new gripper targets while active `PosForce` J7
    /// feedback is stale (`> 100 ms`). Single source of truth wrapped by
    /// [`Self::check_pos_force_gripper_fresh`].
    pub(crate) fn check_pos_force_gripper_fresh_static(&self) -> Result<(), &'static str> {
        if let Some(gate) = self.rt_j7_pf_gate {
            return if gate.stale { Err("J7_FEEDBACK_STALE") } else { Ok(()) };
        }
        if self.is_pos_force_j7_stale() {
            return Err("J7_FEEDBACK_STALE");
        }
        Ok(())
    }
}
