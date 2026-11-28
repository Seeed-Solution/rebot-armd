//! Legacy 50 Hz control loop, commissioning post-tick stepper, and `tick_shared`.

#[allow(unused_imports)]
use super::*;

// scaffold-extra-0
impl ArmSupervisor {
    /// Shared Legacy `armd-control` loop body with tiered panic recovery (FU-PANIC / DRV-F1):
    /// - Enters [`crate::panic_coord::ControlPanicScope`] so the process panic hook routes
    ///   control-thread panics to local `catch_unwind` -> SAFE-HOLD.
    /// - Preallocates [`crate::rt_core::safe_hold::SafeHoldBuffers`] before the loop and mirrors
    ///   last fresh measured positions after every clean tick.
    /// - On an unwind panic inside a legacy control tick while a hardware driver is attached and
    ///   not confirmed disarmed: fences all commands/rearm, extracts the `Box<dyn ArmDriver>`
    ///   exclusively ONCE (recovering poison if needed), and transitions into the zero-alloc,
    ///   zero-lock SAFE-HOLD loop until thermal trip or deliberate `ProcessExit` protective disable.
    pub fn run_legacy_control_loop_blocking(
        self: &Arc<Self>,
        period: Duration,
        control_stop: Arc<AtomicBool>,
        loop_stats: Arc<crate::rt_core::executor::ExecutorStats>,
        runtime: tokio::runtime::Handle,
        panic_hook_step: Option<Arc<AtomicU64>>,
    ) {
        let _panic_scope = crate::panic_coord::ControlPanicScope::enter();
        self.register_legacy_control_stop(Arc::clone(&control_stop));
        let cfg = runtime.block_on(async { self.inner().read().await.config.clone() });
        let mut safe_hold_bufs =
            crate::rt_core::safe_hold::SafeHoldBuffers::from_config(&cfg, period);
        {
            let guard = runtime.block_on(self.inner.read());
            safe_hold_bufs.attach_stop_mirrors(
                Arc::clone(&self.estop_state),
                Arc::clone(&guard.estop_applied),
                guard.stop_requests.clone(),
            );
        }
        let safe_hold_ctrl = Arc::clone(&self.legacy_safe_hold);
        #[cfg(not(target_arch = "wasm32"))]
        let mut disabled_shadow_spare = Some(crate::torque::shadow::ShadowRuntime::disabled(&cfg));
        let mut step_counter: u64 = 0;
        let mut panicked = false;
        let mut handoff_driver: Option<Box<dyn ArmDriver>> = None;
        #[cfg(not(target_arch = "wasm32"))]
        let mut handoff_shadow: Option<crate::torque::shadow::ShadowRuntime> = None;
        let mut handoff_reason: Option<crate::rt_core::safe_hold::SafeHoldReason> = None;

        crate::rt_core::executor::run_periodic_with_deadline(
            period,
            &control_stop,
            &loop_stats,
            |dt, release_deadline| {
                step_counter = step_counter.saturating_add(1);
                let mut extracted: Option<(
                    Box<dyn ArmDriver>,
                    Option<crate::torque::shadow::ShadowRuntime>,
                    crate::rt_core::safe_hold::SafeHoldReason,
                )> = None;
                let tick_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    if panic_hook_step
                        .as_ref()
                        .is_some_and(|target| target.load(Ordering::Acquire) == step_counter)
                    {
                        panic!("simulated legacy control thread panic at tick {step_counter}");
                    }
                    runtime.block_on(self.tick_shared_with_handoff(
                        dt,
                        false,
                        Some(release_deadline),
                        Some((
                            &mut safe_hold_bufs,
                            &control_stop,
                            &mut disabled_shadow_spare,
                            &mut extracted,
                        )),
                    ));
                }));
                match tick_res {
                    Ok(()) => {
                        if let Some((drv, sh, reason)) = extracted {
                            handoff_driver = Some(drv);
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                handoff_shadow = sh;
                            }
                            #[cfg(target_arch = "wasm32")]
                            let _ = sh;
                            handoff_reason = Some(reason);
                            control_stop.store(true, Ordering::Release);
                        }
                    }
                    Err(_) => {
                        panicked = true;
                        crate::rt_core::executor::account_periodic_overrun_on_panic_exit(
                            &loop_stats,
                        );
                        self.estop_state.latch();
                        self.internal_fault_lane.latch_control_safe_hold_fence();
                        tracing::error!(
                            "Legacy control thread panicked during tick {step_counter}; transitioning to SAFE-HOLD"
                        );
                        control_stop.store(true, Ordering::Release);
                    }
                }
            },
        );

        // Unified Legacy loop exit disposition (DRV-F1 B1/B2/B5 + PR0C Blocker 7):
        // - If `tick_shared_with_handoff` already extracted the driver directly under its own
        //   single write guard (e.g. `SessionUncertainState`), enter SAFE-HOLD immediately with
        //   zero extra lock attempts and zero contention delay.
        // - Otherwise handle panic / teardown-while-energized via bounded extraction.
        if let Some(mut drv) = handoff_driver {
            let reason = handoff_reason
                .unwrap_or(crate::rt_core::safe_hold::SafeHoldReason::SessionUncertainState);
            #[cfg(not(target_arch = "wasm32"))]
            let _ = safe_hold_bufs.run_loop_with_shadow(
                drv.as_mut(),
                &safe_hold_ctrl,
                reason,
                Instant::now,
                std::thread::sleep,
                handoff_shadow.as_mut(),
            );
            #[cfg(target_arch = "wasm32")]
            let _ = safe_hold_bufs.run_loop(
                drv.as_mut(),
                &safe_hold_ctrl,
                reason,
                Instant::now,
                std::thread::sleep,
            );
            return;
        }
        {
            let reason = if panicked {
                crate::rt_core::safe_hold::SafeHoldReason::ControlPanic
            } else {
                crate::rt_core::safe_hold::SafeHoldReason::TeardownWhileEnergized
            };
            if panicked {
                safe_hold_ctrl.mark_active(reason);
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow.wake();
            }
            let mut extracted_driver: Option<Box<dyn ArmDriver>> = None;
            #[cfg(not(target_arch = "wasm32"))]
            let mut extracted_shadow: Option<crate::torque::shadow::ShadowRuntime> = None;
            loop {
                if let Ok(mut guard) = self.inner.try_write() {
                    if !guard.detached && !guard.shutdown_committed {
                        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            #[cfg(not(target_arch = "wasm32"))]
                            guard.on_safe_hold_enter(if panicked {
                                "CONTROL_PANIC"
                            } else {
                                "SAFE_HOLD"
                            });
                            safe_hold_bufs.seed_on_entry(&guard);
                            #[cfg(not(target_arch = "wasm32"))]
                            if panicked {
                                let live_intent = guard.shadow_live_intent();
                                let live_is_torqued = guard.is_torqued;
                                guard.torque_shadow.enqueue(
                                    crate::torque::shadow::ShadowInputKind::Panic {
                                        tier: crate::torque::PanicTier::CaughtControlPanic,
                                        live_intent,
                                        live_is_torqued,
                                        live_stopped: true,
                                    },
                                );
                                guard.flush_torque_shadow();
                            }
                        }));
                        if !crate::rt_core::executor::core_is_confirmed_disarmed(&guard) {
                            self.estop_state.latch();
                            self.internal_fault_lane.latch_control_safe_hold_fence();
                            safe_hold_ctrl.mark_active(reason);
                            #[cfg(not(target_arch = "wasm32"))]
                            if !panicked {
                                let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                    let live_intent = guard.shadow_live_intent();
                                    let live_is_torqued = guard.is_torqued;
                                    guard.torque_shadow.enqueue(
                                        crate::torque::shadow::ShadowInputKind::Stop {
                                            kind: crate::torque::StopKind::EStop,
                                            live_intent,
                                            live_is_torqued,
                                            live_stopped: true,
                                        },
                                    );
                                    guard.flush_torque_shadow();
                                }));
                            }
                            extracted_driver = guard
                                .driver
                                .take()
                                .map(crate::supervisor_lock::DriverSlot::into_driver_recover_for_safe_hold);
                            if extracted_driver.is_some() {
                                // Atomically detach the shell AND publish `OwnerSlot::Failed` +
                                // `WriterEndpoint::DetachedSafeHold` inside the same write-guard
                                // transaction so `try_set_driver()` and `start_executor()` can
                                // NEVER admit a second bus writer while SAFE-HOLD runs (DRV-F1 B1).
                                guard.detached = true;
                                #[cfg(not(target_arch = "wasm32"))]
                                {
                                    let disabled =
                                        crate::torque::shadow::ShadowRuntime::disabled(&guard.config);
                                    extracted_shadow = Some(std::mem::replace(
                                        &mut guard.torque_shadow,
                                        disabled,
                                    ));
                                }
                                {
                                    let mut slot =
                                        self.owner.lock().unwrap_or_else(|e| e.into_inner());
                                    *slot = OwnerSlot::Failed;
                                    self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                                }
                                {
                                    let mut w = self
                                        .active_writer
                                        .lock()
                                        .unwrap_or_else(|e| e.into_inner());
                                    *w = WriterEndpoint::DetachedSafeHold {
                                        safe_hold: Arc::clone(&safe_hold_ctrl),
                                        control_stop: Arc::clone(&control_stop),
                                    };
                                }
                            }
                        }
                    }
                    break;
                }
                if safe_hold_ctrl.process_exit_requested() {
                    // Process exit requested while shell write guard is still contended: run
                    // bounded legacy panic disable once the lock frees within the exit deadline.
                    self.process_exit_requested.store(true, Ordering::Release);
                    let dl = safe_hold_ctrl.process_exit_deadline();
                    let rem = dl.saturating_duration_since(Instant::now());
                    let outcome = self.attempt_legacy_panic_disable(rem);
                    safe_hold_ctrl.record_process_exit_outcome_lockfree(&outcome);
                    safe_hold_ctrl.mark_exited();
                    if matches!(
                        outcome,
                        PanicDisableOutcome::Confirmed | PanicDisableOutcome::NoDriver
                    ) {
                        return;
                    }
                    // Unconfirmed/TimedOut/Failed: continue looping to acquire the shell lock and
                    // retain the energized driver in SAFE-HOLD rather than returning to drop it.
                }
                std::thread::sleep(Duration::from_millis(1));
            }
            if let Some(mut drv) = extracted_driver {
                #[cfg(not(target_arch = "wasm32"))]
                let _ = safe_hold_bufs.run_loop_with_shadow(
                    drv.as_mut(),
                    &safe_hold_ctrl,
                    reason,
                    Instant::now,
                    std::thread::sleep,
                    extracted_shadow.as_mut(),
                );
                #[cfg(target_arch = "wasm32")]
                let _ = safe_hold_bufs.run_loop(
                    drv.as_mut(),
                    &safe_hold_ctrl,
                    reason,
                    Instant::now,
                    std::thread::sleep,
                );
                drop(drv);
            } else if panicked {
                safe_hold_ctrl.mark_exited();
            }
        }
    }

    /// Forbidden legacy `DriverLock` acquisition attempts observed on `armd-control`.
    pub fn driver_lock_attempts(&self) -> u64 {
        self.driver_lock_attempts.load(Ordering::Acquire)
    }

    /// Shared atomic counter for positive-control tests verifying `DriverLock` instrumentation.
    #[doc(hidden)]
    pub fn driver_lock_attempts_counter(&self) -> Arc<AtomicU64> {
        Arc::clone(&self.driver_lock_attempts)
    }

    /// Test-only override for legacy WS `/api/v1/torque` request deadline (0 / `None` = default 5 s).
    #[doc(hidden)]
    pub fn set_legacy_torque_deadline_for_test(&self, deadline: Option<Duration>) {
        let ms = deadline.map_or(0, |d| d.as_millis().clamp(1, u128::from(u64::MAX)) as u64);
        self.legacy_torque_deadline_override_ms
            .store(ms, Ordering::Release);
    }

    /// Returns a lock-free handle to `SupervisorInner::control_release_seq_atomic` plus its current
    /// sequence value so off-lock legacy commissioning waiters can detect whether a live 50 Hz
    /// control ticker is actively servicing `service_commissioning_release`.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) async fn legacy_tick_seq_counter(&self) -> Option<(Arc<AtomicU64>, u64)> {
        if self.owner_mode() != OWNER_LEGACY {
            return None;
        }
        let inner = self.inner.read().await;
        if inner.detached {
            return None;
        }
        let counter = Arc::clone(&inner.control_release_seq_atomic);
        let cur = counter.load(Ordering::Acquire);
        Some((counter, cur))
    }

    /// Headless fallback stepper for legacy ticked commissioning when no background 50 Hz control
    /// loop is running (mirrors TQ-LOCK headless fallback). Re-verifies under `SupervisorLock::write`
    /// that `control_release_seq == expected_seq`, then runs at most ONE bounded step (`<= 1 ms`,
    /// max 1 TX frame) without double-subtracting publish margin so write-guard holds stay `<= 2 ms`.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) async fn step_legacy_commissioning_headless(
        &self,
        op_id: u64,
        expected_seq: u64,
    ) -> (u64, bool) {
        // 1 ms cutoff with max 1 TX frame (no second margin subtraction) keeps SG2002 headless
        // progress non-starving while bounding every SupervisorLock::write hold <= 2 ms.
        const LEGACY_HEADLESS_COMM_BUDGET: Duration = Duration::from_millis(1);
        let mut inner = self.inner.write().await;
        let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
        inner.set_command(crate::mit::TIMEOUT_PATH);
        if inner.detached
            || inner.control_release_seq != expected_seq
            || !inner.commissioning.is_running_op(op_id)
        {
            let seq = inner.control_release_seq;
            inner.set_io_timing(io_scope.finish());
            return (seq, false);
        }
        inner.control_release_seq = inner.control_release_seq.wrapping_add(1).max(1);
        inner
            .control_release_seq_atomic
            .store(inner.control_release_seq, Ordering::Release);
        let seq = inner.control_release_seq;
        if let Some(ref mut slot) = inner.driver {
            let _ = slot.try_with_mut(|drv| drv.begin_control_release(seq));
        }
        let step_now = Instant::now();
        let ctx = crate::rt_core::ops::CommitCtx {
            arm: inner.core.id(),
            boot: crate::rt_core::broker::BootId::process(),
            now: step_now,
            stop_generation: inner.current_stop_gen(),
        };
        let mut headless_ledger = crate::rt_core::budget::ReleaseBudget::new(
            seq,
            step_now,
            step_now + LEGACY_HEADLESS_COMM_BUDGET,
            crate::rt_core::ops::RX_FRAME_BUDGET,
            inner.release_tx_budget_limit(),
        );
        let mut sys_now = Instant::now;
        let _ = inner.service_commissioning_step_with_budget(
            &ctx,
            &mut headless_ledger,
            step_now + LEGACY_HEADLESS_COMM_BUDGET,
            1,
            Some(1),
            true,
            &mut sys_now,
        );
        inner.flush_torque_shadow();
        inner.set_io_timing(io_scope.finish());
        (seq, true)
    }

    /// Post-tick legacy commissioning scheduler invoked by [`Self::tick_shared`] strictly AFTER
    /// the control tick's own RX/output/publish write guard has dropped.
    ///
    /// - Derives the release cutoff strictly from `release_deadline - comm_margin` (matching
    ///   TQ-LOCK TX deadline separation, never `tick_wall_start + 1 ms`).
    /// - Splits commissioning I/O across short bounded `SupervisorLock::write` guards (`<= 1.2 ms`
    ///   and max 1 TX frame per guard) so even on SG2002 (1.0–1.5 ms RX drain, 0.4–0.6 ms/frame TX)
    ///   EVERY write-guard hold stays strictly `<= 2 ms` and commissioning makes progress every
    ///   release without delaying control releases.
    /// - Bounds total commissioning register I/O to `COMM_MAX_TX_PER_RELEASE` frames per release
    ///   (and at most one `GatherRestEvidence` `0xCC` burst per release), sleeping only OFF-LOCK
    ///   when awaiting an in-flight reply within the remaining release budget.
    #[cfg(not(target_arch = "wasm32"))]
    pub(super) async fn service_legacy_commissioning_post_tick(
        &self,
        tick_started: Instant,
        release_deadline: Instant,
        comm_margin: Duration,
        tx_budget_dur: Duration,
        mut ledger: crate::rt_core::budget::ReleaseBudget,
    ) {
        const OFF_LOCK_REPLY_POLL_INTERVAL: Duration = Duration::from_millis(1);
        const MAX_OFF_LOCK_POLLS_PER_REQ: u8 = 3;
        const MAX_GUARD_STEPS_PER_RELEASE: u8 = 16;

        let Some(comm_release_cap) = release_deadline.checked_sub(comm_margin) else {
            return;
        };
        // Keep off-lock reply polling strictly inside BOTH `release_deadline - comm_margin` AND
        // the driver's per-release TX pass duration so subsequent same-release sends never hit
        // spurious TX-duration backpressure.
        let comm_wait_cap = comm_release_cap.min(
            tick_started + tx_budget_dur.saturating_sub(Duration::from_millis(1)),
        );

        let mut reg_tx_left: u8 = crate::mit::COMM_MAX_TX_PER_RELEASE;
        let mut probe_tx_left: u8 = crate::rt_core::mailbox::MAX_DOF as u8;
        let mut first_step = true;
        let mut wait_polls_for_req: u8 = 0;
        let mut guard_steps: u8 = 0;
        let mut lock_yields: u8 = 0;

        while guard_steps < MAX_GUARD_STEPS_PER_RELEASE {
            if self.owner_mode() != OWNER_LEGACY
                || self.estop_state.is_latched()
                || self.process_exit_requested.load(Ordering::Acquire)
                || self.internal_fault_lane.is_fenced()
            {
                break;
            }
            // Never acquire `SupervisorLock` for optional commissioning once the release cutoff
            // (`release_deadline - comm_margin`) has elapsed.
            if Instant::now() >= comm_release_cap {
                break;
            }
            // Non-blocking acquisition: optional post-tick commissioning must NEVER queue behind
            // another `SupervisorLock` holder and delay the next 50 Hz control release.
            let Ok(mut inner) = self.inner.try_write() else {
                if lock_yields >= 1 || Instant::now() >= comm_release_cap {
                    break;
                }
                lock_yields = lock_yields.saturating_add(1);
                tokio::task::yield_now().await;
                continue;
            };
            lock_yields = 0;
            let progress = {
                let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                inner.set_command(crate::mit::TIMEOUT_PATH);
                let guard_start = Instant::now();
                if self.owner_mode() != OWNER_LEGACY
                    || guard_start >= comm_release_cap
                    || !inner.legacy_commissioning_eligible()
                {
                    inner.set_io_timing(io_scope.finish());
                    break;
                }
                let ctx = crate::rt_core::ops::CommitCtx {
                    arm: inner.core.id(),
                    boot: crate::rt_core::broker::BootId::process(),
                    now: guard_start,
                    stop_generation: inner.current_stop_gen(),
                };
                // Each guard executes at most 1 commissioning TX frame (keeping guard hold <= 2 ms
                // on SG2002 0.4–0.6 ms/frame TX) while passing the true `comm_release_cap`
                // (`release_deadline - comm_margin`) as the hardware TX/flush cutoff so transient
                // OS jitter never causes false `SentFlushFailed` on non-noop flush transports.
                let step_reg_tx = reg_tx_left.min(1);
                let step_probe_tx = probe_tx_left.min(1);
                let mut sys_now = Instant::now;
                let p = inner.service_commissioning_step_with_budget(
                    &ctx,
                    &mut ledger,
                    comm_release_cap,
                    step_reg_tx,
                    Some(step_probe_tx),
                    first_step,
                    &mut sys_now,
                );
                inner.flush_torque_shadow();
                inner.set_io_timing(io_scope.finish());
                p
            };
            drop(inner);
            first_step = false;
            guard_steps = guard_steps.saturating_add(1);
            if progress.probe_tx_attempted > 0 {
                probe_tx_left = probe_tx_left.saturating_sub(progress.probe_tx_attempted);
            }
            if progress.reg_tx_attempted > 0 {
                reg_tx_left = reg_tx_left.saturating_sub(progress.reg_tx_attempted);
                wait_polls_for_req = 0;
            }
            if !progress.still_running
                || progress.contended
                || progress.backpressured
                || self.estop_state.is_latched()
                || self.owner_mode() != OWNER_LEGACY
            {
                break;
            }
            let now = Instant::now();
            if now >= comm_release_cap {
                break;
            }
            if progress.rest_probe_pending {
                if probe_tx_left == 0 {
                    break;
                }
                tokio::task::yield_now().await;
                continue;
            }
            if progress.more_immediate_work {
                if reg_tx_left == 0 {
                    break;
                }
                tokio::task::yield_now().await;
                continue;
            }
            if progress.awaiting_reply {
                if reg_tx_left == 0 || wait_polls_for_req >= MAX_OFF_LOCK_POLLS_PER_REQ {
                    break;
                }
                if now + OFF_LOCK_REPLY_POLL_INTERVAL + Duration::from_millis(1) >= comm_wait_cap {
                    break;
                }
                wait_polls_for_req = wait_polls_for_req.saturating_add(1);
                tokio::time::sleep(OFF_LOCK_REPLY_POLL_INTERVAL).await;
                continue;
            }
            break;
        }
    }

    /// Execute a control cycle step
    pub async fn tick(&self, dt: f64) -> ArmObservation {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::tick");
        self.tick_shared(dt, true, None).await.expect("tick keeps a copy when asked")
    }

    /// [`Self::tick`] for the control loop, which does not use the observation: saves a copy.
    pub async fn control_tick(&self, dt: f64) {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::control_tick");
        self.tick_shared(dt, false, None).await;
    }

    /// [`Self::control_tick`] bounded by the caller's absolute scheduled release grid deadline
    /// (as passed from [`crate::rt_core::executor::run_periodic_with_deadline`]).
    pub async fn control_tick_with_deadline(&self, dt: f64, release_deadline: Instant) {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::control_tick_with_deadline");
        self.tick_shared(dt, false, Some(release_deadline)).await;
    }

    /// One tick; publishes its observation (`crate::rt_core::publish`) and returns a copy
    /// when `keep`.
    pub(super) async fn tick_shared(
        &self,
        dt: f64,
        keep: bool,
        scheduled_deadline: Option<Instant>,
    ) -> Option<ArmObservation> {
        self.tick_shared_with_handoff(dt, keep, scheduled_deadline, None)
            .await
    }

    #[allow(clippy::type_complexity)]
    pub(super) async fn tick_shared_with_handoff(
        &self,
        dt: f64,
        keep: bool,
        scheduled_deadline: Option<Instant>,
        mut legacy_safe_hold_handoff: Option<(
            &mut crate::rt_core::safe_hold::SafeHoldBuffers,
            &Arc<AtomicBool>,
            &mut Option<crate::torque::shadow::ShadowRuntime>,
            &mut Option<(
                Box<dyn ArmDriver>,
                Option<crate::torque::shadow::ShadowRuntime>,
                crate::rt_core::safe_hold::SafeHoldReason,
            )>,
        )>,
    ) -> Option<ArmObservation> {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::tick_shared");
        let wake_lateness_ns = crate::diagnostics::tick::take_current_wake_lateness_ns();
        let wake_lateness_us = (wake_lateness_ns / 1_000).min(u64::from(u32::MAX)) as u32;
        if self.owner_mode() != OWNER_LEGACY {
            return None;
        }
        use crate::diagnostics::tick::{mid_csw_delta, us, CswMark, ReleasePhasesUs, UsageMark};
        let started = Instant::now();
        let usage_start = UsageMark::now();
        let (
            mut obs,
            captured_any,
            wait,
            control_hz,
            pending_zc,
            pending_qual,
            pending_wd,
            phases,
            log,
            locked,
            usage_locked,
            pre,
            publish,
            mid_csw_start,
            phase_ack_flush,
            phase_terminal,
            lock_end,
            tick_tq_dir,
            tick_io,
            settled_legacy,
            legacy_comm_plan,
        ) = {
            let (mut inner, generation) = self.inner.write_tick().await;
            if inner.detached || self.owner_mode() != OWNER_LEGACY {
                return None;
            }
            let locked = Instant::now();
            let usage_locked = UsageMark::now();
            // Reset allocator counters and enable deferral BEFORE draining pending disconnects so
            // any terminal or allocation inside the entire guarded interval is covered.
            let _ = crate::diagnostics::alloc::take();
            inner.defer_terminal_capture = true;
            inner.defer_zero_check_persist = true;
            inner.defer_watchdog_park_plan = true;
            inner.defer_legacy_commissioning = true;
            if self.pending_disconnect_hold.load(Ordering::Acquire) > 0 {
                let _ = self.drain_pending_disconnects_under_guard(&mut inner, None);
            }
            // No other guard exists now: the log is complete up to the guard waited for.
            let log = self.inner.take_log();
            inner.tick_phases_us = [0; 5];
            inner.last_tick_torque_ws_direction = None;
            let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
            let body_start = Instant::now();
            inner.tick_phase_mark = Some(body_start);
            let release_deadline = scheduled_deadline.unwrap_or_else(|| {
                let period = Duration::from_secs_f64(1.0 / f64::from(inner.config.control_frequency_hz.max(1)));
                started + period
            });
            let TickPublished {
                kept,
                body_end,
                phase_terminal_us,
                publish_only_us,
                captured_any,
                mid_csw_start,
            } = inner.tick_publish_with_deadline(
                dt,
                generation,
                keep,
                Some(release_deadline),
            );
            let tick_io = io_scope.finish();
            let tick_tq_dir = inner.last_tick_torque_ws_direction.take();
            let _ = body_end.unwrap_or(body_start);
            let phases = inner.tick_phases_us;
            let _ = inner.take_phase_terminal_us();
            inner.defer_terminal_capture = false;
            inner.defer_zero_check_persist = false;
            inner.defer_watchdog_park_plan = false;
            inner.defer_legacy_commissioning = false;
            // Blocker 7: Mirror SAFE-HOLD buffers AND perform atomic zero-contention driver
            // extraction directly inside the single tick write guard when a session requests
            // SAFE-HOLD — eliminating post-tick `try_read()` and `try_write()` contention.
            if let Some((bufs, control_stop, disabled_shadow_spare, out)) =
                legacy_safe_hold_handoff.as_mut()
            {
                if !inner.detached {
                    bufs.mirror_from_core(&inner);
                    #[cfg(not(target_arch = "wasm32"))]
                    if inner.sessions.is_safe_hold_requested()
                        && !inner.shutdown_committed
                        && !crate::rt_core::executor::core_is_confirmed_disarmed(&inner)
                    {
                        let reason =
                            crate::rt_core::safe_hold::SafeHoldReason::SessionUncertainState;
                        inner.on_safe_hold_enter("SAFE_HOLD");
                        bufs.seed_on_entry(&inner);
                        self.estop_state.latch();
                        self.internal_fault_lane.latch_control_safe_hold_fence();
                        self.legacy_safe_hold.mark_active(reason);
                        let live_intent = inner.shadow_live_intent();
                        let live_is_torqued = inner.is_torqued;
                        inner.torque_shadow.enqueue(
                            crate::torque::shadow::ShadowInputKind::Stop {
                                kind: crate::torque::StopKind::EStop,
                                live_intent,
                                live_is_torqued,
                                live_stopped: true,
                            },
                        );
                        inner.flush_torque_shadow();
                        if let Some(drv) = inner
                            .driver
                            .take()
                            .map(crate::supervisor_lock::DriverSlot::into_driver_recover_for_safe_hold)
                        {
                            inner.detached = true;
                            // R3 Major 7: swap in the preconstructed disabled shadow runtime
                            // (zero heap allocations on armd-control during SAFE-HOLD handoff!).
                            let sh = disabled_shadow_spare
                                .take()
                                .map(|spare| std::mem::replace(&mut inner.torque_shadow, spare));
                            // R2 Major 4: publish atomic `OWNER_FAILED` lock-freely and update
                            // `owner`/`active_writer` via non-blocking `try_lock()` so a contending
                            // off-thread mutex holder can NEVER delay the first SAFE-HOLD hold frame!
                            self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                            if let Ok(mut slot) = self.owner.try_lock() {
                                *slot = OwnerSlot::Failed;
                            }
                            if let Ok(mut w) = self.active_writer.try_lock() {
                                *w = WriterEndpoint::DetachedSafeHold {
                                    safe_hold: Arc::clone(&self.legacy_safe_hold),
                                    control_stop: Arc::clone(control_stop),
                                };
                            }
                            **out = Some((drv, sh, reason));
                        }
                    }
                }
            }
            #[cfg(not(target_arch = "wasm32"))]
            let legacy_comm_plan = if inner.legacy_commissioning_eligible() {
                let comm_ledger = inner.last_legacy_release_budget.take().unwrap_or_else(|| {
                    crate::rt_core::budget::ReleaseBudget::new(
                        inner.control_release_seq,
                        started,
                        release_deadline,
                        crate::rt_core::ops::RX_FRAME_BUDGET,
                        inner.release_tx_budget_limit(),
                    )
                });
                Some((
                    release_deadline,
                    inner.tx_release_publish_margin(),
                    inner.config.damiao_tx_budget().max_duration,
                    comm_ledger,
                ))
            } else {
                inner.last_legacy_release_budget = None;
                None
            };
            #[cfg(target_arch = "wasm32")]
            let legacy_comm_plan: Option<(Instant, Duration, Duration)> = None;
            let pending_zc = inner.pending_zero_check_persist.take();
            #[cfg(feature = "policy-runtime")]
            let pending_qual = inner
                .pending_policy_qual_persist
                .take()
                .map(|(run, reason, state)| {
                    (
                        run,
                        reason,
                        state,
                        inner.policy_ledger_path.clone(),
                        inner.policy_live_allowed(),
                    )
                });
            #[cfg(not(feature = "policy-runtime"))]
            let pending_qual: Option<()> = None;
            let pending_wd = inner.watchdog_park_pending.filter(|p| !p.waiting_mit_handback);
            let control_hz = inner.config.control_frequency_hz;
            let mid_csw_start = mid_csw_start.unwrap_or_else(CswMark::now);
            let after_publish = Instant::now();
            Self::flush_legacy_estop_acks_into(&self.estop_reg, &mut inner);
            let after_ack_flush = Instant::now();
            let settled_legacy =
                std::mem::replace(&mut inner.legacy_torque_settled_this_tick, false);
            let lock_end = Instant::now();
            (
                kept,
                captured_any,
                locked.saturating_duration_since(started),
                control_hz,
                pending_zc,
                pending_qual,
                pending_wd,
                phases,
                log,
                locked,
                usage_locked,
                body_start.saturating_duration_since(locked),
                publish_only_us,
                mid_csw_start,
                us(after_ack_flush.saturating_duration_since(after_publish)),
                phase_terminal_us,
                lock_end,
                tick_tq_dir,
                tick_io,
                settled_legacy,
                legacy_comm_plan,
            )
        };
        let released = Instant::now();
        // Sample allocator totals immediately after write-guard drop so the alloc proof covers
        // disconnect drain, tick control, edge-triggered terminal capture, publish, ACK flush,
        // and write-guard teardown, and `phases_us.alloc_read` measures the actual `alloc::take()`.
        let alloc_read_start = Instant::now();
        let allocs = crate::diagnostics::alloc::take();
        let phase_alloc_read = us(alloc_read_start.elapsed());
        let phase_guard_drop = us(released.saturating_duration_since(lock_end));
        let usage_released = UsageMark::now();
        let lock_held = released.saturating_duration_since(locked);
        if settled_legacy {
            self.legacy_torque_notify.notify_waiters();
        }
        let tick_usage = crate::diagnostics::tick::thread_usage(
            &usage_start,
            &usage_locked,
            &usage_released,
        );
        #[cfg(target_os = "linux")]
        let tick_cpu_hold = Some(Duration::from_nanos(
            usage_released.cpu_ns().saturating_sub(usage_locked.cpu_ns()),
        ));
        #[cfg(not(target_os = "linux"))]
        let tick_cpu_hold = None;
        // Record EVERY control-tick guard into opt-in test history (single Relaxed load when
        // disabled) so slow non-torque ticks are never hidden from lock-proof assertions.
        self.inner.record_tick_hold_for_history(
            std::panic::Location::caller(),
            tick_tq_dir.map(|_| "/api/v1/torque"),
            tick_tq_dir,
            Some(tick_io),
            started,
            locked,
            released,
            tick_cpu_hold,
            tick_usage.lock_vcsw,
            tick_usage.lock_ivcsw,
        );
        let (mid_nvcsw, mid_nivcsw) = mid_csw_delta(&mid_csw_start, &usage_released);
        if captured_any {
            self.hub.wake();
        }
        self.control_period_us.store((1e6 / f64::from(control_hz.max(1))) as u64, Ordering::Relaxed);
        if let Some(wd) = pending_wd {
            if self.watchdog_park_planning.compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire).is_ok() {
                let sup_inner = Arc::clone(&self.inner);
                let hub = Arc::clone(&self.hub);
                let planning_flag = Arc::clone(&self.watchdog_park_planning);
                if keep {
                    if Self::run_watchdog_park_off_lock_inner(sup_inner, hub, planning_flag, wd.generation).await {
                        let inner = self.inner.read().await;
                        if !inner.detached {
                            obs = Some(inner.current_observation());
                        }
                    }
                } else {
                    tokio::spawn(async move {
                        Self::run_watchdog_park_off_lock_inner(sup_inner, hub, planning_flag, wd.generation).await;
                    });
                }
            }
        }
        if let Some(job) = pending_zc {
            let sup_inner = Arc::clone(&self.inner);
            let cw_lock = Arc::clone(&self.config_write_mutex);
            let pub_hub = Arc::clone(&self.published_config);
            let hooks = self.config_commit_test_hooks();
            tokio::spawn(async move {
                crate::rt_core::requests::config::run_legacy_zero_check_commit(
                    sup_inner, cw_lock, pub_hub, hooks, job,
                )
                .await;
            });
        }
        #[cfg(not(feature = "policy-runtime"))]
        let _ = pending_qual;
        #[cfg(feature = "policy-runtime")]
        if let Some((run, reason, state, ledger_path, live_allowed)) = pending_qual {
            if !live_allowed {
                if let (Some(gate), Some(path)) = (
                    run.verified_release_gate.as_ref().filter(|g| {
                        g.authorization_tier()
                            == crate::policy_runtime::AuthorizationTier::SupervisedFirstRun
                    }),
                    ledger_path,
                ) {
                    let stop_reason = reason.as_str().to_string();
                    let passed = stop_reason == "MAX_SECONDS" && state.as_str() == "done";
                    let record = crate::policy_runtime::qualification::QualificationRecord::for_run(
                        gate,
                        run.action_id,
                        run.steps,
                        &stop_reason,
                        passed,
                    )
                    .with_policy_stats(run.step_stats.report());
                    let qual_coord = Arc::clone(&self.policy_qualification);
                    let persist = move || {
                        if let Err(e) = qual_coord.append_and_release_pending(&path, &record, None) {
                            tracing::warn!(
                                "Cannot append policy qualification record to {}: {e}",
                                path.display()
                            );
                        }
                    };
                    if keep {
                        let _ = tokio::task::spawn_blocking(persist).await;
                    } else {
                        drop(tokio::task::spawn_blocking(persist));
                    }
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        if let Some((comm_rd, comm_margin, tx_budget_dur, comm_ledger)) = legacy_comm_plan {
            self.service_legacy_commissioning_post_tick(
                started,
                comm_rd,
                comm_margin,
                tx_budget_dur,
                comm_ledger,
            )
            .await;
        }
        #[cfg(target_arch = "wasm32")]
        let _ = legacy_comm_plan;
        let late = !dt.is_finite() || dt > 2.0 / f64::from(control_hz.max(1));
        let done = Instant::now();
        let post_us = us(done.saturating_duration_since(released));
        let log = log.unwrap_or_default();
        let holder = |r: crate::supervisor_lock::HoldRecord| {
            let signed = |a: Instant, b: Instant| if a >= b { us(a - b) as i64 } else { -(us(b - a) as i64) };
            crate::diagnostics::tick::Holder {
                kind: r.kind.as_str(),
                location: r.location,
                command: r.command,
                direction: r.direction,
                action_id: r.action_id,
                io_timing: r.io_timing,
                hold_us: us(r.hold()),
                acquire_us: us(r.acquire()),
                acquired_before_wait_us: signed(started, r.acquired),
                released_before_lock_us: signed(locked, r.released),
            }
        };
        let sample = crate::diagnostics::tick::TickSample {
            tick_us: us(done.saturating_duration_since(started)),
            release_us: 0,
            wait_us: us(wait),
            lock_us: us(lock_held),
            post_us,
            period_us: if dt.is_finite() && dt >= 0.0 { us(Duration::from_secs_f64(dt.min(4.0e3))) } else { 0 },
            wake_lateness_us,
            reader_hold_us: us(log.reader_hold),
            writer_hold_us: us(log.writer_hold),
            reader_acquire_us: us(log.reader_acquire),
            writer_acquire_us: us(log.writer_acquire),
            stats_wait_us: 0,
            phases: crate::diagnostics::tick::LockPhases {
                pre_us: us(pre),
                rx_us: phases[0], control_us: phases[1], tx_us: phases[2], finish_us: phases[3], obs_us: phases[4],
                publish_us: publish,
            },
            phases_us: ReleasePhasesUs {
                alloc_read: phase_alloc_read,
                ack_flush: phase_ack_flush,
                terminal: phase_terminal,
                guard_drop: phase_guard_drop,
                post: post_us,
                publish: 0,
            },
            post_phases_us: None,
            post_cpu_us: None,
            usage: crate::diagnostics::tick::ThreadUsage {
                mid_nvcsw,
                mid_nivcsw,
                lock_alloc_calls: allocs.calls.min(u64::from(u32::MAX)) as u32,
                lock_alloc_us: (allocs.total_ns / 1000).min(u64::from(u32::MAX)) as u32,
                lock_alloc_max_us: (allocs.max_ns / 1000).min(u64::from(u32::MAX)) as u32,
                ..tick_usage
            },
            waited_for: log.last_release.map(holder),
            longest_hold: log.longest.map(holder),
            torque_step: tick_tq_dir.map(|direction| crate::diagnostics::tick::TorqueStepAttribution {
                command: "/api/v1/torque",
                direction,
                tx_us: tick_io.tx_us,
                rx_us: tick_io.rx_us,
                sleep_us: tick_io.sleep_us,
            }),
            late,
            release_seq: 0,
            safety_epoch: obs.as_ref().map_or(0, |o| o.epoch_id),
            maint_req_id: 0,
            release_flags: 0,
        };
        if crate::diagnostics::tick::periodic_defers_sample_publish() {
            // Single-completion point in `run_periodic_with_clock` will stamp full `release_us`
            // and `phases_us.publish` (covering wake through `block_on` return and stats
            // publication) and record once.
            crate::diagnostics::tick::stash_periodic_sample_with_post_end(sample, done);
        } else {
            let _ = self.tick_stats.record_staged_with_publish(&sample, || {
                let now = Instant::now();
                (
                    us(now.saturating_duration_since(started)),
                    us(now.saturating_duration_since(done)),
                )
            });
        }
        obs
    }

    /// Spawns at most one bounded off-lock cleanup task in headless mode (when no live control
    /// loop is ticking) so a contended `DetachedCancel` or expired `Detached` disable left behind
    /// after waiter timeout still drains once the driver mutex frees. Exits immediately if a live
    /// control tick advances `control_release_seq` or ownership transfers.
    pub(super) fn spawn_headless_detached_drain_if_needed(&self, observed_seq: u64) {
        if self
            .headless_detached_drain_active
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return;
        }
        let inner_lock = Arc::clone(&self.inner);
        let active_flag = Arc::clone(&self.headless_detached_drain_active);
        let notify = Arc::clone(&self.legacy_torque_notify);
        tokio::spawn(async move {
            struct ClearFlag(Arc<AtomicBool>);
            impl Drop for ClearFlag {
                fn drop(&mut self) {
                    self.0.store(false, Ordering::Release);
                }
            }
            let _guard = ClearFlag(active_flag);
            const DRAIN_INTERVAL: Duration = Duration::from_millis(25);
            const DRAIN_BUDGET: Duration = Duration::from_micros(500);
            let hard_deadline = Instant::now() + Duration::from_secs(5);
            while Instant::now() < hard_deadline {
                tokio::time::sleep(DRAIN_INTERVAL).await;
                let (keep_going, settled_any) = {
                    let mut inner = inner_lock.write().await;
                    let io_scope = crate::diagnostics::tick::HoldIoScope::enter();
                    inner.set_command("/api/v1/torque");
                    if let Some(dir) = match inner.torque_transition {
                        Some(driver_transitions::TorqueTransition::Enable(_)) => Some("on"),
                        Some(driver_transitions::TorqueTransition::Disable { .. }) => Some("off"),
                        None => None,
                    } {
                        inner.set_direction(dir);
                    }
                    let res = if inner.detached
                        || inner.control_release_seq != observed_seq
                        || !inner.has_detached_torque_transition()
                    {
                        (false, false)
                    } else {
                        let now = Instant::now();
                        let budget = crate::rt_core::ops::ServiceBudget {
                            now,
                            deadline: now + DRAIN_BUDGET,
                            max_rx_frames: crate::rt_core::ops::RX_FRAME_BUDGET,
                        };
                        #[cfg(not(target_arch = "wasm32"))]
                        {
                            let live_intent = inner.shadow_live_intent();
                            let live_is_torqued = inner.is_torqued;
                            let live_stopped = inner.safety.state().is_stopped();
                            inner.torque_shadow.enqueue_at(
                                now,
                                crate::torque::shadow::ShadowInputKind::TransitionWake {
                                    live_intent,
                                    live_is_torqued,
                                    live_stopped,
                                },
                            );
                        }
                        let _ = inner.service_torque_transition(&budget);
                        inner.flush_torque_shadow();
                        let settled =
                            std::mem::replace(&mut inner.legacy_torque_settled_this_tick, false);
                        (inner.has_detached_torque_transition(), settled)
                    };
                    inner.set_io_timing(io_scope.finish());
                    res
                };
                if settled_any {
                    notify.notify_waiters();
                }
                if !keep_going {
                    break;
                }
            }
        });
    }

    pub(super) async fn run_watchdog_park_off_lock_inner(
        sup_inner: Arc<SupervisorLock<SupervisorInner>>,
        hub: Arc<crate::rt_core::publish::StateHub>,
        planning_flag: Arc<AtomicBool>,
        generation: u64,
    ) -> bool {
        use crate::rt_core::requests::motion as req;
        struct ResetOnDrop(Arc<AtomicBool>);
        impl Drop for ResetOnDrop {
            fn drop(&mut self) { self.0.store(false, Ordering::Release); }
        }
        let _guard = ResetOnDrop(planning_flag);
        const MAX_RETRIES: usize = 5;
        for attempt in 0..MAX_RETRIES {
            let cached_cfg = hub.legacy_motion_config();
            let (snap, cache_hit) = {
                let inner = sup_inner.read().await;
                if inner.detached
                    || !inner.watchdog_park_pending.is_some_and(|p| p.generation == generation && !p.waiting_mit_handback)
                {
                    return false;
                }
                req::MotionPlanSnapshot::from_supervisor_with_cached_config(&inner, cached_cfg.as_ref())
            };
            drop(cached_cfg);
            if !cache_hit {
                hub.set_legacy_motion_config_if_newer(Arc::clone(&snap.config));
            }
            let planned = tokio::task::spawn_blocking(move || req::prepare_motion(&snap, &req::MotionRequest::park())).await;
            let mut inner = sup_inner.write().await;
            if inner.detached
                || !inner.watchdog_park_pending.is_some_and(|p| p.generation == generation && !p.waiting_mit_handback)
            {
                return false;
            }
            match planned {
                Ok(Ok(plan)) => match req::install_watchdog_park(&mut inner, generation, &plan.slot, plan.heap_traj) {
                    Ok(_) => return true,
                    Err(req::MotionInstallError::StartDrifted) if attempt + 1 < MAX_RETRIES => continue,
                    Err(req::MotionInstallError::ContinuationMismatch) => return false,
                    Err(e) => {
                        req::fail_watchdog_park_plan(&mut inner, generation, crate::safety::BoundedText::new(e.code()));
                        return true;
                    }
                },
                Ok(Err(e)) => {
                    let mut msg = crate::safety::BoundedText::<160>::empty();
                    let _ = core::fmt::write(&mut msg, format_args!("watchdog park could not be planned: {e}"));
                    req::fail_watchdog_park_plan(&mut inner, generation, msg);
                    return true;
                }
                Err(e) => {
                    let mut msg = crate::safety::BoundedText::<160>::empty();
                    let _ = core::fmt::write(&mut msg, format_args!("watchdog park planner aborted: {e}"));
                    req::fail_watchdog_park_plan(&mut inner, generation, msg);
                    return true;
                }
            }
        }
        let mut inner = sup_inner.write().await;
        if !inner.detached {
            req::fail_watchdog_park_plan(
                &mut inner,
                generation,
                crate::safety::BoundedText::new("watchdog park drifted repeatedly"),
            );
            return true;
        }
        false
    }
}
