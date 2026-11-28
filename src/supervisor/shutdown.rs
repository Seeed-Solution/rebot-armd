//! Process-exit/shutdown panic disable, checked E-stop, and urgent-stop handlers.

#[allow(unused_imports)]
use super::*;

// scaffold-extra-0
impl ArmSupervisor {
    pub async fn shutdown_ready(&self) -> bool {
        match self.owner_mode() {
            OWNER_EXECUTOR => match self.executor_handle() {
                Some(h) => h.shutdown_ready_async().await.unwrap_or(false),
                None => false,
            },
            OWNER_LEGACY => {
                let inner = self.inner.read().await;
                if self.check_legacy_under_guard(&inner).is_ok() {
                    return inner.shutdown_ready();
                }
                drop(inner);
                match self.executor_handle() {
                    Some(h) => h.shutdown_ready_async().await.unwrap_or(false),
                    None => false,
                }
            }
            _ => false,
        }
    }

    /// Authoritative shutdown gate routed through whichever single owner holds `ArmCore`.
    /// Bounded last-chance hardware disable for the legacy control-thread panic handler.
    /// Latches E-stop immediately and attempts one disable through the legacy driver within
    /// `timeout` without ever blocking indefinitely (even if another task holds `inner` or a
    /// blocking transport syscall stalls). `timeout` is the hard upper bound, with a small
    /// internal cooperative reserve so clean `Unconfirmed` exhaustion finishes logging, lock
    /// release, and channel handoff before `rx.recv_timeout` triggers `TimedOut`.
    /// Explicit test-only authorization hook for legacy `ProcessExit` protective disable tests.
    #[doc(hidden)]
    pub fn authorize_process_exit_for_test(&self) {
        self.process_exit_requested.store(true, Ordering::Release);
    }

    /// Bounded legacy `DisableProtective { ProcessExit }` helper. Requires explicit owner-bound
    /// `ProcessExit` authorization (`emergency_process_exit_disable_until`, `shutdown_committed`,
    /// or `authorize_process_exit_for_test`) so ad-hoc callers cannot issue a protective cut
    /// outside process exit (DRV-F1 B7). Never returns `NoDriver` from a detached shell.
    pub fn attempt_legacy_panic_disable(&self, timeout: Duration) -> PanicDisableOutcome {
        match self.attempt_legacy_panic_disable_step(timeout) {
            LegacyPanicDisableStep::Outcome(outcome) => outcome,
            LegacyPanicDisableStep::Detached => PanicDisableOutcome::Failed(
                "legacy supervisor shell is detached; active writer owns driver".into(),
            ),
        }
    }

    pub(super) fn attempt_legacy_panic_disable_step(&self, timeout: Duration) -> LegacyPanicDisableStep {
        let exit_authorized = self.process_exit_requested.load(Ordering::Acquire)
            || self
                .inner
                .try_read()
                .is_ok_and(|g| !g.detached && g.shutdown_committed);
        if !exit_authorized {
            return LegacyPanicDisableStep::Outcome(PanicDisableOutcome::Failed(
                "legacy protective disable requires explicit ProcessExit authorization".into(),
            ));
        }
        if self.owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING {
            return LegacyPanicDisableStep::Detached;
        }
        self.estop_state.request_and_latch();
        let owner_mode = Arc::clone(&self.owner_mode);
        #[cfg(not(target_arch = "wasm32"))]
        type PrePanicShadowSnap = Option<(crate::torque::TorqueIntent, bool, bool, Instant)>;
        #[cfg(target_arch = "wasm32")]
        type PrePanicShadowSnap = ();

        let run_inline = |inner: Arc<SupervisorLock<SupervisorInner>>,
                          owner_mode: Arc<AtomicU8>,
                          coop_deadline: Instant,
                          hard_deadline: Instant,
                          coop_budget: Duration|
         -> (LegacyPanicDisableStep, PrePanicShadowSnap) {
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let mut guard = loop {
                    if owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING {
                        return (LegacyPanicDisableStep::Detached, Default::default());
                    }
                    if let Ok(g) = inner.try_write() {
                        break g;
                    }
                    let rem = hard_deadline.saturating_duration_since(Instant::now());
                    if rem.is_zero() {
                        return (
                            LegacyPanicDisableStep::Outcome(PanicDisableOutcome::TimedOut),
                            Default::default(),
                        );
                    }
                    std::thread::sleep(rem.min(Duration::from_millis(2)));
                };
                if guard.detached || owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING {
                    // Ownership moved or is actively transferring out of the Legacy shell;
                    // NEVER dispatch Legacy disable or falsely report `NoDriver`.
                    return (LegacyPanicDisableStep::Detached, Default::default());
                }
                if Instant::now() >= hard_deadline {
                    return (
                        LegacyPanicDisableStep::Outcome(PanicDisableOutcome::TimedOut),
                        Default::default(),
                    );
                }
                #[cfg(not(target_arch = "wasm32"))]
                let pre_snap = Some((
                    guard.shadow_live_intent(),
                    guard.is_torqued,
                    guard.safety.state().is_stopped(),
                    guard.cached_driver_clock_now(),
                ));
                #[cfg(target_arch = "wasm32")]
                let pre_snap = ();
                let outcome = guard.attempt_panic_disable_until(
                    coop_deadline,
                    hard_deadline,
                    coop_budget,
                );
                (LegacyPanicDisableStep::Outcome(outcome), pre_snap)
            }))
            .unwrap_or_else(|_| {
                (
                    LegacyPanicDisableStep::Outcome(PanicDisableOutcome::Failed(
                        "panic during legacy panic-disable".into(),
                    )),
                    Default::default(),
                )
            })
        };

        let origin = Instant::now();
        let (coop_deadline, hard_deadline) = driver_transitions::panic_disable_deadlines(origin, timeout);
        let coop_budget = coop_deadline.saturating_duration_since(origin);
        #[cfg(not(target_arch = "wasm32"))]
        {
            let inner = Arc::clone(&self.inner);
            let owner_mode_thread = Arc::clone(&owner_mode);
            let shadow_hub = Arc::clone(&self.torque_shadow);
            let (tx, rx) = std::sync::mpsc::sync_channel(1);
            match std::thread::Builder::new()
                .name("armd-panic-disable".into())
                .spawn(move || {
                    let (step, pre_snap, done_at) = {
                        let (res, pre_snap) = run_inline(
                            inner.clone(),
                            owner_mode_thread,
                            coop_deadline,
                            hard_deadline,
                            coop_budget,
                        );
                        (res, pre_snap, Instant::now())
                    };
                    // Seal and deliver the live outcome FIRST before touching shadow recording.
                    let _ = tx.send((step.clone(), done_at));
                    if let LegacyPanicDisableStep::Outcome(outcome) = step {
                        match inner.try_write() {
                            Ok(mut g) if !g.detached => {
                                if let Some((pre_intent, pre_torqued, pre_stopped, pre_ts)) = pre_snap {
                                    g.record_post_panic_disable_shadow_at(
                                        true,
                                        &outcome,
                                        pre_intent,
                                        pre_torqued,
                                        pre_stopped,
                                        pre_ts,
                                    );
                                } else {
                                    g.record_post_panic_disable_shadow(true, &outcome);
                                }
                            }
                            _ => {
                                // Lock contended or detached after fatal panic-disable: latch
                                // session staging-loss cutoff lock-free on the hub instead of
                                // silently dropping post-panic shadow state.
                                shadow_hub.record_contended_staging_loss(1);
                            }
                        }
                    }
                }) {
                Ok(_) => {
                    let wait = hard_deadline.saturating_duration_since(Instant::now());
                    match rx.recv_timeout(wait) {
                        Ok((step, completed_at)) if completed_at < hard_deadline => step,
                        Ok(_) | Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                            LegacyPanicDisableStep::Outcome(PanicDisableOutcome::TimedOut)
                        }
                        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                            LegacyPanicDisableStep::Outcome(PanicDisableOutcome::Failed(
                                "panic-disable helper thread disconnected".into(),
                            ))
                        }
                    }
                }
                Err(e) => {
                    // Never fall back to an unbounded inline driver call when the bounded
                    // helper thread cannot be spawned; only confirm NoDriver when NOT detached.
                    if self.owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING {
                        return LegacyPanicDisableStep::Detached;
                    }
                    if let Ok(guard) = self.inner.try_write() {
                        if guard.detached
                            || self.owner_mode.load(Ordering::Acquire) == OWNER_TRANSFERRING
                        {
                            return LegacyPanicDisableStep::Detached;
                        }
                        if guard.driver.is_none() {
                            return LegacyPanicDisableStep::Outcome(PanicDisableOutcome::NoDriver);
                        }
                    }
                    LegacyPanicDisableStep::Outcome(PanicDisableOutcome::Failed(format!(
                        "failed to spawn bounded panic-disable helper thread: {e}"
                    )))
                }
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            run_inline(
                Arc::clone(&self.inner),
                owner_mode,
                coop_deadline,
                hard_deadline,
                coop_budget,
            )
            .0
        }
    }

    pub async fn request_shutdown(&self) -> Result<bool, ArmError> {
        let deadline = Instant::now() + PANIC_DISABLE_TIMEOUT + Duration::from_millis(100);
        self.request_shutdown_until(deadline).await
    }

    /// Bounded shutdown admission carrying ONE absolute `deadline` from daemon entry (B3).
    pub async fn request_shutdown_until(&self, deadline: Instant) -> Result<bool, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => match self.executor_handle() {
                Some(h) => h.request_shutdown_until(deadline).await.map_err(Into::into),
                None => Err(executor_unavailable()),
            },
            OWNER_LEGACY => {
                let rem = deadline.saturating_duration_since(Instant::now());
                if rem.is_zero() {
                    return Err(crate::rt_core::executor::OwnerError::TimedOut.into());
                }
                let mut inner = match tokio::time::timeout(rem, self.inner.clone().write_owned()).await {
                    Ok(g) => g,
                    Err(_) => return Err(crate::rt_core::executor::OwnerError::TimedOut.into()),
                };
                if self.check_legacy_under_guard(&inner).is_ok() {
                    let rem_after_lock = deadline.saturating_duration_since(Instant::now());
                    if rem_after_lock.is_zero() {
                        return Err(crate::rt_core::executor::OwnerError::TimedOut.into());
                    }
                    let task = tokio::task::spawn_blocking(move || inner.request_shutdown_until(deadline));
                    return match tokio::time::timeout(rem_after_lock, task).await {
                        Ok(Ok(ok)) => Ok(ok),
                        Ok(Err(e)) => Err(ArmError::Internal(format!(
                            "request_shutdown blocking task failed: {e}"
                        ))),
                        Err(_) => Err(crate::rt_core::executor::OwnerError::TimedOut.into()),
                    };
                }
                drop(inner);
                match self.executor_handle() {
                    Some(h) => h.request_shutdown_until(deadline).await.map_err(Into::into),
                    None => Err(executor_unavailable()),
                }
            }
            _ => {
                if self.has_detached_safe_hold() {
                    // A detached SAFE-HOLD writer holds an energized/unknown driver; ordinary
                    // shutdown admission is false until deliberate process exit disables it.
                    Ok(false)
                } else {
                    Err(executor_unavailable())
                }
            }
        }
    }

    /// Deliberate process-exit protective disable routed through the authoritative active writer
    /// endpoint (`Executor`, `Legacy`, or `DetachedSafeHold`) under ONE absolute `deadline` (B3/B4/B11).
    /// Waits across `WriterEndpoint::Transferring` and re-resolves if the Legacy shell was detached
    /// during extraction or an Executor handback completed concurrently.
    pub async fn emergency_process_exit_disable_until(
        self: &Arc<Self>,
        deadline: Instant,
    ) -> PanicDisableOutcome {
        self.process_exit_requested.store(true, Ordering::Release);
        self.estop_state.latch();
        self.internal_fault_lane.latch_control_safe_hold_fence();
        loop {
            let rem = deadline.saturating_duration_since(Instant::now());
            if rem.is_zero() {
                return PanicDisableOutcome::TimedOut;
            }
            match self.active_writer_endpoint() {
                WriterEndpoint::Transferring => {
                    // Ownership extraction/transfer is in progress; wait for the authoritative
                    // endpoint (`Executor`, `DetachedSafeHold`, or rolled-back `Legacy`) rather
                    // than selecting Legacy or touching the detached shell.
                    tokio::time::sleep(rem.min(Duration::from_millis(2))).await;
                    continue;
                }
                WriterEndpoint::DetachedSafeHold {
                    safe_hold,
                    control_stop,
                } => {
                    control_stop.store(true, Ordering::Release);
                    safe_hold.request_process_exit_disable(deadline);
                    return tokio::task::spawn_blocking(move || {
                        safe_hold.wait_process_exit_until(deadline)
                    })
                    .await
                    .unwrap_or_else(|je| PanicDisableOutcome::Failed(je.to_string()));
                }
                WriterEndpoint::Executor(handle) => {
                    let h_em = handle.clone();
                    let outcome = tokio::task::spawn_blocking(move || {
                        h_em.emergency_disable_and_join_until(deadline)
                    })
                    .await
                    .unwrap_or_else(|je| PanicDisableOutcome::Failed(je.to_string()));
                    // If the executor completed a concurrent handback to Legacy right as ProcessExit
                    // arrived, re-resolve the authoritative endpoint and disable via Legacy within
                    // the SAME `deadline` (DRV-F1 B3).
                    if handle.was_handed_back() {
                        continue;
                    }
                    return outcome;
                }
                WriterEndpoint::Legacy {
                    control_stop,
                    safe_hold,
                } => {
                    if let Some(stop) = control_stop {
                        self.signal_legacy_stop(&stop);
                    }
                    if safe_hold.is_active() {
                        safe_hold.request_process_exit_disable(deadline);
                        return tokio::task::spawn_blocking(move || {
                            safe_hold.wait_process_exit_until(deadline)
                        })
                        .await
                        .unwrap_or_else(|je| PanicDisableOutcome::Failed(je.to_string()));
                    }
                    let sup = Arc::clone(self);
                    let step = tokio::task::spawn_blocking(move || {
                        let rem = deadline.saturating_duration_since(Instant::now());
                        sup.attempt_legacy_panic_disable_step(rem)
                    })
                    .await
                    .map_err(|je| PanicDisableOutcome::Failed(je.to_string()));
                    match step {
                        Ok(LegacyPanicDisableStep::Outcome(outcome)) => return outcome,
                        Ok(LegacyPanicDisableStep::Detached) => {
                            // Shell was detached by a concurrent extraction into `Executor` or
                            // `DetachedSafeHold`: re-resolve the authoritative writer endpoint.
                            tokio::time::sleep(Duration::from_millis(1)).await;
                            continue;
                        }
                        Err(outcome) => return outcome,
                    }
                }
            }
        }
    }

    pub async fn is_at_safe_resting_pose(&self) -> bool {
        let inner = self.inner.read().await;
        inner.is_at_safe_resting_pose()
    }

    pub(super) async fn await_estop_waiter(
        &self,
        mut rx: tokio::sync::oneshot::Receiver<Result<(SafetyState, u64), ArmError>>,
    ) -> Result<(SafetyState, u64), ArmError> {
        loop {
            match tokio::time::timeout(Duration::from_millis(2), &mut rx).await {
                Ok(Ok(res)) => return res,
                Ok(Err(_)) => {
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "EXECUTOR_STOPPED".into(),
                        message: "RT executor stopped before acknowledging E-stop".into(),
                    });
                }
                Err(_) => {
                    self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).drain();
                }
            }
        }
    }

    pub(crate) fn flush_legacy_estop_acks_into(
        estop_reg: &std::sync::Mutex<crate::rt_core::executor::EstopRegistry>,
        inner: &mut SupervisorInner,
    ) {
        if inner.pending_estop_acks.is_empty() && inner.estop_ack_overflow_gen == 0 {
            return;
        }
        let mut reg = estop_reg.lock().unwrap_or_else(|e| e.into_inner());
        while let Some(ack) = inner.pending_estop_acks.pop_front() {
            reg.record_ack(ack);
        }
        if inner.estop_ack_overflow_gen > 0 {
            let ov = std::mem::replace(&mut inner.estop_ack_overflow_gen, 0);
            reg.record_overflow(ov);
        }
    }

    /// Test-only hook: holds the ACK-drain `estop_reg` mutex so tests can prove the WS reader
    /// E-stop path latches and increments `requested_estop_count` purely via lock-free atomics.
    #[doc(hidden)]
    pub fn test_lock_estop_registry(&self) -> impl Drop + '_ {
        self.estop_reg.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Test-only accessor for the monotonic lock-free E-stop request counter.
    #[doc(hidden)]
    pub fn test_requested_estop_count(&self) -> u64 {
        self.requested_estop_count.load(Ordering::SeqCst)
    }

    /// Synchronously latches the physical E-stop flag and increments the requested generation
    /// using ONLY lock-free atomics (never takes `estop_reg` mutex or scans waiters on the
    /// reader path). Call [`Self::finish_e_stop_checked`] off-reader to register/resolve and
    /// obtain the applied `(SafetyState, epoch_id)`.
    pub(crate) fn begin_e_stop_checked(&self) -> CheckedEstopTicket {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::begin_e_stop_checked");
        let req = self.estop_state.request_and_latch();
        let mode = self.owner_mode.load(Ordering::Acquire);
        CheckedEstopTicket { req, mode }
    }

    pub(crate) async fn finish_e_stop_checked(
        &self,
        ticket: CheckedEstopTicket,
    ) -> Result<(SafetyState, u64), ArmError> {
        let CheckedEstopTicket { req, mode } = ticket;
        // Off-reader waiter registration / early-ACK resolution (never runs on WS reader).
        let wait = self
            .estop_reg
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .register_or_resolve(req);
        let mut rx = match wait {
            crate::rt_core::executor::EstopWait::Resolved(res) => return res,
            crate::rt_core::executor::EstopWait::Pending(rx) => rx,
        };
        match mode {
            OWNER_EXECUTOR => {
                self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).drain();
                self.await_estop_waiter(rx).await
            }
            OWNER_LEGACY | OWNER_TRANSFERRING => {
                if let Ok(res) = rx.try_recv() {
                    return res;
                }
                let inner = Arc::clone(&self.inner);
                let mut guard = inner.write().await;
                if let Ok(res) = rx.try_recv() {
                    drop(guard);
                    return res;
                }
                if !guard.detached {
                    guard.consume_pending_estop();
                    Self::flush_legacy_estop_acks_into(&self.estop_reg, &mut guard);
                    let fallback = (guard.safety.state().clone(), guard.safety.epoch_id());
                    drop(guard);
                    return rx.try_recv().unwrap_or(Ok(fallback));
                }
                drop(guard);
                // Ownership changed while waiting on `inner.write()`: if a concurrent legacy
                // call or the newly attached executor already applied `req`, return that pinned result.
                if let Ok(res) = rx.try_recv() {
                    return res;
                }
                match self.owner_mode() {
                    OWNER_EXECUTOR => {
                        self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).drain();
                        self.await_estop_waiter(rx).await
                    }
                    _ => {
                        self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).cancel_waiter(
                            req,
                            503,
                            "EXECUTOR_UNAVAILABLE",
                            "RT executor ownership unavailable while applying E-stop",
                        );
                        if let Ok(res) = rx.try_recv() {
                            return res;
                        }
                        Err(executor_unavailable())
                    }
                }
            }
            _ => {
                self.estop_reg.lock().unwrap_or_else(|e| e.into_inner()).cancel_waiter(
                    req,
                    503,
                    "EXECUTOR_UNAVAILABLE",
                    "RT executor ownership is unavailable",
                );
                Err(executor_unavailable())
            }
        }
    }

    /// Lock-free physical stop latch + exact per-request pinned acknowledgement across both
    /// Legacy and Executor ownership (including concurrent Legacy -> Executor handoff).
    pub async fn trigger_e_stop_checked(&self) -> Result<(SafetyState, u64), ArmError> {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::trigger_e_stop_checked");
        let ticket = self.begin_e_stop_checked();
        self.finish_e_stop_checked(ticket).await
    }

    pub async fn trigger_e_stop(&self) -> (SafetyState, u64) {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::trigger_e_stop");
        self.trigger_e_stop_checked()
            .await
            .unwrap_or((SafetyState::EStop, 0))
    }

    /// Non-blocking urgent-stop enqueue in `OWNER_EXECUTOR` mode so a WS receive loop can
    /// await the reply in `tokio::select!` without blocking subsequent `estop` frames on
    /// the same socket. Returns `Ok(None)` in `OWNER_LEGACY`.
    pub(crate) fn try_enqueue_executor_urgent_stop(
        &self,
        stop: crate::rt_core::urgent::UrgentStop,
    ) -> Result<
        Option<(
            Arc<crate::rt_core::urgent::UrgentStopClient>,
            tokio::sync::oneshot::Receiver<
                Result<crate::rt_core::urgent::UrgentStopOutcome, crate::rt_core::urgent::UrgentStopError>,
            >,
        )>,
        ArmError,
    > {
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::try_enqueue_executor_urgent_stop");
        match self.owner_mode() {
            OWNER_LEGACY => Ok(None),
            OWNER_EXECUTOR => {
                let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                let client = h.urgent_stop_client();
                let rx = client.enqueue_async(stop).map_err(ArmError::from)?;
                Ok(Some((client, rx)))
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Authorized-stop handoff routed through whichever single owner holds `ArmCore`
    /// (including concurrent Legacy -> Executor handoff).
    /// - In `OWNER_EXECUTOR`: forwards through the bounded urgent `Stop` channel without
    ///   touching the legacy supervisor lock; only an authorized stop advances `StopRequests`.
    /// - In `OWNER_LEGACY`: validates under `inner.write()` and preserves exact legacy
    ///   `/api/v1/motion/stop` semantics (gravity-comp exit, `stop_motion`, `request_macro_stop`).
    pub async fn urgent_stop(
        &self,
        stop: crate::rt_core::urgent::UrgentStop,
    ) -> Result<crate::rt_core::urgent::UrgentStopOutcome, ArmError> {
        self.urgent_stop_with_legacy_hook(stop, |_, _| {}).await
    }

    /// Like [`Self::urgent_stop`], running `on_legacy_applied(epoch, stop_gen)` strictly inside
    /// the legacy `inner.write()` critical section alongside the stop mutation.
    pub(crate) async fn urgent_stop_with_legacy_hook(
        &self,
        stop: crate::rt_core::urgent::UrgentStop,
        on_legacy_applied: impl FnOnce(u64, u64),
    ) -> Result<crate::rt_core::urgent::UrgentStopOutcome, ArmError> {
        use crate::rt_core::urgent::UrgentStopOutcome;
        crate::rt_core::executor::assert_not_in_rt_release("ArmSupervisor::urgent_stop");
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                h.urgent_stop_async(stop).await.map_err(ArmError::from)
            }
            OWNER_LEGACY => {
                let mut guard = self.inner.write().await;
                if !guard.detached {
                    guard.consume_pending_estop();
                    Self::flush_legacy_estop_acks_into(&self.estop_reg, &mut guard);
                    let epoch = guard.safety.epoch_id();
                    if stop.lease_generation == 0
                        || !guard.validate_lease_at(stop.connection.as_str(), stop.lease_generation, Instant::now())
                    {
                        return Ok(UrgentStopOutcome::LeaseLost { epoch });
                    }
                    if stop.epoch != Some(epoch) {
                        return Ok(UrgentStopOutcome::EpochMismatch { epoch });
                    }
                    if !stop.request_id_valid {
                        return Ok(UrgentStopOutcome::InvalidRequestId);
                    }
                    if let Some(message) = stop.body_error {
                        return Ok(UrgentStopOutcome::InvalidBody { message, epoch });
                    }
                    let applied_to_motion = guard.authorized_stop();
                    let stop_gen = if applied_to_motion {
                        guard.current_stop_gen()
                    } else {
                        guard.advance_stop_generation()
                    };
                    let applied_epoch = guard.safety.epoch_id();
                    on_legacy_applied(applied_epoch, stop_gen);
                    return Ok(UrgentStopOutcome::Applied {
                        stop_generation: stop_gen,
                        applied_to_motion,
                        epoch: applied_epoch,
                        safety_state: guard.safety.state_arc(),
                        is_torqued: guard.is_torqued,
                        control_mode_gravity_comp: guard.gravity_comp_active(),
                    });
                }
                drop(guard);
                if self.owner_mode() == OWNER_EXECUTOR {
                    let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                    h.urgent_stop_async(stop).await.map_err(ArmError::from)
                } else {
                    Err(executor_unavailable())
                }
            }
            _ => Err(executor_unavailable()),
        }
    }

    pub async fn clear_fault(&self) -> Result<SafetyState, ArmError> {
        self.clear_fault_for_epoch(None).await
    }

    pub async fn clear_fault_for_epoch(&self, expected_epoch: Option<u64>) -> Result<SafetyState, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                let res = h.clear_fault_for_epoch_async(expected_epoch).await.map_err(ArmError::from)?;
                res.map_err(ArmError::Safety)
            }
            OWNER_LEGACY => {
                let mut inner = self.inner.clone().write_owned().await;
                if self.check_legacy_under_guard(&inner).is_err() {
                    drop(inner);
                    if self.owner_mode() == OWNER_EXECUTOR {
                        let h = self.executor_handle().ok_or_else(executor_unavailable)?;
                        let res = h.clear_fault_for_epoch_async(expected_epoch).await.map_err(ArmError::from)?;
                        return res.map_err(ArmError::Safety);
                    }
                    return Err(executor_unavailable());
                }
                let estop_reg = Arc::clone(&self.estop_reg);
                let res = tokio::task::spawn_blocking(move || {
                    let r = inner.clear_fault_for_epoch(expected_epoch);
                    inner.safety.drain_retired_state_arcs_off_thread();
                    ArmSupervisor::flush_legacy_estop_acks_into(&estop_reg, &mut inner);
                    r
                })
                    .await
                    .map_err(|e| ArmError::Internal(format!("clear_fault blocking task failed: {e}")))?;
                res.map_err(ArmError::Safety)
            }
            _ => Err(executor_unavailable()),
        }
    }

    pub async fn apply_stop_policy(&self, policy: StopPolicy) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.apply_stop_policy(policy);
        }
    }

    pub async fn is_unloading(&self) -> bool {
        let inner = self.inner.read().await;
        inner.pending_normal_unload || inner.active_trajectory.is_some() || inner.unload_settle.is_some()
    }

    pub async fn has_active_trajectory(&self) -> bool {
        let inner = self.inner.read().await;
        inner.active_trajectory.is_some()
    }
}
