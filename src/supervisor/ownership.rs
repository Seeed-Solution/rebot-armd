//! Ownership mode transitions, executor handoff/handback, `Drop`, and process identity.

#[allow(unused_imports)]
use super::*;

impl Drop for TransferGuard<'_> {
    fn drop(&mut self) {
        if self.armed {
            {
                let mut slot = self.sup.owner.lock().unwrap_or_else(|e| e.into_inner());
                if matches!(*slot, OwnerSlot::Transferring) {
                    *slot = OwnerSlot::Legacy;
                    self.sup.owner_mode.store(OWNER_LEGACY, Ordering::Release);
                }
            }
            {
                let mut w = self.sup.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                if matches!(*w, WriterEndpoint::Transferring) {
                    *w = WriterEndpoint::Legacy {
                        control_stop: self.saved_legacy_stop.take(),
                        safe_hold: Arc::clone(&self.sup.legacy_safe_hold),
                    };
                }
            }
            self.sup.try_drain_pending_disconnects_legacy();
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl Drop for SupervisorInner {
    fn drop(&mut self) {
        // Last-resort safety barrier (Blocker 3): even if a caller held a cloned
        // `Arc<SupervisorLock<SupervisorInner>>` across `ArmSupervisor::drop` so `try_write()`
        // could not extract the driver into SAFE-HOLD, NEVER drop an energized/unconfirmed driver
        // when the final `Arc` releases.
        if !self.detached
            && !self.shutdown_committed
            && self.driver.is_some()
            && !crate::rt_core::executor::core_is_confirmed_disarmed(self)
        {
            if let Some(slot) = self.driver.take() {
                tracing::error!(
                    "SupervisorInner::drop reached with energized/unconfirmed driver still attached; leaking driver slot rather than dropping energized hardware"
                );
                std::mem::forget(slot);
            }
        }
    }
}

impl Drop for ArmSupervisor {
    fn drop(&mut self) {
        // DRV-F1 B2: NEVER destroy an energized or unconfirmed Legacy driver on Drop — even after
        // an unconfirmed/timed-out Legacy `ProcessExit` (`process_exit_requested == true`).
        // If this Legacy shell still holds a non-disarmed driver, extract it and hand it to a
        // detached SAFE-HOLD writer thread; if spawning fails, retain sole ownership and run
        // SAFE-HOLD inline (NEVER dropping an energized driver inside a failed spawn closure).
        #[cfg(not(target_arch = "wasm32"))]
        let mut extracted_for_drop: Option<(
            Box<dyn crate::driver::ArmDriver>,
            crate::rt_core::safe_hold::SafeHoldBuffers,
            crate::torque::shadow::ShadowRuntime,
        )> = None;
        #[cfg(not(target_arch = "wasm32"))]
        if self.owner_mode.load(Ordering::Acquire) == OWNER_LEGACY {
            if let Ok(mut guard) = self.inner.try_write() {
                if !guard.detached
                    && !guard.shutdown_committed
                    && !crate::rt_core::executor::core_is_confirmed_disarmed(&guard)
                {
                    let period = Duration::from_secs_f64(
                        1.0 / f64::from(guard.config.control_frequency_hz.max(1)),
                    );
                    let mut safe_hold_bufs =
                        crate::rt_core::safe_hold::SafeHoldBuffers::from_config(&guard.config, period);
                    safe_hold_bufs.attach_stop_mirrors(
                        Arc::clone(&self.estop_state),
                        Arc::clone(&guard.estop_applied),
                        guard.stop_requests.clone(),
                    );
                    let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        guard.on_safe_hold_enter("SAFE_HOLD");
                        safe_hold_bufs.seed_on_entry(&guard);
                    }));
                    let extracted = guard
                        .driver
                        .take()
                        .map(crate::supervisor_lock::DriverSlot::into_driver_recover_for_safe_hold);
                    let extracted_shadow = {
                        let disabled =
                            crate::torque::shadow::ShadowRuntime::disabled(&guard.config);
                        std::mem::replace(&mut guard.torque_shadow, disabled)
                    };
                    if let Some(drv) = extracted {
                        guard.detached = true;
                        {
                            let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                            *slot = OwnerSlot::Failed;
                            self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                        }
                        {
                            let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                            *w = WriterEndpoint::DetachedSafeHold {
                                safe_hold: Arc::clone(&self.legacy_safe_hold),
                                control_stop: Arc::new(AtomicBool::new(true)),
                            };
                        }
                        extracted_for_drop = Some((drv, safe_hold_bufs, extracted_shadow));
                    }
                }
            }
        }
        self.hub.close();
        #[cfg(not(target_arch = "wasm32"))]
        if let Some((drv, safe_hold_bufs, extracted_shadow)) = extracted_for_drop {
            let sh_ctrl = Arc::clone(&self.legacy_safe_hold);
            // Store the energized driver in a shared slot BEFORE calling `Builder::spawn` so a
            // spawn error NEVER drops the closure's captured driver.
            let handoff = Arc::new(std::sync::Mutex::new(Some((
                drv,
                safe_hold_bufs,
                extracted_shadow,
            ))));
            let handoff_thread = Arc::clone(&handoff);
            let sh_thread = Arc::clone(&sh_ctrl);
            let spawn_res = if self.fail_drop_safe_hold_spawn.load(Ordering::Acquire) {
                Err(std::io::Error::other(
                    "injected SAFE-HOLD retention thread spawn failure",
                ))
            } else {
                std::thread::Builder::new()
                    .name("armd-safe-hold-drop".into())
                    .spawn(move || {
                        let Some((mut drv, mut safe_hold_bufs, mut extracted_shadow)) =
                            handoff_thread.lock().unwrap_or_else(|e| e.into_inner()).take()
                        else {
                            return;
                        };
                        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            let _ = safe_hold_bufs.run_loop_with_shadow(
                                drv.as_mut(),
                                &sh_thread,
                                crate::rt_core::safe_hold::SafeHoldReason::TeardownWhileEnergized,
                                Instant::now,
                                std::thread::sleep,
                                Some(&mut extracted_shadow),
                            );
                        }));
                        if res.is_err() {
                            // Never drop an energized driver on unexpected unwind.
                            std::mem::forget(drv);
                        } else {
                            drop(drv);
                        }
                    })
            };
            if let Err(e) = spawn_res {
                tracing::error!(
                    "Failed to spawn SAFE-HOLD retention thread on ArmSupervisor::drop ({e}); retaining driver in inline SAFE-HOLD"
                );
                if let Some((mut drv, mut safe_hold_bufs, mut extracted_shadow)) =
                    handoff.lock().unwrap_or_else(|e| e.into_inner()).take()
                {
                    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        let _ = safe_hold_bufs.run_loop_with_shadow(
                            drv.as_mut(),
                            &sh_ctrl,
                            crate::rt_core::safe_hold::SafeHoldReason::TeardownWhileEnergized,
                            Instant::now,
                            std::thread::sleep,
                            Some(&mut extracted_shadow),
                        );
                    }));
                    if res.is_err() {
                        std::mem::forget(drv);
                    } else {
                        drop(drv);
                    }
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow
            .stop_and_flush_bounded(Duration::from_millis(50));
    }
}

impl ArmSupervisor {
    #[inline(never)]
    pub fn new(config: ArmConfig) -> Self {
        let _ = init_process_identity();
        let (tx, _) = broadcast::channel(128);
        let estop_state = Arc::new(crate::rt_core::urgent::EstopState::new());
        let e_stop_latched = crate::rt_core::urgent::EstopLatchView::new(estop_state.clone());
        let requested_estop_count = crate::rt_core::urgent::EstopCounterView::new(estop_state.clone());
        let stop_gen = Arc::new(AtomicU64::new(0));
        let lease_gen_mirror = Arc::new(AtomicU64::new(0));
        let closed_bitset = Arc::new(crate::rt_core::requests::actions::ClosedBitset::new());
        let atomic_fenced_lease_gen = Arc::new(AtomicU64::new(0));
        let pending_disconnect_hold = Arc::new(AtomicU64::new(0));
        #[cfg(feature = "policy-runtime")]
        let rt_policy_auth = crate::policy_runtime::PolicyAuthorization::default();
        #[cfg(feature = "policy-runtime")]
        let atomic_diag_sealed_policy_action_id = Arc::new(AtomicU64::new(0));
        #[cfg(feature = "policy-runtime")]
        let atomic_closed_policy_action_id = Arc::new(AtomicU64::new(0));
        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
        let policy_cancel_slots = Arc::new(crate::rt_core::requests::policy::PolicyCancelSlots::new());
        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
        let (policy_terminal_tx, policy_qualification) =
            crate::rt_core::requests::policy::PolicyQualificationCoordinator::channel();
        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
        let policy_backend_permits = Arc::new(tokio::sync::Semaphore::new(2));
        let terminals = Arc::new(TerminalBroadcaster::default());
        let internal_fault_lane = Arc::new(crate::panic_coord::InternalFaultLane::new());
        let legacy_safe_hold = Arc::new(crate::rt_core::safe_hold::SafeHoldControl::new());
        let (terminal_capture_tx, terminal_capture_channel) =
            terminal_capture::TerminalCaptureChannel::new();
        let mut inner = SupervisorInner::new(config);
        inner.internal_fault_lane = Arc::clone(&internal_fault_lane);
        #[cfg(not(target_arch = "wasm32"))]
        let torque_shadow = inner
            .torque_shadow
            .hub()
            .cloned()
            .expect("SupervisorInner::new always pairs a TorqueShadowHub");
        inner.estop_state = estop_state.clone();
        inner.e_stop_latched = e_stop_latched.clone();
        inner.requested_estop_count = requested_estop_count.clone();
        inner.stop_gen = stop_gen.clone();
        inner.lease_gen_mirror = lease_gen_mirror.clone();
        inner.shared_closed_bitset = closed_bitset.clone();
        inner.atomic_fenced_lease_gen = atomic_fenced_lease_gen.clone();
        inner.pending_disconnect_hold = pending_disconnect_hold.clone();
        #[cfg(feature = "policy-runtime")]
        {
            inner.rt_policy_auth = rt_policy_auth.clone();
            inner.policy_run_spare = Some(crate::policy_runtime::PolicyRun::preallocated_rt_spare(
                rt_policy_auth.clone(),
            ));
            inner.atomic_diag_sealed_policy_action_id = atomic_diag_sealed_policy_action_id.clone();
            inner.atomic_closed_policy_action_id = atomic_closed_policy_action_id.clone();
        }
        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
        {
            inner.policy_cancel_slots = Arc::clone(&policy_cancel_slots);
            inner.policy_terminal_tx = Some(policy_terminal_tx);
        }
        inner.terminals = Some(Arc::clone(&terminals));
        inner.terminal_capture_tx = Some(terminal_capture_tx);
        inner.terminal_capture_channel = Some(Arc::clone(&terminal_capture_channel));
        let driver_lock_attempts = Arc::clone(&inner.driver_lock_attempts);
        let template_obs = Box::new(inner.current_observation());
        inner.template_obs_for_drain = Some(Arc::new((*template_obs).clone()));
        let initial_pub_cfg = crate::rt_core::requests::config::PublishedConfig::from_inner(&inner);
        let estop_epoch = inner.safety.epoch_mirror();
        let hub = inner.core.hub().clone();
        hub.set_boot_instant(inner.start_time);
        #[cfg(feature = "policy-runtime")]
        hub.attach_policy_diag_seal(atomic_diag_sealed_policy_action_id.clone());
        hub.update_prep_env(Arc::new(crate::rt_core::requests::actions::ActionPrepEnv::from_inner(&inner)));
        hub.set_service_identity(Arc::new(crate::rt_core::publish::ServiceIdentity::from_inner(&inner)));
        hub.set_legacy_motion_config(Arc::new(crate::rt_core::requests::motion::MotionConfigSnapshot::from_supervisor(&inner)));
        let control_frequency_hz = inner.config.control_frequency_hz.max(1);
        let control_period_us = (1e6 / f64::from(control_frequency_hz)) as u64;
        static NEXT_INSTANCE_SEQ: AtomicU64 = AtomicU64::new(1);
        let boot_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos() as u64);
        let seq = NEXT_INSTANCE_SEQ.fetch_add(1, Ordering::Relaxed);
        let instance_id = format!("armd-{}-{boot_ns:x}-{seq}", std::process::id());
        let detached_torque_pending_atomic = Arc::clone(&inner.detached_torque_pending_atomic);
        Self {
            inner: Arc::new(SupervisorLock::new(inner)),
            telemetry_tx: tx,
            terminals,
            terminal_capture_channel,
            estop_state,
            e_stop_latched,
            requested_estop_count,
            stop_gen,
            estop_epoch,
            lease_gen_mirror,
            closed_bitset,
            atomic_fenced_lease_gen,
            pending_disconnect_hold,
            #[cfg(feature = "policy-runtime")]
            rt_policy_auth,
            #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
            policy_cancel_slots,
            #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
            policy_qualification,
            #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
            policy_backend_permits,
            pending_disconnects: std::sync::Mutex::new(std::collections::VecDeque::new()),
            tick_stats: Arc::new(crate::diagnostics::tick::LockFreeTickStats::new()),
            control_sched: Arc::new(std::sync::Mutex::new(None)),
            hub,
            dispatcher_start: std::sync::Mutex::new(()),
            control_period_us: Arc::new(AtomicU64::new(control_period_us)),
            control_frequency_hz,
            obs_cache_hits: AtomicU64::new(0),
            obs_cache_misses: AtomicU64::new(0),
            obs_waits: AtomicU64::new(0),
            obs_unavailable: AtomicU64::new(0),
            config_write_mutex: Arc::new(tokio::sync::Mutex::new(())),
            published_config: Arc::new(crate::rt_core::requests::config::PublishedConfigHub::new(
                initial_pub_cfg,
                0,
            )),
            config_commit_hooks: std::sync::Mutex::new(
                crate::rt_core::requests::config::ConfigCommitTestHooks::default(),
            ),
            sequences_revision: Arc::new(AtomicU64::new(0)),
            owner: std::sync::Mutex::new(OwnerSlot::Legacy),
            owner_mode: Arc::new(AtomicU8::new(OWNER_LEGACY)),
            owner_reason: Arc::new(AtomicU8::new(crate::model::OwnerReason::Default as u8)),
            executor_base_gen: AtomicU64::new(0),
            estop_reg: Arc::new(std::sync::Mutex::new(crate::rt_core::executor::EstopRegistry::default())),
            sinks: Arc::new(std::sync::Mutex::new(None)),
            template_obs,
            legacy_loop_stats: std::sync::Mutex::new(None),
            ws_slots: Arc::new(WsConnectionSlotPool::new()),
            #[cfg(not(target_arch = "wasm32"))]
            torque_shadow,
            instance_id,
            watchdog_park_planning: Arc::new(AtomicBool::new(false)),
            executor_hover_jobs: std::sync::Mutex::new(Vec::new()),
            #[cfg(not(target_arch = "wasm32"))]
            executor_recorder: Arc::new(crate::teach::ExecutorRecorder::new()),
            legacy_torque_deadline_override_ms: AtomicU64::new(0),
            legacy_torque_notify: Arc::new(tokio::sync::Notify::new()),
            headless_detached_drain_active: Arc::new(AtomicBool::new(false)),
            detached_torque_pending_atomic,
            driver_lock_attempts,
            cell: std::sync::RwLock::new(None),
            internal_fault_lane,
            legacy_safe_hold: Arc::clone(&legacy_safe_hold),
            active_writer: std::sync::Mutex::new(WriterEndpoint::Legacy {
                control_stop: None,
                safe_hold: legacy_safe_hold,
            }),
            process_exit_requested: AtomicBool::new(false),
            #[cfg(not(target_arch = "wasm32"))]
            fail_drop_safe_hold_spawn: AtomicBool::new(false),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[doc(hidden)]
    pub fn set_fail_drop_safe_hold_spawn_for_test(&self, fail: bool) {
        self.fail_drop_safe_hold_spawn.store(fail, Ordering::Release);
    }

    pub fn internal_fault_lane(&self) -> &Arc<crate::panic_coord::InternalFaultLane> {
        &self.internal_fault_lane
    }

    #[inline]
    pub fn internal_fault_fenced(&self) -> bool {
        self.internal_fault_lane.is_fenced()
    }

    pub fn legacy_safe_hold_control(&self) -> Arc<crate::rt_core::safe_hold::SafeHoldControl> {
        Arc::clone(&self.legacy_safe_hold)
    }

    /// Registers the active legacy control loop's stop flag on the authoritative writer endpoint.
    pub fn register_legacy_control_stop(&self, stop: Arc<AtomicBool>) {
        let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
        if matches!(*w, WriterEndpoint::Legacy { .. }) {
            *w = WriterEndpoint::Legacy {
                control_stop: Some(stop),
                safe_hold: Arc::clone(&self.legacy_safe_hold),
            };
        }
    }

    /// Returns a snapshot of the authoritative active bus-writer endpoint (B4).
    pub fn active_writer_endpoint(&self) -> WriterEndpoint {
        self.active_writer
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    /// True if a detached `armd-control` thread currently holds an energized/unknown-state driver
    /// in SAFE-HOLD (e.g. after a startup timeout or stalled handback).
    pub fn has_detached_safe_hold(&self) -> bool {
        match self.active_writer_endpoint() {
            WriterEndpoint::DetachedSafeHold { safe_hold, .. } => !safe_hold.is_exited(),
            WriterEndpoint::Executor(h) => {
                let sh = h.safe_hold_control();
                sh.is_active() && !sh.is_exited()
            }
            WriterEndpoint::Legacy { safe_hold, .. } => {
                safe_hold.is_active() && !safe_hold.is_exited()
            }
            WriterEndpoint::Transferring => false,
        }
    }

    /// True ONLY once the active writer's `armd-control` thread has actually entered the
    /// SAFE-HOLD loop (`safe_hold.is_active() && !safe_hold.is_exited()`).
    pub fn is_detached_safe_hold_active(&self) -> bool {
        match self.active_writer_endpoint() {
            WriterEndpoint::DetachedSafeHold { safe_hold, .. } => {
                safe_hold.is_active() && !safe_hold.is_exited()
            }
            WriterEndpoint::Executor(h) => {
                let sh = h.safe_hold_control();
                sh.is_active() && !sh.is_exited()
            }
            WriterEndpoint::Legacy { safe_hold, .. } => {
                safe_hold.is_active() && !safe_hold.is_exited()
            }
            WriterEndpoint::Transferring => false,
        }
    }

    /// Returns the active writer's [`SafeHoldControl`] handle for observability and testing.
    pub fn active_safe_hold_control(&self) -> Arc<crate::rt_core::safe_hold::SafeHoldControl> {
        match self.active_writer_endpoint() {
            WriterEndpoint::DetachedSafeHold { safe_hold, .. } => safe_hold,
            WriterEndpoint::Executor(h) => h.safe_hold_control(),
            WriterEndpoint::Legacy { safe_hold, .. } => safe_hold,
            WriterEndpoint::Transferring => Arc::clone(&self.legacy_safe_hold),
        }
    }

    /// Current ownership mode (`OWNER_LEGACY`, `OWNER_TRANSFERRING`, `OWNER_EXECUTOR`, `OWNER_FAILED`).
    pub fn owner_mode(&self) -> u8 {
        self.owner_mode.load(Ordering::Acquire)
    }

    /// Configured control frequency (Hz, immutable after startup, lock-free).
    pub fn control_frequency_hz(&self) -> u32 {
        self.control_frequency_hz
    }

    /// Lock-free current stop generation (bumped under the owner lock on Stop).
    pub fn stop_generation(&self) -> u64 {
        self.stop_gen.load(Ordering::SeqCst)
    }

    /// Lock-free current E-stop / safety epoch id.
    pub fn estop_epoch(&self) -> u64 {
        self.estop_epoch.load(Ordering::SeqCst)
    }

    /// Lock-free mirror of the active lease generation (`0` when no lease is held).
    pub fn lease_generation(&self) -> u64 {
        self.lease_gen_mirror.load(Ordering::SeqCst)
    }

    /// Lock-free admission stamp captured ONCE at request entry (WS reader or REST entry).
    pub fn capture_admission_stamp(&self) -> AdmissionStamp {
        AdmissionStamp {
            stop_gen: self.stop_generation(),
            estop_epoch: self.estop_epoch(),
            lease_gen: self.lease_generation(),
            connection_id: 0,
            request_id: next_admission_request_id(),
        }
    }

    /// Human-readable ownership mode name exposed in `GET /api/v1/diagnostics/tick`.
    pub fn owner_mode_name(&self) -> &'static str {
        match self.owner_mode() {
            OWNER_LEGACY => "legacy",
            OWNER_TRANSFERRING => "transferring",
            OWNER_EXECUTOR => "executor",
            OWNER_FAILED => "failed",
            _ => "unknown",
        }
    }

    /// Effective control-loop owner (`Legacy`, `Transferring`, `Executor`, or `Failed`, lock-free).
    pub fn effective_owner(&self) -> crate::model::EffectiveOwner {
        match self.owner_mode() {
            OWNER_EXECUTOR => crate::model::EffectiveOwner::Executor,
            OWNER_TRANSFERRING => crate::model::EffectiveOwner::Transferring,
            OWNER_FAILED => crate::model::EffectiveOwner::Failed,
            _ => crate::model::EffectiveOwner::Legacy,
        }
    }

    /// Why the daemon is running under its current effective owner (lock-free).
    pub fn owner_reason(&self) -> crate::model::OwnerReason {
        crate::model::OwnerReason::from_u8(self.owner_reason.load(Ordering::Acquire))
    }

    /// Records the startup owner selection reason before ownership transfer or after a recoverable
    /// DRV-F1 handback to legacy.
    pub async fn set_owner_reason(&self, reason: crate::model::OwnerReason) {
        self.owner_reason.store(reason as u8, Ordering::Release);
        if self.owner_mode() == OWNER_LEGACY {
            let mut inner = self.inner.write().await;
            if !inner.detached {
                inner.owner_reason = reason;
            }
        }
    }

    /// Mock-only test hook: transitions a driverless supervisor into `OWNER_FAILED` with
    /// `StartupFailedDetachedSafeHold` so E2E tests can exercise fresh-load cockpit behavior
    /// against `503 EXECUTOR_UNAVAILABLE` gating on observation/diagnostics.
    #[doc(hidden)]
    pub async fn force_owner_failed_for_mock_test(&self) -> Result<(), String> {
        let guard = self.inner.write().await;
        if guard.driver.is_some() {
            return Err("force_owner_failed_for_mock_test refuses when a hardware driver is mounted".into());
        }
        self.published_config.set_available(false);
        self.owner_reason.store(
            crate::model::OwnerReason::StartupFailedDetachedSafeHold as u8,
            Ordering::Release,
        );
        {
            let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
            *slot = OwnerSlot::Failed;
            self.owner_mode.store(OWNER_FAILED, Ordering::Release);
        }
        drop(guard);
        Ok(())
    }

    /// Attaches the legacy periodic control loop's [`ExecutorStats`] so `GET /api/v1/diagnostics/tick`
    /// reports `skipped_releases`, monotonic release peaks/thresholds, and single-completion publication
    /// into [`LockFreeTickStats`] in Legacy mode.
    pub fn attach_legacy_loop_stats(&self, stats: Arc<crate::rt_core::executor::ExecutorStats>) {
        stats.attach_legacy_tick_ring(Arc::clone(&self.tick_stats));
        *self.legacy_loop_stats.lock().unwrap_or_else(|e| e.into_inner()) = Some(stats);
    }

    /// Signals stop on the legacy periodic control loop, recording the monotonic shutdown timestamp
    /// and waking Linux futex sleep when legacy loop stats are attached.
    pub fn signal_legacy_stop(&self, stop: &AtomicBool) {
        let stats = self.legacy_loop_stats.lock().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(stats) = stats {
            stats.signal_stop(stop);
        } else {
            stop.store(true, Ordering::SeqCst);
        }
    }

    /// Handle to the synchronous RT executor when ownership has transferred.
    pub fn executor_handle(&self) -> Option<crate::rt_core::executor::RtExecutorHandle> {
        let slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
        match &*slot {
            OwnerSlot::Executor(h) => Some(h.clone()),
            _ => None,
        }
    }

    /// Re-publishes the revision-tagged off-thread [`ActionPrepEnv`] snapshot into [`StateHub`]
    /// from the Legacy owner lock (no-op in Executor mode, where config mutations update `StateHub`).
    pub async fn refresh_prep_env(&self) {
        if self.owner_mode() != OWNER_LEGACY {
            return;
        }
        let inner = self.inner.read().await;
        if !inner.detached {
            self.hub.update_prep_env(Arc::new(
                crate::rt_core::requests::actions::ActionPrepEnv::from_inner(&inner),
            ));
            self.hub.set_service_identity(Arc::new(
                crate::rt_core::publish::ServiceIdentity::from_inner(&inner),
            ));
        }
    }

    pub(super) fn check_legacy_under_guard(&self, inner: &SupervisorInner) -> Result<(), ArmError> {
        let mode = self.owner_mode.load(Ordering::Acquire);
        if !inner.detached && mode == OWNER_LEGACY && !self.has_detached_safe_hold() {
            Ok(())
        } else if mode == OWNER_EXECUTOR {
            Err(owner_transferred())
        } else {
            Err(executor_unavailable())
        }
    }

    pub(super) fn drain_diag(&self) {
        let sink = self.sinks.lock().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(s) = sink {
            s.drain_diag(&self.tick_stats);
        }
    }

    /// Transfers ownership of `SupervisorInner` (`ArmCore` + bus driver) once at startup
    /// into the synchronous RT executor OS thread.
    pub async fn start_executor(
        &self,
        cfg: crate::rt_core::executor::ExecutorConfig,
    ) -> Result<crate::rt_core::executor::RtExecutorHandle, ArmError> {
        self.start_executor_with_hooks(cfg, crate::rt_core::executor::PersistTestHooks::default()).await
    }

    #[doc(hidden)]
    pub async fn start_executor_with_hooks(
        &self,
        cfg: crate::rt_core::executor::ExecutorConfig,
        hooks: crate::rt_core::executor::PersistTestHooks,
    ) -> Result<crate::rt_core::executor::RtExecutorHandle, ArmError> {
        self.ensure_dispatcher()?;
        let saved_legacy_stop = {
            let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
            if !matches!(*slot, OwnerSlot::Legacy) {
                return Err(ArmError::Rejected {
                    status: 409,
                    code: "OWNER_ALREADY_TRANSFERRED".into(),
                    message: "RT executor ownership can only be transferred once from Legacy mode".into(),
                });
            }
            *slot = OwnerSlot::Transferring;
            let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
            let prev_stop = match &*w {
                WriterEndpoint::Legacy { control_stop, .. } => control_stop.clone(),
                _ => None,
            };
            *w = WriterEndpoint::Transferring;
            drop(w);
            self.owner_mode.store(OWNER_TRANSFERRING, Ordering::Release);
            prev_stop
        };
        let mut rollback = TransferGuard {
            sup: self,
            armed: true,
            saved_legacy_stop,
        };

        // The single `.await` point in handoff: if cancelled here, `TransferGuard` rolls back to `Legacy`.
        let mut guard = self.inner.write().await;

        let restore_legacy = |rollback: &mut TransferGuard<'_>| {
            {
                let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                if matches!(*slot, OwnerSlot::Transferring) {
                    *slot = OwnerSlot::Legacy;
                    self.owner_mode.store(OWNER_LEGACY, Ordering::Release);
                }
            }
            {
                let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                if matches!(*w, WriterEndpoint::Transferring) {
                    *w = WriterEndpoint::Legacy {
                        control_stop: rollback.saved_legacy_stop.take(),
                        safe_hold: Arc::clone(&self.legacy_safe_hold),
                    };
                }
            }
            rollback.armed = false;
        };

        // Zero `.await` points below this line: drain all socket-close disconnects queued while
        // the write lock was held or during `OWNER_TRANSFERRING` BEFORE checking transfer state,
        // seeding `ConnLifecycleClient`, or spawning RT, so RT never inherits a stale connection
        // or lease.
        if !guard.detached {
            let _ = self.drain_pending_disconnects_under_guard(&mut guard, None);
            guard.purge_closed_live_connections();
        }
        if guard.detached
            || guard.calibration_busy
            || guard.motor_zero_busy
            || guard.zero_check_persisting.is_some()
            || guard.gravity_comp.is_some()
            || guard.macro_run.is_some()
            || guard.mit.owner != crate::mit::MitOwner::Off
            || guard.mit.revoke_pending
            || guard.mode_switch.active_session.is_some()
        {
            restore_legacy(&mut rollback);
            return Err(ArmError::Rejected {
                status: 409,
                code: "TRANSFER_BLOCKED".into(),
                message: "Cannot transfer ownership while calibration, zero_check persistence, motor_zero, gravity compensation, macro action, or MIT tracking (active or lost) is active".into(),
            });
        }
        #[cfg(feature = "policy-runtime")]
        if guard.policy_loading || guard.policy_run.is_some() {
            restore_legacy(&mut rollback);
            return Err(ArmError::Rejected {
                status: 409,
                code: "TRANSFER_BLOCKED".into(),
                message: "Cannot transfer ownership while policy runtime is active or loading".into(),
            });
        }
        if guard.config.dof > crate::rt_core::mailbox::MAX_DOF {
            restore_legacy(&mut rollback);
            return Err(ArmError::Rejected {
                status: 503,
                code: "UNSUPPORTED_RT_CONFIG".into(),
                message: format!(
                    "Profile DOF {} exceeds synchronous RT executor mailbox limit {}; run under legacy owner",
                    guard.config.dof,
                    crate::rt_core::mailbox::MAX_DOF
                ),
            });
        }
        // Pre-allocate MIT runtime vectors before handing ownership to `armd-control` so
        // activation and handback perform zero heap allocations on RT.
        let j_len = guard.mit.joints.len();
        if guard.mit.joints.capacity() < crate::rt_core::mailbox::MAX_DOF {
            guard.mit.joints.reserve(crate::rt_core::mailbox::MAX_DOF.saturating_sub(j_len));
        }
        let ff_len = guard.mit.last_ff.len();
        if guard.mit.last_ff.capacity() < crate::rt_core::mailbox::MAX_DOF {
            guard.mit.last_ff.reserve(crate::rt_core::mailbox::MAX_DOF.saturating_sub(ff_len));
        }
        guard.mit_gate_proof = None;
        // Flush any legacy E-stop acks already applied before handoff so waiting callers
        // receive their exact recorded (state, epoch) instead of stalling after transfer,
        // and drop any legacy retired fault Arcs off-thread before transferring ownership.
        // Driver preflight (`position_limits` / `supports_rt_executor`) executes inside
        // `armd-control`'s bounded `StartupSlot` transaction so no driver callback can stall
        // or unwind on the caller thread while `SupervisorLock` is held.
        Self::flush_legacy_estop_acks_into(&self.estop_reg, &mut guard);
        guard.safety.drain_retired_state_arcs_off_thread();
        guard.tracking.normalize_spares_off_thread();
        guard.prepare_workspace_eval_bufs();
        guard.prepare_park_release_spare();
        guard.refresh_schema_cache();

        let initial_gen = self.inner.generation();
        let mut candidate_pub_cfg =
            crate::rt_core::requests::config::PublishedConfig::from_inner(&guard);
        candidate_pub_cfg.gravity_comp_supported =
            guard.gravity_comp_support_for_owner(true).is_ok();
        let staged_hover_jobs = guard.hover_jobs.clone();
        let staged_service_identity = Arc::new(
            crate::rt_core::publish::ServiceIdentity::from_published_config(
                guard.core.id(),
                cfg.boot,
                &candidate_pub_cfg,
            ),
        );
        let shell = SupervisorInner::detached_shell(
            &guard.config,
            self.estop_state.clone(),
            self.stop_gen.clone(),
            self.lease_gen_mirror.clone(),
            self.closed_bitset.clone(),
            self.atomic_fenced_lease_gen.clone(),
            self.pending_disconnect_hold.clone(),
        );
        // Clone the preallocated legacy waiter slots and transform any in-flight LegacyWs torque
        // transition BEFORE moving `real` into the executor:
        // - LegacyWs enable -> DetachedCancel (`Cancelling`) so the executor never retries or arms
        //   an abandoned legacy WS enable and cleanly calls `drv.abort_enable()`.
        // - LegacyWs disable -> Detached so dispatched/protective hardware disables complete cleanly.
        // Waiters are settled as `OwnerTransferred` ONLY AFTER `spawn_executor` commits (or fails
        // unrecoverably); on rollback (`Recoverable` or `into_exclusive` failure), the original
        // transition and waiter remain intact.
        let handoff_slots = guard.legacy_torque_slots.clone();
        let saved_legacy_transition = guard.torque_transition;
        match guard.torque_transition {
            Some(driver_transitions::TorqueTransition::Enable(mut reducer))
                if matches!(reducer.origin, driver_transitions::EnableOrigin::LegacyWs { .. }) =>
            {
                reducer.origin = driver_transitions::EnableOrigin::DetachedCancel;
                reducer.phase = driver_transitions::EnableReducerPhase::Cancelling;
                reducer.authority = None;
                guard.torque_transition = Some(driver_transitions::TorqueTransition::Enable(reducer));
            }
            Some(driver_transitions::TorqueTransition::Disable {
                origin: driver_transitions::DisableOrigin::LegacyWs { deadline, .. },
                kind,
            }) => {
                // Preserve `Some(deadline)` across owner handoff so an undispatched disable still
                // expires cleanly on the new owner without transmitting past its deadline; once
                // dispatched (or protective), the new owner clears the deadline and completes it.
                let handoff_dl = if matches!(kind, crate::driver::DisableKind::DisableProtective { .. })
                    || guard.torque_intent == driver_transitions::TorqueIntent::Off
                {
                    None
                } else {
                    Some(deadline)
                };
                guard.torque_transition = Some(driver_transitions::TorqueTransition::Disable {
                    origin: driver_transitions::DisableOrigin::Detached { deadline: handoff_dl },
                    kind,
                });
            }
            _ => {}
        }
        let mut real = std::mem::replace(&mut *guard, shell);
        let live_ids: Vec<u64> = real.live_connection_ids().collect();
        let seeded_ws_sinks = self.ws_slots.sinks_for_live_ids(&live_ids);
        if let Some(slot) = real.driver.take() {
            match slot.into_exclusive() {
                Ok(exclusive) => real.driver = Some(exclusive),
                Err((legacy_slot, reason)) => {
                    real.driver = Some(legacy_slot);
                    *guard = real;
                    guard.torque_transition = saved_legacy_transition;
                    restore_legacy(&mut rollback);
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "UNSUPPORTED_RT_DRIVER".into(),
                        message: format!("Cannot transfer ownership: {reason}"),
                    });
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            real.flush_torque_shadow();
            real.torque_shadow
                .set_owner(crate::torque::shadow::ShadowOwner::Executor);
            if let Err(e) = self.torque_shadow.start_worker() {
                tracing::warn!("Failed to start torque shadow worker thread on executor handoff: {e}");
            }
        }

        match crate::rt_core::executor::spawn_executor(
            real,
            cfg,
            initial_gen,
            Arc::clone(&self.config_write_mutex),
            Arc::clone(&self.published_config),
            Arc::clone(&self.estop_reg),
            Arc::clone(&self.tick_stats),
            Arc::clone(&self.control_sched),
            Arc::clone(&self.control_period_us),
            Arc::clone(&self.internal_fault_lane),
            hooks,
        ) {
            Ok((handle, sinks, ready_payload, startup_gate)) => {
                // Single startup activation barrier (R5 Major 2 / M2):
                // Commit staged hub snapshots, observation prewarm, scheduler readiness,
                // published_config, sinks, and `OWNER_EXECUTOR` atomically strictly OUTSIDE the
                // StartupGate mutex and strictly BEFORE releasing `armd-control` to Release #1.
                if Instant::now() >= startup_gate.deadline() {
                    startup_gate.mark_aborted();
                    let mut any_settled = false;
                    for slot in &handoff_slots {
                        any_settled |= slot.settle_owner_transferred();
                    }
                    self.published_config.set_available(false);
                    {
                        let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                        *slot = OwnerSlot::Failed;
                        self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                    }
                    {
                        let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                        *w = WriterEndpoint::DetachedSafeHold {
                            safe_hold: handle.safe_hold_control(),
                            control_stop: Arc::new(AtomicBool::new(true)),
                        };
                    }
                    rollback.armed = false;
                    drop(guard);
                    if any_settled {
                        self.legacy_torque_notify.notify_waiters();
                    }
                    return Err(ArmError::Internal(
                        "RT executor startup timed out before activation barrier".into(),
                    ));
                }
                let mut any_settled = false;
                for slot in &handoff_slots {
                    any_settled |= slot.settle_owner_transferred();
                }
                handle.seed_connection_reply_sinks(seeded_ws_sinks);
                self.hub.set_motion_config(ready_payload.staged_motion_cfg);
                self.hub.update_prep_env(ready_payload.staged_prep_env);
                self.hub.set_service_identity(staged_service_identity);
                self.hub.set_schema_cache(
                    ready_payload.staged_schema.0,
                    ready_payload.staged_schema.1,
                );
                self.hub.install_prewarm_template_and_front(
                    ready_payload.staged_obs_template,
                    ready_payload.staged_front_obs,
                );
                *self
                    .control_sched
                    .lock()
                    .unwrap_or_else(|e| e.into_inner()) = Some(ready_payload.staged_sched);
                if let Some(readiness) = ready_payload.staged_readiness {
                    crate::rt_core::sched::set_readiness(readiness);
                }
                self.published_config.publish(candidate_pub_cfg, initial_gen);
                *self
                    .executor_hover_jobs
                    .lock()
                    .unwrap_or_else(|e| e.into_inner()) = staged_hover_jobs;
                *self.sinks.lock().unwrap_or_else(|e| e.into_inner()) = Some(sinks);
                self.executor_base_gen.store(initial_gen, Ordering::Release);
                {
                    let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                    *slot = OwnerSlot::Executor(handle.clone());
                    self.owner_mode.store(OWNER_EXECUTOR, Ordering::Release);
                }
                {
                    let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                    *w = WriterEndpoint::Executor(handle.clone());
                }
                if !startup_gate.mark_activated() {
                    // `armd-control` already timed out waiting for activation: roll owner to
                    // `Failed` and record detached SAFE-HOLD endpoint.
                    self.published_config.set_available(false);
                    {
                        let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                        *slot = OwnerSlot::Failed;
                        self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                    }
                    {
                        let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                        *w = WriterEndpoint::DetachedSafeHold {
                            safe_hold: handle.safe_hold_control(),
                            control_stop: Arc::new(AtomicBool::new(true)),
                        };
                    }
                    rollback.armed = false;
                    drop(guard);
                    return Err(ArmError::Internal(
                        "RT executor startup activation expired".into(),
                    ));
                }
                rollback.armed = false;
                drop(guard);
                if any_settled {
                    self.legacy_torque_notify.notify_waiters();
                }
                // Forward any socket-close disconnects queued in the tiny window between draining
                // `guard` and publishing `OWNER_EXECUTOR` directly to the new RT owner.
                self.forward_pending_disconnects_to_executor(&handle);
                self.hub.wake();
                Ok(handle)
            }
            Err(crate::rt_core::executor::SpawnExecutorError::Recoverable {
                mut core,
                unsupported_reason,
                msg,
            }) => {
                if let Some(mut slot) = core.driver.take() {
                    if let Some(drv) = slot.as_exclusive_mut() {
                        drv.set_rt_mode(false);
                    }
                    core.driver = Some(slot.into_legacy(Arc::clone(&self.driver_lock_attempts)));
                }
                *guard = *core;
                guard.torque_transition = saved_legacy_transition;
                #[cfg(not(target_arch = "wasm32"))]
                guard
                    .torque_shadow
                    .set_owner(crate::torque::shadow::ShadowOwner::Legacy);
                let _ = self.drain_pending_disconnects_under_guard(&mut guard, None);
                guard.purge_closed_live_connections();
                restore_legacy(&mut rollback);
                drop(guard);
                self.try_drain_pending_disconnects_legacy();
                if let Some(reason) = unsupported_reason {
                    Err(ArmError::Rejected {
                        status: 503,
                        code: "UNSUPPORTED_RT_DRIVER".into(),
                        message: format!("Cannot transfer ownership: {reason}"),
                    })
                } else {
                    Err(ArmError::Internal(msg))
                }
            }
            Err(crate::rt_core::executor::SpawnExecutorError::Fatal(msg)) => {
                let mut any_settled = false;
                for slot in &handoff_slots {
                    any_settled |= slot.settle_owner_transferred();
                }
                self.published_config.set_available(false);
                {
                    let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                    *slot = OwnerSlot::Failed;
                    self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                }
                {
                    // `Fatal` only fires after `armd-control` confirmed disarmed/driverless and
                    // dropped `core`; publish a terminal exited endpoint so `active_writer` does
                    // not remain stuck in `Transferring`.
                    let exited_sh = Arc::new(crate::rt_core::safe_hold::SafeHoldControl::new());
                    exited_sh.record_process_exit_outcome_lockfree(&PanicDisableOutcome::NoDriver);
                    exited_sh.mark_exited();
                    let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                    *w = WriterEndpoint::DetachedSafeHold {
                        safe_hold: exited_sh,
                        control_stop: Arc::new(AtomicBool::new(true)),
                    };
                }
                rollback.armed = false;
                drop(guard);
                if any_settled {
                    self.legacy_torque_notify.notify_waiters();
                }
                Err(ArmError::Internal(msg))
            }
            Err(crate::rt_core::executor::SpawnExecutorError::FatalDetachedSafeHold {
                msg,
                safe_hold,
                control_stop,
            }) => {
                let mut any_settled = false;
                for slot in &handoff_slots {
                    any_settled |= slot.settle_owner_transferred();
                }
                self.published_config.set_available(false);
                {
                    let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                    *slot = OwnerSlot::Failed;
                    self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                }
                {
                    let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                    *w = WriterEndpoint::DetachedSafeHold {
                        safe_hold,
                        control_stop,
                    };
                }
                rollback.armed = false;
                drop(guard);
                if any_settled {
                    self.legacy_torque_notify.notify_waiters();
                }
                Err(ArmError::Internal(msg))
            }
        }
    }

    /// Hands ownership cleanly from `OWNER_EXECUTOR` back to `OWNER_LEGACY` when the driver is
    /// already confirmed physically disarmed. Refuses energized handback without a prepared live
    /// periodic legacy receiver — energized callers must use [`Self::handback_to_legacy_with_live_loop`]
    /// so an energized arm is never left without a periodic writer (DRV-F1 B2).
    pub async fn handback_to_legacy(&self, timeout: Duration) -> Result<(), ArmError> {
        self.handback_to_legacy_inner(timeout, None).await
    }

    /// Starts a live legacy control loop parked on a ready gate, executes energized handback,
    /// and unparks the legacy loop immediately after restoring `self.inner` and publishing
    /// `OWNER_LEGACY` (BEFORE worker cleanup finishes — B2). On handback failure, cancels the
    /// parked thread cleanly so no second writer ever runs.
    pub async fn handback_to_legacy_with_live_loop(
        self: &Arc<Self>,
        timeout: Duration,
        period: Duration,
    ) -> Result<(Arc<AtomicBool>, std::thread::JoinHandle<()>), ArmError> {
        let ready = Arc::new((std::sync::Mutex::new(Option::<bool>::None), std::sync::Condvar::new()));
        let control_stop = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(crate::rt_core::executor::ExecutorStats::default());
        let sup_bg = Arc::clone(self);
        let ready_bg = Arc::clone(&ready);
        let stop_bg = Arc::clone(&control_stop);
        let stats_bg = Arc::clone(&stats);
        let runtime = tokio::runtime::Handle::current();
        let jh = std::thread::Builder::new()
            .name("armd-control-legacy".into())
            .spawn(move || {
                let (lock, cv) = &*ready_bg;
                let proceed = {
                    let mut g = lock.lock().unwrap_or_else(|e| e.into_inner());
                    while g.is_none() {
                        g = cv.wait(g).unwrap_or_else(|e| e.into_inner());
                    }
                    g.unwrap_or(false)
                };
                if !proceed {
                    return;
                }
                sup_bg.attach_legacy_loop_stats(stats_bg.clone());
                sup_bg.run_legacy_control_loop_blocking(period, stop_bg, stats_bg, runtime, None);
            })
            .map_err(|e| ArmError::Internal(format!("failed to prepare legacy receiver thread: {e}")))?;

        match self
            .handback_to_legacy_inner(timeout, Some((&ready, &control_stop)))
            .await
        {
            Ok(()) => Ok((control_stop, jh)),
            Err(e) => {
                {
                    let (lock, cv) = &*ready;
                    *lock.lock().unwrap_or_else(|e| e.into_inner()) = Some(false);
                    cv.notify_all();
                }
                let _ = jh.join();
                Err(e)
            }
        }
    }

    pub(super) async fn handback_to_legacy_inner(
        &self,
        timeout: Duration,
        live_receiver: Option<(
            &Arc<(std::sync::Mutex<Option<bool>>, std::sync::Condvar)>,
            &Arc<AtomicBool>,
        )>,
    ) -> Result<(), ArmError> {
        let deadline = Instant::now() + timeout;
        let handle = {
            let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
            match std::mem::replace(&mut *slot, OwnerSlot::Transferring) {
                OwnerSlot::Executor(h) => {
                    self.owner_mode.store(OWNER_TRANSFERRING, Ordering::Release);
                    h
                }
                other => {
                    *slot = other;
                    return Err(ArmError::Rejected {
                        status: 409,
                        code: "OWNER_NOT_EXECUTOR".into(),
                        message: "Handback requires active RT executor ownership".into(),
                    });
                }
            }
        };

        struct HandbackGuard<'a> {
            sup: &'a ArmSupervisor,
            handle: Option<crate::rt_core::executor::RtExecutorHandle>,
        }
        impl Drop for HandbackGuard<'_> {
            fn drop(&mut self) {
                if let Some(h) = self.handle.take() {
                    let mut slot = self.sup.owner.lock().unwrap_or_else(|e| e.into_inner());
                    if matches!(*slot, OwnerSlot::Transferring) {
                        *slot = OwnerSlot::Executor(h);
                        self.sup.owner_mode.store(OWNER_EXECUTOR, Ordering::Release);
                    }
                }
            }
        }
        let mut hb_guard = HandbackGuard {
            sup: self,
            handle: Some(handle.clone()),
        };

        // Acquire the legacy shell write lock BEFORE signaling `armd-control` to stop.
        // If cancelled or timed out here, `HandbackGuard` restores `OWNER_EXECUTOR` cleanly.
        let rem = deadline.saturating_duration_since(Instant::now());
        if rem.is_zero() {
            return Err(ArmError::Rejected {
                status: 503,
                code: "HANDBACK_TIMED_OUT".into(),
                message: "Handback deadline expired before acquiring legacy shell".into(),
            });
        }
        let mut guard = match tokio::time::timeout(rem, self.inner.write()).await {
            Ok(g) => g,
            Err(_) => {
                return Err(ArmError::Rejected {
                    status: 503,
                    code: "HANDBACK_TIMED_OUT".into(),
                    message: "Handback timed out waiting for legacy shell lock".into(),
                });
            }
        };

        // Reject bare energized handback without a prepared live receiver BEFORE signaling stop (B2).
        if live_receiver.is_none() && !handle.shutdown_ready().unwrap_or(false) {
            return Err(ArmError::Rejected {
                status: 409,
                code: "HANDBACK_REQUIRES_LIVE_RECEIVER".into(),
                message: "Energized handback requires a prepared live legacy receiver (use handback_to_legacy_with_live_loop)".into(),
            });
        }

        // Zero `.await` points below: commit handback request on `armd-control` within the SAME
        // deadline, carrying `live_receiver.is_some()` to the owner-side commit point (DRV-F1 B4).
        match handle.take_handback_core_until(deadline, live_receiver.is_some()) {
            Ok(mut core) => {
                if let Some(slot) = core.driver.take() {
                    core.driver = Some(slot.into_legacy(Arc::clone(&self.driver_lock_attempts)));
                }
                core.detached = false;
                core.rt_executor_owned = false;
                core.advance_owner_generation();
                core.internal_fault_lane = Arc::clone(&self.internal_fault_lane);
                #[cfg(not(target_arch = "wasm32"))]
                {
                    core.flush_torque_shadow();
                    core.torque_shadow
                        .set_owner(crate::torque::shadow::ShadowOwner::Legacy);
                    self.torque_shadow.wake();
                }
                *guard = *core;
                guard.refresh_schema_cache();
                guard.hover_jobs = self
                    .executor_hover_jobs
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
                let _ = self.drain_pending_disconnects_under_guard(&mut guard, None);
                guard.purge_closed_live_connections();
                *self.sinks.lock().unwrap_or_else(|e| e.into_inner()) = None;
                {
                    let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                    *slot = OwnerSlot::Legacy;
                    self.owner_mode.store(OWNER_LEGACY, Ordering::Release);
                }
                {
                    let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                    *w = WriterEndpoint::Legacy {
                        control_stop: live_receiver.map(|(_, stop)| Arc::clone(stop)),
                        safe_hold: Arc::clone(&self.legacy_safe_hold),
                    };
                }
                hb_guard.handle = None;
                drop(guard);
                // Release the pre-spawned live legacy receiver IMMEDIATELY after dropping `guard`
                // and BEFORE running bounded worker teardown so energized holds continue with
                // minimal inter-frame gap (B2).
                if let Some((ready, _)) = live_receiver {
                    let (lock, cv) = &**ready;
                    *lock.lock().unwrap_or_else(|e| e.into_inner()) = Some(true);
                    cv.notify_all();
                }
                handle.finish_handback_cleanup_until(deadline);
                self.try_drain_pending_disconnects_legacy();
                Ok(())
            }
            Err(crate::rt_core::executor::HandbackFailure::TimedOutSafeHold) => {
                self.published_config.set_available(false);
                {
                    let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                    *slot = OwnerSlot::Failed;
                    self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                }
                {
                    let mut w = self.active_writer.lock().unwrap_or_else(|e| e.into_inner());
                    *w = WriterEndpoint::DetachedSafeHold {
                        safe_hold: handle.safe_hold_control(),
                        control_stop: Arc::new(AtomicBool::new(true)),
                    };
                }
                hb_guard.handle = None;
                drop(guard);
                Err(ArmError::Rejected {
                    status: 503,
                    code: "HANDBACK_STALLED_SAFE_HOLD".into(),
                    message: "Energized handback timed out; exclusive driver retained in SAFE-HOLD"
                        .into(),
                })
            }
            Err(crate::rt_core::executor::HandbackFailure::ExecutorExited) => {
                self.published_config.set_available(false);
                {
                    let mut slot = self.owner.lock().unwrap_or_else(|e| e.into_inner());
                    *slot = OwnerSlot::Failed;
                    self.owner_mode.store(OWNER_FAILED, Ordering::Release);
                }
                hb_guard.handle = None;
                drop(guard);
                Err(ArmError::Rejected {
                    status: 503,
                    code: "EXECUTOR_STOPPED".into(),
                    message: "RT executor exited before handback completed".into(),
                })
            }
        }
    }
}
