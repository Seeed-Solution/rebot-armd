//! Off-thread recording deletion claims, hover jobs, power/torque diagnostics, and tick stats.

#[allow(unused_imports)]
use super::*;

// scaffold-extra-0
impl ArmSupervisor {
    /// Owner-aware storage of a `measure_multiview` measurement payload for action `id`.
    /// On `OWNER_EXECUTOR`, stores in `self.hub` (never touching `SupervisorLock` or `armd-control`)
    /// so `StateHub::refresh` attaches `action.measurement` into every published observation and
    /// delivered terminal identically to the Legacy owner.
    pub async fn record_action_measurement(&self, id: u64, value: serde_json::Value) {
        if self.owner_mode() == OWNER_EXECUTOR {
            self.hub.record_action_measurement(id, value);
        } else {
            self.inner()
                .write()
                .await
                .update_action(id, |a| a.measurement = Some(value));
        }
    }

    /// Owner-aware readback of the measurement payload recorded for action `id`.
    /// Never touches `SupervisorLock` when `OWNER_EXECUTOR` is active.
    pub async fn action_measurement(&self, id: u64) -> Option<serde_json::Value> {
        if self.owner_mode() == OWNER_EXECUTOR {
            self.hub.action_measurement(id)
        } else {
            let inner = self.inner().read().await;
            inner
                .last_action
                .as_ref()
                .filter(|a| a.id == id)
                .and_then(|a| a.measurement.clone())
                .or_else(|| {
                    inner
                        .previous_action
                        .as_ref()
                        .filter(|(a, _)| a.id == id)
                        .and_then(|(a, _)| a.measurement.clone())
                })
        }
    }

    /// Owner-aware storage of a `calib_hover` job record (never touches `SupervisorLock` in
    /// `OWNER_EXECUTOR` mode, and re-checks detachment if a handoff lands while waiting on the
    /// legacy write lock).
    pub async fn put_hover_job(&self, job: crate::calib_hover::HoverJob) {
        if self.owner_mode() == OWNER_EXECUTOR {
            self.put_executor_hover_job(job);
            return;
        }
        let mut inner = self.inner().write().await;
        if inner.detached {
            drop(inner);
            if self.owner_mode() == OWNER_EXECUTOR {
                self.put_executor_hover_job(job);
            }
            return;
        }
        inner.hover_jobs.retain(|j| j.id != job.id);
        inner.hover_jobs.push(job);
        let n = inner.hover_jobs.len();
        if n > crate::calib_hover::KEEP_JOBS {
            inner.hover_jobs.drain(..n - crate::calib_hover::KEEP_JOBS);
        }
    }

    fn put_executor_hover_job(&self, job: crate::calib_hover::HoverJob) {
        let mut jobs = self
            .executor_hover_jobs
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        jobs.retain(|j| j.id != job.id);
        jobs.push(job);
        let n = jobs.len();
        if n > crate::calib_hover::KEEP_JOBS {
            jobs.drain(..n - crate::calib_hover::KEEP_JOBS);
        }
    }

    /// Owner-aware snapshot of stored `calib_hover` jobs (never touches `SupervisorLock` in
    /// `OWNER_EXECUTOR` mode, and re-checks detachment if a handoff lands while waiting on the
    /// legacy read lock).
    pub async fn hover_jobs_snapshot(&self) -> Vec<crate::calib_hover::HoverJob> {
        if self.owner_mode() == OWNER_EXECUTOR {
            return self
                .executor_hover_jobs
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone();
        }
        let inner = self.inner().read().await;
        if inner.detached {
            drop(inner);
            if self.owner_mode() == OWNER_EXECUTOR {
                return self
                    .executor_hover_jobs
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
            }
            return Vec::new();
        }
        inner.hover_jobs.clone()
    }

    /// Current live `SequenceStore::revision` mirror used to linearize episode deletes against
    /// concurrent `Sequences` save commits without blocking on `config_write_mutex`.
    pub fn sequences_revision(&self) -> u64 {
        self.sequences_revision.load(Ordering::Acquire)
    }

    pub(crate) fn set_sequences_revision(&self, rev: u64) {
        self.sequences_revision.store(rev, Ordering::Release);
    }

    /// Checks on the single live owner (`Legacy` or `Executor`) whether episode `id` is free of
    /// saved, running, or reserved sequence references (`409 IN_USE` otherwise).
    pub async fn check_episode_delete_allowed(&self, id: &str) -> Result<(), ArmError> {
        use crate::rt_core::requests::config::{ConfigReply, ConfigRequest};
        if self.owner_mode() == OWNER_LEGACY {
            let store = {
                let inner = self.inner.read().await;
                inner.episode_delete_allowed(id)?;
                inner.episodes.clone()
            };
            if store.is_pinned(id) {
                return Err(ArmError::detailed(
                    409,
                    "IN_USE",
                    format!("Episode '{id}' is reserved by an in-progress sequence save"),
                    serde_json::json!({"sequences": ["pending_save"]}),
                ));
            }
            return Ok(());
        }
        match self
            .dispatch_config_request_owned_reliable(ConfigRequest::CheckEpisodeDeleteAllowed(
                crate::model::HolderLabel::new(id),
            ))
            .await
        {
            Ok(ConfigReply::EpisodeDeleteAllowed(true)) => Ok(()),
            Ok(ConfigReply::EpisodeDeleteAllowed(false)) => {
                let mut users = self
                    .published_config()
                    .await
                    .map(|p| p.sequences.using_episode(id))
                    .unwrap_or_default();
                if users.is_empty() {
                    users.push("pending_save".to_string());
                }
                Err(ArmError::detailed(
                    409,
                    "IN_USE",
                    format!("Episode '{id}' is used by {} sequence(s)", users.len()),
                    serde_json::json!({"sequences": users}),
                ))
            }
            Err(e) => Err(e.to_arm_error()),
            _ => Err(ArmError::Internal("unexpected CheckEpisodeDeleteAllowed reply".into())),
        }
    }

    /// Linearizably verifies episode `id` is unreferenced (saved, running, or reserved) on the
    /// live owner, removes it from the `EpisodeStore` catalog on a blocking thread (off the Tokio
    /// worker), and returns its RAII deletion claim guard (`Some(guard)`) if removed, `None` if
    /// absent, or `Err(409 IN_USE)` if referenced. Dropping the guard on async cancellation marks
    /// the claim retryable so the episode ID is never stranded.
    pub async fn take_episode_for_delete_claim(
        &self,
        id: &str,
    ) -> Result<Option<crate::recording::store::EpisodeDeleteClaimGuard>, ArmError> {
        use crate::recording::store::TakeEpisodeOutcome;
        loop {
            let rev0 = self.sequences_revision();
            self.check_episode_delete_allowed(id).await?;
            let store = self.try_episode_store().await?;
            let id_owned = id.to_string();
            let seq_rev = Arc::clone(&self.sequences_revision);
            let outcome = tokio::task::spawn_blocking(move || {
                store.take_if_unpinned_and_rev_matches(&id_owned, &seq_rev, rev0)
            })
            .await
            .map_err(|e| ArmError::Internal(format!("episode delete take failed: {e}")))?;
            match outcome {
                TakeEpisodeOutcome::Removed(guard) => return Ok(Some(guard)),
                TakeEpisodeOutcome::NotFound => return Ok(None),
                TakeEpisodeOutcome::Pinned => {
                    let mut users = self
                        .published_config()
                        .await
                        .map(|p| p.sequences.using_episode(id))
                        .unwrap_or_default();
                    if users.is_empty() {
                        users.push("pending_save".to_string());
                    }
                    return Err(ArmError::detailed(
                        409,
                        "IN_USE",
                        format!("Episode '{id}' is used by {} sequence(s)", users.len()),
                        serde_json::json!({"sequences": users}),
                    ));
                }
                TakeEpisodeOutcome::SequenceRevisionChanged => continue,
            }
        }
    }

    /// Linearizably verifies episode `id` is unreferenced (saved, running, or reserved) on the
    /// live owner and removes it from the `EpisodeStore` catalog. Returns `Ok(true)` if removed,
    /// `Ok(false)` if absent, or `Err(409 IN_USE)` if referenced or pinned by a sequence save.
    pub async fn take_episode_for_delete(&self, id: &str) -> Result<bool, ArmError> {
        self.take_episode_for_delete_claim(id)
            .await
            .map(|opt| opt.is_some())
    }

    /// Linearizably verifies every episode in `dataset` is unreferenced (saved, running, or
    /// reserved) and no recording/import pins `dataset`, then atomically removes all of them from
    /// the `EpisodeStore` catalog while returning their deletion claim tokens and a
    /// [`DatasetDeleteGuard`] blocking new pins.
    pub async fn take_dataset_episodes_for_delete(
        &self,
        dataset: &str,
    ) -> Result<(Vec<(String, u64)>, crate::recording::store::DatasetDeleteGuard), ArmError> {
        use crate::recording::store::TakeDatasetOutcome;
        loop {
            let rev0 = self.sequences_revision();
            let store = self.try_episode_store().await?;
            let ids = store.episodes_of(dataset);
            for id in &ids {
                self.check_episode_delete_allowed(id).await?;
            }
            match store.take_dataset_if_unpinned_and_rev_matches(
                dataset,
                &ids,
                &self.sequences_revision,
                rev0,
            ) {
                TakeDatasetOutcome::Removed(taken, guard) => return Ok((taken, guard)),
                TakeDatasetOutcome::DatasetRecordingBusy => {
                    return Err(ArmError::detailed(
                        409,
                        "RECORDER_BUSY",
                        "a recording into this dataset is running",
                        serde_json::json!({}),
                    ));
                }
                TakeDatasetOutcome::Pinned(pinned_id) => {
                    let mut users = self
                        .published_config()
                        .await
                        .map(|p| p.sequences.using_episode(&pinned_id))
                        .unwrap_or_default();
                    if users.is_empty() {
                        users.push("pending_save".to_string());
                    }
                    return Err(ArmError::detailed(
                        409,
                        "IN_USE",
                        format!("Episode '{pinned_id}' is used by {} sequence(s)", users.len()),
                        serde_json::json!({"sequences": users}),
                    ));
                }
                TakeDatasetOutcome::SequenceRevisionChanged => continue,
            }
        }
    }

    /// Current monotonic stop generation across both `Legacy` and `Executor` owners (lock-free).
    pub fn current_stop_gen(&self) -> u64 {
        let legacy = self.stop_gen.load(Ordering::Acquire);
        self.executor_handle()
            .map_or(legacy, |h| legacy.max(h.stop_requests().current()))
    }

    pub async fn set_operator_auth_enforced(&self, enforced: bool) {
        if self.owner_mode() == OWNER_LEGACY {
            let mut inner = self.inner.write().await;
            if self.check_legacy_under_guard(&inner).is_ok() {
                inner.operator_auth_enforced = enforced;
                let snap = crate::rt_core::requests::config::PublishedConfig::from_inner(&inner);
                let gen = self.inner.generation();
                drop(inner);
                self.published_config.publish(snap, gen);
                return;
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        if self.owner_mode() == OWNER_EXECUTOR {
            let _ = self
                .dispatch_config_request_reliable(|| {
                    crate::rt_core::requests::config::ConfigRequest::SetOperatorAuthEnforced(
                        enforced,
                    )
                })
                .await;
        }
        self.published_config
            .update(|cfg| cfg.operator_auth_enforced = enforced);
    }

    /// Snapshot of shadow `TorqueController` diagnostics (lock-free / off-RT; never touches RT owner).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn torque_shadow_diagnostics(&self) -> crate::torque::shadow::TorqueShadowDiagnostics {
        self.torque_shadow.snapshot()
    }

    /// Off-thread bounded torque back-off diagnostics read from [`StateHub`] without taking
    /// `SupervisorLock`.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn torque_backoff_diagnostics(&self) -> crate::torque_guard::TorqueBackoffDiagnostics {
        if let Some(snap) = self.hub().latest() {
            return snap.rt.torque_backoff.to_diagnostics();
        }
        self.inner()
            .try_read()
            .map(|g| g.torque_guard.backoff_history.to_diagnostics())
            .unwrap_or_else(|_| crate::torque_guard::TorqueBackoffHistory::default().to_diagnostics())
    }

    /// Off-thread `/api/v1/diagnostics/power` payload that reads live published RT telemetry and
    /// `torque_backoff` history from [`StateHub`] when executor-owned (never reading the detached
    /// Legacy shell), and reads `SupervisorInner::power_diagnostics()` under Legacy ownership.
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn power_diagnostics(&self) -> serde_json::Value {
        if self.owner_mode() == OWNER_EXECUTOR {
            let budget = self
                .published_config_hub()
                .latest()
                .and_then(|c| c.config.power_budget.clone())
                .filter(|b| b.enabled);
            let Some(budget) = budget else {
                return serde_json::json!({"enabled": false});
            };
            let snap = self.hub().latest();
            let obs_power = snap.as_ref().and_then(|s| s.obs.power.clone());
            let power_diag = snap
                .as_ref()
                .map(|s| s.rt.power_diag)
                .unwrap_or_default();
            let torque_backoff = snap
                .as_ref()
                .map(|s| s.rt.torque_backoff.to_diagnostics())
                .unwrap_or_else(|| {
                    crate::torque_guard::TorqueBackoffHistory::default().to_diagnostics()
                });
            return power_diag.to_json(&budget, obs_power, torque_backoff);
        }
        self.inner().read().await.power_diagnostics()
    }

    /// Shared handle to the off-thread shadow comparator/diagnostics hub.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn torque_shadow_hub(&self) -> &Arc<crate::torque::shadow::TorqueShadowHub> {
        &self.torque_shadow
    }

    /// Lock-free atomic overflow/summary handle for primary `bb.jsonl` piggyback summaries.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn torque_shadow_overflow_atomics(
        &self,
    ) -> Arc<crate::torque::shadow::ShadowOverflowAtomics> {
        self.torque_shadow.overflow_atomics().clone()
    }

    /// Exposes the lock-free tick ring for deterministic paused-reader contention tests.
    #[doc(hidden)]
    pub fn tick_stats_ring_for_test(&self) -> &Arc<crate::diagnostics::tick::LockFreeTickStats> {
        &self.tick_stats
    }

    /// The raw samples of the last ticks, oldest first (tests, diagnostics).
    pub fn tick_samples(&self) -> Vec<crate::diagnostics::tick::TickSample> {
        self.drain_diag();
        self.tick_stats.window().samples().to_vec()
    }

    /// Requests a coherent control-thread baseline marker (`POST /api/v1/diagnostics/tick/mark`)
    /// on the currently active periodic owner. Returns 503 if no control loop is actively running.
    pub fn mark_tick_baseline(&self) -> Result<u64, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let Some(h) = self.executor_handle() else {
                    return Err(executor_unavailable());
                };
                if h.is_exited() || !h.stats().is_loop_active() {
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "EXECUTOR_STOPPED".into(),
                        message: "RT executor control loop is not active".into(),
                    });
                }
                Ok(h.stats().request_marker())
            }
            OWNER_LEGACY => {
                let stats = self
                    .legacy_loop_stats
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .clone();
                let Some(s) = stats else {
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "CONTROL_LOOP_INACTIVE".into(),
                        message: "Legacy periodic control loop is not active".into(),
                    });
                };
                if !s.is_loop_active() {
                    return Err(ArmError::Rejected {
                        status: 503,
                        code: "CONTROL_LOOP_INACTIVE".into(),
                        message: "Legacy periodic control loop is not active".into(),
                    });
                }
                Ok(s.request_marker())
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Timing of the last ticks (`GET /api/v1/diagnostics/tick`). Reads the lock-free ring
    /// without ever blocking the control thread.
    pub fn tick_stats(&self) -> crate::diagnostics::tick::TickSnapshot {
        const MAX_ALIGN_ATTEMPTS: usize = 4;
        let executor_stats = self.executor_handle().map(|h| Arc::clone(h.stats()));
        let is_executor = executor_stats.is_some();
        let active_stats = if let Some(s) = executor_stats {
            Some(s)
        } else {
            self.legacy_loop_stats
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
        };

        let mut snap = if let Some(ref s) = active_stats {
            // Bounded alignment loop: read monotonic snapshot FIRST (Acquire on `stats_seq`),
            // drain off-thread diagnostics (guaranteed to include all samples & amendments
            // published before `mono1`), copy the lock-free window (fast memcpy, no sort),
            // and re-read monotonic snapshot to confirm no release crossed the capture window.
            let mut chosen: Option<(
                crate::diagnostics::tick::TickWindow,
                crate::rt_core::executor::MonotonicStatsSnapshot,
            )> = None;
            let mut last_attempt = None;
            for _ in 0..MAX_ALIGN_ATTEMPTS {
                let mono1 = s.snapshot_monotonic();
                self.drain_diag();
                let win = self.tick_stats.window();
                let mono2 = s.snapshot_monotonic();
                let aligned = mono1.coherent
                    && mono2.coherent
                    && !win.is_lapped()
                    && mono1.releases_started == mono2.releases_started
                    && mono1.releases_total == mono2.releases_total
                    && mono1.skipped_releases == mono2.skipped_releases
                    && mono1.loop_active == mono2.loop_active;
                if aligned {
                    chosen = Some((win, mono2));
                    break;
                }
                last_attempt = Some((win, mono2));
            }
            let (win, mono, align_lapped) = match chosen {
                Some((win, mono)) => (win, mono, false),
                None => {
                    let (win, mono) = last_attempt.expect("MAX_ALIGN_ATTEMPTS > 0");
                    (win, mono, true)
                }
            };
            let mut snap = win.snapshot();
            snap.stats_reader_retries = self.tick_stats.reader_retries();
            snap.stats_writer_drops = self
                .tick_stats
                .writer_contention_drops()
                .saturating_add(s.amend_missed.load(Ordering::Relaxed));
            if !mono.coherent || align_lapped {
                snap.stats_snapshot_lapped = true;
            }
            snap.loop_active = mono.loop_active;
            snap.releases_started = mono.releases_started;
            snap.releases_total = mono.releases_total;
            snap.in_flight_release_us = mono.in_flight_release_us;
            snap.release_us_max_ever = mono.release_us_max_ever;
            snap.releases_over_10ms = mono.releases_over_10ms;
            snap.releases_over_15ms = mono.releases_over_15ms;
            snap.releases_over_20ms = mono.releases_over_20ms;
            snap.wake_late_max_ever = mono.wake_late_max_ever;
            snap.skipped_releases = mono.skipped_releases;
            snap.diag_dropped = mono.diag_dropped;
            snap.wake_lateness_over_1ms = mono.wake_lateness_over_1ms;
            snap.wake_lateness_over_5ms = mono.wake_lateness_over_5ms;
            snap.wake_lateness_over_10ms = mono.wake_lateness_over_10ms;
            snap.since_marker = mono.since_marker;
            snap
        } else {
            self.drain_diag();
            self.tick_stats.snapshot()
        };
        snap.instance_id = self.instance_id.clone();
        let ident = init_process_identity();
        snap.boot_id = ident.boot_id.clone();
        snap.process_start_unix_ns = ident.process_start_unix_ns;
        snap.owner_mode = self.owner_mode_name();
        snap.effective_owner = self.effective_owner().as_str();
        snap.owner_reason = self.owner_reason().as_str();
        snap.control_thread = self.control_sched.lock().unwrap_or_else(|e| e.into_inner()).clone();
        snap.observation_cache = crate::diagnostics::tick::ObservationCacheStats {
            hits: self.obs_cache_hits.load(Ordering::Relaxed),
            misses: self.obs_cache_misses.load(Ordering::Relaxed),
            publish_skipped: 0,
            waited: self.obs_waits.load(Ordering::Relaxed),
            unavailable: self.obs_unavailable.load(Ordering::Relaxed),
            published: self.hub.published_seq(),
            tick_frees: self.hub.tick_frees(),
        };
        snap.terminal_capture = self.terminal_capture_channel.stats();
        snap.safety_log_dropped = self
            .sinks
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map_or_else(
                || self.inner.try_read().map_or(0, |g| g.safety.log_dropped().saturating_add(g.safety_log_ring_dropped)),
                |s| s.safety_log_dropped(),
            );
        snap.log_writer_dropped = crate::diagnostics::logwriter::total_dropped();
        if is_executor {
            snap.control_thread_lock_attempts = Some(
                self.inner
                    .control_thread_attempts()
                    .saturating_add(self.driver_lock_attempts.load(Ordering::Acquire))
                    .saturating_add(self.terminal_capture_channel.control_thread_lock_attempts()),
            );
        }
        if let Some(ref s) = active_stats {
            if s.alloc_timing_enabled.load(Ordering::Acquire) {
                snap.alloc_timing_enabled = Some(true);
                if is_executor {
                    snap.control_thread_alloc_ops = Some(s.total_allocs().total());
                }
            }
            if is_executor {
                snap.executor_wakes = Some(s.completed_wakes.load(Ordering::Acquire));
            }
        }
        snap
    }

    /// Thread-CPU-time percentiles of lock-held tick execution over the current tick window
    /// (`CLOCK_THREAD_CPUTIME_ID` on Linux, excluding off-CPU OS preemption under host load).
    #[doc(hidden)]
    pub fn tick_lock_cpu_us(&self) -> crate::diagnostics::tick::Percentiles {
        self.drain_diag();
        self.tick_stats.window().lock_cpu_us()
    }

    pub async fn set_mock_gripper_stall_offset(&self, offset: Option<f32>) {
        let mut inner = self.inner.write().await;
        if self.check_legacy_under_guard(&inner).is_ok() {
            inner.mock_gripper_stall_offset = offset;
        }
    }

    pub async fn gripper_calibration_response(&self) -> crate::gripper_calib::GripperCalibrationResponse {
        self.inner.read().await.gripper_calibration_response()
    }
}
