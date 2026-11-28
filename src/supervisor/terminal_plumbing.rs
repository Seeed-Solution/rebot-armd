//! Terminal capture plumbing, `TerminalBroadcaster`, telemetry dispatcher, and terminal test hooks.

#[allow(unused_imports)]
use super::*;

impl Default for TerminalBroadcaster {
    fn default() -> Self {
        Self {
            subscriber_count: AtomicUsize::new(0),
            pending_total: AtomicUsize::new(0),
            max_fifo_len: AtomicUsize::new(0),
            capacity_atomic: AtomicUsize::new(TERMINAL_FIFO_CAPACITY),
            dispatch_paused: AtomicBool::new(false),
            inner: std::sync::Mutex::new(TerminalBroadcasterInner {
                next_id: 1,
                capacity: TERMINAL_FIFO_CAPACITY,
                sinks: Vec::new(),
                recent_finished: std::collections::VecDeque::with_capacity(64),
            }),
        }
    }
}

impl TerminalBroadcaster {
    fn recompute_atomics(&self, g: &TerminalBroadcasterInner) {
        self.subscriber_count.store(g.sinks.len(), Ordering::Release);
        let mut pending = 0usize;
        let mut max_len = 0usize;
        for q in &g.sinks {
            let l = q.fifo.len();
            pending += l;
            if l > max_len {
                max_len = l;
            }
        }
        self.pending_total.store(pending, Ordering::Release);
        self.max_fifo_len.store(max_len, Ordering::Release);
        self.capacity_atomic.store(g.capacity, Ordering::Release);
    }

    #[cfg(test)]
    pub(crate) fn subscribe(&self, sink: TerminalSinkFn) -> u64 {
        self.subscribe_with_cutoff(sink, 0)
    }

    pub(crate) fn subscribe_with_cutoff(&self, sink: TerminalSinkFn, min_terminal_seq: u64) -> u64 {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let id = g.next_id;
        g.next_id += 1;
        let cap = g.capacity;
        g.sinks.push(ConnectionTerminalQueue {
            sub_id: id,
            min_terminal_seq,
            fifo: std::collections::VecDeque::with_capacity(cap.min(TERMINAL_FIFO_CAPACITY)),
            seen: std::collections::VecDeque::with_capacity(cap.max(128)),
            sink,
        });
        self.recompute_atomics(&g);
        id
    }

    /// Atomically reads `read_cutoff()` and installs `min_terminal_seq` under `self.inner.lock()`
    /// so no concurrent enqueue can observe the temporary `u64::MAX` filter after the cutoff is read.
    pub(crate) fn activate_subscriber_cutoff(
        &self,
        sub_id: u64,
        read_cutoff: impl FnOnce() -> u64,
    ) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let cutoff = read_cutoff();
        if let Some(q) = g.sinks.iter_mut().find(|q| q.sub_id == sub_id) {
            q.min_terminal_seq = cutoff;
        }
    }

    pub(crate) fn unsubscribe(&self, id: u64) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.sinks.retain(|q| q.sub_id != id);
        self.recompute_atomics(&g);
    }

    #[cfg(test)]
    pub(crate) fn can_start_action(&self, has_running_action: bool, lock_free: bool) -> bool {
        self.can_start_action_with_inflight(
            has_running_action,
            lock_free,
            0,
            0,
            TERMINAL_FIFO_CAPACITY,
        )
    }

    /// End-to-end admission check charging `staged + inflight + fifo + needed` against both the
    /// per-connection FIFO capacity and the capture ring's admission cap.
    ///
    /// Caller reads `inflight` with `Acquire` strictly BEFORE this method reads `max_fifo_len`
    /// with `Acquire`, matching the inverse write order in `TerminalCaptureChannel::drain_into`
    /// (`max_fifo_len` Release -> `inflight` Release) so handoffs never under-count.
    pub(crate) fn can_start_action_with_inflight(
        &self,
        has_running_action: bool,
        lock_free: bool,
        staged: usize,
        inflight: usize,
        ring_admission_cap: usize,
    ) -> bool {
        if self.subscriber_count.load(Ordering::Acquire) == 0 {
            return true;
        }
        // Reserve 1 slot for the new action's eventual terminal, plus 1 immediate slot if a
        // currently running action will be superseded right now.
        let needed = if has_running_action { 2 } else { 1 };
        let in_pipeline = staged.saturating_add(inflight);
        if in_pipeline.saturating_add(needed) > ring_admission_cap {
            return false;
        }
        if lock_free {
            let cap = self.capacity_atomic.load(Ordering::Acquire);
            let max_len = self.max_fifo_len.load(Ordering::Acquire);
            return max_len.saturating_add(in_pipeline).saturating_add(needed) <= cap;
        }
        let g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if g.sinks.is_empty() {
            return true;
        }
        g.sinks
            .iter()
            .all(|q| q.fifo.len().saturating_add(in_pipeline).saturating_add(needed) <= g.capacity)
    }

    #[cfg(test)]
    pub(crate) fn record_finished(&self, action: &ActionStatus) -> bool {
        self.record_finished_with_seq(0, action.clone(), None, None)
    }

    /// Enqueues a terminal action with authoritative `terminal_seq` (0 when unassigned in direct
    /// unit tests) into every subscriber whose `min_terminal_seq < terminal_seq` (or 0) and that
    /// has not already seen `action.id`. Carries `required_publish_seq` through to delivery so
    /// `drain_all` and owner reply sinks never pair a terminal with a snapshot older than its
    /// required publication. Updates `max_fifo_len` and `pending_total` with `Release`.
    pub(crate) fn record_finished_with_seq(
        &self,
        terminal_seq: u64,
        action: ActionStatus,
        obs_override: Option<ArmObservation>,
        required_publish_seq: Option<u64>,
    ) -> bool {
        if action.is_running() {
            return false;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let queued = QueuedTerminal {
            action: action.clone(),
            obs_override: obs_override.clone(),
            required_publish_seq,
        };
        if let Some(existing) = g.recent_finished.iter_mut().find(|e| e.action.id == action.id) {
            *existing = queued.clone();
        } else {
            if g.recent_finished.len() >= 64 {
                g.recent_finished.pop_front();
            }
            g.recent_finished.push_back(queued.clone());
        }
        if g.sinks.is_empty() {
            return false;
        }
        let cap = g.capacity;
        let mut enqueued = false;
        for q in &mut g.sinks {
            if terminal_seq != 0 && terminal_seq <= q.min_terminal_seq {
                continue;
            }
            if q.seen.contains(&action.id) {
                continue;
            }
            // Never drop or skip a finished action's terminal even if `q.fifo.len() >= cap`:
            // `can_start_action_with_inflight` refuses NEW starts with `TERMINALS_BUSY` once
            // `len >= cap`, while any already-running action's terminal is preserved in strict FIFO order.
            let seen_cap = cap.max(128).max(q.fifo.len() + 1);
            if q.seen.len() >= seen_cap {
                q.seen.pop_front();
            }
            q.seen.push_back(action.id);
            q.fifo.push_back(queued.clone());
            enqueued = true;
        }
        if enqueued {
            self.recompute_atomics(&g);
        }
        enqueued
    }

    pub(crate) fn lookup_finished_entry(&self, action_id: u64) -> Option<QueuedTerminal> {
        let g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.recent_finished
            .iter()
            .rev()
            .find(|e| e.action.id == action_id)
            .cloned()
    }

    pub(crate) fn update_failed_step(&self, action_id: u64, step: Option<usize>) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(entry) = g.recent_finished.iter_mut().find(|e| e.action.id == action_id) {
            entry.action.failed_step = step;
            if let Some(obs) = entry.obs_override.as_mut() {
                if let Some(a) = obs.action.as_mut().filter(|a| a.id == action_id) {
                    a.failed_step = step;
                }
            }
        }
        for q in &mut g.sinks {
            if let Some(entry) = q.fifo.iter_mut().find(|e| e.action.id == action_id) {
                entry.action.failed_step = step;
                if let Some(obs) = entry.obs_override.as_mut() {
                    if let Some(a) = obs.action.as_mut().filter(|a| a.id == action_id) {
                        a.failed_step = step;
                    }
                }
            }
        }
    }

    pub(crate) fn has_pending(&self) -> bool {
        !self.dispatch_paused.load(Ordering::Acquire)
            && self.pending_total.load(Ordering::Acquire) > 0
    }

    /// Drains ready queued terminals onto subscriber sinks, enforcing `required_publish_seq <= base_seq`
    /// whenever an entry relies on `base_obs`. Returns `true` if any subscriber FIFO stopped at a
    /// head entry waiting for a newer snapshot (`base_seq < required_publish_seq`).
    pub(crate) fn drain_all(&self, base_obs: &ArmObservation, base_seq: u64) -> bool {
        if self.dispatch_paused.load(Ordering::Acquire)
            || self.pending_total.load(Ordering::Acquire) == 0
        {
            return false;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut blocked_on_seq = false;
        // Pop a terminal ONLY after its sink confirms a successful enqueue. On failure
        // (e.g., outbound queue full -> fatal close triggered) or when `base_seq < required_publish_seq`,
        // keep the terminal at the head of the FIFO to preserve strict FIFO order and snapshot parity.
        for q in &mut g.sinks {
            while let Some(entry) = q.fifo.front() {
                if let Some(req_seq) = entry.required_publish_seq {
                    if entry.obs_override.is_none() && base_seq < req_seq {
                        blocked_on_seq = true;
                        break;
                    }
                }
                let mut obs = match entry.obs_override.as_ref() {
                    Some(captured) => captured.clone(),
                    None => base_obs.clone(),
                };
                obs.action = Some(entry.action.clone());
                obs.previous_action = None;
                if (q.sink)(&obs) {
                    q.fifo.pop_front();
                } else {
                    break;
                }
            }
        }
        self.recompute_atomics(&g);
        blocked_on_seq
    }

    /// Drains queued observer terminals with `action.id < max_exclusive_id` (enforcing
    /// `required_publish_seq <= base_seq` whenever an entry relies on `base_obs`) so an
    /// owner-routed terminal for `max_exclusive_id` never overtakes an earlier observer terminal
    /// on the same socket. Returns `true` when no earlier observer terminal remains blocked.
    pub(crate) fn drain_before_action_id(
        &self,
        base_obs: &ArmObservation,
        base_seq: u64,
        max_exclusive_id: u64,
    ) -> bool {
        if self.dispatch_paused.load(Ordering::Acquire)
            || self.pending_total.load(Ordering::Acquire) == 0
        {
            return true;
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut all_earlier_flushed = true;
        for q in &mut g.sinks {
            while let Some(entry) = q.fifo.front() {
                if entry.action.id >= max_exclusive_id {
                    break;
                }
                if let Some(req_seq) = entry.required_publish_seq {
                    if entry.obs_override.is_none() && base_seq < req_seq {
                        all_earlier_flushed = false;
                        break;
                    }
                }
                let mut obs = match entry.obs_override.as_ref() {
                    Some(captured) => captured.clone(),
                    None => base_obs.clone(),
                };
                obs.action = Some(entry.action.clone());
                obs.previous_action = None;
                if (q.sink)(&obs) {
                    q.fifo.pop_front();
                } else {
                    all_earlier_flushed = false;
                    break;
                }
            }
        }
        self.recompute_atomics(&g);
        all_earlier_flushed
    }

    pub(crate) fn set_capacity(&self, capacity: usize) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.capacity = capacity.max(1);
        self.recompute_atomics(&g);
    }

    pub(crate) fn set_dispatch_paused(&self, paused: bool) {
        self.dispatch_paused.store(paused, Ordering::Release);
    }

    pub(crate) fn pending_total(&self) -> usize {
        self.pending_total.load(Ordering::Acquire)
    }

    pub(crate) fn lock_for_test(&self) -> impl Drop + '_ {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn try_lock_for_test(&self) -> bool {
        self.inner.try_lock().is_ok()
    }
}

impl Drop for TerminalSubscription {
    fn drop(&mut self) {
        self.broadcaster.unsubscribe(self.id);
    }
}

impl SupervisorInner {
    /// Ensures every subscribed connection's bounded terminal FIFO (plus staged and in-flight
    /// SPSC captures) has room for any action superseded by this start plus the new action's
    /// eventual terminal (never drops a terminal).
    pub(crate) fn check_terminal_fifo_for_new_action(&self) -> Result<(), SafetyError> {
        let Some(terminals) = self.terminals.as_ref() else {
            return Ok(());
        };
        let has_running = self.last_action.as_ref().is_some_and(|a| a.is_running());
        let staged = self.staged_terminals.len();
        let (inflight, ring_admission_cap) = match self.terminal_capture_channel.as_ref() {
            Some(ch) => (ch.inflight(), ch.admission_cap()),
            None => (0, TERMINAL_FIFO_CAPACITY),
        };
        if !terminals.can_start_action_with_inflight(
            has_running,
            self.rt_executor_owned || self.defer_terminal_capture,
            staged,
            inflight,
            ring_admission_cap,
        ) {
            return Err(SafetyError::TerminalsBusy);
        }
        Ok(())
    }

    /// Lock-free read of the attached driver's clock without ever acquiring `self.driver.lock()`.
    #[inline]
    pub(crate) fn cached_driver_clock_now(&self) -> Instant {
        self.driver_clock.as_ref().map_or_else(Instant::now, |c| c.now())
    }

    /// Per-tick cached hardware enable state for terminal capture (zero driver mutex acquisitions,
    /// isolated from hardware interlock `last_poll_enable_state`).
    #[inline]
    pub(crate) fn cached_enable_state(&self) -> EnableState {
        if self.driver.is_some() {
            self.capture_driver_cache.enable_state
        } else if self.is_torqued {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        }
    }

    /// Per-tick cached driver capabilities for terminal capture (zero driver mutex acquisitions).
    #[inline]
    pub(crate) fn cached_capture_driver_caps(&self) -> DriverCapabilities {
        if self.driver.is_some() {
            self.capture_driver_cache.driver_caps
        } else {
            self.driver_caps
        }
    }

    /// Per-tick cached hardware feedback freshness within `max_age` (zero driver mutex acquisitions).
    /// Derives freshness from the exact same per-joint age computation used for `feedback_age_ms`
    /// in `TerminalCaptureRecord`, never mutating `ArmCore::feedback().instants()` or hardware park
    /// interlock fields.
    #[cfg(test)]
    #[inline]
    pub(crate) fn cached_hardware_feedback_fresh(&self, max_age: Duration) -> bool {
        let dof = self.config.dof.min(terminal_capture::CAPTURE_MAX_DOF);
        terminal_capture::evaluate_cached_feedback_fresh(
            self.feedback_numeric_valid,
            self.capture_driver_cache.feedback_valid,
            self.driver.is_some(),
            dof,
            self.core.feedback().instants(),
            &self.capture_driver_cache.ages[..usize::from(self.capture_driver_cache.ages_len)],
            self.capture_driver_cache.feedback_at,
            self.cached_driver_clock_now(),
            max_age,
        )
    }

    /// Returns a cloned handle to the shared off-thread `TerminalCaptureChannel` (wired by
    /// `ArmSupervisor::new`, `None` on standalone `SupervisorInner::new`).
    pub(crate) fn terminal_capture_channel(
        &self,
    ) -> Option<Arc<terminal_capture::TerminalCaptureChannel>> {
        self.terminal_capture_channel.clone()
    }

    /// Returns cloned off-thread handles required for normal executor teardown to drain the
    /// authoritative `TerminalCaptureChannel` into `TerminalBroadcaster` before final broker
    /// reply delivery and route retirement (zero RT locks/allocs; cloned once at spawn).
    pub(crate) fn terminal_capture_drain_handles(
        &self,
    ) -> Option<(
        Arc<terminal_capture::TerminalCaptureChannel>,
        Arc<TerminalBroadcaster>,
        Arc<ArmObservation>,
    )> {
        Some((
            self.terminal_capture_channel.clone()?,
            self.terminals.clone()?,
            self.template_obs_for_drain.clone()?,
        ))
    }

    /// Shared record-construction entry point wrapping `TerminalCaptureRecord::capture_from` so
    /// early capture paths (immediate supersession, pre-overwrite materialization, staged
    /// eviction) are audited by `terminal_capture_test_hook` identically to `capture_edge_terminals`.
    #[inline]
    pub(super) fn capture_terminal_record(
        &self,
        action: &ActionStatus,
        terminal_seq: u64,
        obs_timestamp: f64,
        invoke_hook: bool,
    ) -> terminal_capture::TerminalCaptureRecord {
        if invoke_hook {
            if let Some(hook) = self.terminal_capture_test_hook.as_ref() {
                hook(self, true);
            }
        }
        let rec = terminal_capture::TerminalCaptureRecord::capture_from(
            self,
            action,
            terminal_seq,
            obs_timestamp,
        );
        if invoke_hook {
            if let Some(hook) = self.terminal_capture_test_hook.as_ref() {
                hook(self, false);
            }
        }
        rec
    }

    /// Off-RT helper mirroring a rich macro `ActionStatus` into `TerminalCaptureChannel`'s
    /// bounded sidecar cache so delayed dispatch past the 2 s window preserves every field.
    /// Never called on `armd-control` (`rt_executor_owned || defer_terminal_capture`).
    pub(crate) fn mirror_rich_action_off_rt(&self, action: &ActionStatus) {
        if self.rt_executor_owned || self.defer_terminal_capture {
            return;
        }
        if let Some(ch) = self.terminal_capture_channel.as_ref() {
            ch.mirror_rich_action(action);
        }
    }

    /// Off-tick Legacy helper: drains any queued SPSC terminal captures into `TerminalBroadcaster`
    /// while `previous_action` and `last_action` are still live so no rich macro payload can be
    /// evicted before preservation. Early-returns without touching `hub` or `rx` when empty.
    pub(crate) fn drain_terminal_captures_with_live_slots(&self) {
        if self.rt_executor_owned || self.defer_terminal_capture {
            return;
        }
        let (Some(ch), Some(terminals)) = (self.terminal_capture_channel.as_ref(), self.terminals.as_ref()) else {
            return;
        };
        if ch.inflight() == 0 {
            return;
        }
        let latest = self.core.hub().latest();
        let live_slots = [
            self.previous_action.as_ref().map(|(a, _)| a),
            self.last_action.as_ref(),
        ];
        if let Some(tmpl) = self.template_obs_for_drain.as_ref() {
            ch.drain_into(
                terminals,
                self.core.hub(),
                latest.as_deref(),
                &live_slots,
                tmpl,
                false,
            );
        } else {
            let fallback = ArmObservation::blank();
            ch.drain_into(
                terminals,
                self.core.hub(),
                latest.as_deref(),
                &live_slots,
                &fallback,
                false,
            );
        }
    }

    /// Allocates the next monotonic `terminal_seq` from the shared channel (or returns 0 on
    /// standalone `SupervisorInner::new` without an `ArmSupervisor`).
    #[inline]
    pub(super) fn alloc_terminal_seq(&self) -> u64 {
        self.terminal_capture_channel
            .as_ref()
            .map_or(0, |ch| ch.alloc_terminal_seq())
    }

    #[inline]
    pub(super) fn find_terminal_slot(&self, action_id: u64) -> Option<&ActionStatus> {
        if let Some(a) = self.last_action.as_ref().filter(|a| a.id == action_id && !a.is_running()) {
            return Some(a);
        }
        self.previous_action
            .as_ref()
            .map(|(a, _)| a)
            .filter(|a| a.id == action_id && !a.is_running())
    }

    #[inline]
    pub(super) fn flush_evicted_staged(
        &mut self,
        evicted: terminal_capture::StagedTerminal,
        obs_timestamp: f64,
        publish_seq: Option<u64>,
        charge_timing: bool,
    ) {
        if self.first_terminal_csw_mark.is_none() {
            self.first_terminal_csw_mark = Some(crate::diagnostics::tick::CswMark::now());
        }
        let t0 = charge_timing.then(Instant::now);
        let rec = match evicted {
            terminal_capture::StagedTerminal::Captured(mut r) => {
                r.publish_seq = publish_seq;
                r.obs_timestamp = obs_timestamp;
                Some(r)
            }
            terminal_capture::StagedTerminal::Deferred {
                action_id,
                terminal_seq,
            } => self.find_terminal_slot(action_id).map(|a| {
                let mut r =
                    self.capture_terminal_record(a, terminal_seq, obs_timestamp, true);
                r.publish_seq = publish_seq;
                r
            }),
        };
        if let (Some(r), Some(tx)) = (rec, self.terminal_capture_tx.as_mut()) {
            let _ = tx.try_push(r);
        }
        if let Some(t0) = t0 {
            self.last_phase_terminal_us = self
                .last_phase_terminal_us
                .saturating_add(crate::diagnostics::tick::us(t0.elapsed()));
        }
    }

    /// Materializes a `StagedTerminal::Deferred` entry in-place into `StagedTerminal::Captured`
    /// right before `action_id`'s live slot (`previous_action` / `last_action`) is overwritten so
    /// no deferred terminal can ever be silently skipped at `capture_edge_terminals`.
    pub(crate) fn materialize_deferred_staged_by_id(&mut self, action_id: u64, obs_timestamp: f64) {
        if self.first_terminal_csw_mark.is_none() {
            self.first_terminal_csw_mark = Some(crate::diagnostics::tick::CswMark::now());
        }
        let t0 = Instant::now();
        let Some(seq) = self.staged_terminals.deferred_seq_for(action_id) else {
            return;
        };
        let Some(action) = self.find_terminal_slot(action_id) else {
            return;
        };
        let rec = self.capture_terminal_record(action, seq, obs_timestamp, true);
        self.staged_terminals.replace_deferred_with_captured(action_id, rec);
        self.last_phase_terminal_us = self
            .last_phase_terminal_us
            .saturating_add(crate::diagnostics::tick::us(t0.elapsed()));
    }

    /// Authoritative `running -> !running` transition handler for a superseded `ActionStatus`
    /// (called from `begin_action_inner` when `old` is moving into `previous_action`).
    /// - Off-tick Legacy (`!rt_executor_owned && !defer_terminal_capture`): drains any earlier SPSC
    ///   captures first (preserving strict `terminal_seq` order) and inserts the full `ActionStatus`
    ///   directly into `TerminalBroadcaster`.
    /// - In-tick Legacy or RT Executor (`defer_terminal_capture || rt_executor_owned`): captures a
    ///   fixed-size `Copy` `TerminalCaptureRecord` into inline `staged_terminals` (0 alloc, 0 lock)
    ///   and accounts its construction time in `last_phase_terminal_us`.
    pub(crate) fn record_terminal_transition_for(&mut self, action: &ActionStatus) {
        if action.is_running() || action.id == 0 {
            return;
        }
        if self.terminal_edge_tracker.has_seen_action(action.id) {
            return;
        }
        if self.rt_executor_owned || self.defer_terminal_capture {
            if self.first_terminal_csw_mark.is_none() {
                self.first_terminal_csw_mark = Some(crate::diagnostics::tick::CswMark::now());
            }
            let t0 = Instant::now();
            let seq = self.alloc_terminal_seq();
            if !self.terminal_edge_tracker.record_edge(action.id, seq) {
                return;
            }
            let ts = self.now_s();
            let rec = self.capture_terminal_record(action, seq, ts, true);
            if let Some(oldest) = self
                .staged_terminals
                .push_evicting_oldest(terminal_capture::StagedTerminal::Captured(rec))
            {
                let next_pub_seq = Some(self.core.next_publish_seq());
                self.flush_evicted_staged(oldest, ts, next_pub_seq, false);
            }
            self.last_phase_terminal_us = self
                .last_phase_terminal_us
                .saturating_add(crate::diagnostics::tick::us(t0.elapsed()));
            return;
        }
        self.drain_terminal_captures_with_live_slots();
        let seq = self.alloc_terminal_seq();
        if !self.terminal_edge_tracker.record_edge(action.id, seq) {
            return;
        }
        if let Some(terminals) = self.terminals.as_ref() {
            if terminals.record_finished_with_seq(seq, action.clone(), None, None) {
                self.core.hub().wake();
            }
        }
    }

    /// Convenience wrapper: emits the authoritative terminal transition when `self.last_action`
    /// matches `id` and is terminal (`!is_running()`).
    /// On RT/guarded ticks, stages a lightweight `Deferred { action_id, terminal_seq }` edge so
    /// `TerminalCaptureRecord::capture_from` runs inside `capture_edge_terminals` (covered by
    /// `phases_us.terminal`, `terminal_capture_test_hook`, and capture alloc/lock proofs).
    pub(crate) fn record_terminal_transition_by_id(&mut self, id: u64) {
        if let Some(a) = self.last_action.as_ref().filter(|a| a.id == id && !a.is_running()) {
            if self.terminal_edge_tracker.has_seen_action(a.id) {
                return;
            }
            if self.rt_executor_owned || self.defer_terminal_capture {
                let action_id = a.id;
                let seq = self.alloc_terminal_seq();
                let _ = self.terminal_edge_tracker.record_edge(action_id, seq);
                #[cfg(not(target_arch = "wasm32"))]
                self.core
                    .hub()
                    .record_action_terminal_cutoff_rt(action_id, self.start_time.elapsed());
                let ts = self.now_s();
                if let Some(oldest) = self.staged_terminals.push_evicting_oldest(
                    terminal_capture::StagedTerminal::Deferred {
                        action_id,
                        terminal_seq: seq,
                    },
                ) {
                    let next_pub_seq = Some(self.core.next_publish_seq());
                    self.flush_evicted_staged(oldest, ts, next_pub_seq, true);
                }
                return;
            }
            // Off-tick Legacy: drain ring with live slots first, then enqueue full ActionStatus.
            self.drain_terminal_captures_with_live_slots();
            let seq = self.alloc_terminal_seq();
            if !self.terminal_edge_tracker.record_edge(id, seq) {
                return;
            }
            if let Some(a) = self.last_action.as_ref().filter(|a| a.id == id && !a.is_running()) {
                if let Some(terminals) = self.terminals.as_ref() {
                    if terminals.record_finished_with_seq(seq, a.clone(), None, None) {
                        self.core.hub().wake();
                    }
                }
            }
        }
    }

    /// Edge-triggered, allocation-free, lock-free capture and flush of staged terminal transitions
    /// (plus once-per-id fallback for direct test struct mutations to `previous_action`/`last_action`)
    /// into the preallocated SPSC ring strictly before `StatePublisher::publish_with_owner`.
    ///
    /// Returns `(total_terminal_phase_us, captured_any)` covering both record construction and
    /// SPSC push.
    pub(crate) fn capture_edge_terminals(
        &mut self,
        obs_timestamp: f64,
        publish_seq: Option<u64>,
    ) -> (u32, bool) {
        let t0 = Instant::now();
        if let Some(hook) = self.terminal_capture_test_hook.as_ref() {
            hook(self, true);
        }
        // Fallback edge check for direct test mutations that bypassed `finish_action` /
        // `begin_action_inner`. Respects the 2 s `previous_action` window and checks
        // `previous_action` before `last_action` for chronological FIFO order.
        if let Some((prev, until)) = self.previous_action.as_ref() {
            if !prev.is_running()
                && obs_timestamp < *until
                && prev.id != 0
                && !self.terminal_edge_tracker.has_seen_edge(prev.id, 0)
            {
                let prev_id = prev.id;
                let seq = self.alloc_terminal_seq();
                let _ = self.terminal_edge_tracker.record_edge(prev_id, seq);
                if let Some(oldest) = self.staged_terminals.push_evicting_oldest(
                    terminal_capture::StagedTerminal::Deferred {
                        action_id: prev_id,
                        terminal_seq: seq,
                    },
                ) {
                    self.flush_evicted_staged(oldest, obs_timestamp, publish_seq, false);
                }
            }
        }
        if let Some(cur) = self.last_action.as_ref() {
            if !cur.is_running()
                && cur.id != 0
                && !self.terminal_edge_tracker.has_seen_edge(cur.id, 0)
            {
                let cur_id = cur.id;
                let seq = self.alloc_terminal_seq();
                let _ = self.terminal_edge_tracker.record_edge(cur_id, seq);
                if let Some(oldest) = self.staged_terminals.push_evicting_oldest(
                    terminal_capture::StagedTerminal::Deferred {
                        action_id: cur_id,
                        terminal_seq: seq,
                    },
                ) {
                    self.flush_evicted_staged(oldest, obs_timestamp, publish_seq, false);
                }
            }
        }

        let mut captured_any = false;
        while let Some(staged) = self.staged_terminals.pop_front() {
            let rec = match staged {
                terminal_capture::StagedTerminal::Captured(mut r) => {
                    r.publish_seq = publish_seq;
                    r.obs_timestamp = obs_timestamp;
                    Some(r)
                }
                terminal_capture::StagedTerminal::Deferred {
                    action_id,
                    terminal_seq,
                } => self.find_terminal_slot(action_id).map(|a| {
                    let mut r = terminal_capture::TerminalCaptureRecord::capture_from(
                        self,
                        a,
                        terminal_seq,
                        obs_timestamp,
                    );
                    r.publish_seq = publish_seq;
                    r
                }),
            };
            if let (Some(r), Some(tx)) = (rec, self.terminal_capture_tx.as_mut()) {
                if tx.try_push(r) {
                    captured_any = true;
                }
            }
        }
        let elapsed_us = crate::diagnostics::tick::us(t0.elapsed());
        if let Some(hook) = self.terminal_capture_test_hook.as_ref() {
            hook(self, false);
        }
        let total_us = self.last_phase_terminal_us.saturating_add(elapsed_us);
        self.last_phase_terminal_us = total_us;
        (total_us, captured_any)
    }

    /// Test-only helper: begins and immediately finishes a zero-alloc RT action inside the
    /// deferred terminal-capture staging buffer so unit/integration tests can stage an edge.
    #[doc(hidden)]
    pub fn test_stage_completed_action_rt(&mut self, kind: &str) -> u64 {
        let prev = self.defer_terminal_capture;
        self.defer_terminal_capture = true;
        let id = self.begin_action_rt(kind);
        self.finish_action(id, "done", None, None, Some(true), None, None);
        self.defer_terminal_capture = prev;
        id
    }

    /// Test-only helper: installs a short zero-heap inline `ActiveTrajectory::quintic` (no
    /// `motion_limits` heap Vec) so Legacy `control_tick` runs the real in-tick trajectory
    /// completion -> `finish_action` -> `capture_from` -> SPSC push path with zero unrelated
    /// trajectory deallocations.
    #[doc(hidden)]
    pub fn test_start_zero_heap_jog_rt(&mut self, delta_j0: f32, duration_s: f64) -> u64 {
        let prev = self.defer_terminal_capture;
        self.defer_terminal_capture = true;
        let id = self.begin_action_rt("jog");
        let start = self.commanded_positions.clone();
        let mut target = start.clone();
        if let Some(q0) = target.first_mut() {
            *q0 += delta_j0;
        }
        let zeros = vec![0.0f32; start.len()];
        let traj = MultiJointQuinticTrajectory::new(&start, &zeros, &target, &zeros, duration_s)
            .expect("zero-heap quintic trajectory");
        self.active_trajectory = Some(ActiveTrajectory::quintic(traj, Some(id)));
        self.target_positions = target;
        let _ = self.safety.set_torque_rt(true);
        let _ = self.safety.mark_motion_started();
        self.is_torqued = true;
        self.defer_terminal_capture = prev;
        id
    }

    /// Consumes and resets the accumulated `phases_us.terminal` duration (µs) and first-capture
    /// CSW baseline for the current release.
    #[inline]
    pub(crate) fn take_phase_terminal_us(&mut self) -> u32 {
        self.first_terminal_csw_mark = None;
        std::mem::replace(&mut self.last_phase_terminal_us, 0)
    }

    /// Consumes the capture-window `CswMark` recorded at the first terminal capture or `publish_at`
    /// of the current release (used by RT Executor to attribute `mid_nvcsw`/`mid_nivcsw`).
    #[inline]
    pub(crate) fn take_mid_csw_start(&mut self) -> Option<crate::diagnostics::tick::CswMark> {
        self.last_mid_csw_start
            .take()
            .or_else(|| self.first_terminal_csw_mark.take())
    }

    /// Consumes and resets the disjoint `StatePublisher::publish_with_rt_meta` duration (µs) for
    /// the current release (excludes `phases_us.terminal`).
    #[inline]
    pub(crate) fn take_publish_only_us(&mut self) -> u32 {
        std::mem::replace(&mut self.last_publish_only_us, 0)
    }

    /// Patches `failed_step` on a queued terminal action if still awaiting dispatcher drain.
    /// Drains any pending SPSC captures first so an in-tick capture is never missed.
    pub(crate) fn update_recorded_terminal_failed_step(&self, id: u64, step: Option<usize>) {
        if self.rt_executor_owned {
            return;
        }
        self.drain_terminal_captures_with_live_slots();
        if let Some(terminals) = self.terminals.as_ref() {
            terminals.update_failed_step(id, step);
        }
    }
}

impl ArmSupervisor {
    /// Every tick's observation, forwarded by the dispatcher thread (started here when it
    /// does not run yet; an error when it cannot start); a lagging receiver skips
    /// observations, a lagging dispatcher forwards only the newest.
    pub fn subscribe_telemetry(&self) -> Result<broadcast::Receiver<ArmObservation>, ArmError> {
        self.ensure_dispatcher()?;
        Ok(self.telemetry_tx.subscribe())
    }

    /// Subscribes a WS connection's lossless ordinary control queue to unseen action-terminal
    /// observations so terminals are never lost to telemetry broadcast lag.
    ///
    /// Uses a race-free 0->1 / N->N+1 registration protocol under `(channel.rx -> terminals.inner)`:
    /// 1. Register the queue with `min_terminal_seq = u64::MAX` (activates admission backpressure
    ///    immediately while ignoring pre-existing records).
    /// 2. Under `channel.rx`, drain ready pre-subscription captures (enforcing the publish barrier
    ///    so unpublished RT captures are never exposed early), then atomically read
    ///    `latest_terminal_seq()` and set `min_terminal_seq` inside `terminals.inner`.
    pub(crate) fn subscribe_action_terminals(
        &self,
        sink: impl Fn(&ArmObservation) -> bool + Send + Sync + 'static,
    ) -> TerminalSubscription {
        let id = self
            .terminals
            .subscribe_with_cutoff(Box::new(sink), u64::MAX);
        let latest = self.hub.latest();
        self.terminal_capture_channel
            .drain_and_activate_subscriber(
                id,
                &self.terminals,
                &self.hub,
                latest.as_deref(),
                &self.template_obs,
            );
        TerminalSubscription {
            broadcaster: Arc::clone(&self.terminals),
            id,
        }
    }

    /// Looks up the enriched finished [`QueuedTerminal`] (including `obs_override` and
    /// `required_publish_seq`) for `action_id` from the bounded off-RT terminal cache populated
    /// by the single authoritative `TerminalCaptureChannel`.
    pub(crate) fn lookup_finished_entry(&self, action_id: u64) -> Option<QueuedTerminal> {
        self.terminals.lookup_finished_entry(action_id)
    }

    /// True if any captured terminal record is still in-flight in `TerminalCaptureChannel`.
    #[inline]
    pub(crate) fn terminal_capture_has_inflight(&self) -> bool {
        self.terminal_capture_channel.inflight() > 0
    }

    /// Flushes any queued observer terminals strictly earlier than `max_exclusive_id` (enforcing
    /// `required_publish_seq <= base_seq`) onto subscribed WS ordinary queues before an
    /// owner-routed terminal for `max_exclusive_id` is enqueued, preserving exact chronological
    /// order across both sources on the same socket.
    pub(crate) fn flush_observer_terminals_before(&self, max_exclusive_id: u64, base_seq: u64) -> bool {
        if !self.terminals.has_pending() {
            return true;
        }
        let base = self
            .hub
            .latest()
            .map(|s| s.obs.clone())
            .unwrap_or_else(|| (*self.template_obs).clone());
        self.terminals
            .drain_before_action_id(&base, base_seq, max_exclusive_id)
    }

    /// True if `action_id` has been emitted via a synthetic post-publish terminal (`obs_override`)
    /// across any consumer. Used by `ws.rs` writer lag-recovery and buffered telemetry send to
    /// prevent sending a stale `running` frame after a synthetic terminal for the same action.
    #[inline]
    pub(crate) fn is_action_synthetically_terminated(&self, action_id: u64) -> bool {
        self.terminal_capture_channel
            .was_synthetically_terminated(action_id)
    }

    /// Starts the dispatcher thread (armd does at startup, before the control loop): it
    /// frees the observations the tick retired, forwards telemetry and drains the
    /// recording ring. Idempotent.
    pub fn start_dispatcher(&self) -> Result<(), ArmError> { self.ensure_dispatcher() }

    /// Starts the dispatcher thread once: a normal-priority thread woken by every
    /// publication that forwards samples to [`Self::subscribe_telemetry`] receivers,
    /// drains the recording ring, and drains executor diagnostics/reclamation sinks.
    pub(crate) fn ensure_dispatcher(&self) -> Result<(), ArmError> {
        if self.hub.has_dispatcher() { return Ok(()); }
        let _start = self.dispatcher_start.lock().unwrap_or_else(|e| e.into_inner());
        if self.hub.has_dispatcher() { return Ok(()); }
        let (hub, tx, terminals, terminal_capture, sinks, tick_stats, template_obs) = (
            Arc::clone(&self.hub),
            self.telemetry_tx.clone(),
            Arc::clone(&self.terminals),
            Arc::clone(&self.terminal_capture_channel),
            Arc::clone(&self.sinks),
            Arc::clone(&self.tick_stats),
            self.template_obs.clone(),
        );
        let handle = std::thread::Builder::new()
            .name("armd-dispatch".into())
            .spawn(move || {
                dispatch_loop(
                    &hub,
                    &tx,
                    &terminals,
                    &terminal_capture,
                    &sinks,
                    &tick_stats,
                    &template_obs,
                )
            })
            .map_err(|e| ArmError::Internal(format!("cannot start the observation dispatcher thread: {e}")))?;
        self.hub.set_dispatcher(handle.thread().clone());
        #[cfg(not(target_arch = "wasm32"))]
        if let Err(e) = self.torque_shadow.start_worker() {
            tracing::warn!("Failed to start torque shadow worker thread: {e}");
        }
        Ok(())
    }

    /// Test-only hook: configure per-connection bounded terminal FIFO and capture admission capacity.
    #[doc(hidden)]
    pub fn test_set_terminal_fifo_capacity(&self, capacity: usize) {
        self.terminals.set_capacity(capacity);
        self.terminal_capture_channel.set_admission_cap(capacity);
    }

    /// Test-only hook: pause or resume draining of the SPSC terminal capture ring.
    #[doc(hidden)]
    pub fn test_set_terminal_capture_drain_paused(&self, paused: bool) {
        self.terminal_capture_channel.set_drain_paused(paused);
        if !paused {
            self.hub.wake();
        }
    }

    /// Test-only hook: configure effective push capacity of the SPSC terminal capture ring
    /// (for deterministic ring-saturation tests).
    #[doc(hidden)]
    pub fn test_set_terminal_capture_push_cap(&self, cap: usize) {
        self.terminal_capture_channel.set_test_push_cap(cap);
    }

    /// Test-only hook: synchronously drain ready SPSC terminal capture records into
    /// `TerminalBroadcaster` and flush subscriber queues (for deterministic tests without waiting
    /// on dispatcher wake jitter).
    #[doc(hidden)]
    pub fn test_drain_terminal_captures(&self) -> usize {
        let latest = self.hub.latest();
        let drained = self.terminal_capture_channel.drain_into(
            &self.terminals,
            &self.hub,
            latest.as_deref(),
            &[],
            &self.template_obs,
            true,
        );
        if self.terminals.has_pending() {
            let base = latest
                .as_ref()
                .map(|s| &s.obs)
                .unwrap_or(&self.template_obs);
            let base_seq = latest.as_ref().map_or(0, |s| s.meta.seq);
            let _ = self.terminals.drain_all(base, base_seq);
        }
        drained
    }

    /// Test-only accessor for lock-free terminal capture statistics.
    #[doc(hidden)]
    pub fn test_terminal_capture_stats(&self) -> crate::diagnostics::tick::TerminalCaptureStats {
        self.terminal_capture_channel.stats()
    }

    /// Test-only accessor for forbidden `TerminalCaptureChannel` (`rx` / `rich_action_cache`)
    /// lock attempts detected on the marked RT control thread.
    #[doc(hidden)]
    pub fn test_terminal_capture_control_lock_attempts(&self) -> u64 {
        self.terminal_capture_channel.control_thread_lock_attempts()
    }

    /// Test-only hook: hold `TerminalCaptureChannel::rx` mutex (for executor lock proofs).
    #[doc(hidden)]
    pub fn test_lock_terminal_capture_rx(&self) -> impl Drop + '_ {
        self.terminal_capture_channel.lock_rx_for_test()
    }

    /// Test-only hook: hold `TerminalCaptureChannel::rich_action_cache` mutex (for executor lock proofs).
    #[doc(hidden)]
    pub fn test_lock_terminal_capture_rich_cache(&self) -> impl Drop + '_ {
        self.terminal_capture_channel.lock_rich_cache_for_test()
    }

    /// Test-only positive control: attempt `TerminalCaptureChannel::rx` lock on the current thread.
    #[doc(hidden)]
    pub fn test_touch_terminal_capture_rx_lock(&self) {
        self.terminal_capture_channel.touch_rx_lock_for_test();
    }

    /// Test-only positive control: attempt `TerminalCaptureChannel::rich_action_cache` lock on the current thread.
    #[doc(hidden)]
    pub fn test_touch_terminal_capture_rich_cache_lock(&self) {
        self.terminal_capture_channel.touch_rich_cache_lock_for_test();
    }

    /// Test-only hook: hold `TerminalBroadcaster`'s internal mutex (for deterministic lock proofs).
    #[doc(hidden)]
    pub fn test_lock_terminal_broadcaster(&self) -> impl Drop + '_ {
        self.terminals.lock_for_test()
    }

    /// Test-only hook: non-blocking `try_lock` on `TerminalBroadcaster`'s internal mutex
    /// (positive control for lock proofs).
    #[doc(hidden)]
    pub fn test_try_lock_terminal_broadcaster(&self) -> bool {
        self.terminals.try_lock_for_test()
    }

    /// Test-only hook: pause or resume draining of per-connection terminal FIFOs in `armd-dispatch`.
    #[doc(hidden)]
    pub fn test_set_terminal_dispatch_paused(&self, paused: bool) {
        self.terminals.set_dispatch_paused(paused);
        if !paused {
            self.hub.wake();
        }
    }

    /// Test-only accessor for total pending finished actions across subscribed terminal FIFOs.
    /// Drains any ready SPSC terminal captures into `TerminalBroadcaster` first (respecting
    /// `drain_paused`, without flushing subscriber sinks) so immediate post-tick checks observe
    /// deterministic FIFO occupancy.
    #[doc(hidden)]
    pub fn test_pending_terminal_count(&self) -> usize {
        let latest = self.hub.latest();
        let _ = self.terminal_capture_channel.drain_into(
            &self.terminals,
            &self.hub,
            latest.as_deref(),
            &[],
            &self.template_obs,
            true,
        );
        self.terminals.pending_total()
    }

    /// Test-only accessor for subscribed WS terminal FIFO count.
    #[doc(hidden)]
    pub fn test_terminal_subscriber_count(&self) -> usize {
        self.terminals.subscriber_count.load(Ordering::Acquire)
    }

    /// Test-only hook: subscribe to the lossless `TerminalBroadcaster` terminal stream.
    #[doc(hidden)]
    pub fn test_subscribe_action_terminals(
        &self,
        sink: impl Fn(&ArmObservation) -> bool + Send + Sync + 'static,
    ) -> TerminalSubscription {
        self.subscribe_action_terminals(sink)
    }
}
