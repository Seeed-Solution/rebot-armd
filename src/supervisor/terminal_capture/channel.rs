//! SPSC terminal capture ring, staging buffers, edge tracker, and off-RT drain channel.

#[allow(unused_imports)]
use super::*;

/// Inline staged entry for one release before `capture_edge_terminals` runs:
/// - `Deferred`: ordinary in-tick `finish_action` / `config.rs` transition whose `ActionStatus`
///   remains in `last_action` (or `previous_action`), so `TerminalCaptureRecord::capture_from`
///   runs inside `capture_edge_terminals` (timed in `phases_us.terminal` and covered by capture
///   alloc/lock proofs).
/// - `Captured`: inline record captured immediately when an in-tick superseding `begin_action_inner`
///   is about to overwrite the old slot before `capture_edge_terminals`.
#[allow(clippy::large_enum_variant)]
pub(crate) enum StagedTerminal {
    Deferred {
        action_id: u64,
        terminal_seq: u64,
    },
    Captured(TerminalCaptureRecord),
}

/// Fixed-capacity preallocated staging buffer for terminal transitions produced during one control
/// release before `publish_at` flushes them into the SPSC ring (0 RT heap allocations; backing
/// slice is allocated once at `SupervisorInner::new` to keep `SupervisorInner` small on the stack).
pub(crate) struct StagedTerminalBuf {
    slots: Box<[Option<StagedTerminal>]>,
    len: usize,
}

/// Fixed-size deduplication tracker for terminal edges keyed by `(action_id, terminal_seq)`.
/// Guarantees an already-captured action slot is never re-captured during dwell ticks or
/// after moving from `last_action` to `previous_action`.
#[derive(Debug, Default)]
pub(crate) struct TerminalEdgeTracker {
    /// Recent `(action_id, terminal_seq)` pairs that have emitted a terminal edge.
    seen: [(u64, u64); 16],
    head: usize,
}

/// Exclusive-access (`&mut self` only) producer wrapper around `rtrb::Producer<TerminalCaptureRecord>`
/// so `SupervisorInner` remains `Sync` while `armd-control` pushes lock-free with zero allocations.
pub(crate) struct TerminalCaptureTx {
    producer: rtrb::Producer<TerminalCaptureRecord>,
    channel: Arc<TerminalCaptureChannel>,
}


#[derive(Clone)]
struct CachedRichEntry {
    id: u64,
    status: Option<ActionStatus>,
    message: Option<String>,
    durability_warning: Option<String>,
}

/// Shared off-thread consumer and lock-free atomic accounting for terminal capture.
/// `rx` and `rich_action_cache` are NEVER locked on `armd-control`; lock order is strictly
/// `channel.rx -> terminals.inner` (and `rich_action_cache` is a leaf mutex).
pub(crate) struct TerminalCaptureChannel {
    rx: Mutex<rtrb::Consumer<TerminalCaptureRecord>>,
    /// Off-RT mirror of off-RT mutated `ActionStatus` payloads and >256-byte worker strings
    /// (bounded to `TERMINAL_FIFO_CAPACITY`) so delayed dispatch past the 2 s window never drops
    /// or truncates any field without touching `SupervisorLock`. Never accessed on `armd-control`.
    rich_action_cache: Mutex<VecDeque<CachedRichEntry>>,
    /// Monotonic high-water mark of action IDs emitted via synthetic (`obs_override`) terminals
    /// across ALL drain consumers. Because action IDs are strictly monotonic and single-active-slot
    /// (`begin_action` always terminates any earlier running action before assigning a higher ID),
    /// any `running` snapshot with `action.id <= max_synthetic_action_id` is guaranteed stale
    /// without ring eviction or mutex overhead.
    max_synthetic_action_id: AtomicU64,
    control_thread_lock_attempts: AtomicU64,
    next_terminal_seq: AtomicU64,
    captured: AtomicU64,
    saturated: AtomicU64,
    ever_saturated: AtomicBool,
    inflight: AtomicUsize,
    admission_cap: AtomicUsize,
    test_push_cap: AtomicUsize,
    drain_paused: AtomicBool,
    /// Off-RT sequence repeat commit barrier: when non-zero, holds `action_id` of a sequence
    /// whose RT-committed repeat episode is still finishing disk/catalog publication so terminal
    /// delivery waits until `episodes` and `EpisodeStore` are finalized (never touched on RT).
    sequence_commit_inflight: AtomicU64,
    /// Condvar signaled when `sequence_commit_inflight` clears so normal shutdown drains and
    /// delivers every originating WS terminal and observer broadcast with the committed episode ID
    /// before broker/router finalization runs.
    sequence_commit_wait: (Mutex<()>, std::sync::Condvar),
}

/// Off-RT RAII guard holding [`TerminalCaptureChannel::sequence_commit_inflight`] while an
/// RT-committed sequence repeat episode finishes off-thread publication and rich-status mirroring.
pub(crate) struct SequenceCommitGuard {
    channel: Arc<TerminalCaptureChannel>,
    hub: Arc<StateHub>,
    action_id: u64,
}

fn match_action(cand: &ActionStatus, rec: &TerminalCaptureRecord) -> bool {
    cand.id == rec.action_id
        && !cand.is_running()
        && cand.state == rec.state
        && cand.code == rec.code
}

fn merge_sequence_rich_fields(dst: &mut ActionStatus, src: &ActionStatus) {
    if dst.sequence_id.is_none() {
        dst.sequence_id = src.sequence_id.clone();
    }
    if dst.repeats.is_none() {
        dst.repeats = src.repeats;
    }
    if src.repeat_index.is_some() {
        dst.repeat_index = src.repeat_index;
    }
    if src.step_index.is_some() {
        dst.step_index = src.step_index;
    }
    if src.step_type.is_some() {
        dst.step_type = src.step_type.clone();
    }
    if dst.is_running() {
        if src.phase.is_some() {
            dst.phase = src.phase;
        }
        if src.progress > dst.progress {
            dst.progress = src.progress;
        }
    }
    if dst.episodes.len() < src.episodes.len() {
        dst.episodes = src.episodes.clone();
    }
    if !dst.is_running() && dst.state.as_str() != "done" && dst.failed_step.is_none() {
        dst.failed_step = src.failed_step.or(src.step_index);
    } else if dst.failed_step.is_none() && src.failed_step.is_some() {
        dst.failed_step = src.failed_step;
    }
}

fn resolve_full_action(
    rec: &TerminalCaptureRecord,
    live_slots: &[Option<&ActionStatus>],
    latest_obs: Option<&ArmObservation>,
    channel: &TerminalCaptureChannel,
) -> Option<ActionStatus> {
    let entry = channel.take_rich_entry(rec.action_id);
    let mut base: Option<ActionStatus> = None;
    for slot in live_slots.iter().flatten() {
        if match_action(slot, rec) {
            base = Some((*slot).clone());
            break;
        }
    }
    if base.is_none() {
        if let Some(obs) = latest_obs {
            for slot in [obs.previous_action.as_ref(), obs.action.as_ref()]
                .into_iter()
                .flatten()
            {
                if match_action(slot, rec) {
                    base = Some(slot.clone());
                    break;
                }
            }
        }
    }
    let mut out = match (base, entry) {
        (Some(mut matched), sidecar) => {
            // Re-apply captured terminal fields even when a live slot or later snapshot matched
            // `(id, state, code)`, so subsequent in-place reconciliation (e.g., clearing a
            // dir-sync durability warning) cannot alter the captured terminal payload depending
            // on drain timing.
            rec.apply_terminal_overlay(&mut matched);
            if let Some(sc) = sidecar {
                if let Some(cached) = sc.status.as_ref() {
                    if matched.kind.as_str() == "sequence" {
                        merge_sequence_rich_fields(&mut matched, cached);
                    }
                }
                if rec.has_message {
                    if let Some(full_msg) = sc.message {
                        if rec.message.is_truncated() || full_msg.len() > rec.message.as_str().len() {
                            matched.message = Some(full_msg);
                        }
                    }
                }
                if rec.has_durability_warning {
                    if let Some(full_warn) = sc.durability_warning {
                        if rec.durability_warning.is_truncated()
                            || full_warn.len() > rec.durability_warning.as_str().len()
                        {
                            matched.durability_warning = Some(full_warn);
                        }
                    }
                }
            }
            matched
        }
        (None, Some(sc)) => {
            let mut out = match sc.status {
                Some(mut cached) => {
                    rec.apply_terminal_overlay(&mut cached);
                    if cached.kind.as_str() == "sequence"
                        && cached.state.as_str() != "done"
                        && cached.failed_step.is_none()
                    {
                        cached.failed_step = cached.step_index;
                    }
                    cached
                }
                None => rec.build_action_status(),
            };
            if rec.has_message {
                if let Some(full_msg) = sc.message {
                    if rec.message.is_truncated() || full_msg.len() > rec.message.as_str().len() {
                        out.message = Some(full_msg);
                    }
                }
            }
            if rec.has_durability_warning {
                if let Some(full_warn) = sc.durability_warning {
                    if rec.durability_warning.is_truncated()
                        || full_warn.len() > rec.durability_warning.as_str().len()
                    {
                        out.durability_warning = Some(full_warn);
                    }
                }
            }
            out
        }
        (None, None) => return None,
    };
    if !rec.has_durability_warning {
        out.durability_warning = None;
    }
    Some(out)
}

/// Single authoritative per-joint feedback age & freshness computation shared by
/// `TerminalCaptureRecord::capture_from` (`feedback_age_ms` + `feedback_valid`) and
/// `cached_hardware_feedback_fresh`. Never mutates `ArmCore::feedback().instants()`.
#[inline]
#[allow(clippy::too_many_arguments)]
pub(crate) fn compute_cached_feedback_state(
    feedback_numeric_valid: bool,
    last_poll_feedback_valid: bool,
    has_driver: bool,
    dof: usize,
    instants: &[Option<Instant>],
    cached_ages: &[Option<Duration>],
    cached_feedback_at: Option<Instant>,
    now: Instant,
    max_age: Duration,
) -> ([Option<u64>; CAPTURE_MAX_DOF], bool) {
    let n = dof.min(CAPTURE_MAX_DOF);
    let mut age_ms = [None; CAPTURE_MAX_DOF];
    if !has_driver {
        for slot in &mut age_ms[..n] {
            *slot = Some(0);
        }
        return (age_ms, feedback_numeric_valid);
    }

    let complete_instants =
        instants.len() >= n && n > 0 && instants[..n].iter().all(Option::is_some);
    let elapsed_since_poll = cached_feedback_at.map(|t| now.saturating_duration_since(t));
    let mut all_fresh = feedback_numeric_valid && last_poll_feedback_valid && n > 0;

    for (i, slot) in age_ms.iter_mut().enumerate().take(n) {
        let dur = if complete_instants {
            instants[i].map(|rx| now.saturating_duration_since(rx))
        } else {
            let from_poll = cached_ages
                .get(i)
                .copied()
                .flatten()
                .zip(elapsed_since_poll)
                .map(|(age, elapsed)| age.saturating_add(elapsed));
            from_poll.or_else(|| {
                instants
                    .get(i)
                    .copied()
                    .flatten()
                    .map(|rx| now.saturating_duration_since(rx))
            })
        };
        match dur {
            Some(d) => {
                *slot = Some(d.as_millis().min(u128::from(u64::MAX)) as u64);
                if d > max_age {
                    all_fresh = false;
                }
            }
            None => {
                *slot = None;
                all_fresh = false;
            }
        }
    }
    (age_ms, all_fresh)
}

/// Pure, lock-free cached driver freshness check shared by terminal capture tests.
/// Derives from [`compute_cached_feedback_state`] so `feedback_valid` and `feedback_age_ms`
/// always agree on the exact same per-joint sample.
#[cfg(test)]
#[inline]
#[allow(clippy::too_many_arguments)]
pub(crate) fn evaluate_cached_feedback_fresh(
    feedback_numeric_valid: bool,
    last_poll_feedback_valid: bool,
    has_driver: bool,
    dof: usize,
    instants: &[Option<Instant>],
    cached_ages: &[Option<Duration>],
    cached_feedback_at: Option<Instant>,
    now: Instant,
    max_age: Duration,
) -> bool {
    compute_cached_feedback_state(
        feedback_numeric_valid,
        last_poll_feedback_valid,
        has_driver,
        dof,
        instants,
        cached_ages,
        cached_feedback_at,
        now,
        max_age,
    )
    .1
}


impl Default for StagedTerminalBuf {
    fn default() -> Self {
        let mut v = Vec::with_capacity(4);
        for _ in 0..4 {
            v.push(None);
        }
        Self {
            slots: v.into_boxed_slice(),
            len: 0,
        }
    }
}

impl StagedTerminalBuf {
    #[inline]
    pub(crate) fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub(crate) fn has_deferred(&self, action_id: u64) -> bool {
        self.deferred_seq_for(action_id).is_some()
    }

    #[inline]
    pub(crate) fn deferred_seq_for(&self, action_id: u64) -> Option<u64> {
        for slot in &self.slots[..self.len] {
            if let Some(StagedTerminal::Deferred {
                action_id: id,
                terminal_seq,
            }) = slot
            {
                if *id == action_id {
                    return Some(*terminal_seq);
                }
            }
        }
        None
    }

    #[inline]
    pub(crate) fn replace_deferred_with_captured(
        &mut self,
        action_id: u64,
        rec: TerminalCaptureRecord,
    ) -> bool {
        for slot in &mut self.slots[..self.len] {
            if let Some(StagedTerminal::Deferred { action_id: id, .. }) = slot {
                if *id == action_id {
                    *slot = Some(StagedTerminal::Captured(rec));
                    return true;
                }
            }
        }
        false
    }

    /// Stages a terminal entry inline; if all 4 slots are already occupied, evicts and returns
    /// the OLDEST staged entry so the caller can flush it to the SPSC ring first and preserve
    /// strict `terminal_seq` FIFO ordering.
    #[inline]
    pub(crate) fn push_evicting_oldest(&mut self, item: StagedTerminal) -> Option<StagedTerminal> {
        let evicted = if self.len == self.slots.len() {
            self.pop_front()
        } else {
            None
        };
        self.slots[self.len] = Some(item);
        self.len += 1;
        evicted
    }

    #[inline]
    pub(crate) fn pop_front(&mut self) -> Option<StagedTerminal> {
        if self.len == 0 {
            return None;
        }
        let item = self.slots[0].take();
        for i in 1..self.len {
            self.slots[i - 1] = self.slots[i].take();
        }
        self.len -= 1;
        item
    }
}

impl TerminalEdgeTracker {
    #[inline]
    pub(crate) fn has_seen_action(&self, action_id: u64) -> bool {
        action_id != 0 && self.seen.iter().any(|&(id, _)| id == action_id)
    }

    #[inline]
    pub(crate) fn has_seen_edge(&self, action_id: u64, terminal_seq: u64) -> bool {
        action_id != 0
            && self
                .seen
                .iter()
                .any(|&(id, seq)| id == action_id && (terminal_seq == 0 || seq == terminal_seq))
    }

    #[inline]
    pub(crate) fn record_edge(&mut self, action_id: u64, terminal_seq: u64) -> bool {
        if action_id == 0 || self.has_seen_action(action_id) {
            return false;
        }
        self.seen[self.head] = (action_id, terminal_seq);
        self.head = (self.head + 1) % self.seen.len();
        true
    }
}

impl TerminalCaptureTx {
    /// Attempts a lock-free, allocation-free push into the preallocated SPSC ring.
    /// Charges `channel.inflight` with `Release` strictly BEFORE making the element visible to
    /// the consumer; on saturation (`Full`), rolls back `inflight`, increments `saturated`, and
    /// latches `ever_saturated` without allocating or blocking.
    #[inline]
    pub(crate) fn try_push(&mut self, rec: TerminalCaptureRecord) -> bool {
        let cap_limit = self.channel.test_push_cap.load(Ordering::Relaxed);
        let cur_inflight = self.channel.inflight.load(Ordering::Acquire);
        if cur_inflight >= cap_limit {
            self.channel.saturated.fetch_add(1, Ordering::Relaxed);
            self.channel.ever_saturated.store(true, Ordering::Release);
            return false;
        }
        // Charge inflight BEFORE pushing so the consumer can never pop & decrement before this add.
        self.channel.inflight.fetch_add(1, Ordering::Release);
        match self.producer.push(rec) {
            Ok(()) => {
                self.channel.captured.fetch_add(1, Ordering::Relaxed);
                true
            }
            Err(rtrb::PushError::Full(_)) => {
                self.channel.inflight.fetch_sub(1, Ordering::AcqRel);
                self.channel.saturated.fetch_add(1, Ordering::Relaxed);
                self.channel.ever_saturated.store(true, Ordering::Release);
                false
            }
        }
    }
}

impl Drop for SequenceCommitGuard {
    fn drop(&mut self) {
        if self.action_id != 0 {
            let _g = self
                .channel
                .sequence_commit_wait
                .0
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            let _ = self.channel.sequence_commit_inflight.compare_exchange(
                self.action_id,
                0,
                Ordering::AcqRel,
                Ordering::Acquire,
            );
            self.channel.sequence_commit_wait.1.notify_all();
            drop(_g);
            self.hub.wake();
        }
    }
}

impl TerminalCaptureChannel {
    pub(crate) fn new() -> (TerminalCaptureTx, Arc<Self>) {
        let (producer, consumer) = rtrb::RingBuffer::new(TERMINAL_CAPTURE_RING_CAPACITY);
        let channel = Arc::new(Self {
            rx: Mutex::new(consumer),
            rich_action_cache: Mutex::new(VecDeque::with_capacity(TERMINAL_FIFO_CAPACITY)),
            max_synthetic_action_id: AtomicU64::new(0),
            control_thread_lock_attempts: AtomicU64::new(0),
            next_terminal_seq: AtomicU64::new(0),
            captured: AtomicU64::new(0),
            saturated: AtomicU64::new(0),
            ever_saturated: AtomicBool::new(false),
            inflight: AtomicUsize::new(0),
            admission_cap: AtomicUsize::new(TERMINAL_CAPTURE_ADMISSION_CAP),
            test_push_cap: AtomicUsize::new(TERMINAL_CAPTURE_RING_CAPACITY),
            drain_paused: AtomicBool::new(false),
            sequence_commit_inflight: AtomicU64::new(0),
            sequence_commit_wait: (Mutex::new(()), std::sync::Condvar::new()),
        });
        let tx = TerminalCaptureTx {
            producer,
            channel: Arc::clone(&channel),
        };
        (tx, channel)
    }

    pub(crate) fn acquire_sequence_commit_guard(
        self: &Arc<Self>,
        action_id: u64,
        hub: Arc<StateHub>,
    ) -> SequenceCommitGuard {
        if action_id != 0 {
            let _g = self
                .sequence_commit_wait
                .0
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            self.sequence_commit_inflight.store(action_id, Ordering::Release);
        }
        SequenceCommitGuard {
            channel: Arc::clone(self),
            hub,
            action_id,
        }
    }

    #[inline]
    pub(crate) fn is_sequence_commit_inflight(&self, action_id: u64) -> bool {
        action_id != 0 && self.sequence_commit_inflight.load(Ordering::Acquire) == action_id
    }

    /// Bounded off-RT wait used by normal shutdown finalization: waits up to `deadline` for any
    /// in-flight RT-committed sequence repeat to finish local disk publication and update
    /// `rich_action_cache`. Returns `true` if idle before `deadline`, `false` if still active.
    pub(crate) fn wait_for_sequence_commit_idle_until(&self, deadline: std::time::Instant) -> bool {
        if self.sequence_commit_inflight.load(Ordering::Acquire) == 0 {
            return true;
        }
        let mut g = self
            .sequence_commit_wait
            .0
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        while self.sequence_commit_inflight.load(Ordering::Acquire) != 0 {
            let rem = deadline.saturating_duration_since(std::time::Instant::now());
            if rem.is_zero() {
                return false;
            }
            let (ng, _) = self
                .sequence_commit_wait
                .1
                .wait_timeout(g, rem)
                .unwrap_or_else(|e| e.into_inner());
            g = ng;
        }
        true
    }

    #[inline]
    fn record_synthetic_terminal_id(&self, action_id: u64) {
        if action_id != 0 {
            self.max_synthetic_action_id
                .fetch_max(action_id, Ordering::AcqRel);
        }
    }

    /// True if `action_id` (a `running` snapshot candidate) is at or below the highest action ID
    /// already emitted via a synthetic (`obs_override`) terminal by ANY drain consumer.
    /// Lock-free, O(1), and immune to eviction across arbitrary burst sizes.
    #[inline]
    pub(crate) fn was_synthetically_terminated(&self, action_id: u64) -> bool {
        action_id != 0 && action_id <= self.max_synthetic_action_id.load(Ordering::Acquire)
    }

    #[inline]
    fn lock_rx(&self) -> std::sync::MutexGuard<'_, rtrb::Consumer<TerminalCaptureRecord>> {
        if crate::rt_core::executor::on_rt_control_thread() {
            self.control_thread_lock_attempts
                .fetch_add(1, Ordering::SeqCst);
            crate::rt_core::rt_gate::record_rt_lock_attempt();
            crate::rt_core::executor::assert_not_on_rt_control_thread(
                "TerminalCaptureChannel::rx.lock",
            );
        }
        if crate::rt_core::executor::in_rt_release() {
            crate::rt_core::rt_gate::record_rt_lock_attempt();
            crate::rt_core::executor::assert_not_in_rt_release("TerminalCaptureChannel::rx.lock");
        }
        self.rx.lock().unwrap_or_else(|e| e.into_inner())
    }

    #[inline]
    fn lock_rich_cache(&self) -> std::sync::MutexGuard<'_, VecDeque<CachedRichEntry>> {
        if crate::rt_core::executor::on_rt_control_thread() {
            self.control_thread_lock_attempts
                .fetch_add(1, Ordering::SeqCst);
            crate::rt_core::rt_gate::record_rt_lock_attempt();
            crate::rt_core::executor::assert_not_on_rt_control_thread(
                "TerminalCaptureChannel::rich_action_cache.lock",
            );
        }
        if crate::rt_core::executor::in_rt_release() {
            crate::rt_core::rt_gate::record_rt_lock_attempt();
            crate::rt_core::executor::assert_not_in_rt_release(
                "TerminalCaptureChannel::rich_action_cache.lock",
            );
        }
        self.rich_action_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn control_thread_lock_attempts(&self) -> u64 {
        self.control_thread_lock_attempts.load(Ordering::Acquire)
    }

    pub(crate) fn lock_rx_for_test(&self) -> impl Drop + '_ {
        self.rx.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn lock_rich_cache_for_test(&self) -> impl Drop + '_ {
        self.rich_action_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    pub(crate) fn touch_rx_lock_for_test(&self) {
        let _g = self.lock_rx();
    }

    pub(crate) fn touch_rich_cache_lock_for_test(&self) {
        let _g = self.lock_rich_cache();
    }

    /// Unconditional off-RT mirror for `ActionStatus` payloads (never called on `armd-control`).
    pub(crate) fn mirror_rich_action(&self, action: &ActionStatus) {
        let mut cache = self.lock_rich_cache();
        if let Some(existing) = cache.iter_mut().find(|e| e.id == action.id) {
            match existing.status.as_mut() {
                Some(s) => s.clone_from(action),
                None => existing.status = Some(action.clone()),
            }
            return;
        }
        while cache.len() >= TERMINAL_FIFO_CAPACITY {
            cache.pop_front();
        }
        cache.push_back(CachedRichEntry {
            id: action.id,
            status: Some(action.clone()),
            message: None,
            durability_warning: None,
        });
    }

    /// Off-RT helper updating a sequence action's rich metadata in `rich_action_cache` without
    /// touching `SupervisorLock` or `armd-control`.
    pub(crate) fn update_rich_action_off_rt(
        &self,
        action_id: u64,
        kind: &str,
        f: impl FnOnce(&mut ActionStatus),
    ) {
        if action_id == 0 {
            return;
        }
        let make_blank = || {
            let mut a = ActionStatus::blank();
            a.id = action_id;
            a.kind = HolderLabel::new(kind);
            a.state = HolderLabel::new("running");
            a
        };
        let mut cache = self.lock_rich_cache();
        if let Some(existing) = cache.iter_mut().find(|e| e.id == action_id) {
            let status = existing.status.get_or_insert_with(make_blank);
            f(status);
            return;
        }
        while cache.len() >= TERMINAL_FIFO_CAPACITY {
            cache.pop_front();
        }
        let mut status = make_blank();
        f(&mut status);
        cache.push_back(CachedRichEntry {
            id: action_id,
            status: Some(status),
            message: None,
            durability_warning: None,
        });
    }

    /// Off-RT helper applying mirrored sequence rich fields onto an observation's `ActionStatus`
    /// without removing the cache entry (used for live telemetry & `observation()` reads).
    pub(crate) fn apply_sequence_overlay_off_rt(&self, action: &mut ActionStatus) {
        if action.id == 0 || action.kind.as_str() != "sequence" {
            return;
        }
        // Acquire-load `sequence_commit_inflight` BEFORE locking `rich_action_cache`:
        // - If true, hold back any terminal state to `"running"` so polling clients never observe
        //   a terminal missing an RT-committed repeat episode.
        // - If false, Acquire synchronizes with `SequenceCommitGuard::drop`'s Release, guaranteeing
        //   `rich_action_cache` already holds the committed episode ID.
        let commit_inflight = self.is_sequence_commit_inflight(action.id);
        let cache = self.lock_rich_cache();
        if let Some(cached) = cache
            .iter()
            .find(|e| e.id == action.id)
            .and_then(|e| e.status.as_ref())
        {
            merge_sequence_rich_fields(action, cached);
        }
        drop(cache);
        if commit_inflight && !action.is_running() {
            action.state = HolderLabel::new("running");
            action.code = None;
            action.finished_at = None;
        }
    }

    /// Off-RT mirror for >256-byte persistence worker error messages so delayed recovery past
    /// the 2 s window never truncates `ActionStatus::message`.
    pub(crate) fn mirror_action_message(&self, action_id: u64, message: &str) {
        if action_id == 0 || message.len() <= 256 {
            return;
        }
        let mut cache = self.lock_rich_cache();
        if let Some(existing) = cache.iter_mut().find(|e| e.id == action_id) {
            existing.message = Some(message.to_owned());
            return;
        }
        while cache.len() >= TERMINAL_FIFO_CAPACITY {
            cache.pop_front();
        }
        cache.push_back(CachedRichEntry {
            id: action_id,
            status: None,
            message: Some(message.to_owned()),
            durability_warning: None,
        });
    }

    /// Off-RT mirror for >256-byte persistence worker durability warnings.
    pub(crate) fn mirror_action_durability_warning(&self, action_id: u64, warning: &str) {
        if action_id == 0 || warning.len() <= 256 {
            return;
        }
        let mut cache = self.lock_rich_cache();
        if let Some(existing) = cache.iter_mut().find(|e| e.id == action_id) {
            existing.durability_warning = Some(warning.to_owned());
            return;
        }
        while cache.len() >= TERMINAL_FIFO_CAPACITY {
            cache.pop_front();
        }
        cache.push_back(CachedRichEntry {
            id: action_id,
            status: None,
            message: None,
            durability_warning: Some(warning.to_owned()),
        });
    }

    fn take_rich_entry(&self, action_id: u64) -> Option<CachedRichEntry> {
        let cache = self.lock_rich_cache();
        cache.iter().find(|e| e.id == action_id).cloned()
    }

    /// Allocates the next authoritative monotonic `terminal_seq` (`1, 2, 3, ...`), visible
    /// immediately to subscription cutoffs (`Release`).
    #[inline]
    pub(crate) fn alloc_terminal_seq(&self) -> u64 {
        self.next_terminal_seq.fetch_add(1, Ordering::AcqRel) + 1
    }

    /// Latest authoritative `terminal_seq` assigned so far (`Acquire`).
    #[inline]
    pub(crate) fn latest_terminal_seq(&self) -> u64 {
        self.next_terminal_seq.load(Ordering::Acquire)
    }

    /// Number of captured records currently in flight between producer push and broadcaster enqueue.
    #[inline]
    pub(crate) fn inflight(&self) -> usize {
        self.inflight.load(Ordering::Acquire)
    }

    #[inline]
    pub(crate) fn admission_cap(&self) -> usize {
        self.admission_cap.load(Ordering::Acquire)
    }

    pub(crate) fn set_admission_cap(&self, cap: usize) {
        self.admission_cap.store(cap.max(1), Ordering::Release);
    }

    pub(crate) fn set_test_push_cap(&self, cap: usize) {
        self.test_push_cap
            .store(cap.clamp(1, TERMINAL_CAPTURE_RING_CAPACITY), Ordering::Release);
    }

    pub(crate) fn set_drain_paused(&self, paused: bool) {
        self.drain_paused.store(paused, Ordering::Release);
    }

    pub(crate) fn stats(&self) -> crate::diagnostics::tick::TerminalCaptureStats {
        crate::diagnostics::tick::TerminalCaptureStats {
            captured: self.captured.load(Ordering::Relaxed),
            saturated: self.saturated.load(Ordering::Relaxed),
            ever_saturated: self.ever_saturated.load(Ordering::Acquire),
            queued: self.inflight.load(Ordering::Acquire) as u64,
        }
    }

    fn drain_locked(
        &self,
        rx: &mut rtrb::Consumer<TerminalCaptureRecord>,
        terminals: &TerminalBroadcaster,
        hub: &StateHub,
        latest_snap: Option<&Snapshot>,
        live_slots: &[Option<&ActionStatus>],
        template_obs: &ArmObservation,
        respect_publish_barrier: bool,
    ) -> usize {
        if self.drain_paused.load(Ordering::Acquire) {
            return 0;
        }
        let mut drained = 0usize;
        let hub_closed = hub.is_closed();
        // Single authoritative barrier policy governing BOTH ring eligibility AND synthesis:
        // when the hub is closed or barrier is disabled, an unmet `publish_seq` synthesizes a
        // self-contained captured observation instead of carrying an unmet sequence into the FIFO.
        let enforce_barrier = respect_publish_barrier && !hub_closed;
        let consumed_seq = latest_snap.map_or(0, |s| s.meta.seq);
        let latest_obs = latest_snap.map(|s| &s.obs);
        while let Ok(head) = rx.peek() {
            // Keep the sequence repeat commit barrier active in EVERY drain (including shutdown
            // drains where `respect_publish_barrier == false`) so a terminal never omits an
            // RT-committed repeat episode.
            if self.is_sequence_commit_inflight(head.action_id) {
                break;
            }
            if enforce_barrier {
                if let Some(req_seq) = head.publish_seq {
                    if consumed_seq < req_seq {
                        break;
                    }
                }
            }
            let Ok(rec) = rx.pop() else {
                break;
            };
            let mut action = resolve_full_action(&rec, live_slots, latest_obs, self)
                .unwrap_or_else(|| rec.build_action_status());
            hub.attach_action_measurement_to_action(&mut action);
            hub.seal_and_attach_action_diag_to_action(&mut action);

            let needs_captured_obs = rec.publish_seq.is_none()
                || (latest_obs.is_none() && hub.published_seq() == 0)
                || (!enforce_barrier && rec.publish_seq.is_some_and(|req| consumed_seq < req));
            let (obs_override, required_publish_seq) = if needs_captured_obs {
                self.record_synthetic_terminal_id(rec.action_id);
                (
                    Some(rec.build_observation(latest_obs.unwrap_or(template_obs), action.clone())),
                    None,
                )
            } else {
                (None, rec.publish_seq)
            };

            // Enqueue into `TerminalBroadcaster` (updating `max_fifo_len` & `pending_total` with
            // `Release`) STRICTLY BEFORE decrementing `self.inflight` (`Release`).
            let _ = terminals.record_finished_with_seq(
                rec.terminal_seq,
                action,
                obs_override,
                required_publish_seq,
            );
            self.inflight.fetch_sub(1, Ordering::Release);
            drained += 1;
        }
        drained
    }

    /// Drains ready records from the SPSC ring into `terminals` in strict `terminal_seq` order.
    pub(crate) fn drain_into(
        &self,
        terminals: &TerminalBroadcaster,
        hub: &StateHub,
        latest_snap: Option<&Snapshot>,
        live_slots: &[Option<&ActionStatus>],
        template_obs: &ArmObservation,
        respect_publish_barrier: bool,
    ) -> usize {
        if self.drain_paused.load(Ordering::Acquire) {
            return 0;
        }
        let mut rx = self.lock_rx();
        self.drain_locked(
            &mut rx,
            terminals,
            hub,
            latest_snap,
            live_slots,
            template_obs,
            respect_publish_barrier,
        )
    }

    /// Atomically drains ready pre-subscription captures and activates `sub_id`'s cutoff under
    /// the `(channel.rx -> terminals.inner)` lock order so a newly registered subscriber can
    /// neither inherit a full pre-subscription ring nor lose its first eligible terminal.
    pub(crate) fn drain_and_activate_subscriber(
        &self,
        sub_id: u64,
        terminals: &TerminalBroadcaster,
        hub: &StateHub,
        latest_snap: Option<&Snapshot>,
        template_obs: &ArmObservation,
    ) {
        let mut rx = self.lock_rx();
        let _ = self.drain_locked(
            &mut rx,
            terminals,
            hub,
            latest_snap,
            &[],
            template_obs,
            true,
        );
        terminals.activate_subscriber_cutoff(sub_id, || self.latest_terminal_seq());
    }
}

// SAFETY: `TerminalCaptureTx` exposes no `&self` access to `self.producer`; all push operations
// require `&mut self`, enforcing single-producer exclusive access (identical to `PolicyTerminalTx`).
unsafe impl Sync for TerminalCaptureTx {}
