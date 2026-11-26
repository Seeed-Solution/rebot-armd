//! RT-safe off-thread recording stream and single close-fence primitive (Adapter 2).
//!
//! - Samples are fixed-size `Copy` [`RecorderSample`] records pushed over PR0B
//!   [`crate::rt_core::stream`] (`rtrb` SPSC) directly from the control release (`armd-control`)
//!   with zero heap allocations, zero heap frees, and zero locks.
//! - Construction and destruction of the stream channel and [`RtRecorderShared`] happen strictly
//!   off-RT (`SupervisorInner::new` / `spawn_executor` / off-RT teardown).
//! - All RT session exits (manual save/discard, monitor save/discard, sequence repeat commit/discard,
//!   SAFE-HOLD, fault fence, shutdown/handback) converge on [`RtRecorder::close_active`], which
//!   freezes `(pushed, dropped, last_ok_stream_seq)` — where `last_ok_stream_seq` is strictly the
//!   last sequence accepted by `rtrb` (0 when `pushed == 0`), so the off-RT consumer always has a
//!   reachable drain boundary even under trailing overflow drops.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};

use crate::recording::RecordFrame;

use super::mailbox::MAX_DOF;
use super::session::SessionId;
use super::stream::{StreamConsumer, StreamProducer, StreamSample};

/// Preallocated SPSC stream capacity for executor recording (>10 s at 50 Hz).
pub const RECORDER_STREAM_CAPACITY: usize = 512;

/// Maximum samples drained by `armd-dispatch` per wake so control replies are never starved.
pub const RECORDER_DISPATCH_DRAIN_BOUND: usize = 64;

/// Maximum uncollected sessions retained in [`RecorderAssembly`] before evicting the oldest.
pub const MAX_PENDING_RECORDER_SESSIONS: usize = 4;

/// One control-release sample streamed from `armd-control` to off-RT [`RecorderAssembly`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RecorderSample {
    /// PR0B `SessionId::0` identifying the recording session that emitted this sample.
    pub session_id: u64,
    /// Control release sequence (`SupervisorInner::control_release_seq`).
    pub release_seq: u64,
    /// Elapsed wall seconds since supervisor start (matches legacy `RecordFrame::timestamp`).
    pub timestamp: f64,
    /// Active joint count (`<= MAX_DOF`).
    pub dof: u8,
    /// Measured joint positions (`observation.state`, `[..dof]`).
    pub observation_state: [f32; MAX_DOF],
    /// Commanded joint positions (`action`, `[..dof]`).
    pub action: [f32; MAX_DOF],
}

/// RT acknowledgement returned when `SessionSlot::Recorder` is admitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecorderStartAck {
    pub op_id: u64,
    pub session_id: SessionId,
    pub start_calib_rev: u64,
    pub start_tool_digest_hash: u64,
    pub start_dof: u8,
    pub start_control_hz: u32,
}

/// Zero-alloc refusal reason from RT `MaintenanceOp::RecorderStart`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecorderStartError {
    SlotBusy,
    FaultFenced,
    ParentNotAlive,
    StreamUnattached,
}

/// Frozen per-session boundary emitted by [`RtRecorder::close_active`] on every RT exit path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecorderStopFence {
    pub op_id: u64,
    pub session_id: SessionId,
    /// Exact number of samples accepted by `StreamProducer::try_push` for this session.
    pub pushed: usize,
    /// Exact number of samples dropped by `StreamProducer::try_push` (ring full / closed).
    pub dropped: u64,
    /// Last stream sequence number accepted by `rtrb` for this session (`0` iff `pushed == 0`).
    /// Strictly reachable by `StreamConsumer` even when trailing attempts dropped.
    pub last_ok_stream_seq: u64,
    pub stop_calib_rev: u64,
    pub stop_tool_digest_hash: u64,
    pub stop_dof: u8,
    pub stop_control_hz: u32,
    pub revoked: bool,
}

/// Zero-alloc refusal reason from RT `MaintenanceOp::RecorderStop`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecorderStopError {
    /// Save requested with 0 recorded samples; session remains running.
    EmptyEpisode,
    /// No active or recently-closed session matched.
    NotActive,
    /// Active session does not match requested `session_id` / `op_id`.
    SessionMismatch,
    /// Sequence parent macro was no longer alive at the RT commit boundary; session was closed
    /// and its frozen fence is returned for off-RT drain and discard.
    ParentNotAlive { fence: RecorderStopFence },
}

/// Lock-free atomic view of RT recorder progress and frozen close boundary, allocated once off-RT.
#[derive(Debug, Default)]
pub struct RtRecorderShared {
    pub live_op_id: AtomicU64,
    pub live_session: AtomicU64,
    pub live_pushed: AtomicUsize,
    pub live_dropped: AtomicU64,
    pub live_cap_reached: AtomicBool,
    // Frozen last-close fence mirrored on `close_active` (written Release before `live_session = 0`).
    closed_op_id: AtomicU64,
    closed_session: AtomicU64,
    closed_pushed: AtomicUsize,
    closed_dropped: AtomicU64,
    closed_last_ok_seq: AtomicU64,
    closed_calib_rev: AtomicU64,
    closed_tool_digest_hash: AtomicU64,
    closed_dof: AtomicU8,
    closed_control_hz: AtomicU32,
    closed_revoked: AtomicBool,
}

impl RtRecorderShared {
    pub fn store_closed_fence(&self, fence: &RecorderStopFence) {
        self.closed_session.store(fence.session_id.0, Ordering::Relaxed);
        self.closed_pushed.store(fence.pushed, Ordering::Relaxed);
        self.closed_dropped.store(fence.dropped, Ordering::Relaxed);
        self.closed_last_ok_seq.store(fence.last_ok_stream_seq, Ordering::Relaxed);
        self.closed_calib_rev.store(fence.stop_calib_rev, Ordering::Relaxed);
        self.closed_tool_digest_hash.store(fence.stop_tool_digest_hash, Ordering::Relaxed);
        self.closed_dof.store(fence.stop_dof, Ordering::Relaxed);
        self.closed_control_hz.store(fence.stop_control_hz, Ordering::Relaxed);
        self.closed_revoked.store(fence.revoked, Ordering::Relaxed);
        // Release store of `closed_op_id` publishes all closed_* fields above.
        self.closed_op_id.store(fence.op_id, Ordering::Release);
    }

    /// Reads the frozen fence if RT already closed `op_id` / `session_id` (e.g. SAFE-HOLD or shutdown).
    pub fn load_closed_fence_for(&self, op_id: u64, session_id: SessionId) -> Option<RecorderStopFence> {
        let closed_op = self.closed_op_id.load(Ordering::Acquire);
        if closed_op == 0 || closed_op != op_id {
            return None;
        }
        let closed_sid = self.closed_session.load(Ordering::Relaxed);
        if session_id.0 != 0 && closed_sid != session_id.0 {
            return None;
        }
        Some(RecorderStopFence {
            op_id: closed_op,
            session_id: SessionId(closed_sid),
            pushed: self.closed_pushed.load(Ordering::Relaxed),
            dropped: self.closed_dropped.load(Ordering::Relaxed),
            last_ok_stream_seq: self.closed_last_ok_seq.load(Ordering::Relaxed),
            stop_calib_rev: self.closed_calib_rev.load(Ordering::Relaxed),
            stop_tool_digest_hash: self.closed_tool_digest_hash.load(Ordering::Relaxed),
            stop_dof: self.closed_dof.load(Ordering::Relaxed),
            stop_control_hz: self.closed_control_hz.load(Ordering::Relaxed),
            revoked: self.closed_revoked.load(Ordering::Relaxed),
        })
    }

    pub fn is_live_for(&self, op_id: u64, session_id: SessionId) -> bool {
        self.live_session.load(Ordering::Acquire) == session_id.0
            && self.live_op_id.load(Ordering::Acquire) == op_id
    }

    pub fn live_pushed(&self) -> usize {
        self.live_pushed.load(Ordering::Acquire)
    }

    pub fn live_cap_reached(&self) -> bool {
        self.live_cap_reached.load(Ordering::Acquire)
    }
}

/// Inline RT recorder state embedded in `SupervisorInner` (0 alloc, 0 free, 0 lock on `armd-control`).
pub struct RtRecorder {
    tx: Option<StreamProducer<RecorderSample>>,
    active_op_id: u64,
    active_session: u64,
    bound_action_id: u64,
    max_frames: usize,
    pushed: usize,
    dropped: u64,
    last_ok_stream_seq: u64,
    last_closed_fence: Option<RecorderStopFence>,
    shared: Arc<RtRecorderShared>,
}

impl Default for RtRecorder {
    fn default() -> Self {
        Self::new()
    }
}

impl RtRecorder {
    /// Constructs inline RT recorder state off-RT (preallocates the single `Arc<RtRecorderShared>`).
    pub fn new() -> Self {
        Self {
            tx: None,
            active_op_id: 0,
            active_session: 0,
            bound_action_id: 0,
            max_frames: 0,
            pushed: 0,
            dropped: 0,
            last_ok_stream_seq: 0,
            last_closed_fence: None,
            shared: Arc::new(RtRecorderShared::default()),
        }
    }

    /// Attaches the preallocated `StreamProducer` off-RT in `spawn_executor`.
    pub fn attach_producer(&mut self, tx: StreamProducer<RecorderSample>) {
        self.tx = Some(tx);
    }

    #[inline]
    pub fn has_producer(&self) -> bool {
        self.tx.is_some()
    }

    #[inline]
    pub fn shared(&self) -> Arc<RtRecorderShared> {
        Arc::clone(&self.shared)
    }

    #[inline]
    pub fn active_session(&self) -> Option<SessionId> {
        (self.active_session != 0).then_some(SessionId(self.active_session))
    }

    #[inline]
    pub fn active_op_id(&self) -> u64 {
        self.active_op_id
    }

    #[inline]
    pub fn bound_action_id(&self) -> u64 {
        self.bound_action_id
    }

    #[inline]
    pub fn pushed(&self) -> usize {
        self.pushed
    }

    #[inline]
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Begins a new RT recording session (0 alloc, 0 lock).
    pub fn begin(&mut self, op_id: u64, session_id: SessionId, max_frames: usize, bound_action_id: u64) {
        self.active_op_id = op_id;
        self.active_session = session_id.0;
        self.bound_action_id = bound_action_id;
        self.max_frames = max_frames;
        self.pushed = 0;
        self.dropped = 0;
        self.last_ok_stream_seq = 0;
        self.shared.live_pushed.store(0, Ordering::Relaxed);
        self.shared.live_dropped.store(0, Ordering::Relaxed);
        self.shared.live_cap_reached.store(false, Ordering::Relaxed);
        self.shared.live_op_id.store(op_id, Ordering::Relaxed);
        self.shared.live_session.store(session_id.0, Ordering::Release);
    }

    /// Pushes one sample from the control release (0 alloc, 0 free, 0 lock).
    /// Returns `true` if accepted into the SPSC ring.
    #[inline]
    pub fn try_push_sample(
        &mut self,
        release_seq: u64,
        timestamp: f64,
        measured: &[f32],
        commanded: &[f32],
    ) -> bool {
        if self.active_session == 0 || self.pushed >= self.max_frames {
            return false;
        }
        let Some(tx) = self.tx.as_mut() else {
            return false;
        };
        let dof = measured.len().min(commanded.len()).min(MAX_DOF);
        let mut obs = [0.0f32; MAX_DOF];
        let mut act = [0.0f32; MAX_DOF];
        obs[..dof].copy_from_slice(&measured[..dof]);
        act[..dof].copy_from_slice(&commanded[..dof]);
        let sample = RecorderSample {
            session_id: self.active_session,
            release_seq,
            timestamp,
            dof: dof as u8,
            observation_state: obs,
            action: act,
        };
        match tx.try_push(sample) {
            Ok(seq) => {
                self.pushed = self.pushed.saturating_add(1);
                self.last_ok_stream_seq = seq;
                self.shared.live_pushed.store(self.pushed, Ordering::Release);
                if self.pushed >= self.max_frames {
                    self.shared.live_cap_reached.store(true, Ordering::Release);
                }
                true
            }
            Err(_) => {
                self.dropped = self.dropped.saturating_add(1);
                self.shared.live_dropped.store(self.dropped, Ordering::Release);
                false
            }
        }
    }

    /// Single RT close transition shared by save, discard, revoke, SAFE-HOLD, and teardown.
    /// Freezes exact per-session `(pushed, dropped, last_ok_stream_seq)` with zero alloc/lock.
    pub fn close_active(
        &mut self,
        calib_rev: u64,
        tool_digest_hash: u64,
        dof: u8,
        control_hz: u32,
        revoked: bool,
    ) -> Option<RecorderStopFence> {
        if self.active_session == 0 {
            return None;
        }
        let fence = RecorderStopFence {
            op_id: self.active_op_id,
            session_id: SessionId(self.active_session),
            pushed: self.pushed,
            dropped: self.dropped,
            last_ok_stream_seq: self.last_ok_stream_seq,
            stop_calib_rev: calib_rev,
            stop_tool_digest_hash: tool_digest_hash,
            stop_dof: dof,
            stop_control_hz: control_hz,
            revoked,
        };
        self.shared.store_closed_fence(&fence);
        self.shared.live_session.store(0, Ordering::Release);
        self.shared.live_op_id.store(0, Ordering::Release);
        self.active_op_id = 0;
        self.active_session = 0;
        self.bound_action_id = 0;
        self.max_frames = 0;
        self.pushed = 0;
        self.dropped = 0;
        self.last_ok_stream_seq = 0;
        self.last_closed_fence = Some(fence);
        Some(fence)
    }

    #[inline]
    pub fn last_closed_fence_for(&self, op_id: u64, session_id: SessionId) -> Option<RecorderStopFence> {
        self.last_closed_fence.filter(|f| {
            f.op_id == op_id && (session_id.0 == 0 || f.session_id == session_id)
        })
    }
}

/// Result of collecting a closed session's samples off-RT.
#[derive(Debug, Default)]
pub struct CollectedSession {
    pub frames: Vec<RecordFrame>,
    pub dropped: u64,
    pub first_release_seq: Option<u64>,
    pub last_release_seq: Option<u64>,
}

struct SessionBucket {
    session_id: u64,
    frames: Vec<RecordFrame>,
    first_release_seq: Option<u64>,
    last_release_seq: Option<u64>,
}

struct AssemblyInner {
    rx: StreamConsumer<RecorderSample>,
    sessions: VecDeque<SessionBucket>,
    evicted: VecDeque<(u64, u64)>,
    max_seen_seq: u64,
}

impl AssemblyInner {
    fn ingest(&mut self, sample: StreamSample<RecorderSample>) {
        self.max_seen_seq = self.max_seen_seq.max(sample.seq);
        let p = sample.payload;
        if p.session_id == 0 {
            return;
        }
        if let Some(ev) = self.evicted.iter_mut().find(|e| e.0 == p.session_id) {
            ev.1 = ev.1.saturating_add(1);
            return;
        }
        if self.sessions.back().is_none_or(|b| b.session_id != p.session_id) {
            if let Some(pos) = self.sessions.iter().position(|b| b.session_id == p.session_id) {
                let b = &mut self.sessions[pos];
                let dof = usize::from(p.dof).min(MAX_DOF);
                b.frames.push(RecordFrame {
                    timestamp: p.timestamp,
                    observation_state: p.observation_state[..dof].to_vec(),
                    action: p.action[..dof].to_vec(),
                });
                b.first_release_seq.get_or_insert(p.release_seq);
                b.last_release_seq = Some(p.release_seq);
                return;
            }
            if self.sessions.len() >= MAX_PENDING_RECORDER_SESSIONS {
                if let Some(old) = self.sessions.pop_front() {
                    if self.evicted.len() >= MAX_PENDING_RECORDER_SESSIONS {
                        self.evicted.pop_front();
                    }
                    self.evicted.push_back((old.session_id, old.frames.len() as u64));
                }
            }
            self.sessions.push_back(SessionBucket {
                session_id: p.session_id,
                frames: Vec::new(),
                first_release_seq: None,
                last_release_seq: None,
            });
        }
        let b = self.sessions.back_mut().expect("pushed above");
        let dof = usize::from(p.dof).min(MAX_DOF);
        b.frames.push(RecordFrame {
            timestamp: p.timestamp,
            observation_state: p.observation_state[..dof].to_vec(),
            action: p.action[..dof].to_vec(),
        });
        b.first_release_seq.get_or_insert(p.release_seq);
        b.last_release_seq = Some(p.release_seq);
    }

    fn drain_up_to(&mut self, max: usize) -> usize {
        let mut n = 0;
        while n < max {
            let Some(sample) = self.rx.try_pop() else {
                break;
            };
            self.ingest(sample);
            n += 1;
        }
        n
    }

    fn drain_all(&mut self) -> usize {
        let mut n = 0;
        while let Some(sample) = self.rx.try_pop() {
            self.ingest(sample);
            n += 1;
        }
        n
    }
}

/// Off-RT SPSC consumer and per-session frame assembler. Never locked on `armd-control`.
pub struct RecorderAssembly {
    inner: Mutex<AssemblyInner>,
    control_thread_lock_attempts: AtomicU64,
}

impl RecorderAssembly {
    pub fn new(rx: StreamConsumer<RecorderSample>) -> Self {
        Self {
            inner: Mutex::new(AssemblyInner {
                rx,
                sessions: VecDeque::with_capacity(MAX_PENDING_RECORDER_SESSIONS),
                evicted: VecDeque::with_capacity(MAX_PENDING_RECORDER_SESSIONS),
                max_seen_seq: 0,
            }),
            control_thread_lock_attempts: AtomicU64::new(0),
        }
    }

    #[inline]
    fn lock_inner(&self) -> MutexGuard<'_, AssemblyInner> {
        if super::executor::on_rt_control_thread() {
            self.control_thread_lock_attempts.fetch_add(1, Ordering::SeqCst);
            super::rt_gate::record_rt_lock_attempt();
            super::executor::assert_not_on_rt_control_thread("RecorderAssembly::inner.lock");
        }
        if super::executor::in_rt_release() {
            super::rt_gate::record_rt_lock_attempt();
            super::executor::assert_not_in_rt_release("RecorderAssembly::inner.lock");
        }
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn control_thread_lock_attempts(&self) -> u64 {
        self.control_thread_lock_attempts.load(Ordering::Acquire)
    }

    /// Test-only hook: holds the off-RT assembly mutex to prove `armd-control` never contends on it.
    #[doc(hidden)]
    pub fn lock_for_test(&self) -> impl Drop + '_ {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Bounded drain called by `armd-dispatch` on each wake so recording traffic never starves replies.
    pub fn drain_bounded(&self, max: usize) -> usize {
        let mut g = self.lock_inner();
        g.drain_up_to(max)
    }

    /// Non-blocking bounded drain for `armd-dispatch`: if a blocking `collect_session` currently
    /// holds the assembly mutex, yields immediately so unrelated E-stop/broker replies never stall.
    pub fn try_drain_bounded(&self, max: usize) -> usize {
        if super::executor::on_rt_control_thread() {
            self.control_thread_lock_attempts.fetch_add(1, Ordering::SeqCst);
            super::rt_gate::record_rt_lock_attempt();
            super::executor::assert_not_on_rt_control_thread("RecorderAssembly::inner.try_lock");
        }
        if super::executor::in_rt_release() {
            super::rt_gate::record_rt_lock_attempt();
            super::executor::assert_not_in_rt_release("RecorderAssembly::inner.try_lock");
        }
        match self.inner.try_lock() {
            Ok(mut g) => g.drain_up_to(max),
            Err(std::sync::TryLockError::Poisoned(p)) => p.into_inner().drain_up_to(max),
            Err(std::sync::TryLockError::WouldBlock) => 0,
        }
    }

    /// Collects all samples belonging to `fence.session_id` (blocking-thread safe; leaves later
    /// sessions intact). Call strictly AFTER RT has closed `fence.session_id`.
    pub fn collect_session(&self, fence: RecorderStopFence) -> CollectedSession {
        let mut g = self.lock_inner();
        let _ = g.drain_all();
        if fence.pushed == 0 {
            // Explicit empty case: remove any stray bucket if present and preserve RT drop count.
            if let Some(pos) = g.sessions.iter().position(|b| b.session_id == fence.session_id.0) {
                g.sessions.remove(pos);
            }
            return CollectedSession {
                frames: Vec::new(),
                dropped: fence.dropped,
                first_release_seq: None,
                last_release_seq: None,
            };
        }
        debug_assert!(
            fence.last_ok_stream_seq == 0 || g.max_seen_seq >= fence.last_ok_stream_seq,
            "consumer must reach last_ok_stream_seq={} (max_seen_seq={})",
            fence.last_ok_stream_seq,
            g.max_seen_seq
        );
        let mut out = CollectedSession {
            frames: Vec::new(),
            dropped: fence.dropped,
            first_release_seq: None,
            last_release_seq: None,
        };
        if let Some(pos) = g.sessions.iter().position(|b| b.session_id == fence.session_id.0) {
            let bucket = g.sessions.remove(pos).expect("position in range");
            let missing = fence.pushed.saturating_sub(bucket.frames.len()) as u64;
            out.frames = bucket.frames;
            out.first_release_seq = bucket.first_release_seq;
            out.last_release_seq = bucket.last_release_seq;
            out.dropped = out.dropped.saturating_add(missing);
        } else if let Some(pos) = g.evicted.iter().position(|e| e.0 == fence.session_id.0) {
            let (_, ev_count) = g.evicted.remove(pos).expect("position in range");
            let missing = ev_count.max(fence.pushed as u64);
            out.dropped = out.dropped.saturating_add(missing);
        } else {
            out.dropped = out.dropped.saturating_add(fence.pushed as u64);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rt_core::stream;

    #[test]
    fn trailing_drops_empty_revoked_and_immediate_restart_preserve_exact_boundaries() {
        let (tx, rx) = stream::channel::<RecorderSample>(3);
        let mut rt = RtRecorder::new();
        rt.attach_producer(tx);
        let asm = RecorderAssembly::new(rx);

        // 1) Session 1: push 3 ok, then 2 trailing drops -> last_ok_stream_seq must remain reachable (3).
        rt.begin(10, SessionId(1), 100, 0);
        for i in 1..=3u64 {
            assert!(rt.try_push_sample(i, i as f64 * 0.02, &[i as f32; 2], &[i as f32; 2]));
        }
        assert!(!rt.try_push_sample(4, 0.08, &[4.0; 2], &[4.0; 2]));
        assert!(!rt.try_push_sample(5, 0.10, &[5.0; 2], &[5.0; 2]));
        let f1 = rt.close_active(1, 2, 2, 50, false).unwrap();
        assert_eq!((f1.pushed, f1.dropped, f1.last_ok_stream_seq), (3, 2, 3));

        // 2) Immediate restart: Session 2 starts BEFORE Session 1 is collected; partial drain moves
        //    Session 1 frames out of ring so Session 2 can push cleanly.
        assert_eq!(asm.drain_bounded(2), 2);
        rt.begin(11, SessionId(2), 100, 0);
        assert!(rt.try_push_sample(10, 0.20, &[9.0; 2], &[8.0; 2]));
        let f2 = rt.close_active(1, 2, 2, 50, false).unwrap();
        assert_eq!((f2.pushed, f2.dropped), (1, 0));

        // Collect Session 1: gets exactly its 3 frames and 2 drops, leaving Session 2 intact!
        let c1 = asm.collect_session(f1);
        assert_eq!((c1.frames.len(), c1.dropped, c1.first_release_seq, c1.last_release_seq), (3, 2, Some(1), Some(3)));
        // Collect Session 2: gets its 1 frame intact!
        let c2 = asm.collect_session(f2);
        assert_eq!((c2.frames.len(), c2.dropped, c2.first_release_seq), (1, 0, Some(10)));
        assert_eq!(c2.frames[0].observation_state, vec![9.0, 9.0]);
        assert_eq!(c2.frames[0].action, vec![8.0, 8.0]);

        // 3) Empty revoked session: 0 pushed, 0 dropped, last_ok_stream_seq == 0.
        rt.begin(12, SessionId(3), 100, 0);
        let f3 = rt.close_active(1, 2, 2, 50, true).unwrap();
        assert_eq!((f3.pushed, f3.dropped, f3.last_ok_stream_seq, f3.revoked), (0, 0, 0, true));
        assert_eq!(rt.shared().load_closed_fence_for(12, SessionId(3)), Some(f3));
        let c3 = asm.collect_session(f3);
        assert_eq!((c3.frames.len(), c3.dropped), (0, 0));
    }
}
