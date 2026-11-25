//! Latest-target mailbox: servo / teleop / policy targets from the broker to the RT
//! (armd 1.6 RT core, step 2 foundation; design section 3, "Urgent control and servo").
//!
//! One writer (the broker, normal priority) and one reader (the RT): [`TargetWriter`] and
//! [`TargetReader`] are neither `Clone` nor shareable (`TargetReader` is not `Sync`), so a
//! frame is consumed by one consumer. The reader takes the newest unconsumed frame; a newer
//! frame replaces an unconsumed one only when that one is replaceable
//! ([`TargetFrame::replaceable`]), otherwise the submit is refused (`Busy`). Whether a frame
//! was replaced unconsumed (`Coalesced`) is decided by the publishing swap itself, so a frame
//! the reader took concurrently is never reported as coalesced.
//!
//! The writer keeps the last sequence of each stream ([`MAX_STREAMS`] at a time; a further
//! stream is refused with `StreamsFull` until one is closed) and refuses a frame whose
//! sequence does not advance on its stream (duplicate or wrapped), whatever other streams
//! sent in between. Frames are `Copy` with fixed-size joint arrays: neither side allocates.
//!
//! At consumption the RT rechecks a frame ([`TargetFrame::check`], track D): arm and boot
//! identity, its original deadline, the authority (stop, epoch, lease, config revision), the
//! stream the core authorized, the policy generation, the input and vision ages
//! ([`TargetAuthority`]); the core adds its configuration hold (`CONFIG_BUSY`). The outcome
//! of every consumed frame goes back to the writer ([`TargetOutcome`], with a [`Refusal`]
//! detail) for the stream's protocol reply.
//!
//! Lease release / disconnect: consumption checks the lease **valid at that instant**, so a
//! frame buffered before its lease was released is refused (`LEASE_LOST`), never applied.
//! A closing stream still calls [`TargetWriter::close_stream`] and the core
//! `revoke_stream`; an explicit stop goes through `StopRequests` as for any stop.

use std::time::{Duration, Instant};

use super::broker::{Authority, BootId};
use super::ids::ArmId;
use super::triple::{triple_buffer, Consumer, Producer};

/// Joints a frame carries at most.
pub const MAX_DOF: usize = 8;

/// Streams whose sequence state a writer keeps.
pub const MAX_STREAMS: usize = 8;

/// A target stream (one WS servo stream, one policy run).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StreamId(pub u64);

/// Absolute validity deadlines carried atomically in the triple-buffer slot alongside a
/// policy [`TargetFrame`] (zero allocations, `Copy`, `None` for non-policy / detector frames).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PolicyValidityBounds {
    /// Earliest expiry across the embedding window (`min(window_newest, window_oldest)`).
    pub vision_window_expires_at: Option<Instant>,
    /// Absolute clock-handshake expiry (`clock_handshake_expires_at_ms` anchored at submit).
    pub handshake_expires_at: Option<Instant>,
    /// Strictest contributing source expiry for a chunked slot (inclusive: `now > exp` refuses
    /// with `DEADLINE_EXPIRED` -> clean hold).
    pub chunk_slot_expires_at: Option<Instant>,
    /// Exclusive end of the selected chunked control period (`now >= end` refuses with
    /// `DEADLINE_EXPIRED`, matching `commit_gate`'s half-open `[t_N, t_{N+1})` slot boundary).
    pub chunk_period_end: Option<Instant>,
}

/// One target.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetFrame {
    /// The arm and boot the frame was produced for.
    pub arm: ArmId,
    pub boot: BootId,
    pub stream: StreamId,
    /// Stream sequence; must increase within a stream.
    pub seq: u64,
    pub authority: Authority,
    /// Policy authorization generation (0: not a policy frame).
    pub policy_generation: u64,
    /// When the input arrived (WS receive, policy observation).
    pub input_at: Instant,
    /// Camera sample time behind the frame (vision policies).
    pub vision_at: Option<Instant>,
    /// The vision frame behind it (`vision_frame_id` telemetry), None without vision.
    pub vision_frame_id: Option<u64>,
    /// Original monotonic deadline: not consumed after it (`DEADLINE_EXPIRED`).
    pub deadline: Instant,
    pub dof: u8,
    pub q: [f32; MAX_DOF],
    /// The protocol lets a newer frame replace this one before it is consumed.
    pub replaceable: bool,
}

/// Why a target (or a command) was refused: the code and what the client needs to act on
/// it, without allocating.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Refusal {
    pub code: &'static str,
    pub detail: RefusalDetail,
}

/// Detail of a [`Refusal`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RefusalDetail {
    #[default]
    None,
    /// The current revision (`PLAN_STALE`, `REVISION_MISMATCH`).
    CurrentRevision(u64),
    /// `LEASE_LOST`: `"preempted"` (a forced acquire evicted it) or `"expired"`.
    LeaseLoss { reason: &'static str },
    /// `WORKSPACE_VIOLATION` / `JOINT_LIMIT`: the joint (or axis) that violated, if known.
    Limit { joint: Option<u8> },
}

impl Refusal {
    pub fn code(code: &'static str) -> Self { Self { code, detail: RefusalDetail::None } }
}

impl From<&'static str> for Refusal {
    fn from(code: &'static str) -> Self { Self::code(code) }
}

/// The RT's answer to one consumed frame, back to the stream's writer
/// ([`TargetReader::report`], [`TargetWriter::outcomes`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TargetOutcome {
    pub stream: StreamId,
    pub seq: u64,
    pub result: Result<(), Refusal>,
}

/// Outcomes buffered RT -> writer; when full the RT drops one and counts it.
pub const OUTCOME_CAPACITY: usize = 16;

/// What the core currently authorizes for targets (core state, `ArmCore::target_authority`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TargetAuthority {
    /// The one stream whose frames are consumed (None: none).
    pub stream: Option<StreamId>,
    /// Current policy authorization generation (0: the stream is not a policy).
    pub policy_generation: u64,
    /// Oldest input a consumed frame may carry (None: not bounded).
    pub max_input_age: Option<Duration>,
    /// Oldest camera sample behind a consumed frame (None: not bounded).
    pub max_vision_age: Option<Duration>,
}

impl TargetFrame {
    pub fn positions(&self) -> &[f32] { &self.q[..usize::from(self.dof).min(MAX_DOF)] }

    /// The refusal code at consumption, None to apply it. Order: `INVALID_COMMAND` (dof),
    /// `WRONG_TARGET`, `DEADLINE_EXPIRED`, authority codes ([`Authority::admits`]),
    /// `STREAM_NOT_AUTHORIZED`, `POLICY_REVOKED`, `INPUT_STALE`, `VISION_STALE`.
    pub fn check(&self, arm: ArmId, boot: BootId, now: Instant, current: &Authority, targets: &TargetAuthority) -> Option<&'static str> {
        if self.dof == 0 || usize::from(self.dof) > MAX_DOF { return Some("INVALID_COMMAND"); }
        if self.arm != arm || self.boot != boot || self.boot.is_zero() { return Some("WRONG_TARGET"); }
        if now > self.deadline { return Some("DEADLINE_EXPIRED"); }
        if let Some(code) = self.authority.admits(current) { return Some(code); }
        if targets.stream != Some(self.stream) { return Some("STREAM_NOT_AUTHORIZED"); }
        if self.policy_generation != targets.policy_generation { return Some("POLICY_REVOKED"); }
        if targets.max_input_age.is_some_and(|m| now.saturating_duration_since(self.input_at) > m) { return Some("INPUT_STALE"); }
        if let (Some(m), Some(at)) = (targets.max_vision_age, self.vision_at) {
            if now.saturating_duration_since(at) > m { return Some("VISION_STALE"); }
        }
        None
    }

    /// Single effective refusal code for a policy target frame (`policy_generation != 0`).
    /// Precedence: `INVALID_COMMAND` -> `WRONG_TARGET` -> authority (`STOPPED`/`LEASE_LOST`/
    /// `EPOCH_MISMATCH`/`PLAN_STALE`/`PARENT_REPLACED`) -> `STREAM_NOT_AUTHORIZED` ->
    /// `POLICY_REVOKED` -> `INPUT_STALE` (aged or future) -> `VISION_STALE` (missing, aged,
    /// or future when `max_vision_age` is set) -> `DEADLINE_EXPIRED`.
    pub fn check_policy(
        &self,
        arm: ArmId,
        boot: BootId,
        now: Instant,
        current: &Authority,
        targets: &TargetAuthority,
    ) -> Option<&'static str> {
        self.check_policy_with_validity(
            arm,
            boot,
            now,
            current,
            targets,
            PolicyValidityBounds::default(),
        )
    }

    /// Single effective refusal code for a policy target frame together with its atomic
    /// [`PolicyValidityBounds`]. Enforces `HANDSHAKE_EXPIRED` and embedding window expiry
    /// independently of `vision_at` / `vision_frame_id`.
    pub fn check_policy_with_validity(
        &self,
        arm: ArmId,
        boot: BootId,
        now: Instant,
        current: &Authority,
        targets: &TargetAuthority,
        validity: PolicyValidityBounds,
    ) -> Option<&'static str> {
        if self.dof == 0 || usize::from(self.dof) > MAX_DOF { return Some("INVALID_COMMAND"); }
        if self.arm != arm || self.boot != boot || self.boot.is_zero() { return Some("WRONG_TARGET"); }
        if let Some(code) = self.authority.admits(current) { return Some(code); }
        if targets.stream != Some(self.stream) { return Some("STREAM_NOT_AUTHORIZED"); }
        if self.policy_generation != targets.policy_generation { return Some("POLICY_REVOKED"); }
        if let Some(m) = targets.max_input_age {
            if now.checked_duration_since(self.input_at).is_none_or(|age| age > m) {
                return Some("INPUT_STALE");
            }
        }
        if validity.handshake_expires_at.is_some_and(|exp| now > exp) {
            return Some("HANDSHAKE_EXPIRED");
        }
        if validity.vision_window_expires_at.is_some_and(|exp| now > exp) {
            return Some("VISION_STALE");
        }
        if let Some(m) = targets.max_vision_age {
            if self.vision_at.and_then(|at| now.checked_duration_since(at)).is_none_or(|age| age > m) {
                return Some("VISION_STALE");
            }
        }
        if validity.chunk_period_end.is_some_and(|end| now >= end) {
            return Some("DEADLINE_EXPIRED");
        }
        if validity.chunk_slot_expires_at.is_some_and(|exp| now > exp) {
            return Some("DEADLINE_EXPIRED");
        }
        if now > self.deadline { return Some("DEADLINE_EXPIRED"); }
        None
    }
}

/// Result of [`TargetWriter::submit`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Submitted {
    Accepted,
    /// Accepted and replaced the unconsumed frame with this stream and sequence.
    Coalesced { replaced: u64 },
    /// Refused: the unconsumed frame is not replaceable.
    Busy,
    /// Refused: the sequence does not advance on this stream (duplicate or wrapped).
    OutOfOrder,
    /// Refused: [`MAX_STREAMS`] streams are open ([`TargetWriter::close_stream`]).
    StreamsFull,
}

#[derive(Clone, Copy)]
struct Slot {
    frame: Option<TargetFrame>,
    validity: PolicyValidityBounds,
}

/// The broker's side.
pub struct TargetWriter {
    producer: Producer<Slot>,
    outcomes: rtrb::Consumer<TargetOutcome>,
    /// Last sequence of each open stream.
    streams: [Option<(StreamId, u64)>; MAX_STREAMS],
    last: Option<TargetFrame>,
    coalesced: u64,
}

/// The RT's side. Not `Clone`, not `Sync`: one consumer.
///
/// ```compile_fail
/// fn sync<T: Sync>() {}
/// sync::<armd::rt_core::mailbox::TargetReader>();
/// ```
pub struct TargetReader {
    consumer: Consumer<Slot>,
    outcomes: rtrb::Producer<TargetOutcome>,
    outcomes_dropped: u64,
}

pub fn target_mailbox() -> (TargetWriter, TargetReader) {
    let empty = Slot { frame: None, validity: PolicyValidityBounds::default() };
    let (producer, consumer) = triple_buffer(empty, empty, empty);
    let (otx, orx) = rtrb::RingBuffer::new(OUTCOME_CAPACITY);
    (TargetWriter { producer, outcomes: orx, streams: [None; MAX_STREAMS], last: None, coalesced: 0 }, TargetReader { consumer, outcomes: otx, outcomes_dropped: 0 })
}

impl TargetWriter {
    pub fn submit(&mut self, frame: TargetFrame) -> Submitted {
        self.submit_with_validity(frame, PolicyValidityBounds::default())
    }

    pub fn submit_with_validity(
        &mut self,
        frame: TargetFrame,
        validity: PolicyValidityBounds,
    ) -> Submitted {
        let entry = match self.streams.iter().position(|e| e.is_some_and(|(s, _)| s == frame.stream)) {
            Some(i) => {
                if self.streams[i].is_some_and(|(_, last)| frame.seq <= last) { return Submitted::OutOfOrder; }
                i
            }
            None => match self.streams.iter().position(Option::is_none) {
                Some(i) => i,
                None => return Submitted::StreamsFull,
            },
        };
        // conservative: the reader may take the unconsumed frame right after this check
        if self.producer.unconsumed() && self.last.is_some_and(|l| !l.replaceable) { return Submitted::Busy; }
        let slot = self.producer.back_mut();
        slot.frame = Some(frame);
        slot.validity = validity;
        let overwrote = self.producer.publish();
        self.streams[entry] = Some((frame.stream, frame.seq));
        let replaced = self.last.replace(frame).filter(|_| overwrote);
        match replaced {
            Some(old) => { self.coalesced += 1; Submitted::Coalesced { replaced: old.seq } }
            None => Submitted::Accepted,
        }
    }

    /// Forgets `stream`'s sequence state (the stream ended); its later frames start anew.
    pub fn close_stream(&mut self, stream: StreamId) {
        for e in self.streams.iter_mut() { if e.is_some_and(|(s, _)| s == stream) { *e = None; } }
    }

    /// Frames replaced before consumption.
    pub fn coalesced(&self) -> u64 { self.coalesced }

    /// Drains the RT's outcomes of consumed frames (the stream's protocol replies).
    pub fn outcomes(&mut self, mut each: impl FnMut(TargetOutcome)) -> usize {
        let mut n = 0;
        while let Ok(o) = self.outcomes.pop() { each(o); n += 1; }
        n
    }
}

impl TargetReader {
    /// The newest unconsumed frame (None when nothing new).
    pub fn take(&mut self) -> Option<TargetFrame> {
        self.take_with_validity().map(|(f, _)| f)
    }

    /// The newest unconsumed frame together with its atomic [`PolicyValidityBounds`].
    pub fn take_with_validity(&mut self) -> Option<(TargetFrame, PolicyValidityBounds)> {
        if !self.consumer.refresh() { return None; }
        let slot = self.consumer.front();
        slot.frame.map(|f| (f, slot.validity))
    }

    /// Reports how a consumed frame was applied (never waits: counted when the ring is full).
    pub fn report(&mut self, outcome: TargetOutcome) {
        if self.outcomes.push(outcome).is_err() { self.outcomes_dropped += 1; }
    }

    /// Outcomes dropped on a full outcome ring.
    pub fn outcomes_dropped(&self) -> u64 { self.outcomes_dropped }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(stream: u64, seq: u64, replaceable: bool) -> TargetFrame {
        let now = Instant::now();
        TargetFrame { arm: ArmId::LOCAL, boot: BootId(1), stream: StreamId(stream), seq, authority: Authority::default(), policy_generation: 0,
            input_at: now, vision_at: None, vision_frame_id: None, deadline: now + Duration::from_secs(5), dof: 2, q: [seq as f32; MAX_DOF], replaceable }
    }

    #[test]
    fn a_burst_coalesces_to_the_newest_frame() {
        let (mut w, mut r) = target_mailbox();
        assert_eq!(w.submit(frame(1, 1, true)), Submitted::Accepted);
        assert_eq!(w.submit(frame(1, 2, true)), Submitted::Coalesced { replaced: 1 });
        assert_eq!(w.submit(frame(1, 3, true)), Submitted::Coalesced { replaced: 2 });
        let f = r.take().unwrap();
        assert_eq!((f.seq, f.positions()), (3, &[3.0, 3.0][..]));
        assert_eq!(r.take(), None, "consumed once");
        assert_eq!(w.submit(frame(1, 4, true)), Submitted::Accepted);
        assert_eq!(w.coalesced(), 2);
    }

    #[test]
    fn duplicates_and_wrapped_sequences_are_refused() {
        let (mut w, mut r) = target_mailbox();
        w.submit(frame(1, u64::MAX - 1, true));
        assert_eq!(w.submit(frame(1, u64::MAX - 1, true)), Submitted::OutOfOrder);
        assert_eq!(w.submit(frame(1, 0, true)), Submitted::OutOfOrder, "wrapped");
        assert_eq!(r.take().unwrap().seq, u64::MAX - 1);
        // another stream starts its own sequence
        assert_eq!(w.submit(frame(2, 0, true)), Submitted::Accepted);
    }

    #[test]
    fn an_unconsumed_non_replaceable_frame_is_kept() {
        let (mut w, mut r) = target_mailbox();
        w.submit(frame(1, 1, false));
        assert_eq!(w.submit(frame(1, 2, true)), Submitted::Busy);
        assert_eq!(r.take().unwrap().seq, 1);
        assert_eq!(w.submit(frame(1, 2, true)), Submitted::Accepted);
    }

    #[test]
    fn delayed_consumption_reads_the_newest_frame_and_times_are_kept() {
        let (mut w, mut r) = target_mailbox();
        let mut f = frame(1, 1, true);
        let t = Instant::now() - std::time::Duration::from_millis(30);
        f.input_at = t;
        f.vision_at = Some(t);
        w.submit(f);
        std::thread::sleep(std::time::Duration::from_millis(2));
        let got = r.take().unwrap();
        assert_eq!((got.input_at, got.vision_at), (t, Some(t)));
    }

    #[test]
    fn each_stream_keeps_its_own_sequence() {
        let (mut w, mut r) = target_mailbox();
        assert_eq!(w.submit(frame(1, 10, true)), Submitted::Accepted);
        r.take();
        assert_eq!(w.submit(frame(2, 1, true)), Submitted::Accepted);
        r.take();
        assert_eq!(w.submit(frame(1, 9, true)), Submitted::OutOfOrder, "stale A:9 after A:10, B:1");
        assert_eq!(w.submit(frame(1, 11, true)), Submitted::Accepted);
        // bounded: a ninth stream waits until one closes
        for k in 3..=8 { r.take(); assert_eq!(w.submit(frame(k, 1, true)), Submitted::Accepted); }
        r.take();
        assert_eq!(w.submit(frame(9, 1, true)), Submitted::StreamsFull);
        w.close_stream(StreamId(2));
        assert_eq!(w.submit(frame(9, 1, true)), Submitted::Accepted);
    }

    #[test]
    fn a_frame_taken_concurrently_is_never_reported_coalesced() {
        let (mut w, mut r) = target_mailbox();
        let reader = std::thread::spawn(move || {
            let mut got = Vec::new();
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(2) {
                if let Some(f) = r.take() { got.push(f.seq); if f.seq == 20_000 { break; } }
            }
            got
        });
        let mut coalesced = Vec::new();
        for seq in 1..=20_000u64 {
            if let Submitted::Coalesced { replaced } = w.submit(frame(1, seq, true)) { coalesced.push(replaced); }
        }
        let got = reader.join().unwrap();
        assert!(got.windows(2).all(|p| p[0] < p[1]), "delivered in order, once");
        for c in &coalesced { assert!(got.binary_search(c).is_err(), "frame {c} reported coalesced but delivered"); }
        assert_eq!(coalesced.len() + got.len(), 20_000, "every frame either delivered or coalesced");
    }

    #[test]
    fn consumption_checks_identity_deadline_stream_policy_and_ages() {
        let now = Instant::now();
        let a = Authority::default();
        let ta = TargetAuthority { stream: Some(StreamId(1)), policy_generation: 3, max_input_age: Some(Duration::from_millis(50)), max_vision_age: Some(Duration::from_millis(100)) };
        let ok = TargetFrame { policy_generation: 3, vision_at: Some(now), ..frame(1, 1, true) };
        assert_eq!(ok.check(ArmId::LOCAL, BootId(1), now, &a, &ta), None);
        assert_eq!(ok.check(ArmId(1), BootId(1), now, &a, &ta), Some("WRONG_TARGET"));
        assert_eq!(ok.check(ArmId::LOCAL, BootId(2), now, &a, &ta), Some("WRONG_TARGET"));
        let zero_boot = TargetFrame { boot: BootId(0), ..ok };
        assert_eq!(zero_boot.check(ArmId::LOCAL, BootId(0), now, &a, &ta), Some("WRONG_TARGET"));
        assert_eq!(ok.check(ArmId::LOCAL, BootId(1), now, &Authority { stop_generation: 1, ..a }, &ta), Some("STOPPED"));
        assert_eq!(ok.check(ArmId::LOCAL, BootId(1), ok.deadline + Duration::from_millis(1), &a, &ta), Some("DEADLINE_EXPIRED"));
        assert_eq!(ok.check(ArmId::LOCAL, BootId(1), now, &Authority { epoch: 1, ..a }, &ta), Some("EPOCH_MISMATCH"));
        assert_eq!(ok.check(ArmId::LOCAL, BootId(1), now, &a, &TargetAuthority { stream: Some(StreamId(2)), ..ta }), Some("STREAM_NOT_AUTHORIZED"));
        assert_eq!(ok.check(ArmId::LOCAL, BootId(1), now, &a, &TargetAuthority { policy_generation: 4, ..ta }), Some("POLICY_REVOKED"));
        assert_eq!(ok.check(ArmId::LOCAL, BootId(1), now + Duration::from_millis(60), &a, &ta), Some("INPUT_STALE"));
        let old_vision = TargetFrame { vision_at: Some(now - Duration::from_millis(150)), ..ok };
        assert_eq!(old_vision.check(ArmId::LOCAL, BootId(1), now, &a, &ta), Some("VISION_STALE"));
        let big = TargetFrame { dof: MAX_DOF as u8 + 1, ..ok };
        assert_eq!(big.check(ArmId::LOCAL, BootId(1), now, &a, &ta), Some("INVALID_COMMAND"));
    }
}
