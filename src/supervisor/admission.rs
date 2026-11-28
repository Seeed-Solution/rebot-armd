//! Admission stamps, single-use proof tokens, connection registries, and stop/gate helpers.

#[allow(unused_imports)]
use super::*;

/// Global monotonic allocator for single-use [`AdmissionStamp::request_id`] values.
pub(super) static NEXT_ADMISSION_REQUEST_ID: AtomicU64 = AtomicU64::new(1);

/// High bit reserving an isolated single-use namespace for RT broker `RequestId` values so
/// per-broker monotonic IDs never collide with global [`next_admission_request_id`] IDs.
pub(super) const BROKER_REQUEST_ID_BIT: u64 = 1u64 << 63;

/// High bit reserving an isolated single-use namespace for RT-internal latest-target and
/// servo-stream authorization stamps so they never collide with caller or `LeaseInstallCtx` IDs.
pub(super) const RT_INTERNAL_REQUEST_ID_BIT: u64 = 1u64 << 62;

/// Allocates a fresh non-zero, single-use admission request ID (lock-free, zero-alloc).
#[inline]
pub fn next_admission_request_id() -> u64 {
    let id = NEXT_ADMISSION_REQUEST_ID.fetch_add(1, Ordering::Relaxed);
    assert!(
        id != 0 && (id & (BROKER_REQUEST_ID_BIT | RT_INTERNAL_REQUEST_ID_BIT)) == 0,
        "admission request_id overflow"
    );
    id
}

/// Generation snapshot captured ONCE at request admission (lock-free in the WS reader
/// before queuing, or at REST/caller entry) and required by every supervisor/owner function
/// that starts motion or output.
///
/// Although `AdmissionStamp` is [`Copy`] so callers can carry it across off-lock planning,
/// each stamp carries a unique non-zero [`request_id`](Self::request_id) consumed once by
/// [`SupervisorInner::check_admission`] under the owner lock; any second use is refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdmissionStamp {
    pub stop_gen: u64,
    pub estop_epoch: u64,
    pub lease_gen: u64,
    /// Monotonic WS connection ID (`>= 1`) captured at WS admission, or `0` for non-WS callers.
    /// Checked against the owner-lock closed-connection set so a stale stamp from a closed
    /// connection is always refused.
    pub connection_id: u64,
    /// Globally unique non-zero request ID consumed once by [`SupervisorInner::check_admission`].
    /// A zero or already-consumed `request_id` is refused fail-closed.
    pub request_id: u64,
}

/// Opaque, single-use admission proof token created ONLY by [`SupervisorInner::check_admission`]
/// under the owner lock and borrowing the owner lock guard for `'g`.
///
/// It cannot be constructed externally, cloned, copied, sent across threads/`.await` points,
/// stored past the guard lifetime `'g`, or held after dropping the owner lock guard, and every
/// function that starts motion or output consumes it by value:
///
/// ```compile_fail
/// use armd::{AdmissionStamp, Admitted};
/// let _forged = Admitted(AdmissionStamp { stop_gen: 0, estop_epoch: 1, lease_gen: 0, connection_id: 0, request_id: 1 });
/// ```
///
/// ```compile_fail
/// fn assert_copy<T: Copy>() {}
/// assert_copy::<armd::Admitted<'_>>();
/// ```
///
/// ```compile_fail
/// fn assert_clone<T: Clone>() {}
/// assert_clone::<armd::Admitted<'_>>();
/// ```
///
/// ```compile_fail
/// fn assert_send<T: Send>() {}
/// assert_send::<armd::Admitted<'_>>();
/// ```
///
/// Holding [`Admitted`] after dropping the owner lock guard does not compile:
///
/// ```compile_fail,E0505
/// use armd::{ArmSupervisor, SupervisorInner};
/// async fn cannot_drop_guard_while_holding_admitted(sup: &ArmSupervisor) {
///     let stamp = sup.capture_admission_stamp();
///     let mut guard = sup.inner().write().await;
///     let admitted = guard.check_admission(stamp).unwrap();
///     drop(guard);
///     let _ = SupervisorInner::enable_torque(admitted);
/// }
/// ```
///
/// ```compile_fail,E0597
/// use armd::{ArmSupervisor, SupervisorInner};
/// async fn cannot_return_admitted_past_guard_scope(sup: &ArmSupervisor) {
///     let stamp = sup.capture_admission_stamp();
///     let admitted = {
///         let mut guard = sup.inner().write().await;
///         guard.check_admission(stamp).unwrap()
///     };
///     let _ = SupervisorInner::enable_torque(admitted);
/// }
/// ```
///
/// Mutating the owner state while holding [`Admitted`] or storing it in `'static` does not compile:
///
/// ```compile_fail,E0499
/// use armd::{ArmConfig, SupervisorInner};
/// let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
/// let stamp = inner.current_admission_stamp();
/// let admitted = inner.check_admission(stamp).unwrap();
/// let _ = inner.authorized_stop();
/// let _ = SupervisorInner::enable_torque(admitted);
/// ```
///
/// ```compile_fail
/// fn assert_static<T: 'static>() {}
/// fn reject_static<'a>() { assert_static::<armd::Admitted<'a>>(); }
/// ```
///
/// Calling a motion or output start without an [`Admitted`] token does not compile:
///
/// ```compile_fail,E0061
/// use armd::SupervisorInner;
/// let _ = SupervisorInner::jog(vec![0.0; 7], None, None);
/// ```
///
/// ```compile_fail,E0061
/// use armd::SupervisorInner;
/// let _ = SupervisorInner::enable_torque();
/// ```
///
/// ```compile_fail,E0061
/// use armd::motion::{GripperCommand, GripperRequest};
/// use armd::SupervisorInner;
/// let req = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: None, tag: None };
/// let _ = SupervisorInner::start_gripper_with_baseline(req, 1, armd::supervisor::GripperBaselineCmd::None);
/// ```
///
/// ```compile_fail,E0061
/// use armd::motion::{GripperCommand, GripperRequest, LearnMode};
/// use armd::SupervisorInner;
/// let req = GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 1.0, timeout_s: None, tag: None };
/// let _ = SupervisorInner::start_macro_gripper_learning(1, req, LearnMode::Calibration);
/// ```
///
/// ```compile_fail,E0061
/// use std::sync::Arc;
/// use armd::{ArmConfig, ArmSupervisor};
/// let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
/// let _ = armd::gripper_calib::begin_calibration(&sup, 1, None);
/// ```
///
/// ```compile_fail,E0061
/// use std::sync::Arc;
/// use armd::{ArmConfig, ArmSupervisor};
/// let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
/// let _ = armd::gripper_calib::start_calibration(&sup);
/// ```
#[must_use = "Admitted token must be consumed by a motion/output start"]
pub struct Admitted<'g> {
    pub(super) inner: &'g mut SupervisorInner,
    pub(super) stamp: AdmissionStamp,
    pub(super) _not_send_sync: std::marker::PhantomData<*mut ()>,
}

/// Crate-private, non-`Copy`, non-`Clone` witness proving a live [`Admitted`] token was
/// verified and consumed inside `armd`. Only [`Admitted::consume`] can construct it.
pub(crate) struct AdmittedProof {
    stamp: AdmissionStamp,
}

/// Complete catalog of motion/output start entry points gated by [`AdmissionStamp`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotionStartKind {
    Jog,
    Home,
    Park,
    Preset,
    MoveTo,
    MoveRelative,
    MoveToPose,
    Place,
    Grasp,
    Replay,
    Sequence,
    CalibHover,
    MeasureMultiview,
    MacroStart,
    MacroChildPath,
    MacroChildQuintic,
    MacroChildGripper,
    MacroChildReplay,
    Servo,
    PolicyShadow,
    PolicyLive,
    GravityComp,
    Gripper,
    GripperCalibrate,
    GripperZeroCheck,
    TorqueEnable,
}

/// Single sealed execution contract for [`SupervisorInner::start_motion`]: an operation can only
/// run when handed a single-use [`Admitted`] token produced by [`SupervisorInner::check_admission`]
/// under the owner lock.
///
/// ```compile_fail,E0061
/// use armd::SupervisorInner;
/// // Calling start_motion without an Admitted token fails to compile:
/// let _ = SupervisorInner::start_motion(());
/// ```
pub trait MotionStartOp {
    type Output;
    fn kind(&self) -> MotionStartKind;
    fn execute(self, admitted: Admitted<'_>) -> Result<Self::Output, ArmError>;
}

#[derive(Debug, Error)]
pub enum ArmError {
    #[error("Safety error: {0}")]
    Safety(#[from] SafetyError),

    #[error("Trajectory error: {0}")]
    Trajectory(#[from] TrajectoryError),

    #[error("Recording error: {0}")]
    Recording(#[from] RecordingError),

    #[error("Unsupported operation: {0}")]
    Unsupported(String),

    #[error("Motion error: {0}")]
    Motion(#[from] MotionError),

    #[error("Internal error: {0}")]
    Internal(String),

    /// Request refused with a specific code (gravity compensation mode changes).
    #[error("{code}: {message}")]
    Rejected { status: u16, code: String, message: String },

    /// Zero-allocation static refusal for RT paths (`armd-control` never allocates `String`).
    #[error("{code}: {message}")]
    RejectedStatic { status: u16, code: &'static str, message: &'static str },

    /// [`Rejected`](Self::Rejected) with extra reply fields (`owner`, `sequences`, `step`, `reason`).
    #[error("{code}: {message}")]
    Detailed { status: u16, code: String, message: String, details: Box<serde_json::Value> },

    /// Workspace rejection preserving structured violation detail (e.g. replay).
    #[error("WORKSPACE_VIOLATION: {message}")]
    WorkspaceRejected {
        message: String,
        detail: Box<crate::kinematics::WorkspaceViolationDetail>,
    },

    /// Config write expected_revision does not match the current revision.
    #[error("Policy revision mismatch: expected {expected}, current {current}")]
    PolicyRevisionMismatch { expected: u64, current: u64 },
}

#[derive(Debug, thiserror::Error, PartialEq, Eq, Clone)]
pub enum PreemptError {
    #[error("Preemption cooldown active: {0}ms remaining")]
    Cooldown(u64),
    #[error("OWNER_TRANSFERRED: control ownership transferred to RT executor; shell lease calls are refused")]
    OwnerTransferred,
}

/// Maximum concurrent live WS connections tracked by the owner (fail-closed when full).
pub const MAX_LIVE_CONNECTIONS: usize = 64;

/// Fixed-capacity, zero-allocation live WS connection registry (`>= 1`).
#[derive(Debug, Clone, Copy)]
pub(crate) struct LiveConnections {
    pub(super) ids: [u64; MAX_LIVE_CONNECTIONS],
    pub(super) len: usize,
}

/// Preallocated capacity for [`ClosedConnections`] intervals: with at most `MAX_LIVE_CONNECTIONS`
/// concurrent open connections, pigeonhole compaction guarantees `intervals.len() <= MAX_LIVE_CONNECTIONS + 1`
/// forever without heap growth on RT.
const CLOSED_CONNECTIONS_PREALLOC: usize = MAX_LIVE_CONNECTIONS + 4;

/// Coalesced sorted inclusive intervals `(lo, hi)` of closed WS connection IDs (`>= 1`) above a
/// monotonic `watermark` (`<= watermark` is always closed). Bounded by `MAX_LIVE_CONNECTIONS + 1`
/// via live-aware pigeonhole compaction with zero RT allocation and O(log K) lookup.
#[derive(Debug)]
pub(crate) struct ClosedConnections {
    watermark: u64,
    intervals: Vec<(u64, u64)>,
}

/// Borrowed snapshot of owner-side live disconnect fence & E-stop state checked immediately before
/// transmitting gravity-comp targets (shared by periodic and immediate write paths).
#[derive(Clone, Copy)]
pub(crate) struct LiveDisconnectFenceView<'a> {
    e_stop_latched: &'a crate::rt_core::urgent::EstopLatchView,
    pending_disconnect_hold: &'a AtomicU64,
    atomic_fenced_lease_gen: &'a AtomicU64,
    fenced_lease_generation: u64,
    active_lease_conn_id: u64,
    active_lease_gen: Option<u64>,
    closed_connections: &'a ClosedConnections,
    rt_closed_connections: Option<&'a crate::rt_core::requests::actions::ClosedBitset>,
    shared_closed_bitset: &'a crate::rt_core::requests::actions::ClosedBitset,
}

/// Preallocated capacity for [`ConsumedStamps`] disjoint intervals above `watermark`.
const CONSUMED_STAMPS_INTERVAL_CAP: usize = 256;

/// Bounded, zero-allocation-in-steady-state set of consumed single-use admission request IDs
/// (`>= 1`), stored as a monotonic `watermark` (every `id <= watermark` is treated as consumed,
/// fail-closed) plus up to [`CONSUMED_STAMPS_INTERVAL_CAP`] sorted coalesced inclusive
/// intervals `(lo, hi)` above `watermark`.
#[derive(Debug)]
pub(crate) struct ConsumedStamps {
    watermark: u64,
    intervals: Vec<(u64, u64)>,
}


impl AdmissionStamp {
    /// Derives a fresh single-use child [`AdmissionStamp`] for a macro sub-motion while
    /// preserving the parent's admission-captured `stop_gen`, `estop_epoch`, `lease_gen`, and
    /// `connection_id` verbatim (never re-sampling live owner authority).
    pub fn for_macro_child(self) -> Self {
        Self {
            request_id: if self.request_id == 0 {
                0
            } else {
                next_admission_request_id()
            },
            ..self
        }
    }

    /// Constructs a single-use [`AdmissionStamp`] for an RT broker request using its broker
    /// `RequestId` in an isolated namespace (zero-alloc; `0` stays `0` and is refused fail-closed).
    pub(crate) fn from_broker(
        stop_gen: u64,
        estop_epoch: u64,
        lease_gen: u64,
        broker_request_id: u64,
    ) -> Self {
        Self::from_broker_conn(stop_gen, estop_epoch, lease_gen, 0, broker_request_id)
    }

    /// Constructs a single-use [`AdmissionStamp`] for an RT broker request carrying the original
    /// WS `connection_id` (`ClientId.0`, or `0` for non-WS callers) so transition authority and
    /// closed-connection checks bind directly to the submitting connection on `armd-control`.
    pub(crate) fn from_broker_conn(
        stop_gen: u64,
        estop_epoch: u64,
        lease_gen: u64,
        connection_id: u64,
        broker_request_id: u64,
    ) -> Self {
        Self {
            stop_gen,
            estop_epoch,
            lease_gen,
            connection_id,
            request_id: if broker_request_id == 0 {
                0
            } else {
                broker_request_id | BROKER_REQUEST_ID_BIT
            },
        }
    }

    /// Constructs a fresh single-use [`AdmissionStamp`] for an RT-internal latest-target or
    /// stream-handback step (zero-alloc atomic counter in an isolated namespace).
    pub(crate) fn for_rt_internal(stop_gen: u64, estop_epoch: u64, lease_gen: u64) -> Self {
        Self {
            stop_gen,
            estop_epoch,
            lease_gen,
            connection_id: 0,
            request_id: next_admission_request_id() | RT_INTERNAL_REQUEST_ID_BIT,
        }
    }
}

impl std::fmt::Debug for Admitted<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Admitted").field("stamp", &self.stamp).finish()
    }
}

impl AdmittedProof {
    #[inline]
    pub(crate) fn stamp(&self) -> AdmissionStamp {
        self.stamp
    }
}

impl<'g> Admitted<'g> {
    /// The validated stamp captured at request admission.
    #[inline]
    pub fn stamp(&self) -> AdmissionStamp {
        self.stamp
    }

    /// Re-verifies this token against live owner state (including lock-free atomic E-stop
    /// and urgent-stop counters) without consuming it.
    pub(crate) fn verify_live(&self) -> Result<AdmissionStamp, SafetyError> {
        self.inner.verify_admission_stamp(self.stamp)?;
        Ok(self.stamp)
    }

    /// Consumes this single-use token by value, re-verifies its stamp against live owner
    /// state, and yields `&'g mut SupervisorInner` plus a crate-private [`AdmittedProof`].
    pub(crate) fn consume(self) -> Result<(&'g mut SupervisorInner, AdmittedProof), SafetyError> {
        let stamp = self.stamp;
        let inner = self.inner;
        inner.verify_admission_stamp(stamp)?;
        // Record the A1 admission `(stop_gen, estop_epoch)` directly from the admitted command's
        // stamp so J7 `Target` intents carry their admission stamp and are never re-stamped later
        // (and never stamped with `max(local, requested)`).
        inner.j7_admitted_stamp = Some(crate::driver::PosForceIntentStamp {
            stop_gen: stamp.stop_gen,
            estop_epoch: stamp.estop_epoch,
            lease_gen: stamp.lease_gen,
        });
        Ok((inner, AdmittedProof { stamp }))
    }
}

impl MotionStartKind {
    pub const ALL: &'static [MotionStartKind] = &[
        Self::Jog,
        Self::Home,
        Self::Park,
        Self::Preset,
        Self::MoveTo,
        Self::MoveRelative,
        Self::MoveToPose,
        Self::Place,
        Self::Grasp,
        Self::Replay,
        Self::Sequence,
        Self::CalibHover,
        Self::MeasureMultiview,
        Self::MacroStart,
        Self::MacroChildPath,
        Self::MacroChildQuintic,
        Self::MacroChildGripper,
        Self::MacroChildReplay,
        Self::Servo,
        Self::PolicyShadow,
        Self::PolicyLive,
        Self::GravityComp,
        Self::Gripper,
        Self::GripperCalibrate,
        Self::GripperZeroCheck,
        Self::TorqueEnable,
    ];
}

impl ArmError {
    pub fn detailed(status: u16, code: &str, message: impl Into<String>, details: serde_json::Value) -> Self {
        Self::Detailed { status, code: code.into(), message: message.into(), details: Box::new(details) }
    }

    pub fn workspace_detail(&self) -> Option<crate::kinematics::WorkspaceViolationDetail> {
        match self {
            Self::Safety(e) => e.workspace_detail(),
            Self::Motion(MotionError::WorkspaceViolation { detail, .. }) => Some((**detail).clone()),
            Self::WorkspaceRejected { detail, .. } => Some((**detail).clone()),
            _ => None,
        }
    }
}

impl From<crate::recording::StoreError> for ArmError {
    fn from(e: crate::recording::StoreError) -> Self {
        let (status, code) = e.status_code();
        ArmError::Rejected { status, code: code.into(), message: e.to_string() }
    }
}

impl Default for LiveConnections {
    fn default() -> Self {
        Self::new()
    }
}

impl LiveConnections {
    pub(crate) const fn new() -> Self {
        Self {
            ids: [0; MAX_LIVE_CONNECTIONS],
            len: 0,
        }
    }

    pub(crate) fn register(
        &mut self,
        id: u64,
        closed: &ClosedConnections,
    ) -> Result<(), &'static str> {
        if id == 0 {
            return Err("INVALID_CONNECTION");
        }
        if closed.is_closed(id) {
            return Err("DISCONNECTED");
        }
        if self.contains(id) {
            return Ok(());
        }
        if self.len >= MAX_LIVE_CONNECTIONS {
            return Err("TOO_MANY_CONNECTIONS");
        }
        self.ids[self.len] = id;
        self.len += 1;
        Ok(())
    }

    pub(crate) fn remove(&mut self, id: u64) -> bool {
        if id == 0 {
            return false;
        }
        for i in 0..self.len {
            if self.ids[i] == id {
                self.len -= 1;
                self.ids[i] = self.ids[self.len];
                self.ids[self.len] = 0;
                return true;
            }
        }
        false
    }

    #[inline]
    pub(crate) fn contains(&self, id: u64) -> bool {
        id != 0 && self.ids[..self.len].contains(&id)
    }

    /// True iff any currently live connection ID lies strictly inside `(lo, hi)`.
    #[inline]
    pub(crate) fn any_in_open_range(&self, lo: u64, hi: u64) -> bool {
        self.ids[..self.len].iter().any(|&id| id > lo && id < hi)
    }

    #[inline]
    pub(crate) fn len(&self) -> usize {
        self.len
    }
}

impl Default for ClosedConnections {
    fn default() -> Self {
        Self::new()
    }
}

impl ClosedConnections {
    pub(crate) fn new() -> Self {
        Self {
            watermark: 0,
            intervals: Vec::with_capacity(CLOSED_CONNECTIONS_PREALLOC),
        }
    }

    pub(crate) fn reserve_rt(&mut self) {
        if self.intervals.capacity() < CLOSED_CONNECTIONS_PREALLOC {
            self.intervals
                .reserve(CLOSED_CONNECTIONS_PREALLOC - self.intervals.len());
        }
    }

    pub(crate) fn mark_closed(&mut self, id: u64, live: &LiveConnections) {
        if id == 0 || id <= self.watermark {
            return;
        }
        let idx = self
            .intervals
            .partition_point(|&(_, end)| end.saturating_add(1) < id);
        if idx < self.intervals.len() && self.intervals[idx].0 <= id.saturating_add(1) {
            let (lo, hi) = self.intervals[idx];
            let new_lo = lo.min(id);
            let mut new_hi = hi.max(id);
            if idx + 1 < self.intervals.len()
                && self.intervals[idx + 1].0 <= new_hi.saturating_add(1)
            {
                new_hi = new_hi.max(self.intervals[idx + 1].1);
                self.intervals.remove(idx + 1);
            }
            self.intervals[idx] = (new_lo, new_hi);
        } else {
            // Ensure capacity BEFORE inserting: with at most `MAX_LIVE_CONNECTIONS` live IDs,
            // whenever `intervals.len() >= MAX_LIVE_CONNECTIONS + 1` the pigeonhole principle
            // guarantees either the prefix gap `(watermark, intervals[0].0)` or an adjacent gap
            // `(intervals[i].1, intervals[i+1].0)` contains zero live connection IDs.
            if self.intervals.len() >= CLOSED_CONNECTIONS_PREALLOC {
                self.compact_against_live(live);
            }
            let idx = self
                .intervals
                .partition_point(|&(_, end)| end.saturating_add(1) < id);
            if idx < self.intervals.len() && self.intervals[idx].0 <= id.saturating_add(1) {
                let (lo, hi) = self.intervals[idx];
                self.intervals[idx] = (lo.min(id), hi.max(id));
            } else {
                self.intervals.insert(idx, (id, id));
            }
        }
        // Collapse any contiguous prefix directly into `watermark`.
        while let Some(&(lo, hi)) = self.intervals.first() {
            if lo <= self.watermark.saturating_add(1) {
                self.watermark = self.watermark.max(hi);
                self.intervals.remove(0);
            } else {
                break;
            }
        }
        // Keep interval count strictly <= MAX_LIVE_CONNECTIONS + 1 even under external ID gaps.
        if self.intervals.len() > MAX_LIVE_CONNECTIONS + 1 {
            self.compact_against_live(live);
        }
    }

    fn compact_against_live(&mut self, live: &LiveConnections) {
        // First fold the lowest interval into `watermark` if no live ID sits below it.
        if let Some(&(lo, hi)) = self.intervals.first() {
            if !live.any_in_open_range(self.watermark, lo) {
                self.watermark = self.watermark.max(hi);
                self.intervals.remove(0);
                return;
            }
        }
        // Otherwise merge the first adjacent pair whose gap contains zero live IDs.
        for i in 0..self.intervals.len().saturating_sub(1) {
            let (lo0, hi0) = self.intervals[i];
            let (lo1, hi1) = self.intervals[i + 1];
            if !live.any_in_open_range(hi0, lo1) {
                self.intervals[i] = (lo0, hi1);
                self.intervals.remove(i + 1);
                return;
            }
        }
    }

    pub(crate) fn is_closed(&self, id: u64) -> bool {
        if id == 0 {
            return false;
        }
        if id <= self.watermark {
            return true;
        }
        let idx = self.intervals.partition_point(|&(_, end)| end < id);
        idx < self.intervals.len() && self.intervals[idx].0 <= id
    }
}

impl<'a> LiveDisconnectFenceView<'a> {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        e_stop_latched: &'a crate::rt_core::urgent::EstopLatchView,
        pending_disconnect_hold: &'a AtomicU64,
        atomic_fenced_lease_gen: &'a AtomicU64,
        fenced_lease_generation: u64,
        active_lease_conn_id: u64,
        active_lease_gen: Option<u64>,
        closed_connections: &'a ClosedConnections,
        rt_closed_connections: Option<&'a crate::rt_core::requests::actions::ClosedBitset>,
        shared_closed_bitset: &'a crate::rt_core::requests::actions::ClosedBitset,
    ) -> Self {
        Self {
            e_stop_latched,
            pending_disconnect_hold,
            atomic_fenced_lease_gen,
            fenced_lease_generation,
            active_lease_conn_id,
            active_lease_gen,
            closed_connections,
            rt_closed_connections,
            shared_closed_bitset,
        }
    }

    pub(crate) fn estop_latched(&self) -> bool {
        self.e_stop_latched.load(Ordering::SeqCst)
    }

    pub(crate) fn blocks_non_hold(&self) -> bool {
        if self.pending_disconnect_hold.load(Ordering::SeqCst) > 0 {
            return true;
        }
        if self.atomic_fenced_lease_gen.load(Ordering::SeqCst) > self.fenced_lease_generation {
            return true;
        }
        let conn_closed = self.active_lease_conn_id != 0
            && (self.closed_connections.is_closed(self.active_lease_conn_id)
                || self
                    .rt_closed_connections
                    .is_some_and(|b| b.is_closed(self.active_lease_conn_id))
                || self
                    .shared_closed_bitset
                    .is_closed(self.active_lease_conn_id));
        conn_closed && self.active_lease_gen.is_some_and(|g| g > self.fenced_lease_generation)
    }
}

impl ConsumedStamps {
    pub(crate) fn new() -> Self {
        Self {
            watermark: 0,
            intervals: Vec::with_capacity(CONSUMED_STAMPS_INTERVAL_CAP),
        }
    }

    pub(crate) fn contains(&self, id: u64) -> bool {
        if id == 0 || id <= self.watermark {
            return true;
        }
        let idx = self.intervals.partition_point(|&(_, end)| end < id);
        idx < self.intervals.len() && self.intervals[idx].0 <= id
    }

    pub(crate) fn record(&mut self, id: u64) {
        if id == 0 || id <= self.watermark {
            return;
        }
        if id == self.watermark.saturating_add(1) {
            self.watermark = id;
            self.fold_contiguous_into_watermark();
            return;
        }
        let mut idx = self
            .intervals
            .partition_point(|&(_, end)| end.saturating_add(1) < id);
        if idx < self.intervals.len() && self.intervals[idx].0 <= id.saturating_add(1) {
            let (lo, hi) = self.intervals[idx];
            let new_lo = lo.min(id);
            let mut new_hi = hi.max(id);
            if idx + 1 < self.intervals.len()
                && self.intervals[idx + 1].0 <= new_hi.saturating_add(1)
            {
                new_hi = new_hi.max(self.intervals[idx + 1].1);
                self.intervals.remove(idx + 1);
            }
            self.intervals[idx] = (new_lo, new_hi);
            self.fold_contiguous_into_watermark();
            return;
        }
        // Capacity full: fold oldest range into `watermark` fail-closed (zero allocation,
        // never forgets any previously consumed ID).
        if self.intervals.len() >= CONSUMED_STAMPS_INTERVAL_CAP {
            if idx == 0 {
                self.watermark = self.watermark.max(id);
                self.fold_contiguous_into_watermark();
                return;
            }
            let (_, oldest_hi) = self.intervals.remove(0);
            self.watermark = self.watermark.max(oldest_hi);
            self.fold_contiguous_into_watermark();
            // Re-locate insertion point after folding into watermark.
            if id <= self.watermark {
                return;
            }
            if id == self.watermark.saturating_add(1) {
                self.watermark = id;
                self.fold_contiguous_into_watermark();
                return;
            }
            idx = self
                .intervals
                .partition_point(|&(_, end)| end.saturating_add(1) < id);
        }
        self.intervals.insert(idx, (id, id));
        self.fold_contiguous_into_watermark();
    }

    fn fold_contiguous_into_watermark(&mut self) {
        while let Some(&(lo, hi)) = self.intervals.first() {
            if lo <= self.watermark.saturating_add(1) {
                self.watermark = self.watermark.max(hi);
                self.intervals.remove(0);
            } else {
                break;
            }
        }
    }
}

impl SupervisorInner {
    /// Advance the stop generation under the owner lock whenever a Stop applies.
    pub fn advance_stop_generation(&mut self) -> u64 {
        self.reset_hold_tx_joint_progress();
        self.j7_admitted_stamp = None;
        self.stop_gen.fetch_add(1, Ordering::SeqCst) + 1
    }

    /// Monotonically synchronize `stop_gen` with an RT executor's `CommitCtx::stop_generation`.
    pub(crate) fn sync_stop_generation(&mut self, gen: u64) {
        let _ = self.stop_gen.fetch_max(gen, Ordering::SeqCst);
    }

    /// Keep `lease_gen_mirror` synchronized with `active_lease` under the owner lock.
    pub(crate) fn sync_lease_gen_mirror(&self) {
        self.lease_gen_mirror.store(self.current_active_lease_gen(), Ordering::SeqCst);
    }

    pub(super) fn verify_admission_stamp(&self, stamp: AdmissionStamp) -> Result<(), SafetyError> {
        if stamp.connection_id != 0 && self.is_connection_closed(stamp.connection_id) {
            return Err(SafetyError::LeaseLost);
        }
        if stamp.stop_gen != self.current_stop_gen() {
            return Err(SafetyError::Stopped);
        }
        let active_gen = self.current_active_lease_gen();
        if stamp.lease_gen != active_gen {
            return Err(SafetyError::LeaseLost);
        }
        let current_epoch = self.safety.epoch_id();
        if stamp.estop_epoch != current_epoch {
            if self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop)
            {
                return Err(SafetyError::EStopActive);
            }
            return Err(SafetyError::EpochMismatch {
                expected: current_epoch,
                received: stamp.estop_epoch,
            });
        }
        if self.internal_fault_lane.is_fenced() {
            return Err(SafetyError::EStopActive);
        }
        if self.e_stop_latched.load(Ordering::SeqCst)
            || matches!(self.safety.state(), SafetyState::EStop)
        {
            return Err(SafetyError::EStopActive);
        }
        Ok(())
    }

    pub(crate) fn is_stamp_consumed(&self, request_id: u64) -> bool {
        if request_id == 0 {
            return true;
        }
        if (request_id & BROKER_REQUEST_ID_BIT) != 0 {
            self.consumed_broker_stamps
                .contains(request_id & !BROKER_REQUEST_ID_BIT)
        } else if (request_id & RT_INTERNAL_REQUEST_ID_BIT) != 0 {
            self.consumed_internal_stamps
                .contains(request_id & !RT_INTERNAL_REQUEST_ID_BIT)
        } else {
            self.consumed_stamps.contains(request_id)
        }
    }

    pub(crate) fn record_consumed_stamp(&mut self, request_id: u64) {
        if request_id == 0 {
            return;
        }
        if (request_id & BROKER_REQUEST_ID_BIT) != 0 {
            self.consumed_broker_stamps
                .record(request_id & !BROKER_REQUEST_ID_BIT);
        } else if (request_id & RT_INTERNAL_REQUEST_ID_BIT) != 0 {
            self.consumed_internal_stamps
                .record(request_id & !RT_INTERNAL_REQUEST_ID_BIT);
        } else {
            self.consumed_stamps.record(request_id);
        }
    }

    /// Read-only preflight admission check used before off-lock planning or non-installing
    /// validation: verifies authority and confirms `stamp.request_id` is non-zero and not yet
    /// consumed, WITHOUT consuming it.
    pub fn preflight_admission(&self, stamp: AdmissionStamp) -> Result<(), SafetyError> {
        if self.is_stamp_consumed(stamp.request_id) {
            return Err(SafetyError::StampAlreadyConsumed);
        }
        self.verify_admission_stamp(stamp)
    }

    /// Single owner-lock motion/output admission gate: the SOLE constructor of [`Admitted`].
    /// Compares the caller's admission-captured [`AdmissionStamp`] against current owner state
    /// without re-sampling, consumes its single-use [`AdmissionStamp::request_id`] under the
    /// owner lock, and returns an [`Admitted<'g>`] token borrowing the owner guard.
    pub fn check_admission<'g>(
        &'g mut self,
        stamp: AdmissionStamp,
    ) -> Result<Admitted<'g>, SafetyError> {
        if self.is_stamp_consumed(stamp.request_id) {
            return Err(SafetyError::StampAlreadyConsumed);
        }
        self.verify_admission_stamp(stamp)?;
        self.record_consumed_stamp(stamp.request_id);
        Ok(Admitted {
            inner: self,
            stamp,
            _not_send_sync: std::marker::PhantomData,
        })
    }

    /// Unified motion-start wrapper: requires an [`Admitted`] token by value under the owner lock.
    pub fn start_motion<Op: MotionStartOp>(admitted: Admitted<'_>, op: Op) -> Result<Op::Output, ArmError> {
        let verify_res = admitted.verify_live();
        #[cfg(not(target_arch = "wasm32"))]
        {
            let inner = &mut *admitted.inner;
            let dof = inner.config.dof.min(crate::torque::MAX_CHANNELS) as u8;
            let live_refusal = match &verify_res {
                Ok(_) => None,
                Err(SafetyError::EStopActive) => Some(crate::torque::shadow::LiveRefusalKind::EStopActive),
                Err(SafetyError::FaultActive(_)) => Some(crate::torque::shadow::LiveRefusalKind::FaultLatched),
                Err(_) => Some(crate::torque::shadow::LiveRefusalKind::Other),
            };
            inner
                .torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::MotionCommand {
                    channels: crate::torque::ChannelMask::from_count(dof),
                    live_allowed: verify_res.is_ok(),
                    live_refusal,
                    live_intent: inner.shadow_live_intent(),
                    live_is_torqued: inner.is_torqued,
                    live_stopped: inner.safety.state().is_stopped(),
                });
        }
        verify_res?;
        op.execute(admitted)
    }

    /// Observation schema: profile capabilities plus the parts that depend on the driver.
    pub fn schema(&self) -> ObservationSchema {
        let mut schema = ObservationSchema::from_config(&self.config);
        schema.capabilities.gravity_comp = self.gravity_comp_support().is_ok();
        let torque_feedback = self.driver.is_some() && self.driver_has_torque_feedback;
        schema.capabilities.collision_guard &= torque_feedback;
        schema.capabilities.torque_guard &= torque_feedback;
        schema.capabilities.power_budget &= torque_feedback;
        schema.capabilities.auto_yaw = self.config.arm_dof() == 5;
        #[cfg(feature = "policy-runtime")]
        if let Some(ref mut pr) = schema.capabilities.policy_runtime {
            pr.modes = if self.policy_live_allowed() {
                vec!["shadow".to_string(), "live_sim".to_string()]
            } else {
                vec!["shadow".to_string(), "live_gated".to_string()]
            };
        }
        if schema.feetech_joints.is_none() {
            schema.feetech_joints = self.config.feetech.as_ref().map(|f| f.joints.clone());
        }
        schema.capabilities.calibration_tools = Some(self.calibration_tools_with(&schema.capabilities));
        if let Some(t) = schema.capabilities.teach.as_mut() { t.episodes = self.episodes.enabled(); }
        schema.capabilities.cell = self.cell.as_ref().map(|c| c.capabilities());
        schema.joint_zero = self.calibration.as_ref().and_then(|c| c.joint_zero.as_ref()).and_then(|z| serde_json::to_value(z).ok());
        schema.zero_convention = self.config.zero_reference.as_ref().and_then(|_| serde_json::to_value(self.zero_convention).ok());
        schema
    }

    /// The E-stop and lease-fence gate of this arm's bus driver: a request not yet applied by the
    /// tick (plus authorized-stop generations when attached to an RT executor, and the lock-free
    /// fence-first disconnect lease atomics).
    pub fn estop_gate(&self) -> crate::rt_core::urgent::EstopGate {
        let gate = crate::rt_core::urgent::EstopGate::from_state(
            self.estop_state.clone(),
            self.estop_applied.clone(),
        )
        .with_owner_stop_gen(self.stop_gen.clone())
        .with_lease_gate(crate::rt_core::urgent::LeaseGate::new(
            self.lease_gen_mirror.clone(),
            self.atomic_fenced_lease_gen.clone(),
            self.pending_disconnect_hold.clone(),
        ));
        match self.stop_requests.as_ref() {
            Some(stops) => gate.with_stops(stops.clone()),
            None => gate,
        }
    }

    /// Current time on the attached driver's clock (or `Instant::now()` when driverless).
    pub(crate) fn driver_clock_now(&self) -> Instant {
        if self.rt_executor_owned {
            return self.driver_clock_now_or(Instant::now());
        }
        self.driver
            .as_ref()
            .and_then(|d| d.with_ref(|drv| drv.clock_now()))
            .unwrap_or_else(Instant::now)
    }

    /// Active disable intent carried by an in-flight or pending background disable, if any.
    pub fn active_disable_kind(&self) -> Option<crate::driver::DisableKind> {
        self.active_disable_kind
    }

    /// Driver-side Rule A′ gate injected into every attached hardware/sim driver so the
    /// transmit choke point enforces the stopped rest-pose rule at send time.
    pub fn stopped_disable_gate(&self) -> crate::driver::StoppedDisableGate {
        let mut arm_joints = [0usize; crate::rt_core::mailbox::MAX_DOF];
        let mut arm_joints_len = 0usize;
        for &j in self.config.arm_joints().iter() {
            if arm_joints_len < arm_joints.len() {
                arm_joints[arm_joints_len] = j;
                arm_joints_len += 1;
            }
        }
        let mut rest_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let rest = self.config.resting_positions();
        for (i, &v) in rest.iter().enumerate().take(rest_positions.len()) {
            rest_positions[i] = v;
        }
        crate::driver::StoppedDisableGate {
            safety_stopped: self.safety.stopped_flag(),
            has_resting_pose: self.config.resting_pose.is_some(),
            arm_joints,
            arm_joints_len,
            rest_positions,
            rest_pose_tolerance_rad: self.config.rest_pose_tolerance_rad,
        }
    }

    /// Attaches the executor's authorized-stop generation tracker and updates the bus
    /// driver's `EstopGate` so mid-batch writes also stop on authorized stop requests.
    pub fn attach_stop_requests(&mut self, stops: crate::rt_core::urgent::StopRequests) {
        self.stop_requests = Some(stops);
        let gate = self.estop_gate();
        if let Some(d) = self.driver.as_mut() {
            let _ = d.with_mut(|drv| drv.set_estop_gate(gate));
        }
    }

    /// An E-stop request the tick has not applied yet (not the latch: hold output while
    /// latched still reaches the motors).
    pub(crate) fn estop_request_unapplied(&self) -> bool {
        self.estop_state.requested_count() != self.applied_estop_count
    }

    /// True when either an E-stop request or an attached authorized-stop request has not
    /// yet been applied by the control tick / release engine.
    pub(crate) fn urgent_stop_unapplied(&self) -> bool {
        self.estop_request_unapplied()
            || self.stop_requests.as_ref().is_some_and(crate::rt_core::urgent::StopRequests::pending)
    }

    pub(crate) fn estop_pending(&self) -> bool {
        let (req, latched) = self.estop_state.snapshot();
        latched || req != self.applied_estop_count
    }

    pub(super) fn reset_gripper_stall(&mut self) {
        self.gripper_stalled = false;
        self.gripper_stall_sign = 0.0;
        self.gripper_prev_hw_overload = false;
        self.gripper_high_torque_ticks = 0;
        self.gripper_low_torque_ticks = 0;
        self.gripper_stall_window.reset();
        self.gripper_force_capped = false;
        self.gripper_force_ticks = 0;
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
        self.gripper_stream_contact_detector.reset();
        self.gripper_stream_lease = None;
        self.gripper_cap_ramp = None;
        self.gripper_over_cap_prev = false;
    }

    /// End the gripper state of a servo stream or live policy (target, contact regulator) on
    /// stop, E-stop, fault and lease loss: nothing may advance J7 for that stream afterwards.
    /// While that state was active and torque is on, J7 holds the measured jaw.
    pub fn end_gripper_stream(&mut self) {
        let active = self.gripper_stream_target.is_some() || self.gripper_stream_contact.is_some();
        self.gripper_stream_target = None;
        self.gripper_stream_contact = None;
        self.gripper_stream_contact_detector.reset();
        self.gripper_stream_lease = None;
        if !active || !self.is_torqued { return; }
        let Some(g) = self.config.gripper_index() else { return };
        let Some(&m) = self.measured_positions.get(g) else { return };
        if !m.is_finite() { return; }
        let m = self.config.limits.get(g).map_or(m, |l| m.clamp(l.min_position, l.max_position));
        if g < self.commanded_positions.len() { self.commanded_positions[g] = m; }
        if g < self.commanded_velocities.len() { self.commanded_velocities[g] = 0.0; }
        if g < self.target_positions.len() { self.target_positions[g] = m; }
        if let Some(st) = self.servo_target.as_mut() { if st.len() > g { st[g] = m; } }
        if let Some(h) = self.gripper_motion_hold.as_mut() { h.0 = m; }
    }

    /// A gripper action (or macro gripper step) ends without finishing (stop, E-stop, fault,
    /// lease loss, cancel): J7 command and target go to the measured jaw (clamped to the limits),
    /// so a command that led the jaw does not keep compressing after the stop.
    pub fn hold_gripper_at_measured(&mut self) {
        if !self.is_torqued { return; }
        let Some(g) = self.config.gripper_index() else { return };
        let Some(&m) = self.measured_positions.get(g) else { return };
        if !m.is_finite() { return; }
        let m = self.config.limits.get(g).map_or(m, |l| m.clamp(l.min_position, l.max_position));
        if g < self.commanded_positions.len() { self.commanded_positions[g] = m; }
        if g < self.commanded_velocities.len() { self.commanded_velocities[g] = 0.0; }
        if g < self.target_positions.len() { self.target_positions[g] = m; }
        if let Some(st) = self.servo_target.as_mut() { if st.len() > g { st[g] = m; } }
        if let Some(h) = self.gripper_motion_hold.as_mut() { h.0 = m; }
    }

    pub(super) fn gripper_reference_active(&self) -> bool {
        matches!(self.active_trajectory.as_ref().map(|a| &a.reference), Some(Reference::Gripper(_)))
    }

    /// Zero-allocation stack version of measured hold pose writing into `dst` (up to `dof`).
    pub(crate) fn measured_hold_pose_into(&self, dst: &mut [f32]) -> usize {
        let dof = self.commanded_positions.len().min(dst.len());
        dst[..dof].copy_from_slice(&self.commanded_positions[..dof]);
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            for (i, (c, m)) in dst[..dof].iter_mut().zip(&self.measured_positions).enumerate() {
                let l = &self.config.limits[i];
                *c = m.clamp(l.min_position, l.max_position);
            }
        }
        if let Some(g) = self.config.gripper_index() {
            if let (Some(c), Some(&cmd)) = (dst[..dof].get_mut(g), self.commanded_positions.get(g)) {
                *c = cmd;
            }
        }
        dof
    }

    /// End every motion and hold the measured pose (clamped to the limits) with torque on;
    /// the gripper keeps its command so that a held object stays held.
    pub(crate) fn hold_measured_pose(&mut self) {
        self.hold_measured_pose_inner(false);
    }

    /// [`Self::hold_measured_pose`] with optional tracking-history preservation (zero heap ops,
    /// zero moves/clones of `self.tracking`; used by `torque_trip` so active tracking lead ramps
    /// smoothly in the relieving direction).
    pub(crate) fn hold_measured_pose_inner(&mut self, preserve_tracking: bool) {
        self.stop_motion();
        self.servo_stream_active = false;
        if !preserve_tracking {
            self.tracking.invalidate();
        }
        let gripper_idx = self.config.gripper_index();
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            for (i, (c, m)) in self
                .commanded_positions
                .iter_mut()
                .zip(&self.measured_positions)
                .enumerate()
            {
                if Some(i) == gripper_idx {
                    continue;
                }
                let l = &self.config.limits[i];
                *c = m.clamp(l.min_position, l.max_position);
            }
        }
        copy_into(&mut self.target_positions, &self.commanded_positions);
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.servo_lead_stall.fill(None);
        self.safety.mark_motion_finished();
    }
}
