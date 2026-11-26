//! Teach v1, armd side of the recording session and the lease-free episode commands
//! (contract: rebot-companion `_scratch/teach/CONTRACT.md`, docs/agent_action_api.md "Teach").
//!
//! - One recording at a time. It has an owner ([`RecorderOwner`]) that is separate from the
//!   motion lease: the legacy REST routes (`legacy-rest`), a WS connection (its `client_id`), or
//!   a sequence run. Only the owner stops it.
//! - On `OWNER_LEGACY`: the control tick appends a frame to the recorder's ring buffer; the
//!   parquet and sidecar are written on a blocking thread after the frames left the lock.
//! - On `OWNER_EXECUTOR`: the recorder slot is a PR0B session (`SessionSlot::Recorder`), samples
//!   stream over `rt_core/stream` (`RecorderSample`) from the control release with zero locks and
//!   zero RT allocations, [`ExecutorRecorder`] owns transitions end-to-end off-RT, and the off-RT
//!   monitor never acquires `SupervisorLock`.
//! - A monitor task per recording stops it at [`MAX_EPISODE_S`] (saved, `truncated` for REST/WS;
//!   discarded for sequence repeats so incomplete repeats are never published), below
//!   [`STOP_MIN_FREE_BYTES`] free space, and discards it 5 s after its owner's WS connection closed.
//!
//! [`MAX_EPISODE_S`]: crate::recording::store::MAX_EPISODE_S
//! [`STOP_MIN_FREE_BYTES`]: crate::recording::store::STOP_MIN_FREE_BYTES
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::recording::store::{self, DatasetPinGuard, EpisodeSidecar, JointZeroStamp, StoreError};
use crate::recording::{EpisodeState, EpisodeStore, ParquetRecordingError, RecordFrame};
use crate::rt_core::executor::{OwnerError, RtExecutorHandle};
use crate::rt_core::publish::{ReadRequirements, ServiceIdentity, StateHub};
use crate::rt_core::recorder::{
    RecorderAssembly, RecorderStartError, RecorderStopError, RtRecorderShared,
};
use crate::rt_core::session::SessionId;
use crate::supervisor::terminal_capture::TerminalCaptureChannel;
use crate::supervisor::{
    ArmError, ArmSupervisor, SupervisorInner, OWNER_EXECUTOR, OWNER_LEGACY,
};

/// A recording whose owner connection closed is discarded after this.
pub const OWNER_LOST_DISCARD: Duration = Duration::from_secs(5);
/// Period of the recording monitor.
pub const MONITOR_PERIOD: Duration = Duration::from_millis(200);
/// Free space is checked by the monitor at most this often.
pub const SPACE_CHECK_PERIOD: Duration = Duration::from_secs(1);

/// Who owns the running recording.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecorderOwner {
    /// `/api/v1/dataset/record/*` (no identity).
    LegacyRest,
    /// A WS connection; `client_id` is what the client calls itself (reported as `owner`).
    Client { connection: String, client_id: String },
    /// The sequence run (action id) that records itself.
    Sequence { action_id: u64 },
}

impl RecorderOwner {
    /// `owner` in replies: `legacy-rest`, the client id, `sequence:<action id>`.
    pub fn label(&self) -> String {
        match self {
            Self::LegacyRest => "legacy-rest".into(),
            Self::Client { client_id, .. } => client_id.clone(),
            Self::Sequence { action_id } => format!("sequence:{action_id}"),
        }
    }

    fn bound_action_id(&self) -> u64 {
        match self {
            Self::Sequence { action_id } => *action_id,
            _ => 0,
        }
    }
}

/// What a new recording is stored as.
#[derive(Debug, Clone, PartialEq)]
pub struct RecordOptions {
    pub dataset: String,
    pub task: String,
    pub source: String,
    pub sequence_id: Option<String>,
    pub repeat_index: Option<u32>,
}

impl Default for RecordOptions {
    fn default() -> Self {
        Self {
            dataset: store::DEFAULT_DATASET.into(),
            task: String::new(),
            source: "sdk".into(),
            sequence_id: None,
            repeat_index: None,
        }
    }
}

/// The running recording (legacy supervisor state; the frames are in `SupervisorInner::recorder`).
#[derive(Debug, Clone)]
pub struct ActiveRecording {
    pub id: String,
    pub owner: RecorderOwner,
    pub options: RecordOptions,
    pub recorded_at: String,
    pub started: Instant,
    pub joint_zero: JointZeroStamp,
    /// The tick appends at most this many frames (the length cap at the control rate).
    pub max_frames: usize,
    /// When the owner's connection closed.
    pub owner_lost_at: Option<Instant>,
    /// Dataset pin protecting the target dataset from deletion while recording/finalizing.
    #[allow(dead_code)]
    dataset_pin: Option<Arc<DatasetPinGuard>>,
}

/// Who asks to end the recording.
#[derive(Debug, Clone, PartialEq)]
pub enum Stopper {
    Owner(RecorderOwner),
    /// The monitor of recording `recording` (length cap, low space: `reason`; owner lost:
    /// discard). It stops only that recording: checked under the coordinator/write guard, so a
    /// replacement recording started meanwhile is never touched.
    Monitor { reason: &'static str, recording: String },
}

/// Result of ending a recording.
#[derive(Debug, Clone, PartialEq)]
pub enum Stopped {
    Saved { sidecar: Box<EpisodeSidecar>, path: PathBuf },
    Discarded,
}

/// Custom parquet writer (tests): writes the frames to the path it gets.
pub type ParquetWriter =
    Box<dyn FnOnce(&Path, &[RecordFrame]) -> Result<(), ParquetRecordingError> + Send>;

fn rejected(status: u16, code: &str, message: impl Into<String>) -> ArmError {
    ArmError::Rejected { status, code: code.into(), message: message.into() }
}

fn executor_unavailable() -> ArmError {
    rejected(503, "EXECUTOR_UNAVAILABLE", "RT executor ownership is transitioning or unavailable")
}

// ============================================================================
// Off-RT Executor Recorder Coordinator (zero SupervisorLock, zero RT alloc)
// ============================================================================

#[derive(Debug, Clone)]
struct SidecarMeta {
    joint_names: Vec<String>,
    profile_id: Option<String>,
    dof: usize,
    arm_dof: usize,
    gripper_joint: Option<usize>,
    tool_id: String,
    tool_config_digest: String,
    tool_digest_hash: u64,
}

impl SidecarMeta {
    fn from_identity(id: &ServiceIdentity) -> Self {
        Self {
            joint_names: id.joint_names.clone(),
            profile_id: id.profile_id.clone(),
            dof: id.dof,
            arm_dof: id.arm_dof,
            gripper_joint: id.gripper_joint,
            tool_id: id.tool_id.clone(),
            tool_config_digest: id.tool_config_digest.clone(),
            tool_digest_hash: crate::rt_core::ops::tool_digest_hash(&id.tool_config_digest),
        }
    }
}

struct PendingStart {
    op_id: u64,
    dataset: String,
    owner: RecorderOwner,
    owner_lost_at: Option<Instant>,
}

struct ExecutorActiveRecording {
    op_id: u64,
    id: String,
    session_id: SessionId,
    owner: RecorderOwner,
    options: RecordOptions,
    recorded_at: String,
    started: Instant,
    joint_zero: JointZeroStamp,
    max_frames: usize,
    max_episode_s: f64,
    owner_lost_at: Option<Instant>,
    start_meta: SidecarMeta,
    rt_shared: Arc<RtRecorderShared>,
    assembly: Arc<RecorderAssembly>,
    dataset_pin: Option<Arc<DatasetPinGuard>>,
}

struct ExecutorRecorderState {
    phase: EpisodeState,
    pending_start: Option<PendingStart>,
    active: Option<ExecutorActiveRecording>,
    stopping: bool,
    finalizing_dataset: Option<String>,
    last_episode_id: String,
    last_output_path: PathBuf,
    last_frame_count: usize,
}

impl Default for ExecutorRecorderState {
    fn default() -> Self {
        Self {
            phase: EpisodeState::Idle,
            pending_start: None,
            active: None,
            stopping: false,
            finalizing_dataset: None,
            last_episode_id: String::new(),
            last_output_path: PathBuf::new(),
            last_frame_count: 0,
        }
    }
}

/// Off-RT coordinator owning executor recording transitions (start reservation -> RT session
/// admission -> monitor -> RT stop fence -> off-thread assembly & atomic store publication).
///
/// Never locked on `armd-control`; its short sync mutex is never held across `.await` or disk I/O.
pub struct ExecutorRecorder {
    state: Mutex<ExecutorRecorderState>,
    next_op_id: AtomicU64,
    control_thread_lock_attempts: AtomicU64,
}

impl Default for ExecutorRecorder {
    fn default() -> Self {
        Self::new()
    }
}

impl ExecutorRecorder {
    pub fn new() -> Self {
        Self {
            state: Mutex::new(ExecutorRecorderState::default()),
            next_op_id: AtomicU64::new(1),
            control_thread_lock_attempts: AtomicU64::new(0),
        }
    }

    #[inline]
    fn lock_state(&self) -> MutexGuard<'_, ExecutorRecorderState> {
        if crate::rt_core::executor::on_rt_control_thread() {
            self.control_thread_lock_attempts.fetch_add(1, Ordering::SeqCst);
            crate::rt_core::executor::assert_not_on_rt_control_thread("ExecutorRecorder::state.lock");
        }
        crate::rt_core::executor::assert_not_in_rt_release("ExecutorRecorder::state.lock");
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn control_thread_lock_attempts(&self) -> u64 {
        self.control_thread_lock_attempts.load(Ordering::Acquire)
    }

    #[doc(hidden)]
    pub fn lock_for_test(&self) -> impl Drop + '_ {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn alloc_op_id(&self) -> u64 {
        self.next_op_id.fetch_add(1, Ordering::Relaxed)
    }

    fn check_idle_locked(st: &ExecutorRecorderState) -> Result<(), ArmError> {
        if let Some(r) = st.active.as_ref() {
            return Err(ArmError::detailed(
                409,
                "RECORDER_BUSY",
                format!("a recording owned by '{}' is running", r.owner.label()),
                json!({"owner": r.owner.label(), "recording_id": r.id}),
            ));
        }
        if let Some(p) = st.pending_start.as_ref() {
            return Err(ArmError::detailed(
                409,
                "RECORDER_BUSY",
                format!("a recording owned by '{}' is starting", p.owner.label()),
                json!({"owner": p.owner.label()}),
            ));
        }
        if matches!(st.phase, EpisodeState::Recording | EpisodeState::Finalizing) || st.stopping {
            return Err(ArmError::detailed(
                409,
                "RECORDER_BUSY",
                "an episode is being saved",
                json!({"owner": Value::Null}),
            ));
        }
        Ok(())
    }

    pub fn check_idle(&self) -> Result<(), ArmError> {
        let st = self.lock_state();
        Self::check_idle_locked(&st)
    }

    pub fn recording_status(&self) -> Value {
        let st = self.lock_state();
        match st.active.as_ref() {
            None => Value::Null,
            Some(r) => json!({
                "recording_id": r.id,
                "owner": r.owner.label(),
                "dataset": r.options.dataset,
                "task": r.options.task,
                "started_at": r.recorded_at,
                "source": r.options.source,
                "frames": r.rt_shared.live_pushed(),
                "elapsed_s": r.started.elapsed().as_secs_f64(),
                "owner_connected": r.owner_lost_at.is_none(),
            }),
        }
    }

    pub fn get_state(&self) -> (EpisodeState, usize) {
        let st = self.lock_state();
        let frames = match st.active.as_ref() {
            Some(r) => r.rt_shared.live_pushed(),
            None => st.last_frame_count,
        };
        (st.phase.clone(), frames)
    }

    pub fn is_recording_dataset(&self, name: &str) -> bool {
        let st = self.lock_state();
        st.active.as_ref().is_some_and(|r| r.options.dataset == name)
            || st.pending_start.as_ref().is_some_and(|p| p.dataset == name)
            || st.finalizing_dataset.as_deref() == Some(name)
    }

    pub fn is_owned_by(&self, owner: &RecorderOwner) -> bool {
        let st = self.lock_state();
        st.active.as_ref().is_some_and(|r| &r.owner == owner)
            || st.pending_start.as_ref().is_some_and(|p| &p.owner == owner)
    }

    pub fn claim(&self, caller: &Caller) -> RecorderOwner {
        let me = RecorderOwner::Client {
            connection: caller.connection.clone(),
            client_id: caller.client_id.clone().unwrap_or_default(),
        };
        let mut st = self.lock_state();
        let stopping = st.stopping;
        if let Some(r) = st.active.as_mut() {
            if let RecorderOwner::Client { connection, client_id } = &mut r.owner {
                if *connection == caller.connection {
                    return r.owner.clone();
                }
                if !stopping
                    && r.owner_lost_at.is_some()
                    && caller.client_id.as_deref() == Some(client_id.as_str())
                {
                    *connection = caller.connection.clone();
                    r.owner_lost_at = None;
                    tracing::info!(
                        "Recording {} taken back by '{client_id}' on a new connection",
                        r.id
                    );
                    return r.owner.clone();
                }
            }
        }
        if let Some(p) = st.pending_start.as_mut() {
            if let RecorderOwner::Client { connection, client_id } = &mut p.owner {
                if *connection == caller.connection {
                    return p.owner.clone();
                }
                if p.owner_lost_at.is_some()
                    && caller.client_id.as_deref() == Some(client_id.as_str())
                {
                    *connection = caller.connection.clone();
                    p.owner_lost_at = None;
                    tracing::info!(
                        "Pending recording start {} taken back by '{client_id}' on a new connection",
                        p.op_id
                    );
                    return p.owner.clone();
                }
            }
        }
        me
    }

    pub fn connection_closed(&self, connection: &str) {
        let mut st = self.lock_state();
        if let Some(r) = st.active.as_mut() {
            if matches!(&r.owner, RecorderOwner::Client { connection: c, .. } if c == connection)
                && r.owner_lost_at.is_none()
            {
                r.owner_lost_at = Some(Instant::now());
                tracing::warn!(
                    "Owner of recording {} disconnected; discarding in {} s unless it reconnects",
                    r.id,
                    OWNER_LOST_DISCARD.as_secs()
                );
            }
        }
        if let Some(p) = st.pending_start.as_mut() {
            if matches!(&p.owner, RecorderOwner::Client { connection: c, .. } if c == connection)
                && p.owner_lost_at.is_none()
            {
                p.owner_lost_at = Some(Instant::now());
            }
        }
    }
}

#[derive(Clone)]
struct ExecutorRecorderCtx {
    recorder: Arc<ExecutorRecorder>,
    handle: RtExecutorHandle,
    store: EpisodeStore,
    hub: Arc<StateHub>,
    terminal_capture: Arc<TerminalCaptureChannel>,
    base_gen: u64,
}

impl ExecutorRecorderCtx {
    fn read_identity(&self) -> Result<Arc<crate::rt_core::publish::ServiceSnapshot>, ArmError> {
        let mut req = ReadRequirements::identity();
        req.min_generation = Some(self.base_gen.saturating_add(1));
        self.hub.read_services(req).map_err(ArmError::from)
    }

    fn read_identity_or_latest(&self) -> Option<Arc<crate::rt_core::publish::ServiceSnapshot>> {
        self.read_identity()
            .ok()
            .or_else(|| self.hub.read_services(ReadRequirements::identity()).ok())
    }

    async fn wait_identity_matching(
        &self,
        mut predicate: impl FnMut(&crate::rt_core::publish::ServiceSnapshot) -> bool,
    ) -> Option<Arc<crate::rt_core::publish::ServiceSnapshot>> {
        for attempt in 0..50 {
            if let Some(svc) = self.read_identity_or_latest() {
                if predicate(&svc) {
                    return Some(svc);
                }
            }
            if attempt < 49 {
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        }
        None
    }
}

fn validate_stop_target(
    st: &ExecutorRecorderState,
    by: &Stopper,
    save: bool,
    expected_op_id: Option<u64>,
) -> Result<u64, ArmError> {
    let Some(r) = st.active.as_ref() else {
        return Err(crate::recording::RecordingError::InvalidState {
            action: if save { "save".into() } else { "discard".into() },
            state: st.phase.clone(),
        }
        .into());
    };
    if let Some(exp) = expected_op_id {
        if r.op_id != exp {
            return Err(rejected(
                409,
                "RECORDING_REPLACED",
                "the targeted recording ended; another one runs",
            ));
        }
    }
    match by {
        Stopper::Owner(o) => {
            if o != &r.owner {
                return Err(ArmError::detailed(
                    409,
                    "RECORDER_NOT_OWNER",
                    format!("the recording is owned by '{}'", r.owner.label()),
                    json!({"owner": r.owner.label()}),
                ));
            }
        }
        Stopper::Monitor { reason, recording } => {
            if recording != &r.id {
                return Err(rejected(
                    409,
                    "RECORDING_REPLACED",
                    "the monitored recording ended; another one runs",
                ));
            }
            if *reason == "owner_lost" {
                let still_lost = r
                    .owner_lost_at
                    .is_some_and(|lost_at| lost_at.elapsed() >= OWNER_LOST_DISCARD);
                if !still_lost {
                    return Err(rejected(
                        409,
                        "RECORDING_REPLACED",
                        "the recording owner reconnected before discard",
                    ));
                }
            }
        }
    }
    Ok(r.op_id)
}

async fn start_recording_executor(
    ctx: ExecutorRecorderCtx,
    owner: RecorderOwner,
    options: RecordOptions,
    legacy_id: Option<String>,
    legacy_path: Option<PathBuf>,
) -> Result<(String, String, PathBuf), ArmError> {
    let op_id = ctx.recorder.alloc_op_id();
    {
        let mut st = ctx.recorder.lock_state();
        ExecutorRecorder::check_idle_locked(&st)?;
        st.pending_start = Some(PendingStart {
            op_id,
            dataset: options.dataset.clone(),
            owner: owner.clone(),
            owner_lost_at: None,
        });
    }
    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        let res = run_start_recording_executor(ctx, op_id, owner, options, legacy_id, legacy_path).await;
        let _ = reply_tx.send(res);
    });
    reply_rx
        .await
        .map_err(|_| ArmError::Internal("executor recorder start task terminated unexpectedly".into()))?
}

/// Retries RT discard until RT confirms a frozen close fence for `(op_id, session_id)` or the
/// executor exits, ensuring a failed start never orphans an occupied `SessionSlot::Recorder`.
async fn reconcile_discard_rt_session(
    ctx: &ExecutorRecorderCtx,
    op_id: u64,
    session_id: SessionId,
    reason: &'static str,
    rt_shared: &Arc<RtRecorderShared>,
    assembly: &Arc<RecorderAssembly>,
) {
    loop {
        if let Some(fence) = rt_shared.load_closed_fence_for(op_id, session_id) {
            let asm = Arc::clone(assembly);
            let _ = tokio::task::spawn_blocking(move || asm.collect_session(fence)).await;
            return;
        }
        if ctx.handle.is_exited() {
            return;
        }
        let sid = if session_id.0 != 0 {
            session_id
        } else {
            SessionId(rt_shared.live_session.load(Ordering::Acquire))
        };
        if sid.0 == 0 && rt_shared.live_op_id.load(Ordering::Acquire) != op_id {
            return;
        }
        match ctx
            .handle
            .stop_recorder_session(op_id, sid, false, reason, 0)
            .await
        {
            Ok(Ok(fence)) => {
                let asm = Arc::clone(assembly);
                let _ = tokio::task::spawn_blocking(move || asm.collect_session(fence)).await;
                return;
            }
            Ok(Err(RecorderStopError::NotActive | RecorderStopError::SessionMismatch)) => {
                if let Some(fence) = rt_shared.load_closed_fence_for(op_id, sid) {
                    let asm = Arc::clone(assembly);
                    let _ = tokio::task::spawn_blocking(move || asm.collect_session(fence)).await;
                    return;
                }
                if rt_shared.live_op_id.load(Ordering::Acquire) != op_id {
                    return;
                }
            }
            Ok(Err(_)) => return,
            Err(OwnerError::QueueFull | OwnerError::TimedOut) => {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            Err(OwnerError::Stopped | OwnerError::ShutdownNotConfirmed | OwnerError::Protocol) => {
                return;
            }
        }
    }
}

async fn run_start_recording_executor(
    ctx: ExecutorRecorderCtx,
    op_id: u64,
    owner: RecorderOwner,
    options: RecordOptions,
    legacy_id: Option<String>,
    legacy_path: Option<PathBuf>,
) -> Result<(String, String, PathBuf), ArmError> {
    let clear_pending = |rec: &ExecutorRecorder| {
        let mut st = rec.lock_state();
        if st.pending_start.as_ref().is_some_and(|p| p.op_id == op_id) {
            st.pending_start = None;
        }
    };

    let svc = match ctx.read_identity() {
        Ok(s) => s,
        Err(e) => {
            clear_pending(&ctx.recorder);
            return Err(e);
        }
    };

    let st_clone = ctx.store.clone();
    let dataset = options.dataset.clone();
    let preflight = tokio::task::spawn_blocking(move || {
        let episode = st_clone.new_episode(legacy_id.as_deref(), legacy_path.as_deref())?;
        let pin = st_clone.pin_dataset_for_recording(&dataset)?;
        st_clone.check_capacity()?;
        st_clone.check_free_for_start()?;
        Ok::<_, StoreError>((episode, pin))
    })
    .await
    .map_err(|e| ArmError::Internal(format!("episode check failed: {e}")))
    .and_then(|r| r.map_err(ArmError::from));

    let ((id, path), dataset_pin) = match preflight {
        Ok(ok) => ok,
        Err(e) => {
            clear_pending(&ctx.recorder);
            return Err(e);
        }
    };

    let max_episode_s = svc.identity.max_episode_s;
    let hz = svc.identity.control_frequency_hz.max(1);
    let max_frames = (max_episode_s * f64::from(hz)).ceil() as usize + 1;
    let bound_action_id = owner.bound_action_id();
    let rt_shared = ctx.handle.recorder_shared();
    let assembly = ctx.handle.recorder_assembly();

    let ack = match ctx
        .handle
        .start_recorder_session(op_id, max_frames, bound_action_id)
        .await
    {
        Ok(Ok(ack)) => ack,
        Ok(Err(start_err)) => {
            clear_pending(&ctx.recorder);
            return Err(match start_err {
                RecorderStartError::SlotBusy => ArmError::detailed(
                    409,
                    "RECORDER_BUSY",
                    "a recording is already running",
                    json!({"owner": Value::Null}),
                ),
                RecorderStartError::FaultFenced => rejected(
                    503,
                    "INTERNAL_FAULT",
                    "Internal fault or SAFE-HOLD active; recording start refused",
                ),
                RecorderStartError::ParentNotAlive => crate::safety::SafetyError::Stopped.into(),
                RecorderStartError::StreamUnattached => executor_unavailable(),
            });
        }
        Err(owner_err) => {
            // Reconcile uncertain completion: retain `pending_start` and `dataset_pin` until RT
            // confirms a close fence for `op_id` or exits so the slot can never be orphaned.
            if rt_shared.live_op_id.load(Ordering::Acquire) == op_id {
                let sid = SessionId(rt_shared.live_session.load(Ordering::Acquire));
                reconcile_discard_rt_session(&ctx, op_id, sid, "aborted", &rt_shared, &assembly)
                    .await;
            }
            drop(dataset_pin);
            clear_pending(&ctx.recorder);
            return Err(match owner_err {
                OwnerError::Stopped => executor_unavailable(),
                OwnerError::TimedOut => rejected(503, "DEADLINE_EXPIRED", "RT recorder start timed out"),
                OwnerError::QueueFull => rejected(503, "MOTION_BUSY", "RT maintenance queue is full"),
                OwnerError::ShutdownNotConfirmed => executor_unavailable(),
                OwnerError::Protocol => ArmError::Internal("RT recorder start protocol mismatch".into()),
            });
        }
    };

    // Bind start-boundary `joint_zero` to the exact `start_calib_rev` active on RT at admission.
    // Fail closed if published identity never matches `ack.start_calib_rev` within the bounded wait,
    // retaining `pending_start` and `dataset_pin` until RT confirms close fence or exit.
    let start_svc = if svc.revisions.calibration == ack.start_calib_rev {
        svc
    } else if let Some(matched) = ctx
        .wait_identity_matching(|s| s.revisions.calibration == ack.start_calib_rev)
        .await
    {
        matched
    } else {
        reconcile_discard_rt_session(
            &ctx,
            op_id,
            ack.session_id,
            "calib_mismatch",
            &rt_shared,
            &assembly,
        )
        .await;
        drop(dataset_pin);
        clear_pending(&ctx.recorder);
        return Err(rejected(
            409,
            "CALIBRATION_CHANGED",
            "calibration revision changed during recording start; retry",
        ));
    };
    let joint_zero = JointZeroStamp::of(&start_svc.identity.joint_zero_offsets);
    let start_meta = SidecarMeta::from_identity(&start_svc.identity);
    let recorded_at = chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true);

    let label = {
        let mut st = ctx.recorder.lock_state();
        let pending = st.pending_start.take().filter(|p| p.op_id == op_id);
        let (effective_owner, inherited_lost) = match pending {
            Some(p) => (p.owner, p.owner_lost_at),
            None => (owner, None),
        };
        let label = effective_owner.label();
        st.phase = EpisodeState::Recording;
        st.last_episode_id = id.clone();
        st.last_output_path = path.clone();
        st.last_frame_count = 0;
        st.active = Some(ExecutorActiveRecording {
            op_id,
            id: id.clone(),
            session_id: ack.session_id,
            owner: effective_owner,
            options,
            recorded_at,
            started: Instant::now(),
            joint_zero,
            max_frames,
            max_episode_s,
            owner_lost_at: inherited_lost,
            start_meta,
            rt_shared,
            assembly,
            dataset_pin: Some(Arc::new(dataset_pin)),
        });
        label
    };

    tracing::info!("Recording {id} started on executor (owner {label}, session {})", ack.session_id.0);
    let mon_id = id.clone();
    let mon_ctx = ctx.clone();
    tokio::spawn(async move {
        executor_monitor(mon_ctx, mon_id, op_id, ack.session_id).await;
    });
    Ok((id, label, path))
}

async fn stop_recording_executor(
    ctx: ExecutorRecorderCtx,
    by: Stopper,
    save: bool,
    outcome: Option<&str>,
    writer: Option<ParquetWriter>,
    expected_op_id: Option<u64>,
) -> Result<Stopped, ArmError> {
    let outcome_str = outcome.unwrap_or("ok");
    if !["ok", "aborted"].contains(&outcome_str) {
        return Err(rejected(400, "INVALID_COMMAND", "outcome must be \"ok\" or \"aborted\""));
    }
    let outcome_owned = outcome_str.to_string();
    let monitor = matches!(by, Stopper::Monitor { .. });
    let target_op_id = {
        let st = ctx.recorder.lock_state();
        validate_stop_target(&st, &by, save, expected_op_id)?
    };
    let capacity_slot = if save && !monitor {
        // Pre-fence capacity reservation: refused while still recording so episodes can be deleted
        // and retried, and reserves a slot so a concurrent import cannot steal it before publish.
        let s2 = ctx.store.clone();
        Some(
            tokio::task::spawn_blocking(move || s2.reserve_capacity_slot())
                .await
                .map_err(|e| ArmError::Internal(format!("episode check failed: {e}")))??,
        )
    } else {
        None
    };

    let (op_id, session_id, require_alive_action_id, rt_shared) = {
        let mut st = ctx.recorder.lock_state();
        let cur_op_id = validate_stop_target(&st, &by, save, Some(target_op_id))?;
        if st.stopping {
            return Err(crate::recording::RecordingError::InvalidState {
                action: if save { "save".into() } else { "discard".into() },
                state: EpisodeState::Finalizing,
            }
            .into());
        }
        let r = st.active.as_ref().expect("validated above");
        let req_action = if save { r.owner.bound_action_id() } else { 0 };
        let tup = (cur_op_id, r.session_id, req_action, Arc::clone(&r.rt_shared));
        st.stopping = true;
        tup
    };

    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        let res = run_stop_recording_executor(
            ctx,
            by,
            save,
            outcome_owned,
            writer,
            op_id,
            session_id,
            require_alive_action_id,
            rt_shared,
            capacity_slot,
        )
        .await;
        let _ = reply_tx.send(res);
    });
    reply_rx
        .await
        .map_err(|_| ArmError::Internal("executor recorder stop task terminated unexpectedly".into()))?
}

#[allow(clippy::too_many_arguments)]
async fn run_stop_recording_executor(
    ctx: ExecutorRecorderCtx,
    by: Stopper,
    save: bool,
    outcome: String,
    writer: Option<ParquetWriter>,
    op_id: u64,
    session_id: SessionId,
    require_alive_action_id: u64,
    rt_shared: Arc<RtRecorderShared>,
    capacity_slot: Option<store::CapacitySlotGuard>,
) -> Result<Stopped, ArmError> {
    let monitor = matches!(by, Stopper::Monitor { .. });
    let stop_reason: &'static str = match &by {
        Stopper::Monitor { reason, .. } => reason,
        Stopper::Owner(_) => if save { "saved" } else { "discarded" },
    };

    // Acquire sequence commit barrier BEFORE sending `stop_recorder_session` to RT so even if a
    // STOP arrives immediately after RT accepts the repeat commit (before this task resumes),
    // `armd-dispatch` holds the sequence terminal until publication finishes (or this guard drops
    // on RT refusal/error).
    let _commit_guard = if save && require_alive_action_id != 0 {
        Some(
            ctx.terminal_capture
                .acquire_sequence_commit_guard(require_alive_action_id, Arc::clone(&ctx.hub)),
        )
    } else {
        None
    };

    let rt_reply = ctx
        .handle
        .stop_recorder_session(op_id, session_id, save, stop_reason, require_alive_action_id)
        .await;

    let fence = match rt_reply {
        Ok(Ok(fence)) => fence,
        Ok(Err(RecorderStopError::EmptyEpisode)) => {
            let mut st = ctx.recorder.lock_state();
            st.stopping = false;
            return Err(crate::recording::RecordingError::EmptyEpisode.into());
        }
        Ok(Err(RecorderStopError::ParentNotAlive { fence })) => {
            let rec = {
                let mut st = ctx.recorder.lock_state();
                st.stopping = false;
                st.phase = EpisodeState::Discarded;
                st.last_frame_count = 0;
                st.active.take()
            };
            if let Some(rec) = rec {
                let asm = Arc::clone(&rec.assembly);
                let _ = tokio::task::spawn_blocking(move || asm.collect_session(fence)).await;
            }
            return Err(crate::safety::SafetyError::Stopped.into());
        }
        Ok(Err(RecorderStopError::NotActive | RecorderStopError::SessionMismatch)) => {
            if let Some(closed) = rt_shared.load_closed_fence_for(op_id, session_id) {
                closed
            } else {
                let mut st = ctx.recorder.lock_state();
                st.stopping = false;
                if !save {
                    st.active = None;
                    st.phase = EpisodeState::Discarded;
                    st.last_frame_count = 0;
                    return Ok(Stopped::Discarded);
                }
                return Err(executor_unavailable());
            }
        }
        Err(owner_err) => {
            if let Some(closed) = rt_shared.load_closed_fence_for(op_id, session_id) {
                closed
            } else {
                match owner_err {
                    // Never clear `active` or report `Discarded` without an RT close fence when
                    // RT is still running and the maintenance queue was saturated or timed out.
                    OwnerError::QueueFull => {
                        ctx.recorder.lock_state().stopping = false;
                        return Err(rejected(503, "MOTION_BUSY", "RT maintenance queue is full"));
                    }
                    OwnerError::TimedOut => {
                        ctx.recorder.lock_state().stopping = false;
                        return Err(rejected(503, "DEADLINE_EXPIRED", "RT recorder stop timed out"));
                    }
                    OwnerError::Stopped | OwnerError::ShutdownNotConfirmed | OwnerError::Protocol => {
                        let mut st = ctx.recorder.lock_state();
                        st.stopping = false;
                        st.active = None;
                        st.phase = EpisodeState::Discarded;
                        st.last_frame_count = 0;
                        if !save {
                            return Ok(Stopped::Discarded);
                        }
                        return Err(executor_unavailable());
                    }
                }
            }
        }
    };

    // If a sequence repeat session was already revoked (SAFE-HOLD / shutdown), never publish it.
    if save && require_alive_action_id != 0 && fence.revoked {
        let rec = {
            let mut st = ctx.recorder.lock_state();
            st.stopping = false;
            st.phase = EpisodeState::Discarded;
            st.last_frame_count = 0;
            st.active.take()
        };
        if let Some(rec) = rec {
            let asm = Arc::clone(&rec.assembly);
            let _ = tokio::task::spawn_blocking(move || asm.collect_session(fence)).await;
        }
        return Err(crate::safety::SafetyError::Stopped.into());
    }

    let Some(rec) = ({
        let mut st = ctx.recorder.lock_state();
        st.stopping = false;
        let taken = st.active.take();
        if save {
            st.phase = EpisodeState::Finalizing;
            st.finalizing_dataset = taken.as_ref().map(|r| r.options.dataset.clone());
        } else {
            st.phase = EpisodeState::Discarded;
            st.finalizing_dataset = None;
            st.last_frame_count = 0;
        }
        taken
    }) else {
        return Err(crate::recording::RecordingError::InvalidState {
            action: if save { "save".into() } else { "discard".into() },
            state: EpisodeState::Idle,
        }
        .into());
    };

    let asm = Arc::clone(&rec.assembly);
    let collected = tokio::task::spawn_blocking(move || asm.collect_session(fence))
        .await
        .map_err(|e| ArmError::Internal(format!("recording frame collection failed: {e}")));

    let collected = match collected {
        Ok(c) => c,
        Err(e) => {
            let mut st = ctx.recorder.lock_state();
            st.phase = EpisodeState::Discarded;
            st.finalizing_dataset = None;
            st.last_frame_count = 0;
            return Err(e);
        }
    };

    if !save {
        drop(rec.dataset_pin);
        drop(capacity_slot);
        tracing::info!("Recording {} discarded", rec.id);
        return Ok(Stopped::Discarded);
    }

    let frames = collected.frames;
    let dropped = collected.dropped;
    if frames.is_empty() {
        let mut st = ctx.recorder.lock_state();
        st.phase = EpisodeState::Discarded;
        st.finalizing_dataset = None;
        st.last_frame_count = 0;
        drop(rec.dataset_pin);
        drop(capacity_slot);
        return Err(crate::recording::RecordingError::EmptyEpisode.into());
    }
    if dropped > 0 {
        tracing::warn!(
            "Recording {}: {} samples dropped on a full recording stream; the episode is incomplete",
            rec.id,
            dropped
        );
    }

    // Bind stop-boundary metadata (`joint_names`, `profile_id`, `arm_dof`, `gripper_joint`, `fps`,
    // `tool_id`, `tool_config_digest`) strictly to the RT stop fence (`stop_tool_digest_hash`,
    // `stop_dof`). Fail closed if metadata diverges rather than stamping mismatched tool identity.
    let stop_meta = if let Some(svc) = ctx
        .wait_identity_matching(|s| {
            s.revisions.tool_digest_hash == fence.stop_tool_digest_hash
                && s.identity.dof == usize::from(fence.stop_dof)
        })
        .await
    {
        SidecarMeta::from_identity(&svc.identity)
    } else if rec.start_meta.tool_digest_hash == fence.stop_tool_digest_hash
        && rec.start_meta.dof == usize::from(fence.stop_dof)
    {
        rec.start_meta.clone()
    } else {
        let mut st = ctx.recorder.lock_state();
        st.phase = EpisodeState::Discarded;
        st.finalizing_dataset = None;
        st.last_frame_count = 0;
        drop(rec.dataset_pin);
        drop(capacity_slot);
        return Err(rejected(
            409,
            "TOOL_MISMATCH",
            "tool metadata diverged from RT stop fence; episode discarded",
        ));
    };

    let has_gripper_motion = store::episode_has_gripper_motion(&frames, stop_meta.gripper_joint);
    let duration = match (frames.first(), frames.last()) {
        (Some(a), Some(b)) => (b.timestamp - a.timestamp).max(0.0),
        _ => 0.0,
    };
    let (truncated, stopped_by) = match &by {
        Stopper::Monitor { reason, .. } => (*reason == "max_duration", Some((*reason).to_string())),
        _ => (false, None),
    };
    let sidecar = EpisodeSidecar {
        id: rec.id.clone(),
        dataset: rec.options.dataset.clone(),
        task: rec.options.task.clone(),
        source: rec.options.source.clone(),
        sequence_id: rec.options.sequence_id.clone(),
        repeat_index: rec.options.repeat_index,
        outcome,
        recorded_at: rec.recorded_at.clone(),
        duration_s: duration,
        frames: frames.len(),
        joint_names: stop_meta.joint_names,
        units: "rad".into(),
        joint_zero: rec.joint_zero.clone(),
        profile_id: stop_meta.profile_id,
        armd_version: env!("CARGO_PKG_VERSION").into(),
        fps_nominal: f64::from(fence.stop_control_hz.max(1)),
        truncated,
        stopped_by,
        arm_dof: Some(stop_meta.arm_dof),
        gripper_joint: stop_meta.gripper_joint,
        recovered: None,
        first_state: frames.first().map(|f| f.observation_state.clone()),
        last_state: frames.last().map(|f| f.observation_state.clone()),
        tool_id: Some(stop_meta.tool_id),
        tool_config_digest: Some(stop_meta.tool_config_digest),
        has_gripper_motion: Some(has_gripper_motion),
        frames_dropped: (dropped > 0).then_some(dropped),
    };

    let sc = sidecar.clone();
    let store = ctx.store.clone();
    // Manual saves already hold a `CapacitySlotGuard` acquired before RT stop (consumed atomically
    // into `in_progress_tokens` inside Phase 1 so 1 episode occupies strictly 1 slot).
    let enforce_capacity = !monitor && capacity_slot.is_none();
    let commit_token = Some(rec.op_id);
    let mut cap_slot = capacity_slot;
    let write_res = tokio::task::spawn_blocking(move || {
        match store.publish_complete_episode_with_slot(
            &sc,
            &frames,
            writer,
            commit_token,
            enforce_capacity,
            cap_slot.as_mut(),
        ) {
            Ok(p) => Ok(p),
            Err(first_err) => {
                // If parquet rename already succeeded and sidecar/dir-sync failed transiently,
                // retry once with the same `commit_token` to complete post-rename recovery.
                if store.path_of(&sc.id).is_ok_and(|p| p.is_file()) {
                    store.publish_complete_episode(&sc, &frames, None, commit_token, false)
                } else {
                    Err(first_err)
                }
            }
        }
    })
    .await
    .unwrap_or_else(|e| {
        Err(StoreError::Io(std::io::Error::other(format!(
            "recording writer panicked: {e}"
        ))))
    });

    {
        let mut st = ctx.recorder.lock_state();
        st.finalizing_dataset = None;
        match &write_res {
            Ok(p) => {
                st.phase = EpisodeState::Committed;
                st.last_episode_id = rec.id.clone();
                st.last_output_path = p.clone();
                st.last_frame_count = sidecar.frames;
            }
            Err(_) => {
                st.phase = EpisodeState::Discarded;
                st.last_frame_count = 0;
            }
        }
    }
    drop(rec.dataset_pin);

    let path = write_res?;
    if require_alive_action_id != 0 {
        let ep_id = sidecar.id.clone();
        ctx.terminal_capture
            .update_rich_action_off_rt(require_alive_action_id, "sequence", |a| {
                if !a.episodes.iter().any(|existing| existing == &ep_id) {
                    a.episodes.push(ep_id);
                }
            });
    }
    tracing::info!(
        "Recording {} saved on executor ({} frames, {:.1} s{})",
        rec.id,
        sidecar.frames,
        sidecar.duration_s,
        if truncated { ", truncated" } else { "" }
    );
    Ok(Stopped::Saved { sidecar: Box::new(sidecar), path })
}

/// Off-RT monitor for executor recordings (NEVER acquires `SupervisorLock`).
async fn executor_monitor(
    ctx: ExecutorRecorderCtx,
    id: String,
    op_id: u64,
    session_id: SessionId,
) {
    let mut last_space: Option<Instant> = None;
    loop {
        tokio::time::sleep(MONITOR_PERIOD).await;
        let snapshot = {
            let st = ctx.recorder.lock_state();
            let Some(r) = st.active.as_ref().filter(|r| r.id == id && r.op_id == op_id) else {
                return;
            };
            if st.stopping {
                continue;
            }
            let rt_live = r.rt_shared.is_live_for(op_id, session_id);
            let pushed = r.rt_shared.live_pushed();
            let cap_reached = r.rt_shared.live_cap_reached();
            let full = cap_reached
                || pushed >= r.max_frames
                || r.started.elapsed().as_secs_f64() >= r.max_episode_s + 1.0;
            let lost = r
                .owner_lost_at
                .is_some_and(|t| t.elapsed() >= OWNER_LOST_DISCARD);
            let is_sequence = matches!(r.owner, RecorderOwner::Sequence { .. });
            (rt_live, full, lost, is_sequence)
        };
        let (rt_live, full, lost, is_sequence) = snapshot;
        let action = if !rt_live {
            Some((false, "rt_revoked"))
        } else if lost {
            Some((false, "owner_lost"))
        } else if full {
            // Incomplete sequence repeats are discarded rather than published as truncated episodes.
            Some((!is_sequence, "max_duration"))
        } else if last_space.is_none_or(|t| t.elapsed() >= SPACE_CHECK_PERIOD) {
            last_space = Some(Instant::now());
            let st = ctx.store.clone();
            let free = tokio::task::spawn_blocking(move || st.free_bytes())
                .await
                .ok()
                .flatten();
            free.filter(|f| *f < store::STOP_MIN_FREE_BYTES)
                .map(|_| (!is_sequence, "low_space"))
        } else {
            None
        };
        let Some((save, reason)) = action else {
            continue;
        };
        match stop_recording_executor(
            ctx.clone(),
            Stopper::Monitor { reason, recording: id.clone() },
            save,
            None,
            None,
            Some(op_id),
        )
        .await
        {
            Ok(_) => {
                tracing::warn!(
                    "Recording {id} {} by armd ({reason})",
                    if save { "saved" } else { "discarded" }
                );
                return;
            }
            Err(ArmError::Rejected { ref code, .. }) if code == "RECORDING_REPLACED" => {
                let still_same = {
                    let st = ctx.recorder.lock_state();
                    st.active.as_ref().is_some_and(|r| r.id == id && r.op_id == op_id)
                };
                if !still_same {
                    return;
                }
            }
            Err(ArmError::Rejected { ref code, .. })
                if code == "MOTION_BUSY" || code == "DEADLINE_EXPIRED" =>
            {
                tracing::warn!("Recording {id}: transient stop ({reason}) error ({code}); retrying");
                continue;
            }
            Err(ArmError::Recording(crate::recording::RecordingError::InvalidState {
                state: EpisodeState::Finalizing,
                ..
            })) => {
                continue;
            }
            Err(e) => {
                tracing::warn!("Recording {id}: stop ({reason}) failed: {e}");
                return;
            }
        }
    }
}

// ============================================================================
// Legacy SupervisorInner helpers + owner-dispatched ArmSupervisor API
// ============================================================================

impl SupervisorInner {
    /// Joint names of the profile (sidecars, import checks).
    pub fn joint_names(&self) -> Vec<String> {
        self.config.limits.iter().map(|l| l.name.clone()).collect()
    }

    /// RECORDER_BUSY while a recording runs or is being written.
    pub fn recorder_idle(&self) -> Result<(), ArmError> {
        if let Some(r) = &self.recording {
            return Err(ArmError::detailed(
                409,
                "RECORDER_BUSY",
                format!("a recording owned by '{}' is running", r.owner.label()),
                json!({"owner": r.owner.label(), "recording_id": r.id}),
            ));
        }
        if matches!(self.recorder.state(), EpisodeState::Recording | EpisodeState::Finalizing) {
            return Err(ArmError::detailed(
                409,
                "RECORDER_BUSY",
                "an episode is being saved",
                json!({"owner": Value::Null}),
            ));
        }
        Ok(())
    }

    /// The tick appends frames while this holds.
    pub(crate) fn recording_frame_room(&self) -> bool {
        self.recorder.state() == &EpisodeState::Recording
            && self
                .recording
                .as_ref()
                .is_none_or(|r| self.recording_frames() < r.max_frames)
    }

    /// Frames of the running recording: those the recorder holds and those the ticks put
    /// into the recording ring.
    pub fn recording_frames(&self) -> usize {
        self.recorder.frame_count()
            + if self.core.recording_active() {
                self.core.record_pushed()
            } else {
                0
            }
    }

    /// `recording` field of `GET /api/v1/episodes` and the recorder status.
    pub fn recording_status(&self) -> Value {
        match &self.recording {
            None => Value::Null,
            Some(r) => json!({
                "recording_id": r.id,
                "owner": r.owner.label(),
                "dataset": r.options.dataset,
                "task": r.options.task,
                "started_at": r.recorded_at,
                "source": r.options.source,
                "frames": self.recording_frames(),
                "elapsed_s": r.started.elapsed().as_secs_f64(),
                "owner_connected": r.owner_lost_at.is_none(),
            }),
        }
    }
}

impl ArmSupervisor {
    async fn executor_recorder_ctx(&self) -> Result<ExecutorRecorderCtx, ArmError> {
        let handle = self.executor_handle().ok_or_else(executor_unavailable)?;
        if handle.is_exited() {
            return Err(executor_unavailable());
        }
        let store = self.try_episode_store().await?;
        Ok(ExecutorRecorderCtx {
            recorder: Arc::clone(self.executor_recorder()),
            handle,
            store,
            hub: self.hub().clone(),
            terminal_capture: self.terminal_capture_channel_arc(),
            base_gen: self.executor_base_gen(),
        })
    }

    /// Load the episode catalog (boot recovery) once; later calls do nothing.
    pub async fn ensure_episodes_loaded(&self) -> Result<Option<store::RecoveryReport>, ArmError> {
        let pub_cfg = self.published_config().await?;
        if pub_cfg.calibration_quarantined {
            return Err(rejected(
                409,
                "CONFIG_BUSY",
                "calibration is quarantined pending offline recovery; episode adoption is paused",
            ));
        }
        let st = pub_cfg.episodes.clone();
        if !st.enabled() || st.is_loaded() {
            return Ok(None);
        }
        let offsets = pub_cfg.joint_zero_offsets.clone();
        let names = pub_cfg.joint_names();
        let profile = pub_cfg.config.profile_id.clone();
        let fps = f64::from(pub_cfg.config.control_frequency_hz);
        let report =
            tokio::task::spawn_blocking(move || st.load(&offsets, &names, profile.as_deref(), fps))
                .await
                .map_err(|e| ArmError::Internal(format!("episode load failed: {e}")))??;
        Ok(Some(report))
    }

    /// Boot: run the episode recovery (after the joint zero was loaded) and log it.
    pub async fn load_episodes(&self) {
        match self.ensure_episodes_loaded().await {
            Ok(Some(r)) => tracing::info!(
                "Episodes: {} listed; recovery: {} temp files removed, {} sidecars rebuilt, {} legacy episodes adopted, {} orphans removed, {} unreadable kept, {} lone sidecars removed",
                r.episodes,
                r.temps_removed.len(),
                r.sidecars_rebuilt.len(),
                r.legacy_adopted.len(),
                r.orphans_removed.len(),
                r.orphans_kept.len(),
                r.sidecars_removed.len()
            ),
            Ok(None) => {}
            Err(e) => tracing::warn!("Episode store not loaded: {e}"),
        }
    }

    /// Start a recording owned by `owner`. Returns `(recording id, owner label, output path)`.
    pub async fn start_recording_as_with_path(
        &self,
        owner: RecorderOwner,
        options: RecordOptions,
        legacy_id: Option<String>,
        legacy_path: Option<PathBuf>,
    ) -> Result<(String, String, PathBuf), ArmError> {
        if !store::EPISODE_SOURCES.contains(&options.source.as_str()) {
            return Err(rejected(
                400,
                "INVALID_COMMAND",
                format!("source must be one of {:?}", store::EPISODE_SOURCES),
            ));
        }
        if options.task.chars().count() > 256 {
            return Err(rejected(
                400,
                "INVALID_COMMAND",
                "task is at most 256 characters",
            ));
        }
        self.ensure_episodes_loaded().await?;
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let ctx = self.executor_recorder_ctx().await?;
                start_recording_executor(ctx, owner, options, legacy_id, legacy_path).await
            }
            OWNER_LEGACY => {
                // drains the recording ring off the tick: refused when it cannot run
                self.ensure_dispatcher()?;
                let st = {
                    let inner = self.inner().read().await;
                    if inner.detached {
                        drop(inner);
                        if self.owner_mode() == OWNER_EXECUTOR {
                            let ctx = self.executor_recorder_ctx().await?;
                            return start_recording_executor(
                                ctx,
                                owner,
                                options,
                                legacy_id,
                                legacy_path,
                            )
                            .await;
                        }
                        return Err(executor_unavailable());
                    }
                    inner.recorder_idle()?;
                    inner.episodes.clone()
                };
                let dataset = options.dataset.clone();
                // file system checks without the lock (the control tick needs it)
                let ((id, path), pin) = tokio::task::spawn_blocking(move || {
                    let episode = st.new_episode(legacy_id.as_deref(), legacy_path.as_deref())?;
                    let pin = st.pin_dataset_for_recording(&dataset)?;
                    st.check_capacity()?;
                    st.check_free_for_start()?;
                    Ok::<_, StoreError>((episode, pin))
                })
                .await
                .map_err(|e| ArmError::Internal(format!("episode check failed: {e}")))??;
                let label = owner.label();
                {
                    let mut inner = self.inner().write().await;
                    if inner.detached {
                        return Err(executor_unavailable());
                    }
                    inner.recorder_idle()?;
                    inner.recorder.start(Some(id.clone()), Some(path.clone()))?;
                    inner.core.begin_recording();
                    let max_frames = (inner.max_episode_s
                        * f64::from(inner.config.control_frequency_hz.max(1)))
                    .ceil() as usize
                        + 1;
                    inner.recording = Some(ActiveRecording {
                        id: id.clone(),
                        owner,
                        options,
                        recorded_at: chrono::Utc::now()
                            .to_rfc3339_opts(chrono::SecondsFormat::Millis, true),
                        started: Instant::now(),
                        joint_zero: JointZeroStamp::of(&inner.joint_zero_offsets),
                        max_frames,
                        owner_lost_at: None,
                        dataset_pin: Some(Arc::new(pin)),
                    });
                }
                tracing::info!("Recording {id} started (owner {label})");
                let lock = self.inner().clone();
                let mid = id.clone();
                tokio::spawn(async move { monitor(lock, mid).await });
                Ok((id, label, path))
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// Start a recording owned by `owner`. `legacy_id` / `legacy_path`: the legacy REST
    /// `episode_id` / `output_path` (else a new ULID). Returns `(recording id, owner label)`.
    pub async fn start_recording_as(
        &self,
        owner: RecorderOwner,
        options: RecordOptions,
        legacy_id: Option<String>,
        legacy_path: Option<PathBuf>,
    ) -> Result<(String, String), ArmError> {
        let (id, label, _) = self
            .start_recording_as_with_path(owner, options, legacy_id, legacy_path)
            .await?;
        Ok((id, label))
    }

    /// End the recording: discard, or save with `outcome` (`ok` | `aborted`). Only its owner
    /// (or the monitor) may; others get RECORDER_NOT_OWNER. `writer`: custom parquet writer.
    pub async fn stop_recording_as(
        &self,
        by: Stopper,
        save: bool,
        outcome: Option<&str>,
        writer: Option<ParquetWriter>,
    ) -> Result<Stopped, ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => {
                let ctx = self.executor_recorder_ctx().await?;
                stop_recording_executor(ctx, by, save, outcome, writer, None).await
            }
            OWNER_LEGACY => stop_recording(self.inner(), by, save, outcome, writer).await,
            _ => Err(executor_unavailable()),
        }
    }

    /// The owner `caller` stands for: its own connection's, or (taking the recording back) that
    /// of a lost owner with the same client id. Anything else is not the owner.
    pub async fn claim_recording(&self, caller: &Caller) -> RecorderOwner {
        if self.owner_mode() == OWNER_EXECUTOR {
            return self.executor_recorder().claim(caller);
        }
        let me = RecorderOwner::Client {
            connection: caller.connection.clone(),
            client_id: caller.client_id.clone().unwrap_or_default(),
        };
        let mut inner = self.inner().write().await;
        if inner.detached {
            drop(inner);
            if self.owner_mode() == OWNER_EXECUTOR {
                return self.executor_recorder().claim(caller);
            }
            return me;
        }
        let Some(r) = inner.recording.as_mut() else {
            return me;
        };
        if let RecorderOwner::Client { connection, client_id } = &mut r.owner {
            if *connection == caller.connection {
                return r.owner.clone();
            }
            if r.owner_lost_at.is_some()
                && caller.client_id.as_deref() == Some(client_id.as_str())
            {
                *connection = caller.connection.clone();
                r.owner_lost_at = None;
                tracing::info!(
                    "Recording {} taken back by '{client_id}' on a new connection",
                    r.id
                );
                return r.owner.clone();
            }
        }
        me
    }

    /// A WS connection closed: a recording it owns is discarded after [`OWNER_LOST_DISCARD`]
    /// unless a connection with the same client id takes it back.
    pub async fn recorder_connection_closed(&self, connection: &str) {
        if self.owner_mode() == OWNER_EXECUTOR {
            self.executor_recorder().connection_closed(connection);
            return;
        }
        if self.owner_mode() != OWNER_LEGACY {
            return;
        }
        let mut inner = self.inner().write().await;
        if inner.detached {
            drop(inner);
            if self.owner_mode() == OWNER_EXECUTOR {
                self.executor_recorder().connection_closed(connection);
            }
            return;
        }
        if let Some(r) = inner.recording.as_mut() {
            if matches!(&r.owner, RecorderOwner::Client { connection: c, .. } if c == connection)
                && r.owner_lost_at.is_none()
            {
                r.owner_lost_at = Some(Instant::now());
                tracing::warn!(
                    "Owner of recording {} disconnected; discarding in {} s unless it reconnects",
                    r.id,
                    OWNER_LOST_DISCARD.as_secs()
                );
            }
        }
    }

    /// Owner-aware `recording` field of `GET /api/v1/episodes` and status queries.
    pub async fn recording_status(&self) -> Value {
        match self.owner_mode() {
            OWNER_EXECUTOR => self.executor_recorder().recording_status(),
            OWNER_LEGACY => {
                let inner = self.inner().read().await;
                if inner.detached {
                    drop(inner);
                    if self.owner_mode() == OWNER_EXECUTOR {
                        return self.executor_recorder().recording_status();
                    }
                    return Value::Null;
                }
                inner.recording_status()
            }
            _ => Value::Null,
        }
    }

    /// Owner-aware idle check (`RECORDER_BUSY` when a recording is starting, active, or finalizing).
    pub async fn check_recorder_idle(&self) -> Result<(), ArmError> {
        match self.owner_mode() {
            OWNER_EXECUTOR => self.executor_recorder().check_idle(),
            OWNER_LEGACY => {
                let inner = self.inner().read().await;
                if inner.detached {
                    drop(inner);
                    if self.owner_mode() == OWNER_EXECUTOR {
                        return self.executor_recorder().check_idle();
                    }
                    return Err(executor_unavailable());
                }
                inner.recorder_idle()
            }
            _ => Err(executor_unavailable()),
        }
    }

    /// True if a recording targeting `dataset` is starting, running, or finalizing.
    pub async fn is_recording_dataset(&self, dataset: &str) -> bool {
        if self
            .try_episode_store()
            .await
            .is_ok_and(|s| s.is_dataset_pinned(dataset))
        {
            return true;
        }
        match self.owner_mode() {
            OWNER_EXECUTOR => self.executor_recorder().is_recording_dataset(dataset),
            OWNER_LEGACY => self
                .inner()
                .read()
                .await
                .recording
                .as_ref()
                .is_some_and(|r| r.options.dataset == dataset),
            _ => false,
        }
    }

    /// True if the running (or starting) recording is owned by `owner`.
    pub async fn is_recording_owned_by(&self, owner: &RecorderOwner) -> bool {
        match self.owner_mode() {
            OWNER_EXECUTOR => self.executor_recorder().is_owned_by(owner),
            OWNER_LEGACY => self
                .inner()
                .read()
                .await
                .recording
                .as_ref()
                .is_some_and(|r| &r.owner == owner),
            _ => false,
        }
    }

    /// Test helper: sets `max_episode_s` consistently across Legacy and Executor snapshots.
    pub async fn set_max_episode_s(&self, max_episode_s: f64) {
        if self.owner_mode() == OWNER_LEGACY {
            let mut inner = self.inner().write().await;
            if !inner.detached {
                inner.max_episode_s = max_episode_s;
                let id = Arc::new(ServiceIdentity::from_inner(&inner));
                self.hub().set_service_identity(id);
            }
        }
        self.published_config_hub().update(|cfg| {
            cfg.max_episode_s = max_episode_s;
        });
        if let Some(cfg) = self.published_config_hub().latest() {
            let arm = self
                .hub()
                .latest()
                .map_or(crate::rt_core::ids::ArmId::LOCAL, |s| s.rt.owner.arm);
            let boot = self
                .hub()
                .latest()
                .map_or_else(crate::rt_core::broker::BootId::process, |s| s.rt.owner.boot);
            self.hub()
                .set_service_identity(Arc::new(ServiceIdentity::from_published_config(
                    arm, boot, &cfg,
                )));
        }
    }

    // ---- legacy REST (`/api/v1/dataset/record/*`, owner `legacy-rest`) ----

    pub async fn start_recording_with_path(
        &self,
        episode_id: Option<String>,
        output_path: Option<PathBuf>,
    ) -> Result<(String, PathBuf), ArmError> {
        let (id, _, path) = self
            .start_recording_as_with_path(
                RecorderOwner::LegacyRest,
                RecordOptions::default(),
                episode_id,
                output_path,
            )
            .await?;
        Ok((id, path))
    }

    pub async fn start_recording(
        &self,
        episode_id: Option<String>,
        output_path: Option<PathBuf>,
    ) -> Result<(), ArmError> {
        self.start_recording_with_path(episode_id, output_path)
            .await
            .map(|_| ())
    }

    pub async fn save_recording(&self) -> Result<PathBuf, ArmError> {
        match self
            .stop_recording_as(Stopper::Owner(RecorderOwner::LegacyRest), true, None, None)
            .await?
        {
            Stopped::Saved { path, .. } => Ok(path),
            Stopped::Discarded => Err(ArmError::Internal("recording was discarded".into())),
        }
    }

    /// Save with a caller-supplied parquet writer (tests). The lock is held only to move the
    /// frames out and to record the outcome; the write runs on a blocking thread.
    #[doc(hidden)]
    pub async fn save_recording_with<W>(&self, write: W) -> Result<PathBuf, ArmError>
    where
        W: FnOnce(&Path, &[RecordFrame]) -> Result<(), ParquetRecordingError> + Send + 'static,
    {
        match self
            .stop_recording_as(
                Stopper::Owner(RecorderOwner::LegacyRest),
                true,
                None,
                Some(Box::new(write)),
            )
            .await?
        {
            Stopped::Saved { path, .. } => Ok(path),
            Stopped::Discarded => Err(ArmError::Internal("recording was discarded".into())),
        }
    }

    pub async fn discard_recording(&self) -> Result<(), ArmError> {
        self.stop_recording_as(Stopper::Owner(RecorderOwner::LegacyRest), false, None, None)
            .await
            .map(|_| ())
    }

    pub async fn get_recording_state(&self) -> (EpisodeState, usize) {
        if self.owner_mode() == OWNER_EXECUTOR {
            return self.executor_recorder().get_state();
        }
        let inner = self.inner().read().await;
        if inner.detached {
            drop(inner);
            if self.owner_mode() == OWNER_EXECUTOR {
                return self.executor_recorder().get_state();
            }
            return (EpisodeState::Idle, 0);
        }
        (inner.recorder.state().clone(), inner.recording_frames())
    }

    /// Set the episode directory (`None`: storage off); the catalog loads on first use.
    pub async fn set_episodes_dir(&self, dir: Option<PathBuf>) {
        let store = EpisodeStore::new(dir);
        if self.owner_mode() == OWNER_LEGACY {
            let mut inner = self.inner().write().await;
            if !inner.detached {
                inner.episodes = store.clone();
                inner.refresh_schema_cache();
            }
        }
        self.published_config_hub().update(|cfg| {
            cfg.episodes = store;
        });
    }

    /// Fail-closed episode store lookup: NEVER falls back to `SupervisorLock` when
    /// `owner_mode != OWNER_LEGACY` (returns `503 CONFIG_UNAVAILABLE` if executor exited or
    /// ownership is transitioning).
    pub async fn try_episode_store(&self) -> Result<EpisodeStore, ArmError> {
        self.published_config().await.map(|c| c.episodes.clone())
    }

    pub async fn episode_store(&self) -> EpisodeStore {
        if let Ok(pub_cfg) = self.published_config().await {
            return pub_cfg.episodes.clone();
        }
        if self.owner_mode() != OWNER_LEGACY {
            return self
                .published_config_hub()
                .latest()
                .map(|c| c.episodes.clone())
                .unwrap_or_else(|| EpisodeStore::new(None));
        }
        self.inner().read().await.episodes.clone()
    }
}

/// Collects a stop fence's frames on a blocking thread: [`Fence::collect`] takes the
/// recording ring's consumer mutex, which a preempted dispatcher can hold, so it must not run
/// on an async worker (it would stall unrelated requests on a single-worker runtime).
///
/// [`Fence::collect`]: crate::rt_core::record_ring::Fence::collect
async fn collect_fence(
    fence: Option<crate::rt_core::record_ring::Fence>,
) -> Result<crate::rt_core::record_ring::Finished, ArmError> {
    let Some(fence) = fence else {
        return Ok(Default::default());
    };
    tokio::task::spawn_blocking(move || fence.collect())
        .await
        .map_err(|e| ArmError::Internal(format!("recording frame collection failed: {e}")))
}

async fn stop_recording(
    lock: &Arc<crate::supervisor_lock::SupervisorLock<SupervisorInner>>,
    by: Stopper,
    save: bool,
    outcome: Option<&str>,
    writer: Option<ParquetWriter>,
) -> Result<Stopped, ArmError> {
    let outcome = outcome.unwrap_or("ok");
    if !["ok", "aborted"].contains(&outcome) {
        return Err(rejected(
            400,
            "INVALID_COMMAND",
            "outcome must be \"ok\" or \"aborted\"",
        ));
    }
    let monitor = matches!(by, Stopper::Monitor { .. });
    let st = {
        let inner = lock.read().await;
        let Some(r) = inner.recording.as_ref() else {
            return Err(crate::recording::RecordingError::InvalidState {
                action: if save { "save".into() } else { "discard".into() },
                state: inner.recorder.state().clone(),
            }
            .into());
        };
        if let Stopper::Owner(o) = &by {
            if o != &r.owner {
                return Err(ArmError::detailed(
                    409,
                    "RECORDER_NOT_OWNER",
                    format!("the recording is owned by '{}'", r.owner.label()),
                    json!({"owner": r.owner.label()}),
                ));
            }
        }
        if let Stopper::Monitor { reason, recording } = &by {
            if recording != &r.id {
                return Err(rejected(
                    409,
                    "RECORDING_REPLACED",
                    "the monitored recording ended; another one runs",
                ));
            }
            if *reason == "owner_lost"
                && !r
                    .owner_lost_at
                    .is_some_and(|t| t.elapsed() >= OWNER_LOST_DISCARD)
            {
                return Err(rejected(
                    409,
                    "RECORDING_REPLACED",
                    "the recording owner reconnected before discard",
                ));
            }
        }
        inner.episodes.clone()
    };
    let _capacity_slot = if save && !monitor {
        // refused before the frames leave the recorder, so the recording can still be
        // saved after episodes were deleted, or discarded; reserves a slot against concurrent imports
        let s2 = st.clone();
        Some(
            tokio::task::spawn_blocking(move || s2.reserve_capacity_slot())
                .await
                .map_err(|e| ArmError::Internal(format!("episode check failed: {e}")))??,
        )
    } else {
        None
    };
    let (rec, _path, direct_frames, sidecar_base, fence) = {
        let mut inner = lock.write().await;
        let Some(r) = inner.recording.clone() else {
            return Err(crate::recording::RecordingError::InvalidState {
                action: "save".into(),
                state: inner.recorder.state().clone(),
            }
            .into());
        };
        if let Stopper::Owner(o) = &by {
            if o != &r.owner {
                return Err(rejected(
                    409,
                    "RECORDER_NOT_OWNER",
                    "the recording changed owner",
                ));
            }
        }
        // under the mutation guard: the recording the monitor watched may have been
        // replaced or reclaimed since its check
        if let Stopper::Monitor { reason, recording } = &by {
            if recording != &r.id {
                return Err(rejected(
                    409,
                    "RECORDING_REPLACED",
                    "the monitored recording ended; another one runs",
                ));
            }
            if *reason == "owner_lost"
                && !r
                    .owner_lost_at
                    .is_some_and(|t| t.elapsed() >= OWNER_LOST_DISCARD)
            {
                return Err(rejected(
                    409,
                    "RECORDING_REPLACED",
                    "the recording owner reconnected before discard",
                ));
            }
        }
        if !save {
            inner.recorder.discard()?;
            inner.recording = None;
            // stop fence; the ring's samples are dropped after the guard is released
            let fence = inner.core.end_recording();
            drop(inner);
            if let Err(e) = collect_fence(fence).await {
                tracing::warn!("{e}");
            }
            tracing::info!("Recording {} discarded", r.id);
            return Ok(Stopped::Discarded);
        }
        // refused while the session still runs (no tick recorded yet): it keeps recording
        if inner.recording_frames() == 0 {
            return Err(crate::recording::RecordingError::EmptyEpisode.into());
        }
        let (path, direct) = inner.recorder.begin_finalize()?;
        // stop fence: the ticks up to here; the frames are collected without the guard
        let fence = inner.core.end_recording();
        inner.recording = None;
        let base = (
            inner.joint_names(),
            inner.config.profile_id.clone(),
            inner.config.arm_dof(),
            inner.config.gripper_index(),
            f64::from(inner.config.control_frequency_hz),
            inner.config.active_tool.id.clone(),
            inner.config.active_tool.config_digest.clone(),
        );
        (r, path, direct, base, fence)
    };
    // the ring's frames of this session (a recorder without a session kept them itself)
    let (frames, dropped) = match fence {
        Some(f) => match collect_fence(Some(f)).await {
            Ok(done) => (done.frames, done.dropped),
            Err(e) => {
                // the recorder is finalizing: end it as failed so a new recording can start
                lock.write().await.recorder.finish_save(
                    Vec::new(),
                    Err(ParquetRecordingError::Io(std::io::Error::other(
                        e.to_string(),
                    ))),
                )?;
                return Err(e);
            }
        },
        None => (direct_frames, 0),
    };
    if dropped > 0 {
        tracing::warn!(
            "Recording {}: {} samples dropped on a full recording ring; the episode is incomplete",
            rec.id,
            dropped
        );
    }
    let (joint_names, profile_id, arm_dof, gripper_joint, fps, tool_id, tool_config_digest) =
        sidecar_base;
    let has_gripper_motion = store::episode_has_gripper_motion(&frames, gripper_joint);
    let duration = match (frames.first(), frames.last()) {
        (Some(a), Some(b)) => (b.timestamp - a.timestamp).max(0.0),
        _ => 0.0,
    };
    let (truncated, stopped_by) = match by {
        Stopper::Monitor { reason, .. } => (reason == "max_duration", Some(reason.to_string())),
        _ => (false, None),
    };
    let sidecar = EpisodeSidecar {
        id: rec.id.clone(),
        dataset: rec.options.dataset.clone(),
        task: rec.options.task.clone(),
        source: rec.options.source.clone(),
        sequence_id: rec.options.sequence_id.clone(),
        repeat_index: rec.options.repeat_index,
        outcome: outcome.into(),
        recorded_at: rec.recorded_at.clone(),
        duration_s: duration,
        frames: frames.len(),
        joint_names,
        units: "rad".into(),
        joint_zero: rec.joint_zero.clone(),
        profile_id,
        armd_version: env!("CARGO_PKG_VERSION").into(),
        fps_nominal: fps,
        truncated,
        stopped_by,
        arm_dof: Some(arm_dof),
        gripper_joint,
        recovered: None,
        first_state: frames.first().map(|f| f.observation_state.clone()),
        last_state: frames.last().map(|f| f.observation_state.clone()),
        tool_id: Some(tool_id),
        tool_config_digest: Some(tool_config_digest),
        has_gripper_motion: Some(has_gripper_motion),
        frames_dropped: (dropped > 0).then_some(dropped),
    };
    static NEXT_LEGACY_COMMIT_TOKEN: AtomicU64 = AtomicU64::new(1 << 61);
    let sc = sidecar.clone();
    let enforce_capacity = !monitor && _capacity_slot.is_none();
    let legacy_token = Some(NEXT_LEGACY_COMMIT_TOKEN.fetch_add(1, Ordering::Relaxed));
    let mut cap_slot = _capacity_slot;
    let (frames, result) = tokio::task::spawn_blocking(move || {
        let result = match st.publish_complete_episode_with_slot(
            &sc,
            &frames,
            writer,
            legacy_token,
            enforce_capacity,
            cap_slot.as_mut(),
        ) {
            Ok(p) => Ok(p),
            Err(first_err) => {
                if st.path_of(&sc.id).is_ok_and(|p| p.is_file()) {
                    st.publish_complete_episode(&sc, &frames, None, legacy_token, false)
                } else {
                    Err(first_err)
                }
            }
        };
        (frames, result)
    })
    .await
    .unwrap_or_else(|e| {
        (
            Vec::new(),
            Err(StoreError::Io(std::io::Error::other(format!(
                "recording writer panicked: {e}"
            )))),
        )
    });
    let parquet_result = match &result {
        Ok(_) => Ok(()),
        Err(StoreError::Parquet(e)) => Err(ParquetRecordingError::Corrupted(e.to_string())),
        Err(e) => Err(ParquetRecordingError::Io(std::io::Error::other(
            e.to_string(),
        ))),
    };
    let finish_res = lock
        .write()
        .await
        .recorder
        .finish_save(frames, parquet_result);
    // Preserve typed `StoreError` (e.g. `507 EPISODES_FULL`) instead of wrapping it into `RECORDING_ERROR`.
    let path = result?;
    finish_res?;
    tracing::info!(
        "Recording {} saved ({} frames, {:.1} s{})",
        rec.id,
        sidecar.frames,
        sidecar.duration_s,
        if truncated { ", truncated" } else { "" }
    );
    Ok(Stopped::Saved { sidecar: Box::new(sidecar), path })
}

/// Stops the recording `id` at the length cap or on low space, discards it after its owner
/// left. Ends when the recording ends.
async fn monitor(
    lock: Arc<crate::supervisor_lock::SupervisorLock<SupervisorInner>>,
    id: String,
) {
    let mut last_space: Option<Instant> = None;
    loop {
        tokio::time::sleep(MONITOR_PERIOD).await;
        let (full, lost, st) = {
            let inner = lock.read().await;
            let Some(r) = inner.recording.as_ref().filter(|r| r.id == id) else {
                return;
            };
            let full = inner.recording_frames() >= r.max_frames
                || r.started.elapsed().as_secs_f64() >= inner.max_episode_s + 1.0;
            (
                full,
                r.owner_lost_at
                    .is_some_and(|t| t.elapsed() >= OWNER_LOST_DISCARD),
                inner.episodes.clone(),
            )
        };
        let action = if lost {
            Some((false, "owner_lost"))
        } else if full {
            Some((true, "max_duration"))
        } else if last_space.is_none_or(|t| t.elapsed() >= SPACE_CHECK_PERIOD) {
            last_space = Some(Instant::now());
            let free = tokio::task::spawn_blocking(move || st.free_bytes())
                .await
                .ok()
                .flatten();
            free.filter(|f| *f < store::STOP_MIN_FREE_BYTES)
                .map(|_| (true, "low_space"))
        } else {
            None
        };
        let Some((save, reason)) = action else {
            continue;
        };
        // only this recording: a newer one is not touched (stop_recording checks it again
        // under the write guard)
        if !lock
            .read()
            .await
            .recording
            .as_ref()
            .is_some_and(|r| r.id == id)
        {
            return;
        }
        match stop_recording(
            &lock,
            Stopper::Monitor { reason, recording: id.clone() },
            save,
            None,
            None,
        )
        .await
        {
            Ok(_) => {
                tracing::warn!(
                    "Recording {id} {} by armd ({reason})",
                    if save { "saved" } else { "discarded" }
                );
                return;
            }
            Err(ArmError::Rejected { ref code, .. }) if code == "RECORDING_REPLACED" => {
                if !lock
                    .read()
                    .await
                    .recording
                    .as_ref()
                    .is_some_and(|r| r.id == id)
                {
                    return;
                }
            }
            Err(e) => {
                tracing::warn!("Recording {id}: stop ({reason}) failed: {e}");
                return;
            }
        }
    }
}

// ---- lease-free WS commands ----

/// Teach commands that need no motion lease (recorder ownership or nothing).
/// WS command paths handled without holding a motion lease.
pub const LEASE_FREE_PATHS: &[&str] = &[
    "/api/v1/record/start",
    "/api/v1/record/stop",
    "/api/v1/episodes/update",
    "/api/v1/episodes/delete",
    "/api/v1/datasets/create",
    "/api/v1/datasets/delete",
];

pub fn is_lease_free(path: &str) -> bool {
    LEASE_FREE_PATHS.contains(&path)
}

fn str_field<'a>(body: &'a Value, key: &str) -> Result<Option<&'a str>, ArmError> {
    match body.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) => Ok(Some(s.as_str())),
        Some(_) => Err(rejected(
            400,
            "INVALID_COMMAND",
            format!("{key} must be a string"),
        )),
    }
}

fn required<'a>(body: &'a Value, key: &str) -> Result<&'a str, ArmError> {
    str_field(body, key)?
        .ok_or_else(|| rejected(400, "INVALID_COMMAND", format!("{key} is required")))
}

/// Who sent a lease-free command.
#[derive(Debug, Clone)]
pub struct Caller {
    /// Internal connection id (never disclosed).
    pub connection: String,
    /// `client_id` of the message, else the label of the lease this connection holds.
    pub client_id: Option<String>,
}

/// Run a lease-free command; Ok = fields of the ack.
pub async fn lease_free(
    sup: &Arc<ArmSupervisor>,
    caller: &Caller,
    path: &str,
    body: &Value,
) -> Result<Value, ArmError> {
    if !body.is_object() {
        return Err(rejected(
            400,
            "INVALID_COMMAND",
            "Command body must be an object",
        ));
    }
    sup.ensure_episodes_loaded().await?;
    let st = sup.try_episode_store().await?;
    if !st.enabled() {
        return Err(StoreError::Disabled.into());
    }
    match path {
        "/api/v1/record/start" => {
            let client_id = caller
                .client_id
                .clone()
                .filter(|c| !c.is_empty() && c.len() <= 128)
                .ok_or_else(|| {
                    rejected(
                        400,
                        "INVALID_COMMAND",
                        "client_id (1-128 characters) is required: in the message, or from acquire_lease on this connection",
                    )
                })?;
            let options = RecordOptions {
                dataset: str_field(body, "dataset")?
                    .unwrap_or(store::DEFAULT_DATASET)
                    .to_string(),
                task: str_field(body, "task")?.unwrap_or("").to_string(),
                source: str_field(body, "source")?.unwrap_or("sdk").to_string(),
                sequence_id: None,
                repeat_index: None,
            };
            let owner = RecorderOwner::Client {
                connection: caller.connection.clone(),
                client_id,
            };
            let (id, owner) = sup.start_recording_as(owner, options, None, None).await?;
            Ok(json!({"recording_id": id, "owner": owner}))
        }
        "/api/v1/record/stop" => {
            let save = match body.get("save") {
                Some(Value::Bool(b)) => *b,
                _ => {
                    return Err(rejected(
                        400,
                        "INVALID_COMMAND",
                        "save must be a boolean",
                    ))
                }
            };
            let outcome = str_field(body, "outcome")?.map(str::to_string);
            let owner = sup.claim_recording(caller).await;
            match sup
                .stop_recording_as(Stopper::Owner(owner), save, outcome.as_deref(), None)
                .await?
            {
                Stopped::Discarded => Ok(json!({"discarded": true})),
                Stopped::Saved { sidecar, .. } => {
                    let svc = sup.read_services(ReadRequirements::identity()).await?;
                    Ok(json!({"episode": st.view(&sidecar.id, &svc.identity.joint_zero_offsets)}))
                }
            }
        }
        "/api/v1/episodes/update" => {
            let id = required(body, "id")?.to_string();
            let task = str_field(body, "task")?.map(str::to_string);
            let dataset = str_field(body, "dataset")?.map(str::to_string);
            let svc = sup.read_services(ReadRequirements::identity()).await?;
            let offsets = svc.identity.joint_zero_offsets.clone();
            let s2 = st.clone();
            let sc = blocking(move || s2.update(&id, task.as_deref(), dataset.as_deref())).await?;
            let stale = sc.joint_zero.stale_against(&offsets);
            let mut v = serde_json::to_value(&sc).unwrap_or_default();
            v["stale"] = json!(stale);
            Ok(json!({"episode": v}))
        }
        "/api/v1/episodes/delete" => {
            let id = required(body, "id")?.to_string();
            let Some(guard) = sup.take_episode_for_delete_claim(&id).await? else {
                return Err(StoreError::NotFound(id).into());
            };
            let s2 = st.clone();
            let id2 = id.clone();
            blocking(move || s2.delete_files_with_claim_guard(&id2, guard)).await?;
            Ok(json!({"deleted": id}))
        }
        "/api/v1/datasets/create" => {
            let name = required(body, "name")?.to_string();
            let description = str_field(body, "description")?.unwrap_or("").to_string();
            let s2 = st.clone();
            let d = blocking(move || s2.create_dataset(&name, &description)).await?;
            Ok(json!({"dataset": d}))
        }
        "/api/v1/datasets/delete" => {
            let name = required(body, "name")?.to_string();
            let force = match body.get("force") {
                None | Some(Value::Null) => false,
                Some(Value::Bool(b)) => *b,
                Some(_) => {
                    return Err(rejected(
                        400,
                        "INVALID_COMMAND",
                        "force must be a boolean",
                    ))
                }
            };
            if sup.is_recording_dataset(&name).await {
                return Err(ArmError::detailed(
                    409,
                    "RECORDER_BUSY",
                    "a recording into this dataset is running",
                    json!({}),
                ));
            }
            if force {
                let ids = st.episodes_of(&name);
                if !st.has_dataset(&name) && ids.is_empty() && !st.has_pending_delete_files(&name) {
                    return Err(StoreError::DatasetNotFound(name).into());
                }
                let (taken, del_guard) = sup.take_dataset_episodes_for_delete(&name).await?;
                let s2 = st.clone();
                let deleted =
                    blocking(move || s2.delete_dataset_taken_files(&name, &taken, del_guard))
                        .await?;
                Ok(json!({"deleted_episodes": deleted}))
            } else {
                let s2 = st.clone();
                let deleted = blocking(move || s2.delete_dataset(&name, false)).await?;
                Ok(json!({"deleted_episodes": deleted}))
            }
        }
        _ => Err(rejected(400, "INVALID_COMMAND", "Unknown command")),
    }
}

async fn blocking<T: Send + 'static>(
    f: impl FnOnce() -> Result<T, StoreError> + Send + 'static,
) -> Result<T, ArmError> {
    tokio::task::spawn_blocking(f)
        .await
        .map_err(|e| ArmError::Internal(format!("episode task failed: {e}")))?
        .map_err(ArmError::from)
}

impl SupervisorInner {
    /// Whether the episode `id` may be deleted now: IN_USE `{sequences}` when a sequence
    /// references it or the running sequence pinned it.
    pub fn episode_delete_allowed(&self, id: &str) -> Result<(), ArmError> {
        let users = self.sequences_using_episode(id);
        if users.is_empty() {
            return Ok(());
        }
        Err(ArmError::detailed(
            409,
            "IN_USE",
            format!("Episode '{id}' is used by {} sequence(s)", users.len()),
            json!({"sequences": users}),
        ))
    }
}

/// `command_ack` / `command_reject` of a lease-free command.
pub fn reply(request_id: &Value, result: Result<Value, ArmError>) -> Value {
    match result {
        Ok(fields) => {
            let mut ack =
                json!({"type":"command_ack", "request_id":request_id, "status":"accepted"});
            if let Value::Object(m) = fields {
                for (k, v) in m {
                    ack[k] = v;
                }
            }
            ack
        }
        Err(e) => {
            let api = crate::api::rest::ApiError::from(e);
            let mut r = json!({"type":"command_reject", "request_id":request_id, "status":api.status.as_u16(),
                "code":api.code, "error":api.error, "message":api.message});
            if let Some(Value::Object(m)) = api.details.map(|d| *d) {
                for (k, v) in m {
                    r[k] = v;
                }
            }
            r
        }
    }
}
