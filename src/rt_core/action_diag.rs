//! Bounded lock-free off-RT action diagnostic patch stream over [`crate::rt_core::stream`].
//!
//! Grasp and macro orchestrators run off-RT and emit compact, `Copy` [`ActionDiagPatch`] records
//! through `rt_core::stream::channel`. [`ActionDiagStore`] is the single off-RT consumer owned by
//! [`crate::rt_core::publish::StateHub`]:
//! - Zero `SupervisorLock` and zero allocations/locks on `armd-control`.
//! - Lossless wire parity for open-ended strings (`WorkspaceViolationDetail`, long reasons,
//!   `approach_flipped`) via per-action off-RT text/blob slots reclaimed on FIFO eviction.
//! - Atomic grouped staging for `height_estimates` / `yaw_estimates` / `width_estimates` so a
//!   dropped frame under overflow never exposes a half-replaced list.
//! - Single terminal freeze cutoff: once an `action_id` is sealed at the terminal capture or
//!   terminal observation boundary, all captured, synthesized, and ordinary observations receive
//!   the exact same frozen overlay and late patches for that `action_id` are ignored.

use std::collections::{BTreeMap, VecDeque};
use std::time::Instant;

use crate::kinematics::WorkspaceViolationDetail;
use crate::model::ArmObservation;
use crate::motion::{ActionStatus, EstimateEntry, MacroMotionDiag};
use crate::rt_core::stream::{self, StreamConsumer, StreamProducer, StreamStats};

/// Default capacity of the SPSC diagnostic patch ring.
pub const ACTION_DIAG_STREAM_CAP: usize = 512;

/// Compact inline UTF-8 label (`Copy`, zero heap allocation).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShortLabel<const N: usize> {
    len: u8,
    buf: [u8; N],
}

impl<const N: usize> ShortLabel<N> {
    pub fn try_new(s: &str) -> Option<Self> {
        if s.len() > N || N > u8::MAX as usize {
            return None;
        }
        let mut buf = [0u8; N];
        buf[..s.len()].copy_from_slice(s.as_bytes());
        Some(Self {
            len: s.len() as u8,
            buf,
        })
    }

    pub fn new_truncated(s: &str) -> Self {
        let mut n = s.len().min(N).min(u8::MAX as usize);
        while !s.is_char_boundary(n) {
            n -= 1;
        }
        let mut buf = [0u8; N];
        buf[..n].copy_from_slice(&s.as_bytes()[..n]);
        Self { len: n as u8, buf }
    }

    pub fn as_str(&self) -> &str {
        let n = usize::from(self.len).min(N);
        std::str::from_utf8(&self.buf[..n]).unwrap_or("")
    }
}

/// Lossless `Copy` string reference: inline up to 48 bytes, else interned in `ActionDiagStore`'s
/// per-action off-RT text table (reclaimed when the action entry is evicted).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagStr {
    Inline(ShortLabel<48>),
    Interned(u32),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BorderKind {
    Cut,
    Shifted,
}

impl BorderKind {
    const fn as_str(self) -> &'static str {
        match self {
            Self::Cut => "cut",
            Self::Shifted => "shifted",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PostLiftKind {
    Held,
    Regripped,
    Dropped,
}

impl PostLiftKind {
    const fn as_str(self) -> &'static str {
        match self {
            Self::Held => "held",
            Self::Regripped => "regripped",
            Self::Dropped => "dropped",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApproachKind {
    SameSide,
    Direct,
}

impl ApproachKind {
    const fn as_str(self) -> &'static str {
        match self {
            Self::SameSide => "same_side",
            Self::Direct => "direct",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EstimateListKind {
    Height,
    Yaw,
    Width,
}

/// Compact `Copy` diagnostic update sent over [`crate::rt_core::stream`].
#[derive(Debug, Clone, Copy)]
pub struct ActionDiagPatch {
    pub action_id: u64,
    pub kind: ActionDiagKind,
}

#[derive(Debug, Clone, Copy)]
pub enum ActionDiagKind {
    Phase {
        phase: ShortLabel<32>,
        progress: f32,
        iteration: Option<u32>,
        error_px: Option<f32>,
        at: Instant,
    },
    AcquireHeight {
        height_m: f32,
    },
    FinalOffset {
        offset_px: [f32; 2],
    },
    Border {
        kind: BorderKind,
        shift_px: Option<[f32; 2]>,
    },
    HeightDecision {
        source: ShortLabel<32>,
        object_height_m: Option<f32>,
        reason: Option<DiagStr>,
    },
    HeightReason {
        reason: DiagStr,
    },
    EstimateBatchEmpty {
        list: EstimateListKind,
        batch_gen: u16,
    },
    EstimateBatchItem {
        list: EstimateListKind,
        batch_gen: u16,
        index: u8,
        total_len: u8,
        source: ShortLabel<32>,
        value: Option<f32>,
        used: bool,
        record_only: bool,
        reason: Option<DiagStr>,
    },
    DepthMeasurement {
        anchor: ShortLabel<32>,
        object_depth_m: f32,
        object_length_m: Option<f32>,
        jaw_clear: Option<bool>,
    },
    CloseHeight {
        close_height_m: f32,
    },
    Align {
        align: ShortLabel<32>,
        turn_rad: Option<f32>,
    },
    AngleSpread {
        spread_deg: f32,
    },
    GraspWidth {
        width_m: f32,
    },
    PreOpen {
        decision: DiagStr,
        pre_open_m: Option<f32>,
    },
    Recentre {
        status: DiagStr,
        iterations: Option<u32>,
    },
    ProbeUp {
        dz_m: f32,
    },
    ProbeLateralStart,
    ProbeLateralBaseline {
        baseline_m: f32,
        err_mm: f32,
    },
    /// Exact Legacy absence semantics: `descend_contact` is left `None` unless
    /// `contact_height_m.is_some()` (never unconditionally written as `Some(false)`).
    DescendContact {
        peak_residual_nm: Option<[f32; 2]>,
        peak_tracking_rad: Option<f32>,
        contact_height_m: Option<f32>,
    },
    HeldWidth {
        held_width_m: f32,
    },
    PostLift {
        kind: PostLiftKind,
        gripper_gap_rad: Option<f32>,
    },
    Approach {
        kind: ApproachKind,
        reason: Option<DiagStr>,
        signs: [i8; 6],
        signs_len: u8,
        flipped_blob: Option<u32>,
    },
    ApproachInterrupted {
        reason: ShortLabel<32>,
    },
    MotionDiag {
        phase: ShortLabel<32>,
        ms: u32,
        track_mm: f32,
        track_deg: f32,
        goal_mm: f32,
        goal_deg: f32,
        contact: Option<bool>,
    },
    WorkspaceViolation {
        blob: u32,
    },
}

// Keep stream slots compact so a 512-slot ring fits well under 64 KiB.
const _: () = assert!(std::mem::size_of::<ActionDiagPatch>() <= 128);

/// Off-RT side-table blobs scoped to a single `action_id` and freed when the entry is evicted.
#[derive(Debug, Clone)]
enum DiagBlob {
    Text(String),
    FlippedJoints(Vec<String>),
    WorkspaceViolation(WorkspaceViolationDetail),
}

#[derive(Debug, Clone, Default)]
struct EstimateStaging {
    batch_gen: u16,
    total_len: u8,
    received: u8,
    items: Vec<Option<EstimateEntry>>,
}

impl EstimateStaging {
    fn reset(&mut self, batch_gen: u16, total_len: u8) {
        self.batch_gen = batch_gen;
        self.total_len = total_len;
        self.received = 0;
        self.items.clear();
        self.items.resize(usize::from(total_len), None);
    }

    fn push(
        &mut self,
        batch_gen: u16,
        index: u8,
        total_len: u8,
        entry: EstimateEntry,
    ) -> Option<Vec<EstimateEntry>> {
        if total_len == 0 || index >= total_len {
            return None;
        }
        if self.batch_gen != batch_gen || self.total_len != total_len {
            self.reset(batch_gen, total_len);
        }
        let slot = &mut self.items[usize::from(index)];
        if slot.is_none() {
            self.received = self.received.saturating_add(1);
        }
        *slot = Some(entry);
        if self.received == self.total_len {
            let mut out = Vec::with_capacity(usize::from(total_len));
            for item in self.items.drain(..) {
                if let Some(e) = item {
                    out.push(e);
                }
            }
            self.total_len = 0;
            self.received = 0;
            return Some(out);
        }
        None
    }
}

/// Accumulated off-RT diagnostic overlay for one `action_id`.
#[derive(Debug, Clone)]
pub(crate) struct ActionDiagEntry {
    pub(crate) action_id: u64,
    pub(crate) sealed: bool,
    current_phase: Option<(String, Instant)>,
    phase: Option<crate::model::HolderLabel>,
    progress: Option<f32>,
    iteration: Option<u32>,
    error_px: Option<f32>,
    final_offset_px: Option<[f32; 2]>,
    acquire_height_m: Option<f32>,
    align: Option<String>,
    align_turn_rad: Option<f32>,
    angle_spread_deg: Option<f32>,
    grasp_width_m: Option<f32>,
    height_source: Option<String>,
    height_reason: Option<String>,
    object_height_m: Option<f32>,
    height_estimates: Vec<EstimateEntry>,
    yaw_estimates: Vec<EstimateEntry>,
    width_estimates: Vec<EstimateEntry>,
    height_staging: EstimateStaging,
    yaw_staging: EstimateStaging,
    width_staging: EstimateStaging,
    close_height_m: Option<f32>,
    probe_dz_m: Option<f32>,
    depth_anchor: Option<String>,
    object_depth_m: Option<f32>,
    object_length_m: Option<f32>,
    jaw_clear: Option<bool>,
    border: Option<String>,
    border_shift_px: Option<[f32; 2]>,
    probe_mode: Option<String>,
    lateral_baseline_m: Option<f32>,
    lateral_baseline_err_mm: Option<f32>,
    recentre: Option<String>,
    recentre_iterations: Option<u32>,
    descend_peak_residual_nm: Option<[f32; 2]>,
    descend_peak_tracking_rad: Option<f32>,
    descend_contact: Option<bool>,
    descend_contact_height_m: Option<f32>,
    post_lift: Option<String>,
    gripper_gap_rad: Option<f32>,
    pre_open: Option<String>,
    pre_open_m: Option<f32>,
    held_width_m: Option<f32>,
    approach: Option<String>,
    approach_reason: Option<String>,
    approach_signs: Option<Vec<i8>>,
    approach_flipped: Option<Vec<String>>,
    phase_ms: BTreeMap<String, u32>,
    motions: Vec<MacroMotionDiag>,
    constraint_id: Option<String>,
    geometry_id: Option<String>,
    violation_phase: Option<String>,
    reason: Option<String>,
    blobs: Vec<DiagBlob>,
}

impl ActionDiagEntry {
    fn new(action_id: u64) -> Self {
        Self {
            action_id,
            sealed: false,
            current_phase: None,
            phase: None,
            progress: None,
            iteration: None,
            error_px: None,
            final_offset_px: None,
            acquire_height_m: None,
            align: None,
            align_turn_rad: None,
            angle_spread_deg: None,
            grasp_width_m: None,
            height_source: None,
            height_reason: None,
            object_height_m: None,
            height_estimates: Vec::new(),
            yaw_estimates: Vec::new(),
            width_estimates: Vec::new(),
            height_staging: EstimateStaging::default(),
            yaw_staging: EstimateStaging::default(),
            width_staging: EstimateStaging::default(),
            close_height_m: None,
            probe_dz_m: None,
            depth_anchor: None,
            object_depth_m: None,
            object_length_m: None,
            jaw_clear: None,
            border: None,
            border_shift_px: None,
            probe_mode: None,
            lateral_baseline_m: None,
            lateral_baseline_err_mm: None,
            recentre: None,
            recentre_iterations: None,
            descend_peak_residual_nm: None,
            descend_peak_tracking_rad: None,
            descend_contact: None,
            descend_contact_height_m: None,
            post_lift: None,
            gripper_gap_rad: None,
            pre_open: None,
            pre_open_m: None,
            held_width_m: None,
            approach: None,
            approach_reason: None,
            approach_signs: None,
            approach_flipped: None,
            phase_ms: BTreeMap::new(),
            motions: Vec::new(),
            constraint_id: None,
            geometry_id: None,
            violation_phase: None,
            reason: None,
            blobs: Vec::new(),
        }
    }

    fn resolve_str(&self, s: DiagStr) -> String {
        match s {
            DiagStr::Inline(l) => l.as_str().to_owned(),
            DiagStr::Interned(slot) => match self.blobs.get(slot as usize) {
                Some(DiagBlob::Text(t)) => t.clone(),
                _ => String::new(),
            },
        }
    }

    fn seal(&mut self, now: Instant) {
        if self.sealed {
            return;
        }
        if let Some((prev_phase, start)) = self.current_phase.take() {
            let ms = now.saturating_duration_since(start).as_millis() as u32;
            *self.phase_ms.entry(prev_phase).or_insert(0) += ms;
        }
        self.sealed = true;
    }

    fn apply_patch(&mut self, kind: ActionDiagKind) {
        if self.sealed {
            return;
        }
        match kind {
            ActionDiagKind::Phase {
                phase,
                progress,
                iteration,
                error_px,
                at,
            } => {
                let phase_str = phase.as_str();
                let changed = self
                    .current_phase
                    .as_ref()
                    .is_none_or(|(p, _)| p.as_str() != phase_str);
                if changed {
                    if let Some((prev_phase, start)) = self.current_phase.take() {
                        let ms = at.saturating_duration_since(start).as_millis() as u32;
                        *self.phase_ms.entry(prev_phase).or_insert(0) += ms;
                    }
                    self.current_phase = Some((phase_str.to_owned(), at));
                }
                self.phase = Some(crate::model::HolderLabel::new(phase_str));
                self.progress = Some(progress);
                if iteration.is_some() {
                    self.iteration = iteration;
                }
                if error_px.is_some() {
                    self.error_px = error_px;
                }
            }
            ActionDiagKind::AcquireHeight { height_m } => {
                self.acquire_height_m = Some(height_m);
            }
            ActionDiagKind::FinalOffset { offset_px } => {
                self.final_offset_px = Some(offset_px);
            }
            ActionDiagKind::Border { kind, shift_px } => {
                self.border = Some(kind.as_str().to_owned());
                self.border_shift_px = shift_px;
            }
            ActionDiagKind::HeightDecision {
                source,
                object_height_m,
                reason,
            } => {
                self.height_source = Some(source.as_str().to_owned());
                if object_height_m.is_some() {
                    self.object_height_m = object_height_m;
                }
                if let Some(r) = reason {
                    self.height_reason = Some(self.resolve_str(r));
                }
            }
            ActionDiagKind::HeightReason { reason } => {
                self.height_reason = Some(self.resolve_str(reason));
            }
            ActionDiagKind::EstimateBatchEmpty { list, batch_gen } => match list {
                EstimateListKind::Height => {
                    self.height_staging.reset(batch_gen, 0);
                    self.height_estimates.clear();
                }
                EstimateListKind::Yaw => {
                    self.yaw_staging.reset(batch_gen, 0);
                    self.yaw_estimates.clear();
                }
                EstimateListKind::Width => {
                    self.width_staging.reset(batch_gen, 0);
                    self.width_estimates.clear();
                }
            },
            ActionDiagKind::EstimateBatchItem {
                list,
                batch_gen,
                index,
                total_len,
                source,
                value,
                used,
                record_only,
                reason,
            } => {
                let entry = EstimateEntry {
                    source: source.as_str().to_owned(),
                    value,
                    used,
                    record_only,
                    reason: reason.map(|r| self.resolve_str(r)),
                };
                match list {
                    EstimateListKind::Height => {
                        if let Some(done) =
                            self.height_staging.push(batch_gen, index, total_len, entry)
                        {
                            self.height_estimates = done;
                        }
                    }
                    EstimateListKind::Yaw => {
                        if let Some(done) =
                            self.yaw_staging.push(batch_gen, index, total_len, entry)
                        {
                            self.yaw_estimates = done;
                        }
                    }
                    EstimateListKind::Width => {
                        if let Some(done) =
                            self.width_staging.push(batch_gen, index, total_len, entry)
                        {
                            self.width_estimates = done;
                        }
                    }
                }
            }
            ActionDiagKind::DepthMeasurement {
                anchor,
                object_depth_m,
                object_length_m,
                jaw_clear,
            } => {
                self.depth_anchor = Some(anchor.as_str().to_owned());
                self.object_depth_m = Some(object_depth_m);
                self.object_length_m = object_length_m;
                self.jaw_clear = jaw_clear;
            }
            ActionDiagKind::CloseHeight { close_height_m } => {
                self.close_height_m = Some(close_height_m);
            }
            ActionDiagKind::Align { align, turn_rad } => {
                self.align = Some(align.as_str().to_owned());
                self.align_turn_rad = turn_rad;
            }
            ActionDiagKind::AngleSpread { spread_deg } => {
                self.angle_spread_deg = Some(spread_deg);
            }
            ActionDiagKind::GraspWidth { width_m } => {
                self.grasp_width_m = Some(width_m);
            }
            ActionDiagKind::PreOpen {
                decision,
                pre_open_m,
            } => {
                self.pre_open = Some(self.resolve_str(decision));
                if pre_open_m.is_some() {
                    self.pre_open_m = pre_open_m;
                }
            }
            ActionDiagKind::Recentre { status, iterations } => {
                self.recentre = Some(self.resolve_str(status));
                self.recentre_iterations = iterations;
            }
            ActionDiagKind::ProbeUp { dz_m } => {
                self.probe_dz_m = Some(dz_m);
                self.probe_mode = Some("up".to_owned());
            }
            ActionDiagKind::ProbeLateralStart => {
                self.probe_mode = Some("lateral".to_owned());
            }
            ActionDiagKind::ProbeLateralBaseline { baseline_m, err_mm } => {
                self.lateral_baseline_m = Some(baseline_m);
                self.lateral_baseline_err_mm = Some(err_mm);
            }
            ActionDiagKind::DescendContact {
                peak_residual_nm,
                peak_tracking_rad,
                contact_height_m,
            } => {
                self.descend_peak_residual_nm = peak_residual_nm;
                self.descend_peak_tracking_rad = peak_tracking_rad;
                if let Some(h) = contact_height_m {
                    self.descend_contact = Some(true);
                    self.descend_contact_height_m = Some(h);
                }
            }
            ActionDiagKind::HeldWidth { held_width_m } => {
                self.held_width_m = Some(held_width_m);
            }
            ActionDiagKind::PostLift {
                kind,
                gripper_gap_rad,
            } => {
                self.post_lift = Some(kind.as_str().to_owned());
                if gripper_gap_rad.is_some() {
                    self.gripper_gap_rad = gripper_gap_rad;
                }
            }
            ActionDiagKind::Approach {
                kind,
                reason,
                signs,
                signs_len,
                flipped_blob,
            } => {
                self.approach = Some(kind.as_str().to_owned());
                self.approach_reason = reason.map(|r| self.resolve_str(r));
                let n = usize::from(signs_len).min(signs.len());
                self.approach_signs = (n > 0).then(|| signs[..n].to_vec());
                self.approach_flipped = flipped_blob.and_then(|slot| match self.blobs.get(slot as usize) {
                    Some(DiagBlob::FlippedJoints(v)) if !v.is_empty() => Some(v.clone()),
                    _ => None,
                });
            }
            ActionDiagKind::ApproachInterrupted { reason } => {
                if self.approach.as_deref() == Some("same_side") {
                    self.approach = Some("direct".to_owned());
                    self.approach_reason = Some(reason.as_str().to_owned());
                    self.approach_signs = None;
                    self.approach_flipped = None;
                }
            }
            ActionDiagKind::MotionDiag {
                phase,
                ms,
                track_mm,
                track_deg,
                goal_mm,
                goal_deg,
                contact,
            } => {
                if self.motions.len() < 32 {
                    self.motions.push(MacroMotionDiag {
                        phase: phase.as_str().to_owned(),
                        ms,
                        track_mm,
                        track_deg,
                        goal_mm,
                        goal_deg,
                        contact: contact.or(self.descend_contact),
                    });
                }
            }
            ActionDiagKind::WorkspaceViolation { blob } => {
                if let Some(DiagBlob::WorkspaceViolation(detail)) = self.blobs.get(blob as usize) {
                    self.constraint_id = detail.constraint_id.clone();
                    self.geometry_id = detail.geometry_id.clone();
                    self.violation_phase = detail.violation_phase.clone();
                    self.reason = detail.reason.clone();
                }
            }
        }
    }

    /// Overlays accumulated diagnostics onto `action` without ever overwriting authoritative RT
    /// terminal fields (`state`, `code`, `message`, `holding`, `finished_at`, or `done`
    /// `phase`/`progress`). On `failed`/`cancelled`, the last running `phase`/`progress` is
    /// preserved just as on Legacy.
    pub(crate) fn apply_to_action(&self, a: &mut ActionStatus) {
        if a.state.as_str() != "done" {
            if let Some(p) = self.phase {
                a.phase = Some(p);
            }
            if let Some(prog) = self.progress {
                a.progress = prog;
            }
        }
        if a.iteration.is_none() {
            a.iteration = self.iteration;
        } else if self.iteration.is_some() {
            a.iteration = self.iteration;
        }
        if self.error_px.is_some() {
            a.error_px = self.error_px;
        }
        if a.final_offset_px.is_none() {
            a.final_offset_px = self.final_offset_px;
        }
        if a.acquire_height_m.is_none() {
            a.acquire_height_m = self.acquire_height_m;
        }
        if a.align.is_none() {
            a.align = self.align.clone();
        }
        if a.align_turn_rad.is_none() {
            a.align_turn_rad = self.align_turn_rad;
        }
        if a.angle_spread_deg.is_none() {
            a.angle_spread_deg = self.angle_spread_deg;
        }
        if a.grasp_width_m.is_none() {
            a.grasp_width_m = self.grasp_width_m;
        }
        if a.height_source.is_none() {
            a.height_source = self.height_source.clone();
        }
        if a.height_reason.is_none() {
            a.height_reason = self.height_reason.clone();
        }
        if a.object_height_m.is_none() {
            a.object_height_m = self.object_height_m;
        }
        if a.height_estimates.is_empty() && !self.height_estimates.is_empty() {
            a.height_estimates = self.height_estimates.clone();
        }
        if a.yaw_estimates.is_empty() && !self.yaw_estimates.is_empty() {
            a.yaw_estimates = self.yaw_estimates.clone();
        }
        if a.width_estimates.is_empty() && !self.width_estimates.is_empty() {
            a.width_estimates = self.width_estimates.clone();
        }
        if a.close_height_m.is_none() {
            a.close_height_m = self.close_height_m;
        }
        if a.probe_dz_m.is_none() {
            a.probe_dz_m = self.probe_dz_m;
        }
        if a.depth_anchor.is_none() {
            a.depth_anchor = self.depth_anchor.clone();
        }
        if a.object_depth_m.is_none() {
            a.object_depth_m = self.object_depth_m;
        }
        if a.object_length_m.is_none() {
            a.object_length_m = self.object_length_m;
        }
        if a.jaw_clear.is_none() {
            a.jaw_clear = self.jaw_clear;
        }
        if a.border.is_none() {
            a.border = self.border.clone();
        }
        if a.border_shift_px.is_none() {
            a.border_shift_px = self.border_shift_px;
        }
        if a.probe_mode.is_none() {
            a.probe_mode = self.probe_mode.clone();
        }
        if a.lateral_baseline_m.is_none() {
            a.lateral_baseline_m = self.lateral_baseline_m;
        }
        if a.lateral_baseline_err_mm.is_none() {
            a.lateral_baseline_err_mm = self.lateral_baseline_err_mm;
        }
        if a.recentre.is_none() {
            a.recentre = self.recentre.clone();
        }
        if a.recentre_iterations.is_none() {
            a.recentre_iterations = self.recentre_iterations;
        }
        if a.descend_peak_residual_nm.is_none() {
            a.descend_peak_residual_nm = self.descend_peak_residual_nm;
        }
        if a.descend_peak_tracking_rad.is_none() {
            a.descend_peak_tracking_rad = self.descend_peak_tracking_rad;
        }
        if a.descend_contact.is_none() {
            a.descend_contact = self.descend_contact;
        }
        if a.descend_contact_height_m.is_none() {
            a.descend_contact_height_m = self.descend_contact_height_m;
        }
        if a.post_lift.is_none() {
            a.post_lift = self.post_lift.clone();
        }
        if a.gripper_gap_rad.is_none() {
            a.gripper_gap_rad = self.gripper_gap_rad;
        }
        if a.pre_open.is_none() {
            a.pre_open = self.pre_open.clone();
        }
        if a.pre_open_m.is_none() {
            a.pre_open_m = self.pre_open_m;
        }
        if a.held_width_m.is_none() {
            a.held_width_m = self.held_width_m;
        }
        if a.approach.is_none() {
            a.approach = self.approach.clone();
            a.approach_reason = self.approach_reason.clone();
            a.approach_signs = self.approach_signs.clone();
            a.approach_flipped = self.approach_flipped.clone();
        }
        if a.phase_ms.is_empty() && !self.phase_ms.is_empty() {
            a.phase_ms = self.phase_ms.clone();
        }
        if a.motions.is_empty() && !self.motions.is_empty() {
            a.motions = self.motions.clone();
        }
        if a.constraint_id.is_none() {
            a.constraint_id = self.constraint_id.clone();
        }
        if a.geometry_id.is_none() {
            a.geometry_id = self.geometry_id.clone();
        }
        if a.violation_phase.is_none() {
            a.violation_phase = self.violation_phase.clone();
        }
        if a.reason.is_none() {
            a.reason = self.reason.clone();
        }
    }
}

/// Single off-RT consumer and bounded diagnostic overlay table embedded in [`StateHub`].
/// Always a leaf lock (`state -> action_diag`, `terminal_capture.rx -> action_diag`,
/// `maint_client -> action_diag`), never called on `armd-control`.
pub(crate) struct ActionDiagStore {
    rx: StreamConsumer<ActionDiagPatch>,
    entries: VecDeque<ActionDiagEntry>,
    next_batch_gen: u16,
}

impl ActionDiagStore {
    pub(crate) fn new_channel(cap: usize) -> (StreamProducer<ActionDiagPatch>, Self) {
        let (tx, rx) = stream::channel::<ActionDiagPatch>(cap);
        let store = Self {
            rx,
            entries: VecDeque::with_capacity(crate::supervisor::TERMINAL_FIFO_CAPACITY),
            next_batch_gen: 1,
        };
        (tx, store)
    }

    pub(crate) fn next_batch_gen(&mut self) -> u16 {
        let g = self.next_batch_gen;
        self.next_batch_gen = g.wrapping_add(1).max(1);
        g
    }

    fn entry_mut(&mut self, action_id: u64) -> &mut ActionDiagEntry {
        if let Some(pos) = self.entries.iter().position(|e| e.action_id == action_id) {
            return &mut self.entries[pos];
        }
        if self.entries.len() >= crate::supervisor::TERMINAL_FIFO_CAPACITY {
            self.entries.pop_front();
        }
        self.entries.push_back(ActionDiagEntry::new(action_id));
        self.entries.back_mut().expect("just pushed")
    }

    fn entry(&self, action_id: u64) -> Option<&ActionDiagEntry> {
        self.entries.iter().find(|e| e.action_id == action_id)
    }

    pub(crate) fn intern_str(&mut self, action_id: u64, s: &str) -> DiagStr {
        if let Some(inline) = ShortLabel::<48>::try_new(s) {
            return DiagStr::Inline(inline);
        }
        let e = self.entry_mut(action_id);
        let slot = e.blobs.len() as u32;
        e.blobs.push(DiagBlob::Text(s.to_owned()));
        DiagStr::Interned(slot)
    }

    pub(crate) fn intern_flipped_joints(&mut self, action_id: u64, joints: Vec<String>) -> u32 {
        let e = self.entry_mut(action_id);
        let slot = e.blobs.len() as u32;
        e.blobs.push(DiagBlob::FlippedJoints(joints));
        slot
    }

    pub(crate) fn intern_workspace_violation(
        &mut self,
        action_id: u64,
        detail: WorkspaceViolationDetail,
    ) -> u32 {
        let e = self.entry_mut(action_id);
        let slot = e.blobs.len() as u32;
        e.blobs.push(DiagBlob::WorkspaceViolation(detail));
        slot
    }

    /// Drains all pending patches from the SPSC stream into the bounded off-RT overlay table.
    pub(crate) fn drain(&mut self) -> usize {
        let mut count = 0usize;
        while let Some(sample) = self.rx.try_pop() {
            let patch = sample.payload;
            if patch.action_id != 0 {
                let entry = self.entry_mut(patch.action_id);
                entry.apply_patch(patch.kind);
            }
            count += 1;
        }
        count
    }

    pub(crate) fn stats(&self) -> StreamStats {
        self.rx.stats()
    }

    /// Drains pending patches, freezes the overlay for `action.id` if non-running (single terminal
    /// cutoff), and applies the overlay onto `action`.
    pub(crate) fn attach_to_action(&mut self, action: &mut ActionStatus, force_seal: bool) {
        self.drain();
        if action.id == 0 {
            return;
        }
        if force_seal || !action.is_running() {
            let now = Instant::now();
            let entry = self.entry_mut(action.id);
            entry.seal(now);
            entry.apply_to_action(action);
        } else if let Some(entry) = self.entry(action.id) {
            entry.apply_to_action(action);
        }
    }

    /// Drains pending patches, seals any terminal `obs.action` / `obs.previous_action`, and
    /// overlays accumulated diagnostics in place.
    pub(crate) fn attach_to_observation(&mut self, obs: &mut ArmObservation) {
        self.drain();
        let now = Instant::now();
        if let Some(a) = obs.action.as_mut() {
            if a.id != 0 {
                if !a.is_running() {
                    let entry = self.entry_mut(a.id);
                    entry.seal(now);
                    entry.apply_to_action(a);
                } else if let Some(entry) = self.entry(a.id) {
                    entry.apply_to_action(a);
                }
            }
        }
        if let Some(a) = obs.previous_action.as_mut() {
            if a.id != 0 {
                let entry = self.entry_mut(a.id);
                entry.seal(now);
                entry.apply_to_action(a);
            }
        }
    }
}
