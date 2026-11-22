//! Per-view camera rotation refinement before the reconstruction ([`refine_poses`]).
//!
//! The camera pose of each view comes from the FK of the measured joints. On the real B601-DM
//! (2026-11-22 debug bundles) every non-reference view's rotation was off by 1-2 mrad
//! (kinematics, backlash): a flat sheet on the table then triangulates several mm above it and
//! passes the object threshold. The refinement estimates a small rotation per non-reference view
//! (the reference view fixes the frame; camera centres are kept) from correspondences of points
//! that lie on the table:
//!
//! - Table texture ([`texture_corrs`]): corners of the reference view outside the target box
//!   (grown by [`POSE_BOX_MARGIN_PX`]) and outside the excluded regions (the gripper), matched in
//!   each other view in 2D, not along the predicted epipolar line: a window of
//!   +-[`POSE_WINDOW_PX`] around the kinematic prediction (the reference ray meets the table
//!   plane), coarse NCC on the half-resolution images with a ratio test, full-resolution 2D
//!   refinement (NCC at least [`POSE_NCC_MIN`]) and a left-right check (the match searched back
//!   in the reference lands within [`POSE_LR_TOL_PX`]).
//! - Marker corners ([`marker_corrs`]): ArUco corners matched across views by marker id and
//!   corner index, weighted [`MARKER_WEIGHT`] times a texture point.
//!
//! The solve ([`refine_poses`]) is a Levenberg-Marquardt bundle adjustment over 3 rotation
//! parameters per free view and the 3D points (Schur complement on the points), Huber loss
//! ([`POSE_HUBER_PX`]) on the reprojection errors, a prior of [`POSE_PRIOR_RAD`] per rotation
//! and, for points on the table, coplanarity with the plane z = table_z (1 mm counts as
//! [`POSE_PLANE_PX_PER_MM`] px). Observations off by more than [`POSE_OUTLIER_PX`] after the
//! first solve are dropped and it is solved again. Guard rails per view: fewer than
//! [`POSE_MIN_POINTS`] points or a correction above [`POSE_MAX_CORRECTION_RAD`] keep the
//! kinematic pose (status `too_few_points`, `correction_too_large`).
use super::*;

/// Search window around the kinematic prediction (px, each direction).
pub const POSE_WINDOW_PX: f64 = 15.0;
/// Second matching pass: window around the prediction of the first pass's refined cameras (px).
pub const POSE_GUIDED_WINDOW_PX: f64 = 4.0;
/// Corners of the reference view the texture matching tries.
pub const POSE_MAX_FEATURES: usize = 300;
/// The target box is grown by this before its corners are left out (px).
pub const POSE_BOX_MARGIN_PX: f64 = 8.0;
/// Full-resolution NCC a texture match needs, and the coarse NCC of its best window position.
pub const POSE_NCC_MIN: f64 = 0.85;
pub const POSE_COARSE_MIN: f64 = 0.7;
/// Ratio test of a window search at full resolution: (1 - best) <= ratio * (1 - second).
pub const POSE_RATIO: f64 = 0.7;
pub const POSE_LR_TOL_PX: f64 = 1.5;
/// Points (texture and marker corners) a view needs for its own rotation.
pub const POSE_MIN_POINTS: usize = 12;
/// A larger correction (rad) is reported and not applied.
pub const POSE_MAX_CORRECTION_RAD: f64 = 0.008;
pub const POSE_PRIOR_RAD: f64 = 0.005;
pub const POSE_HUBER_PX: f64 = 1.0;
pub const POSE_OUTLIER_PX: f64 = 3.0;
/// Coplanarity weight of table points: 1 mm off the plane costs as much as this many px.
pub const POSE_PLANE_PX_PER_MM: f64 = 1.0;
/// Weight of a marker corner relative to a texture point.
pub const MARKER_WEIGHT: f64 = 3.0;
const LM_ITERS: usize = 30;

/// One ArUco marker seen in a view: id and its 4 corners (px) in the provider's order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Marker {
    pub id: i64,
    pub corners: [[f64; 2]; 4],
}

/// One point seen in several views: (view index, pixel) per view.
#[derive(Debug, Clone, PartialEq)]
pub struct Corr {
    pub obs: Vec<(usize, [f64; 2])>,
    /// On the table (coplanarity applies).
    pub on_table: bool,
    pub marker: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseParams {
    pub window_px: f64,
    pub guided_window_px: f64,
    pub min_points: usize,
    pub max_correction_rad: f64,
    pub prior_rad: f64,
    pub huber_px: f64,
    pub outlier_px: f64,
    /// 0: no coplanarity term.
    pub plane_px_per_mm: f64,
    pub marker_weight: f64,
}

impl Default for PoseParams {
    fn default() -> Self {
        Self { window_px: POSE_WINDOW_PX, guided_window_px: POSE_GUIDED_WINDOW_PX, min_points: POSE_MIN_POINTS, max_correction_rad: POSE_MAX_CORRECTION_RAD, prior_rad: POSE_PRIOR_RAD,
            huber_px: POSE_HUBER_PX, outlier_px: POSE_OUTLIER_PX, plane_px_per_mm: POSE_PLANE_PX_PER_MM, marker_weight: MARKER_WEIGHT }
    }
}

/// The refinement of one view.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ViewRefine {
    /// `reference`, `refined`, `too_few_points` or `correction_too_large` (the kinematic pose kept).
    pub status: &'static str,
    /// Rotation applied (or rejected) as a base-frame rotation vector (mrad), and its size.
    pub rot_mrad: [f64; 3],
    pub correction_mrad: f64,
    /// Points seen in this view after the outlier rejection, and of those marker corners.
    pub n_points: usize,
    pub n_marker_corners: usize,
    /// Reprojection rms (px) of those points: triangulated with the kinematic cameras, and
    /// after the refinement.
    pub rms_before_px: Option<f64>,
    pub rms_after_px: Option<f64>,
}

#[cfg(feature = "mview-profile")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MatchDir {
    Fwd,
    Rev,
}

/// Per-pass pose matching profile collected under `mview-profile`.
#[cfg(feature = "mview-profile")]
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct PosePassProfile {
    pub total_ms: f64,
    pub integral_ms: f64,
    pub coarse_score_ms: f64,
    pub peak_select_ms: f64,
    pub refine_ms: f64,
    pub residual_ms: f64,
    pub cells_cached: u64,
    pub cells_flat: u64,
    pub cells_dotted: u64,
    pub run_len_hist: std::collections::BTreeMap<String, u64>,
    pub run_count: u64,
    pub max_run_len: u64,
    pub half_grid_fwd: [u64; 4],
    pub half_grid_rev: [u64; 4],
    pub half_grid_fallback_fwd: u64,
    pub half_grid_fallback_rev: u64,
}

#[cfg(feature = "mview-profile")]
impl PosePassProfile {
    fn new_seeded() -> Self {
        let mut p = Self::default();
        for len in 1..=16 {
            p.run_len_hist.insert(len.to_string(), 0);
        }
        p.run_len_hist.insert("17..32".to_string(), 0);
        p.run_len_hist.insert("33..64".to_string(), 0);
        p.run_len_hist.insert("65+".to_string(), 0);
        p
    }

    #[inline]
    fn record_run(&mut self, len: u64) {
        if len == 0 {
            return;
        }
        self.run_count += 1;
        self.max_run_len = self.max_run_len.max(len);
        let key = match len {
            1..=16 => len.to_string(),
            17..=32 => "17..32".to_string(),
            33..=64 => "33..64".to_string(),
            _ => "65+".to_string(),
        };
        *self.run_len_hist.entry(key).or_insert(0) += 1;
    }
}

/// One Levenberg-Marquardt call of a `refine_poses` (`mview-profile`).
#[cfg(feature = "mview-profile")]
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct LmCallProfile {
    /// `round` (free cameras, outlier/cap rounds), `cap_resolve` (after the last cap round) or
    /// `before` (fixed cameras, the kinematic rms diagnostic).
    pub role: &'static str,
    pub round: usize,
    pub n_free: usize,
    pub n_points: usize,
    pub iters: u32,
    pub accepted: u32,
    pub rejected: u32,
    /// `gain` (relative gain below 1e-7), `lambda` (damping above 1e6), `singular`, `iters`.
    pub exit: &'static str,
    pub ms: f64,
}

/// The solves of one `refine_poses` call: per LM call, and the stage times / counters
/// (`lm.*`: init, assembly, schur, dense, backsub, trial_cost, rms; allocations).
#[cfg(feature = "mview-profile")]
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct LmProfile {
    pub calls: Vec<LmCallProfile>,
    pub stages: std::collections::BTreeMap<String, f64>,
}

#[cfg(feature = "mview-profile")]
mod lm_prof {
    use super::{LmCallProfile, LmProfile};
    use std::cell::RefCell;
    thread_local! {
        static CALLS: RefCell<Vec<LmCallProfile>> = const { RefCell::new(Vec::new()) };
        static ROLE: RefCell<(&'static str, usize)> = const { RefCell::new(("round", 0)) };
    }
    pub(super) fn set_role(role: &'static str, round: usize) { ROLE.with(|r| *r.borrow_mut() = (role, round)); }
    pub(super) fn role() -> (&'static str, usize) { ROLE.with(|r| *r.borrow()) }
    pub(super) fn push(c: LmCallProfile) { CALLS.with(|v| v.borrow_mut().push(c)); }
    pub(super) fn take() -> LmProfile {
        LmProfile { calls: CALLS.with(|v| std::mem::take(&mut *v.borrow_mut())), stages: crate::stage_prof::take() }
    }
}

#[inline(always)]
#[cfg_attr(not(feature = "mview-profile"), allow(unused_variables))]
fn lm_role(role: &'static str, round: usize) {
    #[cfg(feature = "mview-profile")]
    lm_prof::set_role(role, round);
}

/// Top-level pose refinement profile collected under `mview-profile`.
#[cfg(feature = "mview-profile")]
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct PoseProfile {
    /// Preparation sub-stages (`prep.features_ms`, `prep.halves_ms`, `prep.templates_ms`, ...).
    pub prep_stages: std::collections::BTreeMap<String, f64>,
    pub pass1_lm: LmProfile,
    pub pass2_lm: LmProfile,
    pub prep_ms: f64,
    pub pass1_match_ms: f64,
    pub pass1_solve_ms: f64,
    pub pass2_match_ms: f64,
    pub pass2_solve_ms: f64,
    pub total_ms: f64,
    pub pass1: PosePassProfile,
    pub pass2: PosePassProfile,
}

#[derive(Debug, Clone, Default)]
pub struct PoseRefinement {
    /// The cameras to reconstruct with (kinematic ones for views not refined).
    pub cams: Vec<Camera>,
    pub views: Vec<ViewRefine>,
    pub n_corrs: usize,
    pub n_marker_corrs: usize,
    pub ms: f64,
    #[cfg(feature = "mview-profile")]
    pub profile: Option<PoseProfile>,
}

fn inside_any(x: f64, y: f64, rects: &[[f64; 4]]) -> bool { rects.iter().any(|r| in_rect(x, y, r)) }

const MAX_CACHE_CELLS: usize = 1_000_000;
const MAX_CACHE_REFINES: usize = 32_768;
const MAX_CACHE_PAIRS: usize = 4_096;

type RefinedEntry = ((i32, i32), Option<([f64; 2], f64)>);

#[derive(Default)]
struct MatchScratch {
    scores: Vec<f32>,
    peaks: Vec<(f32, [f64; 2])>,
    matches: Vec<([f64; 2], f64)>,
    int_s1: Vec<u32>,
    int_s2: Vec<u64>,
    coarse_dots: Vec<f32>,
    coarse_vars: Vec<f64>,
}

struct CoarseRect {
    cx: i32,
    cy: i32,
    r: i32,
    scores: Vec<f32>,
}

struct RevCache {
    q_bits: [u64; 2],
    px_bits: [u64; 2],
    win_bits: u64,
    tmpl_valid: bool,
    outcome: Option<[f64; 2]>,
    coarse: Option<CoarseRect>,
    refined: Vec<RefinedEntry>,
}

#[derive(Default)]
struct PairCache {
    last_key: Option<([u64; 2], u64)>, // (q0_bits, win_bits)
    last_outcome: Option<[f64; 2]>,
    fwd_coarse: Option<CoarseRect>,
    fwd_refined: Vec<RefinedEntry>,
    rev: Option<RevCache>,
}

struct CacheBudget {
    used_cells: usize,
    max_cells: usize,
    used_refines: usize,
    max_refines: usize,
}

impl CacheBudget {
    fn try_reserve_cells(&mut self, k: usize) -> bool {
        let Some(next) = self.used_cells.checked_add(k) else { return false; };
        if next > self.max_cells { return false; }
        self.used_cells = next;
        true
    }

    fn try_reserve_refine(&mut self) -> bool {
        if self.used_refines >= self.max_refines { return false; }
        self.used_refines += 1;
        true
    }
}

struct MatchCache {
    n_views: usize,
    pairs: Vec<PairCache>, // empty when n_feats * n_views > max_pairs (graceful fallback)
    budget: CacheBudget,
}

impl MatchCache {
    fn new(n_feats: usize, n_views: usize) -> Self {
        Self::with_limits(n_feats, n_views, MAX_CACHE_PAIRS, MAX_CACHE_CELLS, MAX_CACHE_REFINES)
    }

    fn with_limits(n_feats: usize, n_views: usize, max_pairs: usize, max_cells: usize, max_refines: usize) -> Self {
        let pairs = n_feats
            .checked_mul(n_views)
            .filter(|&n| n <= max_pairs)
            .map(|n| (0..n).map(|_| PairCache::default()).collect())
            .unwrap_or_default();
        Self {
            n_views,
            pairs,
            budget: CacheBudget { used_cells: 0, max_cells, used_refines: 0, max_refines },
        }
    }
}

trait GetTmpl {
    fn get_tmpl(&mut self) -> Option<&Tmpl>;
}

impl GetTmpl for &Tmpl {
    fn get_tmpl(&mut self) -> Option<&Tmpl> { Some(self) }
}

struct LazyTmpl<F> {
    slot: Option<Option<Tmpl>>,
    build: F,
}

impl<F: FnMut() -> Option<Tmpl>> LazyTmpl<F> {
    fn new(build: F) -> Self { Self { slot: None, build } }
}

impl<F: FnMut() -> Option<Tmpl>> GetTmpl for LazyTmpl<F> {
    fn get_tmpl(&mut self) -> Option<&Tmpl> {
        self.slot.get_or_insert_with(&mut self.build).as_ref()
    }
}

enum CoarseCache<'a> {
    None,
    Read(&'a CoarseRect),
    ReadWrite(&'a mut Option<CoarseRect>),
}

impl<'a> CoarseCache<'a> {
    fn as_ref(&self) -> Option<&CoarseRect> {
        match self {
            Self::None => None,
            Self::Read(c) => Some(c),
            Self::ReadWrite(c) => c.as_ref(),
        }
    }
}

struct PeakCache<'a> {
    coarse: CoarseCache<'a>,
    refined: Option<&'a mut Vec<RefinedEntry>>,
    budget: Option<&'a mut CacheBudget>,
}

impl PeakCache<'_> {
    fn none() -> Self {
        Self { coarse: CoarseCache::None, refined: None, budget: None }
    }
}

#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn flush_coarse_run(
    backend: crate::ncc_backend::NccBackend,
    img: &Gray,
    a: &[f32],
    tn: usize,
    tr_i: i32,
    gy: i32,
    row_off: usize,
    run_start_gx: i32,
    run_start_i: usize,
    run_len: usize,
    coarse_dots: &mut [f32],
    coarse_vars: &[f64],
    scores: &mut [f32],
    #[cfg(feature = "mview-profile")] prof: &mut Option<&mut PosePassProfile>,
    #[cfg(feature = "mview-profile")] cells_dotted: &mut u64,
) {
    if run_len == 0 {
        return;
    }
    backend
        .coarse_dots(
            img,
            (run_start_gx - tr_i) as usize,
            (gy - tr_i) as usize,
            a,
            tn,
            &mut coarse_dots[..run_len],
        )
        .expect("coarse ROI bounds already validated");
    for k in 0..run_len {
        scores[row_off + run_start_i + k] = coarse_dots[k] / coarse_vars[k].sqrt() as f32;
    }
    #[cfg(feature = "mview-profile")]
    {
        if let Some(p) = prof.as_deref_mut() {
            p.record_run(run_len as u64);
        }
        *cells_dotted += run_len as u64;
    }
}

fn copy_cached_overlap(
    cxi: i32,
    cyi: i32,
    r: i32,
    cr: &CoarseRect,
    scratch: &mut MatchScratch,
    #[cfg(feature = "mview-profile")] prof: Option<&mut PosePassProfile>,
) {
    if cr.r < 0 {
        return;
    }
    let ox0 = (cxi as i64 - r as i64).max(cr.cx as i64 - cr.r as i64);
    let ox1 = (cxi as i64 + r as i64).min(cr.cx as i64 + cr.r as i64);
    let oy0 = (cyi as i64 - r as i64).max(cr.cy as i64 - cr.r as i64);
    let oy1 = (cyi as i64 + r as i64).min(cr.cy as i64 + cr.r as i64);
    if ox0 > ox1 || oy0 > oy1 {
        return;
    }
    #[cfg(feature = "mview-profile")]
    let t_copy = Instant::now();
    let n = (2 * r + 1) as usize;
    let cn = (2 * cr.r + 1) as usize;
    let len = (ox1 - ox0 + 1) as usize;
    let di0 = (ox0 - (cxi as i64 - r as i64)) as usize;
    let si0 = (ox0 - (cr.cx as i64 - cr.r as i64)) as usize;
    for gy in oy0..=oy1 {
        let dj = (gy - (cyi as i64 - r as i64)) as usize;
        let sj = (gy - (cr.cy as i64 - cr.r as i64)) as usize;
        scratch.scores[dj * n + di0..dj * n + di0 + len]
            .copy_from_slice(&cr.scores[sj * cn + si0..sj * cn + si0 + len]);
    }
    #[cfg(feature = "mview-profile")]
    if let Some(p) = prof {
        p.cells_cached += (oy1 - oy0 + 1) as u64 * len as u64;
        p.coarse_score_ms += ms(t_copy);
    }
}

#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn fill_coarse_scores<const N: usize>(
    backend: crate::ncc_backend::NccBackend,
    img: &Gray,
    t: &Tmpl,
    cxi: i32,
    cyi: i32,
    r: i32,
    vx0: i32,
    vx1: i32,
    vy0: i32,
    vy1: i32,
    iw: usize,
    cached: Option<&CoarseRect>,
    s1: &[u32],
    s2: &[u64],
    coarse_dots: &mut Vec<f32>,
    coarse_vars: &mut Vec<f64>,
    scores: &mut [f32],
    #[cfg(feature = "mview-profile")] mut prof: Option<&mut PosePassProfile>,
) {
    let tr = t.r;
    let tr_i = tr as i32;
    let tn = if N > 0 { N } else { 2 * tr + 1 };
    let n = (2 * r + 1) as usize;
    let cn = cached.map_or(0, |cr| 2 * cr.r + 1);
    let sw = f64::from(t.sw);
    let var_floor = 1e-3 * sw;
    let max_run = (vx1 - vx0 + 1) as usize;
    if coarse_dots.len() < max_run {
        coarse_dots.resize(max_run, 0.0);
    }
    if coarse_vars.len() < max_run {
        coarse_vars.resize(max_run, 0.0);
    }
    #[cfg(feature = "mview-profile")]
    let (mut cells_cached, mut cells_flat, mut cells_dotted) = (0u64, 0u64, 0u64);

    for gy in vy0..=vy1 {
        let j = (gy - (cyi - r)) as usize;
        let row_off = j * n;
        let oy = (gy - vy0) as usize;
        let top_row = oy * iw;
        let bot_row = (oy + tn) * iw;
        let cj_opt = cached.and_then(|cr| {
            let cj = gy as i64 - cr.cy as i64 + cr.r as i64;
            (0 <= cj && cj < cn as i64).then_some(cj as usize)
        });
        let mut run_start_gx: i32 = 0;
        let mut run_start_i: usize = 0;
        let mut run_len: usize = 0;
        for gx in vx0..=vx1 {
            let i = (gx - (cxi - r)) as usize;
            if let (Some(cr), Some(cj)) = (cached, cj_opt) {
                let ci = gx as i64 - cr.cx as i64 + cr.r as i64;
                if 0 <= ci && ci < cn as i64 {
                    flush_coarse_run(
                        backend,
                        img,
                        &t.a,
                        tn,
                        tr_i,
                        gy,
                        row_off,
                        run_start_gx,
                        run_start_i,
                        run_len,
                        coarse_dots,
                        coarse_vars,
                        scores,
                        #[cfg(feature = "mview-profile")]
                        &mut prof,
                        #[cfg(feature = "mview-profile")]
                        &mut cells_dotted,
                    );
                    run_len = 0;
                    scores[row_off + i] = cr.scores[cj * cn as usize + ci as usize];
                    #[cfg(feature = "mview-profile")]
                    {
                        cells_cached += 1;
                    }
                    continue;
                }
            }
            let ox = (gx - vx0) as usize;
            let sum1 = ((s1[bot_row + ox + tn] - s1[top_row + ox + tn])
                - (s1[bot_row + ox] - s1[top_row + ox])) as f64;
            let sum2 = ((s2[bot_row + ox + tn] - s2[top_row + ox + tn])
                - (s2[bot_row + ox] - s2[top_row + ox])) as f64;
            let var = sum2 - sum1 * sum1 / sw;
            if var > var_floor {
                if run_len == 0 {
                    run_start_gx = gx;
                    run_start_i = i;
                }
                coarse_vars[run_len] = var;
                run_len += 1;
            } else {
                flush_coarse_run(
                    backend,
                    img,
                    &t.a,
                    tn,
                    tr_i,
                    gy,
                    row_off,
                    run_start_gx,
                    run_start_i,
                    run_len,
                    coarse_dots,
                    coarse_vars,
                    scores,
                    #[cfg(feature = "mview-profile")]
                    &mut prof,
                    #[cfg(feature = "mview-profile")]
                    &mut cells_dotted,
                );
                run_len = 0;
                scores[row_off + i] = 0.0;
                #[cfg(feature = "mview-profile")]
                {
                    cells_flat += 1;
                }
            }
        }
        flush_coarse_run(
            backend,
            img,
            &t.a,
            tn,
            tr_i,
            gy,
            row_off,
            run_start_gx,
            run_start_i,
            run_len,
            coarse_dots,
            coarse_vars,
            scores,
            #[cfg(feature = "mview-profile")]
            &mut prof,
            #[cfg(feature = "mview-profile")]
            &mut cells_dotted,
        );
    }

    #[cfg(feature = "mview-profile")]
    if let Some(p) = prof {
        p.cells_cached += cells_cached;
        p.cells_flat += cells_flat;
        p.cells_dotted += cells_dotted;
    }
}

#[cfg(test)]
fn fill_coarse_window_integral(
    img: &Gray,
    get_t: impl GetTmpl,
    cxi: i32,
    cyi: i32,
    r: i32,
    cached: Option<&CoarseRect>,
    scratch: &mut MatchScratch,
    #[cfg(feature = "mview-profile")] prof: Option<&mut PosePassProfile>,
) {
    fill_coarse_window_integral_with_backend(
        crate::ncc_backend::NccBackend::active(),
        img,
        get_t,
        cxi,
        cyi,
        r,
        cached,
        scratch,
        #[cfg(feature = "mview-profile")]
        prof,
    );
}

fn fill_coarse_window_integral_with_backend(
    backend: crate::ncc_backend::NccBackend,
    img: &Gray,
    mut get_t: impl GetTmpl,
    cxi: i32,
    cyi: i32,
    r: i32,
    cached: Option<&CoarseRect>,
    scratch: &mut MatchScratch,
    #[cfg(feature = "mview-profile")] mut prof: Option<&mut PosePassProfile>,
) {
    if let Some(cr) = cached {
        if cr.r >= 0
            && (cr.cx as i64 - cr.r as i64) <= (cxi as i64 - r as i64)
            && (cxi as i64 + r as i64) <= (cr.cx as i64 + cr.r as i64)
            && (cr.cy as i64 - cr.r as i64) <= (cyi as i64 - r as i64)
            && (cyi as i64 + r as i64) <= (cr.cy as i64 + cr.r as i64)
        {
            copy_cached_overlap(
                cxi,
                cyi,
                r,
                cr,
                scratch,
                #[cfg(feature = "mview-profile")]
                prof,
            );
            return;
        }
    }

    let Some(t) = get_t.get_tmpl() else {
        if let Some(cr) = cached {
            copy_cached_overlap(
                cxi,
                cyi,
                r,
                cr,
                scratch,
                #[cfg(feature = "mview-profile")]
                prof,
            );
        }
        return;
    };

    if t.w.is_some() {
        #[cfg(feature = "mview-profile")]
        let t_w = Instant::now();
        let n = (2 * r + 1) as usize;
        let cn = cached.map_or(0, |cr| 2 * cr.r + 1);
        for j in 0..n {
            let gy = cyi + j as i32 - r;
            let row_off = j * n;
            let cj_opt = cached.and_then(|cr| {
                let cj = gy as i64 - cr.cy as i64 + cr.r as i64;
                (0 <= cj && cj < cn as i64).then_some(cj as usize)
            });
            for i in 0..n {
                let gx = cxi + i as i32 - r;
                if let (Some(cr), Some(cj)) = (cached, cj_opt) {
                    let ci = gx as i64 - cr.cx as i64 + cr.r as i64;
                    if 0 <= ci && ci < cn as i64 {
                        scratch.scores[row_off + i] = cr.scores[cj * cn as usize + ci as usize];
                        #[cfg(feature = "mview-profile")]
                        if let Some(p) = prof.as_deref_mut() {
                            p.cells_cached += 1;
                        }
                        continue;
                    }
                }
                if let Some(v) = ncc(img, t, f64::from(gx), f64::from(gy), false) {
                    scratch.scores[row_off + i] = v;
                }
            }
        }
        #[cfg(feature = "mview-profile")]
        if let Some(p) = prof {
            p.coarse_score_ms += ms(t_w);
        }
        return;
    }

    let tr = t.r;
    let (Ok(w_i32), Ok(h_i32)) = (i32::try_from(img.w), i32::try_from(img.h)) else {
        if let Some(cr) = cached {
            copy_cached_overlap(
                cxi,
                cyi,
                r,
                cr,
                scratch,
                #[cfg(feature = "mview-profile")]
                prof,
            );
        }
        return;
    };
    if 2 * tr + 3 > img.w || 2 * tr + 3 > img.h {
        if let Some(cr) = cached {
            copy_cached_overlap(
                cxi,
                cyi,
                r,
                cr,
                scratch,
                #[cfg(feature = "mview-profile")]
                prof,
            );
        }
        return;
    }
    let tr_i = tr as i32;
    let vx0 = (cxi - r).max(tr_i + 1);
    let vx1 = (cxi + r).min(w_i32 - 2 - tr_i);
    let vy0 = (cyi - r).max(tr_i + 1);
    let vy1 = (cyi + r).min(h_i32 - 2 - tr_i);
    if vx0 > vx1 || vy0 > vy1 {
        if let Some(cr) = cached {
            copy_cached_overlap(
                cxi,
                cyi,
                r,
                cr,
                scratch,
                #[cfg(feature = "mview-profile")]
                prof,
            );
        }
        return;
    }

    if let Some(cr) = cached {
        if cr.r >= 0
            && (cr.cx as i64 - cr.r as i64) <= vx0 as i64
            && vx1 as i64 <= (cr.cx as i64 + cr.r as i64)
            && (cr.cy as i64 - cr.r as i64) <= vy0 as i64
            && vy1 as i64 <= (cr.cy as i64 + cr.r as i64)
        {
            copy_cached_overlap(
                cxi,
                cyi,
                r,
                cr,
                scratch,
                #[cfg(feature = "mview-profile")]
                prof,
            );
            return;
        }
    }

    let ix0 = (vx0 - tr_i) as usize;
    let iy0 = (vy0 - tr_i) as usize;
    let ix1 = (vx1 + tr_i) as usize;
    let iy1 = (vy1 + tr_i) as usize;
    let rw = ix1 - ix0 + 1;
    let rh = iy1 - iy0 + 1;
    let Ok(roi_view) = crate::ncc_backend::PatchView::new(img, ix0, iy0, rw, rh) else {
        if let Some(cr) = cached {
            copy_cached_overlap(
                cxi,
                cyi,
                r,
                cr,
                scratch,
                #[cfg(feature = "mview-profile")]
                prof,
            );
        }
        return;
    };

    #[cfg(feature = "mview-profile")]
    let t_int = Instant::now();
    let iw = ix1 - ix0 + 2;
    let ih = iy1 - iy0 + 2;
    let total = iw * ih;
    scratch.int_s1.resize(total, 0);
    scratch.int_s2.resize(total, 0);
    scratch.int_s1[..iw].fill(0);
    scratch.int_s2[..iw].fill(0);
    for y in iy0..=iy1 {
        let row_idx = y - iy0 + 1;
        let cur = row_idx * iw;
        let prev = cur - iw;
        scratch.int_s1[cur] = 0;
        scratch.int_s2[cur] = 0;
        let px_row = roi_view.row(y - iy0);
        let mut r1 = 0u32;
        let mut r2 = 0u64;
        for (col, &p) in px_row.iter().enumerate() {
            let v = u32::from(p);
            r1 += v;
            r2 += u64::from(v) * u64::from(v);
            scratch.int_s1[cur + col + 1] = scratch.int_s1[prev + col + 1] + r1;
            scratch.int_s2[cur + col + 1] = scratch.int_s2[prev + col + 1] + r2;
        }
    }
    #[cfg(feature = "mview-profile")]
    if let Some(p) = prof.as_deref_mut() {
        p.integral_ms += ms(t_int);
    }

    #[cfg(feature = "mview-profile")]
    let t_score = Instant::now();
    const COARSE_TN: usize = 2 * COARSE_R + 1;
    if tr == COARSE_R {
        fill_coarse_scores::<COARSE_TN>(
            backend,
            img,
            t,
            cxi,
            cyi,
            r,
            vx0,
            vx1,
            vy0,
            vy1,
            iw,
            cached,
            &scratch.int_s1,
            &scratch.int_s2,
            &mut scratch.coarse_dots,
            &mut scratch.coarse_vars,
            &mut scratch.scores,
            #[cfg(feature = "mview-profile")]
            prof.as_deref_mut(),
        );
    } else {
        fill_coarse_scores::<0>(
            backend,
            img,
            t,
            cxi,
            cyi,
            r,
            vx0,
            vx1,
            vy0,
            vy1,
            iw,
            cached,
            &scratch.int_s1,
            &scratch.int_s2,
            &mut scratch.coarse_dots,
            &mut scratch.coarse_vars,
            &mut scratch.scores,
            #[cfg(feature = "mview-profile")]
            prof.as_deref_mut(),
        );
    }
    #[cfg(feature = "mview-profile")]
    if let Some(p) = prof {
        p.coarse_score_ms += ms(t_score);
    }
}

/// Coarse candidates of `t` in `img` (half resolution): the local maxima of the NCC on the
/// integer grid within `r` px of `c` with at least [`POSE_COARSE_MIN`], best first (at most 4).
#[cfg(test)]
fn window_peaks_cached(
    img: &Gray,
    get_t: impl GetTmpl,
    c: [f64; 2],
    r: i32,
    cached: Option<&CoarseRect>,
    scratch: &mut MatchScratch,
) -> (([[f64; 2]; 4], usize), Option<(i32, i32)>) {
    window_peaks_cached_with_backend(
        crate::ncc_backend::NccBackend::active(),
        img,
        get_t,
        c,
        r,
        cached,
        scratch,
    )
}

#[cfg(test)]
fn window_peaks_cached_with_backend(
    backend: crate::ncc_backend::NccBackend,
    img: &Gray,
    get_t: impl GetTmpl,
    c: [f64; 2],
    r: i32,
    cached: Option<&CoarseRect>,
    scratch: &mut MatchScratch,
) -> (([[f64; 2]; 4], usize), Option<(i32, i32)>) {
    window_peaks_cached_profiled_with_backend(
        backend,
        img,
        get_t,
        c,
        r,
        cached,
        scratch,
        #[cfg(feature = "mview-profile")]
        None,
    )
}

fn window_peaks_cached_profiled(
    img: &Gray,
    get_t: impl GetTmpl,
    c: [f64; 2],
    r: i32,
    cached: Option<&CoarseRect>,
    scratch: &mut MatchScratch,
    #[cfg(feature = "mview-profile")] prof: Option<&mut PosePassProfile>,
) -> (([[f64; 2]; 4], usize), Option<(i32, i32)>) {
    window_peaks_cached_profiled_with_backend(
        crate::ncc_backend::NccBackend::active(),
        img,
        get_t,
        c,
        r,
        cached,
        scratch,
        #[cfg(feature = "mview-profile")]
        prof,
    )
}

fn window_peaks_cached_profiled_with_backend(
    backend: crate::ncc_backend::NccBackend,
    img: &Gray,
    mut get_t: impl GetTmpl,
    c: [f64; 2],
    r: i32,
    cached: Option<&CoarseRect>,
    scratch: &mut MatchScratch,
    #[cfg(feature = "mview-profile")] mut prof: Option<&mut PosePassProfile>,
) -> (([[f64; 2]; 4], usize), Option<(i32, i32)>) {
    if r < 0 {
        scratch.scores.clear();
        return (([[0.0; 2]; 4], 0), None);
    }
    let n = (2 * r + 1) as usize;
    let (cx, cy) = (c[0].round(), c[1].round());
    scratch.scores.resize(n * n, f32::NEG_INFINITY);
    scratch.scores.fill(f32::NEG_INFINITY);

    let center = (cx.abs() < 1e7 && cy.abs() < 1e7 && r < 1_000_000).then_some((cx as i32, cy as i32));
    if let Some((cxi, cyi)) = center {
        fill_coarse_window_integral_with_backend(
            backend,
            img,
            get_t,
            cxi,
            cyi,
            r,
            cached,
            scratch,
            #[cfg(feature = "mview-profile")]
            prof.as_deref_mut(),
        );
    } else {
        #[cfg(feature = "mview-profile")]
        let t_fb = Instant::now();
        for j in 0..n {
            let y = cy + j as f64 - r as f64;
            let row_off = j * n;
            for i in 0..n {
                if let Some(t) = get_t.get_tmpl() {
                    if let Some(v) = ncc(img, t, cx + i as f64 - r as f64, y, false) {
                        scratch.scores[row_off + i] = v;
                    }
                }
            }
        }
        #[cfg(feature = "mview-profile")]
        if let Some(p) = prof.as_deref_mut() {
            p.coarse_score_ms += ms(t_fb);
        }
    }

    #[cfg(feature = "mview-profile")]
    let t_peak = Instant::now();
    scratch.peaks.clear();
    let ni = n as i32;
    for j in 0..ni {
        for i in 0..ni {
            let v = scratch.scores[j as usize * n + i as usize];
            if v.is_nan() || f64::from(v) < POSE_COARSE_MIN { continue; }
            let local = (-1..=1).all(|dj| (-1..=1).all(|di| {
                let (a, b) = (i + di, j + dj);
                (di == 0 && dj == 0) || a < 0 || b < 0 || a >= ni || b >= ni || scratch.scores[b as usize * n + a as usize] <= v
            }));
            if local {
                scratch.peaks.push((v, [cx + f64::from(i - r), cy + f64::from(j - r)]));
            }
        }
    }
    scratch.peaks.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut top = [[0.0; 2]; 4];
    let len = scratch.peaks.len().min(4);
    for (dst, src) in top[..len].iter_mut().zip(&scratch.peaks[..len]) {
        *dst = src.1;
    }
    #[cfg(feature = "mview-profile")]
    if let Some(p) = prof {
        p.peak_select_ms += ms(t_peak);
    }
    ((top, len), center)
}

#[cfg(test)]
fn window_peaks(img: &Gray, t: &Tmpl, c: [f64; 2], r: i32) -> Vec<[f64; 2]> {
    let mut scratch = MatchScratch::default();
    let ((top, len), _) = window_peaks_cached(img, t, c, r, None, &mut scratch);
    top[..len].to_vec()
}

fn pick_match(matches: &mut [([f64; 2], f64)]) -> Option<[f64; 2]> {
    matches.sort_by(|a, b| b.1.total_cmp(&a.1));
    let (q, best) = *matches.first()?;
    if best < POSE_NCC_MIN { return None; }
    let second = matches.iter().filter(|(p, _)| (p[0] - q[0]).hypot(p[1] - q[1]) > 2.0).map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    if second.is_finite() && (1.0 - best) > POSE_RATIO * (1.0 - second) { return None; }
    Some(q)
}

/// 2D match of the full-resolution patch `tf` (half-resolution `tc`) of one image in `full` /
/// `half` of another, in a window of `win` px around `q0` (full resolution): the coarse peaks
/// refined at full resolution; the best needs [`POSE_NCC_MIN`] and must pass the ratio test
/// against the best other match more than 2 px away.
#[cfg(test)]
fn match_2d_cached(
    imgs: (&Gray, &Gray),
    get_tc: impl GetTmpl,
    get_tf: impl GetTmpl,
    q0: [f64; 2],
    win: f64,
    pcache: PeakCache<'_>,
    scratch: &mut MatchScratch,
) -> Option<[f64; 2]> {
    match_2d_cached_profiled(
        imgs,
        get_tc,
        get_tf,
        q0,
        win,
        pcache,
        scratch,
        #[cfg(feature = "mview-profile")]
        MatchDir::Fwd,
        #[cfg(feature = "mview-profile")]
        None,
    )
}

fn match_2d_cached_profiled(
    imgs: (&Gray, &Gray),
    get_tc: impl GetTmpl,
    mut get_tf: impl GetTmpl,
    q0: [f64; 2],
    win: f64,
    mut pcache: PeakCache<'_>,
    scratch: &mut MatchScratch,
    #[cfg(feature = "mview-profile")] dir: MatchDir,
    #[cfg(feature = "mview-profile")] mut prof: Option<&mut PosePassProfile>,
) -> Option<[f64; 2]> {
    let (full, half) = imgs;
    let r = (win / 2.0).ceil() as i32;
    let ((top, top_len), center) = window_peaks_cached_profiled(
        half,
        get_tc,
        to_half(q0),
        r,
        pcache.coarse.as_ref(),
        scratch,
        #[cfg(feature = "mview-profile")]
        prof.as_deref_mut(),
    );
    if let (CoarseCache::ReadWrite(slot), Some(budget), Some((cx, cy))) = (pcache.coarse, pcache.budget.as_deref_mut(), center) {
        if slot.is_none() && budget.try_reserve_cells(scratch.scores.len()) {
            *slot = Some(CoarseRect { cx, cy, r, scores: scratch.scores.clone() });
        }
    }
    #[cfg(feature = "mview-profile")]
    let t_ref = Instant::now();
    scratch.matches.clear();
    for h in &top[..top_len] {
        let key = (h[0].abs() < 1e7 && h[1].abs() < 1e7).then(|| (h[0] as i32, h[1] as i32));
        let cached_res = key.and_then(|k| {
            pcache.refined.as_deref().and_then(|rv| rv.iter().find(|&&(rk, _)| rk == k).map(|&(_, v)| v))
        });
        let res = match cached_res {
            Some(v) => v,
            None => {
                #[cfg(feature = "mview-profile")]
                let hg0 = crate::half_grid_profile_counts();
                let computed = get_tf.get_tmpl().and_then(|tf| refine_2d_pose(full, tf, [2.0 * h[0] + 0.5, 2.0 * h[1] + 0.5]));
                #[cfg(feature = "mview-profile")]
                if let Some(p) = prof.as_deref_mut() {
                    let hg1 = crate::half_grid_profile_counts();
                    let dst = match dir {
                        MatchDir::Fwd => &mut p.half_grid_fwd,
                        MatchDir::Rev => &mut p.half_grid_rev,
                    };
                    for idx in 0..4 {
                        dst[idx] += hg1.0[idx] - hg0.0[idx];
                    }
                    let fb = hg1.1 - hg0.1;
                    match dir {
                        MatchDir::Fwd => p.half_grid_fallback_fwd += fb,
                        MatchDir::Rev => p.half_grid_fallback_rev += fb,
                    }
                }
                if let (Some(k), Some(rv), Some(budget)) = (key, pcache.refined.as_deref_mut(), pcache.budget.as_deref_mut()) {
                    if budget.try_reserve_refine() {
                        rv.push((k, computed));
                    }
                }
                computed
            }
        };
        if let Some((q, s)) = res {
            if (q[0] - q0[0]).abs() <= win + 2.0 && (q[1] - q0[1]).abs() <= win + 2.0 {
                scratch.matches.push((q, s));
            }
        }
    }
    let picked = pick_match(&mut scratch.matches);
    #[cfg(feature = "mview-profile")]
    if let Some(p) = prof {
        p.refine_ms += ms(t_ref);
    }
    picked
}

struct PreparedFeat {
    px: [f64; 2],
    x0: Vec3,
    tc: Tmpl,
    tf: Tmpl,
}

struct TexturePrep<'h> {
    reference: usize,
    /// Half-resolution image per view: built here, or shared with the reconstruction.
    halves: std::borrow::Cow<'h, [Gray]>,
    feats: Vec<PreparedFeat>,
}

impl<'h> TexturePrep<'h> {
    fn new(views: &[View], ref_cam: &Camera, reference: usize, object_box: [f64; 4], exclude: &[[f64; 4]], table_z: f64) -> Self {
        Self::prepared(views, None, ref_cam, reference, object_box, exclude, table_z)
    }

    /// [`TexturePrep::new`] with the images prepared for `views` (half images, rebuilt when of
    /// another length); leaves the reference corner map there for the reconstruction.
    fn prepared(views: &[View], prep: Option<&'h Prepared>, ref_cam: &Camera, reference: usize, object_box: [f64; 4], exclude: &[[f64; 4]], table_z: f64) -> Self {
        let halves = prep.map(|pr| pr.halves.as_slice());
        if reference >= views.len() || views.len() < 2 {
            return Self { reference, halves: std::borrow::Cow::Owned(Vec::new()), feats: Vec::new() };
        }
        let rv = &views[reference];
        let m = POSE_BOX_MARGIN_PX;
        let grown = [object_box[0] - m, object_box[1] - m, object_box[2] + m, object_box[3] + m];
        let mut skip = vec![grown];
        skip.extend_from_slice(exclude);
        let mut p = Params::new(table_z);
        (p.max_object_features, p.max_table_features, p.max_edge_features, p.min_feature_dist_px) = (0, POSE_MAX_FEATURES, 0, 8.0);
        // strongest first over the whole image: on the real bundles (2026-11-21/22) the strongest
        // corners (sheet codes, cable edges) matched more often than a grid of weaker ones
        (p.corner_rel_floor, p.corner_cell_px) = (0.01, 0);
        let full = [0.0, 0.0, rv.img.w as f64 - 1.0, rv.img.h as f64 - 1.0];
        // the box quota is 0: corners in it are left out; corners in an excluded region are dropped
        // afterwards, so the quota is taken twice
        p.max_table_features *= 2;
        let t = ProfT::start();
        let mut map = None;
        let raw_feats: Vec<Feature> = features_shared(&rv.img, full, grown, [0.0; 4], &p, None, prep.is_some().then_some(&mut map)).into_iter()
            .filter(|f| !inside_any(f.x as f64, f.y as f64, &skip)).take(POSE_MAX_FEATURES).collect();
        if let (Some(pr), Some(m)) = (prep, map) { pr.put_ref_map(reference, m); }
        t.stop("prep.features_ms");
        let t = ProfT::start();
        let halves: std::borrow::Cow<'h, [Gray]> = match halves {
            Some(h) if h.len() == views.len() => std::borrow::Cow::Borrowed(h),
            _ => std::borrow::Cow::Owned(half_images(views)),
        };
        t.stop("prep.halves_ms");
        let t = ProfT::start();
        let mut feats = Vec::new();
        for f in raw_feats {
            let px = [f.x as f64, f.y as f64];
            let d = ref_cam.ray(px);
            if d[2] > -1e-3 { continue; }
            let t = (table_z - ref_cam.c[2]) / d[2];
            if t.is_nan() || t <= 0.05 { continue; }
            let x0 = [ref_cam.c[0] + t * d[0], ref_cam.c[1] + t * d[1], ref_cam.c[2] + t * d[2]];
            let hq = to_half(px);
            let (Some(tc), Some(tf)) = (
                sample_bilinear(&halves[reference], hq[0], hq[1], COARSE_R).and_then(|v| make_tmpl(v, COARSE_R, None, 0.0)),
                make_tmpl(sample_int(&rv.img, f.x, f.y, PATCH_R), PATCH_R, None, 0.0),
            ) else { continue; };
            feats.push(PreparedFeat { px, x0, tc, tf });
        }
        t.stop("prep.templates_ms");
        prof_count("prep.n_features", feats.len() as u64);
        Self { reference, halves, feats }
    }

    fn corrs(&self, views: &[View], cams: &[Camera], win: &[f64]) -> Vec<Corr> {
        self.corrs_with_cache(views, cams, win, None)
    }

    fn corrs_with_cache(&self, views: &[View], cams: &[Camera], win: &[f64], cache: Option<&mut MatchCache>) -> Vec<Corr> {
        self.corrs_with_cache_profiled(
            views,
            cams,
            win,
            cache,
            #[cfg(feature = "mview-profile")]
            None,
        )
    }

    fn corrs_with_cache_profiled(
        &self,
        views: &[View],
        cams: &[Camera],
        win: &[f64],
        mut cache: Option<&mut MatchCache>,
        #[cfg(feature = "mview-profile")] mut prof: Option<&mut PosePassProfile>,
    ) -> Vec<Corr> {
        #[cfg(feature = "mview-profile")]
        let t_pass = Instant::now();
        if self.reference >= views.len() || views.len() < 2 || cams.len() != views.len() || win.len() != views.len() || self.halves.len() != views.len() {
            #[cfg(feature = "mview-profile")]
            if let Some(p) = prof {
                p.total_ms = ms(t_pass);
                p.residual_ms = (p.total_ms - (p.integral_ms + p.coarse_score_ms + p.peak_select_ms + p.refine_ms)).max(0.0);
            }
            return Vec::new();
        }
        let rv = &views[self.reference];
        let mut out = Vec::new();
        let mut scratch = MatchScratch::default();
        let use_cache = cache.as_ref().is_some_and(|c| {
            !c.pairs.is_empty() && c.n_views == views.len() && c.pairs.len() == self.feats.len() * c.n_views
        });
        for (feat_idx, f) in self.feats.iter().enumerate() {
            let px = f.px;
            let mut obs = vec![(self.reference, px)];
            for (j, v) in views.iter().enumerate() {
                if j == self.reference { continue; }
                let Some(q0) = cams[j].project(&f.x0) else { continue; };

                if !use_cache {
                    let Some(q) = match_2d_cached_profiled(
                        (&v.img, &self.halves[j]),
                        &f.tc,
                        &f.tf,
                        q0,
                        win[j],
                        PeakCache::none(),
                        &mut scratch,
                        #[cfg(feature = "mview-profile")]
                        MatchDir::Fwd,
                        #[cfg(feature = "mview-profile")]
                        prof.as_deref_mut(),
                    ) else { continue; };
                    // left-right: this view's patch searched back in the reference around the feature
                    let hq = to_half(q);
                    let back = sample_bilinear(&self.halves[j], hq[0], hq[1], COARSE_R).and_then(|tv| make_tmpl(tv, COARSE_R, None, 0.0))
                        .zip(sample_bilinear(&v.img, q[0], q[1], PATCH_R).and_then(|tv| make_tmpl(tv, PATCH_R, None, 0.0)))
                        .and_then(|(bc, bf)| {
                            match_2d_cached_profiled(
                                (&rv.img, &self.halves[self.reference]),
                                &bc,
                                &bf,
                                px,
                                win[j],
                                PeakCache::none(),
                                &mut scratch,
                                #[cfg(feature = "mview-profile")]
                                MatchDir::Rev,
                                #[cfg(feature = "mview-profile")]
                                prof.as_deref_mut(),
                            )
                        });
                    if back.is_some_and(|b| (b[0] - px[0]).hypot(b[1] - px[1]) <= POSE_LR_TOL_PX) { obs.push((j, q)); }
                    continue;
                }

                let cache_ref = cache.as_deref_mut().unwrap();
                let pair = &mut cache_ref.pairs[feat_idx * cache_ref.n_views + j];
                let budget = &mut cache_ref.budget;
                let key = ([q0[0].to_bits(), q0[1].to_bits()], win[j].to_bits());
                if pair.last_key == Some(key) {
                    if let Some(q) = pair.last_outcome { obs.push((j, q)); }
                    continue;
                }

                let fwd_q = match_2d_cached_profiled(
                    (&v.img, &self.halves[j]),
                    &f.tc,
                    &f.tf,
                    q0,
                    win[j],
                    PeakCache { coarse: CoarseCache::ReadWrite(&mut pair.fwd_coarse), refined: Some(&mut pair.fwd_refined), budget: Some(budget) },
                    &mut scratch,
                    #[cfg(feature = "mview-profile")]
                    MatchDir::Fwd,
                    #[cfg(feature = "mview-profile")]
                    prof.as_deref_mut(),
                );

                let accepted = fwd_q.and_then(|q| {
                    let q_bits = [q[0].to_bits(), q[1].to_bits()];
                    let px_bits = [px[0].to_bits(), px[1].to_bits()];
                    let win_bits = win[j].to_bits();
                    let back = if let Some(rev) = pair.rev.as_mut().filter(|r| r.q_bits == q_bits && r.px_bits == px_bits) {
                        if !rev.tmpl_valid {
                            None
                        } else if rev.win_bits == win_bits {
                            rev.outcome
                        } else {
                            let hq = to_half(q);
                            let lazy_bc = LazyTmpl::new(|| sample_bilinear(&self.halves[j], hq[0], hq[1], COARSE_R).and_then(|tv| make_tmpl(tv, COARSE_R, None, 0.0)));
                            let lazy_bf = LazyTmpl::new(|| sample_bilinear(&v.img, q[0], q[1], PATCH_R).and_then(|tv| make_tmpl(tv, PATCH_R, None, 0.0)));
                            let coarse = rev.coarse.as_ref().map_or(CoarseCache::None, CoarseCache::Read);
                            match_2d_cached_profiled(
                                (&rv.img, &self.halves[self.reference]),
                                lazy_bc,
                                lazy_bf,
                                px,
                                win[j],
                                PeakCache { coarse, refined: Some(&mut rev.refined), budget: Some(budget) },
                                &mut scratch,
                                #[cfg(feature = "mview-profile")]
                                MatchDir::Rev,
                                #[cfg(feature = "mview-profile")]
                                prof.as_deref_mut(),
                            )
                        }
                    } else {
                        let hq = to_half(q);
                        let tmpls = sample_bilinear(&self.halves[j], hq[0], hq[1], COARSE_R).and_then(|tv| make_tmpl(tv, COARSE_R, None, 0.0))
                            .zip(sample_bilinear(&v.img, q[0], q[1], PATCH_R).and_then(|tv| make_tmpl(tv, PATCH_R, None, 0.0)));
                        match tmpls {
                            None => {
                                if pair.rev.is_none() {
                                    pair.rev = Some(RevCache { q_bits, px_bits, win_bits, tmpl_valid: false, outcome: None, coarse: None, refined: Vec::new() });
                                }
                                None
                            }
                            Some((bc, bf)) => {
                                if pair.rev.is_none() {
                                    let mut rev_coarse = None;
                                    let mut rev_refined = Vec::new();
                                    let b = match_2d_cached_profiled(
                                        (&rv.img, &self.halves[self.reference]),
                                        &bc,
                                        &bf,
                                        px,
                                        win[j],
                                        PeakCache { coarse: CoarseCache::ReadWrite(&mut rev_coarse), refined: Some(&mut rev_refined), budget: Some(budget) },
                                        &mut scratch,
                                        #[cfg(feature = "mview-profile")]
                                        MatchDir::Rev,
                                        #[cfg(feature = "mview-profile")]
                                        prof.as_deref_mut(),
                                    );
                                    pair.rev = Some(RevCache { q_bits, px_bits, win_bits, tmpl_valid: true, outcome: b, coarse: rev_coarse, refined: rev_refined });
                                    b
                                } else {
                                    match_2d_cached_profiled(
                                        (&rv.img, &self.halves[self.reference]),
                                        &bc,
                                        &bf,
                                        px,
                                        win[j],
                                        PeakCache::none(),
                                        &mut scratch,
                                        #[cfg(feature = "mview-profile")]
                                        MatchDir::Rev,
                                        #[cfg(feature = "mview-profile")]
                                        prof.as_deref_mut(),
                                    )
                                }
                            }
                        }
                    };
                    back.filter(|b| (b[0] - px[0]).hypot(b[1] - px[1]) <= POSE_LR_TOL_PX).map(|_| q)
                });

                if pair.last_key.is_none() {
                    pair.last_key = Some(key);
                    pair.last_outcome = accepted;
                }
                if let Some(q) = accepted { obs.push((j, q)); }
            }
            if obs.len() >= 2 { out.push(Corr { obs, on_table: true, marker: false }); }
        }
        #[cfg(feature = "mview-profile")]
        if let Some(p) = prof {
            p.total_ms = ms(t_pass);
            p.residual_ms = (p.total_ms - (p.integral_ms + p.coarse_score_ms + p.peak_select_ms + p.refine_ms)).max(0.0);
        }
        out
    }
}

/// Table-texture correspondences (see the module doc): corners of `views[reference]` outside
/// `object_box` (grown by [`POSE_BOX_MARGIN_PX`]) and `exclude`, matched in 2D in every other
/// view within `win[view]` px of the point where the reference ray meets the plane z =
/// `table_z`, projected with `cams`.
pub fn texture_corrs(views: &[View], cams: &[Camera], reference: usize, object_box: [f64; 4], exclude: &[[f64; 4]], table_z: f64, win: &[f64]) -> Vec<Corr> {
    if reference >= views.len() || views.len() < 2 || cams.len() != views.len() || win.len() != views.len() { return Vec::new(); }
    TexturePrep::new(views, &cams[reference], reference, object_box, exclude, table_z).corrs(views, cams, win)
}

/// Marker-corner correspondences: each (id, corner index) seen in at least two views (an id
/// seen twice in one view is left out there). On the table unless the corner lies in
/// `object_box` of the reference view.
pub fn marker_corrs(markers: &[Vec<Marker>], reference: usize, object_box: [f64; 4]) -> Vec<Corr> {
    let mut by: std::collections::BTreeMap<(i64, usize), Vec<(usize, [f64; 2])>> = Default::default();
    for (v, ms) in markers.iter().enumerate() {
        for m in ms {
            if ms.iter().filter(|o| o.id == m.id).count() > 1 { continue; }
            for (k, c) in m.corners.iter().enumerate() {
                if c[0].is_finite() && c[1].is_finite() { by.entry((m.id, k)).or_default().push((v, *c)); }
            }
        }
    }
    by.into_values().filter(|o| o.len() >= 2).map(|obs| {
        let on_table = obs.iter().find(|o| o.0 == reference).is_none_or(|o| !in_rect(o.1[0], o.1[1], &object_box));
        Corr { obs, on_table, marker: true }
    }).collect()
}

// ---- small linear algebra ----

fn rodrigues(v: &Vec3) -> Mat3 {
    let th = kinematics::norm(v);
    let k = [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]];
    let (a, b) = if th < 1e-12 { (1.0, 0.5) } else { (th.sin() / th, (1.0 - th.cos()) / (th * th)) };
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let k2: f64 = (0..3).map(|l| k[i][l] * k[l][j]).sum();
            r[i][j] = if i == j { 1.0 } else { 0.0 } + a * k[i][j] + b * k2;
        }
    }
    r
}

fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 { for j in 0..3 { r[i][j] = (0..3).map(|l| a[i][l] * b[l][j]).sum(); } }
    r
}

fn inv3(m: &Mat3) -> Option<Mat3> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-18 { return None; }
    let c = |i: usize, j: usize| {
        let (r0, r1) = ((i + 1) % 3, (i + 2) % 3);
        let (c0, c1) = ((j + 1) % 3, (j + 2) % 3);
        m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0]
    };
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 { for j in 0..3 { r[j][i] = c(i, j) / det; } }
    Some(r)
}

/// Solve the dense n x n system `a x = b` (Gaussian elimination, partial pivoting) in place: `a`
/// and `b` are overwritten and the solution goes to `x` (len n); false when singular or not finite.
fn solve_dense_in(a: &mut [f64], b: &mut [f64], n: usize, x: &mut [f64]) -> bool {
    for col in 0..n {
        let Some(piv) = (col..n).max_by(|&i, &j| a[i * n + col].abs().total_cmp(&a[j * n + col].abs())) else { return false; };
        if a[piv * n + col].abs() < 1e-15 { return false; }
        if piv != col {
            for k in 0..n { a.swap(piv * n + k, col * n + k); }
            b.swap(piv, col);
        }
        for r in col + 1..n {
            let f = a[r * n + col] / a[col * n + col];
            if f == 0.0 { continue; }
            for k in col..n { a[r * n + k] -= f * a[col * n + k]; }
            b[r] -= f * b[col];
        }
    }
    for r in (0..n).rev() {
        let s: f64 = (r + 1..n).map(|k| a[r * n + k] * x[k]).sum();
        x[r] = (b[r] - s) / a[r * n + r];
    }
    x[..n].iter().all(|v| v.is_finite())
}

// ---- bundle adjustment ----

/// Huber: (cost, IRLS weight) of a scaled residual of norm `s`.
fn huber(s: f64, h: f64) -> (f64, f64) { if s <= h { (s * s, 1.0) } else { (2.0 * h * s - h * h, h / s) } }

/// Projection of `x` and its Jacobian with respect to `x` (2 x 3), None behind the camera.
fn proj_jac(cam: &Camera, x: &Vec3) -> Option<([f64; 2], [[f64; 3]; 2], Vec3)> {
    let d = kinematics::sub(x, &cam.c);
    let rt = kinematics::transpose(&cam.r);
    let xc = kinematics::mat_vec(&rt, &d);
    if xc[2] <= 1e-3 { return None; }
    let (iz, iz2) = (1.0 / xc[2], 1.0 / (xc[2] * xc[2]));
    let jp = [[cam.fx * iz, 0.0, -cam.fx * xc[0] * iz2], [0.0, cam.fy * iz, -cam.fy * xc[1] * iz2]];
    let mut jx = [[0.0; 3]; 2];
    for r in 0..2 { for c in 0..3 { jx[r][c] = (0..3).map(|k| jp[r][k] * rt[k][c]).sum(); } }
    Some(([cam.fx * xc[0] * iz + cam.cx, cam.fy * xc[1] * iz + cam.cy], jx, d))
}

struct Problem<'a> {
    corrs: &'a [Corr],
    /// Per correspondence and observation: used.
    active: Vec<Vec<bool>>,
    table_z: f64,
    /// Coplanarity weight (px per m); 0 = none.
    plane_w: f64,
    prior_rad: f64,
    huber_px: f64,
    marker_weight: f64,
}

struct State {
    cams: Vec<Camera>,
    /// Accumulated rotation vector per view.
    w: Vec<Vec3>,
    x: Vec<Vec3>,
}

impl Problem<'_> {
    fn weight(&self, c: &Corr) -> f64 { if c.marker { self.marker_weight } else { 1.0 } }

    fn cost(&self, s: &State, free: &[Option<usize>]) -> f64 {
        let mut cost = 0.0;
        for (i, c) in self.corrs.iter().enumerate() {
            let wt = self.weight(c);
            for (k, (a, q)) in c.obs.iter().enumerate() {
                if !self.active[i][k] { continue; }
                match s.cams[*a].project(&s.x[i]) {
                    Some(p) => cost += huber(wt * (p[0] - q[0]).hypot(p[1] - q[1]), self.huber_px).0,
                    None => cost += 1e6,
                }
            }
            if c.on_table && self.plane_w > 0.0 { cost += huber(((s.x[i][2] - self.table_z) * self.plane_w).abs(), self.huber_px).0; }
        }
        for (a, f) in free.iter().enumerate() {
            if f.is_some() { cost += s.w[a].iter().map(|v| (v / self.prior_rad).powi(2)).sum::<f64>(); }
        }
        cost
    }

    /// Levenberg-Marquardt on the rotations of the `free` views (index into the parameter
    /// blocks) and the points.
    fn solve(&self, s: &mut State, free: &[Option<usize>]) {
        #[cfg(feature = "mview-profile")]
        let t_call = Instant::now();
        #[cfg(feature = "mview-profile")]
        let mut cp = {
            let (role, round) = lm_prof::role();
            LmCallProfile { role, round, n_free: free.iter().filter(|f| f.is_some()).count(), n_points: self.corrs.len(), exit: "iters", ..Default::default() }
        };
        let nf = free.iter().filter(|f| f.is_some()).count();
        let nc = 3 * nf;
        let mut lambda = 1e-3;
        let t = ProfT::start();
        let mut cost = self.cost(s, free);
        t.stop("lm.trial_cost_ms");
        // buffers of every iteration (refilled; the arithmetic is the same as with fresh ones)
        let mut sm = vec![0.0; nc * nc];
        let mut bc = vec![0.0; nc];
        let mut rhs = vec![0.0; nc];
        let mut dc = vec![0.0; nc];
        // per point: Hpp^-1, gp, and the range of its blocks with the free views in `hpc_all`
        let mut per: Vec<Option<(Mat3, Vec3, usize, usize)>> = Vec::with_capacity(self.corrs.len());
        let mut hpc_all: Vec<(usize, Mat3)> = Vec::with_capacity(self.corrs.len() * nf.min(4));
        let mut trial = State { cams: s.cams.clone(), w: s.w.clone(), x: s.x.clone() };
        prof_count("lm.alloc", 8);
        for _ in 0..LM_ITERS {
            #[cfg(feature = "mview-profile")]
            {
                cp.iters += 1;
            }
            sm.fill(0.0);
            bc.fill(0.0);
            per.clear();
            hpc_all.clear();
            for (i, c) in self.corrs.iter().enumerate() {
                let t_asm = ProfT::start();
                let wt = self.weight(c);
                let mut hpp = [[0.0; 3]; 3];
                let mut gp = [0.0; 3];
                let h0 = hpc_all.len();
                for (k, (a, q)) in c.obs.iter().enumerate() {
                    if !self.active[i][k] { continue; }
                    let Some((p, jx, d)) = proj_jac(&s.cams[*a], &s.x[i]) else { continue; };
                    let r = [p[0] - q[0], p[1] - q[1]];
                    let (_, hw) = huber(wt * r[0].hypot(r[1]), self.huber_px);
                    let wgt = wt * wt * hw;
                    for u in 0..3 {
                        gp[u] += wgt * (jx[0][u] * r[0] + jx[1][u] * r[1]);
                        for v in 0..3 { hpp[u][v] += wgt * (jx[0][u] * jx[0][v] + jx[1][u] * jx[1][v]); }
                    }
                    if let Some(fi) = free[*a] {
                        // d(pixel)/d(delta) = jx [d]x
                        let dx = [[0.0, -d[2], d[1]], [d[2], 0.0, -d[0]], [-d[1], d[0], 0.0]];
                        let mut jd = [[0.0; 3]; 2];
                        for rr in 0..2 { for cc in 0..3 { jd[rr][cc] = (0..3).map(|l| jx[rr][l] * dx[l][cc]).sum(); } }
                        let o = 3 * fi;
                        let mut b = [[0.0; 3]; 3];
                        for u in 0..3 {
                            bc[o + u] += wgt * (jd[0][u] * r[0] + jd[1][u] * r[1]);
                            for v in 0..3 {
                                sm[(o + u) * nc + o + v] += wgt * (jd[0][u] * jd[0][v] + jd[1][u] * jd[1][v]);
                                b[u][v] = wgt * (jx[0][u] * jd[0][v] + jx[1][u] * jd[1][v]);
                            }
                        }
                        match hpc_all[h0..].iter_mut().find(|e| e.0 == fi) {
                            Some(e) => for u in 0..3 { for v in 0..3 { e.1[u][v] += b[u][v]; } },
                            None => hpc_all.push((fi, b)),
                        }
                    }
                }
                if c.on_table && self.plane_w > 0.0 {
                    let r = (s.x[i][2] - self.table_z) * self.plane_w;
                    let (_, hw) = huber(r.abs(), self.huber_px);
                    hpp[2][2] += hw * self.plane_w * self.plane_w;
                    gp[2] += hw * self.plane_w * r;
                }
                for u in 0..3 { hpp[u][u] = hpp[u][u] * (1.0 + lambda) + 1e-9; }
                t_asm.stop("lm.assembly_ms");
                let t_schur = ProfT::start();
                let Some(inv) = inv3(&hpp) else { per.push(None); continue; };
                let (h1, hpc) = (hpc_all.len(), &hpc_all[h0..]);
                // Schur complement: S -= B1^T inv B2, b -= B1^T inv gp
                for (f1, b1) in hpc {
                    let t = mat_mul(&kinematics::transpose(b1), &inv);
                    let tg = kinematics::mat_vec(&t, &gp);
                    for u in 0..3 { bc[3 * f1 + u] -= tg[u]; }
                    for (f2, b2) in hpc {
                        let m = mat_mul(&t, b2);
                        for u in 0..3 { for v in 0..3 { sm[(3 * f1 + u) * nc + 3 * f2 + v] -= m[u][v]; } }
                    }
                }
                per.push(Some((inv, gp, h0, h1)));
                t_schur.stop("lm.schur_ms");
            }
            for (a, f) in free.iter().enumerate() {
                let Some(fi) = f else { continue; };
                let ip = 1.0 / (self.prior_rad * self.prior_rad);
                for u in 0..3 {
                    sm[(3 * fi + u) * nc + 3 * fi + u] += ip;
                    bc[3 * fi + u] += ip * s.w[a][u];
                }
            }
            for u in 0..nc { sm[u * nc + u] *= 1.0 + lambda; }
            let t = ProfT::start();
            if nc > 0 {
                for (r, v) in rhs.iter_mut().zip(&bc) { *r = -v; }
                if !solve_dense_in(&mut sm, &mut rhs, nc, &mut dc) {
                    #[cfg(feature = "mview-profile")]
                    {
                        cp.exit = "singular";
                    }
                    break;
                }
            }
            t.stop("lm.dense_ms");
            // back-substitution and the trial state
            let t = ProfT::start();
            trial.cams.copy_from_slice(&s.cams);
            trial.w.copy_from_slice(&s.w);
            trial.x.copy_from_slice(&s.x);
            for (i, e) in per.iter().enumerate() {
                let Some((inv, gp, h0, h1)) = e else { continue; };
                let mut g = *gp;
                for (fi, b) in &hpc_all[*h0..*h1] {
                    let bd = kinematics::mat_vec(b, &[dc[3 * fi], dc[3 * fi + 1], dc[3 * fi + 2]]);
                    for u in 0..3 { g[u] += bd[u]; }
                }
                let dx = kinematics::mat_vec(inv, &g);
                for u in 0..3 { trial.x[i][u] -= dx[u]; }
            }
            for (a, f) in free.iter().enumerate() {
                let Some(fi) = f else { continue; };
                let d = [dc[3 * fi], dc[3 * fi + 1], dc[3 * fi + 2]];
                trial.cams[a].r = mat_mul(&rodrigues(&d), &trial.cams[a].r);
                for u in 0..3 { trial.w[a][u] += d[u]; }
            }
            t.stop("lm.backsub_ms");
            let t = ProfT::start();
            let tc = self.cost(&trial, free);
            t.stop("lm.trial_cost_ms");
            if tc < cost {
                #[cfg(feature = "mview-profile")]
                {
                    cp.accepted += 1;
                }
                let gain = (cost - tc) / cost.max(1e-12);
                std::mem::swap(s, &mut trial);
                cost = tc;
                lambda = (lambda * 0.3).max(1e-7);
                if gain < 1e-7 {
                    #[cfg(feature = "mview-profile")]
                    {
                        cp.exit = "gain";
                    }
                    break;
                }
            } else {
                #[cfg(feature = "mview-profile")]
                {
                    cp.rejected += 1;
                }
                lambda *= 10.0;
                if lambda > 1e6 {
                    #[cfg(feature = "mview-profile")]
                    {
                        cp.exit = "lambda";
                    }
                    break;
                }
            }
        }
        #[cfg(feature = "mview-profile")]
        {
            cp.ms = ms(t_call);
            lm_prof::push(cp);
        }
    }

    /// Reprojection rms (px) per view over the active observations.
    fn rms(&self, s: &State, nv: usize) -> Vec<Option<f64>> {
        let _t = RmsTimer(ProfT::start());
        let mut acc = vec![(0.0, 0usize); nv];
        for (i, c) in self.corrs.iter().enumerate() {
            for (k, (a, q)) in c.obs.iter().enumerate() {
                if !self.active[i][k] { continue; }
                if let Some(p) = s.cams[*a].project(&s.x[i]) { let e = (p[0] - q[0]).hypot(p[1] - q[1]); acc[*a].0 += e * e; acc[*a].1 += 1; }
            }
        }
        acc.into_iter().map(|(ss, n)| (n > 0).then(|| (ss / n as f64).sqrt())).collect()
    }
}

struct RmsTimer(ProfT);
impl Drop for RmsTimer { fn drop(&mut self) { std::mem::replace(&mut self.0, ProfT::start()).stop("lm.rms_ms"); } }
struct InitTimer(ProfT);
impl Drop for InitTimer { fn drop(&mut self) { std::mem::replace(&mut self.0, ProfT::start()).stop("lm.init_ms"); } }

/// Initial points: the triangulation of the observations (the table plane for a point that
/// does not triangulate).
fn init_points(cams: &[Camera], corrs: &[Corr], table_z: f64) -> Vec<Vec3> {
    let _t = InitTimer(ProfT::start());
    prof_count("lm.init_calls", 1);
    corrs.iter().map(|c| {
        let cs: Vec<Camera> = c.obs.iter().map(|o| cams[o.0]).collect();
        let ps: Vec<[f64; 2]> = c.obs.iter().map(|o| o.1).collect();
        triangulate(&cs, &ps).map(|t| t.0).unwrap_or_else(|| {
            let (cam, q) = (cams[c.obs[0].0], c.obs[0].1);
            let d = cam.ray(q);
            let t = if d[2].abs() > 1e-9 { (table_z - cam.c[2]) / d[2] } else { 0.4 };
            [cam.c[0] + t * d[0], cam.c[1] + t * d[1], cam.c[2] + t * d[2]]
        })
    }).collect()
}

/// The whole refinement from the images: table-texture correspondences (unless `texture` is
/// false) plus `extra` (marker corners), solved; then the texture matched again within
/// `guided_window_px` of the prediction of the refined cameras (views the first solve did not
/// refine keep `window_px`), and solved again from the kinematic cameras.
pub fn refine_views(views: &[View], reference: usize, object_box: [f64; 4], exclude: &[[f64; 4]], table_z: f64, extra: &[Corr], texture: bool, pp: &PoseParams) -> PoseRefinement {
    refine_views_impl(views, None, reference, object_box, exclude, table_z, extra, texture, pp)
}

/// [`refine_views`] with the images prepared for `views` ([`Prepared`]), shared with
/// [`reconstruct_prepared`] so that they are built once.
pub fn refine_views_prepared(views: &[View], prep: &Prepared, reference: usize, object_box: [f64; 4], exclude: &[[f64; 4]], table_z: f64, extra: &[Corr], texture: bool, pp: &PoseParams) -> PoseRefinement {
    refine_views_impl(views, Some(prep), reference, object_box, exclude, table_z, extra, texture, pp)
}

fn refine_views_impl(views: &[View], prep_imgs: Option<&Prepared>, reference: usize, object_box: [f64; 4], exclude: &[[f64; 4]], table_z: f64, extra: &[Corr], texture: bool, pp: &PoseParams) -> PoseRefinement {
    let t0 = Instant::now();
    let cams: Vec<Camera> = views.iter().map(|v| v.cam).collect();
    let with = |mut c: Vec<Corr>| { c.extend_from_slice(extra); c };
    #[cfg(feature = "mview-profile")]
    let _ = (crate::stage_prof::take(), lm_prof::take());
    #[cfg(feature = "mview-profile")]
    let t_prep = Instant::now();
    let prep = texture.then(|| match cams.get(reference) {
        Some(rc) => TexturePrep::prepared(views, prep_imgs, rc, reference, object_box, exclude, table_z),
        None => TexturePrep { reference, halves: std::borrow::Cow::Owned(Vec::new()), feats: Vec::new() },
    });
    #[cfg(feature = "mview-profile")]
    let prep_ms = if texture { ms(t_prep) } else { 0.0 };
    #[cfg(feature = "mview-profile")]
    let prep_stages = crate::stage_prof::take();
    let mut cache = prep.as_ref().map(|p| MatchCache::new(p.feats.len(), views.len()));
    #[cfg(feature = "mview-profile")]
    let mut pass1_prof = PosePassProfile::new_seeded();
    #[cfg(feature = "mview-profile")]
    let t_p1_match = Instant::now();
    let first = match (&prep, cache.as_mut()) {
        (Some(p), c) => with(p.corrs_with_cache_profiled(
            views,
            &cams,
            &vec![pp.window_px; views.len()],
            c,
            #[cfg(feature = "mview-profile")]
            Some(&mut pass1_prof),
        )),
        (None, _) => extra.to_vec(),
    };
    #[cfg(feature = "mview-profile")]
    let pass1_match_ms = ms(t_p1_match);
    #[cfg(feature = "mview-profile")]
    let t_p1_solve = Instant::now();
    // with the texture, pass one only steers pass two (statuses and cameras): no diagnostics
    let r1 = refine_poses_with(&cams, reference, &first, table_z, pp, prep.is_none());
    #[cfg(feature = "mview-profile")]
    let pass1_solve_ms = ms(t_p1_solve);
    #[cfg(feature = "mview-profile")]
    let pass1_lm = lm_prof::take();
    #[cfg(feature = "mview-profile")]
    let mut pass2_prof = PosePassProfile::new_seeded();
    #[cfg(feature = "mview-profile")]
    let (mut pass2_match_ms, mut pass2_solve_ms) = (0.0, 0.0);
    let mut out = match (&prep, cache.as_mut()) {
        (Some(p), c) => {
            let win: Vec<f64> = r1.views.iter().map(|v| if v.status == "refined" { pp.guided_window_px } else { pp.window_px }).collect();
            #[cfg(feature = "mview-profile")]
            let t_p2_match = Instant::now();
            #[cfg(feature = "mview-profile")]
            let _ = crate::stage_prof::take();
            let second = with(p.corrs_with_cache_profiled(
                views,
                &r1.cams,
                &win,
                c,
                #[cfg(feature = "mview-profile")]
                Some(&mut pass2_prof),
            ));
            #[cfg(feature = "mview-profile")]
            {
                pass2_match_ms = ms(t_p2_match);
            }
            #[cfg(feature = "mview-profile")]
            let t_p2_solve = Instant::now();
            let r2 = refine_poses(&cams, reference, &second, table_z, pp);
            #[cfg(feature = "mview-profile")]
            {
                pass2_solve_ms = ms(t_p2_solve);
            }
            r2
        }
        (None, _) => r1,
    };
    out.ms = ms(t0);
    #[cfg(feature = "mview-profile")]
    {
        out.profile = Some(PoseProfile {
            prep_stages,
            pass1_lm,
            pass2_lm: if prep.is_some() { lm_prof::take() } else { LmProfile::default() },
            prep_ms,
            pass1_match_ms,
            pass1_solve_ms,
            pass2_match_ms,
            pass2_solve_ms,
            total_ms: out.ms,
            pass1: pass1_prof,
            pass2: pass2_prof,
        });
    }
    out
}

/// Refine the rotations of every view but `reference` from `corrs` (see the module doc).
pub fn refine_poses(cams: &[Camera], reference: usize, corrs: &[Corr], table_z: f64, pp: &PoseParams) -> PoseRefinement {
    refine_poses_with(cams, reference, corrs, table_z, pp, true)
}

/// [`refine_poses`]; without `diagnostics` the per-view `rms_before_px` / `rms_after_px` are None
/// (the fixed-camera solve and the rms passes are skipped; cameras and statuses are the same).
fn refine_poses_with(cams: &[Camera], reference: usize, corrs: &[Corr], table_z: f64, pp: &PoseParams, diagnostics: bool) -> PoseRefinement {
    let t0 = Instant::now();
    let nv = cams.len();
    let mut prob = Problem { corrs, active: corrs.iter().map(|c| vec![true; c.obs.len()]).collect(), table_z,
        plane_w: pp.plane_px_per_mm * 1000.0, prior_rad: pp.prior_rad, huber_px: pp.huber_px, marker_weight: pp.marker_weight };
    let count = |prob: &Problem, a: usize| -> (usize, usize) {
        let mut n = (0, 0);
        for (i, c) in corrs.iter().enumerate() {
            if c.obs.iter().enumerate().any(|(k, o)| o.0 == a && prob.active[i][k]) && prob.active[i].iter().filter(|x| **x).count() >= 2 {
                n.0 += 1;
                if c.marker { n.1 += 1; }
            }
        }
        n
    };
    let mut fixed: Vec<Option<&'static str>> = (0..nv).map(|a| (a == reference).then_some("reference")).collect();
    for a in 0..nv { if fixed[a].is_none() && count(&prob, a).0 < pp.min_points { fixed[a] = Some("too_few_points"); } }
    let free_of = |fixed: &[Option<&'static str>]| -> Vec<Option<usize>> {
        let mut k = 0;
        fixed.iter().map(|f| if f.is_none() { k += 1; Some(k - 1) } else { None }).collect()
    };
    // every solve starts from the kinematic cameras and these points
    let x_init = init_points(cams, corrs, table_z);
    let fresh = || State { cams: cams.to_vec(), w: vec![[0.0; 3]; nv], x: x_init.clone() };
    let mut st = fresh();
    let mut rejected = vec![[0.0; 3]; nv];
    // the observations and free views of round 0's solve
    let mut round0: Option<(Vec<Vec<bool>>, Vec<Option<usize>>)> = None;
    // round 0: solve and drop the outliers; then solve until no free view exceeds the cap
    for round in 0..4 {
        let free = free_of(&fixed);
        if free.iter().all(Option::is_none) { st = fresh(); break; }
        // round 1 with round 0's observations and free views is round 0's solve (same start): reuse it
        let same_as_round0 = round == 1 && round0.as_ref().is_some_and(|(a, f)| *a == prob.active && *f == free);
        prof_count("lm.round_reused", u64::from(same_as_round0));
        if !same_as_round0 {
            st = fresh();
            lm_role("round", round);
            prob.solve(&mut st, &free);
        }
        if round == 0 {
            round0 = Some((prob.active.clone(), free));
            for (i, c) in corrs.iter().enumerate() {
                for (k, (a, q)) in c.obs.iter().enumerate() {
                    if st.cams[*a].project(&st.x[i]).is_none_or(|p| (p[0] - q[0]).hypot(p[1] - q[1]) > pp.outlier_px) { prob.active[i][k] = false; }
                }
                if prob.active[i].iter().filter(|x| **x).count() < 2 { prob.active[i].iter_mut().for_each(|x| *x = false); }
            }
            for a in 0..nv { if fixed[a].is_none() && count(&prob, a).0 < pp.min_points { fixed[a] = Some("too_few_points"); } }
            continue;
        }
        let mut changed = false;
        for a in 0..nv {
            if fixed[a].is_none() && kinematics::norm(&st.w[a]) > pp.max_correction_rad {
                fixed[a] = Some("correction_too_large");
                rejected[a] = st.w[a];
                changed = true;
            }
        }
        if !changed { break; }
        if round == 3 { st = fresh(); let free = free_of(&fixed); if free.iter().any(Option::is_some) { lm_role("cap_resolve", round); prob.solve(&mut st, &free); } }
    }
    if free_of(&fixed).iter().all(Option::is_none) { st = fresh(); }
    // before: the same observations triangulated with the kinematic cameras (no coplanarity)
    let (before, after) = if diagnostics {
        let pb = Problem { corrs, active: prob.active.clone(), table_z, plane_w: 0.0, prior_rad: pp.prior_rad, huber_px: pp.huber_px, marker_weight: pp.marker_weight };
        let mut s0 = fresh();
        lm_role("before", 0);
        pb.solve(&mut s0, &vec![None; nv]);
        (pb.rms(&s0, nv), prob.rms(&st, nv))
    } else {
        (vec![None; nv], vec![None; nv])
    };
    let views = (0..nv).map(|a| {
        let (n, nm) = count(&prob, a);
        let w = if fixed[a] == Some("correction_too_large") { rejected[a] } else { st.w[a] };
        ViewRefine {
            status: fixed[a].unwrap_or("refined"),
            rot_mrad: w.map(|v| v * 1000.0), correction_mrad: kinematics::norm(&w) * 1000.0,
            n_points: n, n_marker_corners: nm, rms_before_px: before[a], rms_after_px: after[a],
        }
    }).collect();
    PoseRefinement {
        cams: st.cams,
        views,
        n_corrs: corrs.len(),
        n_marker_corrs: corrs.iter().filter(|c| c.marker).count(),
        ms: ms(t0),
        #[cfg(feature = "mview-profile")]
        profile: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cam_at(c: Vec3, look: Vec3) -> Camera {
        let z = kinematics::unit(&kinematics::sub(&look, &c));
        let x = kinematics::unit(&kinematics::cross(&z, &[0.0, 1.0, 0.0]));
        let y = kinematics::cross(&z, &x);
        Camera { r: kinematics::from_columns(&x, &y, &z), c, fx: 804.0, fy: 804.0, cx: 320.0, cy: 180.0 }
    }

    /// Rotation angle between two rotation matrices (rad).
    fn angle(a: &Mat3, b: &Mat3) -> f64 {
        let m = mat_mul(&kinematics::transpose(a), b);
        ((m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0).acos()
    }

    /// Table points seen by three views ±4 cm apart; views 1 and 2 carry a 1.5 mrad rotation
    /// error (and two observations are wrong): the refinement recovers the true rotations.
    #[test]
    fn rotation_errors_are_recovered_from_table_points() {
        let look = [0.30, 0.0, 0.0];
        let truth = [cam_at([0.30, 0.0, 0.45], look), cam_at([0.30, -0.04, 0.45], look), cam_at([0.30, 0.04, 0.45], look)];
        let mut kin = truth;
        kin[1].r = mat_mul(&rodrigues(&[0.0015, 0.0, 0.0]), &kin[1].r);
        kin[2].r = mat_mul(&rodrigues(&[0.0, -0.001, 0.0011]), &kin[2].r);
        let mut corrs = Vec::new();
        for i in 0..8 {
            for j in 0..6 {
                let x = [0.22 + 0.022 * f64::from(i), -0.10 + 0.04 * f64::from(j), 0.0];
                let obs: Vec<(usize, [f64; 2])> = truth.iter().enumerate().filter_map(|(k, c)| c.project(&x).map(|q| (k, q))).collect();
                corrs.push(Corr { obs, on_table: true, marker: false });
            }
        }
        corrs[3].obs[1].1[0] += 6.0;
        corrs[10].obs[2].1[1] -= 9.0;
        let r = refine_poses(&kin, 0, &corrs, 0.0, &PoseParams::default());
        assert_eq!(r.views[0].status, "reference");
        for k in 1..3 {
            let v = r.views[k];
            assert_eq!(v.status, "refined", "{v:?}");
            assert!(angle(&r.cams[k].r, &truth[k].r) < 1e-4, "view {k}: {} mrad off, {v:?}", angle(&r.cams[k].r, &truth[k].r) * 1000.0);
            assert!((v.correction_mrad - angle(&kin[k].r, &truth[k].r) * 1000.0).abs() < 0.1, "{v:?}");
            assert!(v.rms_after_px.unwrap() < 0.05 && v.rms_before_px.unwrap() > 0.3, "{v:?}");
            assert_eq!(v.n_points, 47, "one outlier observation per view dropped");
        }
        // too few points: the kinematic pose is kept
        let r = refine_poses(&kin, 0, &corrs[..8], 0.0, &PoseParams::default());
        assert!(r.views[1..].iter().all(|v| v.status == "too_few_points"), "{:?}", r.views);
        assert_eq!(r.cams, kin.to_vec());
        // a correction above the cap is reported and not applied
        let pp = PoseParams { max_correction_rad: 0.0013, ..PoseParams::default() };
        let r = refine_poses(&kin, 0, &corrs, 0.0, &pp);
        assert_eq!((r.views[1].status, r.views[2].status), ("correction_too_large", "correction_too_large"), "{:?}", r.views);
        assert!((r.views[1].correction_mrad - 1.5).abs() < 0.2, "{:?}", r.views[1]);
        assert_eq!(r.cams, kin.to_vec());
    }

    /// Solve the dense n x n system `a x = b` (Gaussian elimination, partial pivoting).
    fn solve_dense_oracle(mut a: Vec<f64>, mut b: Vec<f64>, n: usize) -> Option<Vec<f64>> {
        for col in 0..n {
            let piv = (col..n).max_by(|&i, &j| a[i * n + col].abs().total_cmp(&a[j * n + col].abs()))?;
            if a[piv * n + col].abs() < 1e-15 { return None; }
            if piv != col {
                for k in 0..n { a.swap(piv * n + k, col * n + k); }
                b.swap(piv, col);
            }
            for r in col + 1..n {
                let f = a[r * n + col] / a[col * n + col];
                if f == 0.0 { continue; }
                for k in col..n { a[r * n + k] -= f * a[col * n + k]; }
                b[r] -= f * b[col];
            }
        }
        let mut x = vec![0.0; n];
        for r in (0..n).rev() {
            let s: f64 = (r + 1..n).map(|k| a[r * n + k] * x[k]).sum();
            x[r] = (b[r] - s) / a[r * n + r];
        }
        x.iter().all(|v| v.is_finite()).then_some(x)
    }


    fn solve_oracle(pr: &Problem, s: &mut State, free: &[Option<usize>]) {
        let nf = free.iter().filter(|f| f.is_some()).count();
        let nc = 3 * nf;
        let mut lambda = 1e-3;
        let mut cost = pr.cost(s, free);
        for _ in 0..LM_ITERS {
            let mut sm = vec![0.0; nc * nc];
            let mut bc = vec![0.0; nc];
            // per point: Hpp^-1, gp, and its blocks with the free views
            let mut per: Vec<Option<(Mat3, Vec3, Vec<(usize, Mat3)>)>> = Vec::with_capacity(pr.corrs.len());
            for (i, c) in pr.corrs.iter().enumerate() {
                let wt = pr.weight(c);
                let mut hpp = [[0.0; 3]; 3];
                let mut gp = [0.0; 3];
                let mut hpc: Vec<(usize, Mat3)> = Vec::new();
                for (k, (a, q)) in c.obs.iter().enumerate() {
                    if !pr.active[i][k] { continue; }
                    let Some((p, jx, d)) = proj_jac(&s.cams[*a], &s.x[i]) else { continue; };
                    let r = [p[0] - q[0], p[1] - q[1]];
                    let (_, hw) = huber(wt * r[0].hypot(r[1]), pr.huber_px);
                    let wgt = wt * wt * hw;
                    for u in 0..3 {
                        gp[u] += wgt * (jx[0][u] * r[0] + jx[1][u] * r[1]);
                        for v in 0..3 { hpp[u][v] += wgt * (jx[0][u] * jx[0][v] + jx[1][u] * jx[1][v]); }
                    }
                    if let Some(fi) = free[*a] {
                        // d(pixel)/d(delta) = jx [d]x
                        let dx = [[0.0, -d[2], d[1]], [d[2], 0.0, -d[0]], [-d[1], d[0], 0.0]];
                        let mut jd = [[0.0; 3]; 2];
                        for rr in 0..2 { for cc in 0..3 { jd[rr][cc] = (0..3).map(|l| jx[rr][l] * dx[l][cc]).sum(); } }
                        let o = 3 * fi;
                        let mut b = [[0.0; 3]; 3];
                        for u in 0..3 {
                            bc[o + u] += wgt * (jd[0][u] * r[0] + jd[1][u] * r[1]);
                            for v in 0..3 {
                                sm[(o + u) * nc + o + v] += wgt * (jd[0][u] * jd[0][v] + jd[1][u] * jd[1][v]);
                                b[u][v] = wgt * (jx[0][u] * jd[0][v] + jx[1][u] * jd[1][v]);
                            }
                        }
                        match hpc.iter_mut().find(|e| e.0 == fi) {
                            Some(e) => for u in 0..3 { for v in 0..3 { e.1[u][v] += b[u][v]; } },
                            None => hpc.push((fi, b)),
                        }
                    }
                }
                if c.on_table && pr.plane_w > 0.0 {
                    let r = (s.x[i][2] - pr.table_z) * pr.plane_w;
                    let (_, hw) = huber(r.abs(), pr.huber_px);
                    hpp[2][2] += hw * pr.plane_w * pr.plane_w;
                    gp[2] += hw * pr.plane_w * r;
                }
                for u in 0..3 { hpp[u][u] = hpp[u][u] * (1.0 + lambda) + 1e-9; }
                let Some(inv) = inv3(&hpp) else { per.push(None); continue; };
                // Schur complement: S -= B1^T inv B2, b -= B1^T inv gp
                for (f1, b1) in &hpc {
                    let t = mat_mul(&kinematics::transpose(b1), &inv);
                    let tg = kinematics::mat_vec(&t, &gp);
                    for u in 0..3 { bc[3 * f1 + u] -= tg[u]; }
                    for (f2, b2) in &hpc {
                        let m = mat_mul(&t, b2);
                        for u in 0..3 { for v in 0..3 { sm[(3 * f1 + u) * nc + 3 * f2 + v] -= m[u][v]; } }
                    }
                }
                per.push(Some((inv, gp, hpc)));
            }
            for (a, f) in free.iter().enumerate() {
                let Some(fi) = f else { continue; };
                let ip = 1.0 / (pr.prior_rad * pr.prior_rad);
                for u in 0..3 {
                    sm[(3 * fi + u) * nc + 3 * fi + u] += ip;
                    bc[3 * fi + u] += ip * s.w[a][u];
                }
            }
            for u in 0..nc { sm[u * nc + u] *= 1.0 + lambda; }
            let dc = if nc > 0 {
                match solve_dense_oracle(sm, bc.iter().map(|v| -v).collect(), nc) {
                    Some(d) => d,
                    None => {
                        break;
                    }
                }
            } else { Vec::new() };
            // back-substitution and the trial state
            let mut trial = State { cams: s.cams.clone(), w: s.w.clone(), x: s.x.clone() };
            for (i, e) in per.iter().enumerate() {
                let Some((inv, gp, hpc)) = e else { continue; };
                let mut g = *gp;
                for (fi, b) in hpc {
                    let bd = kinematics::mat_vec(b, &[dc[3 * fi], dc[3 * fi + 1], dc[3 * fi + 2]]);
                    for u in 0..3 { g[u] += bd[u]; }
                }
                let dx = kinematics::mat_vec(inv, &g);
                for u in 0..3 { trial.x[i][u] -= dx[u]; }
            }
            for (a, f) in free.iter().enumerate() {
                let Some(fi) = f else { continue; };
                let d = [dc[3 * fi], dc[3 * fi + 1], dc[3 * fi + 2]];
                trial.cams[a].r = mat_mul(&rodrigues(&d), &trial.cams[a].r);
                for u in 0..3 { trial.w[a][u] += d[u]; }
            }
            let tc = pr.cost(&trial, free);
            if tc < cost {
                let gain = (cost - tc) / cost.max(1e-12);
                *s = trial;
                cost = tc;
                lambda = (lambda * 0.3).max(1e-7);
                if gain < 1e-7 {
                    break;
                }
            } else {
                lambda *= 10.0;
                if lambda > 1e6 {
                    break;
                }
            }
        }
    }


    /// The reused LM buffers give bitwise the solve with fresh ones (the previous solve as
    /// oracle): all views free, some fixed, none free, coplanarity on/off, markers, inactive
    /// observations, and a non-finite (singular) start.
    #[test]
    fn lm_solve_with_reused_buffers_matches_fresh_buffers_bitwise() {
        let look = [0.30, 0.0, 0.0];
        let truth = [cam_at([0.30, 0.0, 0.45], look), cam_at([0.30, -0.04, 0.45], look), cam_at([0.30, 0.04, 0.45], look), cam_at([0.34, 0.0, 0.45], look)];
        let mut kin = truth;
        kin[1].r = mat_mul(&rodrigues(&[0.0015, 0.0, 0.0]), &kin[1].r);
        kin[2].r = mat_mul(&rodrigues(&[0.0, -0.001, 0.0011]), &kin[2].r);
        kin[3].r = mat_mul(&rodrigues(&[0.0, 0.0006, -0.0021]), &kin[3].r);
        let mut corrs = Vec::new();
        for i in 0..8 {
            for j in 0..6 {
                let x = [0.22 + 0.022 * f64::from(i), -0.10 + 0.04 * f64::from(j), 0.0];
                let obs: Vec<(usize, [f64; 2])> = truth.iter().enumerate().filter_map(|(k, c)| c.project(&x).map(|q| (k, q))).collect();
                corrs.push(Corr { obs, on_table: (i + j) % 3 != 0, marker: j == 5 });
            }
        }
        corrs[3].obs[1].1[0] += 6.0;
        corrs[10].obs[2].1[1] -= 9.0;
        let mut active: Vec<Vec<bool>> = corrs.iter().map(|c| vec![true; c.obs.len()]).collect();
        active[7][2] = false;
        active[11].iter_mut().for_each(|a| *a = false);
        let nv = kin.len();
        let x0 = init_points(&kin, &corrs, 0.0);
        let frees: [Vec<Option<usize>>; 4] = [
            vec![None, Some(0), Some(1), Some(2)],
            vec![None, None, Some(0), Some(1)],
            vec![None, Some(0), None, None],
            vec![None; 4],
        ];
        for plane_w in [0.0, 1000.0] {
            let prob = Problem { corrs: &corrs, active: active.clone(), table_z: 0.0, plane_w, prior_rad: POSE_PRIOR_RAD, huber_px: POSE_HUBER_PX, marker_weight: MARKER_WEIGHT };
            for free in &frees {
                let mut a = State { cams: kin.to_vec(), w: vec![[0.0; 3]; nv], x: x0.clone() };
                let mut b = State { cams: kin.to_vec(), w: vec![[0.0; 3]; nv], x: x0.clone() };
                prob.solve(&mut a, free);
                solve_oracle(&prob, &mut b, free);
                assert_eq!((&a.cams, &a.w, &a.x), (&b.cams, &b.w, &b.x), "plane_w {plane_w} free {free:?}");
            }
            // a point at the camera centre: singular point block, the dense solve sees NaN
            let mut bad = State { cams: kin.to_vec(), w: vec![[0.0; 3]; nv], x: x0.clone() };
            bad.x[0] = [f64::NAN; 3];
            let mut bad2 = State { cams: kin.to_vec(), w: vec![[0.0; 3]; nv], x: bad.x.clone() };
            prob.solve(&mut bad, &frees[0]);
            solve_oracle(&prob, &mut bad2, &frees[0]);
            assert_eq!(format!("{:?}", (&bad.cams, &bad.w, &bad.x)), format!("{:?}", (&bad2.cams, &bad2.w, &bad2.x)));
        }
    }

    /// The refine_poses of before the shared initial points, the round-0 reuse and the
    /// diagnostics switch (oracle for the equivalence tests).
    fn refine_poses_oracle(cams: &[Camera], reference: usize, corrs: &[Corr], table_z: f64, pp: &PoseParams) -> (Vec<Camera>, Vec<ViewRefine>) {
        let nv = cams.len();
        let mut prob = Problem { corrs, active: corrs.iter().map(|c| vec![true; c.obs.len()]).collect(), table_z,
            plane_w: pp.plane_px_per_mm * 1000.0, prior_rad: pp.prior_rad, huber_px: pp.huber_px, marker_weight: pp.marker_weight };
        let count = |prob: &Problem, a: usize| -> (usize, usize) {
            let mut n = (0, 0);
            for (i, c) in corrs.iter().enumerate() {
                if c.obs.iter().enumerate().any(|(k, o)| o.0 == a && prob.active[i][k]) && prob.active[i].iter().filter(|x| **x).count() >= 2 {
                    n.0 += 1;
                    if c.marker { n.1 += 1; }
                }
            }
            n
        };
        let mut fixed: Vec<Option<&'static str>> = (0..nv).map(|a| (a == reference).then_some("reference")).collect();
        for a in 0..nv { if fixed[a].is_none() && count(&prob, a).0 < pp.min_points { fixed[a] = Some("too_few_points"); } }
        let free_of = |fixed: &[Option<&'static str>]| -> Vec<Option<usize>> {
            let mut k = 0;
            fixed.iter().map(|f| if f.is_none() { k += 1; Some(k - 1) } else { None }).collect()
        };
        let fresh = || State { cams: cams.to_vec(), w: vec![[0.0; 3]; nv], x: init_points(cams, corrs, table_z) };
        let mut st = fresh();
        let mut rejected = vec![[0.0; 3]; nv];
        for round in 0..4 {
            st = fresh();
            let free = free_of(&fixed);
            if free.iter().all(Option::is_none) { break; }
            prob.solve(&mut st, &free);
            if round == 0 {
                for (i, c) in corrs.iter().enumerate() {
                    for (k, (a, q)) in c.obs.iter().enumerate() {
                        if st.cams[*a].project(&st.x[i]).is_none_or(|p| (p[0] - q[0]).hypot(p[1] - q[1]) > pp.outlier_px) { prob.active[i][k] = false; }
                    }
                    if prob.active[i].iter().filter(|x| **x).count() < 2 { prob.active[i].iter_mut().for_each(|x| *x = false); }
                }
                for a in 0..nv { if fixed[a].is_none() && count(&prob, a).0 < pp.min_points { fixed[a] = Some("too_few_points"); } }
                continue;
            }
            let mut changed = false;
            for a in 0..nv {
                if fixed[a].is_none() && kinematics::norm(&st.w[a]) > pp.max_correction_rad {
                    fixed[a] = Some("correction_too_large");
                    rejected[a] = st.w[a];
                    changed = true;
                }
            }
            if !changed { break; }
            if round == 3 { st = fresh(); let free = free_of(&fixed); if free.iter().any(Option::is_some) { prob.solve(&mut st, &free); } }
        }
        if free_of(&fixed).iter().all(Option::is_none) { st = fresh(); }
        let before = {
            let pb = Problem { corrs, active: prob.active.clone(), table_z, plane_w: 0.0, prior_rad: pp.prior_rad, huber_px: pp.huber_px, marker_weight: pp.marker_weight };
            let mut s0 = State { cams: cams.to_vec(), w: vec![[0.0; 3]; nv], x: init_points(cams, corrs, table_z) };
            pb.solve(&mut s0, &vec![None; nv]);
            pb.rms(&s0, nv)
        };
        let after = prob.rms(&st, nv);
        let views = (0..nv).map(|a| {
            let (n, nm) = count(&prob, a);
            let w = if fixed[a] == Some("correction_too_large") { rejected[a] } else { st.w[a] };
            ViewRefine {
                status: fixed[a].unwrap_or("refined"),
                rot_mrad: w.map(|v| v * 1000.0), correction_mrad: kinematics::norm(&w) * 1000.0,
                n_points: n, n_marker_corners: nm, rms_before_px: before[a], rms_after_px: after[a],
            }
        }).collect();
        (st.cams, views)
    }

    /// Bitwise equality with the oracle: unchanged mask (round-0 reuse), changed mask (outliers),
    /// all views fixed, marker-only, and cap rejections in round 1 and later rounds; without
    /// diagnostics only the rms fields differ (None).
    #[test]
    fn refine_poses_matches_oracle_bitwise() {
        let look = [0.30, 0.0, 0.0];
        let truth = [cam_at([0.30, 0.0, 0.45], look), cam_at([0.30, -0.04, 0.45], look), cam_at([0.30, 0.04, 0.45], look), cam_at([0.34, 0.0, 0.45], look)];
        let mut kin = truth;
        kin[1].r = mat_mul(&rodrigues(&[0.0015, 0.0, 0.0]), &kin[1].r);
        kin[2].r = mat_mul(&rodrigues(&[0.0, -0.001, 0.0011]), &kin[2].r);
        kin[3].r = mat_mul(&rodrigues(&[0.0, 0.0006, -0.0021]), &kin[3].r);
        let grid = |marker: bool| -> Vec<Corr> {
            let mut corrs = Vec::new();
            for i in 0..8 {
                for j in 0..6 {
                    let x = [0.22 + 0.022 * f64::from(i), -0.10 + 0.04 * f64::from(j), if marker { 0.003 } else { 0.0 }];
                    let obs: Vec<(usize, [f64; 2])> = truth.iter().enumerate().filter_map(|(k, c)| c.project(&x).map(|q| (k, q))).collect();
                    corrs.push(Corr { obs, on_table: !marker, marker });
                }
            }
            corrs
        };
        let clean = grid(false);
        let mut outliers = clean.clone();
        outliers[3].obs[1].1[0] += 6.0;
        outliers[10].obs[2].1[1] -= 9.0;
        outliers[20].obs[3].1[0] -= 7.0;
        let markers = grid(true);
        let cap = |c: f64| PoseParams { max_correction_rad: c, ..PoseParams::default() };
        let cases: Vec<(&str, &[Corr], PoseParams)> = vec![
            ("unchanged_mask", &clean, PoseParams::default()),
            ("changed_mask", &outliers, PoseParams::default()),
            ("all_fixed", &clean[..8], PoseParams::default()),
            ("marker_only", &markers, PoseParams::default()),
            ("cap_round1_unchanged", &clean, cap(0.0013)),
            ("cap_round1_changed", &outliers, cap(0.0013)),
            ("cap_later_rounds", &clean, cap(0.0019)),
            ("cap_all", &outliers, cap(0.0001)),
        ];
        for (name, corrs, pp) in cases {
            let (cams, views) = refine_poses_oracle(&kin, 0, corrs, 0.0, &pp);
            let r = refine_poses(&kin, 0, corrs, 0.0, &pp);
            assert_eq!(r.cams, cams, "{name}");
            assert_eq!(r.views, views, "{name}");
            let nd = refine_poses_with(&kin, 0, corrs, 0.0, &pp, false);
            assert_eq!(nd.cams, cams, "{name}");
            for (a, b) in nd.views.iter().zip(&views) {
                assert_eq!(*a, ViewRefine { rms_before_px: None, rms_after_px: None, ..*b }, "{name}");
            }
        }
    }

    #[test]
    fn marker_corners_match_by_id_and_corner() {
        let m = |id: i64, x: f64| Marker { id, corners: [[x, 10.0], [x + 10.0, 10.0], [x + 10.0, 20.0], [x, 20.0]] };
        let views = vec![vec![m(1, 100.0), m(2, 300.0)], vec![m(1, 110.0), m(7, 50.0)], vec![m(1, 120.0), m(2, 310.0), m(2, 400.0)]];
        let c = marker_corrs(&views, 0, [290.0, 0.0, 330.0, 40.0]);
        // id 1 in three views (4 corners); id 2 twice in view 2 (left out there) and once
        // elsewhere: no pair; id 7 in one view
        assert_eq!(c.len(), 4, "{c:?}");
        assert!(c.iter().all(|k| k.marker && k.on_table && k.obs.len() == 3));
        assert_eq!(c[1].obs[2], (2, [130.0, 10.0]));
        let views = vec![vec![m(2, 300.0)], vec![m(2, 310.0)]];
        assert!(marker_corrs(&views, 0, [290.0, 0.0, 330.0, 40.0]).iter().all(|k| !k.on_table), "corners in the box");
    }

    #[test]
    fn texture_prep_matches_one_shot_texture_corrs_and_handles_invalid_inputs() {
        let look = [0.30, 0.0, 0.0];
        let c0 = cam_at([0.30, -0.02, 0.45], look);
        let c1 = cam_at([0.30, 0.02, 0.45], look);
        let render = |cam: &Camera| -> Gray {
            let (w, h) = (320usize, 180usize);
            let mut cam_s = *cam;
            cam_s.cx = 160.0;
            cam_s.cy = 90.0;
            let mut px = vec![128u8; w * h];
            for y in 0..h {
                for x in 0..w {
                    let d = cam_s.ray([x as f64, y as f64]);
                    let t = -cam_s.c[2] / d[2];
                    let wx = cam_s.c[0] + t * d[0];
                    let wy = cam_s.c[1] + t * d[1];
                    let gx = (wx / 0.0045).floor() as i64;
                    let gy = (wy / 0.0045).floor() as i64;
                    let h1 = ((gx.wrapping_mul(73856093) ^ gy.wrapping_mul(19349663)).wrapping_mul(83492791) >> 24) & 0xff;
                    let gx2 = (wx / 0.0073).floor() as i64;
                    let gy2 = (wy / 0.0073).floor() as i64;
                    let h2 = ((gx2.wrapping_mul(19349663) ^ gy2.wrapping_mul(83492791)).wrapping_mul(73856093) >> 24) & 0xff;
                    px[y * w + x] = ((h1 + h2) / 2) as u8;
                }
            }
            Gray::new(w, h, px).unwrap()
        };
        let mut cam0 = c0;
        cam0.cx = 160.0;
        cam0.cy = 90.0;
        let mut cam1 = c1;
        cam1.cx = 160.0;
        cam1.cy = 90.0;
        let views = vec![
            View { cam: cam0, img: render(&c0) },
            View { cam: cam1, img: render(&c1) },
        ];
        let object_box = [140.0, 75.0, 180.0, 105.0];
        let exclude = [[0.0, 0.0, 20.0, 20.0]];

        // Invalid inputs: empty without panic
        assert!(TexturePrep::new(&views, &cam0, 2, object_box, &exclude, 0.0).corrs(&views, &[cam0, cam1], &[15.0, 15.0]).is_empty());
        assert!(texture_corrs(&views, &[cam0, cam1], 2, object_box, &exclude, 0.0, &[15.0, 15.0]).is_empty());
        assert!(TexturePrep::new(&views[..1], &cam0, 0, object_box, &exclude, 0.0).corrs(&views[..1], &[cam0], &[15.0]).is_empty());
        assert!(texture_corrs(&views[..1], &[cam0], 0, object_box, &exclude, 0.0, &[15.0]).is_empty());
        assert!(TexturePrep::new(&[], &cam0, 0, object_box, &exclude, 0.0).corrs(&[], &[], &[]).is_empty());
        assert!(texture_corrs(&[], &[], 0, object_box, &exclude, 0.0, &[]).is_empty());

        let prep = TexturePrep::new(&views, &cam0, 0, object_box, &exclude, 0.0);
        assert!(prep.corrs(&views, &[cam0], &[15.0, 15.0]).is_empty());
        assert!(prep.corrs(&views, &[cam0, cam1], &[15.0]).is_empty());
        assert!(texture_corrs(&views, &[cam0], 0, object_box, &exclude, 0.0, &[15.0, 15.0]).is_empty());
        assert!(texture_corrs(&views, &[cam0, cam1], 0, object_box, &exclude, 0.0, &[15.0]).is_empty());

        let r_oob = refine_views(&views, 5, object_box, &exclude, 0.0, &[], true, &PoseParams::default());
        assert_eq!(r_oob.n_corrs, 0);

        // Two passes on synthetic 2-view textured table scene: bit-identical to one-shot texture_corrs
        let cams1 = [cam0, cam1];
        let win1 = [15.0, 15.0];
        let got1 = prep.corrs(&views, &cams1, &win1);
        let exp1 = texture_corrs(&views, &cams1, 0, object_box, &exclude, 0.0, &win1);
        assert!(!got1.is_empty(), "synthetic textured scene should yield correspondences");
        assert_eq!(got1, exp1);

        let mut cam1_p = cam1;
        cam1_p.r = mat_mul(&rodrigues(&[0.0008, -0.0005, 0.0004]), &cam1.r);
        let cams2 = [cam0, cam1_p];
        let win2 = [15.0, 4.0];
        let got2 = prep.corrs(&views, &cams2, &win2);
        let exp2 = texture_corrs(&views, &cams2, 0, object_box, &exclude, 0.0, &win2);
        assert!(!got2.is_empty(), "guided pass should yield correspondences");
        assert_eq!(got2, exp2);
    }

    fn render_table_view(cam: &Camera) -> Gray {
        let (w, h) = (320usize, 180usize);
        let mut px = vec![128u8; w * h];
        for y in 0..h {
            for x in 0..w {
                let d = cam.ray([x as f64, y as f64]);
                let t = -cam.c[2] / d[2];
                let wx = cam.c[0] + t * d[0];
                let wy = cam.c[1] + t * d[1];
                let gx = (wx / 0.0045).floor() as i64;
                let gy = (wy / 0.0045).floor() as i64;
                let h1 = ((gx.wrapping_mul(73856093) ^ gy.wrapping_mul(19349663)).wrapping_mul(83492791) >> 24) & 0xff;
                let gx2 = (wx / 0.0073).floor() as i64;
                let gy2 = (wy / 0.0073).floor() as i64;
                let h2 = ((gx2.wrapping_mul(19349663) ^ gy2.wrapping_mul(83492791)).wrapping_mul(73856093) >> 24) & 0xff;
                px[y * w + x] = ((h1 + h2) / 2) as u8;
            }
        }
        Gray::new(w, h, px).unwrap()
    }

    #[test]
    fn coarse_cache_recomputes_boundary_peaks_on_window_change() {
        let (w, h) = (64usize, 64usize);
        let mut px = vec![128u8; w * h];
        // Textured 5x5 patch centered at (20, 20)
        let patch: [[u8; 5]; 5] = [
            [40, 200, 60, 190, 50],
            [180, 30, 210, 45, 175],
            [70, 220, 25, 205, 80],
            [195, 55, 185, 35, 215],
            [45, 170, 90, 225, 65],
        ];
        // Exact copy at (23, 20): score ~ 1.0
        for dy in 0..5 {
            for dx in 0..5 {
                px[(18 + dy) * w + (21 + dx)] = patch[dy][dx];
            }
        }
        // Slightly perturbed copy at (22, 20) — note (22, 20) overlaps (23, 20) unless separated in y or we place (22, 20) vs (23, 20):
        // Place exact copy at (23, 25) and weaker copy at (22, 25) so they are 8-neighbors? No, (22, 25) and (23, 25) overlap in 5x5!
        // Instead, construct a smooth/non-overlapping test using a 5x5 template with a non-uniform background:
        // Wait: even simpler! Reset px to flat 128, put a 5x5 patch at (10, 10) for template, and put:
        // - exact copy at (43, 40)
        // - slightly attenuated copy at (42, 40) won't overlap if we put exact copy at (20, 43) and check row-by-row? Still overlaps in 5x5!
        // Better: put exact copy at (43, 40) -> peak at (43, 40), and let's see what score (42, 40) gets, OR put a different high-contrast patch:
        // Actually, if we put a 1D-periodic or distinct patch at (42, 40), (43, 40) overlaps it by 4 cols.
        // Wait: what if the template is [0, 0, 255, 0, 0] in a row, and in the image at x=42..=44 we put [100, 200, 255]?
        // Then at x=42 (centered on 100): window sees [0, 0, 100, 200, 255] -> positive NCC (>0.7)? Let's check!
        // Even cleaner: fill px with 20, put template at (10, 10) with centered impulse: [20,20,20,20,20] except center=220 and (dx=0,dy=-1)=180.
        px.fill(20);
        // Template at (10, 10): only center (dx=2,dy=2) is bright (220), neighbors are 20.
        // Wait: at (43, 40) put pixel=220, at (42, 40) put pixel=180!
        // Check NCC with single bright pixel template at (43,40) vs (42,40) vs (44,40):
        // With single pixel 220 at (43,40) and 180 at (42,40), at x=42 center is 180 and right-neighbor is 220, so NCC at x=42 is LOWER than at x=43!
        // What if we put 200 at (42,40) and 250 at (43,40) and 20 elsewhere?
        // At x=42: window [20, 20, 200, 250, 20], template [20, 20, 220, 20, 20] -> dot ~ 200*1, at x=43: [20, 200, 250, 20, 20] -> dot ~ 250*1!
        // Even better: check NCC of template [20,20,220,20,20] (in middle row, 20 elsewhere) against [20, 20, 180, 240, 20]:
        // Wait: with 240 at (43,40) and 180 at (42,40), NCC(43,40) is positive (>0.7), while NCC(42,40) has 240 on off-center!
        // To make BOTH > 0.7 AND strictly ordered, make the template a 5x5 ramp/step or check directly:
        // Put pixel(x=42, y=40)=190, pixel(x=43, y=40)=240? No, if we use a 5-pixel tall vertical bar of width 1 at x=43 (240) and x=42 (180), wait:
        // Template: column dx=2 is 220, all other cols are 20!
        // In image at (40, 40), window [38..=42]: if x=42 is 200 and x=43 is 240:
        // Let's just compute NCC with a template that has col dx=2 = [20,20,220,20,20] -> wait, vertical bar:
        // For col dx=2 = 220, others = 20: dot with window centered at x is proportional to (sum(col x) - mean_window).
        // At x=42: col 42 has 200*5, col 43 has 240*5, mean is (200+240+60)/5 = 100 -> col42 - mean = 100! Wait: var = ... NCC is < 0.7 because col43 is also bright.
        // How to get TWO peaks > 0.7 that are 8-neighbors of each other?
        // Put them 1 pixel apart diagonally? Still 5x5 overlaps. What if they are 1 pixel apart AND have score > 0.7?
        // Can two adjacent pixels BOTH have NCC > 0.7 with the same 5x5 template?
        // YES! Smooth horizontal Gaussian / triangle! E.g. image has a blurred blob centered at x=42.7:
        // Suppose template is a 5x5 Gaussian peak: [50, 150, 240, 150, 50] in each row!
        // At x=42 vs x=43, for a wide smooth blob [20, 80, 160, 230, 250, 170, 90, 30], BOTH x=42 and x=43 match [50, 150, 240, 150, 50] with NCC > 0.85, and x=43 > x=42!
        px.fill(20);
        // Template source at (10, 10): smooth symmetric bump across dx=0..5
        let t_profile = [50u8, 150, 240, 150, 50];
        let v_profile = [40u8, 130, 255, 130, 40];
        for dy in 0..5 {
            for dx in 0..5 {
                let val = ((u16::from(t_profile[dx]) * u16::from(v_profile[dy])) / 255) as u8;
                px[(8 + dy) * w + (8 + dx)] = val;
            }
        }
        // Target smooth bump centered near x=43, y=40:
        // at x=42 (win 40..=44): [20, 60, 180, 245, 160] -> off-center, NCC ~ 0.82
        // at x=43 (win 41..=45): [60, 180, 245, 160, 55] -> centered, NCC ~ 0.99!
        let vals = [20u8, 60, 180, 245, 160, 55]; // x = 40..=45
        for dy in 0..5 {
            for (k, &tx) in vals.iter().enumerate() {
                let val = ((u16::from(tx) * u16::from(v_profile[dy])) / 255) as u8;
                px[(38 + dy) * w + (40 + k)] = val;
            }
        }
        let img = Gray::new(w, h, px).unwrap();
        let tc = make_tmpl(sample_int(&img, 10, 10, COARSE_R), COARSE_R, None, 0.0).unwrap();

        // Wide window centered at (40, 40), r = 4 (x in 36..=44):
        // (43, 40) is inside the window and dominates (42, 40), so (42, 40) is NOT a peak.
        let mut scratch = MatchScratch::default();
        let ((wide_arr, wide_len), center_wide) = window_peaks_cached(&img, &tc, [40.0, 40.0], 4, None, &mut scratch);
        let wide_peaks = &wide_arr[..wide_len];
        let uncached_wide = window_peaks(&img, &tc, [40.0, 40.0], 4);
        assert_eq!(wide_peaks, uncached_wide.as_slice());
        assert!(wide_peaks.contains(&[43.0, 40.0]), "wide window should find true peak at (43,40): {wide_peaks:?}");
        assert!(!wide_peaks.contains(&[42.0, 40.0]), "(42,40) should be suppressed by (43,40) in wide window: {wide_peaks:?}");

        let (cx, cy) = center_wide.unwrap();
        let cr_wide = CoarseRect { cx, cy, r: 4, scores: scratch.scores.clone() };

        // Shrink window at same center (40, 40), r = 2 (x in 38..=42):
        // (43, 40) is now outside the rectangle, so boundary pixel (42, 40) BECOMES a local maximum!
        let mut lazy_called = false;
        let lazy_never = LazyTmpl::new(|| {
            lazy_called = true;
            make_tmpl(sample_int(&img, 10, 10, COARSE_R), COARSE_R, None, 0.0)
        });
        let ((shrunk_arr, shrunk_len), center_shrunk) = window_peaks_cached(&img, lazy_never, [40.0, 40.0], 2, Some(&cr_wide), &mut scratch);
        assert!(!lazy_called, "full cache hit on shrunk window must not construct lazy template");
        let shrunk_peaks = &shrunk_arr[..shrunk_len];
        let uncached_shrunk = window_peaks(&img, &tc, [40.0, 40.0], 2);
        assert!(shrunk_peaks.contains(&[42.0, 40.0]), "boundary pixel (42,40) must become a local maximum when window shrinks: {shrunk_peaks:?}");
        assert_eq!(shrunk_peaks, uncached_shrunk.as_slice(), "cached shrunk peaks must match uncached bit-for-bit");

        // Now shift/grow from cr_shrunk (x in 38..=42) to center (41, 40), r = 2 (x in 39..=43):
        // (43, 40) re-enters the rectangle, so (42, 40) CEASES to be a local maximum!
        let (scx, scy) = center_shrunk.unwrap();
        let cr_shrunk = CoarseRect { cx: scx, cy: scy, r: 2, scores: scratch.scores.clone() };
        let mut lazy_shifted_called = false;
        let lazy_shifted = LazyTmpl::new(|| {
            lazy_shifted_called = true;
            make_tmpl(sample_int(&img, 10, 10, COARSE_R), COARSE_R, None, 0.0)
        });
        let ((shifted_arr, shifted_len), _) = window_peaks_cached(&img, lazy_shifted, [41.0, 40.0], 2, Some(&cr_shrunk), &mut scratch);
        assert!(lazy_shifted_called, "missing column x=43 must trigger lazy template construction");
        let shifted_peaks = &shifted_arr[..shifted_len];
        let uncached_shifted = window_peaks(&img, &tc, [41.0, 40.0], 2);
        assert!(!shifted_peaks.contains(&[42.0, 40.0]), "(42,40) must cease to be a local maximum when (43,40) enters: {shifted_peaks:?}");
        assert!(shifted_peaks.contains(&[43.0, 40.0]), "shifted window must find (43,40): {shifted_peaks:?}");
        assert_eq!(shifted_peaks, uncached_shifted.as_slice(), "cached shifted peaks must match uncached bit-for-bit");
    }

    #[test]
    fn match_cache_reuses_fixed_view_failed_refinements_and_reverse_keys() {
        // Part 1: Failed (None) full refinement is cached and reused on pass 2 without re-evaluating.
        {
            let (w, h) = (32usize, 32usize);
            let mut full_px = vec![40u8; w * h];
            let mut ref_px = vec![40u8; w * h];
            // Bump at full=(6, 16) -> half=(2.75, 7.75), so coarse tc at half=(3, 8) gets NCC > 0.9.
            // tf sampled at ref=(13, 16) (bump at dx=+3 inside tf) walks refine_2d_pose left from
            // x=6.5 toward x=3.0 and hits full.inside(4.0, ..., 4, 0.5) == false -> returns None.
            for dy in -6i32..=6 {
                for dx in -6i32..=6 {
                    let r2 = (dx * dx + dy * dy) as f64;
                    let v = (40.0 + 200.0 * (-r2 / 10.0).exp()).round() as u8;
                    ref_px[(16 + dy) as usize * w + (16 + dx) as usize] = v;
                    let fx = 6 + dx;
                    if fx >= 0 {
                        full_px[(16 + dy) as usize * w + fx as usize] = v;
                    }
                }
            }
            let full = Gray::new(w, h, full_px).unwrap();
            let half = full.half();
            let ref_full = Gray::new(w, h, ref_px).unwrap();
            let ref_half = ref_full.half();
            let tc = make_tmpl(sample_bilinear(&ref_half, 7.75, 7.75, COARSE_R).unwrap(), COARSE_R, None, 0.0).unwrap();
            let tf = make_tmpl(sample_int(&ref_full, 13, 16, PATCH_R), PATCH_R, None, 0.0).unwrap();

            let mut pair = PairCache::default();
            let mut budget = CacheBudget { used_cells: 0, max_cells: 10_000, used_refines: 0, max_refines: 100 };
            let mut scratch = MatchScratch::default();

            let r1 = match_2d_cached(
                (&full, &half), &tc, &tf, [7.0, 17.0], 4.0,
                PeakCache { coarse: CoarseCache::ReadWrite(&mut pair.fwd_coarse), refined: Some(&mut pair.fwd_refined), budget: Some(&mut budget) },
                &mut scratch,
            );
            assert!(r1.is_none());
            assert!(!pair.fwd_refined.is_empty(), "coarse peaks near border must have been refined");
            assert!(pair.fwd_refined.iter().any(|&(_, v)| v.is_none()), "failed refinement (None) must be cached: {:?}", pair.fwd_refined);
            let refines_after_p1 = budget.used_refines;
            let cells_after_p1 = budget.used_cells;

            // Pass 2: shrink window so same peak is hit; refine_2d_pose must NOT run again for cached None peaks.
            let mut tf_sentinel = false;
            let lazy_tf = LazyTmpl::new(|| {
                tf_sentinel = true;
                make_tmpl(sample_int(&ref_full, 13, 16, PATCH_R), PATCH_R, None, 0.0)
            });
            let r2 = match_2d_cached(
                (&full, &half), &tc, lazy_tf, [7.0, 17.0], 2.0,
                PeakCache { coarse: CoarseCache::ReadWrite(&mut pair.fwd_coarse), refined: Some(&mut pair.fwd_refined), budget: Some(&mut budget) },
                &mut scratch,
            );
            assert!(r2.is_none());
            assert_eq!(budget.used_cells, cells_after_p1);
            assert_eq!(budget.used_refines, refines_after_p1);
            assert!(!tf_sentinel, "cached failed refinements must not reconstruct/re-evaluate tf");
        }

        // Part 2: Fixed-view whole-outcome reuse and reverse-key reuse / non-reuse.
        let look = [0.30, 0.0, 0.0];
        let mut cam0 = cam_at([0.30, -0.02, 0.45], look);
        let mut cam1 = cam_at([0.30, 0.02, 0.45], look);
        let mut cam2 = cam_at([0.28, 0.01, 0.45], look);
        for c in [&mut cam0, &mut cam1, &mut cam2] {
            c.cx = 160.0;
            c.cy = 90.0;
        }
        let views = vec![
            View { cam: cam0, img: render_table_view(&cam0) },
            View { cam: cam1, img: render_table_view(&cam1) },
            View { cam: cam2, img: render_table_view(&cam2) },
        ];
        let object_box = [140.0, 75.0, 180.0, 105.0];
        let exclude = [[0.0, 0.0, 20.0, 20.0]];
        let prep = TexturePrep::new(&views, &cam0, 0, object_box, &exclude, 0.0);
        assert!(!prep.feats.is_empty());

        // Fixed view (2-view prep): pass 2 with identical cam1 and win1 must do 0 new cells and 0 new refines.
        let prep2 = TexturePrep::new(&views[..2], &cam0, 0, object_box, &exclude, 0.0);
        assert!(!prep2.feats.is_empty());
        let mut fixed_cache = MatchCache::new(prep2.feats.len(), 2);
        let fixed_p1 = prep2.corrs_with_cache(&views[..2], &[cam0, cam1], &[15.0, 15.0], Some(&mut fixed_cache));
        assert!(!fixed_p1.is_empty());
        let used_cells_fixed = fixed_cache.budget.used_cells;
        let used_refines_fixed = fixed_cache.budget.used_refines;
        assert!(used_cells_fixed > 0 && used_refines_fixed > 0);
        let fixed_p2 = prep2.corrs_with_cache(&views[..2], &[cam0, cam1], &[15.0, 15.0], Some(&mut fixed_cache));
        assert_eq!(fixed_p2, fixed_p1);
        assert_eq!(fixed_cache.budget.used_cells, used_cells_fixed);
        assert_eq!(fixed_cache.budget.used_refines, used_refines_fixed);

        // Reverse exact hit when q0 shifts slightly (same win=15.0, last_key misses, fwd returns same q):
        let mut cam1_tiny = cam1;
        cam1_tiny.r = mat_mul(&rodrigues(&[1e-5, -1e-5, 0.0]), &cam1.r);
        let mut rev_exact_cache = MatchCache::new(prep2.feats.len(), 2);
        let _ = prep2.corrs_with_cache(&views[..2], &[cam0, cam1], &[15.0, 15.0], Some(&mut rev_exact_cache));
        // Flip q_bits on one cached RevCache and poison its outcome: mismatched q must NOT reuse rev.outcome.
        let mut flipped_feat = None;
        for f_idx in 0..prep2.feats.len() {
            let p = &mut rev_exact_cache.pairs[f_idx * 2 + 1];
            if let Some(r) = p.rev.as_mut().filter(|r| r.outcome.is_some()) {
                r.q_bits[0] ^= 1;
                r.outcome = None;
                flipped_feat = Some(f_idx);
                break;
            }
        }
        assert!(flipped_feat.is_some(), "expected at least one valid reverse cache entry");
        let rev_hit = prep2.corrs_with_cache(&views[..2], &[cam0, cam1_tiny], &[15.0, 15.0], Some(&mut rev_exact_cache));
        let rev_exp = prep2.corrs(&views[..2], &[cam0, cam1_tiny], &[15.0, 15.0]);
        assert_eq!(rev_hit, rev_exp);

        // Reverse shrunk win (15.0 -> 4.0) with same q (tiny cam1 perturbation so last_key misses):
        let mut rev_shrunk_cache = MatchCache::new(prep2.feats.len(), 2);
        let _ = prep2.corrs_with_cache(&views[..2], &[cam0, cam1], &[15.0, 15.0], Some(&mut rev_shrunk_cache));
        let rev_shrunk = prep2.corrs_with_cache(&views[..2], &[cam0, cam1_tiny], &[15.0, 4.0], Some(&mut rev_shrunk_cache));
        let rev_shrunk_exp = prep2.corrs(&views[..2], &[cam0, cam1_tiny], &[15.0, 4.0]);
        assert_eq!(rev_shrunk, rev_shrunk_exp);

        // Reverse non-reuse when q changes: perturb cam2 significantly enough to shift fwd_q on some features.
        let mut cam2_p = cam2;
        cam2_p.r = mat_mul(&rodrigues(&[0.0025, -0.002, 0.001]), &cam2.r);
        let mut cache3 = MatchCache::new(prep.feats.len(), 3);
        let c3_p1 = prep.corrs_with_cache(&views, &[cam0, cam1, cam2], &[15.0, 15.0, 15.0], Some(&mut cache3));
        let c3_p2 = prep.corrs_with_cache(&views, &[cam0, cam1, cam2_p], &[15.0, 15.0, 4.0], Some(&mut cache3));
        assert_eq!(c3_p1, texture_corrs(&views, &[cam0, cam1, cam2], 0, object_box, &exclude, 0.0, &[15.0, 15.0, 15.0]));
        assert_eq!(c3_p2, texture_corrs(&views, &[cam0, cam1, cam2_p], 0, object_box, &exclude, 0.0, &[15.0, 15.0, 4.0]));
    }

    #[test]
    fn match_cache_exhaustion_falls_back_to_exact_uncached_results() {
        let look = [0.30, 0.0, 0.0];
        let mut cam0 = cam_at([0.30, -0.02, 0.45], look);
        let mut cam1 = cam_at([0.30, 0.02, 0.45], look);
        let mut cam2 = cam_at([0.28, 0.01, 0.45], look);
        for c in [&mut cam0, &mut cam1, &mut cam2] {
            c.cx = 160.0;
            c.cy = 90.0;
        }
        let views = vec![
            View { cam: cam0, img: render_table_view(&cam0) },
            View { cam: cam1, img: render_table_view(&cam1) },
            View { cam: cam2, img: render_table_view(&cam2) },
        ];
        let object_box = [140.0, 75.0, 180.0, 105.0];
        let exclude = [[0.0, 0.0, 20.0, 20.0]];

        let mut cam2_p = cam2;
        cam2_p.r = mat_mul(&rodrigues(&[0.0008, -0.0005, 0.0004]), &cam2.r);
        let cams1 = [cam0, cam1, cam2];
        let win1 = [15.0, 15.0, 15.0];
        let cams2 = [cam0, cam1, cam2_p];
        let win2 = [15.0, 15.0, 4.0];

        let exp1 = texture_corrs(&views, &cams1, 0, object_box, &exclude, 0.0, &win1);
        let exp2 = texture_corrs(&views, &cams2, 0, object_box, &exclude, 0.0, &win2);
        assert!(!exp1.is_empty() && !exp2.is_empty());

        let prep = TexturePrep::new(&views, &cam0, 0, object_box, &exclude, 0.0);
        let n_feats = prep.feats.len();
        let configs = [
            ("default", MatchCache::new(n_feats, 3)),
            ("zero_pairs", MatchCache::with_limits(n_feats, 3, 0, MAX_CACHE_CELLS, MAX_CACHE_REFINES)),
            ("zero_cells", MatchCache::with_limits(n_feats, 3, MAX_CACHE_PAIRS, 0, MAX_CACHE_REFINES)),
            ("zero_refines", MatchCache::with_limits(n_feats, 3, MAX_CACHE_PAIRS, MAX_CACHE_CELLS, 0)),
            ("tiny_partial", MatchCache::with_limits(n_feats, 3, MAX_CACHE_PAIRS, 100, 2)),
        ];

        for (label, mut cache) in configs {
            let got1 = prep.corrs_with_cache(&views, &cams1, &win1, Some(&mut cache));
            let got2 = prep.corrs_with_cache(&views, &cams2, &win2, Some(&mut cache));
            assert_eq!(got1, exp1, "pass 1 mismatch for budget config '{label}'");
            assert_eq!(got2, exp2, "pass 2 mismatch for budget config '{label}'");
            assert!(cache.budget.used_cells <= cache.budget.max_cells);
            assert!(cache.budget.used_refines <= cache.budget.max_refines);
            if label == "zero_pairs" {
                assert!(cache.pairs.is_empty());
            }
        }
    }

    #[test]
    fn coarse_integral_window_matches_generic_ncc_scores_and_peaks() {
        let reference_window = |img: &Gray, t: &Tmpl, c: [f64; 2], r: i32| -> (Vec<f32>, Vec<[f64; 2]>) {
            let n = (2 * r + 1) as usize;
            let (cx, cy) = (c[0].round(), c[1].round());
            let mut scores = vec![f32::NEG_INFINITY; n * n];
            for j in 0..n {
                let y = cy + f64::from(j as i32 - r);
                for i in 0..n {
                    let x = cx + f64::from(i as i32 - r);
                    if let Some(v) = ncc(img, t, x, y, false) {
                        scores[j * n + i] = v;
                    }
                }
            }
            let ni = n as i32;
            let mut peaks = Vec::new();
            for j in 0..ni {
                for i in 0..ni {
                    let v = scores[j as usize * n + i as usize];
                    if v.is_nan() || f64::from(v) < POSE_COARSE_MIN {
                        continue;
                    }
                    let local = (-1..=1).all(|dj| {
                        (-1..=1).all(|di| {
                            let (a, b) = (i + di, j + dj);
                            (di == 0 && dj == 0)
                                || a < 0
                                || b < 0
                                || a >= ni
                                || b >= ni
                                || scores[b as usize * n + a as usize] <= v
                        })
                    });
                    if local {
                        peaks.push((v, [cx + f64::from(i - r), cy + f64::from(j - r)]));
                    }
                }
            }
            peaks.sort_by(|a, b| b.0.total_cmp(&a.0));
            let top: Vec<[f64; 2]> = peaks.into_iter().take(4).map(|p| p.1).collect();
            (scores, top)
        };

        let (w, h) = (48usize, 40usize);
        let mut px = vec![120u8; w * h];
        // Bright nearly-flat region (250..=255) in top-right quadrant
        for y in 0..20 {
            for x in 24..w {
                px[y * w + x] = 250 + (((x * 3 + y * 5) % 6) as u8);
            }
        }
        // Normal textured patterns near borders and interior
        for y in 0..h {
            for x in 0..24 {
                if x < 12 && y < 12 || ((10..22).contains(&x) && (15..28).contains(&y)) || y >= 32 {
                    let v = (((x * 73) ^ (y * 137)).wrapping_mul(179) % 210) as u8;
                    px[y * w + x] = 20 + v;
                }
            }
        }
        // Plant an exact copy of a 5x5 textured patch near (16, 20) and near edges
        let patch_src_x = 16usize;
        let patch_src_y = 20usize;
        let img = Gray::new(w, h, px).unwrap();

        let tc = make_tmpl(sample_int(&img, patch_src_x, patch_src_y, COARSE_R), COARSE_R, None, 0.0).unwrap();
        let tc_r3 = make_tmpl(sample_int(&img, patch_src_x, patch_src_y, 3), 3, None, 0.0).unwrap();
        let tc_weighted = make_tmpl(
            sample_int(&img, patch_src_x, patch_src_y, COARSE_R),
            COARSE_R,
            Some([1.0, 0.0]),
            1.0,
        )
        .unwrap();

        let mut scratch = MatchScratch::default();
        let check_case = |label: &str, img: &Gray, tmpl: &Tmpl, c: [f64; 2], r: i32, cached: Option<&CoarseRect>, scratch: &mut MatchScratch| {
            let ((top, len), _) = window_peaks_cached(img, tmpl, c, r, cached, scratch);
            let (ref_scores, ref_peaks) = reference_window(img, tmpl, c, r);
            assert_eq!(scratch.scores.len(), ref_scores.len(), "{label}: score len mismatch");
            for (idx, (&got, &exp)) in scratch.scores.iter().zip(&ref_scores).enumerate() {
                assert_eq!(
                    got.to_bits(),
                    exp.to_bits(),
                    "{label}: score[{idx}] mismatch: got={got:?} exp={exp:?}"
                );
            }
            assert_eq!(&top[..len], ref_peaks.as_slice(), "{label}: peaks mismatch");
        };

        // 1. Interior windows (flat, bright nearly-flat, textured)
        check_case("interior_textured", &img, &tc, [16.0, 20.0], 6, None, &mut scratch);
        check_case("interior_flat", &img, &tc, [36.0, 30.0], 4, None, &mut scratch);
        check_case("interior_bright_nearly_flat", &img, &tc, [35.0, 10.0], 5, None, &mut scratch);

        // 2. Straddling all 4 image edges and corners
        check_case("edge_left", &img, &tc, [1.0, 6.0], 5, None, &mut scratch);
        check_case("edge_right", &img, &tc, [(w - 2) as f64, 10.0], 5, None, &mut scratch);
        check_case("edge_top", &img, &tc, [6.0, 1.0], 5, None, &mut scratch);
        check_case("edge_bottom", &img, &tc, [10.0, (h - 2) as f64], 5, None, &mut scratch);
        check_case("corner_tl_tiny_oob", &img, &tc, [1.0, 1.0], 1, None, &mut scratch);

        // 3. Completely out of bounds and tiny image (no valid centers)
        check_case("oob_neg", &img, &tc, [-15.0, -15.0], 4, None, &mut scratch);
        check_case("oob_pos", &img, &tc, [w as f64 + 15.0, h as f64 + 15.0], 4, None, &mut scratch);
        let tiny_img = Gray::new(6, 6, vec![100u8; 36]).unwrap();
        check_case("tiny_img", &tiny_img, &tc, [3.0, 3.0], 3, None, &mut scratch);

        // 4. Partial cache hit + integral miss
        let (_, c_base) = window_peaks_cached(&img, &tc, [15.0, 18.0], 5, None, &mut scratch);
        let (bcx, bcy) = c_base.unwrap();
        let cr_partial = CoarseRect { cx: bcx, cy: bcy, r: 5, scores: scratch.scores.clone() };
        check_case("partial_cache_hit", &img, &tc, [18.0, 20.0], 5, Some(&cr_partial), &mut scratch);

        // 5. Full cache hit fast path must not invoke LazyTmpl
        let mut lazy_called = false;
        let lazy_tc = LazyTmpl::new(|| {
            lazy_called = true;
            make_tmpl(sample_int(&img, patch_src_x, patch_src_y, COARSE_R), COARSE_R, None, 0.0)
        });
        let ((top_hit, len_hit), _) = window_peaks_cached(&img, lazy_tc, [15.0, 18.0], 3, Some(&cr_partial), &mut scratch);
        assert!(!lazy_called, "full cache hit fast path must not evaluate LazyTmpl");
        let (ref_scores_hit, ref_peaks_hit) = reference_window(&img, &tc, [15.0, 18.0], 3);
        assert_eq!(scratch.scores.iter().map(|v| v.to_bits()).collect::<Vec<_>>(), ref_scores_hit.iter().map(|v| v.to_bits()).collect::<Vec<_>>());
        assert_eq!(&top_hit[..len_hit], ref_peaks_hit.as_slice());

        // 6. Valid bounds covered by cache on straddling window must skip integral tables
        let (_, c_edge) = window_peaks_cached(&img, &tc, [6.0, 6.0], 4, None, &mut scratch);
        let (ecx, ecy) = c_edge.unwrap();
        let cr_edge = CoarseRect { cx: ecx, cy: ecy, r: 4, scores: scratch.scores.clone() };
        scratch.int_s1.clear();
        check_case("valid_bounds_covered_by_cache", &img, &tc, [5.0, 5.0], 5, Some(&cr_edge), &mut scratch);
        assert!(scratch.int_s1.is_empty(), "integral tables should not be built when valid bounds are cache-covered");

        // 7. Non-COARSE_R radius and weighted template fallback
        check_case("unweighted_r3", &img, &tc_r3, [16.0, 20.0], 5, None, &mut scratch);
        check_case("weighted_fallback", &img, &tc_weighted, [16.0, 20.0], 5, None, &mut scratch);
        let (_, c_wbase) = window_peaks_cached(&img, &tc_weighted, [15.0, 18.0], 5, None, &mut scratch);
        let (wcx, wcy) = c_wbase.unwrap();
        let cr_weighted_partial = CoarseRect { cx: wcx, cy: wcy, r: 5, scores: scratch.scores.clone() };
        check_case("weighted_fallback_partial_cache", &img, &tc_weighted, [18.0, 20.0], 5, Some(&cr_weighted_partial), &mut scratch);
    }

    #[test]
    fn refine_views_records_scalar_kernel_counts_and_resets() {
        let look = [0.30, 0.0, 0.0];
        let mut cam0 = cam_at([0.30, -0.02, 0.45], look);
        let mut cam1 = cam_at([0.30, 0.02, 0.45], look);
        for c in [&mut cam0, &mut cam1] {
            c.cx = 160.0;
            c.cy = 90.0;
        }
        let views = vec![
            View { cam: cam0, img: render_table_view(&cam0) },
            View { cam: cam1, img: render_table_view(&cam1) },
        ];
        let object_box = [140.0, 75.0, 180.0, 105.0];
        let exclude = [[0.0, 0.0, 20.0, 20.0]];

        let active = crate::ncc_backend::NccBackend::active();
        reset_kernel_counts();
        let res = refine_views(&views, 0, object_box, &exclude, 0.0, &[], true, &PoseParams::default());
        assert!(res.n_corrs > 0);
        let diag = backend_diagnostics();
        assert_eq!(diag.backend, active.kind().as_str());
        #[cfg(any(feature = "xthead-ncc", feature = "mview-profile"))]
        match active {
            crate::ncc_backend::NccBackend::Scalar => {
                assert!(diag.counts.coarse_dots_scalar > 0);
                assert!(diag.counts.coarse_cells_scalar >= diag.counts.coarse_dots_scalar);
                assert_eq!(diag.counts.coarse_dots_xthead071, 0);
                assert_eq!(diag.counts.coarse_cells_xthead071, 0);
                assert!(diag.counts.half_moments_scalar > 0);
                assert_eq!(diag.counts.half_moments_xthead071, 0);
            }
            crate::ncc_backend::NccBackend::Xthead071 => {
                assert!(diag.counts.coarse_dots_xthead071 > 0);
                assert!(diag.counts.coarse_cells_xthead071 >= diag.counts.coarse_dots_xthead071);
                assert_eq!(diag.counts.coarse_dots_scalar, 0);
                assert_eq!(diag.counts.coarse_cells_scalar, 0);
                assert!(diag.counts.half_moments_xthead071 > 0);
                assert_eq!(diag.counts.half_moments_scalar, 0);
            }
        }
        #[cfg(not(any(feature = "xthead-ncc", feature = "mview-profile")))]
        {
            assert_eq!(diag.backend, "scalar");
            assert_eq!(diag.reason, "scalar (built without xthead-ncc)");
            assert_eq!(diag.counts, KernelCallCounts::default());
        }

        reset_kernel_counts();
        assert_eq!(kernel_counts(), KernelCallCounts::default());
    }

    #[cfg(feature = "mview-profile")]
    #[test]
    fn refine_views_populates_pose_profile_with_stage_counts_and_histograms() {
        let look = [0.30, 0.0, 0.0];
        let mut cam0 = cam_at([0.30, -0.02, 0.45], look);
        let mut cam1 = cam_at([0.30, 0.02, 0.45], look);
        let mut cam2 = cam_at([0.28, 0.01, 0.45], look);
        for c in [&mut cam0, &mut cam1, &mut cam2] {
            c.cx = 160.0;
            c.cy = 90.0;
        }
        let views = vec![
            View { cam: cam0, img: render_table_view(&cam0) },
            View { cam: cam1, img: render_table_view(&cam1) },
            View { cam: cam2, img: render_table_view(&cam2) },
        ];
        let object_box = [140.0, 75.0, 180.0, 105.0];
        let exclude = [[0.0, 0.0, 20.0, 20.0]];

        let res = refine_views(&views, 0, object_box, &exclude, 0.0, &[], true, &PoseParams::default());
        let prof = res.profile.expect("refine_views should populate pose profile");
        assert!(prof.total_ms > 0.0 && prof.prep_ms > 0.0);
        assert!(prof.pass1_match_ms > 0.0 && prof.pass2_match_ms >= 0.0);
        assert!(prof.pass1.cells_dotted > 0);
        assert!(prof.pass1.run_count > 0);
        assert!(prof.pass1.max_run_len >= 1);
        let hist_sum: u64 = prof.pass1.run_len_hist.values().sum();
        assert_eq!(hist_sum, prof.pass1.run_count);
        for k in ["1", "16", "17..32", "33..64", "65+"] {
            assert!(prof.pass1.run_len_hist.contains_key(k), "missing key {k}");
        }
        assert!(prof.pass1.half_grid_fwd.iter().sum::<u64>() > 0);
        assert!(prof.pass1.half_grid_rev.iter().sum::<u64>() > 0);
        assert!(prof.pass2.cells_cached > 0, "pass 2 should observe cached coarse cells");

        // Standalone refine_poses must leave profile = None.
        let standalone = refine_poses(&[cam0, cam1, cam2], 0, &[], 0.0, &PoseParams::default());
        assert!(standalone.profile.is_none());
    }

    #[test]
    fn coarse_window_integral_and_window_peaks_reject_truncated_gray_buffers() {
        let (w, h) = (48usize, 40usize);
        let px: Vec<u8> = (0..w * h).map(|k| ((k * 37 + 11) % 210 + 20) as u8).collect();
        let valid_img = Gray::new(w, h, px).unwrap();
        let tc = make_tmpl(sample_int(&valid_img, 16, 20, COARSE_R), COARSE_R, None, 0.0).unwrap();
        let tc_weighted = make_tmpl(
            sample_int(&valid_img, 16, 20, COARSE_R),
            COARSE_R,
            Some([1.0, 0.0]),
            1.0,
        )
        .unwrap();

        let trunc = Gray { w, h, px: vec![128u8; 200] };
        let mut scratch = MatchScratch::default();
        scratch.scores.resize(13 * 13, f32::NEG_INFINITY);

        reset_kernel_counts();
        fill_coarse_window_integral(
            &trunc,
            &tc,
            20,
            20,
            6,
            None,
            &mut scratch,
            #[cfg(feature = "mview-profile")]
            None,
        );
        assert!(scratch.scores.iter().all(|v| *v == f32::NEG_INFINITY));

        let ((peaks, len), _) = window_peaks_cached(&trunc, &tc, [20.0, 20.0], 6, None, &mut scratch);
        assert_eq!(len, 0);
        assert!(peaks.iter().all(|p| *p == [0.0, 0.0]));

        let ((wpeaks, wlen), _) = window_peaks_cached(&trunc, &tc_weighted, [20.0, 20.0], 6, None, &mut scratch);
        assert_eq!(wlen, 0);
        assert!(wpeaks.iter().all(|p| *p == [0.0, 0.0]));
        assert!(window_peaks(&trunc, &tc, [20.0, 20.0], 6).is_empty());
        assert_eq!(kernel_counts(), KernelCallCounts::default());
    }

    #[test]
    fn batched_coarse_runs_match_scalar_score_maps_sentinels_and_peaks_across_holes_flats_and_multi_vl() {
        use crate::ncc_backend::NccBackend;

        let (w, h) = (96usize, 64usize);
        let mut px = vec![120u8; w * h];
        // Textured background with deterministic pseudo-noise.
        for y in 0..h {
            for x in 0..w {
                let v = (((x * 97 + y * 53) ^ (x * y * 13 + 29)).wrapping_mul(181) % 210) as u8;
                px[y * w + x] = 20 + v;
            }
        }
        // Carve flat bands so contiguous runs split into singletons, medium runs, and 0.0 flat gaps.
        for y in 18..28 {
            for x in 28..35 {
                px[y * w + x] = 140;
            }
            for x in 46..54 {
                px[y * w + x] = 200;
            }
        }
        // Plant an exact copy of a 5x5 patch from (14, 14) at (60, 24) and a second copy at (22, 24).
        let mut src = [[0u8; 5]; 5];
        for dy in 0..5 {
            for dx in 0..5 {
                src[dy][dx] = px[(12 + dy) * w + (12 + dx)];
                px[(22 + dy) * w + (58 + dx)] = src[dy][dx];
                px[(22 + dy) * w + (20 + dx)] = src[dy][dx];
            }
        }
        let img = Gray::new(w, h, px).unwrap();
        let tc5 = make_tmpl(sample_int(&img, 14, 14, COARSE_R), COARSE_R, None, 0.0).unwrap();
        let tc3 = make_tmpl(sample_int(&img, 14, 14, 1), 1, None, 0.0).unwrap();

        let mut scalar_scratch = MatchScratch::default();
        let mut cand_scratch = MatchScratch::default();
        let active = NccBackend::active();

        // Build a partial cache hole that overlaps the left-middle of a multi-VL window.
        let (_, hole_center) = window_peaks_cached_with_backend(
            NccBackend::scalar(),
            &img,
            &tc5,
            [36.0, 24.0],
            6,
            None,
            &mut scalar_scratch,
        );
        let (hcx, hcy) = hole_center.unwrap();
        let cr_hole = CoarseRect { cx: hcx, cy: hcy, r: 6, scores: scalar_scratch.scores.clone() };

        // Cases: multi-VL run (r=28 => 57-wide row > 32/48), cached hole + flat cells, straddling edge sentinel.
        let cases: [(&str, &Tmpl, [f64; 2], i32, Option<&CoarseRect>); 4] = [
            ("multi_vl_with_flats_and_hole", &tc5, [42.0, 24.0], 28, Some(&cr_hole)),
            ("multi_vl_edge_sentinels", &tc5, [22.0, 4.0], 26, None),
            ("short_runs_near_hole_and_flat", &tc5, [27.0, 22.0], 4, Some(&cr_hole)),
            ("non5_odd_n3_multi_vl", &tc3, [44.0, 24.0], 24, None),
        ];

        for (label, tmpl, center, r, cached) in cases {
            let ((exp_peaks, exp_len), _) = window_peaks_cached_with_backend(
                NccBackend::scalar(),
                &img,
                tmpl,
                center,
                r,
                cached,
                &mut scalar_scratch,
            );
            let mut backends = vec![active];
            if cfg!(not(all(feature = "xthead-ncc", mview_has_xthead071))) {
                backends.push(NccBackend::Xthead071);
            }
            for backend in backends {
                reset_kernel_counts();
                let ((got_peaks, got_len), _) = window_peaks_cached_with_backend(
                    backend,
                    &img,
                    tmpl,
                    center,
                    r,
                    cached,
                    &mut cand_scratch,
                );
                assert_eq!(got_len, exp_len, "{label}: peak count mismatch");
                assert_eq!(&got_peaks[..got_len], &exp_peaks[..exp_len], "{label}: ordered peaks mismatch");
                assert_eq!(cand_scratch.scores.len(), scalar_scratch.scores.len());
                for (idx, (&g, &e)) in cand_scratch.scores.iter().zip(&scalar_scratch.scores).enumerate() {
                    assert_eq!(
                        g.to_bits(),
                        e.to_bits(),
                        "{label}: score[{idx}] mismatch (got={g:?}, exp={e:?})"
                    );
                }
                if label == "multi_vl_edge_sentinels" {
                    assert!(cand_scratch.scores.contains(&f32::NEG_INFINITY));
                    assert!(cand_scratch.scores.contains(&0.0));
                }
                if backend == active {
                    let counts = kernel_counts();
                    #[cfg(any(feature = "xthead-ncc", feature = "mview-profile"))]
                    match active {
                        NccBackend::Scalar => {
                            assert!(counts.coarse_dots_scalar > 0, "{label}: expected scalar calls");
                            assert!(
                                counts.coarse_cells_scalar >= counts.coarse_dots_scalar,
                                "{label}: expected cells >= calls ({counts:?})"
                            );
                            if label != "short_runs_near_hole_and_flat" {
                                assert!(
                                    counts.coarse_cells_scalar > counts.coarse_dots_scalar,
                                    "{label}: expected batched multi-cell runs ({counts:?})"
                                );
                            }
                        }
                        NccBackend::Xthead071 => {
                            assert!(counts.coarse_dots_xthead071 > 0, "{label}: expected xthead071 calls");
                            assert!(
                                counts.coarse_cells_xthead071 >= counts.coarse_dots_xthead071,
                                "{label}: expected cells >= calls ({counts:?})"
                            );
                            if label != "short_runs_near_hole_and_flat" {
                                assert!(
                                    counts.coarse_cells_xthead071 > counts.coarse_dots_xthead071,
                                    "{label}: expected batched multi-cell runs ({counts:?})"
                                );
                            }
                        }
                    }
                    #[cfg(not(any(feature = "xthead-ncc", feature = "mview-profile")))]
                    assert_eq!(counts, KernelCallCounts::default());
                }
            }
        }
    }
}
