//! Motion-stereo geometry of armd's action `measure_multiview` (src/measure_multiview.rs, re-exported
//! there as `armd::mview_stereo`; a crate of its own so that it builds at opt-level 3): object
//! points triangulated from grayscale images taken at known camera poses. Pure computation, no
//! I/O; the action runs it in `spawn_blocking`.
//!
//! - Camera rotations ([`pose`]): before the reconstruction the caller may refine the rotation of
//!   every view but the reference from points on the table ([`refine_views`]).
//! - Features (reference view): Shi-Tomasi corners (minimum eigenvalue of the 5x5 structure
//!   tensor, at least `corner_rel_floor` of the strongest) in the feature region, at most
//!   `max_object_features` in the object box and `max_table_features` outside it,
//!   `min_feature_dist_px` apart, taken over a grid of `corner_cell_px` cells (the strongest of
//!   every cell first, then the second, ...); plus edge
//!   points (gradient-magnitude maxima across the edge, at least `edge_min_grad` grey levels per
//!   pixel) in the box grown by [`EDGE_REGION_GROW`], for objects without texture: up to
//!   `max_edge_features` edges with a uniform side (a plain face: the outline of a plain object)
//!   and up to a third of it, less a fifth of the corners in the box, textured on both sides.
//! - Depth range: a feature in the box is swept between `table_z - DEPTH_BELOW_TABLE_M` and
//!   `table_z + max_height_m` (0.12 m), one outside it (table) up to `table_z + TABLE_SWEEP_ABOVE_M`.
//!   Features in `exclude_px` (the gripper jaws) are not used.
//! - Views per feature: every other view for a corner; for an edge point only the views whose
//!   epipolar line in the reference image crosses the edge at more than [`EDGE_MIN_ANGLE_DEG`]
//!   (a line along the edge does not locate it). Of those, the [`SWEEP_VIEWS`] spread farthest
//!   apart are swept and left-right checked.
//! - Sweep (coarse to fine): the ray is swept in inverse depth on the 2x downsampled images (5x5
//!   patches, one depth sample per 2 * `sweep_step_px` full-resolution pixels of image motion)
//!   scoring the mean zero-normalised cross-correlation (NCC); the best depth needs `sweep_min` and
//!   passes a ratio test against the best separate peak (`(1 - best) <= ratio * (1 - second)`). It
//!   is then refined at full resolution (9x9 patches, bilinear, parabola). Edge patches are
//!   gradient weighted: each pixel's weight is its gradient across the edge (plus 5 % of the
//!   largest), so the uniform faces on either side count little.
//! - Refinement per view at full resolution: a corner in 2D, an edge point along that view's
//!   epipolar direction only (the aperture problem). NCC at least `ncc_min`. The swept views pass a
//!   left-right check: their patch swept back (coarse) along their own ray must land within
//!   `lr_tol_px` of the feature.
//! - Triangulation: least-squares midpoint of the rays of all matched views; points with a
//!   reprojection error above `reproj_max_px` in any view are dropped. An edge point is
//!   `consistent` when at least two other views matched and the points triangulated from the
//!   reference with each of them agree within [`EDGE_CONSISTENCY_M`]: a
//!   mismatch, or a silhouette of a curved surface that moves with the view, fails it.
//! - Table plane ([`fit_table_plane`]): a plane fitted (RANSAC, then least squares) to the points
//!   outside the box near the calibrated table height; heights are measured from it and the
//!   object threshold becomes max(`object_min_h_m`, 3 x the MAD of the table residuals). Without
//!   a usable fit the plane z = `table_z_m` stays.
//! - Measurement ([`measure`]): points more than `object_min_h_m` above the table plane whose
//!   reference pixel lies in the object box are object points, less isolated ones (false matches,
//!   [`ISOLATION_K`]). The top is the median height of the highest cluster of corners and
//!   consistent edge points ([`TOP_CLUSTER_MIN`] within [`TOP_CLUSTER_DZ_M`] and
//!   [`TOP_CLUSTER_XY_M`]), else the p90 of their locally median-filtered heights; the footprint is
//!   the oriented extent (PCA of x, y, 2nd to 98th percentile) of all object points. Points within
//!   [-0.05, `object_min_h_m`] m of the plane are table points (`table_check`).
use std::time::Instant;

pub mod kinematics;
pub mod ncc_backend;
pub mod pose;

use kinematics::{Mat3, Vec3};
pub use ncc_backend::{
    backend_diagnostics, kernel_counts, reset_kernel_counts, BackendDiagnostics, KernelCallCounts,
    NccBackendKind,
};
pub use pose::{marker_corrs, refine_poses, refine_views, texture_corrs, Corr, Marker, PoseParams, PoseRefinement, ViewRefine};
#[cfg(feature = "mview-profile")]
pub use pose::{PosePassProfile, PoseProfile};

/// Patch radius at full resolution (9x9 patches).
pub const PATCH_R: usize = 4;
/// Patch radius on the 2x downsampled images (5x5 patches).
pub const COARSE_R: usize = 2;
/// A feature's sweep starts this far below the table plane (m).
pub const DEPTH_BELOW_TABLE_M: f64 = 0.02;
/// A feature outside the object box is swept at most this far above the table (m).
pub const TABLE_SWEEP_ABOVE_M: f64 = 0.01;
/// Views (besides the reference) the ray sweep and the left-right check use.
pub const SWEEP_VIEWS: usize = 2;
/// Neighbours (the point included) of the local median before the top percentile.
pub const TOP_NEIGHBOURS: usize = 9;
/// Table points: at most this far below the plane (m).
pub const TABLE_BAND_BELOW_M: f64 = 0.05;
/// An edge point is matched only in views whose epipolar line crosses the edge at more than this.
pub const EDGE_MIN_ANGLE_DEG: f64 = 30.0;
/// Pairwise triangulations of an edge point must agree within this (m) for the top height. (It
/// was max(4 mm, the depth of 3 px of disparity at the shortest pair baseline): at 0.45 m and
/// 4 cm that is 19 mm, so silhouette points of a curved surface passed.)
pub const EDGE_CONSISTENCY_M: f64 = 0.005;
/// Edge points are searched in the object box grown by this fraction of its size on each side.
pub const EDGE_REGION_GROW: f64 = 0.25;
/// Object points farther than max(3 x the median, ISOLATION_MIN_M) from their ISOLATION_K-th
/// nearest object point are dropped as false matches.
pub const ISOLATION_K: usize = 3;
pub const ISOLATION_MIN_M: f64 = 0.004;
/// A side of an edge is uniform (a plain face) below this standard deviation (grey levels, 7x7).
pub const UNIFORM_STD: f32 = 5.0;
/// Points the top height needs (corners and consistent edge points).
pub const MIN_TOP_POINTS: usize = 4;
/// Top cluster: the highest top point with at least [`TOP_CLUSTER_MIN`] top points (itself
/// included) within [`TOP_CLUSTER_DZ_M`] in height and [`TOP_CLUSTER_XY_M`] in x, y; the top is
/// their median height, with the height window re-centred on the median until it settles.
pub const TOP_CLUSTER_MIN: usize = 3;
pub const TOP_CLUSTER_DZ_M: f64 = 0.003;
pub const TOP_CLUSTER_XY_M: f64 = 0.015;

/// 8-bit grayscale image, row-major.
#[derive(Debug, Clone, PartialEq)]
pub struct Gray {
    pub w: usize,
    pub h: usize,
    pub px: Vec<u8>,
}

impl Gray {
    pub fn new(w: usize, h: usize, px: Vec<u8>) -> Option<Self> { (px.len() == w * h && w > 0 && h > 0).then_some(Self { w, h, px }) }

    #[inline]
    fn at(&self, x: usize, y: usize) -> f32 { f32::from(self.px[y * self.w + x]) }

    /// A patch of radius `r` centred at `(x, y)` fits with a margin of `m` pixels.
    #[inline]
    fn inside(&self, x: f64, y: f64, r: usize, m: f64) -> bool {
        let r = r as f64 + m;
        x >= r && y >= r && x <= self.w as f64 - 1.0 - r && y <= self.h as f64 - 1.0 - r
    }

    /// 2x2 box-filtered half-resolution image: its pixel i covers full pixels 2i and 2i + 1, so a
    /// full-resolution coordinate u is (u - 0.5) / 2 there.
    pub fn half(&self) -> Gray {
        let (w, h) = (self.w / 2, self.h / 2);
        let mut px = Vec::with_capacity(w * h);
        for y in 0..h {
            let (r0, r1) = (&self.px[2 * y * self.w..], &self.px[(2 * y + 1) * self.w..]);
            for x in 0..w {
                let s = u16::from(r0[2 * x]) + u16::from(r0[2 * x + 1]) + u16::from(r1[2 * x]) + u16::from(r1[2 * x + 1]);
                px.push(((s + 2) / 4) as u8);
            }
        }
        Gray { w, h, px }
    }
}

#[inline]
fn to_half(q: [f64; 2]) -> [f64; 2] { [(q[0] - 0.5) / 2.0, (q[1] - 0.5) / 2.0] }

/// Pinhole camera in the base frame: `r` = R_base_cam (optical z forward, x right, y down), `c` =
/// optical centre (m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    pub r: Mat3,
    pub c: Vec3,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

impl Camera {
    /// Pixel of a base-frame point; None closer than 1 cm in front of the camera.
    pub fn project(&self, p: &Vec3) -> Option<[f64; 2]> {
        let d = kinematics::sub(p, &self.c);
        let r = &self.r;
        let z = r[0][2] * d[0] + r[1][2] * d[1] + r[2][2] * d[2];
        if z <= 0.01 { return None; }
        let x = r[0][0] * d[0] + r[1][0] * d[1] + r[2][0] * d[2];
        let y = r[0][1] * d[0] + r[1][1] * d[1] + r[2][1] * d[2];
        Some([self.fx * x / z + self.cx, self.fy * y / z + self.cy])
    }

    /// Base-frame ray through `px` per unit optical depth.
    pub fn ray(&self, px: [f64; 2]) -> Vec3 { kinematics::mat_vec(&self.r, &[(px[0] - self.cx) / self.fx, (px[1] - self.cy) / self.fy, 1.0]) }

    #[inline]
    fn at_inv_depth(&self, d: &Vec3, w: f64) -> Vec3 { [self.c[0] + d[0] / w, self.c[1] + d[1] / w, self.c[2] + d[2] / w] }
}

#[derive(Debug, Clone)]
pub struct View {
    pub cam: Camera,
    pub img: Gray,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Params {
    pub table_z_m: f64,
    /// Upper bound of the sweep above the table for features in the box (m).
    pub max_height_m: f64,
    /// Image region without features (px [x0, y0, x1, y1]; the gripper jaws, which move with the
    /// camera).
    pub exclude_px: Option<[f64; 4]>,
    pub max_object_features: usize,
    pub max_table_features: usize,
    pub max_edge_features: usize,
    /// Minimum distance between corners (px).
    pub min_feature_dist_px: f64,
    /// Minimum distance between edge points (px), and from an edge point to a corner.
    pub min_edge_dist_px: f64,
    /// Gradient (grey levels per pixel, central difference) an edge point needs.
    pub edge_min_grad: f32,
    /// Mean coarse NCC the best depth of a ray sweep needs.
    pub sweep_min: f64,
    /// NCC each view's full-resolution match needs.
    pub ncc_min: f64,
    pub ratio: f64,
    pub lr_tol_px: f64,
    pub reproj_max_px: f64,
    /// Full-resolution sweep step (px of image motion per depth sample; the coarse sweep uses twice it).
    pub sweep_step_px: f64,
    /// Height above the table that makes a point an object point (m).
    pub object_min_h_m: f64,
    pub min_object_points: usize,
    /// Table plane [n_x, n_y, n_z, d] (unit normal up, height = n . x + d); None: z = `table_z_m`.
    pub plane: Option<[f64; 4]>,
    /// Corners need a minimum eigenvalue of at least this fraction of the strongest (and 100).
    pub corner_rel_floor: f32,
    /// Corner selection over a grid of cells this size (px): the strongest corner of every cell
    /// first, then the second of every cell, ...; 0 = strongest first over the whole region.
    pub corner_cell_px: usize,
}

impl Params {
    pub fn new(table_z_m: f64) -> Self {
        Self {
            table_z_m, max_height_m: 0.12, exclude_px: None, max_object_features: 200, max_table_features: 60, max_edge_features: 120,
            min_feature_dist_px: 5.0, min_edge_dist_px: 4.0, edge_min_grad: 6.0,
            sweep_min: 0.6, ncc_min: 0.8, ratio: 0.7, lr_tol_px: 2.0, reproj_max_px: 2.0, sweep_step_px: 0.75, object_min_h_m: 0.005,
            min_object_points: 8, plane: None, corner_rel_floor: 0.002, corner_cell_px: 16,
        }
    }

    /// Height of `x` above the table plane (m).
    #[inline]
    pub fn height(&self, x: &Vec3) -> f64 {
        match self.plane { Some(n) => n[0] * x[0] + n[1] * x[1] + n[2] * x[2] + n[3], None => x[2] - self.table_z_m }
    }

    /// z of the table plane at (x, y).
    pub fn table_z_at(&self, x: f64, y: f64) -> f64 {
        match self.plane { Some(n) if n[2].abs() > 1e-6 => -(n[0] * x + n[1] * y + n[3]) / n[2], _ => self.table_z_m }
    }
}

/// Table plane fit: points outside the box within this of the calibrated table height (m).
pub const PLANE_BAND_M: f64 = 0.015;
/// RANSAC inlier distance (m), iterations, points the fit needs, largest tilt (deg) and largest
/// offset from the calibrated table height at the box (m).
pub const PLANE_INLIER_M: f64 = 0.002;
pub const PLANE_ITERS: usize = 300;
pub const PLANE_MIN_POINTS: usize = 10;
pub const PLANE_MAX_TILT_DEG: f64 = 5.0;
pub const PLANE_MAX_OFFSET_M: f64 = 0.01;
/// Object threshold: this many times the MAD of the table residuals (at least `object_min_h_m`).
pub const PLANE_MAD_K: f64 = 3.0;

/// A fitted table plane.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct TablePlane {
    /// [n_x, n_y, n_z, d]: unit normal (up) and offset, height = n . x + d.
    pub plane: [f64; 4],
    /// Candidate points (outside the box, within [`PLANE_BAND_M`]) and RANSAC inliers.
    pub n_points: usize,
    pub n_inliers: usize,
    /// Plane height above `table_z_m` at the box centre (mm) and tilt (deg).
    pub offset_mm: f64,
    pub tilt_deg: f64,
    /// Median absolute deviation of the candidates' residuals (mm).
    pub mad_mm: f64,
    /// The object threshold max(`object_min_h_m`, [`PLANE_MAD_K`] x MAD) (mm).
    pub threshold_mm: f64,
}

/// Least-squares plane z = a x + b y + c through `pts`.
fn lsq_plane(pts: &[Vec3]) -> Option<[f64; 3]> {
    let n = pts.len() as f64;
    let (mx, my, mz) = (pts.iter().map(|p| p[0]).sum::<f64>() / n, pts.iter().map(|p| p[1]).sum::<f64>() / n, pts.iter().map(|p| p[2]).sum::<f64>() / n);
    let (mut sxx, mut sxy, mut syy, mut sxz, mut syz) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for p in pts {
        let (dx, dy, dz) = (p[0] - mx, p[1] - my, p[2] - mz);
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy; sxz += dx * dz; syz += dy * dz;
    }
    let det = sxx * syy - sxy * sxy;
    if det.abs() < 1e-12 { return None; }
    let a = (sxz * syy - syz * sxy) / det;
    let b = (syz * sxx - sxz * sxy) / det;
    Some([a, b, mz - a * mx - b * my])
}

/// Fit the table plane to the points outside the box within [`PLANE_BAND_M`] of `table_z_m`:
/// RANSAC (deterministic, [`PLANE_ITERS`] triples, inliers within [`PLANE_INLIER_M`], normals
/// within 20 deg of vertical), then least squares on the inliers. None with fewer than
/// [`PLANE_MIN_POINTS`] candidates or inliers, or a plane tilted more than
/// [`PLANE_MAX_TILT_DEG`] or more than [`PLANE_MAX_OFFSET_M`] off `table_z_m` at `centre_xy`.
pub fn fit_table_plane(points: &[Point], p: &Params, centre_xy: [f64; 2]) -> Option<TablePlane> {
    let cand: Vec<Vec3> = points.iter().filter(|q| !q.in_box && (q.x[2] - p.table_z_m).abs() <= PLANE_BAND_M).map(|q| q.x).collect();
    if cand.len() < PLANE_MIN_POINTS { return None; }
    let mut seed: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut next = |n: usize| { seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17; (seed % n as u64) as usize };
    let mut best: Vec<bool> = Vec::new();
    let mut best_n = 0;
    for _ in 0..PLANE_ITERS {
        let (i, j, k) = (next(cand.len()), next(cand.len()), next(cand.len()));
        if i == j || j == k || i == k { continue; }
        let nrm = kinematics::cross(&kinematics::sub(&cand[j], &cand[i]), &kinematics::sub(&cand[k], &cand[i]));
        let l = kinematics::norm(&nrm);
        if l < 1e-9 { continue; }
        let nrm = [nrm[0] / l, nrm[1] / l, nrm[2] / l];
        if nrm[2].abs() < 0.94 { continue; }
        let inl: Vec<bool> = cand.iter().map(|x| (kinematics::dot(&nrm, &kinematics::sub(x, &cand[i]))).abs() <= PLANE_INLIER_M).collect();
        let n = inl.iter().filter(|b| **b).count();
        if n > best_n { best_n = n; best = inl; }
    }
    if best_n < PLANE_MIN_POINTS { return None; }
    let inl: Vec<Vec3> = cand.iter().zip(&best).filter(|(_, b)| **b).map(|(x, _)| *x).collect();
    let [a, b, c] = lsq_plane(&inl)?;
    let l = (a * a + b * b + 1.0).sqrt();
    let plane = [-a / l, -b / l, 1.0 / l, -c / l];
    let tilt_deg = (1.0 / l).acos().to_degrees();
    let offset = a * centre_xy[0] + b * centre_xy[1] + c - p.table_z_m;
    if tilt_deg > PLANE_MAX_TILT_DEG || offset.abs() > PLANE_MAX_OFFSET_M { return None; }
    let mut r: Vec<f64> = cand.iter().map(|x| plane[0] * x[0] + plane[1] * x[1] + plane[2] * x[2] + plane[3]).collect();
    r.sort_by(f64::total_cmp);
    let med = quantile(&r, 0.5);
    let mut dev: Vec<f64> = r.iter().map(|x| (x - med).abs()).collect();
    dev.sort_by(f64::total_cmp);
    let mad = quantile(&dev, 0.5);
    Some(TablePlane {
        plane, n_points: cand.len(), n_inliers: inl.len(), offset_mm: offset * 1000.0, tilt_deg, mad_mm: mad * 1000.0,
        threshold_mm: p.object_min_h_m.max(PLANE_MAD_K * mad) * 1000.0,
    })
}

/// One triangulated point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: Vec3,
    /// Feature pixel in the reference view.
    pub ref_px: [f64; 2],
    pub reproj_rms_px: f64,
    /// Views whose rays were triangulated (reference included).
    pub views: usize,
    /// Inside the object box of the reference view.
    pub in_box: bool,
    /// An edge point (else a corner).
    pub edge: bool,
    /// Usable for the top height: every corner; an edge point that passed the consistency check.
    pub consistent: bool,
}

/// Counts and stage times (ms) of a reconstruction (diagnostics).
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize)]
pub struct Stats {
    pub features: usize,
    pub corners: usize,
    pub edges: usize,
    /// Edge points without a view whose epipolar line crosses the edge.
    pub edge_no_view: usize,
    pub no_peak: usize,
    pub ambiguous: usize,
    /// A view's full-resolution match below `ncc_min` (or out of the image).
    pub refine_rejected: usize,
    pub lr_rejected: usize,
    pub reproj_rejected: usize,
    pub points: usize,
    pub edge_points: usize,
    /// Edge points that passed the consistency check.
    pub edge_consistent: usize,
    /// Half-resolution images.
    pub pyramid_ms: f64,
    pub features_ms: f64,
    /// Coarse and fine ray sweeps, left-right back sweeps included.
    pub sweep_ms: f64,
    /// Per-view full-resolution matches.
    pub refine_ms: f64,
    pub triangulate_ms: f64,
    pub total_ms: f64,
}

/// NCC call counts collected during reconstruction when `mview-profile` is enabled.
#[cfg(feature = "mview-profile")]
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct RecProfile {
    pub ncc_calls: std::collections::BTreeMap<String, u64>,
    pub total_ncc_calls: u64,
}

#[derive(Debug, Clone, Default)]
pub struct Reconstruction {
    pub points: Vec<Point>,
    pub stats: Stats,
    /// Every feature and how it ended (`point`, `no_peak`, `flat`, `ambiguous`, `edge_no_view`,
    /// `refine`, `left_right`, `reprojection`), for diagnostics.
    pub outcomes: Vec<(Feature, &'static str)>,
    #[cfg(feature = "mview-profile")]
    pub profile: Option<RecProfile>,
}

#[cfg(feature = "mview-profile")]
mod profile_tls {
    use super::RecProfile;
    use std::cell::{Cell, RefCell};
    use std::collections::BTreeMap;

    thread_local! {
        static HALF_GRID_COUNTS: Cell<([u64; 4], u64)> = const { Cell::new(([0; 4], 0)) };
        static REC_NCC_ACTIVE: RefCell<Option<Vec<((bool, bool, usize), u64)>>> = const { RefCell::new(None) };
    }

    #[inline]
    pub(crate) fn record_half_grid_phase(phase: usize) {
        HALF_GRID_COUNTS.with(|c| {
            let (mut phases, fb) = c.get();
            phases[phase] += 1;
            c.set((phases, fb));
        });
    }

    #[inline]
    pub(crate) fn record_half_grid_fallback() {
        HALF_GRID_COUNTS.with(|c| {
            let (phases, fb) = c.get();
            c.set((phases, fb + 1));
        });
    }

    #[inline]
    pub(crate) fn half_grid_counts() -> ([u64; 4], u64) {
        HALF_GRID_COUNTS.with(Cell::get)
    }

    #[inline]
    pub(crate) fn record_rec_ncc(bilinear: bool, weighted: bool, patch_n: usize) {
        REC_NCC_ACTIVE.with(|cell| {
            let mut borrow = cell.borrow_mut();
            let Some(entries) = borrow.as_mut() else {
                return;
            };
            let key = (bilinear, weighted, patch_n);
            if let Some((_, count)) = entries.iter_mut().find(|(k, _)| *k == key) {
                *count += 1;
            } else {
                entries.push((key, 1));
            }
        });
    }

    pub(crate) struct RecProfileGuard {
        prev: Option<Vec<((bool, bool, usize), u64)>>,
    }

    impl RecProfileGuard {
        pub(crate) fn enter() -> Self {
            let prev = REC_NCC_ACTIVE.with(|cell| cell.borrow_mut().replace(Vec::new()));
            Self { prev }
        }

        pub(crate) fn finish(mut self) -> RecProfile {
            let entries = REC_NCC_ACTIVE.with(|cell| {
                let cur = cell.borrow_mut().take();
                *cell.borrow_mut() = self.prev.take();
                cur.unwrap_or_default()
            });
            let mut ncc_calls = BTreeMap::new();
            let mut total_ncc_calls = 0u64;
            for ((bilinear, weighted, patch_n), count) in entries {
                let interp = if bilinear { "bilinear" } else { "nearest" };
                let wt = if weighted { "weighted" } else { "unweighted" };
                ncc_calls.insert(format!("{interp}_{wt}_n{patch_n}"), count);
                total_ncc_calls += count;
            }
            RecProfile {
                ncc_calls,
                total_ncc_calls,
            }
        }
    }

    impl Drop for RecProfileGuard {
        fn drop(&mut self) {
            if let Some(prev) = self.prev.take() {
                REC_NCC_ACTIVE.with(|cell| {
                    *cell.borrow_mut() = Some(prev);
                });
            } else {
                REC_NCC_ACTIVE.with(|cell| {
                    let _ = cell.borrow_mut().take();
                });
            }
        }
    }
}

#[cfg(feature = "mview-profile")]
pub(crate) use profile_tls::half_grid_counts as half_grid_profile_counts;

// ---- patches ----

/// Patch template: zero-mean, unit-norm weighted values `a` (= w (v - mean) / norm), weights `w`
/// (None = uniform) and their sum.
struct Tmpl {
    r: usize,
    a: Vec<f32>,
    w: Option<Vec<f32>>,
    sw: f32,
}

/// The 2x2 neighbourhood of a patch centred at (x, y): index of the top-left pixel of its first
/// row and the four bilinear weights, shared by every pixel of the patch (same sub-pixel phase).
#[inline]
fn patch_phase(img: &Gray, x: f64, y: f64, r: usize) -> (usize, [f32; 4]) {
    let (x0, y0) = (x.floor(), y.floor());
    let (fx, fy) = ((x - x0) as f32, (y - y0) as f32);
    let base = (y0 as usize - r) * img.w + x0 as usize - r;
    (base, [(1.0 - fx) * (1.0 - fy), fx * (1.0 - fy), (1.0 - fx) * fy, fx * fy])
}

fn sample_bilinear(img: &Gray, x: f64, y: f64, r: usize) -> Option<Vec<f32>> {
    if !img.inside(x, y, r, 1.0) { return None; }
    let n = 2 * r + 1;
    let (base, [w00, w10, w01, w11]) = patch_phase(img, x, y, r);
    let mut v = Vec::with_capacity(n * n);
    for j in 0..n {
        let (r0, r1) = (&img.px[base + j * img.w..][..n + 1], &img.px[base + (j + 1) * img.w..][..n + 1]);
        for i in 0..n {
            v.push(w00 * f32::from(r0[i]) + w10 * f32::from(r0[i + 1]) + w01 * f32::from(r1[i]) + w11 * f32::from(r1[i + 1]));
        }
    }
    Some(v)
}

fn sample_int(img: &Gray, x: usize, y: usize, r: usize) -> Vec<f32> {
    let mut v = Vec::with_capacity((2 * r + 1) * (2 * r + 1));
    for yy in y - r..=y + r { for xx in x - r..=x + r { v.push(img.at(xx, yy)); } }
    v
}

/// Template of patch values `v` (radius `r`); with `normal` (unit, image) the weights are each
/// pixel's gradient along it, plus 5 % of the largest; with `side` +1 / -1 (the uniform side of an
/// outline along +/- the normal) the pixels on the other side count 10 %. None when (weighted) flat.
fn make_tmpl(v: Vec<f32>, r: usize, normal: Option<[f64; 2]>, side: f32) -> Option<Tmpl> {
    let n = 2 * r + 1;
    let w: Option<Vec<f32>> = normal.map(|nn| {
        let (n0, n1) = (nn[0] as f32, nn[1] as f32);
        let g: Vec<f32> = (0..n * n).map(|k| {
            let (i, j) = (k % n, k / n);
            let (il, ir) = (i.saturating_sub(1), (i + 1).min(n - 1));
            let (ju, jd) = (j.saturating_sub(1), (j + 1).min(n - 1));
            let gx = (v[j * n + ir] - v[j * n + il]) / (ir - il) as f32;
            let gy = (v[jd * n + i] - v[ju * n + i]) / (jd - ju) as f32;
            (gx * n0 + gy * n1).abs()
        }).collect();
        let gmax = g.iter().copied().fold(0.0f32, f32::max).max(1e-6);
        // one-sided (an outline): the pixels beyond the edge on the textured side count 10 %
        (0..n * n).map(|k| {
            let (i, j) = ((k % n) as f32 - r as f32, (k / n) as f32 - r as f32);
            let along = side * (i * n0 + j * n1);
            (g[k] / gmax + 0.05) * if along < -0.5 { 0.0 } else { 1.0 }
        }).collect()
    });
    let sw: f32 = w.as_ref().map_or((n * n) as f32, |w| w.iter().sum());
    let mean = match &w { Some(w) => v.iter().zip(w).map(|(a, b)| a * b).sum::<f32>() / sw, None => v.iter().sum::<f32>() / sw };
    let mut a: Vec<f32> = v.iter().map(|x| x - mean).collect();
    let ss: f32 = match &w { Some(w) => a.iter().zip(w).map(|(d, b)| b * d * d).sum(), None => a.iter().map(|d| d * d).sum() };
    // a weighted standard deviation below 2 grey levels carries no structure
    if ss < 4.0 * sw { return None; }
    let inv = 1.0 / ss.sqrt();
    match &w {
        Some(w) => for (x, b) in a.iter_mut().zip(w) { *x *= b * inv; },
        None => for x in a.iter_mut() { *x *= inv; },
    }
    Some(Tmpl { r, a, w, sw })
}

/// NCC of `t` with the patch of `img` at `(x, y)`: the nearest integer pixel, or bilinear.
#[inline]
fn ncc(img: &Gray, t: &Tmpl, x: f64, y: f64, bilinear: bool) -> Option<f32> {
    let r = t.r;
    if !img.inside(x, y, r, if bilinear { 1.0 } else { 0.5 }) { return None; }
    let n = 2 * r + 1;
    let (x0, y0, bilerp) = if bilinear {
        let (fx0, fy0) = (x.floor(), y.floor());
        let (fx, fy) = ((x - fx0) as f32, (y - fy0) as f32);
        (
            (fx0 as usize).checked_sub(r)?,
            (fy0 as usize).checked_sub(r)?,
            Some([(1.0 - fx) * (1.0 - fy), fx * (1.0 - fy), (1.0 - fx) * fy, fx * fy]),
        )
    } else {
        let (cx, cy) = (x.round() as usize, y.round() as usize);
        (cx.checked_sub(r)?, cy.checked_sub(r)?, None)
    };
    let (s1, s2, dot) = ncc_backend::NccBackend::active()
        .patch_moments(img, x0, y0, &t.a, t.w.as_deref(), n, bilerp)
        .ok()?;
    #[cfg(feature = "mview-profile")]
    profile_tls::record_rec_ncc(bilinear, t.w.is_some(), n);
    // s1, s2 in f64: in f32 `s2 - s1*s1/sw` cancels (bright patches: s2 ~ 1e6 x n^2) and was off by up
    // to 0.4 NCC on real reCamera frames.
    let sw = f64::from(t.sw);
    let var = s2 - s1 * s1 / sw;
    Some(if var > 1e-3 * sw { dot / var.sqrt() as f32 } else { 0.0 })
}

const HALF_GRID_MAX_R: usize = ncc_backend::HALF_GRID_MAX_R;

/// Unweighted half-pixel-grid specialization of bilinear `ncc`, falling back to generic `ncc`
/// for weighted templates, unsupported radii (`r > HALF_GRID_MAX_R`), or arbitrary phases.
#[inline]
fn ncc_half_grid(img: &Gray, t: &Tmpl, x: f64, y: f64) -> Option<f32> {
    ncc_half_grid_with_backend(ncc_backend::NccBackend::active(), img, t, x, y)
}

#[inline]
fn ncc_half_grid_with_backend(
    backend: ncc_backend::NccBackend,
    img: &Gray,
    t: &Tmpl,
    x: f64,
    y: f64,
) -> Option<f32> {
    let r = t.r;
    if t.w.is_some() || r > HALF_GRID_MAX_R {
        #[cfg(feature = "mview-profile")]
        profile_tls::record_half_grid_fallback();
        return ncc(img, t, x, y, true);
    }
    if !img.inside(x, y, r, 1.0) {
        return None;
    }
    let (fx0, fy0) = (x.floor(), y.floor());
    let (fx, fy) = ((x - fx0) as f32, (y - fy0) as f32);
    let phase = match (fx, fy) {
        (0.0, 0.0) => 0u8,
        (0.5, 0.0) => 1u8,
        (0.0, 0.5) => 2u8,
        (0.5, 0.5) => 3u8,
        _ => {
            #[cfg(feature = "mview-profile")]
            profile_tls::record_half_grid_fallback();
            return ncc(img, t, x, y, true);
        }
    };
    let x0 = (fx0 as usize).checked_sub(r)?;
    let y0 = (fy0 as usize).checked_sub(r)?;
    let n = 2 * r + 1;
    let (is1, is2, dot) = backend
        .half_moments(img, x0, y0, &t.a, n, phase)
        .ok()?;
    #[cfg(feature = "mview-profile")]
    profile_tls::record_half_grid_phase(usize::from(phase));
    let (s1_scale, s2_scale) = match phase {
        0 => (1.0f64, 1.0f64),
        1 | 2 => (0.5f64, 0.25f64),
        _ => (0.25f64, 0.0625f64),
    };
    let s1 = f64::from(is1) * s1_scale;
    let s2 = f64::from(is2) * s2_scale;
    let sw = f64::from(t.sw);
    let var = s2 - s1 * s1 / sw;
    Some(if var > 1e-3 * sw { dot / var.sqrt() as f32 } else { 0.0 })
}

// ---- features ----

/// A feature of the reference view: integer pixel, edge normal (unit gradient direction) for an
/// edge point, strength.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feature {
    pub x: usize,
    pub y: usize,
    pub normal: Option<[f64; 2]>,
    /// Edge point on an outline: +1 / -1 when the side along +/- the normal is uniform, else 0.
    pub side: f32,
    pub strength: f32,
}

#[inline]
fn in_rect(x: f64, y: f64, r: &[f64; 4]) -> bool { x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3] }

#[inline(always)]
fn stream_corner_row<const SUB_OLD: bool, const EMIT_LAM: bool>(
    gx_slice: &[f32],
    gy_slice: &[f32],
    ring_row: &mut [[f32; 3]],
    vacc: &mut [[f32; 3]],
    lam_row: &mut [f32],
    max_l: &mut f32,
) {
    let rw = vacc.len();
    assert!(gx_slice.len() >= rw + 4 && gy_slice.len() >= rw + 4 && ring_row.len() == rw);
    if EMIT_LAM {
        assert!(lam_row.len() == rw);
    }
    let mut hacc = [0.0f32; 3];
    for i in 0..4 {
        let (a, b) = (gx_slice[i], gy_slice[i]);
        hacc[0] += a * a;
        hacc[1] += b * b;
        hacc[2] += a * b;
    }
    for col in 0..rw {
        let (a, b) = (gx_slice[col + 4], gy_slice[col + 4]);
        hacc[0] += a * a;
        hacc[1] += b * b;
        hacc[2] += a * b;
        if col >= 1 {
            let (a, b) = (gx_slice[col - 1], gy_slice[col - 1]);
            hacc[0] -= a * a;
            hacc[1] -= b * b;
            hacc[2] -= a * b;
        }
        let va = &mut vacc[col];
        for c in 0..3 {
            va[c] += hacc[c];
        }
        if SUB_OLD {
            let old = ring_row[col];
            for c in 0..3 {
                va[c] -= old[c];
            }
        }
        ring_row[col] = hacc;
        if EMIT_LAM {
            let [a, c, b] = *va;
            let half = 0.5 * (a + c);
            let l = half - (0.25 * (a - c) * (a - c) + b * b).max(0.0).sqrt();
            lam_row[col] = l;
            *max_l = (*max_l).max(l);
        }
    }
}

/// Corners in `region` (quotas inside / outside `inner`) and edge points in `edge_region` (see the
/// module doc).
pub fn features(img: &Gray, region: [f64; 4], inner: [f64; 4], edge_region: [f64; 4], p: &Params) -> Vec<Feature> {
    let m = PATCH_R + 4;
    let x0 = (region[0].floor().max(0.0) as usize).max(m);
    let y0 = (region[1].floor().max(0.0) as usize).max(m);
    let x1 = (region[2].ceil().max(0.0) as usize).min(img.w.saturating_sub(m + 1));
    let y1 = (region[3].ceil().max(0.0) as usize).min(img.h.saturating_sub(m + 1));
    if x1 <= x0 + 2 || y1 <= y0 + 2 { return Vec::new(); }
    // gradients over the region plus a 2 px border (structure tensor window 5x5)
    let (gx0, gy0) = (x0 - 2, y0 - 2);
    let (gw, gh) = (x1 - x0 + 5, y1 - y0 + 5);
    let (rw, rh) = (x1 - x0 + 1, y1 - y0 + 1);
    let need_edges = p.max_edge_features > 0;
    let (mut gxv, mut gyv) = if need_edges {
        (vec![0.0f32; gw * gh], vec![0.0f32; gw * gh])
    } else {
        (Vec::new(), Vec::new())
    };
    let (mut gx_row, mut gy_row) = if need_edges {
        (Vec::new(), Vec::new())
    } else {
        (vec![0.0f32; gw], vec![0.0f32; gw])
    };
    // separable 5x5 box sums of gx^2, gy^2, gx gy streamed by row
    let mut hs_ring = vec![[0.0f32; 3]; 5 * rw];
    let mut vacc = vec![[0.0f32; 3]; rw];
    let mut lam = vec![0.0f32; rw * rh];
    let mut max_l = 0.0f32;
    for j in 0..gh {
        let (gx_slice, gy_slice) = if need_edges {
            (&mut gxv[j * gw..(j + 1) * gw], &mut gyv[j * gw..(j + 1) * gw])
        } else {
            (&mut gx_row[..], &mut gy_row[..])
        };
        let y = gy0 + j;
        let row_c = &img.px[y * img.w + gx0 - 1..][..gw + 2];
        let row_u = &img.px[(y - 1) * img.w + gx0..][..gw];
        let row_d = &img.px[(y + 1) * img.w + gx0..][..gw];
        for i in 0..gw {
            gx_slice[i] = 0.5 * (f32::from(row_c[i + 2]) - f32::from(row_c[i]));
            gy_slice[i] = 0.5 * (f32::from(row_d[i]) - f32::from(row_u[i]));
        }
        let ring_row = &mut hs_ring[(j % 5) * rw..(j % 5 + 1) * rw];
        if j < 4 {
            stream_corner_row::<false, false>(gx_slice, gy_slice, ring_row, &mut vacc, &mut [], &mut max_l);
        } else if j == 4 {
            let lam_row = &mut lam[0..rw];
            stream_corner_row::<false, true>(gx_slice, gy_slice, ring_row, &mut vacc, lam_row, &mut max_l);
        } else {
            let out_row = j - 4;
            let lam_row = &mut lam[out_row * rw..(out_row + 1) * rw];
            stream_corner_row::<true, true>(gx_slice, gy_slice, ring_row, &mut vacc, lam_row, &mut max_l);
        }
    }
    // corners: relative and absolute floors (a 5x5 window of 2-grey-level gradients is no corner)
    let floor = (p.corner_rel_floor * max_l).max(100.0);
    let mut cand: Vec<(f32, usize, usize)> = Vec::new();
    for j in 1..rh.saturating_sub(1) {
        for i in 1..rw.saturating_sub(1) {
            let l = lam[j * rw + i];
            if l < floor { continue; }
            let local_max = (0..3).all(|dj| (0..3).all(|di| (di == 1 && dj == 1) || lam[(j + dj - 1) * rw + i + di - 1] <= l));
            if local_max { cand.push((l, x0 + i, y0 + j)); }
        }
    }
    cand.sort_by(|a, b| b.0.total_cmp(&a.0));
    if p.corner_cell_px > 0 {
        // by rank within the cell, then strength: a high-contrast pattern (black and white codes)
        // does not take the whole quota from a weaker texture beside it
        let c = p.corner_cell_px;
        let mut rank: std::collections::HashMap<(usize, usize), usize> = Default::default();
        let mut keyed: Vec<(usize, usize, (f32, usize, usize))> = cand.iter().enumerate().map(|(i, &e)| {
            let r = rank.entry((e.1 / c, e.2 / c)).or_insert(0);
            *r += 1;
            (*r, i, e)
        }).collect();
        keyed.sort_by_key(|k| (k.0, k.1));
        cand = keyed.into_iter().map(|k| k.2).collect();
    }
    let mut out: Vec<Feature> = Vec::new();
    let (mut n_in, mut n_out) = (0, 0);
    let (qi, qo) = (p.max_object_features, p.max_table_features);
    let d2 = p.min_feature_dist_px * p.min_feature_dist_px;
    let mut corner_grid = Spacing::new(x0, y0, rw, rh);
    for (l, x, y) in cand {
        let is_in = in_rect(x as f64, y as f64, &inner);
        if (is_in && n_in >= qi) || (!is_in && n_out >= qo) { continue; }
        if p.exclude_px.is_some_and(|r| in_rect(x as f64, y as f64, &r)) { continue; }
        if corner_grid.near(x, y, d2) { continue; }
        corner_grid.add(x, y);
        out.push(Feature { x, y, normal: None, side: 0.0, strength: l });
        if is_in { n_in += 1 } else { n_out += 1 }
        if n_in >= qi && n_out >= qo { break; }
    }
    // edge points: gradient maxima across the edge in the edge region
    // quotas: edges with a uniform side (outlines of plain faces) up to max_edge_features; edges
    // textured on both sides (texture, covered by the corners) up to a third of it, less a fifth
    // of the corners in the box
    let quota_plain = p.max_edge_features;
    let quota_tex = (p.max_edge_features / 3).saturating_sub(n_in / 5);
    if quota_plain == 0 { return out; }
    let g = |i: usize, j: usize| -> (f32, f32) { let k = (j + 2) * gw + i + 2; (gxv[k], gyv[k]) };
    // squared gradient magnitudes over the region (no hypot: it is slow on the device's libm)
    let mag2: Vec<f32> = (0..rw * rh).map(|k| { let (a, b) = g(k % rw, k / rw); a * a + b * b }).collect();
    let max_m2 = mag2.iter().copied().fold(0.0f32, f32::max);
    let thr = p.edge_min_grad.max(0.1 * max_m2.sqrt());
    let thr2 = thr * thr;
    let (ex0, ey0) = ((edge_region[0].max(0.0) as usize).max(x0 + 1), (edge_region[1].max(0.0) as usize).max(y0 + 1));
    let (ex1, ey1) = ((edge_region[2].max(0.0) as usize).min(x1.saturating_sub(1)), (edge_region[3].max(0.0) as usize).min(y1.saturating_sub(1)));
    let mut ecand: Vec<(f32, usize, usize, [f64; 2])> = Vec::new();
    for y in ey0..=ey1 {
        let j = y - y0;
        for x in ex0..=ex1 {
            let i = x - x0;
            let m2 = mag2[j * rw + i];
            if m2 < thr2 { continue; }
            let (a, b) = g(i, j);
            // neighbours across the edge (gradient direction quantised to 45 deg)
            let (ax, ay) = (a.abs(), b.abs());
            let (di, dj): (isize, isize) = if ax > 2.414 * ay { (1, 0) } else if ay > 2.414 * ax { (0, 1) } else if (a > 0.0) == (b > 0.0) { (1, 1) } else { (1, -1) };
            let at = |ii: isize, jj: isize| mag2[(jj as usize) * rw + ii as usize];
            if m2 < at(i as isize + di, j as isize + dj) || m2 < at(i as isize - di, j as isize - dj) { continue; }
            let mg = m2.sqrt();
            ecand.push((mg, x, y, [f64::from(a / mg), f64::from(b / mg)]));
        }
    }
    // thin to the strongest candidate per 2x2 block (the edge points are kept 4 px apart anyway)
    ecand.sort_by(|a, b| (a.2 / 2, a.1 / 2).cmp(&(b.2 / 2, b.1 / 2)).then(b.0.total_cmp(&a.0)));
    ecand.dedup_by(|a, b| (a.1 / 2, a.2 / 2) == (b.1 / 2, b.2 / 2));
    // ranked with the uniformity of their sides: the smaller of the two sides' standard
    // deviations (7x7 patches 5 px across)
    // integral images of the values and their squares over the region plus 8 px
    let (ix0, iy0) = (x0.saturating_sub(8), y0.saturating_sub(8));
    let (ix1, iy1) = ((x1 + 8).min(img.w - 1), (y1 + 8).min(img.h - 1));
    let iw = ix1 - ix0 + 2;
    let mut s1 = vec![0u64; iw * (iy1 - iy0 + 2)];
    let mut s2 = vec![0u64; s1.len()];
    for y in iy0..=iy1 {
        let (mut r1, mut r2) = (0u64, 0u64);
        for x in ix0..=ix1 {
            let v = u64::from(img.px[y * img.w + x]);
            r1 += v; r2 += v * v;
            let k = (y - iy0 + 1) * iw + x - ix0 + 1;
            s1[k] = s1[k - iw] + r1;
            s2[k] = s2[k - iw] + r2;
        }
    }
    let side_std = |x: usize, y: usize, nrm: [f64; 2], sgn: f64| -> f32 {
        let (cx, cy) = ((x as f64 + sgn * 5.0 * nrm[0]).round(), (y as f64 + sgn * 5.0 * nrm[1]).round());
        if cx < (ix0 + 3) as f64 || cy < (iy0 + 3) as f64 || cx > (ix1 - 3) as f64 || cy > (iy1 - 3) as f64 { return f32::INFINITY; }
        let (cx, cy) = (cx as usize - ix0, cy as usize - iy0);
        let rect = |t: &[u64]| t[(cy + 4) * iw + cx + 4] + t[(cy - 3) * iw + cx - 3] - t[(cy - 3) * iw + cx + 4] - t[(cy + 4) * iw + cx - 3];
        let (a, b) = (rect(&s1) as f64, rect(&s2) as f64);
        ((b / 49.0 - (a / 49.0) * (a / 49.0)).max(0.0)).sqrt() as f32
    };
    let mut ranked: Vec<(f32, usize, usize, [f64; 2], f32)> = ecand.into_iter().map(|(mg, x, y, nrm)| {
        let (sp, sn) = (side_std(x, y, nrm, 1.0), side_std(x, y, nrm, -1.0));
        let uniform = sp.min(sn);
        let side = if uniform < UNIFORM_STD && sp.max(sn) > 2.0 * uniform + 2.0 { if sp < sn { 1.0 } else { -1.0 } } else { 0.0 };
        let plain = uniform < UNIFORM_STD;
        (if plain { mg + 1e4 } else { mg }, x, y, nrm, side)
    }).collect();
    ranked.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut edge_grid = Spacing::new(x0, y0, rw, rh);
    let (de2, dc2) = (p.min_edge_dist_px * p.min_edge_dist_px, 9.0);
    let (mut n_plain, mut n_tex) = (0, 0);
    for (score, x, y, nrm, side) in ranked {
        let plain = score >= 1e4;
        if (plain && n_plain >= quota_plain) || (!plain && n_tex >= quota_tex) { continue; }
        if corner_grid.near(x, y, dc2) || edge_grid.near(x, y, de2) { continue; }
        if p.exclude_px.is_some_and(|r| in_rect(x as f64, y as f64, &r)) { continue; }
        edge_grid.add(x, y);
        out.push(Feature { x, y, normal: Some(nrm), side, strength: if plain { score - 1e4 } else { score } });
        if plain { n_plain += 1 } else { n_tex += 1 }
        if n_plain >= quota_plain && n_tex >= quota_tex { break; }
    }
    out
}

/// Accepted points on a grid of 8 px cells, for minimum-distance checks (distances below 8 px).
struct Spacing {
    x0: usize,
    y0: usize,
    cw: usize,
    ch: usize,
    cells: Vec<Vec<(usize, usize)>>,
}

impl Spacing {
    const CELL: usize = 8;
    fn new(x0: usize, y0: usize, w: usize, h: usize) -> Self {
        let (cw, ch) = (w / Self::CELL + 1, h / Self::CELL + 1);
        Self { x0, y0, cw, ch, cells: vec![Vec::new(); cw * ch] }
    }
    fn cell(&self, x: usize, y: usize) -> (usize, usize) { (((x - self.x0) / Self::CELL).min(self.cw - 1), ((y - self.y0) / Self::CELL).min(self.ch - 1)) }
    fn add(&mut self, x: usize, y: usize) { let (i, j) = self.cell(x, y); self.cells[j * self.cw + i].push((x, y)); }
    fn near(&self, x: usize, y: usize, d2: f64) -> bool {
        let (i, j) = self.cell(x, y);
        for jj in j.saturating_sub(1)..=(j + 1).min(self.ch - 1) {
            for ii in i.saturating_sub(1)..=(i + 1).min(self.cw - 1) {
                if self.cells[jj * self.cw + ii].iter().any(|&(a, b)| { let (dx, dy) = (a as f64 - x as f64, b as f64 - y as f64); dx * dx + dy * dy < d2 }) { return true; }
            }
        }
        false
    }
}

// ---- ray sweep ----

/// A view with its half-resolution image.
struct PView<'a> {
    cam: Camera,
    full: &'a Gray,
    half: Gray,
}

enum SweepEnd { NoPeak, Ambiguous }

/// The projection into a target camera of the point at inverse optical depth w on a source ray:
/// w X_cam(w) = w a + b (a = R_t^T (c_s - c_t), b = R_t^T d), so a pixel costs a few
/// multiply-adds and one division per depth sample.
struct RayProj { a: Vec3, b: Vec3, fx: f64, fy: f64, cx: f64, cy: f64 }

impl RayProj {
    fn new(src: &Camera, d: &Vec3, t: &Camera) -> Self {
        let rt = kinematics::transpose(&t.r);
        Self { a: kinematics::mat_vec(&rt, &kinematics::sub(&src.c, &t.c)), b: kinematics::mat_vec(&rt, d), fx: t.fx, fy: t.fy, cx: t.cx, cy: t.cy }
    }
    /// As [`Camera::project`] of the point at inverse depth `w` (> 0).
    #[inline]
    fn at(&self, w: f64) -> Option<[f64; 2]> {
        let z = self.a[2] * w + self.b[2];
        if z <= 0.01 * w { return None; }
        let inv = 1.0 / z;
        Some([self.fx * (self.a[0] * w + self.b[0]) * inv + self.cx, self.fy * (self.a[1] * w + self.b[1]) * inv + self.cy])
    }
}

/// Sweep the ray of `px` in `src` (inverse depth, world z between `z_lo` and `z_hi`) against
/// `targets`: coarse on the half-resolution images with `tc`, then (with `tf`) refined at full
/// resolution. Returns the inverse optical depth (1/m).
fn sweep(src: &Camera, px: [f64; 2], tc: &Tmpl, tf: Option<&Tmpl>, targets: &[&PView], z_lo: f64, z_hi: f64, p: &Params) -> Result<f64, SweepEnd> {
    let d = src.ray(px);
    if d[2] > -1e-3 { return Err(SweepEnd::NoPeak); }
    let z_far = (z_lo - src.c[2]) / d[2];
    let z_near = ((z_hi - src.c[2]) / d[2]).max(0.05);
    if z_far.is_nan() || z_near.is_nan() || z_far <= z_near { return Err(SweepEnd::NoPeak); }
    let (w0, w1) = (1.0 / z_far, 1.0 / z_near);
    let (pf, pn) = (src.at_inv_depth(&d, w0), src.at_inv_depth(&d, w1));
    let mut span: f64 = 0.0;
    for v in targets {
        match (v.cam.project(&pf), v.cam.project(&pn)) {
            (Some(a), Some(b)) => span = span.max((a[0] - b[0]).hypot(a[1] - b[1])),
            _ => span = span.max(800.0),
        }
    }
    let step = 2.0 * p.sweep_step_px;
    let n = ((span / step).ceil() as usize).clamp(2, 600);
    let dw = (w1 - w0) / n as f64;
    let proj: Vec<RayProj> = targets.iter().map(|v| RayProj::new(src, &d, &v.cam)).collect();
    let mut scores = vec![f32::NEG_INFINITY; n + 1];
    for (i, s) in scores.iter_mut().enumerate() {
        let w = w0 + i as f64 * dw;
        let mut sum = 0.0f32;
        let mut ok = true;
        for (v, pr) in targets.iter().zip(&proj) {
            match pr.at(w).and_then(|q| { let h = to_half(q); ncc(&v.half, tc, h[0], h[1], false) }) {
                Some(c) => sum += c,
                None => { ok = false; break; }
            }
        }
        if ok { *s = sum / targets.len() as f32; }
    }
    let (ib, &best) = scores.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).ok_or(SweepEnd::NoPeak)?;
    if f64::from(best) < p.sweep_min { return Err(SweepEnd::NoPeak); }
    // best separate peak: local maxima at least 2 px of image motion from the best
    let excl = (2.0 / step).ceil() as usize;
    let mut second = f32::NEG_INFINITY;
    for i in 0..=n {
        if i.abs_diff(ib) <= excl || !scores[i].is_finite() { continue; }
        let l = if i > 0 { scores[i - 1] } else { f32::NEG_INFINITY };
        let r = if i < n { scores[i + 1] } else { f32::NEG_INFINITY };
        if scores[i] >= l && scores[i] >= r { second = second.max(scores[i]); }
    }
    if second.is_finite() && (1.0 - f64::from(best)) > p.ratio * (1.0 - f64::from(second)) { return Err(SweepEnd::Ambiguous); }
    let wb = w0 + ib as f64 * dw;
    let Some(tf) = tf else { return Ok(wb); };
    // full resolution: bilinear scores at half coarse steps over +-2 coarse steps, parabola
    let score_at = |w: f64| -> f32 {
        let x = src.at_inv_depth(&d, w);
        let mut sum = 0.0f32;
        for v in targets {
            match v.cam.project(&x).and_then(|q| ncc(v.full, tf, q[0], q[1], true)) {
                Some(c) => sum += c,
                None => return f32::NEG_INFINITY,
            }
        }
        sum / targets.len() as f32
    };
    let q = dw / 2.0;
    let fine: Vec<(f64, f32)> = (-4..=4).map(|k| { let w = wb + f64::from(k) * q; (w, score_at(w)) }).collect();
    let (k, _) = fine.iter().enumerate().max_by(|a, b| a.1.1.total_cmp(&b.1.1)).ok_or(SweepEnd::NoPeak)?;
    let mut w = fine[k].0;
    if k > 0 && k + 1 < fine.len() {
        let (a, b, c) = (f64::from(fine[k - 1].1), f64::from(fine[k].1), f64::from(fine[k + 1].1));
        let den = a - 2.0 * b + c;
        if a.is_finite() && c.is_finite() && den < -1e-9 { w += 0.5 * (a - c) / den * q; }
    }
    Ok(w.clamp(w0, w1))
}

#[inline]
fn parabola(a: f32, b: f32, c: f32) -> f64 {
    let (a, b, c) = (f64::from(a), f64::from(b), f64::from(c));
    let den = a - 2.0 * b + c;
    if den < -1e-9 { (0.5 * (a - c) / den).clamp(-0.5, 0.5) } else { 0.0 }
}

/// 2D sub-pixel match of `t` near `q`: NCC on a 3x3 grid of 0.5 px (moved while the best is on
/// its edge), then a parabola per axis. Returns the pixel and its NCC.
fn refine_2d(img: &Gray, t: &Tmpl, mut q: [f64; 2]) -> Option<([f64; 2], f64)> {
    const H: f64 = 0.5;
    for _ in 0..6 {
        let mut g = [[f32::NEG_INFINITY; 3]; 3];
        let mut best = (1usize, 1usize, f32::NEG_INFINITY);
        for (j, row) in g.iter_mut().enumerate() {
            for (i, cell) in row.iter_mut().enumerate() {
                *cell = ncc(img, t, q[0] + (i as f64 - 1.0) * H, q[1] + (j as f64 - 1.0) * H, true)?;
                if *cell > best.2 { best = (i, j, *cell); }
            }
        }
        let (i, j, s) = best;
        if (i, j) != (1, 1) { q = [q[0] + (i as f64 - 1.0) * H, q[1] + (j as f64 - 1.0) * H]; continue; }
        let dx = parabola(g[1][0], g[1][1], g[1][2]);
        let dy = parabola(g[0][1], g[1][1], g[2][1]);
        return Some(([q[0] + dx * H, q[1] + dy * H], f64::from(s)));
    }
    None
}

fn refine_2d_cached(mut q: [f64; 2], mut eval: impl FnMut([f64; 2]) -> Option<f32>) -> Option<([f64; 2], f64)> {
    const H: f64 = 0.5;
    let mut prev: Option<([[f32; 3]; 3], isize, isize)> = None;
    for _ in 0..6 {
        let mut g = [[f32::NEG_INFINITY; 3]; 3];
        let mut best = (1usize, 1usize, f32::NEG_INFINITY);
        for (j, row) in g.iter_mut().enumerate() {
            for (i, cell) in row.iter_mut().enumerate() {
                let v = match prev {
                    Some((pg, di, dj)) => {
                        let (pi, pj) = (i as isize + di, j as isize + dj);
                        if (0..3).contains(&pi) && (0..3).contains(&pj) {
                            pg[pj as usize][pi as usize]
                        } else {
                            eval([q[0] + (i as f64 - 1.0) * H, q[1] + (j as f64 - 1.0) * H])?
                        }
                    }
                    None => eval([q[0] + (i as f64 - 1.0) * H, q[1] + (j as f64 - 1.0) * H])?,
                };
                *cell = v;
                if v > best.2 { best = (i, j, v); }
            }
        }
        let (i, j, s) = best;
        if (i, j) != (1, 1) {
            let (di, dj) = (i as isize - 1, j as isize - 1);
            q = [q[0] + (i as f64 - 1.0) * H, q[1] + (j as f64 - 1.0) * H];
            prev = Some((g, di, dj));
            continue;
        }
        let dx = parabola(g[1][0], g[1][1], g[1][2]);
        let dy = parabola(g[0][1], g[1][1], g[2][1]);
        return Some(([q[0] + dx * H, q[1] + dy * H], f64::from(s)));
    }
    None
}

pub(crate) fn refine_2d_pose(img: &Gray, t: &Tmpl, q: [f64; 2]) -> Option<([f64; 2], f64)> {
    refine_2d_cached(q, |p| ncc_half_grid(img, t, p[0], p[1]))
}

/// Match of `t` near `q` along the unit image direction `e` only (an edge point): NCC at
/// -0.5, 0, 0.5 px (moved while the best is at an end), parabola.
fn refine_1d(img: &Gray, t: &Tmpl, mut q: [f64; 2], e: [f64; 2]) -> Option<([f64; 2], f64)> {
    const H: f64 = 0.5;
    for _ in 0..8 {
        let s: Vec<f32> = (-1..=1).map(|k| ncc(img, t, q[0] + f64::from(k) * H * e[0], q[1] + f64::from(k) * H * e[1], true)).collect::<Option<_>>()?;
        if s[0] > s[1] && s[0] >= s[2] { q = [q[0] - H * e[0], q[1] - H * e[1]]; continue; }
        if s[2] > s[1] && s[2] > s[0] { q = [q[0] + H * e[0], q[1] + H * e[1]]; continue; }
        let dt = parabola(s[0], s[1], s[2]) * H;
        return Some(([q[0] + dt * e[0], q[1] + dt * e[1]], f64::from(s[1])));
    }
    None
}

// ---- triangulation ----

/// Least-squares midpoint of the rays through `px[i]` of `cams[i]`: the point minimising the sum
/// of squared distances to the rays. Returns it with the rms and max reprojection error (px).
pub fn triangulate(cams: &[Camera], px: &[[f64; 2]]) -> Option<(Vec3, f64, f64)> {
    if cams.len() < 2 || cams.len() != px.len() { return None; }
    let mut a = [[0.0f64; 3]; 3];
    let mut b = [0.0f64; 3];
    for (c, q) in cams.iter().zip(px) {
        let d = kinematics::unit(&c.ray(*q));
        for i in 0..3 {
            for j in 0..3 {
                let m = if i == j { 1.0 } else { 0.0 } - d[i] * d[j];
                a[i][j] += m;
                b[i] += m * c.c[j];
            }
        }
    }
    let x = solve3(&a, &b)?;
    let (mut ss, mut mx) = (0.0f64, 0.0f64);
    for (c, q) in cams.iter().zip(px) {
        let r = c.project(&x)?;
        let e = (r[0] - q[0]).hypot(r[1] - q[1]);
        ss += e * e;
        mx = mx.max(e);
    }
    Some((x, (ss / cams.len() as f64).sqrt(), mx))
}

/// Solve the 3x3 system by Cramer's rule; None when (nearly) singular (parallel rays).
fn solve3(a: &Mat3, b: &Vec3) -> Option<Vec3> {
    let det = |m: &Mat3| m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    let d = det(a);
    if d.abs() < 1e-12 { return None; }
    let mut out = [0.0; 3];
    for (k, o) in out.iter_mut().enumerate() {
        let mut m = *a;
        for i in 0..3 { m[i][k] = b[i]; }
        *o = det(&m) / d;
    }
    Some(out)
}

// ---- reconstruction ----

/// Up to `k` of `cand` (indices into `cams`) spread farthest apart: the farthest from `origin`
/// first, then each next the farthest from those chosen.
fn spread(cams: &[Camera], cand: &[usize], origin: &Vec3, k: usize) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    let dist = |a: &Vec3, b: &Vec3| kinematics::norm(&kinematics::sub(a, b));
    while out.len() < k && out.len() < cand.len() {
        let next = cand.iter().copied().filter(|i| !out.contains(i)).max_by(|&a, &b| {
            let s = |i: usize| if out.is_empty() { dist(&cams[i].c, origin) } else { out.iter().map(|&o| dist(&cams[i].c, &cams[o].c)).fold(f64::INFINITY, f64::min) };
            s(a).total_cmp(&s(b))
        });
        match next { Some(n) => out.push(n), None => break }
    }
    out
}

/// Image direction (unit) of the epipolar line in `cam` at the projection of `x`, for the baseline `b`.
fn epipolar_dir(cam: &Camera, x: &Vec3, b: &Vec3) -> Option<[f64; 2]> {
    let bn = kinematics::unit(b);
    let p0 = cam.project(x)?;
    let p1 = cam.project(&[x[0] + 0.01 * bn[0], x[1] + 0.01 * bn[1], x[2] + 0.01 * bn[2]])?;
    let (dx, dy) = (p1[0] - p0[0], p1[1] - p0[1]);
    let n = dx.hypot(dy);
    (n > 1e-9).then(|| [dx / n, dy / n])
}

/// Features of `views[reference]` in `region` (px), matched in the other views and triangulated.
/// `object_box` (px) splits the feature quotas, bounds the edge region and marks [`Point::in_box`].
pub fn reconstruct(views: &[View], reference: usize, region: [f64; 4], object_box: [f64; 4], p: &Params) -> Reconstruction {
    reconstruct_perturbed(views, reference, region, object_box, p, |_, q| q)
}

/// [`reconstruct`] with `perturb(view index, pixel)` applied to every matched pixel before the
/// triangulation (the tests add localisation noise with it).
pub fn reconstruct_perturbed(views: &[View], reference: usize, region: [f64; 4], object_box: [f64; 4], p: &Params,
    mut perturb: impl FnMut(usize, [f64; 2]) -> [f64; 2]) -> Reconstruction {
    let t_all = Instant::now();
    let mut rec = Reconstruction::default();
    if reference >= views.len() || views.len() < 2 {
        #[cfg(feature = "mview-profile")]
        {
            rec.profile = Some(RecProfile::default());
        }
        return rec;
    }
    #[cfg(feature = "mview-profile")]
    let rec_guard = profile_tls::RecProfileGuard::enter();
    let t0 = Instant::now();
    let pv: Vec<PView> = views.iter().map(|v| PView { cam: v.cam, full: &v.img, half: v.img.half() }).collect();
    let cams: Vec<Camera> = views.iter().map(|v| v.cam).collect();
    rec.stats.pyramid_ms = ms(t0);
    let rv = &pv[reference];
    let others: Vec<usize> = (0..views.len()).filter(|i| *i != reference).collect();
    let corner_sweep = spread(&cams, &others, &rv.cam.c, SWEEP_VIEWS);
    let (bw, bh) = (object_box[2] - object_box[0], object_box[3] - object_box[1]);
    let edge_region = [object_box[0] - EDGE_REGION_GROW * bw - 4.0, object_box[1] - EDGE_REGION_GROW * bh - 4.0,
        object_box[2] + EDGE_REGION_GROW * bw + 4.0, object_box[3] + EDGE_REGION_GROW * bh + 4.0];
    let t0 = Instant::now();
    let feats = features(rv.full, region, object_box, edge_region, p);
    rec.stats.features_ms = ms(t0);
    rec.stats.features = feats.len();
    rec.stats.corners = feats.iter().filter(|f| f.normal.is_none()).count();
    rec.stats.edges = rec.stats.features - rec.stats.corners;
    let sin_min = EDGE_MIN_ANGLE_DEG.to_radians().sin();
    for f in feats {
        let px = [f.x as f64, f.y as f64];
        let in_box = in_rect(px[0], px[1], &object_box);
        let (z_lo, z_hi) = (p.table_z_m - DEPTH_BELOW_TABLE_M, p.table_z_m + if in_box { p.max_height_m } else { TABLE_SWEEP_ABOVE_M });
        let d = rv.cam.ray(px);
        // views for this feature
        let valid: Vec<usize> = match f.normal {
            None => others.clone(),
            Some(nrm) => {
                let tangent = [-nrm[1], nrm[0]];
                let zm = 0.5 * (z_lo + z_hi);
                if d[2] > -1e-3 { rec.stats.no_peak += 1; rec.outcomes.push((f, "no_peak")); continue; }
                let xm = rv.cam.at_inv_depth(&d, d[2] / (zm - rv.cam.c[2]));
                others.iter().copied().filter(|&i| {
                    epipolar_dir(&rv.cam, &xm, &kinematics::sub(&cams[i].c, &rv.cam.c))
                        .is_some_and(|e| (e[0] * tangent[1] - e[1] * tangent[0]).abs() > sin_min)
                }).collect()
            }
        };
        if valid.is_empty() { rec.stats.edge_no_view += 1; rec.outcomes.push((f, "edge_no_view")); continue; }
        let swept = if f.normal.is_none() { corner_sweep.clone() } else { spread(&cams, &valid, &rv.cam.c, SWEEP_VIEWS) };
        let hq = to_half(px);
        let (Some(tc), Some(tf)) = (
            sample_bilinear(&rv.half, hq[0], hq[1], COARSE_R).and_then(|v| make_tmpl(v, COARSE_R, f.normal, f.side)),
            make_tmpl(sample_int(rv.full, f.x, f.y, PATCH_R), PATCH_R, f.normal, f.side),
        ) else { rec.stats.no_peak += 1; rec.outcomes.push((f, "flat")); continue; };
        let t0 = Instant::now();
        let targets: Vec<&PView> = swept.iter().map(|&i| &pv[i]).collect();
        let s = sweep(&rv.cam, px, &tc, Some(&tf), &targets, z_lo, z_hi, p);
        rec.stats.sweep_ms += ms(t0);
        let w = match s {
            Ok(w) => w,
            Err(SweepEnd::NoPeak) => { rec.stats.no_peak += 1; rec.outcomes.push((f, "no_peak")); continue; }
            Err(SweepEnd::Ambiguous) => { rec.stats.ambiguous += 1; rec.outcomes.push((f, "ambiguous")); continue; }
        };
        let x0 = rv.cam.at_inv_depth(&d, w);
        let mut mcams = vec![rv.cam];
        let mut pxs = vec![perturb(reference, px)];
        let mut fail = false;
        for &vi in &valid {
            let v = &pv[vi];
            let t0 = Instant::now();
            let m = v.cam.project(&x0).and_then(|q0| match f.normal {
                None => refine_2d(v.full, &tf, q0),
                Some(_) => {
                    let e = {
                        let (a, b) = (v.cam.project(&rv.cam.at_inv_depth(&d, w * 0.98)), v.cam.project(&rv.cam.at_inv_depth(&d, w * 1.02)));
                        a.zip(b).and_then(|(a, b)| { let (dx, dy) = (b[0] - a[0], b[1] - a[1]); let n = dx.hypot(dy); (n > 1e-9).then(|| [dx / n, dy / n]) })
                    };
                    e.and_then(|e| refine_1d(v.full, &tf, q0, e))
                }
            });
            rec.stats.refine_ms += ms(t0);
            let Some((q, _)) = m.filter(|(_, s)| *s >= p.ncc_min) else { fail = true; rec.stats.refine_rejected += 1; rec.outcomes.push((f, "refine")); break; };
            if swept.contains(&vi) {
                // left-right: this view's patch swept back (coarse) along its own ray into the reference
                let t0 = Instant::now();
                let hq = to_half(q);
                let back = sample_bilinear(&v.half, hq[0], hq[1], COARSE_R).and_then(|tv| make_tmpl(tv, COARSE_R, f.normal, f.side))
                    .and_then(|tb| sweep(&v.cam, q, &tb, None, &[rv], z_lo, z_hi, p).ok())
                    .and_then(|wb| { let db = v.cam.ray(q); rv.cam.project(&v.cam.at_inv_depth(&db, wb)) });
                rec.stats.sweep_ms += ms(t0);
                if !back.is_some_and(|b| (b[0] - px[0]).hypot(b[1] - px[1]) <= p.lr_tol_px) { fail = true; rec.stats.lr_rejected += 1; rec.outcomes.push((f, "left_right")); break; }
            }
            mcams.push(v.cam);
            pxs.push(perturb(vi, q));
        }
        if fail { continue; }
        let t0 = Instant::now();
        let tri = triangulate(&mcams, &pxs);
        let consistent = f.normal.is_none() || (mcams.len() >= 3 && {
            let pts: Vec<Vec3> = (1..mcams.len()).filter_map(|k| triangulate(&[mcams[0], mcams[k]], &[pxs[0], pxs[k]]).map(|t| t.0)).collect();
            let tol = EDGE_CONSISTENCY_M;
            pts.len() == mcams.len() - 1 && pts.iter().enumerate().all(|(i, a)| pts[i + 1..].iter().all(|b| kinematics::norm(&kinematics::sub(a, b)) <= tol))
        });
        rec.stats.triangulate_ms += ms(t0);
        match tri {
            Some((x, rms, mx)) if mx <= p.reproj_max_px => {
                let edge = f.normal.is_some();
                if edge { rec.stats.edge_points += 1; if consistent { rec.stats.edge_consistent += 1; } }
                rec.outcomes.push((f, "point")); rec.points.push(Point { x, ref_px: px, reproj_rms_px: rms, views: mcams.len(), in_box, edge, consistent });
            }
            _ => { rec.stats.reproj_rejected += 1; rec.outcomes.push((f, "reprojection")); }
        }
    }
    rec.stats.points = rec.points.len();
    rec.stats.total_ms = ms(t_all);
    #[cfg(feature = "mview-profile")]
    {
        rec.profile = Some(rec_guard.finish());
    }
    rec
}

#[inline]
fn ms(t: Instant) -> f64 { t.elapsed().as_secs_f64() * 1000.0 }

// ---- measurement ----

#[derive(Debug, Clone, PartialEq)]
pub struct ObjectMeasure {
    pub centroid_m: Vec3,
    pub top_z_m: f64,
    pub height_m: f64,
    pub width_m: f64,
    pub length_m: f64,
    pub yaw_deg: f64,
    pub n_points: usize,
    /// Object points that are edge points, and of those the ones used for the top.
    pub n_edge_points: usize,
    pub n_top_points: usize,
    pub reproj_rms_px: f64,
    /// Indices (into the measured points) of the object points.
    pub points: Vec<usize>,
    /// How the top was found: `cluster` (the highest top cluster) or `p90` (no cluster: the p90
    /// of the locally median-filtered heights).
    pub top_method: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TableCheck {
    pub n_points: usize,
    /// Signed median of (z - table plane) (mm): a common calibration bias of the views shows here
    /// and in the object heights alike.
    pub median_mm: Option<f64>,
    pub median_abs_mm: Option<f64>,
    pub p90_abs_mm: Option<f64>,
}

/// Value at quantile `q` of sorted `v` (linear interpolation).
pub fn quantile(v: &[f64], q: f64) -> f64 {
    if v.is_empty() { return f64::NAN; }
    let pos = q.clamp(0.0, 1.0) * (v.len() - 1) as f64;
    let (i, f) = (pos.floor() as usize, pos - pos.floor());
    if i + 1 < v.len() { v[i] * (1.0 - f) + v[i + 1] * f } else { v[i] }
}

/// Table points vs the plane z = `table_z_m` (see the module doc).
pub fn table_check(points: &[Point], p: &Params) -> TableCheck {
    let mut h: Vec<f64> = points.iter().map(|q| (q.x[2] - p.table_z_m) * 1000.0).filter(|h| (-TABLE_BAND_BELOW_M * 1000.0..=p.object_min_h_m * 1000.0).contains(h)).collect();
    h.sort_by(f64::total_cmp);
    let mut d: Vec<f64> = h.iter().map(|x| x.abs()).collect();
    d.sort_by(f64::total_cmp);
    let n = d.len();
    TableCheck { n_points: n, median_mm: (n > 0).then(|| quantile(&h, 0.5)), median_abs_mm: (n > 0).then(|| quantile(&d, 0.5)), p90_abs_mm: (n > 0).then(|| quantile(&d, 0.9)) }
}

/// Object measurement from the points in the box above the table; `Err("too_few_points")`
/// below `min_object_points` object points or [`MIN_TOP_POINTS`] points usable for the top.
pub fn measure(points: &[Point], p: &Params) -> Result<ObjectMeasure, &'static str> {
    let idx: Vec<usize> = (0..points.len()).filter(|&i| { let q = &points[i]; q.in_box && p.height(&q.x) > p.object_min_h_m }).collect();
    if idx.len() < p.min_object_points { return Err("too_few_points"); }
    // isolated points (false matches): distance to the ISOLATION_K-th nearest object point above
    // max(3 x its median, ISOLATION_MIN_M)
    let kth: Vec<f64> = idx.iter().map(|&i| {
        let mut d: Vec<f64> = idx.iter().map(|&o| kinematics::norm(&kinematics::sub(&points[o].x, &points[i].x))).collect();
        let k = ISOLATION_K.min(d.len() - 1);
        d.select_nth_unstable_by(k, f64::total_cmp);
        d[k]
    }).collect();
    let mut sorted = kth.clone();
    sorted.sort_by(f64::total_cmp);
    let lim = (3.0 * quantile(&sorted, 0.5)).max(ISOLATION_MIN_M);
    let idx: Vec<usize> = idx.into_iter().zip(&kth).filter(|(_, d)| **d <= lim).map(|(i, _)| i).collect();
    let obj: Vec<&Point> = idx.iter().map(|&i| &points[i]).collect();
    if obj.len() < p.min_object_points { return Err("too_few_points"); }
    let top_pts: Vec<&&Point> = obj.iter().filter(|q| q.consistent).collect();
    if top_pts.len() < MIN_TOP_POINTS { return Err("too_few_points"); }
    // top: the highest cluster of top points (see TOP_CLUSTER_MIN), else the p90 of the heights
    // after a local median (each point's height = median of its TOP_NEIGHBOURS nearest top points
    // in x, y), so the per-point noise does not lift the p90
    let th: Vec<f64> = top_pts.iter().map(|q| p.height(&q.x)).collect();
    let mut order: Vec<usize> = (0..top_pts.len()).collect();
    order.sort_by(|&a, &b| th[b].total_cmp(&th[a]));
    // the top points within TOP_CLUSTER_XY_M of point i and TOP_CLUSTER_DZ_M of height c: median, count
    let around = |i: usize, c: f64| -> (f64, usize) {
        let mut nb: Vec<f64> = (0..top_pts.len()).filter(|&j| (th[j] - c).abs() <= TOP_CLUSTER_DZ_M
            && (top_pts[j].x[0] - top_pts[i].x[0]).hypot(top_pts[j].x[1] - top_pts[i].x[1]) <= TOP_CLUSTER_XY_M).map(|j| th[j]).collect();
        nb.sort_by(f64::total_cmp);
        (quantile(&nb, 0.5), nb.len())
    };
    // the seed is the highest point of its window, so its median sits high: re-centre the
    // height window on the median a few times (the local mode)
    let cluster = order.iter().find_map(|&i| {
        let (mut c, n) = around(i, th[i]);
        if n < TOP_CLUSTER_MIN { return None; }
        for _ in 0..5 {
            let (m, k) = around(i, c);
            if k == 0 || (m - c).abs() < 1e-5 { break; }
            c = m;
        }
        Some(c)
    });
    let mut z: Vec<f64> = top_pts.iter().map(|q| {
        let mut nb: Vec<(f64, f64)> = top_pts.iter().map(|o| ((o.x[0] - q.x[0]).hypot(o.x[1] - q.x[1]), p.height(&o.x))).collect();
        let k = TOP_NEIGHBOURS.min(nb.len());
        nb.select_nth_unstable_by(k - 1, |a, b| a.0.total_cmp(&b.0));
        let mut zs: Vec<f64> = nb[..k].iter().map(|n| n.1).collect();
        zs.sort_by(f64::total_cmp);
        quantile(&zs, 0.5)
    }).collect();
    z.sort_by(f64::total_cmp);
    let top_h = cluster.unwrap_or_else(|| quantile(&z, 0.9));
    let n = obj.len() as f64;
    let (mx, my) = (obj.iter().map(|q| q.x[0]).sum::<f64>() / n, obj.iter().map(|q| q.x[1]).sum::<f64>() / n);
    let (mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0);
    for q in &obj {
        let (dx, dy) = (q.x[0] - mx, q.x[1] - my);
        sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
    }
    let theta = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    let (c, s) = (theta.cos(), theta.sin());
    let mut u: Vec<f64> = obj.iter().map(|q| (q.x[0] - mx) * c + (q.x[1] - my) * s).collect();
    let mut v: Vec<f64> = obj.iter().map(|q| -(q.x[0] - mx) * s + (q.x[1] - my) * c).collect();
    u.sort_by(f64::total_cmp);
    v.sort_by(f64::total_cmp);
    let (u0, u1, v0, v1) = (quantile(&u, 0.02), quantile(&u, 0.98), quantile(&v, 0.02), quantile(&v, 0.98));
    let (uc, vc) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let centre = [mx + uc * c - vc * s, my + uc * s + vc * c];
    let yaw = (theta.to_degrees() + 90.0).rem_euclid(180.0) - 90.0;
    let rms = (obj.iter().map(|q| q.reproj_rms_px * q.reproj_rms_px).sum::<f64>() / n).sqrt();
    let tz = p.table_z_at(centre[0], centre[1]);
    Ok(ObjectMeasure {
        centroid_m: [centre[0], centre[1], tz + 0.5 * top_h],
        top_z_m: tz + top_h,
        height_m: top_h,
        width_m: v1 - v0,
        length_m: u1 - u0,
        yaw_deg: yaw,
        n_points: obj.len(),
        n_edge_points: obj.iter().filter(|q| q.edge).count(),
        n_top_points: top_pts.len(),
        reproj_rms_px: rms,
        points: idx,
        top_method: if cluster.is_some() { "cluster" } else { "p90" },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cam_at(c: Vec3, look: Vec3) -> Camera {
        let z = kinematics::unit(&kinematics::sub(&look, &c));
        let x = kinematics::unit(&kinematics::cross(&z, &[0.0, 0.0, 1.0]));
        let y = kinematics::cross(&z, &x);
        Camera { r: kinematics::from_columns(&x, &y, &z), c, fx: 804.0, fy: 804.0, cx: 320.0, cy: 180.0 }
    }

    #[test]
    fn triangulation_is_exact_without_noise_and_reports_reprojection() {
        let x = [0.30, 0.02, 0.05];
        let cams = [cam_at([0.1, -0.04, 0.3], x), cam_at([0.1, 0.0, 0.3], x), cam_at([0.1, 0.04, 0.3], x)];
        let px: Vec<[f64; 2]> = cams.iter().map(|c| c.project(&x).unwrap()).collect();
        let (y, rms, mx) = triangulate(&cams, &px).unwrap();
        assert!(kinematics::norm(&kinematics::sub(&x, &y)) < 1e-9, "{y:?}");
        assert!(rms < 1e-6 && mx < 1e-6);
        // one view off by 3 px: the error shows in the reprojection
        let mut bad = px.clone();
        bad[2][0] += 3.0;
        let (_, rms, mx) = triangulate(&cams, &bad).unwrap();
        assert!(mx > 0.5 && rms > 0.3, "{rms} {mx}");
        // parallel rays (one camera twice) do not triangulate
        assert!(triangulate(&[cams[0], cams[0]], &[px[0], px[0]]).is_none());
        assert!(triangulate(&cams[..1], &px[..1]).is_none());
    }

    fn pt(x: f64, y: f64, z: f64, in_box: bool) -> Point {
        Point { x: [x, y, z], ref_px: [0.0; 2], reproj_rms_px: 0.3, views: 3, in_box, edge: false, consistent: true }
    }

    #[test]
    fn table_and_object_points_separate_by_height_above_the_plane() {
        let p = Params::new(0.01);
        let mut pts = Vec::new();
        // a 40 x 20 mm box, 30 mm high, long side along +45 deg, centre (0.3, 0.05)
        let (c, s) = (std::f64::consts::FRAC_PI_4.cos(), std::f64::consts::FRAC_PI_4.sin());
        for i in 0..=10 {
            for j in 0..=5 {
                let (u, v) = (-0.02 + 0.004 * f64::from(i), -0.01 + 0.004 * f64::from(j));
                pts.push(pt(0.3 + u * c - v * s, 0.05 + u * s + v * c, 0.04, true));
            }
        }
        // table points (in and out of the box), within +-2 mm, and one far below (ignored)
        for i in 0..20 { pts.push(pt(0.25 + 0.005 * f64::from(i), 0.0, 0.01 + if i % 2 == 0 { 0.002 } else { -0.001 }, i % 3 == 0)); }
        pts.push(pt(0.3, 0.0, -0.2, false));
        // a raised point outside the box does not count as object
        pts.push(pt(0.4, 0.2, 0.09, false));
        let m = measure(&pts, &p).unwrap();
        assert_eq!((m.n_points, m.n_top_points, m.n_edge_points), (66, 66, 0));
        assert!((m.height_m - 0.03).abs() < 1e-9 && (m.top_z_m - 0.04).abs() < 1e-9);
        assert!((m.length_m - 0.04).abs() < 1.5e-3 && (m.width_m - 0.02).abs() < 1.5e-3, "{m:?}");
        assert!((m.yaw_deg - 45.0).abs() < 0.5, "{m:?}");
        assert!((m.centroid_m[0] - 0.3).abs() < 1e-3 && (m.centroid_m[1] - 0.05).abs() < 1e-3 && (m.centroid_m[2] - 0.025).abs() < 1e-9);
        let t = table_check(&pts, &p);
        assert_eq!(t.n_points, 20);
        assert!((t.median_abs_mm.unwrap() - 1.5).abs() < 1e-6 && t.p90_abs_mm.unwrap() <= 2.0 + 1e-9, "{t:?}");
        assert_eq!(measure(&pts[66..], &p), Err("too_few_points"));
    }

    #[test]
    fn the_table_plane_is_fitted_robustly_and_sets_the_heights() {
        let mut p = Params::new(0.01);
        // table points 3 mm above the calibrated table, tilted 1 deg about y, +-0.5 mm noise;
        // a cable 8 mm high and points in the box do not count
        let tilt = 1f64.to_radians().tan();
        let z = |x: f64| 0.013 + tilt * (x - 0.3);
        let mut pts: Vec<Point> = (0..60).map(|i| {
            let (x, y) = (0.2 + 0.004 * f64::from(i % 20) * 2.5, -0.1 + 0.01 * f64::from(i / 20) * 10.0);
            pt(x, y, z(x) + if i % 2 == 0 { 0.0005 } else { -0.0005 }, false)
        }).collect();
        pts.extend((0..8).map(|i| pt(0.25 + 0.003 * f64::from(i), 0.05, z(0.25) + 0.008, false)));
        // the object: 40 mm above the plane
        pts.extend((0..20).map(|i| pt(0.29 + 0.001 * f64::from(i), 0.0, z(0.29 + 0.001 * f64::from(i)) + 0.04, true)));
        let t = fit_table_plane(&pts, &p, [0.3, 0.0]).unwrap();
        assert_eq!((t.n_points, t.n_inliers), (68, 60), "{t:?}");
        assert!((t.offset_mm - 3.0).abs() < 0.1 && (t.tilt_deg - 1.0).abs() < 0.1, "{t:?}");
        assert!(t.mad_mm > 0.4 && t.mad_mm < 1.0 && t.threshold_mm == 5.0, "{t:?}");
        p.plane = Some(t.plane);
        let m = measure(&pts, &p).unwrap();
        assert!((m.height_m - 0.04).abs() < 2e-4 && (m.top_z_m - z(m.centroid_m[0]) - 0.04).abs() < 2e-4, "{m:?}");
        // too few points, or a plane far from the calibrated table: no fit
        assert!(fit_table_plane(&pts[..9], &Params::new(0.01), [0.3, 0.0]).is_none());
        assert!(fit_table_plane(&pts, &Params::new(-0.002), [0.3, 0.0]).is_none());
    }

    #[test]
    fn the_top_is_the_highest_cluster_not_a_few_stray_points() {
        let p = Params::new(0.0);
        // a 30 mm top of 20 points over 40 mm, a lower step at 15 mm, and two stray points at 50 mm
        // 30 mm apart (no cluster): the top is 30 mm
        let mut pts: Vec<Point> = (0..20).map(|i| pt(0.3 + 0.002 * f64::from(i), 0.0, 0.030 + 0.0005 * f64::from(i % 3), true)).collect();
        pts.extend((0..20).map(|i| pt(0.3 + 0.002 * f64::from(i), 0.02, 0.015, true)));
        pts.extend([pt(0.31, 0.005, 0.05, true), pt(0.34, 0.005, 0.05, true)]);
        let m = measure(&pts, &p).unwrap();
        assert_eq!(m.top_method, "cluster");
        assert!((m.height_m - 0.0305).abs() < 1e-9, "{m:?}");
        // scattered points, no 3 within 3 mm and 15 mm: the p90 fallback
        let pts: Vec<Point> = (0..10).map(|i| pt(0.3 + 0.02 * f64::from(i), 0.0, 0.01 + 0.004 * f64::from(i), true)).collect();
        let m = measure(&pts, &Params { object_min_h_m: 0.001, ..p }).unwrap();
        assert_eq!(m.top_method, "p90");
    }

    #[test]
    fn inconsistent_edge_points_do_not_set_the_top() {
        let p = Params::new(0.0);
        // 20 corners at 30 mm and 20 view-dependent silhouette points at 60 mm
        let mut pts: Vec<Point> = (0..20).map(|i| pt(0.3 + 0.002 * f64::from(i), 0.0, 0.03, true)).collect();
        pts.extend((0..20).map(|i| Point { edge: true, consistent: false, ..pt(0.3 + 0.002 * f64::from(i), 0.01, 0.06, true) }));
        let m = measure(&pts, &p).unwrap();
        assert!((m.height_m - 0.03).abs() < 1e-9, "{m:?}");
        assert_eq!((m.n_points, m.n_edge_points, m.n_top_points), (40, 20, 20));
        // only inconsistent edge points: no top
        assert_eq!(measure(&pts[20..], &p), Err("too_few_points"));
    }

    #[test]
    fn corners_and_edges_respect_quotas_and_distance() {
        // checkerboard of 8 px squares: corners at the square junctions, edges between them
        let (w, h) = (160, 120);
        let px: Vec<u8> = (0..w * h).map(|k| { let (x, y) = (k % w, k / w); if ((x / 8) + (y / 8)) % 2 == 0 { 40 } else { 210 } }).collect();
        let img = Gray::new(w, h, px).unwrap();
        let mut p = Params::new(0.0);
        (p.max_object_features, p.max_table_features, p.min_feature_dist_px, p.max_edge_features) = (5, 3, 6.0, 0);
        let f = features(&img, [0.0, 0.0, 159.0, 119.0], [40.0, 40.0, 100.0, 80.0], [40.0, 40.0, 100.0, 80.0], &p);
        assert_eq!(f.len(), 8, "{f:?}");
        assert!(f.iter().all(|f| f.normal.is_none()));
        let inside = f.iter().filter(|p| p.x >= 40 && p.x <= 100 && p.y >= 40 && p.y <= 80).count();
        assert_eq!(inside, 5);
        for (i, a) in f.iter().enumerate() {
            for b in &f[i + 1..] { assert!((a.x as f64 - b.x as f64).hypot(a.y as f64 - b.y as f64) >= 6.0); }
        }
        // a plain bright rectangle (50..110 x 40..80) on a dark ground: 4 corners, edge points
        // along its sides with normals across them, apart from the corners and each other
        let px: Vec<u8> = (0..w * h).map(|k| { let (x, y) = (k % w, k / w); if (50..110).contains(&x) && (40..80).contains(&y) { 200 } else { 60 } }).collect();
        let img = Gray::new(w, h, px).unwrap();
        let mut p = Params::new(0.0);
        p.max_edge_features = 30;
        let f = features(&img, [0.0, 0.0, 159.0, 119.0], [50.0, 40.0, 110.0, 80.0], [40.0, 30.0, 120.0, 90.0], &p);
        let (c, e): (Vec<&Feature>, Vec<&Feature>) = f.iter().partition(|f| f.normal.is_none());
        assert_eq!(c.len(), 4, "{c:?}");
        assert_eq!(e.len(), 30, "edges with a uniform side: the full quota");
        assert!(e.iter().all(|q| q.side == 0.0), "both sides uniform");
        for q in &e {
            let n = q.normal.unwrap();
            assert!(n[0].abs() > 0.99 || n[1].abs() > 0.99, "{q:?}");
            assert!(c.iter().all(|k| (k.x as f64 - q.x as f64).hypot(k.y as f64 - q.y as f64) >= 3.0));
        }
        for (i, a) in e.iter().enumerate() {
            for b in &e[i + 1..] { assert!((a.x as f64 - b.x as f64).hypot(a.y as f64 - b.y as f64) >= 4.0); }
        }
        let flat = Gray::new(w, h, vec![128; w * h]).unwrap();
        assert!(features(&flat, [0.0, 0.0, 159.0, 119.0], [0.0; 4], [0.0; 4], &Params::new(0.0)).is_empty());
    }

    #[test]
    fn half_resolution_pixels_average_2x2_blocks() {
        let img = Gray::new(4, 2, vec![0, 4, 8, 8, 0, 4, 16, 16]).unwrap();
        let h = img.half();
        assert_eq!((h.w, h.h, h.px.clone()), (2, 1, vec![2, 12]));
        assert_eq!(to_half([0.5, 0.5]), [0.0, 0.0]);
    }

    fn assert_refine_eq(a: Option<([f64; 2], f64)>, b: Option<([f64; 2], f64)>) {
        match (a, b) {
            (None, None) => {}
            (Some((pa, sa)), Some((pb, sb))) => {
                assert_eq!(pa[0].to_bits(), pb[0].to_bits(), "x mismatch: {pa:?} vs {pb:?}");
                assert_eq!(pa[1].to_bits(), pb[1].to_bits(), "y mismatch: {pa:?} vs {pb:?}");
                assert_eq!(sa.to_bits(), sb.to_bits(), "score mismatch: {sa} vs {sb}");
            }
            _ => panic!("option mismatch: {a:?} vs {b:?}"),
        }
    }

    #[test]
    fn refine_2d_pose_matches_refine_2d_across_all_eight_shift_directions() {
        let (w, h) = (32, 32);
        let px: Vec<u8> = (0..w * h).map(|k| {
            let (x, y) = ((k % w) as f64, (k / w) as f64);
            (128.0 + 50.0 * (0.47 * x + 0.19 * y).sin() + 45.0 * (0.31 * x - 0.53 * y).cos() + 20.0 * (0.11 * x * y).sin())
                .round().clamp(0.0, 255.0) as u8
        }).collect();
        let img = Gray::new(w, h, px).unwrap();
        let q0 = [16.5, 16.5];

        for dj in -1..=1isize {
            for di in -1..=1isize {
                if di == 0 && dj == 0 { continue; }
                for steps in [1.0f64, 2.0f64] {
                    let target = [q0[0] + 0.5 * steps * di as f64, q0[1] + 0.5 * steps * dj as f64];
                    let v = sample_bilinear(&img, target[0], target[1], 4).unwrap();
                    let t = make_tmpl(v, 4, None, 0.0).unwrap();
                    let got = refine_2d_pose(&img, &t, q0);
                    let exp = refine_2d(&img, &t, q0);
                    assert!(got.is_some(), "expected convergence for ({di}, {dj}) steps={steps}");
                    assert_refine_eq(got, exp);
                }

                let tx = q0[0] + 0.5 * di as f64;
                let ty = q0[1] + 0.5 * dj as f64;
                let mut seen: Vec<[f64; 2]> = Vec::new();
                let res = refine_2d_cached(q0, |p| {
                    assert!(!seen.iter().any(|q| q[0] == p[0] && q[1] == p[1]), "re-evaluated {p:?} on ({di}, {dj})");
                    seen.push(p);
                    let (dx, dy) = (p[0] - tx, p[1] - ty);
                    Some(-((dx * dx + dy * dy) as f32))
                }).unwrap();
                let expected_evals = if di == 0 || dj == 0 { 12 } else { 14 };
                assert_eq!(seen.len(), expected_evals, "direction ({di}, {dj})");
                assert!((res.0[0] - tx).abs() < 1e-12 && (res.0[1] - ty).abs() < 1e-12, "{res:?}");
            }
        }
    }

    #[test]
    fn refine_2d_cached_preserves_tie_breaking_and_iteration_limit() {
        // Vertical-invariant image: all 3 rows at the matching x column produce identical NCC.
        let (w, h) = (32, 32);
        let px: Vec<u8> = (0..w * h).map(|k| {
            let x = (k % w) as f64;
            (128.0 + 70.0 * (0.63 * x).sin() + 30.0 * (0.29 * x).cos()).round().clamp(0.0, 255.0) as u8
        }).collect();
        let img = Gray::new(w, h, px).unwrap();
        let t = make_tmpl(sample_int(&img, 16, 16, 4), 4, None, 0.0).unwrap();
        // Horizontal-only shift to x=16.0: row j=0 beats j=1,2 on strict `>` and keeps shifting y by -0.5.
        assert_refine_eq(refine_2d_pose(&img, &t, [16.5, 16.5]), refine_2d(&img, &t, [16.5, 16.5]));
        // Centered on x=16.0: j=0..=2 at i=1 all tie; best stays at (1, 1) due to strict `>`.
        assert_refine_eq(refine_2d_pose(&img, &t, [16.0, 16.5]), refine_2d(&img, &t, [16.0, 16.5]));

        // Flat surface: row-major strict `>` picks (0, 0) over initial -inf every step -> None after 6 iters.
        let mut flat_evals = 0usize;
        let flat = refine_2d_cached([10.0, 10.0], |_| { flat_evals += 1; Some(0.5) });
        assert_eq!(flat_evals, 9 + 5 * 5);
        assert_eq!(flat, None);

        // Tie between center (1, 1) and later cells (2, 1), (2, 2): strict `>` keeps (1, 1).
        let mut center_tie_evals = 0usize;
        let center_tie = refine_2d_cached([10.0, 10.0], |p| {
            center_tie_evals += 1;
            if p[0] >= 10.0 && p[1] >= 10.0 && (p[0] == 10.0 && p[1] == 10.0 || p[0] == 10.5 && p[1] == 10.0 || p[0] == 10.5 && p[1] == 10.5) {
                Some(0.9)
            } else {
                Some(0.1)
            }
        });
        assert_eq!(center_tie_evals, 9);
        assert_eq!(center_tie, Some(([10.25, 10.0], f64::from(0.9f32))));

        // Symmetric tie between (0, 0) and (2, 2): row-major strict `>` picks (0, 0) = [9.5, 9.5].
        let sym = refine_2d_cached([10.0, 10.0], |p| {
            if (p[0] == 9.5 && p[1] == 9.5) || (p[0] == 10.5 && p[1] == 10.5) { Some(0.9) } else { Some(0.1) }
        });
        assert_eq!(sym, Some(([9.5, 9.5], f64::from(0.9f32))));

        // Monotonic ramp: hits the 6-iteration limit and returns None.
        let mut ramp_evals = 0usize;
        let ramp = refine_2d_cached([10.0, 10.0], |p| {
            ramp_evals += 1;
            Some((p[0] + p[1]) as f32)
        });
        assert_eq!(ramp, None);
        assert_eq!(ramp_evals, 9 + 5 * 5);
    }

    #[test]
    fn refine_2d_pose_matches_refine_2d_at_image_boundaries() {
        let (w, h) = (20, 20);
        let mut px: Vec<u8> = (0..w * h).map(|k| {
            let (x, y) = (k % w, k / w);
            ((x * 37 + y * 53) % 100 + 20) as u8
        }).collect();
        // Stamp a distinct 9x9 pattern at (5, 10) and also near the bottom-right at (14, 14).
        let stamp: Vec<u8> = (0..81).map(|k| {
            let (x, y) = (k % 9, k / 9);
            if (x + y) % 2 == 0 { 230 } else { 30 }
        }).collect();
        for sy in 0..9 {
            for sx in 0..9 {
                let v = stamp[sy * 9 + sx];
                px[(10 - 4 + sy) * w + (5 - 4 + sx)] = v;
                px[(14 - 4 + sy) * w + (14 - 4 + sx)] = v;
            }
        }
        let img = Gray::new(w, h, px).unwrap();
        let t = make_tmpl(sample_int(&img, 5, 10, 4), 4, None, 0.0).unwrap();

        // Iter 0 out of bounds (x = 4.5 < 5.0).
        assert_refine_eq(refine_2d_pose(&img, &t, [5.0, 10.0]), refine_2d(&img, &t, [5.0, 10.0]));
        assert!(refine_2d_pose(&img, &t, [5.0, 10.0]).is_none());

        // Iter 0 in bounds at [5.5, 10.0], shifts left to [5.0, 10.0], iter 1 hits x = 4.5 -> None.
        assert_refine_eq(refine_2d_pose(&img, &t, [5.5, 10.0]), refine_2d(&img, &t, [5.5, 10.0]));
        assert!(refine_2d_pose(&img, &t, [5.5, 10.0]).is_none());

        // Iter 0 in bounds at [13.5, 13.5], shifts diagonally to [14.0, 14.0], iter 1 hits x = 14.5 -> None.
        assert_refine_eq(refine_2d_pose(&img, &t, [13.5, 13.5]), refine_2d(&img, &t, [13.5, 13.5]));
        assert!(refine_2d_pose(&img, &t, [13.5, 13.5]).is_none());

        // Direct None propagation check on iter 1 after a cardinal shift.
        let mut calls = 0usize;
        let out = refine_2d_cached([10.0, 10.0], |p| {
            calls += 1;
            if p[0] < 9.5 { None } else if p[0] == 9.5 && p[1] == 10.0 { Some(1.0) } else { Some(0.2) }
        });
        assert_eq!(out, None);
        assert_eq!(calls, 10);
    }

    fn assert_ncc_eq(a: Option<f32>, b: Option<f32>) {
        match (a, b) {
            (None, None) => {}
            (Some(sa), Some(sb)) => assert_eq!(sa.to_bits(), sb.to_bits(), "ncc mismatch: {sa} vs {sb}"),
            _ => panic!("option mismatch: {a:?} vs {b:?}"),
        }
    }

    #[test]
    fn ncc_half_grid_matches_generic_across_all_four_phases() {
        let (w, h) = (32, 32);
        let px: Vec<u8> = (0..w * h).map(|k| {
            let (x, y) = ((k % w) as f64, (k / w) as f64);
            (128.0 + 50.0 * (0.47 * x + 0.19 * y).sin() + 45.0 * (0.31 * x - 0.53 * y).cos() + 20.0 * (0.11 * x * y).sin())
                .round().clamp(0.0, 255.0) as u8
        }).collect();
        let img = Gray::new(w, h, px).unwrap();

        for r in [PATCH_R, 2usize, 6usize] {
            let t_int = make_tmpl(sample_int(&img, 14, 14, r), r, None, 0.0).unwrap();
            let t_half = make_tmpl(sample_bilinear(&img, 14.5, 14.5, r).unwrap(), r, None, 0.0).unwrap();
            for t in [&t_int, &t_half] {
                for (x, y) in [
                    (14.0, 14.0),
                    (14.5, 14.0),
                    (14.0, 14.5),
                    (14.5, 14.5),
                    (12.0, 17.5),
                    (16.5, 13.0),
                ] {
                    let got = ncc_half_grid(&img, t, x, y);
                    let exp = ncc(&img, t, x, y, true);
                    assert!(got.is_some(), "expected Some for r={r} at ({x}, {y})");
                    assert_ncc_eq(got, exp);
                }
            }
        }
    }

    #[test]
    fn ncc_half_grid_bright_nearly_flat_and_max_accumulator_bounds() {
        // Bright nearly-flat region (250..=255) and all-255 flat region: verifies exact f64 variance without f32 cancellation.
        let (w, h) = (32, 32);
        let mut px = vec![255u8; w * h];
        // Texture at [6..15] x [6..15] for building a valid template, and bright nearly-flat texture around (16, 16).
        for y in 0..h {
            for x in 0..w {
                if (11..=21).contains(&x) && (11..=21).contains(&y) {
                    px[y * w + x] = 250 + (((x * 7 + y * 13) % 6) as u8);
                }
            }
        }
        let img = Gray::new(w, h, px).unwrap();
        let t_src: Vec<f32> = (0usize..81).map(|k| if (k % 9 + k / 9).is_multiple_of(2) { 255.0 } else { 245.0 }).collect();
        let t = make_tmpl(t_src, PATCH_R, None, 0.0).unwrap();
        for (x, y) in [(16.0, 16.0), (16.5, 16.0), (16.0, 16.5), (16.5, 16.5)] {
            let got = ncc_half_grid(&img, &t, x, y);
            let exp = ncc(&img, &t, x, y, true);
            assert!(got.is_some() && got.unwrap().abs() > 0.0);
            assert_ncc_eq(got, exp);
        }
        let flat_255 = Gray::new(w, h, vec![255u8; w * h]).unwrap();
        for (x, y) in [(16.0, 16.0), (16.5, 16.0), (16.0, 16.5), (16.5, 16.5)] {
            let got = ncc_half_grid(&flat_255, &t, x, y);
            assert_eq!(got, Some(0.0));
            assert_ncc_eq(got, ncc(&flat_255, &t, x, y, true));
        }

        // Max supported radius (HALF_GRID_MAX_R = 16) with max-brightness (255/254 and all-255) pixels:
        // exercises u32 accumulator upper bound (1089 * 1020^2 = 1_132_995_600 < u32::MAX) without overflow.
        let (bw, bh) = (40usize, 40usize);
        let max_px: Vec<u8> = (0..bw * bh).map(|k| if ((k % bw) + (k / bw)).is_multiple_of(2) { 255 } else { 254 }).collect();
        let max_img = Gray::new(bw, bh, max_px).unwrap();
        let all_255_big = Gray::new(bw, bh, vec![255u8; bw * bh]).unwrap();
        let n16 = 2 * HALF_GRID_MAX_R + 1;
        let t16_src: Vec<f32> = (0..n16 * n16).map(|k| if (k % n16 + k / n16).is_multiple_of(2) { 255.0 } else { 245.0 }).collect();
        let t16 = make_tmpl(t16_src, HALF_GRID_MAX_R, None, 0.0).unwrap();
        for (x, y) in [(18.0, 18.0), (18.5, 18.0), (18.0, 18.5), (18.5, 18.5)] {
            assert_ncc_eq(ncc_half_grid(&max_img, &t16, x, y), ncc(&max_img, &t16, x, y, true));
            let flat_got = ncc_half_grid(&all_255_big, &t16, x, y);
            assert_eq!(flat_got, Some(0.0));
            assert_ncc_eq(flat_got, ncc(&all_255_big, &t16, x, y, true));
        }

        // Unsupported radius (HALF_GRID_MAX_R + 1 = 17) falls back to generic NCC.
        let r17 = HALF_GRID_MAX_R + 1;
        let n17 = 2 * r17 + 1;
        let t17_src: Vec<f32> = (0..n17 * n17).map(|k| if (k % n17 + k / n17).is_multiple_of(2) { 255.0 } else { 245.0 }).collect();
        let t17 = make_tmpl(t17_src, r17, None, 0.0).unwrap();
        for (x, y) in [(18.0, 18.0), (18.5, 18.0), (18.0, 18.5), (18.5, 18.5)] {
            assert_ncc_eq(ncc_half_grid(&max_img, &t17, x, y), ncc(&max_img, &t17, x, y, true));
        }
    }

    #[test]
    fn ncc_half_grid_borders_weighted_and_arbitrary_phase_fallback() {
        let (w, h) = (24, 24);
        let px: Vec<u8> = (0..w * h).map(|k| {
            let (x, y) = ((k % w) as f64, (k / w) as f64);
            (120.0 + 60.0 * (0.51 * x + 0.23 * y).sin() + 40.0 * (0.37 * x - 0.41 * y).cos())
                .round().clamp(0.0, 255.0) as u8
        }).collect();
        let img = Gray::new(w, h, px).unwrap();
        let t = make_tmpl(sample_int(&img, 10, 10, PATCH_R), PATCH_R, None, 0.0).unwrap();

        // Margin is 1.0: for w=h=24 and r=4, valid range is [5.0, 18.0].
        // Integer (x, y) at r + 0.0 (= 4.0) must return None.
        for (x, y) in [
            (4.0, 10.0),
            (4.5, 10.0),
            (10.0, 4.0),
            (10.0, 4.5),
            (18.5, 10.0),
            (19.0, 10.0),
            (10.0, 18.5),
            (10.0, 19.0),
            (f64::NAN, 10.0),
        ] {
            assert_eq!(ncc_half_grid(&img, &t, x, y), None, "expected None at ({x}, {y})");
            assert_ncc_eq(ncc_half_grid(&img, &t, x, y), ncc(&img, &t, x, y, true));
        }

        // Just inside margin 1.0 across all four half-grid phases.
        for (x, y) in [
            (5.0, 5.0),
            (5.5, 5.0),
            (5.0, 5.5),
            (5.5, 5.5),
            (18.0, 18.0),
            (17.5, 18.0),
            (18.0, 17.5),
            (17.5, 17.5),
        ] {
            let got = ncc_half_grid(&img, &t, x, y);
            assert!(got.is_some(), "expected Some at ({x}, {y})");
            assert_ncc_eq(got, ncc(&img, &t, x, y, true));
        }

        // Weighted template fallback (`normal: Some(...)`).
        let tw = make_tmpl(sample_int(&img, 10, 10, PATCH_R), PATCH_R, Some([1.0, 0.0]), 1.0).unwrap();
        assert!(tw.w.is_some());
        for (x, y) in [(10.0, 10.0), (10.5, 10.0), (10.0, 10.5), (10.5, 10.5)] {
            assert_ncc_eq(ncc_half_grid(&img, &tw, x, y), ncc(&img, &tw, x, y, true));
        }

        // Arbitrary fractional coordinate fallback.
        for (x, y) in [(10.25, 11.75), (10.5, 11.3), (11.1, 10.0), (10.0, 11.9)] {
            assert_ncc_eq(ncc_half_grid(&img, &t, x, y), ncc(&img, &t, x, y, true));
        }
    }

    fn features_column_major_ref(img: &Gray, region: [f64; 4], inner: [f64; 4], edge_region: [f64; 4], p: &Params) -> Vec<Feature> {
        let m = PATCH_R + 4;
        let x0 = (region[0].floor().max(0.0) as usize).max(m);
        let y0 = (region[1].floor().max(0.0) as usize).max(m);
        let x1 = (region[2].ceil().max(0.0) as usize).min(img.w.saturating_sub(m + 1));
        let y1 = (region[3].ceil().max(0.0) as usize).min(img.h.saturating_sub(m + 1));
        if x1 <= x0 + 2 || y1 <= y0 + 2 { return Vec::new(); }
        let (gx0, gy0) = (x0 - 2, y0 - 2);
        let (gw, gh) = (x1 - x0 + 5, y1 - y0 + 5);
        let mut gxv = vec![0.0f32; gw * gh];
        let mut gyv = vec![0.0f32; gw * gh];
        for j in 0..gh {
            let y = gy0 + j;
            for i in 0..gw {
                let x = gx0 + i;
                gxv[j * gw + i] = 0.5 * (img.at(x + 1, y) - img.at(x - 1, y));
                gyv[j * gw + i] = 0.5 * (img.at(x, y + 1) - img.at(x, y - 1));
            }
        }
        let (rw, rh) = (x1 - x0 + 1, y1 - y0 + 1);
        let mut hs = vec![[0.0f32; 3]; rw * gh];
        for j in 0..gh {
            let row = j * gw;
            let mut acc = [0.0f32; 3];
            for i in 0..gw {
                let (a, b) = (gxv[row + i], gyv[row + i]);
                acc[0] += a * a; acc[1] += b * b; acc[2] += a * b;
                if i >= 5 { let (a, b) = (gxv[row + i - 5], gyv[row + i - 5]); acc[0] -= a * a; acc[1] -= b * b; acc[2] -= a * b; }
                if i >= 4 { hs[j * rw + i - 4] = acc; }
            }
        }
        let mut lam = vec![0.0f32; rw * rh];
        let mut max_l = 0.0f32;
        for i in 0..rw {
            let mut acc = [0.0f32; 3];
            for j in 0..gh {
                let h = hs[j * rw + i];
                for c in 0..3 { acc[c] += h[c]; }
                if j >= 5 { let h = hs[(j - 5) * rw + i]; for c in 0..3 { acc[c] -= h[c]; } }
                if j >= 4 {
                    let [a, c, b] = acc;
                    let half = 0.5 * (a + c);
                    let l = half - (0.25 * (a - c) * (a - c) + b * b).max(0.0).sqrt();
                    lam[(j - 4) * rw + i] = l;
                    max_l = max_l.max(l);
                }
            }
        }
        let floor = (p.corner_rel_floor * max_l).max(100.0);
        let mut cand: Vec<(f32, usize, usize)> = Vec::new();
        for j in 1..rh.saturating_sub(1) {
            for i in 1..rw.saturating_sub(1) {
                let l = lam[j * rw + i];
                if l < floor { continue; }
                let local_max = (0..3).all(|dj| (0..3).all(|di| (di == 1 && dj == 1) || lam[(j + dj - 1) * rw + i + di - 1] <= l));
                if local_max { cand.push((l, x0 + i, y0 + j)); }
            }
        }
        cand.sort_by(|a, b| b.0.total_cmp(&a.0));
        if p.corner_cell_px > 0 {
            let c = p.corner_cell_px;
            let mut rank: std::collections::HashMap<(usize, usize), usize> = Default::default();
            let mut keyed: Vec<(usize, usize, (f32, usize, usize))> = cand.iter().enumerate().map(|(i, &e)| {
                let r = rank.entry((e.1 / c, e.2 / c)).or_insert(0);
                *r += 1;
                (*r, i, e)
            }).collect();
            keyed.sort_by_key(|k| (k.0, k.1));
            cand = keyed.into_iter().map(|k| k.2).collect();
        }
        let mut out: Vec<Feature> = Vec::new();
        let (mut n_in, mut n_out) = (0, 0);
        let (qi, qo) = (p.max_object_features, p.max_table_features);
        let d2 = p.min_feature_dist_px * p.min_feature_dist_px;
        let mut corner_grid = Spacing::new(x0, y0, rw, rh);
        for (l, x, y) in cand {
            let is_in = in_rect(x as f64, y as f64, &inner);
            if (is_in && n_in >= qi) || (!is_in && n_out >= qo) { continue; }
            if p.exclude_px.is_some_and(|r| in_rect(x as f64, y as f64, &r)) { continue; }
            if corner_grid.near(x, y, d2) { continue; }
            corner_grid.add(x, y);
            out.push(Feature { x, y, normal: None, side: 0.0, strength: l });
            if is_in { n_in += 1 } else { n_out += 1 }
            if n_in >= qi && n_out >= qo { break; }
        }
        let quota_plain = p.max_edge_features;
        let quota_tex = (p.max_edge_features / 3).saturating_sub(n_in / 5);
        if quota_plain == 0 { return out; }
        let g = |i: usize, j: usize| -> (f32, f32) { let k = (j + 2) * gw + i + 2; (gxv[k], gyv[k]) };
        let mag2: Vec<f32> = (0..rw * rh).map(|k| { let (a, b) = g(k % rw, k / rw); a * a + b * b }).collect();
        let max_m2 = mag2.iter().copied().fold(0.0f32, f32::max);
        let thr = p.edge_min_grad.max(0.1 * max_m2.sqrt());
        let thr2 = thr * thr;
        let (ex0, ey0) = ((edge_region[0].max(0.0) as usize).max(x0 + 1), (edge_region[1].max(0.0) as usize).max(y0 + 1));
        let (ex1, ey1) = ((edge_region[2].max(0.0) as usize).min(x1.saturating_sub(1)), (edge_region[3].max(0.0) as usize).min(y1.saturating_sub(1)));
        let mut ecand: Vec<(f32, usize, usize, [f64; 2])> = Vec::new();
        for y in ey0..=ey1 {
            let j = y - y0;
            for x in ex0..=ex1 {
                let i = x - x0;
                let m2 = mag2[j * rw + i];
                if m2 < thr2 { continue; }
                let (a, b) = g(i, j);
                let (ax, ay) = (a.abs(), b.abs());
                let (di, dj): (isize, isize) = if ax > 2.414 * ay { (1, 0) } else if ay > 2.414 * ax { (0, 1) } else if (a > 0.0) == (b > 0.0) { (1, 1) } else { (1, -1) };
                let at = |ii: isize, jj: isize| mag2[(jj as usize) * rw + ii as usize];
                if m2 < at(i as isize + di, j as isize + dj) || m2 < at(i as isize - di, j as isize - dj) { continue; }
                let mg = m2.sqrt();
                ecand.push((mg, x, y, [f64::from(a / mg), f64::from(b / mg)]));
            }
        }
        ecand.sort_by(|a, b| (a.2 / 2, a.1 / 2).cmp(&(b.2 / 2, b.1 / 2)).then(b.0.total_cmp(&a.0)));
        ecand.dedup_by(|a, b| (a.1 / 2, a.2 / 2) == (b.1 / 2, b.2 / 2));
        let (ix0, iy0) = (x0.saturating_sub(8), y0.saturating_sub(8));
        let (ix1, iy1) = ((x1 + 8).min(img.w - 1), (y1 + 8).min(img.h - 1));
        let iw = ix1 - ix0 + 2;
        let mut s1 = vec![0u64; iw * (iy1 - iy0 + 2)];
        let mut s2 = vec![0u64; s1.len()];
        for y in iy0..=iy1 {
            let (mut r1, mut r2) = (0u64, 0u64);
            for x in ix0..=ix1 {
                let v = u64::from(img.px[y * img.w + x]);
                r1 += v; r2 += v * v;
                let k = (y - iy0 + 1) * iw + x - ix0 + 1;
                s1[k] = s1[k - iw] + r1;
                s2[k] = s2[k - iw] + r2;
            }
        }
        let side_std = |x: usize, y: usize, nrm: [f64; 2], sgn: f64| -> f32 {
            let (cx, cy) = ((x as f64 + sgn * 5.0 * nrm[0]).round(), (y as f64 + sgn * 5.0 * nrm[1]).round());
            if cx < (ix0 + 3) as f64 || cy < (iy0 + 3) as f64 || cx > (ix1 - 3) as f64 || cy > (iy1 - 3) as f64 { return f32::INFINITY; }
            let (cx, cy) = (cx as usize - ix0, cy as usize - iy0);
            let rect = |t: &[u64]| t[(cy + 4) * iw + cx + 4] + t[(cy - 3) * iw + cx - 3] - t[(cy - 3) * iw + cx + 4] - t[(cy + 4) * iw + cx - 3];
            let (a, b) = (rect(&s1) as f64, rect(&s2) as f64);
            ((b / 49.0 - (a / 49.0) * (a / 49.0)).max(0.0)).sqrt() as f32
        };
        let mut ranked: Vec<(f32, usize, usize, [f64; 2], f32)> = ecand.into_iter().map(|(mg, x, y, nrm)| {
            let (sp, sn) = (side_std(x, y, nrm, 1.0), side_std(x, y, nrm, -1.0));
            let uniform = sp.min(sn);
            let side = if uniform < UNIFORM_STD && sp.max(sn) > 2.0 * uniform + 2.0 { if sp < sn { 1.0 } else { -1.0 } } else { 0.0 };
            let plain = uniform < UNIFORM_STD;
            (if plain { mg + 1e4 } else { mg }, x, y, nrm, side)
        }).collect();
        ranked.sort_by(|a, b| b.0.total_cmp(&a.0));
        let mut edge_grid = Spacing::new(x0, y0, rw, rh);
        let (de2, dc2) = (p.min_edge_dist_px * p.min_edge_dist_px, 9.0);
        let (mut n_plain, mut n_tex) = (0, 0);
        for (score, x, y, nrm, side) in ranked {
            let plain = score >= 1e4;
            if (plain && n_plain >= quota_plain) || (!plain && n_tex >= quota_tex) { continue; }
            if corner_grid.near(x, y, dc2) || edge_grid.near(x, y, de2) { continue; }
            if p.exclude_px.is_some_and(|r| in_rect(x as f64, y as f64, &r)) { continue; }
            edge_grid.add(x, y);
            out.push(Feature { x, y, normal: Some(nrm), side, strength: if plain { score - 1e4 } else { score } });
            if plain { n_plain += 1 } else { n_tex += 1 }
            if n_plain >= quota_plain && n_tex >= quota_tex { break; }
        }
        out
    }

    fn assert_features_bit_eq(got: &[Feature], exp: &[Feature]) {
        assert_eq!(got.len(), exp.len(), "feature count mismatch");
        for (idx, (g, e)) in got.iter().zip(exp).enumerate() {
            assert_eq!(g.x, e.x, "x mismatch at {idx}");
            assert_eq!(g.y, e.y, "y mismatch at {idx}");
            match (g.normal, e.normal) {
                (None, None) => {}
                (Some(gn), Some(en)) => {
                    assert_eq!(gn[0].to_bits(), en[0].to_bits(), "normal[0] mismatch at {idx}");
                    assert_eq!(gn[1].to_bits(), en[1].to_bits(), "normal[1] mismatch at {idx}");
                }
                _ => panic!("normal presence mismatch at {idx}: {g:?} vs {e:?}"),
            }
            assert_eq!(g.side.to_bits(), e.side.to_bits(), "side mismatch at {idx}");
            assert_eq!(g.strength.to_bits(), e.strength.to_bits(), "strength mismatch at {idx}");
        }
    }

    #[test]
    fn row_streamed_features_match_column_major_reference() {
        let (w, h) = (180usize, 140usize);
        let mut seed = 0x1234_5678_9ABC_DEF0u64;
        let mut px = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let noise = ((seed & 0x1F) as i32) - 16;
                let val = if (40..130).contains(&x) && (35..105).contains(&y) {
                    if ((x / 10) + (y / 10)).is_multiple_of(2) { 215 } else { 45 }
                } else if (x / 12 + y / 12).is_multiple_of(3) {
                    180
                } else {
                    80
                };
                if (135..165).contains(&x) && (30..110).contains(&y) {
                    px[y * w + x] = 220;
                } else if (128..172).contains(&x) && (22..118).contains(&y) {
                    px[y * w + x] = 60;
                } else {
                    px[y * w + x] = (val + noise).clamp(0, 255) as u8;
                }
            }
        }
        let img = Gray::new(w, h, px).unwrap();
        let region = [4.0, 5.0, 174.0, 134.0];
        let inner = [38.0, 32.0, 132.0, 108.0];
        let edge_region = [30.0, 25.0, 168.0, 120.0];

        let mut p_pose = Params::new(0.0);
        p_pose.max_edge_features = 0;
        p_pose.corner_cell_px = 0;
        p_pose.corner_rel_floor = 0.01;
        p_pose.max_object_features = 0;
        p_pose.max_table_features = 100;
        p_pose.min_feature_dist_px = 8.0;
        let got_pose = features(&img, region, inner, edge_region, &p_pose);
        let exp_pose = features_column_major_ref(&img, region, inner, edge_region, &p_pose);
        assert!(!got_pose.is_empty());
        assert_features_bit_eq(&got_pose, &exp_pose);

        let p_grid = Params { corner_cell_px: 16, ..p_pose };
        let got_grid = features(&img, region, inner, edge_region, &p_grid);
        let exp_grid = features_column_major_ref(&img, region, inner, edge_region, &p_grid);
        assert!(!got_grid.is_empty());
        assert_features_bit_eq(&got_grid, &exp_grid);

        let mut p_edge = Params::new(0.0);
        p_edge.max_edge_features = 40;
        let got_edge = features(&img, region, inner, edge_region, &p_edge);
        let exp_edge = features_column_major_ref(&img, region, inner, edge_region, &p_edge);
        assert!(got_edge.iter().any(|f| f.normal.is_none()));
        assert!(got_edge.iter().any(|f| f.normal.is_some()));
        assert_features_bit_eq(&got_edge, &exp_edge);
    }

    #[test]
    fn kernel_counts_track_half_grid_and_generic_ncc() {
        let (w, h) = (32, 32);
        let px: Vec<u8> = (0..w * h)
            .map(|k| {
                let (x, y) = ((k % w) as f64, (k / w) as f64);
                (128.0 + 50.0 * (0.47 * x + 0.19 * y).sin() + 45.0 * (0.31 * x - 0.53 * y).cos())
                    .round()
                    .clamp(0.0, 255.0) as u8
            })
            .collect();
        let img = Gray::new(w, h, px).unwrap();
        let t = make_tmpl(sample_int(&img, 14, 14, PATCH_R), PATCH_R, None, 0.0).unwrap();

        reset_kernel_counts();
        assert!(ncc_half_grid(&img, &t, 14.5, 14.0).is_some());
        assert!(ncc(&img, &t, 14.25, 14.75, true).is_some());
        assert!(ncc(&img, &t, 1.0, 1.0, true).is_none());

        let counts = kernel_counts();
        match ncc_backend::NccBackend::active() {
            ncc_backend::NccBackend::Scalar => {
                assert_eq!(counts.half_moments_scalar, 1);
                assert_eq!(counts.half_moments_xthead071, 0);
            }
            ncc_backend::NccBackend::Xthead071 => {
                assert_eq!(counts.half_moments_xthead071, 1);
                assert_eq!(counts.half_moments_scalar, 0);
            }
        }
        assert_eq!(counts.patch_moments_scalar, 1);
        assert_eq!(counts.patch_moments_xthead071, 0);

        reset_kernel_counts();
        assert_eq!(kernel_counts(), KernelCallCounts::default());
    }

    #[test]
    fn half_grid_active_backend_matches_direct_scalar_across_all_spec_cases() {
        use crate::ncc_backend::NccBackend;

        let (w, h) = (48usize, 48usize);
        let mut px: Vec<u8> = (0..w * h)
            .map(|k| {
                let (x, y) = ((k % w) as f64, (k / w) as f64);
                (128.0
                    + 55.0 * (0.43 * x + 0.21 * y).sin()
                    + 40.0 * (0.29 * x - 0.51 * y).cos())
                .round()
                .clamp(0.0, 255.0) as u8
            })
            .collect();
        // Bright nearly-flat region (240..=255) around (20, 20) for variance cancellation check.
        for y in 14..27 {
            for x in 14..27 {
                px[y * w + x] = 248 + (((x * 5 + y * 9) % 8) as u8);
            }
        }
        let img = Gray::new(w, h, px).unwrap();
        let flat_255 = Gray::new(w, h, vec![255u8; w * h]).unwrap();
        let active = NccBackend::active();
        let scalar = NccBackend::scalar();

        // Radii 2 (n=5), 4 (n=9), 16 (n=33) across all 4 phases.
        for r in [2usize, PATCH_R, HALF_GRID_MAX_R] {
            let t = make_tmpl(sample_int(&img, 20, 20, r), r, None, 0.0).unwrap();
            for (x, y) in [(20.0, 20.0), (20.5, 20.0), (20.0, 20.5), (20.5, 20.5)] {
                reset_kernel_counts();
                let got = ncc_half_grid(&img, &t, x, y);
                let counts = kernel_counts();
                match active {
                    NccBackend::Scalar => {
                        assert_eq!(counts.half_moments_scalar, 1);
                        assert_eq!(counts.half_moments_xthead071, 0);
                    }
                    NccBackend::Xthead071 => {
                        assert_eq!(counts.half_moments_xthead071, 1);
                        assert_eq!(counts.half_moments_scalar, 0);
                    }
                }
                let direct_scalar = ncc_half_grid_with_backend(scalar, &img, &t, x, y);
                assert!(got.is_some());
                assert_ncc_eq(got, direct_scalar);
                assert_ncc_eq(got, ncc(&img, &t, x, y, true));

                // Flat 255 image -> var == 0 -> Some(0.0).
                let got_flat = ncc_half_grid(&flat_255, &t, x, y);
                assert_eq!(got_flat, Some(0.0));
                assert_ncc_eq(got_flat, ncc_half_grid_with_backend(scalar, &flat_255, &t, x, y));
            }
        }

        // Bright nearly-flat patch (240..=255) with tiny-contrast template.
        let bright_t_src: Vec<f32> = (0..81)
            .map(|k| 244.0 + ((k * 3) % 12) as f32)
            .collect();
        let bright_t = make_tmpl(bright_t_src, PATCH_R, None, 0.0).unwrap();
        for (x, y) in [(20.0, 20.0), (20.5, 20.0), (20.0, 20.5), (20.5, 20.5)] {
            let got = ncc_half_grid(&img, &bright_t, x, y);
            let direct_scalar = ncc_half_grid_with_backend(scalar, &img, &bright_t, x, y);
            assert!(got.is_some() && got.unwrap().abs() > 0.0);
            assert_ncc_eq(got, direct_scalar);
            assert_ncc_eq(got, ncc(&img, &bright_t, x, y, true));
        }

        // Radius 17 (n=35) scalar fallback in ncc_half_grid -> dispatches via patch_moments_scalar.
        let r17 = HALF_GRID_MAX_R + 1;
        let t17 = make_tmpl(sample_int(&img, 20, 20, r17), r17, None, 0.0).unwrap();
        reset_kernel_counts();
        let got17 = ncc_half_grid(&img, &t17, 20.5, 20.5);
        let c17 = kernel_counts();
        assert_eq!(c17.half_moments_scalar, 0);
        assert_eq!(c17.half_moments_xthead071, 0);
        assert_eq!(c17.patch_moments_scalar, 1);
        assert_ncc_eq(got17, ncc(&img, &t17, 20.5, 20.5, true));

        // Weighted template & arbitrary-phase fallback.
        let tw = make_tmpl(sample_int(&img, 20, 20, PATCH_R), PATCH_R, Some([1.0, 0.0]), 1.0).unwrap();
        let t4 = make_tmpl(sample_int(&img, 20, 20, PATCH_R), PATCH_R, None, 0.0).unwrap();
        reset_kernel_counts();
        let got_w = ncc_half_grid(&img, &tw, 20.5, 20.0);
        let got_arb = ncc_half_grid(&img, &t4, 20.25, 20.75);
        let c_fb = kernel_counts();
        assert_eq!(c_fb.half_moments_scalar, 0);
        assert_eq!(c_fb.half_moments_xthead071, 0);
        assert_eq!(c_fb.patch_moments_scalar, 2);
        assert_ncc_eq(got_w, ncc(&img, &tw, 20.5, 20.0, true));
        assert_ncc_eq(got_arb, ncc(&img, &t4, 20.25, 20.75, true));

        // Border rejection (just outside inside(..., 1.0)) and truncated Gray.px -> None with 0 calls.
        let trunc = Gray { w, h, px: vec![128u8; 200] };
        reset_kernel_counts();
        for (x, y) in [(4.0, 20.0), (4.5, 20.0), (42.5, 20.0), (20.0, 42.5)] {
            assert_eq!(ncc_half_grid(&img, &t4, x, y), None);
        }
        assert_eq!(ncc_half_grid(&trunc, &t4, 20.5, 20.5), None);
        assert_eq!(kernel_counts(), KernelCallCounts::default());

        // Tie direction and iteration cap in refine_2d_pose vs refine_2d.
        let tie_px: Vec<u8> = (0..w * h)
            .map(|k| {
                let x = (k % w) as f64;
                (128.0 + 70.0 * (0.61 * x).sin() + 25.0 * (0.31 * x).cos())
                    .round()
                    .clamp(0.0, 255.0) as u8
            })
            .collect();
        let tie_img = Gray::new(w, h, tie_px).unwrap();
        let tie_t = make_tmpl(sample_int(&tie_img, 20, 20, PATCH_R), PATCH_R, None, 0.0).unwrap();
        assert_refine_eq(
            refine_2d_pose(&tie_img, &tie_t, [20.5, 20.5]),
            refine_2d(&tie_img, &tie_t, [20.5, 20.5]),
        );
        // Monotonic step-away hitting 6-iteration cap -> None.
        let mut ramp_px = vec![128u8; w * h];
        for y in 0..h {
            for x in 0..w {
                ramp_px[y * w + x] = ((x * 3 + y * 5) % 256) as u8;
            }
        }
        let ramp_img = Gray::new(w, h, ramp_px).unwrap();
        let ramp_t = make_tmpl(sample_int(&ramp_img, 30, 30, PATCH_R), PATCH_R, None, 0.0).unwrap();
        assert_refine_eq(
            refine_2d_pose(&ramp_img, &ramp_t, [20.0, 20.0]),
            refine_2d(&ramp_img, &ramp_t, [20.0, 20.0]),
        );
    }

    #[cfg(feature = "mview-profile")]
    #[test]
    fn reconstruction_profile_counts_ncc_variants_without_pose_pollution() {
        let look = [0.30, 0.0, 0.0];
        let cams = [cam_at([0.1, -0.04, 0.3], look), cam_at([0.1, 0.0, 0.3], look), cam_at([0.1, 0.04, 0.3], look)];
        let (w, h) = (120usize, 90usize);
        let render = |cam: &Camera| -> Gray {
            let mut c = *cam;
            c.fx = 220.0;
            c.fy = 220.0;
            c.cx = 60.0;
            c.cy = 45.0;
            let mut px = vec![120u8; w * h];
            for y in 0..h {
                for x in 0..w {
                    let d = c.ray([x as f64, y as f64]);
                    let t = -c.c[2] / d[2];
                    let wx = c.c[0] + t * d[0];
                    let wy = c.c[1] + t * d[1];
                    let gx = (wx / 0.01).floor() as i64;
                    let gy = (wy / 0.01).floor() as i64;
                    let pat = ((gx.wrapping_mul(73856093) ^ gy.wrapping_mul(19349663)).wrapping_mul(83492791) >> 24) & 0xff;
                    px[y * w + x] = (40 + (pat * 180) / 255) as u8;
                }
            }
            Gray::new(w, h, px).unwrap()
        };
        let views: Vec<View> = cams
            .iter()
            .map(|cam| {
                let mut c = *cam;
                c.fx = 220.0;
                c.fy = 220.0;
                c.cx = 60.0;
                c.cy = 45.0;
                View { cam: c, img: render(cam) }
            })
            .collect();

        // Run a pose-style generic NCC call beforehand; it must NOT pollute RecProfile.
        let t_pre = make_tmpl(sample_int(&views[0].img, 40, 40, PATCH_R), PATCH_R, None, 0.0).unwrap();
        assert!(ncc(&views[0].img, &t_pre, 40.3, 40.2, true).is_some());

        let mut p = Params::new(0.0);
        p.max_object_features = 15;
        p.max_table_features = 10;
        p.max_edge_features = 10;
        let rec = reconstruct(&views, 1, [15.0, 15.0, 105.0, 75.0], [35.0, 25.0, 85.0, 65.0], &p);
        let prof = rec.profile.expect("profile should be populated");
        assert!(prof.total_ncc_calls > 0);
        let sum: u64 = prof.ncc_calls.values().sum();
        assert_eq!(sum, prof.total_ncc_calls);
        assert!(prof.ncc_calls.contains_key("nearest_unweighted_n5"));
        assert!(prof.ncc_calls.contains_key("bilinear_unweighted_n9"));
    }

    #[test]
    fn ncc_and_ncc_half_grid_reject_truncated_gray_buffers_without_panic() {
        let valid_px: Vec<u8> = (0..32 * 32).map(|k| ((k * 29 + 17) % 220 + 20) as u8).collect();
        let valid_img = Gray::new(32, 32, valid_px).unwrap();
        let t_unweighted = make_tmpl(sample_int(&valid_img, 14, 14, PATCH_R), PATCH_R, None, 0.0).unwrap();
        let t_weighted = make_tmpl(sample_int(&valid_img, 14, 14, PATCH_R), PATCH_R, Some([1.0, 0.0]), 1.0).unwrap();

        let trunc = Gray { w: 32, h: 32, px: vec![128u8; 300] };
        reset_kernel_counts();
        for bilinear in [false, true] {
            assert_eq!(ncc(&trunc, &t_unweighted, 16.0, 16.0, bilinear), None);
            assert_eq!(ncc(&trunc, &t_weighted, 16.25, 16.75, bilinear), None);
        }
        for (x, y) in [(16.0, 16.0), (16.5, 16.0), (16.0, 16.5), (16.5, 16.5), (16.3, 16.7)] {
            assert_eq!(ncc_half_grid(&trunc, &t_unweighted, x, y), None);
            assert_eq!(ncc_half_grid(&trunc, &t_weighted, x, y), None);
        }
        assert_eq!(kernel_counts(), KernelCallCounts::default());
    }
}
