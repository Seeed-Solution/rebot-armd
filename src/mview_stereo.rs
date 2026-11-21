//! Motion-stereo geometry of the action `measure_multiview` (src/measure_multiview.rs): object
//! points triangulated from grayscale images taken at known camera poses. Pure computation, no
//! I/O; the action runs it in `spawn_blocking`.
//!
//! - Features (reference view): Shi-Tomasi corners (minimum eigenvalue of the 5x5 structure
//!   tensor) in the feature region, at most `max_object_features` in the object box and
//!   `max_table_features` outside it, strongest first and `min_feature_dist_px` apart; plus edge
//!   points (gradient-magnitude maxima across the edge, at least `edge_min_grad` grey levels per
//!   pixel) in the box grown by [`EDGE_REGION_GROW`], for objects without texture: up to
//!   `max_edge_features` edges with a uniform side (a plain face: the outline of a plain object)
//!   and up to a third of it, less a fifth of the corners in the box, textured on both sides.
//! - Depth range: a feature in the box is swept between `table_z - DEPTH_BELOW_TABLE_M` and
//!   `table_z + max_height_m`, one outside it (table) up to `table_z + TABLE_SWEEP_ABOVE_M`.
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
//!   reference with each of them agree within [`EDGE_CONSISTENCY_M`] (or 3 px of disparity): a
//!   mismatch, or a silhouette of a curved surface that moves with the view, fails it.
//! - Measurement ([`measure`]): points more than `object_min_h_m` above the table plane whose
//!   reference pixel lies in the object box are object points, less isolated ones (false matches,
//!   [`ISOLATION_K`]). The top is the p90 of the locally
//!   median-filtered heights of the corners and the consistent edge points only; the footprint is
//!   the oriented extent (PCA of x, y, 2nd to 98th percentile) of all object points. Points within
//!   [-0.05, `object_min_h_m`] m of the plane are table points (`table_check`).
use std::time::Instant;

use crate::kinematics::{self, Mat3, Vec3};

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
/// Pairwise triangulations of an edge point must agree within max(this (m), the depth of
/// EDGE_CONSISTENCY_PX of disparity at the shortest pair baseline) for the top height.
pub const EDGE_CONSISTENCY_M: f64 = 0.004;
pub const EDGE_CONSISTENCY_PX: f64 = 3.0;
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

    #[inline]
    fn bilinear(&self, x: f64, y: f64) -> f32 {
        let (x0, y0) = (x.floor(), y.floor());
        let (fx, fy) = ((x - x0) as f32, (y - y0) as f32);
        let (x0, y0) = (x0 as usize, y0 as usize);
        let i = y0 * self.w + x0;
        let p = &self.px;
        let a = f32::from(p[i]) * (1.0 - fx) + f32::from(p[i + 1]) * fx;
        let b = f32::from(p[i + self.w]) * (1.0 - fx) + f32::from(p[i + self.w + 1]) * fx;
        a * (1.0 - fy) + b * fy
    }

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
}

impl Params {
    pub fn new(table_z_m: f64) -> Self {
        Self {
            table_z_m, max_height_m: 0.25, max_object_features: 200, max_table_features: 60, max_edge_features: 120,
            min_feature_dist_px: 5.0, min_edge_dist_px: 4.0, edge_min_grad: 6.0,
            sweep_min: 0.6, ncc_min: 0.8, ratio: 0.7, lr_tol_px: 2.0, reproj_max_px: 2.0, sweep_step_px: 0.75, object_min_h_m: 0.005,
            min_object_points: 8,
        }
    }
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

#[derive(Debug, Clone, Default)]
pub struct Reconstruction {
    pub points: Vec<Point>,
    pub stats: Stats,
    /// Every feature and how it ended (`point`, `no_peak`, `flat`, `ambiguous`, `edge_no_view`,
    /// `refine`, `left_right`, `reprojection`), for diagnostics.
    pub outcomes: Vec<(Feature, &'static str)>,
}

// ---- patches ----

/// Patch template: zero-mean, unit-norm weighted values `a` (= w (v - mean) / norm), weights `w`
/// (None = uniform) and their sum.
struct Tmpl {
    r: usize,
    a: Vec<f32>,
    w: Option<Vec<f32>>,
    sw: f32,
}

fn sample_bilinear(img: &Gray, x: f64, y: f64, r: usize) -> Option<Vec<f32>> {
    if !img.inside(x, y, r, 1.0) { return None; }
    let rf = r as f64;
    let n = 2 * r + 1;
    let mut v = Vec::with_capacity(n * n);
    for j in 0..n { for i in 0..n { v.push(img.bilinear(x - rf + i as f64, y - rf + j as f64)); } }
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
    let (mut s1, mut s2, mut dot) = (0.0f32, 0.0f32, 0.0f32);
    if bilinear {
        let rf = r as f64;
        for j in 0..n {
            for i in 0..n {
                let k = j * n + i;
                let v = img.bilinear(x - rf + i as f64, y - rf + j as f64);
                let wv = t.w.as_ref().map_or(v, |w| w[k] * v);
                s1 += wv; s2 += wv * v; dot += t.a[k] * v;
            }
        }
    } else {
        let (cx, cy) = (x.round() as usize, y.round() as usize);
        for j in 0..n {
            let row = &img.px[(cy + j - r) * img.w + cx - r..][..n];
            let a = &t.a[j * n..(j + 1) * n];
            match &t.w {
                None => for (&p, &ak) in row.iter().zip(a) { let v = f32::from(p); s1 += v; s2 += v * v; dot += ak * v; },
                Some(w) => {
                    let w = &w[j * n..(j + 1) * n];
                    for ((&p, &ak), &wk) in row.iter().zip(a).zip(w) { let v = f32::from(p); let wv = wk * v; s1 += wv; s2 += wv * v; dot += ak * v; }
                }
            }
        }
    }
    let var = s2 - s1 * s1 / t.sw;
    Some(if var > 1e-3 * t.sw { dot / var.sqrt() } else { 0.0 })
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
    // separable 5x5 box sums of gx^2, gy^2, gx gy: horizontal, then vertical
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
    // corners: relative and absolute floors (a 5x5 window of 2-grey-level gradients is no corner)
    let floor = (0.01 * max_l).max(100.0);
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
    let mut out: Vec<Feature> = Vec::new();
    let (mut n_in, mut n_out) = (0, 0);
    let (qi, qo) = (p.max_object_features, p.max_table_features);
    let d2 = p.min_feature_dist_px * p.min_feature_dist_px;
    let mut corner_grid = Spacing::new(x0, y0, rw, rh);
    for (l, x, y) in cand {
        let is_in = in_rect(x as f64, y as f64, &inner);
        if (is_in && n_in >= qi) || (!is_in && n_out >= qo) { continue; }
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
    let mut max_m = 0.0f32;
    for j in 0..rh { for i in 0..rw { let (a, b) = g(i, j); max_m = max_m.max(a.hypot(b)); } }
    let thr = p.edge_min_grad.max(0.1 * max_m);
    let mut ecand: Vec<(f32, usize, usize, [f64; 2])> = Vec::new();
    for j in 1..rh.saturating_sub(1) {
        for i in 1..rw.saturating_sub(1) {
            let (x, y) = (x0 + i, y0 + j);
            if !in_rect(x as f64, y as f64, &edge_region) { continue; }
            let (a, b) = g(i, j);
            let mg = a.hypot(b);
            if mg < thr { continue; }
            // neighbours across the edge (gradient direction quantised to 45 deg)
            let (ax, ay) = (a.abs(), b.abs());
            let (di, dj): (isize, isize) = if ax > 2.414 * ay { (1, 0) } else if ay > 2.414 * ax { (0, 1) } else if (a > 0.0) == (b > 0.0) { (1, 1) } else { (1, -1) };
            let m1 = { let (a, b) = g((i as isize + di) as usize, (j as isize + dj) as usize); a.hypot(b) };
            let m2 = { let (a, b) = g((i as isize - di) as usize, (j as isize - dj) as usize); a.hypot(b) };
            if mg < m1 || mg < m2 { continue; }
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
    let mut scores = vec![f32::NEG_INFINITY; n + 1];
    for (i, s) in scores.iter_mut().enumerate() {
        let x = src.at_inv_depth(&d, w0 + i as f64 * dw);
        let mut sum = 0.0f32;
        let mut ok = true;
        for v in targets {
            match v.cam.project(&x).and_then(|q| { let h = to_half(q); ncc(&v.half, tc, h[0], h[1], false) }) {
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
    if reference >= views.len() || views.len() < 2 { return rec; }
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
            // tolerance: EDGE_CONSISTENCY_M, or EDGE_CONSISTENCY_PX of disparity at the shortest pair baseline
            let z = kinematics::norm(&kinematics::sub(&x0, &rv.cam.c));
            let b = (1..mcams.len()).map(|k| kinematics::norm(&kinematics::sub(&mcams[k].c, &mcams[0].c))).fold(f64::INFINITY, f64::min);
            let tol = EDGE_CONSISTENCY_M.max(z * z * EDGE_CONSISTENCY_PX / (rv.cam.fx * b.max(1e-3)));
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
    let obj: Vec<&Point> = points.iter().filter(|q| q.in_box && q.x[2] - p.table_z_m > p.object_min_h_m).collect();
    if obj.len() < p.min_object_points { return Err("too_few_points"); }
    // isolated points (false matches): distance to the ISOLATION_K-th nearest object point above
    // max(3 x its median, ISOLATION_MIN_M)
    let kth: Vec<f64> = obj.iter().map(|q| {
        let mut d: Vec<f64> = obj.iter().map(|o| kinematics::norm(&kinematics::sub(&o.x, &q.x))).collect();
        let k = ISOLATION_K.min(d.len() - 1);
        d.select_nth_unstable_by(k, f64::total_cmp);
        d[k]
    }).collect();
    let mut sorted = kth.clone();
    sorted.sort_by(f64::total_cmp);
    let lim = (3.0 * quantile(&sorted, 0.5)).max(ISOLATION_MIN_M);
    let obj: Vec<&Point> = obj.into_iter().zip(&kth).filter(|(_, d)| **d <= lim).map(|(q, _)| q).collect();
    if obj.len() < p.min_object_points { return Err("too_few_points"); }
    let top_pts: Vec<&&Point> = obj.iter().filter(|q| q.consistent).collect();
    if top_pts.len() < MIN_TOP_POINTS { return Err("too_few_points"); }
    // top: p90 of the heights after a local median (each point's height = median of its
    // TOP_NEIGHBOURS nearest top points in x, y), so the per-point noise does not lift the p90
    let mut z: Vec<f64> = top_pts.iter().map(|q| {
        let mut nb: Vec<(f64, f64)> = top_pts.iter().map(|o| ((o.x[0] - q.x[0]).hypot(o.x[1] - q.x[1]), o.x[2])).collect();
        let k = TOP_NEIGHBOURS.min(nb.len());
        nb.select_nth_unstable_by(k - 1, |a, b| a.0.total_cmp(&b.0));
        let mut zs: Vec<f64> = nb[..k].iter().map(|n| n.1).collect();
        zs.sort_by(f64::total_cmp);
        quantile(&zs, 0.5)
    }).collect();
    z.sort_by(f64::total_cmp);
    let top = quantile(&z, 0.9);
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
    Ok(ObjectMeasure {
        centroid_m: [centre[0], centre[1], 0.5 * (top + p.table_z_m)],
        top_z_m: top,
        height_m: top - p.table_z_m,
        width_m: v1 - v0,
        length_m: u1 - u0,
        yaw_deg: yaw,
        n_points: obj.len(),
        n_edge_points: obj.iter().filter(|q| q.edge).count(),
        n_top_points: top_pts.len(),
        reproj_rms_px: rms,
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
}
