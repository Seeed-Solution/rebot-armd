//! Motion-stereo geometry of the action `measure_multiview` (src/measure_multiview.rs): object
//! points triangulated from grayscale images taken at known camera poses. Pure computation, no
//! I/O; the action runs it in `spawn_blocking`.
//!
//! - Features: Shi-Tomasi corners (minimum eigenvalue of the 5x5 structure tensor) in a region of
//!   the reference view, spread by a minimum distance, with separate quotas inside the object box
//!   and in the margin around it (table).
//! - Matching: for each feature the ray through it is swept in inverse depth between the planes
//!   `table_z - DEPTH_BELOW_TABLE_M` and `table_z + max_height_m`; each depth projects into every
//!   other view (the epipolar segment the known poses give) and scores the mean zero-normalised
//!   cross-correlation (NCC) of a 9x9 patch. The best depth must score `sweep_min` and pass a ratio
//!   test against the best separate peak (`(1 - best) <= ratio * (1 - second)`). Each other view
//!   then refines its match in 2D (sub-pixel, free of the epipolar constraint, so calibration
//!   errors show in the reprojection error; NCC at least `ncc_min`) and passes a left-right check: its patch swept back
//!   along its own ray must land within `lr_tol_px` of the feature in the reference view.
//! - Triangulation: least-squares midpoint of the rays of all views that matched; points with a
//!   reprojection error above `reproj_max_px` in any view are dropped.
//! - Measurement ([`measure`]): points more than `object_min_h_m` above the table plane whose
//!   reference pixel lies in the object box are object points; the top is the p90 of their
//!   locally median-filtered heights, the footprint their oriented extent (PCA of x, y, 2nd to
//!   98th percentile). Points within [-0.05, `object_min_h_m`] m of
//!   the plane are table points (`table_check`).
use crate::kinematics::{self, Mat3, Vec3};

/// Patch radius (9x9 patches).
pub const PATCH_R: usize = 4;
const PATCH_N: usize = (2 * PATCH_R + 1) * (2 * PATCH_R + 1);
/// The ray sweep starts this far below the table plane (m).
pub const DEPTH_BELOW_TABLE_M: f64 = 0.03;
/// Neighbours (the point included) of the local median before the top percentile.
pub const TOP_NEIGHBOURS: usize = 9;
/// Table points: at most this far below the plane (m).
pub const TABLE_BAND_BELOW_M: f64 = 0.05;

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

    /// The patch centred at `(x, y)` fits with a margin of `m` pixels.
    #[inline]
    fn inside(&self, x: f64, y: f64, m: f64) -> bool {
        let r = PATCH_R as f64 + m;
        x >= r && y >= r && x <= self.w as f64 - 1.0 - r && y <= self.h as f64 - 1.0 - r
    }
}

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
}

#[derive(Debug, Clone)]
pub struct View {
    pub cam: Camera,
    pub img: Gray,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Params {
    pub table_z_m: f64,
    /// Upper bound of the sweep above the table (m).
    pub max_height_m: f64,
    pub max_object_features: usize,
    pub max_table_features: usize,
    /// Minimum distance between features (px).
    pub min_feature_dist_px: f64,
    /// Mean NCC the best depth of a ray sweep needs (integer-pixel patches on the epipolar line).
    pub sweep_min: f64,
    /// NCC each view's sub-pixel 2D match needs.
    pub ncc_min: f64,
    pub ratio: f64,
    pub lr_tol_px: f64,
    pub reproj_max_px: f64,
    /// Sweep step: at most this many pixels of image motion in any view per depth sample.
    pub sweep_step_px: f64,
    /// Height above the table that makes a point an object point (m).
    pub object_min_h_m: f64,
    pub min_object_points: usize,
}

impl Params {
    pub fn new(table_z_m: f64) -> Self {
        Self {
            table_z_m, max_height_m: 0.25, max_object_features: 300, max_table_features: 150, min_feature_dist_px: 5.0,
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
}

/// Counts of the matching stages (diagnostics).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize)]
pub struct Stats {
    pub features: usize,
    pub no_peak: usize,
    pub ambiguous: usize,
    pub lr_rejected: usize,
    pub reproj_rejected: usize,
    pub points: usize,
}

#[derive(Debug, Clone, Default)]
pub struct Reconstruction {
    pub points: Vec<Point>,
    pub stats: Stats,
}

// ---- patches ----

/// Zero-mean, unit-norm patch; None when flat.
fn normalise(v: &mut [f32; PATCH_N]) -> Option<()> {
    let mean = v.iter().sum::<f32>() / PATCH_N as f32;
    let mut ss = 0.0f32;
    for x in v.iter_mut() { *x -= mean; ss += *x * *x; }
    // a standard deviation below 2 grey levels carries no texture
    if ss < 4.0 * PATCH_N as f32 { return None; }
    let inv = 1.0 / ss.sqrt();
    for x in v.iter_mut() { *x *= inv; }
    Some(())
}

fn template_int(img: &Gray, x: usize, y: usize) -> Option<[f32; PATCH_N]> {
    let mut t = [0.0f32; PATCH_N];
    let mut k = 0;
    for yy in y - PATCH_R..=y + PATCH_R {
        for xx in x - PATCH_R..=x + PATCH_R { t[k] = img.at(xx, yy); k += 1; }
    }
    normalise(&mut t).map(|_| t)
}

fn template_bilinear(img: &Gray, x: f64, y: f64) -> Option<[f32; PATCH_N]> {
    if !img.inside(x, y, 1.0) { return None; }
    let mut t = [0.0f32; PATCH_N];
    let r = PATCH_R as f64;
    let mut k = 0;
    for j in 0..=2 * PATCH_R {
        for i in 0..=2 * PATCH_R { t[k] = img.bilinear(x - r + i as f64, y - r + j as f64); k += 1; }
    }
    normalise(&mut t).map(|_| t)
}

/// NCC of template `t` with the patch of `img` at the integer pixel nearest `(x, y)`.
#[inline]
fn ncc_int(img: &Gray, t: &[f32; PATCH_N], x: f64, y: f64) -> Option<f32> {
    if !img.inside(x, y, 0.5) { return None; }
    let (cx, cy) = (x.round() as usize, y.round() as usize);
    let (mut s1, mut s2, mut dot) = (0.0f32, 0.0f32, 0.0f32);
    let mut k = 0;
    for yy in cy - PATCH_R..=cy + PATCH_R {
        let row = &img.px[yy * img.w + cx - PATCH_R..=yy * img.w + cx + PATCH_R];
        for &v in row {
            let v = f32::from(v);
            s1 += v; s2 += v * v; dot += v * t[k];
            k += 1;
        }
    }
    let var = s2 - s1 * s1 / PATCH_N as f32;
    Some(if var > 1.0 { dot / var.sqrt() } else { 0.0 })
}

/// NCC with the bilinearly sampled patch at `(x, y)`.
fn ncc_bilinear(img: &Gray, t: &[f32; PATCH_N], x: f64, y: f64) -> Option<f32> {
    if !img.inside(x, y, 1.0) { return None; }
    let r = PATCH_R as f64;
    let (mut s1, mut s2, mut dot) = (0.0f32, 0.0f32, 0.0f32);
    let mut k = 0;
    for j in 0..=2 * PATCH_R {
        for i in 0..=2 * PATCH_R {
            let v = img.bilinear(x - r + i as f64, y - r + j as f64);
            s1 += v; s2 += v * v; dot += v * t[k];
            k += 1;
        }
    }
    let var = s2 - s1 * s1 / PATCH_N as f32;
    Some(if var > 1.0 { dot / var.sqrt() } else { 0.0 })
}

// ---- features ----

/// Shi-Tomasi corners in `region` = [x0, y0, x1, y1] (px), strongest first, at least
/// `min_dist` apart, at most `quota_in` inside `inner` and `quota_out` outside it.
pub fn corners(img: &Gray, region: [f64; 4], inner: [f64; 4], quota_in: usize, quota_out: usize, min_dist: f64) -> Vec<[usize; 2]> {
    let m = PATCH_R + 4;
    let x0 = (region[0].floor().max(0.0) as usize).max(m);
    let y0 = (region[1].floor().max(0.0) as usize).max(m);
    let x1 = (region[2].ceil().max(0.0) as usize).min(img.w.saturating_sub(m + 1));
    let y1 = (region[3].ceil().max(0.0) as usize).min(img.h.saturating_sub(m + 1));
    if x1 <= x0 + 2 || y1 <= y0 + 2 { return Vec::new(); }
    // gradients over the region plus a 2 px border (structure tensor window 5x5)
    let (gx0, gy0) = (x0 - 2, y0 - 2);
    let (gw, gh) = (x1 - x0 + 5, y1 - y0 + 5);
    let mut ixx = vec![0.0f32; gw * gh];
    let mut iyy = vec![0.0f32; gw * gh];
    let mut ixy = vec![0.0f32; gw * gh];
    for j in 0..gh {
        let y = gy0 + j;
        for i in 0..gw {
            let x = gx0 + i;
            let gx = 0.5 * (img.at(x + 1, y) - img.at(x - 1, y));
            let gy = 0.5 * (img.at(x, y + 1) - img.at(x, y - 1));
            let k = j * gw + i;
            ixx[k] = gx * gx; iyy[k] = gy * gy; ixy[k] = gx * gy;
        }
    }
    let (rw, rh) = (x1 - x0 + 1, y1 - y0 + 1);
    let mut lam = vec![0.0f32; rw * rh];
    let mut max_l = 0.0f32;
    for j in 0..rh {
        for i in 0..rw {
            let (mut a, mut b, mut c) = (0.0f32, 0.0f32, 0.0f32);
            for dj in 0..5 {
                let base = (j + dj) * gw + i;
                for di in 0..5 { a += ixx[base + di]; c += iyy[base + di]; b += ixy[base + di]; }
            }
            let half = 0.5 * (a + c);
            let l = half - (0.25 * (a - c) * (a - c) + b * b).sqrt();
            lam[j * rw + i] = l;
            max_l = max_l.max(l);
        }
    }
    // relative and absolute floors (a 5x5 window of 2-grey-level gradients is no corner)
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
    let inside = |x: usize, y: usize| { let (x, y) = (x as f64, y as f64); x >= inner[0] && x <= inner[2] && y >= inner[1] && y <= inner[3] };
    let mut out: Vec<[usize; 2]> = Vec::new();
    let (mut n_in, mut n_out) = (0, 0);
    let d2 = min_dist * min_dist;
    for (_, x, y) in cand {
        let is_in = inside(x, y);
        if (is_in && n_in >= quota_in) || (!is_in && n_out >= quota_out) { continue; }
        if out.iter().any(|p| { let (dx, dy) = (p[0] as f64 - x as f64, p[1] as f64 - y as f64); dx * dx + dy * dy < d2 }) { continue; }
        out.push([x, y]);
        if is_in { n_in += 1 } else { n_out += 1 }
        if n_in >= quota_in && n_out >= quota_out { break; }
    }
    out
}

// ---- ray sweep ----

struct Sweep {
    /// Inverse optical depth of the best match (1/m).
    w: f64,
}

enum SweepEnd { NoPeak, Ambiguous }

/// Sweep the ray of `px` in `src` against `targets` (see the module doc).
fn sweep(src: &Camera, px: [f64; 2], t: &[f32; PATCH_N], targets: &[&View], p: &Params) -> Result<Sweep, SweepEnd> {
    let d = src.ray(px);
    if d[2] > -1e-3 { return Err(SweepEnd::NoPeak); }
    let z_far = (p.table_z_m - DEPTH_BELOW_TABLE_M - src.c[2]) / d[2];
    let z_near = ((p.table_z_m + p.max_height_m - src.c[2]) / d[2]).max(0.05);
    if z_far.is_nan() || z_near.is_nan() || z_far <= z_near { return Err(SweepEnd::NoPeak); }
    let (w0, w1) = (1.0 / z_far, 1.0 / z_near);
    let point = |w: f64| -> Vec3 { [src.c[0] + d[0] / w, src.c[1] + d[1] / w, src.c[2] + d[2] / w] };
    let (pf, pn) = (point(w0), point(w1));
    let mut span: f64 = 0.0;
    for v in targets {
        match (v.cam.project(&pf), v.cam.project(&pn)) {
            (Some(a), Some(b)) => span = span.max((a[0] - b[0]).hypot(a[1] - b[1])),
            _ => span = span.max(800.0),
        }
    }
    let n = ((span / p.sweep_step_px).ceil() as usize).clamp(2, 1200);
    let dw = (w1 - w0) / n as f64;
    let mut scores = vec![f32::NEG_INFINITY; n + 1];
    for (i, s) in scores.iter_mut().enumerate() {
        let x = point(w0 + i as f64 * dw);
        let mut sum = 0.0f32;
        let mut ok = true;
        for v in targets {
            match v.cam.project(&x).and_then(|q| ncc_int(&v.img, t, q[0], q[1])) {
                Some(c) => sum += c,
                None => { ok = false; break; }
            }
        }
        if ok { *s = sum / targets.len() as f32; }
    }
    let (ib, &best) = scores.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).ok_or(SweepEnd::NoPeak)?;
    if f64::from(best) < p.sweep_min { return Err(SweepEnd::NoPeak); }
    // best separate peak: local maxima at least 2 px of image motion from the best
    let excl = (2.0 / p.sweep_step_px).ceil() as usize;
    let mut second = f32::NEG_INFINITY;
    for i in 0..=n {
        if i.abs_diff(ib) <= excl || !scores[i].is_finite() { continue; }
        let l = if i > 0 { scores[i - 1] } else { f32::NEG_INFINITY };
        let r = if i < n { scores[i + 1] } else { f32::NEG_INFINITY };
        if scores[i] >= l && scores[i] >= r { second = second.max(scores[i]); }
    }
    if second.is_finite() && (1.0 - f64::from(best)) > p.ratio * (1.0 - f64::from(second)) { return Err(SweepEnd::Ambiguous); }
    // sub-sample: bilinear scores at quarter steps around the best, parabola through the top three
    let score_at = |w: f64| -> f32 {
        let x = point(w);
        let mut sum = 0.0f32;
        for v in targets {
            match v.cam.project(&x).and_then(|q| ncc_bilinear(&v.img, t, q[0], q[1])) {
                Some(c) => sum += c,
                None => return f32::NEG_INFINITY,
            }
        }
        sum / targets.len() as f32
    };
    let wb = w0 + ib as f64 * dw;
    let q = dw / 4.0;
    let fine: Vec<(f64, f32)> = (-4..=4).map(|k| { let w = wb + f64::from(k) * q; (w, score_at(w)) }).collect();
    let (k, _) = fine.iter().enumerate().max_by(|a, b| a.1.1.total_cmp(&b.1.1)).ok_or(SweepEnd::NoPeak)?;
    let mut w = fine[k].0;
    if k > 0 && k + 1 < fine.len() {
        let (a, b, c) = (f64::from(fine[k - 1].1), f64::from(fine[k].1), f64::from(fine[k + 1].1));
        let den = a - 2.0 * b + c;
        if a.is_finite() && c.is_finite() && den < -1e-9 { w += 0.5 * (a - c) / den * q; }
    }
    Ok(Sweep { w: w.clamp(w0, w1) })
}

/// 2D sub-pixel match of `t` in `img` near `q`: NCC on a 0.5 px grid (±1 px, moved while the
/// best is on its edge), then a parabola per axis. Returns the pixel and its NCC.
fn refine_2d(img: &Gray, t: &[f32; PATCH_N], mut q: [f64; 2]) -> Option<([f64; 2], f64)> {
    const H: f64 = 0.5;
    for _ in 0..4 {
        let mut g = [[f32::NEG_INFINITY; 5]; 5];
        let mut best = (2usize, 2usize, f32::NEG_INFINITY);
        for (j, row) in g.iter_mut().enumerate() {
            for (i, cell) in row.iter_mut().enumerate() {
                let (x, y) = (q[0] + (i as f64 - 2.0) * H, q[1] + (j as f64 - 2.0) * H);
                *cell = ncc_bilinear(img, t, x, y)?;
                if *cell > best.2 { best = (i, j, *cell); }
            }
        }
        let (i, j, s) = best;
        if i == 0 || i == 4 || j == 0 || j == 4 {
            q = [q[0] + (i as f64 - 2.0) * H, q[1] + (j as f64 - 2.0) * H];
            continue;
        }
        let para = |a: f32, b: f32, c: f32| -> f64 {
            let (a, b, c) = (f64::from(a), f64::from(b), f64::from(c));
            let den = a - 2.0 * b + c;
            if den < -1e-9 { (0.5 * (a - c) / den).clamp(-0.5, 0.5) } else { 0.0 }
        };
        let dx = para(g[j][i - 1], g[j][i], g[j][i + 1]);
        let dy = para(g[j - 1][i], g[j][i], g[j + 1][i]);
        return Some(([q[0] + (i as f64 - 2.0 + dx) * H, q[1] + (j as f64 - 2.0 + dy) * H], f64::from(s)));
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

/// Features of `views[reference]` in `region` (px), matched in every other view and
/// triangulated. `object_box` (px) splits the feature quotas and marks [`Point::in_box`].
pub fn reconstruct(views: &[View], reference: usize, region: [f64; 4], object_box: [f64; 4], p: &Params) -> Reconstruction {
    reconstruct_perturbed(views, reference, region, object_box, p, |_, q| q)
}

/// [`reconstruct`] with `perturb(view index, pixel)` applied to every matched pixel before the
/// triangulation (the tests add localisation noise with it).
pub fn reconstruct_perturbed(views: &[View], reference: usize, region: [f64; 4], object_box: [f64; 4], p: &Params,
    mut perturb: impl FnMut(usize, [f64; 2]) -> [f64; 2]) -> Reconstruction {
    let mut rec = Reconstruction::default();
    let Some(rv) = views.get(reference) else { return rec; };
    let other_idx: Vec<usize> = (0..views.len()).filter(|i| *i != reference).collect();
    let others: Vec<&View> = other_idx.iter().map(|&i| &views[i]).collect();
    if others.is_empty() { return rec; }
    let feats = corners(&rv.img, region, object_box, p.max_object_features, p.max_table_features, p.min_feature_dist_px);
    rec.stats.features = feats.len();
    for f in feats {
        let px = [f[0] as f64, f[1] as f64];
        let Some(t) = template_int(&rv.img, f[0], f[1]) else { rec.stats.no_peak += 1; continue; };
        let s = match sweep(&rv.cam, px, &t, &others, p) {
            Ok(s) => s,
            Err(SweepEnd::NoPeak) => { rec.stats.no_peak += 1; continue; }
            Err(SweepEnd::Ambiguous) => { rec.stats.ambiguous += 1; continue; }
        };
        let d = rv.cam.ray(px);
        let x0 = [rv.cam.c[0] + d[0] / s.w, rv.cam.c[1] + d[1] / s.w, rv.cam.c[2] + d[2] / s.w];
        let mut cams = vec![rv.cam];
        let mut pxs = vec![perturb(reference, px)];
        let mut lr_fail = false;
        for (&vi, v) in other_idx.iter().zip(&others) {
            let Some(q0) = v.cam.project(&x0) else { lr_fail = true; break; };
            let Some((q, score)) = refine_2d(&v.img, &t, q0) else { lr_fail = true; break; };
            if score < p.ncc_min { lr_fail = true; break; }
            // left-right: this view's patch swept back along its own ray into the reference
            let back = template_bilinear(&v.img, q[0], q[1])
                .and_then(|tb| sweep(&v.cam, q, &tb, &[rv], p).ok())
                .and_then(|sb| { let db = v.cam.ray(q); rv.cam.project(&[v.cam.c[0] + db[0] / sb.w, v.cam.c[1] + db[1] / sb.w, v.cam.c[2] + db[2] / sb.w]) });
            match back {
                Some(b) if (b[0] - px[0]).hypot(b[1] - px[1]) <= p.lr_tol_px => {}
                _ => { lr_fail = true; break; }
            }
            cams.push(v.cam);
            pxs.push(perturb(vi, q));
        }
        if lr_fail { rec.stats.lr_rejected += 1; continue; }
        match triangulate(&cams, &pxs) {
            Some((x, rms, mx)) if mx <= p.reproj_max_px => {
                let in_box = px[0] >= object_box[0] && px[0] <= object_box[2] && px[1] >= object_box[1] && px[1] <= object_box[3];
                rec.points.push(Point { x, ref_px: px, reproj_rms_px: rms, views: cams.len(), in_box });
            }
            _ => rec.stats.reproj_rejected += 1,
        }
    }
    rec.stats.points = rec.points.len();
    rec
}

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
/// below `min_object_points`.
pub fn measure(points: &[Point], p: &Params) -> Result<ObjectMeasure, &'static str> {
    let obj: Vec<&Point> = points.iter().filter(|q| q.in_box && q.x[2] - p.table_z_m > p.object_min_h_m).collect();
    if obj.len() < p.min_object_points { return Err("too_few_points"); }
    // top: p90 of the heights after a local median (each point's height = median of its
    // TOP_NEIGHBOURS nearest object points in x, y), so the per-point noise does not lift the p90
    let mut z: Vec<f64> = obj.iter().map(|q| {
        let mut nb: Vec<(f64, f64)> = obj.iter().map(|o| ((o.x[0] - q.x[0]).hypot(o.x[1] - q.x[1]), o.x[2])).collect();
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

    fn pt(x: f64, y: f64, z: f64, in_box: bool) -> Point { Point { x: [x, y, z], ref_px: [0.0; 2], reproj_rms_px: 0.3, views: 3, in_box } }

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
        assert_eq!(m.n_points, 66);
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
    fn corners_respect_quotas_and_distance() {
        // checkerboard of 8 px squares: corners at the square junctions
        let (w, h) = (160, 120);
        let px: Vec<u8> = (0..w * h).map(|k| { let (x, y) = (k % w, k / w); if ((x / 8) + (y / 8)) % 2 == 0 { 40 } else { 210 } }).collect();
        let img = Gray::new(w, h, px).unwrap();
        let f = corners(&img, [0.0, 0.0, 159.0, 119.0], [40.0, 40.0, 100.0, 80.0], 5, 3, 6.0);
        assert_eq!(f.len(), 8, "{f:?}");
        let inside = f.iter().filter(|p| p[0] >= 40 && p[0] <= 100 && p[1] >= 40 && p[1] <= 80).count();
        assert_eq!(inside, 5);
        for (i, a) in f.iter().enumerate() {
            for b in &f[i + 1..] { assert!((a[0] as f64 - b[0] as f64).hypot(a[1] as f64 - b[1] as f64) >= 6.0); }
        }
        let flat = Gray::new(w, h, vec![128; w * h]).unwrap();
        assert!(corners(&flat, [0.0, 0.0, 159.0, 119.0], [0.0; 4], 10, 10, 5.0).is_empty());
    }
}
