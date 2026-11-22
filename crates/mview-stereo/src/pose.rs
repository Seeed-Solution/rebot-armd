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

#[derive(Debug, Clone, Default)]
pub struct PoseRefinement {
    /// The cameras to reconstruct with (kinematic ones for views not refined).
    pub cams: Vec<Camera>,
    pub views: Vec<ViewRefine>,
    pub n_corrs: usize,
    pub n_marker_corrs: usize,
    pub ms: f64,
}

fn inside_any(x: f64, y: f64, rects: &[[f64; 4]]) -> bool { rects.iter().any(|r| in_rect(x, y, r)) }

/// Coarse candidates of `t` in `img` (half resolution): the local maxima of the NCC on the
/// integer grid within `r` px of `c` with at least [`POSE_COARSE_MIN`], best first (at most 4).
fn window_peaks(img: &Gray, t: &Tmpl, c: [f64; 2], r: i32) -> Vec<[f64; 2]> {
    let n = (2 * r + 1) as usize;
    let (cx, cy) = (c[0].round(), c[1].round());
    let mut s = vec![f32::NEG_INFINITY; n * n];
    for j in 0..n {
        for i in 0..n {
            if let Some(v) = ncc(img, t, cx + i as f64 - r as f64, cy + j as f64 - r as f64, false) { s[j * n + i] = v; }
        }
    }
    let mut peaks: Vec<(f32, [f64; 2])> = Vec::new();
    for j in 0..n as i32 {
        for i in 0..n as i32 {
            let v = s[j as usize * n + i as usize];
            if v.is_nan() || f64::from(v) < POSE_COARSE_MIN { continue; }
            let local = (-1..=1).all(|dj| (-1..=1).all(|di| {
                let (a, b) = (i + di, j + dj);
                (di == 0 && dj == 0) || a < 0 || b < 0 || a >= n as i32 || b >= n as i32 || s[b as usize * n + a as usize] <= v
            }));
            if local { peaks.push((v, [cx + f64::from(i - r), cy + f64::from(j - r)])); }
        }
    }
    peaks.sort_by(|a, b| b.0.total_cmp(&a.0));
    peaks.into_iter().take(4).map(|p| p.1).collect()
}

/// 2D match of the full-resolution patch `tf` (half-resolution `tc`) of one image in `full` /
/// `half` of another, in a window of `win` px around `q0` (full resolution): the coarse peaks
/// refined at full resolution; the best needs [`POSE_NCC_MIN`] and must pass the ratio test
/// against the best other match more than 2 px away.
fn match_2d(full: &Gray, half: &Gray, tc: &Tmpl, tf: &Tmpl, q0: [f64; 2], win: f64) -> Option<[f64; 2]> {
    let mut m: Vec<([f64; 2], f64)> = window_peaks(half, tc, to_half(q0), (win / 2.0).ceil() as i32).into_iter()
        .filter_map(|h| refine_2d(full, tf, [2.0 * h[0] + 0.5, 2.0 * h[1] + 0.5]))
        .filter(|(q, _)| (q[0] - q0[0]).abs() <= win + 2.0 && (q[1] - q0[1]).abs() <= win + 2.0).collect();
    m.sort_by(|a, b| b.1.total_cmp(&a.1));
    let (q, best) = *m.first()?;
    if best < POSE_NCC_MIN { return None; }
    let second = m.iter().filter(|(p, _)| (p[0] - q[0]).hypot(p[1] - q[1]) > 2.0).map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    if second.is_finite() && (1.0 - best) > POSE_RATIO * (1.0 - second) { return None; }
    Some(q)
}

struct PreparedFeat {
    px: [f64; 2],
    x0: Vec3,
    tc: Tmpl,
    tf: Tmpl,
}

struct TexturePrep {
    reference: usize,
    halves: Vec<Gray>,
    feats: Vec<PreparedFeat>,
}

impl TexturePrep {
    fn new(views: &[View], ref_cam: &Camera, reference: usize, object_box: [f64; 4], exclude: &[[f64; 4]], table_z: f64) -> Self {
        if reference >= views.len() || views.len() < 2 {
            return Self { reference, halves: Vec::new(), feats: Vec::new() };
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
        let raw_feats: Vec<Feature> = features(&rv.img, full, grown, [0.0; 4], &p).into_iter()
            .filter(|f| !inside_any(f.x as f64, f.y as f64, &skip)).take(POSE_MAX_FEATURES).collect();
        let halves: Vec<Gray> = views.iter().map(|v| v.img.half()).collect();
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
        Self { reference, halves, feats }
    }

    fn corrs(&self, views: &[View], cams: &[Camera], win: &[f64]) -> Vec<Corr> {
        if self.reference >= views.len() || views.len() < 2 || cams.len() != views.len() || win.len() != views.len() || self.halves.len() != views.len() {
            return Vec::new();
        }
        let rv = &views[self.reference];
        let mut out = Vec::new();
        for f in &self.feats {
            let px = f.px;
            let mut obs = vec![(self.reference, px)];
            for (j, v) in views.iter().enumerate() {
                if j == self.reference { continue; }
                let Some(q0) = cams[j].project(&f.x0) else { continue; };
                let Some(q) = match_2d(&v.img, &self.halves[j], &f.tc, &f.tf, q0, win[j]) else { continue; };
                // left-right: this view's patch searched back in the reference around the feature
                let hq = to_half(q);
                let back = sample_bilinear(&self.halves[j], hq[0], hq[1], COARSE_R).and_then(|tv| make_tmpl(tv, COARSE_R, None, 0.0))
                    .zip(sample_bilinear(&v.img, q[0], q[1], PATCH_R).and_then(|tv| make_tmpl(tv, PATCH_R, None, 0.0)))
                    .and_then(|(bc, bf)| match_2d(&rv.img, &self.halves[self.reference], &bc, &bf, px, win[j]));
                if back.is_some_and(|b| (b[0] - px[0]).hypot(b[1] - px[1]) <= POSE_LR_TOL_PX) { obs.push((j, q)); }
            }
            if obs.len() >= 2 { out.push(Corr { obs, on_table: true, marker: false }); }
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

/// Solve the dense n x n system `a x = b` (Gaussian elimination, partial pivoting).
fn solve_dense(mut a: Vec<f64>, mut b: Vec<f64>, n: usize) -> Option<Vec<f64>> {
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
        let nf = free.iter().filter(|f| f.is_some()).count();
        let nc = 3 * nf;
        let mut lambda = 1e-3;
        let mut cost = self.cost(s, free);
        for _ in 0..LM_ITERS {
            let mut sm = vec![0.0; nc * nc];
            let mut bc = vec![0.0; nc];
            // per point: Hpp^-1, gp, and its blocks with the free views
            let mut per: Vec<Option<(Mat3, Vec3, Vec<(usize, Mat3)>)>> = Vec::with_capacity(self.corrs.len());
            for (i, c) in self.corrs.iter().enumerate() {
                let wt = self.weight(c);
                let mut hpp = [[0.0; 3]; 3];
                let mut gp = [0.0; 3];
                let mut hpc: Vec<(usize, Mat3)> = Vec::new();
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
                        match hpc.iter_mut().find(|e| e.0 == fi) {
                            Some(e) => for u in 0..3 { for v in 0..3 { e.1[u][v] += b[u][v]; } },
                            None => hpc.push((fi, b)),
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
                let ip = 1.0 / (self.prior_rad * self.prior_rad);
                for u in 0..3 {
                    sm[(3 * fi + u) * nc + 3 * fi + u] += ip;
                    bc[3 * fi + u] += ip * s.w[a][u];
                }
            }
            for u in 0..nc { sm[u * nc + u] *= 1.0 + lambda; }
            let dc = if nc > 0 { match solve_dense(sm, bc.iter().map(|v| -v).collect(), nc) { Some(d) => d, None => break } } else { Vec::new() };
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
            let tc = self.cost(&trial, free);
            if tc < cost {
                let gain = (cost - tc) / cost.max(1e-12);
                *s = trial;
                cost = tc;
                lambda = (lambda * 0.3).max(1e-7);
                if gain < 1e-7 { break; }
            } else {
                lambda *= 10.0;
                if lambda > 1e6 { break; }
            }
        }
    }

    /// Reprojection rms (px) per view over the active observations.
    fn rms(&self, s: &State, nv: usize) -> Vec<Option<f64>> {
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

/// Initial points: the triangulation of the observations (the table plane for a point that
/// does not triangulate).
fn init_points(cams: &[Camera], corrs: &[Corr], table_z: f64) -> Vec<Vec3> {
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
    let t0 = Instant::now();
    let cams: Vec<Camera> = views.iter().map(|v| v.cam).collect();
    let with = |mut c: Vec<Corr>| { c.extend_from_slice(extra); c };
    let prep = texture.then(|| match cams.get(reference) {
        Some(rc) => TexturePrep::new(views, rc, reference, object_box, exclude, table_z),
        None => TexturePrep { reference, halves: Vec::new(), feats: Vec::new() },
    });
    let first = match &prep {
        Some(p) => with(p.corrs(views, &cams, &vec![pp.window_px; views.len()])),
        None => extra.to_vec(),
    };
    let r1 = refine_poses(&cams, reference, &first, table_z, pp);
    let mut out = match &prep {
        Some(p) => {
            let win: Vec<f64> = r1.views.iter().map(|v| if v.status == "refined" { pp.guided_window_px } else { pp.window_px }).collect();
            let second = with(p.corrs(views, &r1.cams, &win));
            refine_poses(&cams, reference, &second, table_z, pp)
        }
        None => r1,
    };
    out.ms = ms(t0);
    out
}

/// Refine the rotations of every view but `reference` from `corrs` (see the module doc).
pub fn refine_poses(cams: &[Camera], reference: usize, corrs: &[Corr], table_z: f64, pp: &PoseParams) -> PoseRefinement {
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
    let fresh = || State { cams: cams.to_vec(), w: vec![[0.0; 3]; nv], x: init_points(cams, corrs, table_z) };
    let mut st = fresh();
    let mut rejected = vec![[0.0; 3]; nv];
    // round 0: solve and drop the outliers; then solve until no free view exceeds the cap
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
    // before: the same observations triangulated with the kinematic cameras (no coplanarity)
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
    PoseRefinement { cams: st.cams, views, n_corrs: corrs.len(), n_marker_corrs: corrs.iter().filter(|c| c.marker).count(), ms: ms(t0) }
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
}
