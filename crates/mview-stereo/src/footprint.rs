//! Object footprint from per-view silhouettes (measure_multiview): each view's object outline (a
//! segmentation-model contour or a colour mask, image pixels) is projected through that view's
//! camera onto the table plane raised to half the stereo height, rasterised on a 1 mm grid of that
//! plane, and the views that agree are fused by strict majority. The footprint is the
//! minimum-area rectangle of the fused region's convex hull: length and width are straight spans
//! (not the length along a curved object). The stereo height is an input and is not changed.
//!
//! - Projection: ray C + t r of each vertex meets n . X + d = h_p at
//!   t = (h_p - d - n . C) / (n . r); a vertex whose ray is backward, grazing (more than
//!   `max_ray_angle_deg` from the plane normal) or not finite rejects that view's silhouette.
//! - Mid-height is a shape hypothesis: it bounds the height mismatch of a boundary anywhere in
//!   [0, h] by h / 2; the lateral error of a ray at angle theta from the normal is |dh| tan(theta).
//! - Occlusion: each view may carry image rectangles it cannot see through (the gripper jaws);
//!   grid cells under them are not observed by that view and do not vote there (a cell no view
//!   observes takes the majority of all views).
//! - Agreement: pairwise IoU over the cells both views observe; the largest group of at least
//!   `min_views` views with every pair at `min_iou` or more is kept (ties: the higher mean IoU).
//! - Fusion: a cell is occupied when more than half of the kept views that observe it see it
//!   (both of two views).
//! - Grades ([`Grade`]): a dimension is `Medium` when the spread of the kept views' own spans along
//!   the rectangle's axis is at most `tolerance` of their median and the height sensitivity (a
//!   boundary anywhere in [0, h]: h / 2 tan(theta) along the side at each end, per view, largest)
//!   is at most `tolerance` of it; else `Low`. Quality grades, not calibrated probabilities.
//! - Yaw: the long side's direction in the base frame, modulo 180 deg in [-90, 90); indeterminate
//!   (`yaw_determinate` false) when the sides differ by less than `square_ratio`.
use crate::kinematics::{self, Vec3};
use crate::Camera;

/// Where a silhouette came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaskSource {
    /// The segmentation model's contour of the selected detection.
    Seg,
    /// A colour mask extracted in armd ([`classical_silhouette`]).
    Classical,
}

impl MaskSource {
    pub fn name(self) -> &'static str { match self { Self::Seg => "seg_mask", Self::Classical => "classical_mask" } }
}

/// One view's object outline.
#[derive(Debug, Clone)]
pub struct Silhouette {
    /// Index of the view (camera) it belongs to.
    pub view: usize,
    /// Closed polygon in image pixels (no repeated last vertex).
    pub poly_px: Vec<[f64; 2]>,
    pub source: MaskSource,
    /// Image rectangles [x0, y0, x1, y1] (px) this view cannot see through.
    pub occluders_px: Vec<[f64; 4]>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FootprintParams {
    /// Grid cell (m).
    pub cell_m: f64,
    /// Largest grid side (m): a larger projected region is refused.
    pub max_extent_m: f64,
    pub min_views: usize,
    pub min_iou: f64,
    /// Largest ray angle from the plane normal (deg).
    pub max_ray_angle_deg: f64,
    /// Relative tolerance of the grades.
    pub tolerance: f64,
    /// Sides differing by less than this fraction of the long side: yaw indeterminate.
    pub square_ratio: f64,
}

impl Default for FootprintParams {
    fn default() -> Self {
        Self { cell_m: 0.001, max_extent_m: 0.5, min_views: 2, min_iou: 0.6, max_ray_angle_deg: 70.0, tolerance: 0.10, square_ratio: 0.10 }
    }
}

/// Quality grade of a footprint dimension.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Grade { Low, Medium }

impl Grade {
    pub fn name(self) -> &'static str { match self { Self::Low => "low", Self::Medium => "medium" } }
}

/// The fused footprint.
#[derive(Debug, Clone, PartialEq)]
pub struct Footprint {
    pub length_m: f64,
    pub width_m: f64,
    /// Rectangle centre on the half-height plane, and the same x, y on the full-height plane (z).
    pub centroid_m: Vec3,
    pub top_z_m: f64,
    pub yaw_deg: f64,
    pub yaw_determinate: bool,
    /// Views fused (indices), and their sources.
    pub views: Vec<usize>,
    /// `seg_mask`, `classical_mask` or `mixed_mask`.
    pub source: &'static str,
    pub length_confidence: Grade,
    pub width_confidence: Grade,
    /// Spans of each fused view's own projection along the long and short side (m).
    pub view_spans_m: Vec<[f64; 2]>,
    /// Largest error of length and width (m) of a boundary anywhere in [0, h] instead of at h / 2:
    /// h / 2 tan(theta) along the side at each end, of the fused view with the largest.
    pub height_sensitivity_m: [f64; 2],
    /// Largest ray angle from the plane normal of the fused views' vertices (deg).
    pub max_ray_angle_deg: f64,
    /// Silhouettes not used: (view, reason).
    pub rejected: Vec<(usize, String)>,
}

/// An orthonormal frame of the table plane: unit normal `n` (up), offset `d` (height = n . x + d),
/// in-plane axes `e1` (base x projected) and `e2` = n x e1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneFrame {
    pub n: Vec3,
    pub d: f64,
    pub e1: Vec3,
    pub e2: Vec3,
}

impl PlaneFrame {
    /// `plane` = [n_x, n_y, n_z, d]; None when the normal is not finite, zero or not up.
    pub fn new(plane: [f64; 4]) -> Option<Self> {
        let raw = [plane[0], plane[1], plane[2]];
        let len = kinematics::norm(&raw);
        if !len.is_finite() || len < 1e-9 || !plane[3].is_finite() { return None; }
        let n = [raw[0] / len, raw[1] / len, raw[2] / len];
        if n[2] <= 0.1 { return None; }
        let d = plane[3] / len;
        let x = [1.0 - n[0] * n[0], -n[0] * n[1], -n[0] * n[2]];
        let e1 = kinematics::unit(&x);
        let e2 = kinematics::cross(&n, &e1);
        Some(Self { n, d, e1, e2 })
    }

    /// The plane z = `z`.
    pub fn horizontal(z: f64) -> Self { Self::new([0.0, 0.0, 1.0, -z]).expect("horizontal plane") }

    /// In-plane coordinates of a point.
    pub fn uv(&self, x: &Vec3) -> [f64; 2] { [kinematics::dot(x, &self.e1), kinematics::dot(x, &self.e2)] }

    /// The point at in-plane coordinates `uv` and height `h` above the plane.
    pub fn point(&self, uv: [f64; 2], h: f64) -> Vec3 {
        let s = h - self.d;
        [uv[0] * self.e1[0] + uv[1] * self.e2[0] + s * self.n[0], uv[0] * self.e1[1] + uv[1] * self.e2[1] + s * self.n[1], uv[0] * self.e1[2] + uv[1] * self.e2[2] + s * self.n[2]]
    }

    /// The point of the plane raised by `h` at base x, y.
    pub fn at_xy(&self, x: f64, y: f64, h: f64) -> Vec3 { [x, y, (h - self.d - self.n[0] * x - self.n[1] * y) / self.n[2]] }

    /// Base-frame yaw (deg) of the in-plane direction at angle `phi` (rad) from `e1`.
    pub fn yaw_deg(&self, phi: f64) -> f64 {
        let (c, s) = (phi.cos(), phi.sin());
        let dir = [c * self.e1[0] + s * self.e2[0], c * self.e1[1] + s * self.e2[1]];
        wrap_yaw_deg(dir[1].atan2(dir[0]).to_degrees())
    }
}

/// `deg` modulo 180 in [-90, 90).
pub fn wrap_yaw_deg(deg: f64) -> f64 {
    let y = (deg + 90.0).rem_euclid(180.0) - 90.0;
    if y >= 90.0 { y - 180.0 } else { y }
}

/// Intersection of the ray of pixel `px` with the plane raised by `h`; also the ray's angle from
/// the normal (deg). Err for a backward, grazing or non-finite intersection.
pub fn ray_plane(cam: &Camera, px: [f64; 2], frame: &PlaneFrame, h: f64, max_angle_deg: f64) -> Result<(Vec3, f64), &'static str> {
    if !px[0].is_finite() || !px[1].is_finite() { return Err("nonfinite_vertex"); }
    let r = cam.ray(px);
    let rn = kinematics::norm(&r);
    let nr = kinematics::dot(&frame.n, &r);
    if !rn.is_finite() || rn < 1e-12 || !nr.is_finite() { return Err("nonfinite_ray"); }
    // the ray goes down towards the plane: n . r < 0
    let cos = -nr / rn;
    let angle = cos.clamp(-1.0, 1.0).acos().to_degrees();
    if cos <= 0.0 { return Err("backward_ray"); }
    if angle > max_angle_deg { return Err("grazing_ray"); }
    let t = (h - frame.d - kinematics::dot(&frame.n, &cam.c)) / nr;
    if !t.is_finite() { return Err("nonfinite_ray"); }
    if t <= 0.0 { return Err("plane_behind_camera"); }
    Ok(([cam.c[0] + t * r[0], cam.c[1] + t * r[1], cam.c[2] + t * r[2]], angle))
}

/// A polygon of image pixels projected onto the plane raised by `h` (in-plane coordinates), and
/// the largest ray angle.
pub fn project_polygon(cam: &Camera, poly_px: &[[f64; 2]], frame: &PlaneFrame, h: f64, max_angle_deg: f64) -> Result<(Vec<[f64; 2]>, f64), &'static str> {
    let mut out = Vec::with_capacity(poly_px.len());
    let mut amax: f64 = 0.0;
    for p in poly_px {
        let (x, a) = ray_plane(cam, *p, frame, h, max_angle_deg)?;
        amax = amax.max(a);
        out.push(frame.uv(&x));
    }
    Ok((out, amax))
}

/// Twice the signed area of a polygon.
pub fn polygon_area2(poly: &[[f64; 2]]) -> f64 {
    let n = poly.len();
    (0..n).map(|i| { let (a, b) = (poly[i], poly[(i + 1) % n]); a[0] * b[1] - b[0] * a[1] }).sum()
}

/// Checks a model or extracted contour: at least 3 finite vertices, at most `max_vertices`, a
/// non-zero area.
pub fn validate_polygon(poly: &[[f64; 2]], max_vertices: usize) -> Result<(), &'static str> {
    if poly.len() < 3 { return Err("contour_too_few_vertices"); }
    if poly.len() > max_vertices { return Err("contour_too_many_vertices"); }
    if poly.iter().any(|p| !p[0].is_finite() || !p[1].is_finite()) { return Err("contour_nonfinite"); }
    if polygon_area2(poly).abs() < 1e-9 { return Err("contour_zero_area"); }
    Ok(())
}

/// A bounded grid on the plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grid {
    pub u0: f64,
    pub v0: f64,
    pub cell: f64,
    pub nu: usize,
    pub nv: usize,
}

impl Grid {
    /// Covering all `polys` with a one-cell margin; None when empty or larger than `max_extent`.
    pub fn covering(polys: &[&[[f64; 2]]], cell: f64, max_extent: f64) -> Option<Self> {
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for p in polys.iter().flat_map(|p| p.iter()) {
            for k in 0..2 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); }
        }
        if !(lo[0].is_finite() && hi[0].is_finite() && lo[1].is_finite() && hi[1].is_finite()) || cell <= 0.0 { return None; }
        if hi[0] - lo[0] > max_extent || hi[1] - lo[1] > max_extent { return None; }
        let u0 = (lo[0] / cell).floor() * cell - cell;
        let v0 = (lo[1] / cell).floor() * cell - cell;
        let nu = ((hi[0] - u0) / cell).ceil() as usize + 2;
        let nv = ((hi[1] - v0) / cell).ceil() as usize + 2;
        Some(Self { u0, v0, cell, nu, nv })
    }

    /// Cells whose centre is inside `poly` (even-odd), as a row-major mask.
    pub fn rasterize(&self, poly: &[[f64; 2]]) -> Vec<bool> {
        let mut m = vec![false; self.nu * self.nv];
        self.fill(poly, &mut m);
        m
    }

    /// Sets the cells whose centre is inside `poly` (even-odd).
    pub fn fill(&self, poly: &[[f64; 2]], m: &mut [bool]) {
        let n = poly.len();
        if n < 3 { return; }
        let mut xs: Vec<f64> = Vec::with_capacity(16);
        for j in 0..self.nv {
            let v = self.v0 + (j as f64 + 0.5) * self.cell;
            xs.clear();
            for i in 0..n {
                let (a, b) = (poly[i], poly[(i + 1) % n]);
                if (a[1] <= v) != (b[1] <= v) {
                    xs.push(a[0] + (v - a[1]) / (b[1] - a[1]) * (b[0] - a[0]));
                }
            }
            xs.sort_by(f64::total_cmp);
            for pair in xs.chunks_exact(2) {
                // cells with centre u in [pair0, pair1)
                let i0 = ((pair[0] - self.u0) / self.cell - 0.5).ceil().max(0.0) as usize;
                let i1 = ((pair[1] - self.u0) / self.cell - 0.5).ceil().max(0.0) as usize;
                for i in i0..i1.min(self.nu) { m[j * self.nu + i] = true; }
            }
        }
    }

    /// Centre of cell (i, j).
    pub fn centre(&self, i: usize, j: usize) -> [f64; 2] { [self.u0 + (i as f64 + 0.5) * self.cell, self.v0 + (j as f64 + 0.5) * self.cell] }
}

/// Convex hull (counter-clockwise, Andrew's monotone chain); collinear points dropped.
pub fn convex_hull(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut p: Vec<[f64; 2]> = points.iter().copied().filter(|q| q[0].is_finite() && q[1].is_finite()).collect();
    p.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    p.dedup();
    if p.len() < 3 { return p; }
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut h: Vec<[f64; 2]> = Vec::with_capacity(2 * p.len());
    for &q in &p {
        while h.len() >= 2 && cross(h[h.len() - 2], h[h.len() - 1], q) <= 0.0 { h.pop(); }
        h.push(q);
    }
    let lower = h.len() + 1;
    for &q in p.iter().rev().skip(1) {
        while h.len() >= lower && cross(h[h.len() - 2], h[h.len() - 1], q) <= 0.0 { h.pop(); }
        h.push(q);
    }
    h.pop();
    h
}

/// A rectangle in the plane: centre, long and short side, long side's angle from `e1` (rad).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub centre: [f64; 2],
    pub long: f64,
    pub short: f64,
    pub angle: f64,
}

impl Rect {
    pub fn axes(&self) -> ([f64; 2], [f64; 2]) {
        let (c, s) = (self.angle.cos(), self.angle.sin());
        ([c, s], [-s, c])
    }
}

/// Minimum-area enclosing rectangle of a convex hull (one side on a hull edge).
pub fn min_area_rect(hull: &[[f64; 2]]) -> Option<Rect> {
    if hull.len() < 3 { return None; }
    let mut best: Option<(f64, Rect)> = None;
    for i in 0..hull.len() {
        let (a, b) = (hull[i], hull[(i + 1) % hull.len()]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = dx.hypot(dy);
        if len < 1e-12 { continue; }
        let (ux, uy) = (dx / len, dy / len);
        let (mut s0, mut s1, mut t0, mut t1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for p in hull {
            let s = p[0] * ux + p[1] * uy;
            let t = -p[0] * uy + p[1] * ux;
            s0 = s0.min(s); s1 = s1.max(s); t0 = t0.min(t); t1 = t1.max(t);
        }
        let area = (s1 - s0) * (t1 - t0);
        let (sc, tc) = (0.5 * (s0 + s1), 0.5 * (t0 + t1));
        let centre = [sc * ux - tc * uy, sc * uy + tc * ux];
        let (ds, dt) = (s1 - s0, t1 - t0);
        let angle = uy.atan2(ux);
        let r = if ds >= dt { Rect { centre, long: ds, short: dt, angle } } else { Rect { centre, long: dt, short: ds, angle: angle + std::f64::consts::FRAC_PI_2 } };
        if best.as_ref().is_none_or(|(a0, _)| area < *a0 - 1e-15) { best = Some((area, r)); }
    }
    best.map(|(_, r)| r)
}

/// Hull points of a cell mask: the centres of the first and last occupied cell of each row.
fn mask_outline_points(g: &Grid, m: &[bool]) -> Vec<[f64; 2]> {
    let mut pts = Vec::new();
    for j in 0..g.nv {
        let row = &m[j * g.nu..(j + 1) * g.nu];
        let (Some(i0), Some(i1)) = (row.iter().position(|x| *x), row.iter().rposition(|x| *x)) else { continue };
        pts.push(g.centre(i0, j));
        pts.push(g.centre(i1, j));
    }
    pts
}

/// Minimum-area rectangle of a cell mask: of the occupied cells' centres, grown by one cell (the
/// centres of a span of length L cover L - cell on average).
fn mask_rect(g: &Grid, m: &[bool]) -> Option<Rect> {
    let mut r = min_area_rect(&convex_hull(&mask_outline_points(g, m)))?;
    r.long += g.cell;
    r.short += g.cell;
    Some(r)
}

/// Projections of the silhouettes (and their occluders) at plane height `h`.
struct Projected {
    poly: Vec<[f64; 2]>,
    occl: Vec<Vec<[f64; 2]>>,
    angle: f64,
}

fn project_all(cams: &[Camera], sils: &[&Silhouette], frame: &PlaneFrame, h: f64, p: &FootprintParams) -> Vec<Result<Projected, &'static str>> {
    sils.iter().map(|s| {
        let cam = cams.get(s.view).ok_or("no_camera")?;
        let (poly, angle) = project_polygon(cam, &s.poly_px, frame, h, p.max_ray_angle_deg)?;
        // an occluder whose corners do not meet the plane hides nothing measurable here
        let occl = s.occluders_px.iter().filter_map(|r| {
            let corners = [[r[0], r[1]], [r[2], r[1]], [r[2], r[3]], [r[0], r[3]]];
            project_polygon(cam, &corners, frame, h, 89.0).ok().map(|(q, _)| q)
        }).collect();
        Ok(Projected { poly, occl, angle })
    }).collect()
}

/// Rasterised silhouettes: occupied and observed masks per view.
struct Masks {
    grid: Grid,
    occ: Vec<Vec<bool>>,
    obs: Vec<Vec<bool>>,
}

fn rasterize_all(proj: &[&Projected], p: &FootprintParams) -> Option<Masks> {
    let polys: Vec<&[[f64; 2]]> = proj.iter().map(|q| q.poly.as_slice()).collect();
    let grid = Grid::covering(&polys, p.cell_m, p.max_extent_m)?;
    let occ = proj.iter().map(|q| grid.rasterize(&q.poly)).collect();
    let obs = proj.iter().map(|q| {
        let mut hidden = vec![false; grid.nu * grid.nv];
        for o in &q.occl { grid.fill(o, &mut hidden); }
        hidden.into_iter().map(|x| !x).collect()
    }).collect();
    Some(Masks { grid, occ, obs })
}

/// IoU of two views' masks over the cells both observe.
fn iou(m: &Masks, a: usize, b: usize) -> f64 {
    let (mut inter, mut uni) = (0usize, 0usize);
    for k in 0..m.occ[a].len() {
        if !(m.obs[a][k] && m.obs[b][k]) { continue; }
        let (x, y) = (m.occ[a][k], m.occ[b][k]);
        inter += usize::from(x && y);
        uni += usize::from(x || y);
    }
    if uni == 0 { 0.0 } else { inter as f64 / uni as f64 }
}

/// Strict-majority fusion of the views `keep` over the views that observe each cell; a cell no
/// view observes (inside every view's occluder rectangle) takes the majority of all of them: the
/// rectangle is coarser than the occluder, and a mask stops at the occluder's real edge.
fn fuse(m: &Masks, keep: &[usize]) -> Vec<bool> {
    (0..m.grid.nu * m.grid.nv).map(|k| {
        let (mut votes, mut seen, mut all) = (0usize, 0usize, 0usize);
        for &i in keep {
            all += usize::from(m.occ[i][k]);
            if m.obs[i][k] { seen += 1; votes += usize::from(m.occ[i][k]); }
        }
        if seen > 0 { 2 * votes > seen } else { 2 * all > keep.len() }
    }).collect()
}

/// The largest group of views with every pair at `min_iou` or more; ties: higher mean IoU.
fn agreeing_group(ious: &[Vec<f64>], min_views: usize, min_iou: f64) -> Option<Vec<usize>> {
    let n = ious.len();
    if n > 16 { return None; }
    let mut best: Option<(usize, f64, Vec<usize>)> = None;
    for set in 1u32..(1 << n) {
        let idx: Vec<usize> = (0..n).filter(|i| set & (1 << i) != 0).collect();
        if idx.len() < min_views.max(1) { continue; }
        let mut ok = true;
        let (mut sum, mut cnt) = (0.0, 0usize);
        'pairs: for (a, &i) in idx.iter().enumerate() {
            for &j in &idx[a + 1..] {
                if ious[i][j] < min_iou { ok = false; break 'pairs; }
                sum += ious[i][j];
                cnt += 1;
            }
        }
        if !ok { continue; }
        let mean = if cnt > 0 { sum / cnt as f64 } else { 1.0 };
        let better = best.as_ref().is_none_or(|(k, m, _)| idx.len() > *k || (idx.len() == *k && mean > *m + 1e-12));
        if better { best = Some((idx.len(), mean, idx)); }
    }
    best.map(|b| b.2)
}

/// The footprint from `sils` (cameras by view index; table plane [n, d] with height = n . x + d)
/// and the stereo height `height_m`. Err(reason) when fewer than `min_views` silhouettes project,
/// none agree, or the fused region is empty.
pub fn footprint(cams: &[Camera], sils: &[Silhouette], plane: [f64; 4], height_m: f64, p: &FootprintParams) -> Result<Footprint, String> {
    let frame = PlaneFrame::new(plane).ok_or("invalid_table_plane")?;
    if !(height_m.is_finite() && height_m > 0.0) { return Err("invalid_height".into()); }
    let mut rejected: Vec<(usize, String)> = Vec::new();
    let all: Vec<&Silhouette> = sils.iter().collect();
    let mid = project_all(cams, &all, &frame, 0.5 * height_m, p);
    let mut ok_sils: Vec<&Silhouette> = Vec::new();
    let mut ok_proj: Vec<Projected> = Vec::new();
    for (s, r) in sils.iter().zip(mid) {
        match r {
            Ok(q) => { ok_sils.push(s); ok_proj.push(q); }
            Err(e) => rejected.push((s.view, e.to_string())),
        }
    }
    if ok_proj.len() < p.min_views {
        return Err(format!("{} silhouettes projected (minimum {})", ok_proj.len(), p.min_views));
    }
    let refs: Vec<&Projected> = ok_proj.iter().collect();
    let m = rasterize_all(&refs, p).ok_or("footprint_too_large")?;
    let n = refs.len();
    let mut ious = vec![vec![1.0; n]; n];
    for a in 0..n { for b in a + 1..n { let v = iou(&m, a, b); ious[a][b] = v; ious[b][a] = v; } }
    let keep = agreeing_group(&ious, p.min_views, p.min_iou)
        .ok_or_else(|| format!("no {} silhouettes agree (IoU >= {})", p.min_views, p.min_iou))?;
    for (k, s) in ok_sils.iter().enumerate() {
        if !keep.contains(&k) { rejected.push((s.view, "disagrees".into())); }
    }
    let fused = fuse(&m, &keep);
    let rect = mask_rect(&m.grid, &fused).ok_or("empty_fused_footprint")?;
    if rect.long <= 0.0 || rect.short <= 0.0 { return Err("empty_fused_footprint".into()); }
    let (a_long, a_short) = rect.axes();
    let view_spans_m: Vec<[f64; 2]> = keep.iter().map(|&k| {
        let span = |ax: [f64; 2]| {
            let s: Vec<f64> = refs[k].poly.iter().map(|q| q[0] * ax[0] + q[1] * ax[1]).collect();
            s.iter().copied().fold(f64::NEG_INFINITY, f64::max) - s.iter().copied().fold(f64::INFINITY, f64::min)
        };
        [span(a_long), span(a_short)]
    }).collect();
    // height sensitivity: a boundary point anywhere in [0, h] lands up to h/2 tan(theta) from its
    // mid-height projection along the ray's in-plane direction; per side the rays at both ends
    let kept_sils: Vec<&Silhouette> = keep.iter().map(|&k| ok_sils[k]).collect();
    let mut sens = [0.0f64; 2];
    for s in &kept_sils {
        let Some(cam) = cams.get(s.view) else { continue };
        for (k, (ax, half)) in [(a_long, 0.5 * rect.long), (a_short, 0.5 * rect.short)].into_iter().enumerate() {
            let mut e = 0.0;
            for sign in [-1.0, 1.0] {
                let uv = [rect.centre[0] + sign * half * ax[0], rect.centre[1] + sign * half * ax[1]];
                let x = frame.point(uv, 0.5 * height_m);
                let r = kinematics::sub(&x, &cam.c);
                let down = -kinematics::dot(&r, &frame.n);
                if down <= 1e-9 { e = f64::INFINITY; break; }
                // in-plane displacement per unit height along the ray
                let g = [kinematics::dot(&r, &frame.e1) / down, kinematics::dot(&r, &frame.e2) / down];
                e += 0.5 * height_m * (g[0] * ax[0] + g[1] * ax[1]).abs();
            }
            sens[k] = sens[k].max(e);
        }
    }
    let grade = |k: usize, dim: f64| {
        let mut s: Vec<f64> = view_spans_m.iter().map(|v| v[k]).collect();
        s.sort_by(f64::total_cmp);
        let med = crate::quantile(&s, 0.5);
        let spread = s.last().copied().unwrap_or(0.0) - s.first().copied().unwrap_or(0.0);
        if spread <= p.tolerance * med && sens[k] <= p.tolerance * dim { Grade::Medium } else { Grade::Low }
    };
    let length_confidence = grade(0, rect.long);
    let width_confidence = grade(1, rect.short);
    let centroid = frame.point(rect.centre, 0.5 * height_m);
    let top = frame.at_xy(centroid[0], centroid[1], height_m);
    let srcs: Vec<MaskSource> = kept_sils.iter().map(|s| s.source).collect();
    let source = if srcs.iter().all(|s| *s == MaskSource::Seg) { "seg_mask" } else if srcs.iter().all(|s| *s == MaskSource::Classical) { "classical_mask" } else { "mixed_mask" };
    Ok(Footprint {
        length_m: rect.long,
        width_m: rect.short,
        centroid_m: centroid,
        top_z_m: top[2],
        yaw_deg: frame.yaw_deg(rect.angle),
        yaw_determinate: rect.long - rect.short >= p.square_ratio * rect.long,
        views: kept_sils.iter().map(|s| s.view).collect(),
        source,
        length_confidence,
        width_confidence,
        view_spans_m,
        height_sensitivity_m: sens,
        max_ray_angle_deg: keep.iter().map(|&k| refs[k].angle).fold(0.0, f64::max),
        rejected,
    })
}

// ---- contours of binary masks ----

/// Exterior contour of the largest 4-connected component of a binary mask (`w` x `h`, row-major,
/// non-zero = set), traced along pixel edges: vertices at pixel corners (x, y in pixel units, the
/// mask's top-left corner at 0, 0), clockwise in image coordinates, corners only. Empty when the
/// mask is empty.
pub fn trace_exterior(mask: &[u8], w: usize, h: usize) -> Vec<[f64; 2]> {
    let Some(lab) = largest_component(mask, w, h) else { return Vec::new() };
    trace_component(&lab, w, h)
}

/// The largest 4-connected component as a 0/1 mask, and the sizes of the two largest.
pub fn components(mask: &[u8], w: usize, h: usize) -> (Vec<u8>, usize, usize) {
    let mut label = vec![0u32; w * h];
    let mut sizes: Vec<usize> = vec![0];
    let mut stack: Vec<usize> = Vec::new();
    for s in 0..w * h {
        if mask[s] == 0 || label[s] != 0 { continue; }
        let id = sizes.len() as u32;
        let mut n = 0;
        label[s] = id;
        stack.push(s);
        while let Some(k) = stack.pop() {
            n += 1;
            let (x, y) = (k % w, k / w);
            let mut visit = |q: usize| { if mask[q] != 0 && label[q] == 0 { label[q] = id; stack.push(q); } };
            if x > 0 { visit(k - 1); }
            if x + 1 < w { visit(k + 1); }
            if y > 0 { visit(k - w); }
            if y + 1 < h { visit(k + w); }
        }
        sizes.push(n);
    }
    let Some((best, &n1)) = sizes.iter().enumerate().skip(1).max_by_key(|(_, n)| **n) else { return (vec![0; w * h], 0, 0) };
    let n2 = sizes.iter().enumerate().skip(1).filter(|(i, _)| *i != best).map(|(_, n)| *n).max().unwrap_or(0);
    (label.iter().map(|l| u8::from(*l as usize == best)).collect(), n1, n2)
}

fn largest_component(mask: &[u8], w: usize, h: usize) -> Option<Vec<u8>> {
    if w == 0 || h == 0 || mask.len() < w * h { return None; }
    let (lab, n1, _) = components(mask, w, h);
    (n1 > 0).then_some(lab)
}

/// Crack following around a single 4-connected component (0/1 mask).
fn trace_component(m: &[u8], w: usize, h: usize) -> Vec<[f64; 2]> {
    let at = |x: i64, y: i64| x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h && m[y as usize * w + x as usize] != 0;
    let Some(first) = m.iter().position(|v| *v != 0) else { return Vec::new() };
    let (sx, sy) = ((first % w) as i64, (first / w) as i64);
    // directions: 0 right, 1 down, 2 left, 3 up (image y down); the component stays on the right
    const D: [(i64, i64); 4] = [(1, 0), (0, 1), (-1, 0), (0, -1)];
    let ahead = |x: i64, y: i64, d: usize| -> (bool, bool) {
        // (front-left, front-right) pixels of vertex (x, y) facing d
        match d {
            0 => (at(x, y - 1), at(x, y)),
            1 => (at(x, y), at(x - 1, y)),
            2 => (at(x - 1, y), at(x - 1, y - 1)),
            _ => (at(x - 1, y - 1), at(x, y - 1)),
        }
    };
    let (mut x, mut y, mut d) = (sx, sy, 0usize);
    let mut out: Vec<[f64; 2]> = Vec::new();
    let limit = 4 * (w + 1) * (h + 1) + 8;
    for _ in 0..limit {
        let (fl, fr) = ahead(x, y, d);
        let nd = if !fr { (d + 1) % 4 } else if fl { (d + 3) % 4 } else { d };
        if nd != d { out.push([x as f64, y as f64]); }
        d = nd;
        if !fr || fl {
            // turned: the step is taken from the same vertex in the new direction next round
            if x == sx && y == sy && d == 0 && !out.is_empty() { break; }
            continue;
        }
        x += D[d].0;
        y += D[d].1;
        if x == sx && y == sy && d == 0 { break; }
    }
    // the start vertex is a corner (top-left of the first pixel): make it the first vertex
    if let Some(k) = out.iter().position(|p| p[0] == sx as f64 && p[1] == sy as f64) { out.rotate_left(k); }
    out.dedup();
    if out.len() > 1 && out.first() == out.last() { out.pop(); }
    out
}

/// Douglas-Peucker simplification of a closed polygon to at most `max_vertices` vertices: the
/// tolerance starts at `eps` and doubles until it fits. The extreme vertices are kept.
pub fn simplify_closed(poly: &[[f64; 2]], eps: f64, max_vertices: usize) -> Vec<[f64; 2]> {
    if poly.len() <= 3 || max_vertices < 3 { return poly.to_vec(); }
    // split at the vertex farthest from the first one
    let far = (1..poly.len()).max_by(|&a, &b| d2(poly[0], poly[a]).total_cmp(&d2(poly[0], poly[b]))).unwrap_or(1);
    let mut e = eps.max(1e-9);
    loop {
        let mut keep = vec![false; poly.len()];
        keep[0] = true;
        keep[far] = true;
        dp(poly, 0, far, e, &mut keep);
        dp_wrap(poly, far, e, &mut keep);
        let out: Vec<[f64; 2]> = poly.iter().zip(&keep).filter(|(_, k)| **k).map(|(p, _)| *p).collect();
        if out.len() <= max_vertices { return out; }
        e *= 2.0;
    }
}

fn d2(a: [f64; 2], b: [f64; 2]) -> f64 { (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) }

fn seg_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    if l2 < 1e-18 { return d2(p, a).sqrt(); }
    let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0);
    d2(p, [a[0] + t * dx, a[1] + t * dy]).sqrt()
}

fn dp(poly: &[[f64; 2]], i: usize, j: usize, eps: f64, keep: &mut [bool]) {
    if j <= i + 1 { return; }
    let (mut best, mut bi) = (-1.0, i);
    for k in i + 1..j {
        let dd = seg_dist(poly[k], poly[i], poly[j]);
        if dd > best { best = dd; bi = k; }
    }
    if best > eps {
        keep[bi] = true;
        dp(poly, i, bi, eps, keep);
        dp(poly, bi, j, eps, keep);
    }
}

/// [`dp`] over the closing chain far .. n-1, 0.
fn dp_wrap(poly: &[[f64; 2]], far: usize, eps: f64, keep: &mut [bool]) {
    let mut chain: Vec<[f64; 2]> = poly[far..].to_vec();
    chain.push(poly[0]);
    let mut k2 = vec![false; chain.len()];
    dp(&chain, 0, chain.len() - 1, eps, &mut k2);
    for (k, v) in k2.iter().enumerate() { if *v && far + k < poly.len() { keep[far + k] = true; } }
}

// ---- classical colour mask ----

/// An RGB image (8 bits per channel, packed, row-major).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Rgb {
    pub w: usize,
    pub h: usize,
    pub px: Vec<u8>,
}

/// Bins per channel of the background colour histogram.
const HIST_BINS: usize = 16;

/// Parameters of [`classical_silhouette`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClassicalParams {
    /// Background ring outside the box: from `ring_gap_px` to `ring_gap_px` + max(`ring_min_px`,
    /// `ring_frac` x the box's short side).
    pub ring_gap_px: f64,
    pub ring_min_px: f64,
    pub ring_frac: f64,
    /// A colour is background when the ring holds at least this fraction of its pixels in the
    /// colour's bin and its 26 neighbours (and at least 3 pixels).
    pub bg_frac: f64,
    /// The ring may hold at most this fraction of foreground colours (else low contrast).
    pub max_ring_fg: f64,
    /// The component must cover this fraction of the box.
    pub min_box_frac: f64,
    pub max_box_frac: f64,
    /// A second component of this fraction of the largest makes it ambiguous.
    pub max_second_frac: f64,
    /// A box this close to the image border (px) is clipped.
    pub border_px: f64,
    pub max_vertices: usize,
}

impl Default for ClassicalParams {
    fn default() -> Self {
        Self { ring_gap_px: 4.0, ring_min_px: 8.0, ring_frac: 0.15, bg_frac: 0.002, max_ring_fg: 0.10, min_box_frac: 0.15, max_box_frac: 0.95,
            max_second_frac: 0.5, border_px: 2.0, max_vertices: 128 }
    }
}

/// The object's outline in `img` inside `box_px` [x0, y0, x1, y1] by colour contrast: colours
/// frequent in a ring around the box are background, the rest of the box is foreground; the
/// largest 4-connected foreground component is the object. Err(reason): `clipped` (box at the
/// image border), `low_contrast` (the ring holds foreground colours or too few ring pixels),
/// `ambiguous` (a second large component), `component_size` (outside the box fractions).
pub fn classical_silhouette(img: &Rgb, box_px: [f64; 4], p: &ClassicalParams) -> Result<Vec<[f64; 2]>, &'static str> {
    let (w, h) = (img.w as f64, img.h as f64);
    if img.px.len() != img.w * img.h * 3 || img.w == 0 || img.h == 0 { return Err("bad_image"); }
    if !box_px.iter().all(|v| v.is_finite()) || box_px[2] <= box_px[0] || box_px[3] <= box_px[1] { return Err("bad_box"); }
    if box_px[0] < p.border_px || box_px[1] < p.border_px || box_px[2] > w - p.border_px || box_px[3] > h - p.border_px { return Err("clipped"); }
    let bx0 = box_px[0].floor().max(0.0) as usize;
    let by0 = box_px[1].floor().max(0.0) as usize;
    let bx1 = (box_px[2].ceil() as usize).min(img.w);
    let by1 = (box_px[3].ceil() as usize).min(img.h);
    let (bw, bh) = (bx1 - bx0, by1 - by0);
    if bw < 4 || bh < 4 { return Err("bad_box"); }
    let ring = p.ring_min_px.max(p.ring_frac * bw.min(bh) as f64);
    let g0 = p.ring_gap_px;
    let inner = [box_px[0] - g0, box_px[1] - g0, box_px[2] + g0, box_px[3] + g0];
    let outer = [inner[0] - ring, inner[1] - ring, inner[2] + ring, inner[3] + ring];
    let bin = |k: usize| -> [usize; 3] { let q = &img.px[3 * k..3 * k + 3]; [q[0] as usize * HIST_BINS / 256, q[1] as usize * HIST_BINS / 256, q[2] as usize * HIST_BINS / 256] };
    let mut hist = vec![0u32; HIST_BINS * HIST_BINS * HIST_BINS];
    let idx = |b: [usize; 3]| (b[0] * HIST_BINS + b[1]) * HIST_BINS + b[2];
    let mut ring_px: Vec<usize> = Vec::new();
    let ox0 = outer[0].floor().max(0.0) as usize;
    let oy0 = outer[1].floor().max(0.0) as usize;
    let ox1 = (outer[2].ceil().max(0.0) as usize).min(img.w);
    let oy1 = (outer[3].ceil().max(0.0) as usize).min(img.h);
    for y in oy0..oy1 {
        for x in ox0..ox1 {
            let (fx, fy) = (x as f64 + 0.5, y as f64 + 0.5);
            if fx >= inner[0] && fx < inner[2] && fy >= inner[1] && fy < inner[3] { continue; }
            let k = y * img.w + x;
            hist[idx(bin(k))] += 1;
            ring_px.push(k);
        }
    }
    if ring_px.len() < 200 { return Err("low_contrast"); }
    let thr = (p.bg_frac * ring_px.len() as f64).max(3.0);
    // background support of every bin: its count plus its 26 neighbours'
    let n = HIST_BINS as i64;
    let mut support = vec![0u32; hist.len()];
    for r in 0..n { for g in 0..n { for b in 0..n {
        let mut s = 0;
        for dr in -1..=1 { for dg in -1..=1 { for db in -1..=1 {
            let (rr, gg, bb) = (r + dr, g + dg, b + db);
            if rr < 0 || gg < 0 || bb < 0 || rr >= n || gg >= n || bb >= n { continue; }
            s += hist[idx([rr as usize, gg as usize, bb as usize])];
        }}}
        support[idx([r as usize, g as usize, b as usize])] = s;
    }}}
    let is_fg = |k: usize| f64::from(support[idx(bin(k))]) < thr;
    let mut mask = vec![0u8; bw * bh];
    for y in 0..bh { for x in 0..bw { mask[y * bw + x] = u8::from(is_fg((by0 + y) * img.w + bx0 + x)); } }
    // contrast: colours the box calls foreground must be rare in the ring
    let fg_colours: std::collections::HashSet<usize> = mask.iter().enumerate().filter(|(_, v)| **v != 0)
        .map(|(k, _)| idx(bin((by0 + k / bw) * img.w + bx0 + k % bw))).collect();
    let ring_fg = ring_px.iter().filter(|&&k| fg_colours.contains(&idx(bin(k)))).count();
    if ring_fg as f64 > p.max_ring_fg * ring_px.len() as f64 { return Err("low_contrast"); }
    let (comp, n1, n2) = components(&mask, bw, bh);
    if n1 == 0 { return Err("component_size"); }
    if n2 as f64 >= p.max_second_frac * n1 as f64 { return Err("ambiguous"); }
    let frac = n1 as f64 / (bw * bh) as f64;
    if frac < p.min_box_frac || frac > p.max_box_frac { return Err("component_size"); }
    let poly = trace_component(&comp, bw, bh);
    let poly = simplify_closed(&poly, 0.7, p.max_vertices);
    Ok(poly.into_iter().map(|q| [q[0] + bx0 as f64, q[1] + by0 as f64]).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cam_at(c: Vec3, look: Vec3) -> Camera {
        let z = kinematics::unit(&kinematics::sub(&look, &c));
        let up = if z[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
        let x = kinematics::unit(&kinematics::cross(&z, &up));
        let y = kinematics::cross(&z, &x);
        Camera { r: kinematics::from_columns(&x, &y, &z), c, fx: 500.0, fy: 500.0, cx: 320.0, cy: 180.0 }
    }

    /// Corners of a box (centre, size l x w x h, yaw) on the plane z = 0.
    fn box_corners(c: [f64; 2], l: f64, w: f64, h: f64, yaw: f64) -> Vec<Vec3> {
        let (cs, sn) = (yaw.cos(), yaw.sin());
        let mut out = Vec::new();
        for z in [0.0, h] { for (a, b) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            let (u, v) = (a * 0.5 * l, b * 0.5 * w);
            out.push([c[0] + u * cs - v * sn, c[1] + u * sn + v * cs, z]);
        }}
        out
    }

    /// The image outline of a box: convex hull of its projected corners.
    fn silhouette(cam: &Camera, corners: &[Vec3], view: usize) -> Silhouette {
        let px: Vec<[f64; 2]> = corners.iter().map(|x| cam.project(x).unwrap()).collect();
        Silhouette { view, poly_px: convex_hull(&px), source: MaskSource::Seg, occluders_px: Vec::new() }
    }

    fn views(centre: Vec3, n: usize, dist: f64, tilt_deg: f64) -> Vec<Camera> {
        (0..n).map(|k| {
            let a = k as f64 * std::f64::consts::TAU / n as f64;
            let r = dist * tilt_deg.to_radians().sin();
            cam_at([centre[0] + r * a.cos(), centre[1] + r * a.sin(), centre[2] + dist * tilt_deg.to_radians().cos()], centre)
        }).collect()
    }

    #[test]
    fn ray_plane_hits_raised_and_tilted_planes_and_rejects_bad_rays() {
        let cam = cam_at([0.3, 0.0, 0.3], [0.3, 0.0, 0.0]);
        let f = PlaneFrame::horizontal(0.01);
        let (x, a) = ray_plane(&cam, [320.0, 180.0], &f, 0.02, 70.0).unwrap();
        assert!((x[2] - 0.03).abs() < 1e-12 && (x[0] - 0.3).abs() < 1e-9 && a < 1e-6, "{x:?} {a}");
        // tilted plane: height = n.x + d
        let n = kinematics::unit(&[0.05, 0.0, 1.0]);
        let tf = PlaneFrame::new([n[0], n[1], n[2], -0.01]).unwrap();
        let (x, _) = ray_plane(&cam, [300.0, 150.0], &tf, 0.02, 70.0).unwrap();
        assert!((kinematics::dot(&tf.n, &x) + tf.d - 0.02).abs() < 1e-12);
        // a ray pointing up, a grazing ray, a plane above the camera, NaN
        let up = cam_at([0.3, 0.0, 0.3], [0.3, 0.0, 0.6]);
        assert_eq!(ray_plane(&up, [320.0, 180.0], &f, 0.0, 70.0).unwrap_err(), "backward_ray");
        let graze = cam_at([0.3, 0.0, 0.3], [1.3, 0.0, 0.25]);
        assert_eq!(ray_plane(&graze, [320.0, 180.0], &f, 0.0, 70.0).unwrap_err(), "grazing_ray");
        assert_eq!(ray_plane(&cam, [320.0, 180.0], &f, 0.5, 70.0).unwrap_err(), "plane_behind_camera");
        assert_eq!(ray_plane(&cam, [f64::NAN, 180.0], &f, 0.0, 70.0).unwrap_err(), "nonfinite_vertex");
        assert!(PlaneFrame::new([0.0, 0.0, 0.0, 0.0]).is_none() && PlaneFrame::new([0.0, 0.0, -1.0, 0.0]).is_none());
    }

    #[test]
    fn min_area_rect_and_yaw_range() {
        for yaw in [-1.5, -0.7, 0.0, 0.3, 1.2, 1.5] {
            let c = box_corners([0.1, -0.05], 0.12, 0.04, 0.0, yaw);
            let pts: Vec<[f64; 2]> = c[..4].iter().map(|x| [x[0], x[1]]).collect();
            let r = min_area_rect(&convex_hull(&pts)).unwrap();
            assert!((r.long - 0.12).abs() < 1e-9 && (r.short - 0.04).abs() < 1e-9, "{r:?}");
            assert!((r.centre[0] - 0.1).abs() < 1e-9 && (r.centre[1] + 0.05).abs() < 1e-9);
            let y = PlaneFrame::horizontal(0.0).yaw_deg(r.angle);
            assert!((-90.0..90.0).contains(&y));
            let want = wrap_yaw_deg(yaw.to_degrees());
            let dy = (y - want + 90.0).rem_euclid(180.0) - 90.0;
            assert!(dy.abs() < 1e-6, "yaw {yaw}: {y} vs {want}");
        }
        assert_eq!(wrap_yaw_deg(90.0), -90.0);
        assert_eq!(wrap_yaw_deg(-90.0), -90.0);
        assert!((wrap_yaw_deg(270.5) + 89.5).abs() < 1e-9);
    }

    #[test]
    fn rasterize_matches_area() {
        let g = Grid { u0: -0.1, v0: -0.1, cell: 0.001, nu: 200, nv: 200 };
        let sq = [[-0.025, -0.015], [0.025, -0.015], [0.025, 0.015], [-0.025, 0.015]];
        let n = g.rasterize(&sq).iter().filter(|x| **x).count();
        assert_eq!(n, 50 * 30);
        let tri = [[0.0, 0.0], [0.05, 0.0], [0.0, 0.05]];
        let n = g.rasterize(&tri).iter().filter(|x| **x).count() as f64;
        assert!((n - 1250.0).abs() < 60.0, "{n}");
    }

    #[test]
    fn near_normal_views_recover_the_box() {
        // 50 x 30 mm box, yaw 0.3 rad, five views 10 deg off the normal at 0.30 m; the image
        // outline is the hull of the top and bottom rims
        let (l, w, yaw) = (0.050, 0.030, 0.3);
        let cams = views([0.30, 0.02, 0.0], 5, 0.30, 10.0);
        for h in [0.008, 0.040] {
            let corners = box_corners([0.30, 0.02], l, w, h, yaw);
            let sils: Vec<Silhouette> = cams.iter().enumerate().map(|(k, c)| silhouette(c, &corners, k)).collect();
            let fp = footprint(&cams, &sils, [0.0, 0.0, 1.0, 0.0], h, &FootprintParams::default()).unwrap();
            // vertical walls: the top rim projected to mid height lies outside the box by up to
            // h/2 tan(ray angle) per side, which the height sensitivity bounds
            let tol = [0.003 + fp.height_sensitivity_m[0], 0.003 + fp.height_sensitivity_m[1]];
            assert!(fp.length_m >= l - 0.002 && fp.length_m - l < tol[0] && fp.width_m >= w - 0.002 && fp.width_m - w < tol[1], "h {h}: {fp:?}");
            if h < 0.01 { assert!((fp.length_m - l).abs() < 0.003 && (fp.width_m - w).abs() < 0.003, "{fp:?}"); }
            assert!((fp.centroid_m[0] - 0.30).abs() < 0.003 && (fp.centroid_m[1] - 0.02).abs() < 0.003, "{fp:?}");
            assert!((fp.centroid_m[2] - 0.5 * h).abs() < 1e-9 && (fp.top_z_m - h).abs() < 1e-9);
            let dy = (fp.yaw_deg - yaw.to_degrees() + 90.0).rem_euclid(180.0) - 90.0;
            assert!(dy.abs() < 5.0, "{fp:?}");
            assert_eq!(fp.views, vec![0, 1, 2, 3, 4]);
            assert_eq!(fp.source, "seg_mask");
            assert!(fp.yaw_determinate);
        }
    }

    #[test]
    fn tilted_table_and_outlier_view() {
        let (l, w, h) = (0.060, 0.025, 0.030);
        let n = kinematics::unit(&[0.03, -0.02, 1.0]);
        let frame = PlaneFrame::new([n[0], n[1], n[2], -0.005]).unwrap();
        // the box's mid-height outline on the tilted plane
        let c0 = frame.uv(&frame.at_xy(0.28, -0.03, 0.0));
        let outline: Vec<Vec3> = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]].iter()
            .map(|s| frame.point([c0[0] + s[0] * 0.5 * l, c0[1] + s[1] * 0.5 * w], 0.5 * h)).collect();
        let cams = views([0.28, -0.03, 0.0], 4, 0.32, 15.0);
        let mut sils: Vec<Silhouette> = cams.iter().enumerate().map(|(k, c)| silhouette(c, &outline, k)).collect();
        // view 3 sees something else (shifted by 4 cm)
        for p in &mut sils[3].poly_px { p[0] += 120.0; }
        let fp = footprint(&cams, &sils, [n[0], n[1], n[2], -0.005], h, &FootprintParams::default()).unwrap();
        assert!((fp.length_m - l).abs() < 0.002 && (fp.width_m - w).abs() < 0.002, "{fp:?}");
        assert_eq!(fp.views, vec![0, 1, 2]);
        assert_eq!(fp.rejected, vec![(3, "disagrees".to_string())]);
        let ctr = frame.point(c0, 0.5 * h);
        assert!((fp.centroid_m[0] - ctr[0]).abs() < 0.002 && (fp.centroid_m[1] - ctr[1]).abs() < 0.002, "{fp:?}");
    }

    #[test]
    fn two_views_that_disagree_fail_and_two_that_agree_need_both_votes() {
        let h = 0.02;
        let out: Vec<Vec3> = box_corners([0.3, 0.0], 0.05, 0.03, 0.0, 0.0)[..4].iter().map(|x| [x[0], x[1], 0.5 * h]).collect();
        let cams = views([0.3, 0.0, 0.0], 2, 0.3, 10.0);
        let mut sils: Vec<Silhouette> = cams.iter().enumerate().map(|(k, c)| silhouette(c, &out, k)).collect();
        let ok = footprint(&cams, &sils, [0.0, 0.0, 1.0, 0.0], h, &FootprintParams::default()).unwrap();
        assert!((ok.length_m - 0.05).abs() < 0.002);
        for p in &mut sils[1].poly_px { p[1] += 60.0; }
        let e = footprint(&cams, &sils, [0.0, 0.0, 1.0, 0.0], h, &FootprintParams::default()).unwrap_err();
        assert!(e.contains("agree"), "{e}");
        // one silhouette is not enough
        let e = footprint(&cams, &sils[..1], [0.0, 0.0, 1.0, 0.0], h, &FootprintParams::default()).unwrap_err();
        assert!(e.contains("minimum 2"), "{e}");
    }

    #[test]
    fn occluded_cells_do_not_vote() {
        let h = 0.02;
        let out: Vec<Vec3> = box_corners([0.3, 0.0], 0.08, 0.03, 0.0, 0.0)[..4].iter().map(|x| [x[0], x[1], 0.5 * h]).collect();
        let cams = views([0.3, 0.0, 0.0], 3, 0.3, 10.0);
        let mut sils: Vec<Silhouette> = cams.iter().enumerate().map(|(k, c)| silhouette(c, &out, k)).collect();
        // views 0 and 1 lose the +x end of the box behind an occluder (their outlines cut there)
        for s in sils.iter_mut().take(2) {
            let cam = &cams[s.view];
            let cut = cam.project(&[0.32, 0.0, 0.5 * h]).unwrap();
            let far = cam.project(&[0.34, 0.0, 0.5 * h]).unwrap();
            // image direction of +x; occluder: everything beyond the cut along it (axis-aligned approx)
            let occ = if far[0] > cut[0] + 1.0 { [cut[0], 0.0, 640.0, 360.0] } else if far[0] < cut[0] - 1.0 { [0.0, 0.0, cut[0], 360.0] }
                else if far[1] > cut[1] { [0.0, cut[1], 640.0, 360.0] } else { [0.0, 0.0, 640.0, cut[1]] };
            s.poly_px = s.poly_px.iter().map(|p| {
                let mut q = *p;
                if occ[0] > 0.0 { q[0] = q[0].min(occ[0]); } else if occ[2] < 640.0 { q[0] = q[0].max(occ[2]); }
                else if occ[1] > 0.0 { q[1] = q[1].min(occ[1]); } else { q[1] = q[1].max(occ[3]); }
                q
            }).collect();
            s.occluders_px = vec![occ];
        }
        let fp = footprint(&cams, &sils, [0.0, 0.0, 1.0, 0.0], h, &FootprintParams::default()).unwrap();
        assert!((fp.length_m - 0.08).abs() < 0.004, "{fp:?}");
        // without the occluders the truncated views win the majority
        for s in &mut sils { s.occluders_px.clear(); }
        let fp2 = footprint(&cams, &sils, [0.0, 0.0, 1.0, 0.0], h, &FootprintParams::default()).unwrap();
        assert!(fp2.length_m < 0.075, "{fp2:?}");
    }

    #[test]
    fn grades_follow_view_spread_and_height_sensitivity() {
        let h = 0.01;
        let out: Vec<Vec3> = box_corners([0.3, 0.0], 0.06, 0.03, 0.0, 0.0)[..4].iter().map(|x| [x[0], x[1], 0.5 * h]).collect();
        let cams = views([0.3, 0.0, 0.0], 3, 0.3, 10.0);
        let sils: Vec<Silhouette> = cams.iter().enumerate().map(|(k, c)| silhouette(c, &out, k)).collect();
        let fp = footprint(&cams, &sils, [0.0, 0.0, 1.0, 0.0], h, &FootprintParams::default()).unwrap();
        assert_eq!((fp.length_confidence, fp.width_confidence), (Grade::Medium, Grade::Medium), "{fp:?}");
        // a tall object seen obliquely: the projection height matters
        let tall = 0.2;
        let out: Vec<Vec3> = box_corners([0.3, 0.0], 0.06, 0.03, 0.0, 0.0)[..4].iter().map(|x| [x[0], x[1], 0.5 * tall]).collect();
        let cams = views([0.3, 0.0, 0.0], 3, 0.35, 30.0);
        let sils: Vec<Silhouette> = cams.iter().enumerate().map(|(k, c)| silhouette(c, &out, k)).collect();
        let fp = footprint(&cams, &sils, [0.0, 0.0, 1.0, 0.0], tall, &FootprintParams::default()).unwrap();
        assert_eq!(fp.width_confidence, Grade::Low, "{fp:?}");
        assert!(fp.height_sensitivity_m[1] > 0.1 * fp.width_m);
    }

    #[test]
    fn trace_and_simplify() {
        // an L shape: 3x3 block plus a 1x2 tail, and a separate small blob
        let (w, h) = (6, 5);
        let mut m = vec![0u8; w * h];
        for y in 0..3 { for x in 0..3 { m[y * w + x] = 1; } }
        m[3 * w] = 1;
        m[4 * w] = 1;
        m[4 * w + 5] = 1;
        let c = trace_exterior(&m, w, h);
        assert_eq!(c, vec![[0.0, 0.0], [3.0, 0.0], [3.0, 3.0], [1.0, 3.0], [1.0, 5.0], [0.0, 5.0]]);
        assert!((polygon_area2(&c).abs() * 0.5 - 11.0).abs() < 1e-12);
        // diagonal neighbours are separate components
        let mut d = vec![0u8; 4];
        d[0] = 1; d[3] = 1;
        assert_eq!(trace_exterior(&d, 2, 2).len(), 4);
        assert!(trace_exterior(&[0u8; 4], 2, 2).is_empty());
        // a disk: many staircase vertices, simplified under the cap
        let (w, h) = (101, 101);
        let disk: Vec<u8> = (0..w * h).map(|k| { let (x, y) = ((k % w) as f64 - 50.0, (k / w) as f64 - 50.0); u8::from(x * x + y * y < 45.0 * 45.0) }).collect();
        let c = trace_exterior(&disk, w, h);
        assert!(c.len() > 128);
        let s = simplify_closed(&c, 0.7, 128);
        assert!(s.len() <= 128 && s.len() >= 8);
        let a = polygon_area2(&s).abs() * 0.5;
        assert!((a - std::f64::consts::PI * 45.0 * 45.0).abs() < 0.03 * a, "{a}");
        let s = simplify_closed(&c, 0.7, 16);
        assert!(s.len() <= 16);
        assert!(validate_polygon(&s, 128).is_ok());
        assert_eq!(validate_polygon(&s[..2], 128), Err("contour_too_few_vertices"));
        assert_eq!(validate_polygon(&c, 128), Err("contour_too_many_vertices"));
        assert_eq!(validate_polygon(&[[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]], 128), Err("contour_zero_area"));
        assert_eq!(validate_polygon(&[[0.0, 0.0], [f64::NAN, 1.0], [2.0, 0.0]], 128), Err("contour_nonfinite"));
    }

    fn scene(w: usize, h: usize, bg: impl Fn(usize, usize) -> [u8; 3]) -> Rgb {
        let mut px = Vec::with_capacity(w * h * 3);
        for y in 0..h { for x in 0..w { px.extend(bg(x, y)); } }
        Rgb { w, h, px }
    }

    fn paint(img: &mut Rgb, f: impl Fn(usize, usize) -> Option<[u8; 3]>) {
        for y in 0..img.h { for x in 0..img.w { if let Some(c) = f(x, y) { img.px[3 * (y * img.w + x)..3 * (y * img.w + x) + 3].copy_from_slice(&c); } } }
    }

    #[test]
    fn classical_mask_on_checkered_background() {
        // white paper with black squares, a yellow ellipse
        let mut img = scene(200, 160, |x, y| if (x / 12 + y / 12) % 2 == 0 { [235, 235, 230] } else { [30, 30, 35] });
        let inside = |x: usize, y: usize| { let (u, v) = ((x as f64 - 100.0) / 40.0, (y as f64 - 80.0) / 20.0); u * u + v * v < 1.0 };
        paint(&mut img, |x, y| inside(x, y).then_some([230, 200, 30]));
        let poly = classical_silhouette(&img, [58.0, 58.0, 142.0, 102.0], &ClassicalParams::default()).unwrap();
        assert!(poly.len() <= 128);
        let a = polygon_area2(&poly).abs() * 0.5;
        let want = std::f64::consts::PI * 40.0 * 20.0;
        assert!((a - want).abs() < 0.05 * want, "{a} vs {want}");
        // clipped box, ambiguous (two blobs), low contrast (object colour around it)
        assert_eq!(classical_silhouette(&img, [0.0, 58.0, 142.0, 102.0], &ClassicalParams::default()), Err("clipped"));
        let mut two = scene(200, 160, |_, _| [235, 235, 230]);
        paint(&mut two, |x, y| ((60..95).contains(&x) && (60..100).contains(&y) || (105..140).contains(&x) && (60..100).contains(&y)).then_some([30, 160, 40]));
        assert_eq!(classical_silhouette(&two, [58.0, 58.0, 142.0, 102.0], &ClassicalParams::default()), Err("ambiguous"));
        let mut low = scene(200, 160, |x, _| if x < 100 { [230, 200, 30] } else { [235, 235, 230] });
        paint(&mut low, |x, y| inside(x, y).then_some([230, 200, 30]));
        // the object's colour is background: no object component
        assert!(matches!(classical_silhouette(&low, [58.0, 58.0, 142.0, 102.0], &ClassicalParams::default()), Err("low_contrast" | "component_size")));
    }
}
