//! Planar polygon geometry of an object footprint (base-frame xy, metres) and small 3-D
//! helpers of the grasp candidate search.

use crate::kinematics::{self, Mat3, Vec3};

pub type V2 = [f64; 2];

pub fn sub2(a: V2, b: V2) -> V2 { [a[0] - b[0], a[1] - b[1]] }
pub fn add2(a: V2, b: V2) -> V2 { [a[0] + b[0], a[1] + b[1]] }
pub fn scale2(a: V2, s: f64) -> V2 { [a[0] * s, a[1] * s] }
pub fn dot2(a: V2, b: V2) -> f64 { a[0] * b[0] + a[1] * b[1] }
pub fn cross2(a: V2, b: V2) -> f64 { a[0] * b[1] - a[1] * b[0] }
pub fn norm2(a: V2) -> f64 { a[0].hypot(a[1]) }
/// Unit vector of `a`; `[1, 0]` for a (near) zero vector.
pub fn unit2(a: V2) -> V2 {
    let n = norm2(a);
    if n > 1e-12 { [a[0] / n, a[1] / n] } else { [1.0, 0.0] }
}
/// `a` turned by +90 deg.
pub fn perp2(a: V2) -> V2 { [-a[1], a[0]] }

/// Shoelace area, positive for counter-clockwise vertices.
pub fn signed_area(p: &[V2]) -> f64 {
    let n = p.len();
    (0..n).map(|i| cross2(p[i], p[(i + 1) % n])).sum::<f64>() / 2.0
}

/// Reverse `p` when it is clockwise.
pub fn ensure_ccw(p: &mut [V2]) {
    if signed_area(p) < 0.0 { p.reverse(); }
}

/// Area centroid; the vertex mean for a degenerate polygon.
pub fn centroid(p: &[V2]) -> V2 {
    let a = signed_area(p);
    let n = p.len();
    if a.abs() < 1e-12 {
        let s = p.iter().fold([0.0, 0.0], |acc, v| add2(acc, *v));
        return scale2(s, 1.0 / n.max(1) as f64);
    }
    let (mut cx, mut cy) = (0.0, 0.0);
    for i in 0..n {
        let (u, v) = (p[i], p[(i + 1) % n]);
        let c = cross2(u, v);
        cx += (u[0] + v[0]) * c;
        cy += (u[1] + v[1]) * c;
    }
    [cx / (6.0 * a), cy / (6.0 * a)]
}

/// Long axis of the polygon (unit, from its area second moments about the centroid) and the
/// ratio of the major to the minor radius of gyration (1 for a round shape).
pub fn principal_axis(p: &[V2]) -> (V2, f64) {
    let c = centroid(p);
    let n = p.len();
    let (mut ixx, mut iyy, mut ixy, mut area) = (0.0, 0.0, 0.0, 0.0);
    for i in 0..n {
        let (u, v) = (sub2(p[i], c), sub2(p[(i + 1) % n], c));
        let k = cross2(u, v);
        area += k / 2.0;
        // second moments of area about the centroid (integrals of y², x², xy)
        ixx += (u[1] * u[1] + u[1] * v[1] + v[1] * v[1]) * k / 12.0;
        iyy += (u[0] * u[0] + u[0] * v[0] + v[0] * v[0]) * k / 12.0;
        ixy += (u[0] * v[1] + 2.0 * u[0] * u[1] + 2.0 * v[0] * v[1] + v[0] * u[1]) * k / 24.0;
    }
    if area < 0.0 { ixx = -ixx; iyy = -iyy; ixy = -ixy; }
    // covariance of x, y: [[iyy, ixy], [ixy, ixx]] (iyy = ∫x², ixx = ∫y²)
    let (sxx, syy, sxy) = (iyy, ixx, ixy);
    let tr = sxx + syy;
    let det = sxx * syy - sxy * sxy;
    let disc = (tr * tr / 4.0 - det).max(0.0).sqrt();
    let (l1, l2) = (tr / 2.0 + disc, (tr / 2.0 - disc).max(0.0));
    let axis = if sxy.abs() > 1e-15 { unit2([l1 - syy, sxy]) } else if sxx >= syy { [1.0, 0.0] } else { [0.0, 1.0] };
    // a fixed sign: the axis points into the half-plane x > 0 (or +y on the y axis)
    let axis = if axis[0] < -1e-12 || (axis[0].abs() <= 1e-12 && axis[1] < 0.0) { scale2(axis, -1.0) } else { axis };
    let ratio = if l2 > 1e-18 { (l1 / l2).sqrt() } else { f64::INFINITY };
    (axis, ratio)
}

/// Even-odd point in polygon.
pub fn point_in_polygon(p: &[V2], q: V2) -> bool {
    let n = p.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (p[i], p[j]);
        if (a[1] > q[1]) != (b[1] > q[1]) {
            let x = a[0] + (q[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
            if q[0] < x { inside = !inside; }
        }
        j = i;
    }
    inside
}

/// Distance from `q` to the polygon's boundary.
pub fn dist_to_boundary(p: &[V2], q: V2) -> f64 {
    let n = p.len();
    (0..n).map(|i| {
        let (a, b) = (p[i], p[(i + 1) % n]);
        let ab = sub2(b, a);
        let l2 = dot2(ab, ab);
        let t = if l2 > 1e-18 { (dot2(sub2(q, a), ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
        norm2(sub2(q, add2(a, scale2(ab, t))))
    }).fold(f64::INFINITY, f64::min)
}

/// Signed distance to the polygon: negative inside.
pub fn signed_dist(p: &[V2], q: V2) -> f64 {
    let d = dist_to_boundary(p, q);
    if point_in_polygon(p, q) { -d } else { d }
}

/// A crossing of the line `origin + t·dir` with polygon edge `edge` (from vertex `edge` to the next).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Crossing { pub t: f64, pub edge: usize }

/// Crossings of the line `origin + t·dir` with the polygon, ascending in `t`. An edge counts
/// when its end points lie strictly on different sides (one side half-open), so a line through
/// a vertex is counted once.
pub fn line_crossings(p: &[V2], origin: V2, dir: V2) -> Vec<Crossing> {
    let n = p.len();
    let side = |v: V2| cross2(dir, sub2(v, origin));
    let mut out = Vec::new();
    for i in 0..n {
        let (a, b) = (p[i], p[(i + 1) % n]);
        let (sa, sb) = (side(a), side(b));
        if (sa > 0.0) != (sb > 0.0) {
            let f = sa / (sa - sb);
            let x = add2(a, scale2(sub2(b, a), f));
            out.push(Crossing { t: dot2(sub2(x, origin), dir), edge: i });
        }
    }
    out.sort_by(|x, y| x.t.total_cmp(&y.t).then(x.edge.cmp(&y.edge)));
    out
}

/// Intervals of the line inside the polygon: crossing pairs (0, 1), (2, 3), ...
pub fn inside_intervals(p: &[V2], origin: V2, dir: V2) -> Vec<(Crossing, Crossing)> {
    line_crossings(p, origin, dir).chunks_exact(2).map(|c| (c[0], c[1])).collect()
}

/// Outward unit normal of edge `i` of a counter-clockwise polygon.
pub fn edge_normal(p: &[V2], i: usize) -> V2 {
    let d = sub2(p[(i + 1) % p.len()], p[i]);
    unit2([d[1], -d[0]])
}

/// Range of `dot(v - origin, dir)` over the vertices.
pub fn extent_along(p: &[V2], origin: V2, dir: V2) -> (f64, f64) {
    p.iter().map(|v| dot2(sub2(*v, origin), dir))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| (lo.min(s), hi.max(s)))
}

/// Rotation with columns `x`, `y`, `x × y`; `y` is made orthogonal to `x`. None when `y` is
/// (nearly) parallel to `x` (projected length below `min_perp`).
pub fn frame_xy(x: &Vec3, y: &Vec3, min_perp: f64) -> Option<Mat3> {
    let x = kinematics::unit(x);
    let d = kinematics::dot(y, &x);
    let yp = kinematics::sub(y, &x.map(|v| v * d));
    if kinematics::norm(&yp) < min_perp { return None; }
    let y = kinematics::unit(&yp);
    let z = kinematics::cross(&x, &y);
    Some(kinematics::from_columns(&x, &y, &z))
}

/// Eigenvalues of a symmetric n×n matrix (row-major, n <= 8) by cyclic Jacobi rotations.
pub fn sym_eigenvalues(m: &mut [f64], n: usize) -> Vec<f64> {
    for _sweep in 0..30 {
        let off: f64 = (0..n).flat_map(|i| (0..n).filter(move |&j| j != i).map(move |j| (i, j)))
            .map(|(i, j)| m[i * n + j] * m[i * n + j]).sum();
        if off < 1e-22 { break; }
        for p in 0..n {
            for q in p + 1..n {
                let apq = m[p * n + q];
                if apq.abs() < 1e-30 { continue; }
                let (app, aqq) = (m[p * n + p], m[q * n + q]);
                let theta = (aqq - app) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let (akp, akq) = (m[k * n + p], m[k * n + q]);
                    m[k * n + p] = c * akp - s * akq;
                    m[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (m[p * n + k], m[q * n + k]);
                    m[p * n + k] = c * apk - s * aqk;
                    m[q * n + k] = s * apk + c * aqk;
                }
            }
        }
    }
    (0..n).map(|i| m[i * n + i]).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(s: f64) -> Vec<V2> { vec![[0.0, 0.0], [s, 0.0], [s, s], [0.0, s]] }

    #[test]
    fn area_centroid_and_orientation() {
        let mut p = square(2.0);
        assert!((signed_area(&p) - 4.0).abs() < 1e-12);
        p.reverse();
        assert!(signed_area(&p) < 0.0);
        ensure_ccw(&mut p);
        assert!(signed_area(&p) > 0.0);
        let c = centroid(&p);
        assert!((c[0] - 1.0).abs() < 1e-12 && (c[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn principal_axis_of_a_rotated_rectangle() {
        let a = 30f64.to_radians();
        let (u, v) = ([a.cos(), a.sin()], [-a.sin(), a.cos()]);
        let p: Vec<V2> = [[-0.1, -0.02], [0.1, -0.02], [0.1, 0.02], [-0.1, 0.02]].iter()
            .map(|q| add2(scale2(u, q[0]), scale2(v, q[1]))).collect();
        let (axis, ratio) = principal_axis(&p);
        assert!(dot2(axis, u).abs() > 0.9999, "{axis:?}");
        assert!((ratio - 5.0).abs() < 1e-6, "{ratio}");
        let (_, r_sq) = principal_axis(&square(1.0));
        assert!((r_sq - 1.0).abs() < 1e-9);
    }

    #[test]
    fn crossings_through_a_vertex_count_once() {
        let p = square(1.0);
        // diagonal through two vertices
        let c = line_crossings(&p, [-1.0, -1.0], unit2([1.0, 1.0]));
        assert_eq!(c.len(), 2, "{c:?}");
        let i = inside_intervals(&p, [0.5, -1.0], [0.0, 1.0]);
        assert_eq!(i.len(), 1);
        assert!((i[0].0.t - 1.0).abs() < 1e-12 && (i[0].1.t - 2.0).abs() < 1e-12);
        assert_eq!(edge_normal(&p, i[0].0.edge), [0.0, -1.0]);
        assert_eq!(edge_normal(&p, i[0].1.edge), [0.0, 1.0]);
    }

    #[test]
    fn concave_polygon_gives_two_intervals() {
        // a U: the line y = 0.8 crosses both arms
        let p = vec![[0.0, 0.0], [3.0, 0.0], [3.0, 1.0], [2.0, 1.0], [2.0, 0.5], [1.0, 0.5], [1.0, 1.0], [0.0, 1.0]];
        let i = inside_intervals(&p, [-1.0, 0.8], [1.0, 0.0]);
        assert_eq!(i.len(), 2);
        assert!(!point_in_polygon(&p, [1.5, 0.8]));
        assert!(point_in_polygon(&p, [0.5, 0.8]));
        assert!((signed_dist(&p, [1.5, 0.8]) - 0.3).abs() < 1e-12);
        assert!((signed_dist(&p, [0.5, 0.2]) + 0.2).abs() < 1e-12);
    }

    #[test]
    fn frame_and_eigenvalues() {
        let r = frame_xy(&[0.0, 0.0, -2.0], &[1.0, 0.0, 0.3], 0.5).expect("not parallel");
        assert!((kinematics::norm(&kinematics::column(&r, 1)) - 1.0).abs() < 1e-12);
        assert!(kinematics::dot(&kinematics::column(&r, 0), &kinematics::column(&r, 1)).abs() < 1e-12);
        assert!(frame_xy(&[0.0, 0.0, 1.0], &[0.0, 0.1, 1.0], 0.5).is_none());
        let mut m = vec![2.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0];
        let mut e = sym_eigenvalues(&mut m, 3);
        e.sort_by(f64::total_cmp);
        assert!((e[0] - 1.0).abs() < 1e-9 && (e[1] - 3.0).abs() < 1e-9 && (e[2] - 5.0).abs() < 1e-9, "{e:?}");
    }
}
