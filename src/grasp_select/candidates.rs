//! Candidate generation: contact chords along the footprint, approach directions over the
//! upper hemisphere, both jaw half-turns, and the cheap geometric checks run before any IK.

use super::geometry::{self as g, V2};
use super::snapshot::{Side, Snapshot, Target, ToolVolumes};
use crate::grasp::TABLE_CLEARANCE_M;
use crate::kinematics::{self, Mat3, Pose, Vec3};

/// Stations along the long axis (fraction of the length), central ones first.
pub const STATIONS: [f64; 5] = [0.5, 0.35, 0.65, 0.2, 0.8];
/// Tangent estimate: neighbouring stations at ± this fraction of the length.
const TANGENT_STEP: f64 = 0.08;
/// Oblique tilts from vertical (deg) for every direction class.
pub const TILTS_DEG: [f64; 2] = [30.0, 50.0];
/// Near-horizontal tilt, outward only.
pub const NEAR_HORIZONTAL_DEG: f64 = 70.0;
/// Jaw axis may tilt this much out of the horizontal plane (deg).
pub const MAX_JAW_ELEVATION_DEG: f64 = 20.0;
/// Least pad length inside the object's height (m, worst height).
pub const MIN_PAD_OVERLAP_M: f64 = 0.008;
/// Least clearance per side between the error-shifted object and an open finger (m).
pub const MIN_OPENING_RESERVE_M: f64 = 0.003;
/// Narrowest chord the jaws close on (m).
pub const MIN_CHORD_M: f64 = 0.005;
/// Pre-grasp distances tried along -a (m), shortest first.
pub const PREGRASP_DISTANCES_M: [f64; 5] = [0.04, 0.06, 0.08, 0.10, 0.12];
/// Insertion sweep step (m).
pub const SWEEP_STEP_M: f64 = 0.005;
/// Added to the table clearance at the contact: the path IK reaches poses within
/// [`kinematics::PAR`] `tol_pos` (1 mm), so the envelope may end that much lower (m).
pub const IK_POSITION_ALLOWANCE_M: f64 = 0.0015;
/// Vertical extra margin above the object's top in the clearance checks (m).
const TOP_MARGIN_M: f64 = 0.003;

/// Approach direction class: the horizontal part of `a` relative to the base direction of the
/// contact. `Outward`: `a` points away from the base (the wrist nearer the base).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DirClass { Vertical, Outward, Inward, LateralLeft, LateralRight }

impl DirClass {
    pub fn name(self) -> &'static str {
        match self { Self::Vertical => "vertical", Self::Outward => "radial_outward", Self::Inward => "radial_inward",
            Self::LateralLeft => "lateral_left", Self::LateralRight => "lateral_right" }
    }
    pub fn is_radial_oblique(self) -> bool { matches!(self, Self::Outward | Self::Inward) }
    pub fn side(self) -> Side {
        match self { Self::Vertical => Side::Any, Self::Outward => Side::Outward, Self::Inward => Side::Inward, _ => Side::Lateral }
    }
}

/// An opposing contact pair across the footprint.
#[derive(Debug, Clone, PartialEq)]
pub struct Chord {
    /// Station fraction along the long axis and which axis (0 = principal, 1 = the other).
    pub station: f64,
    pub axis_index: usize,
    pub centre: V2,
    /// Unit direction from contact 1 to contact 2 (horizontal).
    pub dir: V2,
    pub width_m: f64,
    /// Local centreline tangent.
    pub tangent: V2,
    /// Opposition of the silhouette normals at the contacts, -1..1 (1 = antipodal).
    pub opposition: f64,
    /// 1 - (distance from the footprint centroid to the chord line) / half length, 0..1.
    pub moment_q: f64,
    /// Smallest cross-section within the finger width, as a fraction of `width_m`.
    pub pad_support: f64,
}

/// A rejected item of the search with its phase and numerical reason.
#[derive(Debug, Clone, PartialEq)]
pub struct Rejection {
    pub phase: &'static str,
    pub code: String,
    pub message: String,
    /// The value that failed and its bound, when there is one.
    pub value: Option<f64>,
    pub bound: Option<f64>,
    /// The geometry that failed (`tcp`, `gripper_envelope:<i>`, `camera_lens`, `camera_body:<i>`, ...).
    pub geometry: Option<String>,
    /// Path parameter of the failure within its phase (0..1).
    pub s: Option<f64>,
    /// True when this rejection came from a planning-only joint limit violation.
    pub planning_limit: bool,
}

impl Rejection {
    pub fn new(phase: &'static str, code: &str, message: impl Into<String>) -> Self {
        Self { phase, code: code.into(), message: message.into(), value: None, bound: None, geometry: None, s: None, planning_limit: false }
    }
    pub fn with(mut self, value: f64, bound: f64) -> Self { self.value = Some(value); self.bound = Some(bound); self }
}

/// Midpoint of the longest inside interval of the cut across `axis` at `s` (from `origin`).
fn station_mid(poly: &[V2], origin: V2, axis: V2, s: f64) -> Option<V2> {
    let p = g::add2(origin, g::scale2(axis, s));
    let u = g::perp2(axis);
    g::inside_intervals(poly, p, u).into_iter()
        .max_by(|a, b| (a.1.t - a.0.t).total_cmp(&(b.1.t - b.0.t)))
        .map(|(a, b)| g::add2(p, g::scale2(u, (a.t + b.t) / 2.0)))
}

/// Signed inside interval `(lo_m, hi_m)` relative to `p` along unit `u` of the line through `p`
/// that contains `p` or lies nearest `p` (optionally capped at `max_gap_m` so a disjoint chord
/// far from `p` is never substituted).
pub fn cut_interval_near(poly: &[V2], p: V2, u: V2, max_gap_m: Option<f64>) -> Option<(f64, f64)> {
    g::inside_intervals(poly, p, u)
        .into_iter()
        .filter(|iv| max_gap_m.is_none_or(|g| interval_gap(iv) <= g + 1e-9))
        .min_by(|a, b| interval_gap(a).total_cmp(&interval_gap(b)))
        .map(|(a, b)| (a.t, b.t))
}

fn interval_gap(iv: &(g::Crossing, g::Crossing)) -> f64 {
    if iv.0.t <= 0.0 && iv.1.t >= 0.0 { 0.0 } else { iv.0.t.abs().min(iv.1.t.abs()) }
}

/// The chord of the cut `m + t·u` between crossings `(t0, edge0, t1, edge1)`: contacts, width,
/// opposition, pad support within the finger width and moment quality (`len`: footprint length
/// along `tangent`). Station and axis are 0.5 / 0 (callers set them).
fn chord_from(t: &Target, m: V2, u: V2, cut: (f64, usize, f64, usize), tangent: V2, len: f64, tool: &ToolVolumes) -> Result<Chord, Rejection> {
    let poly = &t.poly;
    let (t0, edge0, t1, edge1) = cut;
    let (e1, e2) = (g::add2(m, g::scale2(u, t0)), g::add2(m, g::scale2(u, t1)));
    let width = t1 - t0;
    let centre = g::scale2(g::add2(e1, e2), 0.5);
    let (n1, n2) = (g::edge_normal(poly, edge0), g::edge_normal(poly, edge1));
    let opposition = (-g::dot2(n1, u) + g::dot2(n2, u)) / 2.0;
    let half = tool.finger_width_m / 2.0;
    let support = [-1.0, 1.0].iter()
        .map(|k| cut_interval_near(poly, g::add2(centre, g::scale2(tangent, k * half)), u, None).map_or(0.0, |(lo, hi)| (hi - lo) / width))
        .fold(f64::INFINITY, f64::min);
    if support < 0.5 {
        return Err(Rejection::new("geometry", "PAD_BEYOND_TIP", format!("the cross-section within the finger width shrinks to {:.0} % of the chord", support * 100.0)).with(support, 0.5));
    }
    let d = g::cross2(u, g::sub2(t.centroid, centre)).abs();
    let moment_q = 1.0 - (d / (len / 2.0).max(1e-6)).min(1.0);
    Ok(Chord { station: 0.5, axis_index: 0, centre, dir: u, width_m: width, tangent, opposition, moment_q, pad_support: support.min(1.0) })
}

/// The chord along the closing axis `u` through `p` (the top-down target): the cut interval
/// containing `p`, else the nearest one. The jaws centre on the chord midpoint.
pub fn chord_through(t: &Target, p: V2, u: V2, tool: &ToolVolumes) -> Result<Chord, Rejection> {
    let u = g::unit2(u);
    let tangent = g::perp2(u);
    let (lo, hi) = g::extent_along(&t.poly, t.centroid, tangent);
    let Some((c0, c1)) = g::inside_intervals(&t.poly, p, u).into_iter().min_by(|a, b| interval_gap(a).total_cmp(&interval_gap(b))) else {
        return Err(Rejection::new("geometry", "NO_CROSS_SECTION", "the closing line through the top-down target misses the footprint"));
    };
    chord_from(t, p, u, (c0.t, c0.edge, c1.t, c1.edge), tangent, hi - lo, tool)
}

/// Contact chords at [`STATIONS`] along the local centreline (both axes when the long axis is
/// ambiguous) and the rejected stations.
pub fn chords(t: &Target, tool: &ToolVolumes) -> (Vec<Chord>, Vec<Rejection>) {
    let poly = &t.poly;
    let mut axes = vec![t.axis];
    if t.axis_ambiguous { axes.push(g::perp2(t.axis)); }
    let mut out = Vec::new();
    let mut rej = Vec::new();
    for (ai, axis) in axes.iter().enumerate() {
        let (smin, smax) = g::extent_along(poly, t.centroid, *axis);
        let len = smax - smin;
        for f in STATIONS {
            let s = smin + f * len;
            let Some(m) = station_mid(poly, t.centroid, *axis, s) else {
                rej.push(Rejection::new("geometry", "NO_CROSS_SECTION", format!("station {f:.2}: the cut misses the footprint")));
                continue;
            };
            let tangent = match (station_mid(poly, t.centroid, *axis, s - TANGENT_STEP * len), station_mid(poly, t.centroid, *axis, s + TANGENT_STEP * len)) {
                (Some(a), Some(b)) if g::norm2(g::sub2(b, a)) > 1e-6 => g::unit2(g::sub2(b, a)),
                _ => *axis,
            };
            let u = g::perp2(tangent);
            let Some((c0, c1)) = g::inside_intervals(poly, m, u).into_iter().find(|(a, b)| a.t <= 1e-9 && b.t >= -1e-9) else {
                rej.push(Rejection::new("geometry", "CHORD_CROSSES_EMPTY_SPACE", format!("station {f:.2}: the centreline point lies outside the cut")));
                continue;
            };
            let chord = match chord_from(t, m, u, (c0.t, c0.edge, c1.t, c1.edge), tangent, len, tool) {
                Ok(c) => Chord { station: f, axis_index: ai, ..c },
                Err(why) => { rej.push(Rejection { message: format!("station {f:.2}: {}", why.message), ..why }); continue; }
            };
            let centre = chord.centre;
            // the same contact from both axes (round objects) once
            if !out.iter().any(|c: &Chord| g::norm2(g::sub2(c.centre, centre)) < 0.002 && g::dot2(c.dir, chord.dir).abs() > 0.995) {
                out.push(chord);
            }
        }
    }
    (out, rej)
}

/// A grasp hypothesis: a chord, an approach direction and a jaw half-turn, with its contact TCP.
#[derive(Debug, Clone)]
pub struct Hypothesis {
    pub index: usize,
    pub chord: usize,
    pub class: DirClass,
    pub tilt_deg: f64,
    /// Second contact depth (deeper into the band) instead of the shallowest admissible one.
    pub deep: bool,
    pub half_turn: bool,
    /// Approach direction (unit, pre-grasp -> grasp).
    pub a: Vec3,
    /// TCP orientation (columns: a, jaw closing y, z).
    pub r: Mat3,
    /// Contact TCP (jaw tips).
    pub g: Vec3,
    /// Pre-grasp distance along -a.
    pub insertion_m: f64,
    /// Chord width along the jaw axis (m).
    pub width_m: f64,
    pub pad_overlap_m: f64,
    /// Per-side clearance of the open fingers to the error-shifted object (m).
    pub opening_reserve_m: f64,
    /// Clearance of the lowest finger / envelope point above the table at the contact (m).
    pub table_clearance_m: f64,
    /// Jaw axis elevation from horizontal (deg).
    pub jaw_elevation_deg: f64,
}

impl Hypothesis {
    pub fn grasp_pose(&self) -> Pose { Pose::new(self.r, self.g) }
    pub fn pregrasp_pose(&self) -> Pose { Pose::new(self.r, kinematics::sub(&self.g, &self.a.map(|v| v * self.insertion_m))) }
}

/// Approach directions for a contact at `c`: vertical, then per tilt the radial and lateral
/// classes, then near-horizontal outward. (class, tilt, horizontal unit).
pub fn directions(c: V2) -> Vec<(DirClass, f64, V2)> {
    let rho = if g::norm2(c) > 0.03 { g::unit2(c) } else { [1.0, 0.0] };
    let lam = g::perp2(rho);
    let mut out = vec![(DirClass::Vertical, 0.0, rho)];
    for t in TILTS_DEG {
        out.push((DirClass::Outward, t, rho));
        out.push((DirClass::Inward, t, g::scale2(rho, -1.0)));
        out.push((DirClass::LateralLeft, t, lam));
        out.push((DirClass::LateralRight, t, g::scale2(lam, -1.0)));
    }
    out.push((DirClass::Outward, NEAR_HORIZONTAL_DEG, rho));
    out
}

/// Approach vector of tilt `tilt_deg` from straight down toward horizontal unit `h`.
pub fn approach_vector(tilt_deg: f64, h: V2) -> Vec3 {
    let (s, c) = tilt_deg.to_radians().sin_cos();
    [h[0] * s, h[1] * s, -c]
}

/// The box corners of a finger (`sign` ±1 along y) or the palm (`sign` 0) at jaw opening `w`, in
/// the jaw frame, as a 3×3×3 lattice (corners, edge and face centres, centre).
pub fn volume_lattice(tool: &ToolVolumes, w: f64, sign: f64) -> Vec<Vec3> {
    let (x0, x1, y0, y1, hz) = if sign == 0.0 {
        let half = w / 2.0 + tool.finger_thickness_m;
        (-tool.finger_length_m - tool.palm_depth_m, -tool.finger_length_m, -half, half, tool.finger_width_m / 2.0 + 0.01)
    } else {
        let (a, b) = (w / 2.0, w / 2.0 + tool.finger_thickness_m);
        let (y0, y1) = if sign > 0.0 { (a, b) } else { (-b, -a) };
        (-tool.finger_length_m, 0.0, y0, y1, tool.finger_width_m / 2.0)
    };
    let mut out = Vec::with_capacity(27);
    for fx in [0.0, 0.5, 1.0] { for fy in [0.0, 0.5, 1.0] { for fz in [-1.0, 0.0, 1.0] {
        out.push([x0 + (x1 - x0) * fx, y0 + (y1 - y0) * fy, hz * fz]);
    }}}
    out
}

/// Lattice of both fingers and the palm at opening `w`.
pub fn gripper_lattice(tool: &ToolVolumes, w: f64) -> Vec<Vec3> {
    let mut v = volume_lattice(tool, w, 1.0);
    v.extend(volume_lattice(tool, w, -1.0));
    v.extend(volume_lattice(tool, w, 0.0));
    v
}

/// The object as a prism over the footprint, `inflate` m wider, from the table to the top.
pub struct Prism<'a> { pub poly: &'a [V2], pub z0: f64, pub z1: f64, pub inflate: f64, lo: V2, hi: V2 }

impl<'a> Prism<'a> {
    pub fn new(poly: &'a [V2], z0: f64, z1: f64, inflate: f64) -> Self {
        let lo = poly.iter().fold([f64::INFINITY; 2], |m, v| [m[0].min(v[0]), m[1].min(v[1])]);
        let hi = poly.iter().fold([f64::NEG_INFINITY; 2], |m, v| [m[0].max(v[0]), m[1].max(v[1])]);
        Self { poly, z0, z1, inflate, lo, hi }
    }
    /// Whether the base-frame point lies in the (inflated) prism.
    pub fn contains(&self, p: &Vec3) -> bool {
        if p[2] < self.z0 || p[2] > self.z1 { return false; }
        let e = self.inflate;
        if p[0] < self.lo[0] - e || p[0] > self.hi[0] + e || p[1] < self.lo[1] - e || p[1] > self.hi[1] + e { return false; }
        g::signed_dist(self.poly, [p[0], p[1]]) <= e
    }
}

/// First sample of the straight move of the gripper volumes at opening `w` from `from` to `to`
/// (TCP poses, same orientation) that meets the prism: (s, point in base) or None.
pub fn sweep_hits(tool: &ToolVolumes, w: f64, from: &Pose, to: &Pose, prism: &Prism, tests: &mut usize) -> Option<(f64, Vec3)> {
    let lattice = gripper_lattice(tool, w);
    let d = kinematics::sub(&to.p, &from.p);
    let n = (kinematics::norm(&d) / SWEEP_STEP_M).ceil().max(1.0) as usize;
    for k in 0..=n {
        let s = k as f64 / n as f64;
        let pose = Pose::new(from.r, kinematics::add(&from.p, &d.map(|v| v * s)));
        for l in &lattice {
            *tests += 1;
            let p = pose.transform_point(l);
            if prism.contains(&p) { return Some((s, p)); }
        }
    }
    None
}

/// Object footprint vertices (at three heights) inside the finger boxes beyond the pads at
/// contact width `w`: contact off the pads during closure. Returns the deepest such vertex's
/// jaw-frame x (m) or None.
pub fn off_pad_contact(tool: &ToolVolumes, t: &Target, pose: &Pose, w: f64) -> Option<f64> {
    let rt = kinematics::transpose(&pose.r);
    let top = t.table_z + t.height[0];
    let half = w / 2.0;
    let mut hit: Option<f64> = None;
    for z in [t.table_z + 0.002, (t.table_z + top) / 2.0, top] {
        for v in &t.poly {
            let q = kinematics::mat_vec(&rt, &kinematics::sub(&[v[0], v[1], z], &pose.p));
            let in_finger_x = q[0] >= -tool.finger_length_m && q[0] < -tool.pad_length_m;
            let in_z = q[2].abs() <= tool.finger_width_m / 2.0;
            let in_y = q[1].abs() >= half + 0.001 && q[1].abs() <= half + tool.finger_thickness_m;
            if in_finger_x && in_z && in_y { hit = Some(hit.map_or(q[0], |h: f64| h.min(q[0]))); }
        }
    }
    hit
}

/// Lowest z of `pts` (jaw/TCP frame) at orientation `r`, relative to the TCP.
fn lowest(r: &Mat3, pts: &[Vec3]) -> f64 { pts.iter().map(|p| kinematics::mat_vec(r, p)[2]).fold(0.0, f64::min) }

/// Lowest z of `end_link`-frame `pts` at orientation `r`, relative to the TCP (`p - tcp_offset`).
pub(super) fn lowest_end(r: &Mat3, pts: &[Vec3], tcp_offset: &Vec3) -> f64 {
    pts.iter()
        .map(|p| kinematics::mat_vec(r, &kinematics::sub(p, tcp_offset))[2])
        .fold(0.0, f64::min)
}

/// Counters of the cheap stage.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CheapStats { pub generated: usize, pub lattice_tests: usize }

/// All hypotheses for the chords and the rejected ones (geometry / table / pre-open /
/// insertion / closure / preference), in a fixed order: chord, direction, half-turn, depth.
pub fn hypotheses(s: &Snapshot, t: &Target, chords: &[Chord], stats: &mut CheapStats) -> (Vec<Hypothesis>, Vec<(Hypothesis, Rejection)>) {
    let mut ok = Vec::new();
    let mut rejected = Vec::new();
    let mut index = 0;
    for (ci, ch) in chords.iter().enumerate() {
        for (class, tilt, h) in directions(ch.centre) {
            for half_turn in [false, true] {
                for deep in [false, true] {
                    match make(s, t, ch, ci, (class, tilt, h), half_turn, deep, index, stats) {
                        Ok(hy) => ok.push(hy),
                        Err(r) => rejected.push(*r),
                    }
                    index += 1;
                }
            }
        }
    }
    (ok, rejected)
}

/// One hypothesis: chord `ch` (index `ci`), direction (class, tilt, horizontal unit), jaw
/// half-turn and contact depth, through the cheap checks.
#[allow(clippy::too_many_arguments)]
pub fn make(s: &Snapshot, t: &Target, ch: &Chord, ci: usize, dir: (DirClass, f64, V2), half_turn: bool, deep: bool, index: usize, stats: &mut CheapStats) -> Result<Hypothesis, Box<(Hypothesis, Rejection)>> {
    stats.generated += 1;
    let (class, tilt, h) = dir;
    let prism = Prism::new(&t.poly, t.table_z - 1.0, t.table_z + t.height[1] + TOP_MARGIN_M, t.hard_error_m);
    let u = [ch.dir[0], ch.dir[1], 0.0];
    let y_in = if half_turn { u.map(|v| -v) } else { u };
    let base = Hypothesis {
        index, chord: ci, class, tilt_deg: tilt, deep, half_turn, a: approach_vector(tilt, h), r: kinematics::IDENTITY,
        g: [ch.centre[0], ch.centre[1], t.table_z], insertion_m: 0.0, width_m: ch.width_m, pad_overlap_m: 0.0,
        opening_reserve_m: 0.0, table_clearance_m: 0.0, jaw_elevation_deg: 0.0,
    };
    build(s, t, ch, base.clone(), y_in, &prism, stats).map_err(|r| Box::new((base, r)))
}

fn build(s: &Snapshot, t: &Target, ch: &Chord, mut hy: Hypothesis, y_in: Vec3, prism: &Prism, stats: &mut CheapStats) -> Result<Hypothesis, Rejection> {
    let tool = &s.tool;
    let table = t.table_z;
    if let Some(m) = s.prefs.hard_max_tilt_deg.filter(|m| hy.tilt_deg > *m + 1e-9) {
        return Err(Rejection::new("geometry", "PREFERENCE_HARD", format!("tilt {:.0} deg exceeds hard.max_tilt_deg {m:.0}", hy.tilt_deg)).with(hy.tilt_deg, m));
    }
    let Some(r) = g::frame_xy(&hy.a, &y_in, 0.5) else {
        return Err(Rejection::new("geometry", "DEGENERATE_JAW", "the chord is nearly parallel to the approach"));
    };
    let y = kinematics::column(&r, 1);
    let elev = y[2].abs().asin().to_degrees();
    if elev > MAX_JAW_ELEVATION_DEG {
        return Err(Rejection::new("geometry", "JAW_AXIS_TILTED", format!("the jaw axis leaves the horizontal by {elev:.0} deg")).with(elev, MAX_JAW_ELEVATION_DEG));
    }
    let cos_elev = elev.to_radians().cos();
    let width = ch.width_m / cos_elev;
    if width < MIN_CHORD_M && width.is_finite() {
        return Err(Rejection::new("geometry", "TOO_NARROW", format!("chord {width:.4} m")).with(width, MIN_CHORD_M));
    }
    let (lo_h, hi_h) = cut_interval_near(&t.poly, ch.centre, ch.dir, None)
        .unwrap_or((-ch.width_m / 2.0, ch.width_m / 2.0));
    let cut = super::admission::JawCut::new(lo_h / cos_elev, hi_h / cos_elev, t.source.name());
    let fit = super::admission::jaw_check(&cut, t.hard_error_m, tool.open_width_m, tool.max_grasp_width_cap)?;
    let reserve = fit.reserve_m;
    // shallowest admissible contact: envelope points (in end_link frame) and open fingers (in
    // TCP/jaw frame) TABLE_CLEARANCE_M above the table, the tips at least grasp_height_m
    let finger_low = lowest(&r, &gripper_lattice(tool, tool.open_width_m));
    let env_low = lowest_end(&r, &s.envelope, &s.input.tcp_offset);
    let z_min = (table + TABLE_CLEARANCE_M + IK_POSITION_ALLOWANCE_M - finger_low.min(env_low)).max(table + s.setup.grasp_height_m);
    let top_lo = table + t.height[0];
    let cos_t = -hy.a[2];
    let overlap = |gz: f64| -> f64 {
        let pad = tool.pad_length_m;
        if cos_t < 0.2 { return if gz <= top_lo { pad } else { 0.0 }; }
        ((top_lo - gz) / cos_t).clamp(0.0, pad)
    };
    let gz = if hy.deep {
        let deeper = z_min + 0.35 * (top_lo - z_min);
        if deeper < z_min + 0.004 {
            return Err(Rejection::new("geometry", "DEPTH_BAND", "no second contact depth inside the band"));
        }
        deeper
    } else { z_min };
    let ov = overlap(gz);
    // TODO(1.6.2): align pad-engagement check across both strategies after measuring real-arm
    // pad geometry (on-site item G8).
    if ov < MIN_PAD_OVERLAP_M {
        return Err(Rejection::new("table", "PAD_OVERLAP", format!("the pads overlap the object by {:.4} m above the table clearance", ov)).with(ov, MIN_PAD_OVERLAP_M));
    }
    // the pad centre (pad_length / 2 behind the tips) on the chord's vertical line
    let back = (tool.pad_length_m / 2.0).min(ov / 2.0);
    let horiz = [hy.a[0], hy.a[1]];
    let gxy = g::add2(ch.centre, g::scale2(horiz, back));
    hy.g = [gxy[0], gxy[1], gz];
    hy.r = r;
    hy.width_m = width;
    hy.pad_overlap_m = ov;
    hy.opening_reserve_m = reserve;
    hy.table_clearance_m = gz + finger_low.min(env_low) - table;
    hy.jaw_elevation_deg = elev;
    let grasp = hy.grasp_pose();
    // pre-grasp: the shortest distance whose open gripper clears the inflated object, then the
    // insertion sweep from there
    let mut chosen = None;
    for d in PREGRASP_DISTANCES_M {
        hy.insertion_m = d;
        let pre = hy.pregrasp_pose();
        let clear = gripper_lattice(tool, tool.open_width_m).iter().all(|l| { stats.lattice_tests += 1; !prism.contains(&pre.transform_point(l)) });
        if clear { chosen = Some(d); break; }
    }
    let Some(d) = chosen else {
        return Err(Rejection::new("pre_open", "PREGRASP_BLOCKED", format!("the open gripper meets the object at every pre-grasp distance up to {:.2} m", PREGRASP_DISTANCES_M[4])));
    };
    hy.insertion_m = d;
    let pre = hy.pregrasp_pose();
    if let Some((sv, p)) = sweep_hits(tool, tool.open_width_m, &pre, &grasp, prism, &mut stats.lattice_tests) {
        return Err(Rejection::new("insertion", "FINGER_COLLISION", format!("an open finger or the palm meets the error-shifted object at s={sv:.2} ({:.3}, {:.3}, {:.3})", p[0], p[1], p[2])).with(sv, 1.0));
    }
    // closure: the nominal object between the fingers touches the pads only
    if let Some(x) = off_pad_contact(tool, t, &grasp, width) {
        return Err(Rejection::new("closure", "OFF_PAD_CONTACT", format!("the object reaches a finger {:.3} m behind the tips, beyond the pads", -x)).with(-x, tool.pad_length_m));
    }
    Ok(hy)
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::snapshot::FootprintSource;

    /// A curved banana footprint: an arc of radius `rc` (centre at `c`), `half_angle`, width `w`.
    pub fn banana(c: V2, rc: f64, half_angle: f64, w: f64, n: usize) -> Vec<V2> {
        let mut out = Vec::new();
        for k in 0..=n {
            let t = -half_angle + 2.0 * half_angle * k as f64 / n as f64;
            out.push([c[0] + (rc + w / 2.0) * t.cos(), c[1] + (rc + w / 2.0) * t.sin()]);
        }
        for k in (0..=n).rev() {
            let t = -half_angle + 2.0 * half_angle * k as f64 / n as f64;
            out.push([c[0] + (rc - w / 2.0) * t.cos(), c[1] + (rc - w / 2.0) * t.sin()]);
        }
        let mut p = out;
        g::ensure_ccw(&mut p);
        p
    }

    fn target_of(poly: Vec<V2>) -> Target {
        let c = g::centroid(&poly);
        let (axis, aspect) = g::principal_axis(&poly);
        Target {
            poly_lo: poly.clone(), poly_hi: poly.clone(), poly, source: FootprintSource::Contour, fallback_reason: None, table_z: 0.0, height: [0.03, 0.04], height_source: "evidence",
            centroid: c, axis, aspect, axis_ambiguous: aspect < 1.2, projection_spread_m: 0.0, hard_error_m: 0.005, scale_error_m: 0.005,
            hand_eye_error_m: 0.002, contour_error_m: 0.001, view_distance_m: 0.25,
            margin: crate::grasp_select::margin::margin(&Default::default(), 0.005, 0.001, 0.25, 0.0),
        }
    }

    #[test]
    fn curved_banana_chords_follow_the_centreline() {
        // arc of radius 0.12 m, ±40 deg, 34 mm wide: the centre of curvature at the origin, so
        // the centreline passes 0.12 m from it
        let poly = banana([-0.12 + 0.40, 0.0], 0.12, 40f64.to_radians(), 0.034, 24);
        let t = target_of(poly);
        let tool = ToolVolumes::b601dm_assumed(0.09, 0.075);
        let (cs, _rej) = chords(&t, &tool);
        assert!(cs.len() >= 3, "{cs:?}");
        for c in &cs {
            // every chord centre lies on the arc of radius 0.12 about (0.28, 0)
            let r = g::norm2(g::sub2(c.centre, [0.28, 0.0]));
            assert!((r - 0.12).abs() < 0.004, "station {} centre {:?} at radius {r}", c.station, c.centre);
            // across the banana: about 34 mm, along the local radius
            assert!((c.width_m - 0.034).abs() < 0.004, "width {}", c.width_m);
            let radial = g::unit2(g::sub2(c.centre, [0.28, 0.0]));
            assert!(g::dot2(c.dir, radial).abs() > 0.98, "chord direction {:?} vs radial {radial:?}", c.dir);
            assert!(c.opposition > 0.9, "{}", c.opposition);
        }
        // the central station is near the centroid line
        assert!(cs[0].moment_q > 0.8, "{}", cs[0].moment_q);
    }

    #[test]
    fn tapered_tip_is_rejected() {
        // a triangle: the 0.8 station's chord is short and the pad runs off the tip
        let mut poly = vec![[0.30, -0.015], [0.30, 0.015], [0.36, 0.0]];
        g::ensure_ccw(&mut poly);
        let t = target_of(poly);
        let tool = ToolVolumes::b601dm_assumed(0.09, 0.075);
        let (cs, rej) = chords(&t, &tool);
        assert!(rej.iter().any(|r| r.code == "PAD_BEYOND_TIP"), "{rej:?}");
        assert!(cs.iter().all(|c| c.pad_support >= 0.5));
    }

    #[test]
    fn approach_vectors_and_lattice() {
        let a = approach_vector(30.0, [1.0, 0.0]);
        assert!((a[0] - 0.5).abs() < 1e-12 && (a[2] + 3f64.sqrt() / 2.0).abs() < 1e-12);
        let d = directions([0.4, 0.0]);
        assert_eq!(d.len(), 10);
        assert_eq!(d[0].0, DirClass::Vertical);
        let tool = ToolVolumes::b601dm_assumed(0.09, 0.075);
        let l = gripper_lattice(&tool, 0.05);
        assert_eq!(l.len(), 81);
        // nothing of the fingers between them
        assert!(volume_lattice(&tool, 0.05, 1.0).iter().all(|p| p[1] >= 0.025 - 1e-12));
        let poly = vec![[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]];
        let pr = Prism::new(&poly, 0.0, 0.03, 0.002);
        assert!(pr.contains(&[0.011, 0.0, 0.01]));
        assert!(!pr.contains(&[0.013, 0.0, 0.01]));
        assert!(!pr.contains(&[0.0, 0.0, 0.031]));
    }
}
