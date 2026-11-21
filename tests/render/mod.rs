//! Pinhole ray-casting renderer of a textured table plane with boxes on it, for the
//! motion-stereo tests (tests/test_mview_stereo.rs, tests/test_measure_multiview.rs).
#![allow(dead_code)]
use armd::kinematics::{self, Vec3};
use armd::mview_stereo::{Camera, Gray};

/// A box standing on the table: centre (x, y), yaw about z (rad), size [length (along yaw), width, height] (m).
#[derive(Debug, Clone, Copy)]
pub struct BoxObj {
    pub xy: [f64; 2],
    pub yaw: f64,
    pub size: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct Scene {
    pub table_z: f64,
    pub boxes: Vec<BoxObj>,
    /// Texture cell (m) of the coarse octave.
    pub cell_m: f64,
    /// A periodic dot pattern (4 mm period in x and y) on the table instead of noise: every
    /// patch repeats along the epipolar lines.
    pub periodic: bool,
}

impl Scene {
    pub fn new(table_z: f64, boxes: Vec<BoxObj>) -> Self { Self { table_z, boxes, cell_m: 0.003, periodic: false } }
}

fn hash(ix: i64, iy: i64, seed: u64) -> f64 {
    let mut h = (ix as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ (iy as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F) ^ seed.wrapping_mul(0x1656_67B1_9E37_79F9);
    h ^= h >> 31;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 29;
    (h >> 11) as f64 / (1u64 << 53) as f64
}

/// Smooth value noise in [0, 1].
fn value_noise(u: f64, v: f64, cell: f64, seed: u64) -> f64 {
    let (x, y) = (u / cell, v / cell);
    let (x0, y0) = (x.floor(), y.floor());
    let (fx, fy) = (x - x0, y - y0);
    let s = |t: f64| t * t * (3.0 - 2.0 * t);
    let (sx, sy) = (s(fx), s(fy));
    let (ix, iy) = (x0 as i64, y0 as i64);
    let a = hash(ix, iy, seed) * (1.0 - sx) + hash(ix + 1, iy, seed) * sx;
    let b = hash(ix, iy + 1, seed) * (1.0 - sx) + hash(ix + 1, iy + 1, seed) * sx;
    a * (1.0 - sy) + b * sy
}

fn texture(u: f64, v: f64, cell: f64, seed: u64) -> f64 {
    let n = (value_noise(u, v, cell, seed) + 0.5 * value_noise(u, v, 0.4 * cell, seed ^ 0x55)) / 1.5;
    30.0 + 195.0 * n
}

/// First hit of the ray `o + t d` (t > 0): (t, grey level).
fn trace(scene: &Scene, o: &Vec3, d: &Vec3) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    if d[2] < -1e-9 {
        let t = (scene.table_z - o[2]) / d[2];
        if t > 0.0 {
            let (x, y) = (o[0] + t * d[0], o[1] + t * d[1]);
            let k = 2.0 * std::f64::consts::PI / 0.004;
            let g = if scene.periodic { 128.0 + 100.0 * (k * x).sin() * (k * y).sin() } else { texture(x, y, scene.cell_m, 1) };
            best = Some((t, g));
        }
    }
    for (k, b) in scene.boxes.iter().enumerate() {
        let (c, s) = (b.yaw.cos(), b.yaw.sin());
        let centre = [b.xy[0], b.xy[1], scene.table_z + b.size[2] / 2.0];
        let rel = kinematics::sub(o, &centre);
        // box frame: x along the length
        let lo = [c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1], rel[2]];
        let ld = [c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]];
        let half = [b.size[0] / 2.0, b.size[1] / 2.0, b.size[2] / 2.0];
        let (mut t0, mut t1, mut axis) = (f64::NEG_INFINITY, f64::INFINITY, 0usize);
        let mut ok = true;
        for i in 0..3 {
            if ld[i].abs() < 1e-12 {
                if lo[i].abs() > half[i] { ok = false; break; }
                continue;
            }
            let (mut a, mut bb) = ((-half[i] - lo[i]) / ld[i], (half[i] - lo[i]) / ld[i]);
            if a > bb { std::mem::swap(&mut a, &mut bb); }
            if a > t0 { t0 = a; axis = i; }
            t1 = t1.min(bb);
        }
        if !ok || t0 > t1 || t0 <= 0.0 { continue; }
        if best.is_some_and(|(t, _)| t <= t0) { continue; }
        let p = [lo[0] + t0 * ld[0], lo[1] + t0 * ld[1], lo[2] + t0 * ld[2]];
        let face_seed = 100 + 10 * k as u64 + axis as u64 * 2 + u64::from(ld[axis] > 0.0);
        let (u, v) = match axis { 0 => (p[1], p[2]), 1 => (p[0], p[2]), _ => (p[0], p[1]) };
        // faces slightly brighter / darker than each other
        let shade = [0.92, 0.85, 1.0][axis];
        best = Some((t0, 20.0 + shade * (texture(u, v, scene.cell_m, face_seed) - 20.0)));
    }
    best
}

/// Image noise of one rendered view.
#[derive(Debug, Clone, Copy, Default)]
pub struct Noise {
    /// Principal point shift (px): the camera model the measurement uses is off by this much
    /// (a systematic pose / calibration error of the view).
    pub shift_px: [f64; 2],
    /// Gaussian intensity noise (grey levels).
    pub sigma: f64,
}

/// Render `w` x `h` with `ss` x `ss` supersampling as seen by `cam`, with `noise` from `seed`.
pub fn render(scene: &Scene, cam: &Camera, w: usize, h: usize, ss: usize, noise: Noise, seed: u64) -> Gray {
    let true_cam = Camera { cx: cam.cx + noise.shift_px[0], cy: cam.cy + noise.shift_px[1], ..*cam };
    let sigma = noise.sigma;
    let mut rng = Rng(seed.wrapping_mul(0x2545_F491_4F6C_DD1D) | 1);
    let mut px = Vec::with_capacity(w * h);
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0;
            for j in 0..ss {
                for i in 0..ss {
                    let u = x as f64 - 0.5 + (i as f64 + 0.5) / ss as f64;
                    let v = y as f64 - 0.5 + (j as f64 + 0.5) / ss as f64;
                    let d = true_cam.ray([u, v]);
                    acc += trace(scene, &true_cam.c, &d).map_or(10.0, |(_, g)| g);
                }
            }
            let g = acc / (ss * ss) as f64 + sigma * rng.gauss();
            px.push(g.round().clamp(0.0, 255.0) as u8);
        }
    }
    Gray::new(w, h, px).unwrap()
}

pub struct Rng(pub u64);

impl Rng {
    pub fn next_f64(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn gauss(&mut self) -> f64 {
        let (u1, u2) = (self.next_f64().max(1e-12), self.next_f64());
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

/// Camera at `c` looking at `look` (image x horizontal).
pub fn look_at(c: Vec3, look: Vec3, fx: f64, size: [f64; 2]) -> Camera {
    let z = kinematics::unit(&kinematics::sub(&look, &c));
    let x = kinematics::unit(&kinematics::cross(&z, &[0.0, 0.0, 1.0]));
    let y = kinematics::cross(&z, &x);
    Camera { r: kinematics::from_columns(&x, &y, &z), c, fx, fy: fx, cx: size[0] / 2.0, cy: size[1] / 2.0 }
}

/// `cam` turned about the point `pivot` so that its centre moves by the chord `offset_m` along its
/// own image x axis (the pivot stays at the same pixel).
pub fn orbit(cam: &Camera, pivot: Vec3, offset_m: f64) -> Camera {
    let rel = kinematics::sub(&cam.c, &pivot);
    let d = kinematics::norm(&rel);
    let ray = kinematics::unit(&kinematics::sub(&pivot, &cam.c));
    let xc = kinematics::column(&cam.r, 0);
    let lat = kinematics::unit(&kinematics::sub(&xc, &kinematics::mat_vec(&[[ray[0] * ray[0], ray[0] * ray[1], ray[0] * ray[2]], [ray[1] * ray[0], ray[1] * ray[1], ray[1] * ray[2]], [ray[2] * ray[0], ray[2] * ray[1], ray[2] * ray[2]]], &xc)));
    let axis = kinematics::unit(&kinematics::cross(&lat, &ray));
    let theta = 2.0 * (offset_m / (2.0 * d)).asin();
    let r = kinematics::axis_r(&axis, theta);
    Camera { r: kinematics::mat_mul(&r, &cam.r), c: kinematics::add(&pivot, &kinematics::mat_vec(&r, &rel)), ..*cam }
}

/// Bounding box (px) of a box's 8 corners in `cam`.
pub fn box_px(scene: &Scene, b: &BoxObj, cam: &Camera) -> [f64; 4] {
    let (c, s) = (b.yaw.cos(), b.yaw.sin());
    let mut r = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for i in 0..8 {
        let u = if i & 1 == 0 { -0.5 } else { 0.5 } * b.size[0];
        let v = if i & 2 == 0 { -0.5 } else { 0.5 } * b.size[1];
        let z = if i & 4 == 0 { 0.0 } else { b.size[2] };
        let p = [b.xy[0] + u * c - v * s, b.xy[1] + u * s + v * c, scene.table_z + z];
        if let Some(q) = cam.project(&p) { r = [r[0].min(q[0]), r[1].min(q[1]), r[2].max(q[0]), r[3].max(q[1])]; }
    }
    r
}
