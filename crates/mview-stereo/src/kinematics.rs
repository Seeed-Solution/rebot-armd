//! The vector helpers of armd's `kinematics` the stereo geometry uses (same arithmetic).
pub type Mat3 = [[f64; 3]; 3];
pub type Vec3 = [f64; 3];

#[inline]
pub fn sub(a: &Vec3, b: &Vec3) -> Vec3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
#[inline]
pub fn cross(a: &Vec3, b: &Vec3) -> Vec3 { [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]] }
#[inline]
pub fn norm(a: &Vec3) -> f64 { (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt() }
/// Unit vector; a zero vector is returned unchanged.
pub fn unit(a: &Vec3) -> Vec3 {
    let n = norm(a);
    let n = if n == 0.0 || n.is_nan() { 1.0 } else { n };
    [a[0] / n, a[1] / n, a[2] / n]
}
pub fn mat_vec(m: &Mat3, v: &Vec3) -> Vec3 {
    [m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2], m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2], m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]]
}
pub fn transpose(m: &Mat3) -> Mat3 { [[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]] }
pub fn from_columns(x: &Vec3, y: &Vec3, z: &Vec3) -> Mat3 { [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]] }
