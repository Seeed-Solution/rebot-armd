use std::path::Path;
use thiserror::Error;

/// Expected SHA256 checksum of assets/reBot-DevArm_gripper.urdf
pub const EXPECTED_URDF_SHA256: &str =
    "9075632d119f5a8c2d39c55e63d70680e3c8692af69baa222c4da8184159b5ad";

#[derive(Debug, Clone, PartialEq, Error)]
pub enum GravityError {
    #[error("Non-finite joint angle at index {0}: {1}")]
    NonFiniteAngle(usize, f32),
    #[error("URDF file not found: {0}")]
    FileNotFound(String),
    #[error("URDF SHA256 checksum mismatch: expected {expected}, got {actual}")]
    ChecksumMismatch { expected: String, actual: String },
}

/// Verify the SHA256 checksum of the specified URDF file
pub fn verify_urdf_sha256(path: impl AsRef<Path>) -> Result<(), GravityError> {
    let path = path.as_ref();
    let content = std::fs::read(path).map_err(|e| {
        GravityError::FileNotFound(format!("{}: {}", path.display(), e))
    })?;
    let hash = compute_sha256(&content);
    if hash != EXPECTED_URDF_SHA256 {
        return Err(GravityError::ChecksumMismatch {
            expected: EXPECTED_URDF_SHA256.to_string(),
            actual: hash,
        });
    }
    Ok(())
}

/// Compute SHA-256 in pure Rust (FIPS 180-4 standard) with zero dependencies
pub fn compute_sha256(data: &[u8]) -> String {
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
    ];

    let k: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
    ];

    let bit_len = (data.len() as u64) * 8;
    let mut padded = data.to_vec();
    padded.push(0x80);
    while (padded.len() % 64) != 56 {
        padded.push(0x00);
    }
    padded.extend_from_slice(&bit_len.to_be_bytes());

    for chunk in padded.chunks_exact(64) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([
                chunk[i * 4],
                chunk[i * 4 + 1],
                chunk[i * 4 + 2],
                chunk[i * 4 + 3],
            ]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16].wrapping_add(s0).wrapping_add(w[i - 7]).wrapping_add(s1);
        }

        let mut a = h[0];
        let mut b = h[1];
        let mut c = h[2];
        let mut d = h[3];
        let mut e = h[4];
        let mut f = h[5];
        let mut g = h[6];
        let mut h_var = h[7];

        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ ((!e) & g);
            let temp1 = h_var
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(k[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let temp2 = s0.wrapping_add(maj);

            h_var = g;
            g = f;
            f = e;
            e = d.wrapping_add(temp1);
            d = c;
            c = b;
            b = a;
            a = temp1.wrapping_add(temp2);
        }

        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
        h[5] = h[5].wrapping_add(f);
        h[6] = h[6].wrapping_add(g);
        h[7] = h[7].wrapping_add(h_var);
    }

    format!(
        "{:08x}{:08x}{:08x}{:08x}{:08x}{:08x}{:08x}{:08x}",
        h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]
    )
}

// =========================================================================
// Kinematics and Dynamics model constants from assets/reBot-DevArm_gripper.urdf
// =========================================================================

pub const NUM_JOINTS: usize = 9;
pub const PARENTS: [usize; 9] = [0, 0, 1, 2, 3, 4, 5, 6, 6];

pub const TRANS: [[f64; 3]; 9] = [
    [0.0, 0.0, 0.0],                                           // universe
    [-8.415999999999999382e-05, 0.0, 8.465000000000000302e-02], // joint1
    [2.008400000000000116e-02, 3.162500000000000033e-02, 5.555000000000000215e-02], // joint2
    [-2.640000000000000124e-01, 0.0, 0.0],                     // joint3
    [2.426000000000000101e-01, -5.399999999999999939e-02, -1.624999999999999925e-03], // joint4
    [7.830800000000000261e-02, -3.749999999999999861e-02, -2.999999999999999889e-02], // joint5
    [2.800800000000000164e-02, 0.0, 4.000000000000000083e-02], // joint6
    [0.0, 0.0, 1.553900000000000003e-01],                     // left_finger_joint
    [0.0, 0.0, 1.553900000000000003e-01],                     // right_finger_joint
];

pub const ROT: [[[f64; 3]; 3]; 9] = [
    [ // universe
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    [ // joint1
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    [ // joint2
        [1.0, 0.0, 0.0],
        [0.0, -3.673205103638110813e-06, 9.999999999932539518e-01],
        [0.0, -9.999999999932539518e-01, -3.673205103638110813e-06],
    ],
    [ // joint3
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    [ // joint4
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    [ // joint5
        [1.0, 0.0, 0.0],
        [0.0, -3.673205103638110813e-06, 9.999999999932539518e-01],
        [0.0, -9.999999999932539518e-01, -3.673205103638110813e-06],
    ],
    [ // joint6
        [-3.673205103638110813e-06, 0.0, 9.999999999932539518e-01],
        [0.0, 1.0, 0.0],
        [-9.999999999932539518e-01, 0.0, -3.673205103638110813e-06],
    ],
    [ // left_finger_joint
        [3.673205087539876956e-06, -9.265358966049024427e-05, 9.999999957009100271e-01],
        [-3.403356383821356391e-10, -9.999999957076561863e-01, -9.265358965986518816e-05],
        [9.999999999932538408e-01, 1.725935459944511522e-21, -3.673205103416066208e-06],
    ],
    [ // right_finger_joint
        [3.673205087539876956e-06, -9.265358966049024427e-05, 9.999999957009100271e-01],
        [-3.403356383821356391e-10, -9.999999957076561863e-01, -9.265358965986518816e-05],
        [9.999999999932538408e-01, 1.725935459944511522e-21, -3.673205103416066208e-06],
    ],
];

pub const MASS: [f64; 9] = [
    8.366000000000000103e-01, // universe
    1.612999999999999989e-01, // joint1
    1.326600000000000001e+00, // joint2
    8.353000000000000425e-01, // joint3
    5.200000000000000178e-01, // joint4
    3.830000000000000071e-01, // joint5
    9.163000000000001144e-01, // joint6
    2.999999999999999889e-02, // left_finger_joint
    2.999999999999999889e-02, // right_finger_joint
];

pub const COM: [[f64; 3]; 9] = [
    [-7.849000000000000575e-06, -1.153100000000000022e-06, 2.984099999999999947e-02], // universe
    [1.136145529516270050e-04, -6.163195270513229873e-04, 2.364763726713940001e-02], // joint1
    [-1.322562230888799928e-01, -3.061703638630900153e-03, -3.083069670302050064e-02], // joint2
    [1.210400357918429981e-01, -5.362110766279489654e-02, -3.101378546080769960e-02], // joint3
    [6.082009562931359886e-02, -5.117119066131219779e-02, -3.029945862392700168e-02], // joint4
    [-5.028020589825169841e-03, 1.738662066923640099e-06, 3.862332363267550334e-02], // joint5
    [5.570319630005777725e-06, -3.463253422493070083e-05, 5.912060238961115710e-02], // joint6
    [-1.277999999999999969e-01, 1.569999999999999868e-02, 0.0], // left_finger_joint
    [-1.277999999999999969e-01, -1.569999999999999868e-02, 0.0], // right_finger_joint
];

pub const AXES: [[f64; 3]; 9] = [
    [0.0, 0.0, 0.0],  // universe
    [0.0, 0.0, 1.0],  // joint1
    [0.0, 0.0, -1.0], // joint2 (shoulder pitch reversed axis)
    [0.0, 0.0, 1.0],  // joint3
    [0.0, 0.0, 1.0],  // joint4
    [0.0, 0.0, 1.0],  // joint5
    [0.0, 0.0, 1.0],  // joint6
    [0.0, 1.0, 0.0],  // left_finger_joint
    [0.0, -1.0, 0.0], // right_finger_joint
];

// 3D vector and matrix helper routines
#[inline]
pub(crate) fn rot_axis(axis: &[f64; 3], angle: f64) -> [[f64; 3]; 3] {
    let c = angle.cos();
    let s = angle.sin();
    let len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    let (ux, uy, uz) = if len > 1e-12 {
        (axis[0] / len, axis[1] / len, axis[2] / len)
    } else {
        (0.0, 0.0, 1.0)
    };
    [
        [c + ux * ux * (1.0 - c), ux * uy * (1.0 - c) - uz * s, ux * uz * (1.0 - c) + uy * s],
        [uy * ux * (1.0 - c) + uz * s, c + uy * uy * (1.0 - c), uy * uz * (1.0 - c) - ux * s],
        [uz * ux * (1.0 - c) - uy * s, uz * uy * (1.0 - c) + ux * s, c + uz * uz * (1.0 - c)],
    ]
}

#[inline]
pub(crate) fn mat3_mul(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut res = [[0.0; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            res[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
        }
    }
    res
}

#[inline]
pub(crate) fn mat3_vec3_mul(m: &[[f64; 3]; 3], v: &[f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

#[inline]
fn vec3_add(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
fn vec3_sub(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
fn vec3_dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Compute generalized gravity torques g(q) for joints 1..6.
///
/// World coordinate system: Z-axis points up, gravity vector is [0.0, 0.0, -9.81] m/s^2.
/// Returns the holding torques in N·m required to counteract gravity at pose q.
pub fn compute_gravity_torques(q: &[f32; 6]) -> Result<[f32; 6], GravityError> {
    for (i, &val) in q.iter().enumerate() {
        if !val.is_finite() {
            return Err(GravityError::NonFiniteAngle(i, val));
        }
    }

    let q64: [f64; 6] = [
        q[0] as f64,
        q[1] as f64,
        q[2] as f64,
        q[3] as f64,
        q[4] as f64,
        q[5] as f64,
    ];

    // Forward kinematics
    let mut o_r = [[[0.0; 3]; 3]; NUM_JOINTS];
    let mut o_p = [[0.0; 3]; NUM_JOINTS];
    o_r[0] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];

    for i in 1..NUM_JOINTS {
        let parent = PARENTS[i];
        let p_r = o_r[parent];
        let p_p = o_p[parent];

        let t_r = &ROT[i];
        let t_p = &TRANS[i];

        let (r_j, p_j) = if i <= 6 {
            (rot_axis(&AXES[i], q64[i - 1]), [0.0, 0.0, 0.0])
        } else {
            // Gripper prismatic fingers at q=0
            (
                [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                [0.0, 0.0, 0.0],
            )
        };

        let rel_r = mat3_mul(t_r, &r_j);
        let rel_p = vec3_add(t_p, &mat3_vec3_mul(t_r, &p_j));

        o_r[i] = mat3_mul(&p_r, &rel_r);
        o_p[i] = vec3_add(&p_p, &mat3_vec3_mul(&p_r, &rel_p));
    }

    // Compute gravity torques for joints 1..6
    let g_acc = 9.81;
    let mut tau = [0.0f32; 6];

    for k in 1..=6 {
        let axis_world = mat3_vec3_mul(&o_r[k], &AXES[k]);
        let origin_k = &o_p[k];

        let mut joint_tau_f64 = 0.0;
        for i in k..NUM_JOINTS {
            // Check if k is an ancestor of i
            let mut curr = i;
            let mut is_descendant = false;
            while curr > 0 {
                if curr == k {
                    is_descendant = true;
                    break;
                }
                curr = PARENTS[curr];
            }
            if !is_descendant {
                continue;
            }

            let com_world = vec3_add(&o_p[i], &mat3_vec3_mul(&o_r[i], &COM[i]));
            let r = vec3_sub(&com_world, origin_k);
            let t_balance = [
                r[1] * MASS[i] * g_acc,
                -r[0] * MASS[i] * g_acc,
                0.0,
            ];
            joint_tau_f64 += vec3_dot(&axis_world, &t_balance);
        }

        tau[k - 1] = joint_tau_f64 as f32;
    }

    Ok(tau)
}
