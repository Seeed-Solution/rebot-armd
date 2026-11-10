use std::path::Path;
use thiserror::Error;

/// Expected SHA256 checksum of assets/reBot-DevArm_gripper.urdf
pub const EXPECTED_URDF_SHA256: &str =
    "bc1b3d490deb795b3e259ac8ec36a6be6342cffc3d4e145c2529eac76abdcb62";

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

    for chunk in padded.as_chunks::<64>().0 {
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
    [-8.416e-5, 0.0, 8.465e-2], // joint1
    [2.008_4e-2, 3.162_5e-2, 5.555e-2], // joint2
    [-2.64e-1, 0.0, 0.0],                     // joint3
    [2.426e-1, -5.4e-2, -1.625e-3], // joint4
    [7.830_8e-2, -3.75e-2, -3e-2], // joint5
    [2.800_8e-2, 0.0, 4e-2], // joint6
    [0.0, 0.0, 1.553_9e-1],                     // left_finger_joint
    [0.0, 0.0, 1.553_9e-1],                     // right_finger_joint
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
        [0.0, -3.673_205_103_638_111e-6, 9.999_999_999_932_54e-1],
        [0.0, -9.999_999_999_932_54e-1, -3.673_205_103_638_111e-6],
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
        [0.0, -3.673_205_103_638_111e-6, 9.999_999_999_932_54e-1],
        [0.0, -9.999_999_999_932_54e-1, -3.673_205_103_638_111e-6],
    ],
    [ // joint6
        [-3.673_205_103_638_111e-6, 0.0, 9.999_999_999_932_54e-1],
        [0.0, 1.0, 0.0],
        [-9.999_999_999_932_54e-1, 0.0, -3.673_205_103_638_111e-6],
    ],
    [ // left_finger_joint
        [3.673_205_087_539_877e-6, -9.265_358_966_049_024e-5, 9.999_999_957_009_1e-1],
        [-3.403_356_383_821_356_4e-10, -9.999_999_957_076_562e-1, -9.265_358_965_986_519e-5],
        [9.999_999_999_932_538e-1, 1.725_935_459_944_511_5e-21, -3.673_205_103_416_066e-6],
    ],
    [ // right_finger_joint
        [3.673_205_087_539_877e-6, -9.265_358_966_049_024e-5, 9.999_999_957_009_1e-1],
        [-3.403_356_383_821_356_4e-10, -9.999_999_957_076_562e-1, -9.265_358_965_986_519e-5],
        [9.999_999_999_932_538e-1, 1.725_935_459_944_511_5e-21, -3.673_205_103_416_066e-6],
    ],
];

pub const MASS: [f64; 9] = [
    8.366e-1, // universe
    1.613e-1, // joint1
    1.326_6, // joint2
    8.353e-1, // joint3
    5.2e-1, // joint4
    3.83e-1, // joint5
    9.163_000_000_000_001e-1, // joint6
    3e-2, // left_finger_joint
    3e-2, // right_finger_joint
];

pub const COM: [[f64; 3]; 9] = [
    [-7.849e-6, -1.153_1e-6, 2.984_1e-2], // universe
    [1.136_145_529_516_27e-4, -6.163_195_270_513_23e-4, 2.364_763_726_713_94e-2], // joint1
    [-1.322_562_230_888_8e-1, -3.061_703_638_630_9e-3, -3.083_069_670_302_05e-2], // joint2
    [1.210_400_357_918_43e-1, -5.362_110_766_279_49e-2, -3.101_378_546_080_77e-2], // joint3
    [6.082_009_562_931_36e-2, -5.117_119_066_131_22e-2, -3.029_945_862_392_7e-2], // joint4
    [-5.028_020_589_825_17e-3, 1.738_662_066_923_64e-6, 3.862_332_363_267_55e-2], // joint5
    [5.570_319_630_005_778e-6, -3.463_253_422_493_07e-5, 5.912_060_238_961_116e-2], // joint6
    [-1.278e-1, 1.57e-2, 0.0], // left_finger_joint
    [-1.278e-1, -1.57e-2, 0.0], // right_finger_joint
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
    Ok(gravity_torques_f64(&q64).map(|t| t as f32))
}

/// [`compute_gravity_torques`] in f64, without the finiteness check.
pub fn gravity_torques_f64(q64: &[f64; 6]) -> [f64; 6] {

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
    let mut tau = [0.0f64; 6];

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

        tau[k - 1] = joint_tau_f64;
    }

    tau
}
